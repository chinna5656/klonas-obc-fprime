/**
 * ============================================================================
 * KLONAS Phase-1 CubeSat Flight Software
 * DataLogger Component Implementation
 * ============================================================================
 */

#include "obc/Components/DataLogger/DataLogger.hpp"
#include <Fw/Types/Assert.hpp>
#include <cstdio>
#include <cstring>

namespace Obc {

  DataLogger::DataLogger(const char* const compName) :
    DataLoggerComponentBase(compName),
    m_activeBuffer(m_pingBuffer),
    m_activeBufPos(0),
    m_usingPing(true),
    m_fprimeTx(m_spiTx, sizeof(m_spiTx)),
    m_fprimeRx(m_spiRx, sizeof(m_spiRx)),
    m_currentSector(100), // Start logging at sector 100
    m_loggingEnabled(true),
    m_cardReady(true),
    m_recordsLogged(0),
    m_bytesWritten(0),
    m_sectorsWritten(0),
    m_bufferUtilizationPct(0.0f),
    m_sdWriteErrors(0)
  {
    memset(m_pingBuffer, 0, sizeof(m_pingBuffer));
    memset(m_pongBuffer, 0, sizeof(m_pongBuffer));
    memset(m_spiTx, 0xFF, sizeof(m_spiTx));
    memset(m_spiRx, 0, sizeof(m_spiRx));
  }

  void DataLogger::init(
      FwSizeType queueDepth,
      FwEnumStoreType instance
  ) {
    DataLoggerComponentBase::init(queueDepth, instance);
    this->log_ACTIVITY_HI_SdCardInitialized();
  }

  void DataLogger::logRecordIn_handler(
      FwIndexType portNum,
      const Obc::FlightLogRecord& flightData
  ) {
    FW_ASSERT(portNum == 0);

    if (!m_loggingEnabled) {
      return;
    }

    char csvLine[128];
    FwSizeType lineLen = this->formatCsvLine(flightData, csvLine, sizeof(csvLine));
    if (lineLen == 0) {
      return;
    }

    // Check if CSV line fits in active 512-byte staging sector
    if ((m_activeBufPos + lineLen) > SD_SECTOR_SIZE) {
      // Current sector full: commit to MicroSD over SPI2
      this->writeSdSector(m_activeBuffer, m_currentSector++);

      // Ping-pong switch
      m_usingPing = !m_usingPing;
      m_activeBuffer = m_usingPing ? m_pingBuffer : m_pongBuffer;
      m_activeBufPos = 0;
      memset(m_activeBuffer, 0, SD_SECTOR_SIZE);
    }

    // Append record to active sector buffer
    memcpy(&m_activeBuffer[m_activeBufPos], csvLine, lineLen);
    m_activeBufPos += lineLen;
    m_recordsLogged++;

    m_bufferUtilizationPct = (static_cast<F32>(m_activeBufPos) / static_cast<F32>(SD_SECTOR_SIZE)) * 100.0f;
  }

  void DataLogger::schedIn_handler(
      FwIndexType portNum,
      U32 context
  ) {
    FW_ASSERT(portNum == 0);
    (void)context;

    // Periodic flush timer (e.g. flush partial sector every 50 ticks ~ 5 seconds)
    static U32 flushTick = 0;
    if (++flushTick >= 50) {
      flushTick = 0;
      if (m_activeBufPos > 0) {
        this->flushActiveBuffer();
      }
    }

    // Update telemetry channels
    this->tlmWrite_RecordsLogged(m_recordsLogged);
    this->tlmWrite_BytesWritten(m_bytesWritten);
    this->tlmWrite_SectorsWritten(m_sectorsWritten);
    this->tlmWrite_BufferUtilizationPct(m_bufferUtilizationPct);
    this->tlmWrite_SdWriteErrors(m_sdWriteErrors);
    this->tlmWrite_CardReady(m_cardReady);
  }

  void DataLogger::LOG_FORCE_FLUSH_cmdHandler(
      FwOpcodeType opCode,
      U32 cmdSeq
  ) {
    this->flushActiveBuffer();
    this->tlmWrite_SectorsWritten(m_sectorsWritten);
    this->tlmWrite_BytesWritten(m_bytesWritten);
    this->tlmWrite_BufferUtilizationPct(m_bufferUtilizationPct);
    this->cmdResponse_out(opCode, cmdSeq, Fw::CmdResponse::OK);
  }

  void DataLogger::LOG_ENABLE_cmdHandler(
      FwOpcodeType opCode,
      U32 cmdSeq,
      Fw::Enabled enable
  ) {
    m_loggingEnabled = (enable == Fw::Enabled::ENABLED);
    this->cmdResponse_out(opCode, cmdSeq, Fw::CmdResponse::OK);
  }

  void DataLogger::LOG_CLEAR_COUNTERS_cmdHandler(
      FwOpcodeType opCode,
      U32 cmdSeq
  ) {
    m_recordsLogged = 0;
    m_bytesWritten = 0;
    m_sectorsWritten = 0;
    m_sdWriteErrors = 0;
    this->tlmWrite_RecordsLogged(0);
    this->tlmWrite_BytesWritten(0);
    this->tlmWrite_SectorsWritten(0);
    this->tlmWrite_SdWriteErrors(0);
    this->cmdResponse_out(opCode, cmdSeq, Fw::CmdResponse::OK);
  }

  FwSizeType DataLogger::formatCsvLine(
      const Obc::FlightLogRecord& r,
      char* dest,
      FwSizeType maxLen
  ) {
    // CSV Format: timestamp,lat,lon,alt,vz,roll,pitch,yaw,t_int,p_int,t_ext,p_ext,hum,vbat,soc,state
    int written = snprintf(
        dest,
        maxLen,
        "%lu,%.6f,%.6f,%.1f,%.2f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.2f,%.1f,%u\n",
        static_cast<unsigned long>(r.get_timestampMs()),
        r.get_lat(),
        r.get_lon(),
        r.get_alt(),
        r.get_descentRate(),
        r.get_roll(),
        r.get_pitch(),
        r.get_yaw(),
        r.get_tempInt(),
        r.get_pressInt(),
        r.get_tempExt(),
        r.get_pressExt(),
        r.get_humidity(),
        r.get_vBat(),
        r.get_soc(),
        r.get_state()
    );

    if (written < 0 || static_cast<FwSizeType>(written) >= maxLen) {
      return 0;
    }

    return static_cast<FwSizeType>(written);
  }

  bool DataLogger::writeSdSector(const uint8_t* sectorData, U32 sectorIndex) {
    // Standard SPI SD Block Write:
    // 1. Assert CS LOW (PB12)
    if (this->isConnected_csOut_OutputPort(0)) {
      this->csOut_out(0, Fw::Logic::LOW);
    }

    // 2. Format SD CMD24 (WRITE_BLOCK) packet
    m_spiTx[0] = 0x40 | 24; // CMD24
    m_spiTx[1] = static_cast<uint8_t>((sectorIndex >> 24) & 0xFF);
    m_spiTx[2] = static_cast<uint8_t>((sectorIndex >> 16) & 0xFF);
    m_spiTx[3] = static_cast<uint8_t>((sectorIndex >> 8) & 0xFF);
    m_spiTx[4] = static_cast<uint8_t>(sectorIndex & 0xFF);
    m_spiTx[5] = 0xFF;      // Dummy CRC
    m_spiTx[6] = 0xFE;      // Start Block Data Token

    memcpy(&m_spiTx[7], sectorData, SD_SECTOR_SIZE);
    m_spiTx[7 + SD_SECTOR_SIZE] = 0xFF; // CRC16 byte 1
    m_spiTx[8 + SD_SECTOR_SIZE] = 0xFF; // CRC16 byte 2

    FwSizeType totalTransfer = 9 + SD_SECTOR_SIZE;
    m_fprimeTx.setSize(totalTransfer);
    m_fprimeRx.setSize(totalTransfer);

    Drv::SpiStatus status = Drv::SpiStatus::SPI_OTHER_ERR;
    if (this->isConnected_spiOut_OutputPort(0)) {
      status = this->spiOut_out(0, m_fprimeTx, m_fprimeRx);
    }

    // 3. Deassert CS HIGH (PB12)
    if (this->isConnected_csOut_OutputPort(0)) {
      this->csOut_out(0, Fw::Logic::HIGH);
    }

    if (status != Drv::SpiStatus::SPI_OK) {
      m_sdWriteErrors++;
      this->log_WARNING_HI_SdWriteError(sectorIndex, static_cast<U8>(status));
      return false;
    }

    m_sectorsWritten++;
    m_bytesWritten += SD_SECTOR_SIZE;
    return true;
  }

  void DataLogger::flushActiveBuffer() {
    if (m_activeBufPos == 0) {
      return;
    }

    // Pad remainder of 512-byte sector with spaces or nulls
    memset(&m_activeBuffer[m_activeBufPos], 0, SD_SECTOR_SIZE - m_activeBufPos);
    this->writeSdSector(m_activeBuffer, m_currentSector++);

    m_usingPing = !m_usingPing;
    m_activeBuffer = m_usingPing ? m_pingBuffer : m_pongBuffer;
    m_activeBufPos = 0;
    m_bufferUtilizationPct = 0.0f;
    memset(m_activeBuffer, 0, SD_SECTOR_SIZE);

    this->log_ACTIVITY_HI_FlightLogFlushed(m_sectorsWritten, m_recordsLogged);
  }

}

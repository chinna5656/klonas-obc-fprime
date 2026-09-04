/**
 * ============================================================================
 * KLONAS Phase-1 CubeSat Flight Software
 * DataLogger Component Header
 * ============================================================================
 */

#ifndef OBC_DATALOGGER_HPP_
#define OBC_DATALOGGER_HPP_

#include "obc/Components/DataLogger/DataLoggerComponentAc.hpp"

namespace Obc {

  class DataLogger : public DataLoggerComponentBase {

    public:

      //! Standard SD card physical block/sector size in bytes
      static constexpr FwSizeType SD_SECTOR_SIZE = 512;

      //! Construct DataLogger instance
      DataLogger(const char* const compName);

      //! Destructor
      ~DataLogger() override = default;

      //! Initialize component
      void init(
          FwSizeType queueDepth,
          FwEnumStoreType instance = 0
      );

    private:

      // ----------------------------------------------------------------------
      // Handlers for input ports
      // ----------------------------------------------------------------------

      void logRecordIn_handler(
          FwIndexType portNum,
          const Obc::FlightLogRecord& flightData
      ) override;

      void schedIn_handler(
          FwIndexType portNum,
          U32 context
      ) override;

      // ----------------------------------------------------------------------
      // Command handlers
      // ----------------------------------------------------------------------

      void LOG_FORCE_FLUSH_cmdHandler(
          FwOpcodeType opCode,
          U32 cmdSeq
      ) override;

      void LOG_ENABLE_cmdHandler(
          FwOpcodeType opCode,
          U32 cmdSeq,
          Fw::Enabled enable
      ) override;

      void LOG_CLEAR_COUNTERS_cmdHandler(
          FwOpcodeType opCode,
          U32 cmdSeq
      ) override;

      // ----------------------------------------------------------------------
      // MicroSD Flash Storage Routines
      // ----------------------------------------------------------------------

      //! Format telemetry record into CSV string (zero-alloc snprintf)
      FwSizeType formatCsvLine(const Obc::FlightLogRecord& record, char* dest, FwSizeType maxLen);

      //! Commit 512-byte sector to SD Card over SPI2
      bool writeSdSector(const uint8_t* sectorData, U32 sectorIndex);

      //! Flush active staging buffer to SD card
      void flushActiveBuffer();

      // ----------------------------------------------------------------------
      // Member variables (zero dynamic allocation)
      // ----------------------------------------------------------------------

      // Double-buffered 512-byte ping-pong buffers (total 1024 B RAM)
      uint8_t m_pingBuffer[SD_SECTOR_SIZE];
      uint8_t m_pongBuffer[SD_SECTOR_SIZE];
      uint8_t* m_activeBuffer;
      FwSizeType m_activeBufPos;
      bool m_usingPing;

      // SPI transmission wrapper buffers
      uint8_t m_spiTx[SD_SECTOR_SIZE + 16];
      uint8_t m_spiRx[SD_SECTOR_SIZE + 16];
      Fw::Buffer m_fprimeTx;
      Fw::Buffer m_fprimeRx;

      U32 m_currentSector;
      bool m_loggingEnabled;
      bool m_cardReady;

      // Telemetry metrics
      U32 m_recordsLogged;
      U32 m_bytesWritten;
      U32 m_sectorsWritten;
      F32 m_bufferUtilizationPct;
      U32 m_sdWriteErrors;

  };

}

#endif /* OBC_DATALOGGER_HPP_ */

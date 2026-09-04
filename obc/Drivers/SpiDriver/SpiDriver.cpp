/**
 * ============================================================================
 * KLONAS Phase-1 CubeSat Flight Software
 * STM32F411 SPI Driver Component Implementation
 * ============================================================================
 */

#include "obc/Drivers/SpiDriver/SpiDriver.hpp"
#include <Fw/Types/Assert.hpp>

namespace Obc {

  SpiDriver::SpiDriver(const char* const compName) :
    SpiDriverComponentBase(compName),
    m_hspi(nullptr),
    m_busId(0),
    m_configured(false),
    m_bytesTransferred(0),
    m_spiErrors(0)
  {
  }

  void SpiDriver::init(FwEnumStoreType instance) {
    SpiDriverComponentBase::init(instance);
  }

  void SpiDriver::configure(uint8_t busId) {
    m_busId = busId;

    if (m_busId == 1) {
      m_hspi = &hspi1; // SPI1: PA5 SCK, PA6 MISO, PA7 MOSI
    } else if (m_busId == 2) {
      m_hspi = &hspi2; // SPI2: PB13 SCK, PB14 MISO, PB15 MOSI
    } else {
      m_hspi = nullptr;
    }

    m_configured = (m_hspi != nullptr);

    if (m_configured) {
      this->log_ACTIVITY_HI_SpiBusOpened(m_busId);
    }
  }

  Drv::SpiStatus SpiDriver::SpiWriteRead_handler(
      FwIndexType portNum,
      Fw::Buffer& writeBuffer,
      Fw::Buffer& readBuffer
  ) {
    FW_ASSERT(portNum == 0);

    if (!m_configured || m_hspi == nullptr) {
      m_spiErrors++;
      this->tlmWrite_SpiErrors(m_spiErrors);
      return Drv::SpiStatus::SPI_OPEN_ERR;
    }

    const uint8_t* txData = writeBuffer.getData();
    uint8_t* rxData = readBuffer.getData();
    const FwSizeType txSize = writeBuffer.getSize();
    const FwSizeType rxSize = readBuffer.getSize();

    // Use larger size for full-duplex exchange
    FwSizeType transferSize = (txSize > rxSize) ? txSize : rxSize;
    if (transferSize == 0) {
      return Drv::SpiStatus::SPI_OK;
    }

    HAL_StatusTypeDef status = HAL_SPI_TransmitReceive(
        m_hspi,
        txData,
        rxData,
        static_cast<uint16_t>(transferSize),
        100 // 100ms bounded timeout
    );

    if (status != HAL_OK) {
      m_spiErrors++;
      this->log_WARNING_HI_SpiTransferError(m_busId, static_cast<U8>(status));
      this->tlmWrite_SpiErrors(m_spiErrors);
      return Drv::SpiStatus::SPI_OTHER_ERR;
    }

    m_bytesTransferred += static_cast<U32>(transferSize);
    this->tlmWrite_BytesTransferred(m_bytesTransferred);

    return Drv::SpiStatus::SPI_OK;
  }

}

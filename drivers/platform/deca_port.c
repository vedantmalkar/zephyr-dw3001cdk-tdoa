#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include "deca_interface.h"
#include "dw3000_hw.h"
#include "dw3000_spi.h"

LOG_MODULE_REGISTER(deca_port, LOG_LEVEL_DBG);

decaIrqStatus_t decamutexon(void)
{
    dw3000_hw_interrupt_disable();
    return 1;
}

void decamutexoff(decaIrqStatus_t s)
{
    dw3000_hw_interrupt_enable();
}

void deca_sleep(unsigned int time_ms)
{
    k_msleep(time_ms);
}

void deca_usleep(unsigned long time_us)
{
    k_usleep(time_us);
}

static const struct dwt_spi_s dw3000_spi_fct = {
    .readfromspi = dw3000_spi_read,
    .writetospi = dw3000_spi_write,
    .writetospiwithcrc = dw3000_spi_write_crc,
    .setslowrate = dw3000_spi_speed_slow,
    .setfastrate = dw3000_spi_speed_fast,
};

const struct dwt_probe_s dw3000_probe_interf = {
    .dw = NULL,
    .spi = (void*)&dw3000_spi_fct,
    .wakeup_device_with_io = dw3000_hw_wakeup,
};

void test_spi_direct(void)
{
    LOG_INF("=== Testing SPI struct directly ===");
    LOG_INF("dw3000_spi_fct addr: %p", &dw3000_spi_fct);
    LOG_INF("  readfromspi: %p", dw3000_spi_fct.readfromspi);
    LOG_INF("  writetospi: %p", dw3000_spi_fct.writetospi);
    LOG_INF("  setslowrate: %p", dw3000_spi_fct.setslowrate);
    LOG_INF("  setfastrate: %p", dw3000_spi_fct.setfastrate);
    
    LOG_INF("=== Testing direct function call ===");
    uint8_t header[1] = {0x00};
    uint8_t data[4] = {0};
    
    int ret = dw3000_spi_fct.readfromspi(1, header, 4, data);
    LOG_INF("Direct call ret=%d", ret);
    LOG_INF("Data: 0x%02X 0x%02X 0x%02X 0x%02X", 
            data[0], data[1], data[2], data[3]);
}
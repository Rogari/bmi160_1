#ifndef __BMI160_H__
#define __BMI160_H__

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <sys/unistd.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/spi_master.h"
#include "driver/gpio.h"

#include "driver/spi_master.h"
#include "esp_log.h"


// datasheet, A.4. IO_MUX
#define PIN_NUM_MISO 19     // GPIO18 I/O/T VSPICLK I/O/T GPIO18 I/O/T HS1_DATA7 I1/O/T
#define PIN_NUM_MOSI 23     // GPIO27 I/O/T               GPIO27 I/O/T
#define PIN_NUM_CLK  18      // GPIO5 I/O/T VSPICS0 I/O/T  GPIO5 I/O/T HS1_DATA6 I1/O/T
#define PIN_NUM_CS   5     // GPIO16 I/O/T               GPIO16 I/O/T HS1_DATA4 I1/O/T

/* SPI Commands *********************************************************************/
#define BMI160_SPI_WRITE_MASK   0x7F
#define BMI160_SPI_READ_MASK    0x80
#define BMI160_COMMAND_REG_ADDR 0x7E
#define BMI160_SOFT_RESET_CMD   0xB6
#define BMI160_START_FOC        0X03
#define BMI160_PRGO_NVM         0XA0

#define BMI160_CHIP_ID          0x00
#define BMI160_ERR_REG          0x02
#define BMI160_PMU_STATUS       0x03
#define BMI160_NV_CONF          0x70
#define BMI160_STATUS           0x1B
#define BMI160_ACC_CONF         0x40
#define BMI160_ACC_RANGE        0x41
#define BMI160_GYR_CONF         0x42
#define BMI160_GYR_RANGE        0x43

#define BMI160_FOC_CONF         0x69
#define BMI160_OFFSET_6         0x77

spi_device_handle_t spi;

void spi_set_data(spi_device_handle_t, uint8_t, uint8_t);
void spi_get_data(spi_device_handle_t, uint8_t);
void spi_data(spi_device_handle_t , const uint8_t *, int);
void spi_read(spi_device_handle_t , const uint8_t *, int );
void spi_cmd(spi_device_handle_t , const uint8_t );
void bmi160_active(void);
void bmi160_deactive(void);
void bmi160_init(spi_device_handle_t);
void bmi160_FOC(void);
#endif


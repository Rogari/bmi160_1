#ifndef __BMI160_H__
#define __BMI160_H__

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/unistd.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_system.h"
#include "soc/gpio_struct.h"
#include "esp_task_wdt.h"

#define CHECK_ERROR_CODE(returned, expected) ({                        \
            if(returned != expected){                                  \
                printf("TWDT ERROR\n");                                \
                abort();                                               \
            }                                                          \
})


// datasheet, A.4. IO_MUX
#define PIN_NUM_MISO 19     // GPIO18 I/O/T VSPICLK I/O/T GPIO18 I/O/T HS1_DATA7 I1/O/T
#define PIN_NUM_MOSI 23     // GPIO27 I/O/T               GPIO27 I/O/T
#define PIN_NUM_CLK  18      // GPIO5 I/O/T VSPICS0 I/O/T  GPIO5 I/O/T HS1_DATA6 I1/O/T
#define PIN_NUM_CS   5     // GPIO16 I/O/T               GPIO16 I/O/T HS1_DATA4 I1/O/T

/* SPI Commands *********************************************************************/
#define BMI160_SPI_WRITE_MASK   0x7F
#define BMI160_SPI_READ_MASK    0x80

#define BMI160_SPI_DUMMY_READ   0x7F
#define BMI160_COMMAND_REG_ADDR 0x7E
#define BMI160_SOFT_RESET_CMD   0xB6
#define BMI160_START_FOC_CMD    0X03
#define BMI160_PROG_NVM_CMD     0XA0

#define BMI160_CHIP_ID          0x00
#define BMI160_ERR_REG          0x02
#define BMI160_PMU_STATUS       0x03
#define BMI160_STATUS           0x1B
#define BMI160_INT_STATUS_0     0x1C
#define BMI160_INT_STATUS_1     0x1D
#define BMI160_INT_STATUS_2     0x1E
#define BMI160_INT_STATUS_3     0x1F

#define BMI160_ACC_CONF         0x40    //Output data rate in Hz 100/2^(8 - val(acc_odr))
#define BMI160_ACC_RANGE        0x41    //2,4,8,16g
#define BMI160_GYR_CONF         0x42
#define BMI160_GYR_RANGE        0x43
#define BMI160_NV_CONF          0x70

#define BMI160_INT_EN_0         0x50
#define BMI160_INT_EN_1         0x51
#define BMI160_INT_EN_2         0x52
#define BMI160_INT_MOTION_0     0x5F
#define BMI160_INT_MOTION_1     0x60
#define BMI160_INT_MOTION_2     0x61

#define BMI160_FOC_CONF         0x69
#define BMI160_OFFSET_6         0x77

typedef enum {
    BMI160_ACCEL_RANGE_2G  = 0X03, /**<  +/-  2g range */
    BMI160_ACCEL_RANGE_4G  = 0X05, /**<  +/-  4g range */
    BMI160_ACCEL_RANGE_8G  = 0X08, /**<  +/-  8g range */
    BMI160_ACCEL_RANGE_16G = 0X0C, /**<  +/- 16g range */
} BMI160AccelRange;

typedef enum {
    BMI160_ACCEL_RATE_25_2HZ = 5,  /**<   25/2  Hz */
    BMI160_ACCEL_RATE_25HZ,        /**<   25    Hz */
    BMI160_ACCEL_RATE_50HZ,        /**<   50    Hz */
    BMI160_ACCEL_RATE_100HZ,       /**<  100    Hz */
    BMI160_ACCEL_RATE_200HZ,       /**<  200    Hz */
    BMI160_ACCEL_RATE_400HZ,       /**<  400    Hz */
    BMI160_ACCEL_RATE_800HZ,       /**<  800    Hz */
    BMI160_ACCEL_RATE_1600HZ,      /**< 1600    Hz */
} BMI160AccelRate;

spi_device_handle_t spi;

void spi_set_data(spi_device_handle_t, uint8_t, uint8_t);
void spi_get_data(spi_device_handle_t, uint8_t);
void spi_data(spi_device_handle_t , const uint8_t *, int);
void spi_read(spi_device_handle_t , const uint8_t *, int );
void spi_cmd(spi_device_handle_t , const uint8_t );
void spi_init(void);
void bmi160_active(void);
void bmi160_deactive(void);
void bmi160_init(void);
void bmi160_set_ACC_range(uint8_t);
void bmi160_set_ACC_ODR(uint8_t);
void bmi160_FOC(void);
#endif


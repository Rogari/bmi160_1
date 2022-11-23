
#include "bmi160_drv.h"

static const char TAG[] = "bmi160";

spi_device_handle_t spi;


void bmi160_max_min(void)
{

}

void bmi160_FOC(void)
{
    uint8_t data[1] = {0};

 /*
    FOC_CONF bit 7      : reserved = 0;
    FOC_CONF bit 6      : foc_gyro_en = 0;
    FOC_CONF bit 5-4    : foc_acc_x = 3; //0b00: disable, 0b01:1g, 0b10:-1g, 0b11: 0g 로 세팅
    FOC_CONF bit 3-2    : foc_acc_y = 3; //0b00: disable, 0b01:1g, 0b10:-1g, 0b11: 0g 로 세팅
    FOC_CONF bit 1-0    : foc_acc_z = 1; //0b00: disable, 0b01:1g, 0b10:-1g, 0b11: 0g 로 세팅
*/

    uint8_t foc_conf = 0b00111101;
    uint8_t offset_6 = 0b01000000;

    spi_set_data(spi, BMI160_OFFSET_6, offset_6);   // FOC(센서 보정 명령)
    spi_set_data(spi, BMI160_FOC_CONF, foc_conf);   //FOC 레지스테 설정(x=0,y=0,z=1)
    spi_set_data(spi, BMI160_COMMAND_REG_ADDR, BMI160_START_FOC_CMD);  //FOC 모드 스타트 명령

    while(1){  // FOC 모드 완료 확인 루틴
        bmi160_active();
        spi_cmd(spi, BMI160_STATUS | BMI160_SPI_READ_MASK);
        spi_read(spi, data, 1);
        bmi160_deactive();
        if ((data[0] & 0x08) == 0x08) {
            break;
        }
        vTaskDelay(1/ portTICK_RATE_MS);
    }
    
    spi_set_data(spi, BMI160_COMMAND_REG_ADDR, BMI160_PROG_NVM_CMD);

}



void bmi160_init(void)
{

    spi_set_data(spi, BMI160_COMMAND_REG_ADDR, BMI160_SOFT_RESET_CMD);  //BMI160 soft reset
    vTaskDelay(1 / portTICK_RATE_MS);

    spi_get_data(spi, BMI160_SPI_DUMMY_READ);                            //for SPI enable
    vTaskDelay(1 / portTICK_RATE_MS);
   
    spi_set_data(spi, BMI160_ACC_RANGE, BMI160_ACCEL_RANGE_2G);         //set ACC range +/- 2g
    vTaskDelay(1 / portTICK_RATE_MS);

    spi_set_data(spi, BMI160_NV_CONF, 0x01);                            //for SPI enable
    vTaskDelay(1 / portTICK_RATE_MS);

    spi_set_data(spi,BMI160_COMMAND_REG_ADDR, 0x11);                    // acc_set_pmu_mode: 0b0001 00nn
    vTaskDelay(1 / portTICK_RATE_MS);
    
}

void spi_get_data(spi_device_handle_t spi, uint8_t addr)
{
    uint8_t buffer[1] = {0};
    uint8_t cmd;

    cmd = addr | BMI160_SPI_READ_MASK;

    bmi160_active();
    spi_cmd(spi,cmd);
    spi_read(spi, buffer, 1);
    bmi160_deactive();

    vTaskDelay(20 / portTICK_RATE_MS);
    
    ESP_LOGI(TAG, "GET DAT from 0x%02X : 0x%02X", addr, buffer[0]);

}

void spi_set_data(spi_device_handle_t spi, uint8_t addr, uint8_t code)
{

    uint8_t cmd;
    uint8_t buffer[1] = {0};

    cmd = addr & BMI160_SPI_WRITE_MASK;
    buffer[0] = code;

    bmi160_active();
    spi_cmd(spi,cmd);
    spi_data(spi, buffer, 1);
    bmi160_deactive();

    vTaskDelay(20 / portTICK_RATE_MS);

    //ESP_LOGI(TAG, "SET DATA 0x%02X to 0x%02X", code, addr);

}

void bmi160_active(void)
{
    gpio_set_level(PIN_NUM_CS, 0);
}

void bmi160_deactive(void)
{
    gpio_set_level(PIN_NUM_CS, 1);
}

void spi_cmd(spi_device_handle_t spi, const uint8_t cmd)
{
    esp_err_t ret;
    spi_transaction_t t;

    memset(&t, 0, sizeof(t));       //Zero out the transaction

    t.length=8;                     //Command is 8 bits
    t.tx_buffer=&cmd;               //The data is the cmd itself

    ret=spi_device_transmit(spi, &t);  //Transmit!
    assert(ret==ESP_OK);            //Should have had no issues.
}

void spi_data(spi_device_handle_t spi, const uint8_t *data, int len)
{
    esp_err_t ret;
    spi_transaction_t t;

    if (len==0) return;             //no need to send anything
    memset(&t, 0, sizeof(t));       //Zero out the transaction

    t.length=len*8;                 //Len is in bytes, transaction length is in bits.
    t.tx_buffer=data;               //Data

    ret=spi_device_transmit(spi, &t);  //Transmit!
    assert(ret==ESP_OK);            //Should have had no issues.
}

void spi_read(spi_device_handle_t spi, const uint8_t *data, int len)
{
    spi_transaction_t t;

    memset(&t, 0, sizeof(t));

    t.length = 8*len;
    t.rx_buffer = (void *)data;

    esp_err_t ret = spi_device_transmit(spi, &t);
    assert( ret == ESP_OK );
}
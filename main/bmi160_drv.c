
#include "bmi160_drv.h"

static const char TAG[] = "main";

void bmi160_init(spi_device_handle_t spi)
{
    uint8_t cmd;
    uint8_t addr;
    uint8_t buffer[1] = {0};

    addr = BMI160_COMMAND_REG_ADDR;
    cmd = addr & BMI160_SPI_WRITE_MASK;
    buffer[0] = BMI160_SOFT_RESET_CMD;

    bmi160_active();
    spi_cmd(spi,cmd);
    spi_data(spi, buffer, 1);
    bmi160_deactive();

    vTaskDelay(1000 / portTICK_RATE_MS);

    addr = BMI160_NV_CONF;
    cmd = addr & BMI160_SPI_WRITE_MASK;
    buffer[0] = 0x01;
   
    bmi160_active();
    spi_cmd(spi,cmd);
    spi_data(spi, buffer, 1);
    bmi160_deactive();

//    vTaskDelay(1000 / portTICK_RATE_MS);

    addr = BMI160_COMMAND_REG_ADDR;
    cmd = addr & BMI160_SPI_WRITE_MASK;
    buffer[0] = 0x11;
   
    bmi160_active();
    spi_cmd(spi,cmd);
    spi_data(spi, buffer, 1);
    bmi160_deactive();


    ESP_LOGI(TAG, "SET DATA 0x%02X to 0x%02X", 0X11, BMI160_COMMAND_REG_ADDR);

    
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
    
    ESP_LOGI(TAG, "SET DATA 0x%02X to 0x%02X", code, addr);

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
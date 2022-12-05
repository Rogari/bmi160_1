#include "sdkconfig.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "esp_err.h"
#include <sys/unistd.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "driver/spi_master.h"
#include "soc/gpio_struct.h"
#include "driver/gpio.h"
#include "esp_task_wdt.h"
#include "esp_err.h"

#include "esp_log.h"

#include "bmi160_drv.h"

void read_vib_data(void);

static const char TAG[] = "main";

#define PIN_NUM_MISO 19     // GPIO18 I/O/T VSPICLK I/O/T GPIO18 I/O/T HS1_DATA7 I1/O/T
#define PIN_NUM_MOSI 23     // GPIO27 I/O/T               GPIO27 I/O/T
#define PIN_NUM_CLK  18      // GPIO5 I/O/T VSPICS0 I/O/T  GPIO5 I/O/T HS1_DATA6 I1/O/T
#define PIN_NUM_CS   5     // GPIO16 I/O/T               GPIO16 I/O/T HS1_DATA4 I1/O/T

#define CHECK_ERROR_CODE(returned, expected) ({                        \
            if(returned != expected){                                  \
                printf("TWDT ERROR\n");                                \
                abort();                                               \
            }                                                          \
})

void init_SPI(void)
{
    esp_err_t ret;
//    spi_device_handle_t spi;

    spi_bus_config_t buscfg={
        .miso_io_num = PIN_NUM_MISO,
        .mosi_io_num = PIN_NUM_MOSI,
        .sclk_io_num = PIN_NUM_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 0, // 0 is default 4096
        .flags = 0,
    };

    spi_device_interface_config_t devcfg={
      .command_bits = 0,
      .address_bits = 0,
      .dummy_bits = 0,
      .mode = 0, // 0 ~ 3, SPI modes '00' (CPOL=CPHA=0) and '11' (CPOL=CPHA=1).
      .duty_cycle_pos = 0, // 0 is default 128, 50% duty
      .cs_ena_pretrans = 0,
      .cs_ena_posttrans = 0,
      .clock_speed_hz = SPI_MASTER_FREQ_20M,
      .input_delay_ns = 0,
      .spics_io_num = -1,// PIN_NUM_CS,
      .flags = 0,
      .queue_size = 3,
      .pre_cb = NULL,
      .post_cb = NULL,
    };

    gpio_set_direction(PIN_NUM_CS, GPIO_MODE_OUTPUT);
    gpio_set_pull_mode(PIN_NUM_CS, GPIO_PULLUP_ONLY);

    
    //Initialize the SPI bus
    ret=spi_bus_initialize(HSPI_HOST, &buscfg, 1);
    ESP_ERROR_CHECK(ret);

    //Attach the SPI bus
    ret=spi_bus_add_device(HSPI_HOST, &devcfg, &spi);
    ESP_ERROR_CHECK(ret);
}

void read_vib_data(void)
{

    //12번지 부터 9개의; 데이타를 읽을 예정
    uint8_t data[9] = {0,};
    uint16_t num_Data=500;

    uint8_t addr = 0x12;
    uint8_t cmd = addr | BMI160_SPI_READ_MASK;

    uint16_t i;

    CHECK_ERROR_CODE(esp_task_wdt_init(20, false), ESP_OK);


    //while(1)
    {

        ESP_LOGI(TAG, "Memory Setting"); 
        
        float * arr_x = (float*) malloc(sizeof(float) * num_Data);
        float * arr_y = (float*) malloc(sizeof(float) * num_Data);
        float * arr_z = (float*) malloc(sizeof(float) * num_Data);
        uint32_t * t1 = (uint32_t*) malloc(sizeof(uint32_t) * num_Data);


        for(i=0;i< num_Data; i++){


            while(1){
                bmi160_active();
                spi_cmd(spi, 0x1B | BMI160_SPI_READ_MASK);
                spi_read(spi, data, 1);
                bmi160_deactive();
                if ((data[0] & 0x80) == 0x80) {
                    break;
                }
                vTaskDelay(1/ portTICK_RATE_MS);
            }

            bmi160_active();
            spi_cmd(spi,cmd);
            spi_read(spi, data, 9);
            bmi160_deactive();

            int16_t x_data = data[1]<<8 | data[0];
            int16_t y_data = data[3]<<8 | data[2];
            int16_t z_data = data[5]<<8 | data[4];

            float xx = x_data/(32768.0 / 2 );  //가속도 측정 범위 = 2 
            float yy = y_data/(32768.0 / 2 );  //가속도 측정 범위 = 2
            float zz = z_data/(32768.0 / 2 );  //가속도 측정 범위 = 2

            uint32_t sensor_time = data[8]<<16 | data[7]<<8 | data[6];

            arr_x[i] = xx;    
            arr_y[i] = yy;    
            arr_z[i] = zz;    
            t1[i] = sensor_time;
            
            //ACC_data[i]=xx; //[0]=xx;ACC_data[i][1]=yy;ACC_data[i][2]=zz;
            //ST_data[i] = sensor_time;

            //ESP_LOGI(TAG, "x:%d, y:%d, z:%d", x_data, y_data, z_data); 
            //ESP_LOGI(TAG, "st: %x, x:%7.4f, y:%7.4f, z:%7.4f", sensor_time, xx, yy, zz); 
    /*
            ESP_LOGI(TAG, "0x%02X, 0x%02X, 0x%02X, 0x%02X, 0x%02X, 0x%02X, 0x%02X, 0x%02X, 0x%02X, 0x%02X, 0x%02X, 0x%02X, 0x%02X, 0x%02X, 0x%02X, 0x%02X, 0x%02X, 0x%02X, 0x%02X, 0x%02X", 
                data[0], data[1], data[2], data[3], data[4], data[5], data[6], data[7],data[8], data[9],
                data[10], data[11], data[12], data[13], data[14], data[15], data[16], data[17],data[18], data[19] );
    */    
            vTaskDelay(1/ portTICK_RATE_MS);

/*
            if(i%100==0) {
                CHECK_ERROR_CODE(esp_task_wdt_reset(), ESP_OK);  //Comment this line to trigger a TWDT timeout
                ESP_LOGI(TAG, "Reset Watch Dog");
            }
*/
        }

        
        for(i=0;i<num_Data;i++){

            if (i > 0) {
                int td = t1[i]-t1[i-1];
                ESP_LOGI(TAG, "t:%d, x:%7.4f y:%7.4f z:%7.4f",  t1[i], arr_x[i], arr_y[i], arr_z[i]);
                vTaskDelay(10/ portTICK_RATE_MS);


                if(i%500==0) {
                    esp_task_wdt_reset();
                    ESP_LOGI(TAG, "Reset Watch Dog");
                }

            }

            //ESP_LOGI(TAG, "x:%7.4f",  arr[i]); 
            //ESP_LOGI(TAG, "st:%x, x:%7.4f, y:%7.4f, z:%7.4f", ST_data[i], ACC_data[i]); //[0], ACC_data[i][1], ACC_data[i][2]); 

        }

        float tt = ((((t1[num_Data-1] - t1[0])) >> 4 )+1) * 10; 
        ESP_LOGI(TAG, "x:%f",  tt);
        vTaskDelay(1000/ portTICK_RATE_MS);

        free(t1);
        free(arr_x);
        free(arr_y);
        free(arr_z);


    }
    
    //CHECK_ERROR_CODE(esp_task_wdt_deinit(), ESP_OK);
    //CHECK_ERROR_CODE(esp_task_wdt_status(NULL), ESP_ERR_INVALID_STATE);     //Confirm TWDT has been deinitialized
    CHECK_ERROR_CODE(esp_task_wdt_init(5, false), ESP_OK);

}

void app_main()
{

    init_SPI();

    vTaskDelay(1000 / portTICK_RATE_MS);

    bmi160_init();
    bmi160_FOC();

    vTaskDelay(1000 / portTICK_RATE_MS);

    spi_set_data(spi, BMI160_INT_EN_0, 0b00000111); //any motion interrupt enable(x,y,z axis)
    spi_set_data(spi, BMI160_INT_MOTION_1, 0x2F);   //가속도가 0.5g이상이면 인터럽터 발생
    spi_set_data(spi, BMI160_INT_MOTION_0, 0x0F);   //

    uint8_t data[9] = {0,};

    for(;;){
        int i = 0;
        while(1){
            bmi160_active();
            spi_cmd(spi, BMI160_INT_STATUS_0 | BMI160_SPI_READ_MASK);
            spi_read(spi, data, 1);
            bmi160_deactive();
/*
            if(i >=1000) {
                i=0;
                esp_task_wdt_reset();
                ESP_LOGI(TAG, "Reset Watch Dog");
            } else {
                i++;
            }
*/           
            if ((data[0] & 0x04) == 0x04) {
                break;
            }
            vTaskDelay(20/ portTICK_RATE_MS);
        }

        spi_get_data(spi, BMI160_INT_STATUS_2);

        vTaskDelay(1000 / portTICK_RATE_MS);
        //read_vib_data();
    }

}


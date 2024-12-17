// Copyright 2015-2022 Espressif Systems (Shanghai) PTE LTD
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at

//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_event.h"
#include "esp_event_base.h"
#include "bsp_board.h"
#include "lv_port.h"
#include "ui/ui.h"
#include "driver/uart.h"
#include "indicator_btn.h"
#include "pwm_audio.h"
#include "audio_data1.h"

int power = 1;
int PWM_SET = 0;

#include <stdio.h>
#include "driver/i2c.h"
#include "esp_log.h"

#define I2C_MASTER_SCL_IO 40    // SCL引脚
#define I2C_MASTER_SDA_IO 39    // SDA引脚
#define I2C_MASTER_NUM I2C_NUM_0 // I2C端口号
#define I2C_MASTER_FREQ_HZ 100000 // I2C频率
#define PCA9535_ADDRESS 0x20    // 根据你的硬件配置调整地址

static const char *TAG = "PCA9535";

void i2c_master_init() {
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    i2c_param_config(I2C_MASTER_NUM, &conf);
    i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
}

void pca9535_write_register(uint8_t reg_addr, uint8_t data) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (PCA9535_ADDRESS << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg_addr, true);
    i2c_master_write_byte(cmd, data, true);
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
}

uint8_t pca9535_read_register(uint8_t reg_addr) {
    uint8_t data = 0;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (PCA9535_ADDRESS << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg_addr, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (PCA9535_ADDRESS << 1) | I2C_MASTER_READ, true);
    i2c_master_read_byte(cmd, &data, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return data;
}

void IIC_INT(){
    i2c_master_init();
    ESP_LOGI(TAG, "I2C INT OK");

    // 配置PCA9535的I/O方向为输出
    pca9535_write_register(0x06, 0x00); // 配置P0端口为输出
    pca9535_write_register(0x07, 0x00); // 配置P1端口为输出

    // 设置IO1_1为高电平
    uint8_t output_p1 = pca9535_read_register(0x03); // 读取P1输出寄存器
    output_p1 |= ~(1 << 1); // 设置P1_1为高电平
    pca9535_write_register(0x03, output_p1); // 写回P1输出寄存器

    ESP_LOGI(TAG, "PCA9535 IS OK IO1_1 IS LOW");
}


#define VERSION      "v1.0.0"
#define LOG_MEM_INFO 1
#define SENSECAP     "\n\
   _____                      _________    ____         \n\
  / ___/___  ____  ________  / ____/   |  / __ \\       \n\
  \\__ \\/ _ \\/ __ \\/ ___/ _ \\/ /   / /| | / /_/ /   \n\
 ___/ /  __/ / / (__  )  __/ /___/ ___ |/ ____/         \n\
/____/\\___/_/ /_/____/\\___/\\____/_/  |_/_/           \n\
--------------------------------------------------------\n\
 Version: %s %s %s\n\
--------------------------------------------------------\n\
"

ESP_EVENT_DEFINE_BASE(VIEW_EVENT_BASE);
esp_event_loop_handle_t view_event_handle;

extern void indicator_model_init(void);
extern void indicator_view_init(void);

extern char buffer [];

int audio_set = 0;

#include "indicator_lorawan.h"

extern TaskHandle_t lorawanTaskHandle;

void app_main(void) {
    int level = gpio_get_level(38);
    if (level)
    {
        ESP_LOGI("TAG", "power on btn");
        vTaskDelay(3000 / portTICK_PERIOD_MS);
    }
    
    int audio_hz = 16000;//采样率
    int audio_bit = 16;//音频位数
    
    int audio_vol = -15;//音量-16~16
    ESP_LOGI("TAG", "system start");
    ESP_LOGI("", SENSECAP, VERSION, __DATE__, __TIME__);

    ESP_ERROR_CHECK(bsp_board_init());
    lv_port_init();

#if CONFIG_LCD_AVOID_TEAR
    ESP_LOGI(TAG, "Avoid lcd tearing effect");
    #if CONFIG_LCD_LVGL_FULL_REFRESH
    ESP_LOGI(TAG, "LVGL full-refresh");
    #elif CONFIG_LCD_LVGL_DIRECT_MODE
    ESP_LOGI(TAG, "LVGL direct-mode");
    #endif
#endif

    esp_event_loop_args_t view_event_task_args = {
        .queue_size      = 10,
        .task_name       = "view_event_task",
        .task_priority   = uxTaskPriorityGet(NULL),
        .task_stack_size = 10240,
        .task_core_id    = tskNO_AFFINITY
    };
    ESP_ERROR_CHECK(esp_event_loop_create(&view_event_task_args, &view_event_handle));
    indicator_view_init();
    indicator_model_init();
    IIC_INT();
        pwm_audio_config_t pac;
        pac.duty_resolution    = LEDC_TIMER_10_BIT;
        pac.gpio_num_left      = 46;
        pac.ledc_channel_left  = LEDC_CHANNEL_1;
        pac.gpio_num_right     = -1;  // 不使用右声道
        pac.ledc_channel_right = LEDC_CHANNEL_0;
        pac.ledc_timer_sel     = LEDC_TIMER_0;
        pac.ringbuf_len        = 1024 * 8;
        pwm_audio_init(&pac);
        pwm_audio_set_volume(audio_vol); // 设置音量为 9.1% 对应 0.3V 输出
        pwm_audio_set_param(audio_hz, audio_bit, 1);  // 设置采样率、位数和声道数（单声道）
        pwm_audio_start();
    vTaskDelay(500 / portTICK_PERIOD_MS);
    //size_t length = 62937;
    size_t length = sizeof(audio_data) / sizeof(audio_data[0]);
    size_t index = 0;
    size_t cnt;
    uint32_t block_w = 256;
    ESP_LOGI(TAG, "AUDIO PLAY111 %d",index);
    /////////////////////
    //每次上电后确保wifi和lora关闭，并播放一次音频后，在打开wifi和lora
    /////////////////////

#if LOG_MEM_INFO
    static char buffer[128]; /* Make sure buffer is enough for `sprintf` */
    while (1) {
    if (PWM_SET == 1 && audio_set == 0) {
        // 设置IO1_1为高电平
        uint8_t output_p1 = pca9535_read_register(0x03); // 读取P1输出寄存器
        output_p1 |= (1 << 1); // 设置P1_1为高电平
        pca9535_write_register(0x03, output_p1); // 写回P1输出寄存器

        ESP_LOGI(TAG, "PCA9535 IS OK IO1_1 IS HIGH");
        //vTaskDelay(pdMS_TO_TICKS(100));
        pwm_audio_init(&pac);
        pwm_audio_set_volume(audio_vol); // 设置音量为 9.1% 对应 0.3V 输出
        pwm_audio_set_param(audio_hz, audio_bit, 1);  // 设置采样率、位数和声道数（单声道）
        pwm_audio_start();
        audio_set = 1;
    }

    if (PWM_SET == 0 && audio_set == 1) {
        uint8_t output_p1 = pca9535_read_register(0x03); // 读取P1输出寄存器
        output_p1 &= ~(1 << 1); // 设置P1_1为低电平
        pca9535_write_register(0x03, output_p1); // 写回P1输出寄存器
        audio_set = 0;
        pwm_audio_stop();
        pwm_audio_deinit();
        index = 0;
        
    }

    if (audio_set == 1 && PWM_SET == 1) {
        ESP_LOGI(TAG, "AUDIO PLAY %d",index);
        if (index < length / 4) {
            if ((length - index) < block_w) {
                block_w = length - index;
            }
            pwm_audio_write((uint8_t *)(audio_data + index*2), block_w * sizeof(uint16_t), &cnt, 3000 / portTICK_PERIOD_MS);
            index += cnt / sizeof(uint16_t);
        } else {
            index = 0;  // 重置索引以重新播放
            // if (player_set >= 1)
            // {
            //     audio_set = 1;
            //     PWM_SET = 0;/* code */
            // }
            // player_set ++;

            //audio_set = 1;
            PWM_SET = 0;/* code */
            // vTaskDelay(60000 / portTICK_PERIOD_MS);//间隔10s
            // PWM_SET = 1;
        }
    }

    }
#endif
}

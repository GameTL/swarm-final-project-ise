/*
 * Copyright (c) 2019 David Antliff
 *
 * This program provides an example using the esp32-rotary-encoder component.
 * Events are received via an event queue and displayed on the serial console.
 * The task also polls the device position every second to show that the latest
 * event always matches the current position.
 *
 * esp32-rotary-encoder is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * esp32-rotary-encoder is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with esp32-rotary-encoder.  If not, see <https://www.gnu.org/licenses/>.
 */
// STD Libraries
#define TAG "app"
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
// ESP Libraries
#include "esp_system.h"
#include "esp_log.h"
#include "esp_timer.h"

// Encoder
#include "rotary_encoder.h"
#define ROT_ENC_A_GPIO (CONFIG_ROT_ENC_A_GPIO) // Deteremine in menu-config
#define ROT_ENC_B_GPIO (CONFIG_ROT_ENC_B_GPIO) // Deteremine in menu-config
#define ENABLE_HALF_STEPS false                // Set to true to enable tracking of rotary encoder at half step resolution
#define RESET_AT 0                             // Set to a positive non-zero number to reset the position if this value is exceeded
#define FLIP_DIRECTION false                   // Set to true to reverse the clockwise/counterclockwise sense

// Motor
#include "driver/ledc.h"
#define ENC_PULSE_PER_REV 11                             // 11 pulse/rev
#define GEAR_RATIO 3 / 19                                // Gear Ratio of 1/6.3 assumed to be 1/6.333333...
#define MOTOR_PULSEPERREV ENC_PULSE_PER_REV / GEAR_RATIO // 69.6666... which is not so nice.
// PWN Configs
#define LEDC_TIMER LEDC_TIMER_0
#define LEDC_MODE LEDC_LOW_SPEED_MODE
#define LEDC_OUTPUT_IO (6) // Define the output GPIO
#define LEDC_CHANNEL LEDC_CHANNEL_0
#define LEDC_DUTY_RES LEDC_TIMER_13_BIT // Set duty resolution to 13 bits
#define LEDC_DUTY (4096)                // Set duty to 50%. (2 ** 13) * 50% = 4096 | Max Range: 0 - 8192 |
#define LEDC_DUTY_HIGH (4096)           // Set duty to 50%. (2 ** 13) * 50% = 4096
#define LEDC_DUTY_LOW (2048)            // Set duty to 50%. (2 ** 13) * 50% = 4096
#define LEDC_FREQUENCY (4000)           // Frequency in Hertz. Set frequency at 4 kHz

typedef float pos_rad;
typedef float pos_pul;
typedef float vel_rad_per_sec;
typedef float vel_pul_per_sec;
// --- Control Loop Timing ---
#define CONTROL_LOOP_PERIOD_MS 1 // Target 1ms period

static void ledc_init(void)
{
    // Prepare and then apply the LEDC PWM timer configuration
    ledc_timer_config_t ledc_timer = {
        .speed_mode = LEDC_MODE,
        .duty_resolution = LEDC_DUTY_RES,
        .timer_num = LEDC_TIMER,
        .freq_hz = LEDC_FREQUENCY, // Set output frequency at 4 kHz
        .clk_cfg = LEDC_AUTO_CLK};
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    // Prepare and then apply the LEDC PWM channel configuration
    ledc_channel_config_t ledc_channel = {
        .speed_mode = LEDC_MODE,
        .channel = LEDC_CHANNEL,
        .timer_sel = LEDC_TIMER,
        .intr_type = LEDC_INTR_DISABLE,
        .gpio_num = LEDC_OUTPUT_IO,
        .duty = 0, // Set duty to 0%
        .hpoint = 0};
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));
}
static void pin_init()
{
    gpio_set_direction(GPIO_NUM_8, GPIO_MODE_OUTPUT);
    gpio_set_direction(GPIO_NUM_9, GPIO_MODE_OUTPUT);
    gpio_set_level(GPIO_NUM_8, 1);
    gpio_set_level(GPIO_NUM_9, 0);
}

void app_main()
{
    pin_init();
    // For Printing
    static int last_print_pos = 0;      // Checking: Encoder Position
    static int64_t last_print_time = 0; // Checking: Program Time
    rotary_encoder_position_t previous_encoder_position = 0;

    // Set the LEDC peripheral configuration
    ledc_init();
    // Set duty to 50%
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, LEDC_DUTY));
    // Update duty to apply the new value
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL));

    // esp32-rotary-encoder requires that the GPIO ISR service is installed before calling rotary_encoder_register()
    ESP_ERROR_CHECK(gpio_install_isr_service(0));
    // TODO Add interupts for UART
    // Initialise the rotary encoder device with the GPIOs for A and B signals
    rotary_encoder_info_t info = {0};
    QueueHandle_t event_queue_en1 = rotary_encoder_create_queue();
    ESP_ERROR_CHECK(rotary_encoder_init(&info, ROT_ENC_A_GPIO, ROT_ENC_B_GPIO));
    ESP_ERROR_CHECK(rotary_encoder_enable_half_steps(&info, ENABLE_HALF_STEPS));
    #ifdef FLIP_DIRECTION
    ESP_ERROR_CHECK(rotary_encoder_flip_direction(&info));
    #endif
    
    // Create a queue for events from the rotary encoder driver.
    // Tasks can read from this queue to receive up to date position information.
    ESP_ERROR_CHECK(rotary_encoder_set_queue(&info, event_queue_en1));
    // Encoder 1
    rotary_encoder_position_t prev_pos_en1 = 0;
    pos_rad curr_pos_rad = 0;
    pos_rad prev_pos_rad = 0;

    vel_pul_per_sec prev_vel_en1 = 0;
    vel_pul_per_sec curr_vel_en1 = 0;
    int64_t prev_time_e1 = esp_timer_get_time() / 1000; // microsecond --> milisec
    int64_t curr_time_e1 = esp_timer_get_time() / 1000; // microsecond --> milisec
    ESP_LOGI(TAG, "Encoder and Motor Starting...");
    while (1)
    {
        
        vel_pul_per_sec urr_vel_en1 = 0;
        rotary_encoder_event_t event_en1 = {0};
        
        //* Encoder 1
        //* All calculation code will stay in this block of code, Only do work when there's interupts.
        if (xQueueReceive(event_queue_en1, &event_en1, 100 / portTICK_PERIOD_MS) == pdTRUE)
        {
            int64_t curr_time_e1 = esp_timer_get_time() / 1000; // microsecond --> milisec
            float delta_t_e1 = (float)(curr_time_e1 - prev_time_e1) / 1000.0f;
            
            /*
            - calculate the velocity
            - There's a new interupt
            - overwrite the position
            */
           curr_pos_rad = event_en1.state.position * 6.28 / MOTOR_PULSEPERREV;
           vel_rad_per_sec curr_ang_vel_en1 = (float)(curr_pos_rad - prev_pos_rad) / delta_t_e1;
           // ESP_LOGI(TAG, "pos: %d, d: %s, vel_pulse: %.10f, vel_rad: %.10f", event_en1.state.position,
           //     event_en1.state.direction ? (event_en1.state.direction == ROTARY_ENCODER_DIRECTION_CLOCKWISE ? "CW" : "CCW") : "NOT_SET", curr_vel_en1, curr_ang_vel_en1);
           ESP_LOGI(TAG, "pos: %d, d: %s, vel_pulse: %.4f, vel_rad: %.4f, delta_t: %.4f ms", 
            event_en1.state.position,
            event_en1.state.direction ? (event_en1.state.direction == ROTARY_ENCODER_DIRECTION_CLOCKWISE ? "CW" : "CCW") : "NOT_SET", 
            curr_vel_en1, 
            curr_ang_vel_en1,
            delta_t_e1 * 1000);
            // Store var for next interupts
            prev_vel_en1 = curr_vel_en1;
            prev_pos_en1 = event_en1.state.position;
            prev_time_e1 = curr_time_e1;
            prev_pos_rad = curr_pos_rad;
        }
        // Motor 1
        vel_rad_per_sec target_ang_vel_m1 = 0;
        vel_pul_per_sec target_vel_m1 = 0;
        target_vel_m1 = target_ang_vel_m1 * 1.751592356; // 11/6.28 =  1.751592356
        vel_pul_per_sec prev_err_vel_m1 = 0;
        vel_pul_per_sec curr_err_vel_m1 = target_vel_m1 - curr_vel_en1;
        vel_pul_per_sec i_err_vel = 0;
        // PID 
        float Kp = 1;
        float Ki = 0.1;
        float Kd = 0.1;
        if (curr_err_vel_m1 > 10)
        {

            float d_err_vel = curr_err_vel_m1 - prev_err_vel_m1;
            i_err_vel = i_err_vel + curr_err_vel_m1;
            float out_m1 = (Kp * curr_err_vel_m1) + (Ki * i_err_vel) + (Kd * d_err_vel);
            
            // Store var for next interupts
            prev_err_vel_m1 = curr_err_vel_m1;
            ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, LEDC_DUTY));
            // TODO set the direction of the pins 
            
            ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL));
            // TODO: adjust the 
        }
        
        // prev_pos = event_en1.state.position;
        // if (abs(event_en1.state.position - last_print_pos) >= 10 || (current_time - last_print_time) >= 100)
        // {
            //     ESP_LOGI(TAG, "Event: position %d, direction %s, velocity %.20f, acceleration %.20f", event_en1.state.position,
            //              event_en1state.direction ? (event_en1state.direction == ROTARY_ENCODER_DIRECTION_CLOCKWISE ? "CW" : "CCW") : "NOT_SET", vel, accel);
            //     last_print_pos = event_en1.state.position;
            //     last_print_time = curr_time;
            // } // position change more than 10 or been 100ms
    };
    ESP_LOGE(TAG, "queue receive failed");
    ESP_ERROR_CHECK(rotary_encoder_uninit(&info));
}

#include <stdio.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_spi_flash.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/i2c.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "ssd1306.h"
#include "font8x8_basic.h"
#include "image.h"
#include <string.h>
#include "esp_timer.h"
#include "ssd1306_i2c_new.c"
#include "esp_sleep.h"
#include <math.h>

#define TAG "SSD1306"
#define TAG "MAX17260"

#define CONFIG_SDA_GPIO 1
#define CONFIG_SCL_GPIO 2
#define CONFIG_RESET_GPIO 0

#define BTN_UP GPIO_NUM_14
#define BTN_DOWN GPIO_NUM_17
#define BTN_PWR GPIO_NUM_15
#define BTN_OK GPIO_NUM_16

#define IN1 GPIO_NUM_3
#define IN2 GPIO_NUM_5
#define EN1 GPIO_NUM_4
#define EN2 GPIO_NUM_6
#define nRESET GPIO_NUM_34
#define nSLEEP GPIO_NUM_33

#define WINDOW_SIZE 10
#define THRESH_LOW -1.5
#define THRESH_HIGH 0

//PWM Controller declarations
#define LEDC_TIMER              LEDC_TIMER_0
#define LEDC_MODE               LEDC_LOW_SPEED_MODE
#define LEDC_OUTPUT_IO          (21) // Define the output GPIO
#define LEDC_CHANNEL            LEDC_CHANNEL_0
#define LEDC_DUTY_RES           LEDC_TIMER_13_BIT // Set duty resolution to 13 bits
#define LEDC_DUTY               (0) // 8192 for 100%. To be decided.
#define LEDC_FREQUENCY          (5000) // Frequency in Hertz. Set frequency at 5 kHz


static char tag[] = "mpu6050";

#undef ESP_ERROR_CHECK
#define ESP_ERROR_CHECK(x)   do { esp_err_t rc = (x); if (rc != ESP_OK) { ESP_LOGE("err", "esp_err_t = %d", rc); assert(0 && #x);} } while(0);



SSD1306_t dev;
bool isDisp_menu;
int Disp_menu_count = 1;
int pos_count = 1;
bool isDisp_setup;
bool isDisp_walk;
bool isDisp_pos;

xQueueHandle BTN_UPQueue;
xQueueHandle BTN_DOWNQueue;
xQueueHandle BTN_PWRQueue;
xQueueHandle BTN_OKQueue;
xQueueHandle FIRE_ONQUEUE;
xQueueHandle FIRE_OFFQUEUE;


static esp_timer_handle_t fire_timer;
static bool fire_on = false;
static bool below_neg_15 = false;
bool t_detect_flag = true;
float angle_roll = 0;

int strength_current = 0;

int STIMStrength[] = {0, 819, 1638, 2457, 3276, 4095, 4914, 5733, 6552, 7371, 8190};

uint32_t pre_time_up = 0;
uint32_t pre_time_down = 0;
uint32_t pre_time_pwr = 0;
uint32_t pre_time_ok = 0;
uint64_t pre_time= 0;
uint64_t intr_time = 0;
uint64_t curr_time = 0;
int freq_list_index = 0;

int16_t accel_x, accel_y, accel_z;
int16_t gyro_x, gyro_y, gyro_z;


float soc =0;
int freq_list[11] = {0,1,2,3,4,5,6,7,8,9};

bool long_press_detected = false;
bool animation_running = false;


static void example_ledc_init(void)
{
    // Prepare and then apply the LEDC PWM timer configuration
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_MODE,
        .timer_num        = LEDC_TIMER,
        .duty_resolution  = LEDC_DUTY_RES,
        .freq_hz          = LEDC_FREQUENCY,  // Set output frequency at 5 kHz
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    // Prepare and then apply the LEDC PWM channel configuration
    ledc_channel_config_t ledc_channel = {
        .speed_mode     = LEDC_MODE,
        .channel        = LEDC_CHANNEL,
        .timer_sel      = LEDC_TIMER,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = LEDC_OUTPUT_IO,
        .duty           = LEDC_DUTY, // Set duty to 0%
        .hpoint         = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));
}


float moving_average_filter(float *window, int window_size) {
    float sum = 0;
    for (int i = 0; i < window_size; i++) {
        sum += window[i];
    }
    return sum / window_size;
}

bool peak_detected(float curr_val, float prev_val)
{
	if(curr_val < prev_val)
		return true;
	return false;
}
bool trough_detected(float curr_val, float prev_val)
{
	if(curr_val > prev_val)
		return true;
	return false;
}

void enter_deep_sleep() {
    ESP_LOGI("NO TAG", "Entering deep sleep in 500ms");
    vTaskDelay(500 / portTICK_PERIOD_MS); // Wait for 500ms to ensure button is released
    esp_sleep_enable_ext0_wakeup(BTN_PWR, 0); // Wake up when button is pressed (falling edge)
    ssd1306_clear_screen(&dev, false);
    esp_deep_sleep_start();
}


void display_logo()
{
    ssd1306_init(&dev, 128, 64);
    vTaskDelay(500 / portTICK_PERIOD_MS);
    ssd1306_bitmaps(&dev, 0, 0, biostim_logo, 128, 64, false);
    vTaskDelay(2000 / portTICK_PERIOD_MS);
    ssd1306_clear_screen(&dev, false);
}

void disp_menu()
{
    
    isDisp_setup = false;
    isDisp_walk=false;
    isDisp_menu=true;
    char str[4];
    char ch1 = 'F';
    char ch2 = 'S';
    char display_str[20];
    char display_str1[20];
    char display_str2[20];
    int frequency = freq_list[freq_list_index];
    snprintf(str,sizeof(str),"%d",frequency);
    snprintf(display_str,sizeof(display_str)," Setup        %s",str);
    snprintf(display_str1, sizeof(display_str1), " Position     %c", ch1);
    snprintf(display_str2, sizeof(display_str2), " Position     %c", ch2);
    //ESP_LOGI(TAG, "State of Charge: %.2f%%", soc);
    if(soc>=75.0)
    {
        ssd1306_bitmaps(&dev, 100, 5, battery_100, 32, 17, false);        
    }
    if(soc<75.0&&soc>=50.0)
    {
        ssd1306_bitmaps(&dev, 100, 5, battery_75, 32, 17, false);
    }
    if(soc<50.0&&soc>=25.0)
    {
        ssd1306_bitmaps(&dev, 100, 5, battery_50, 32, 17, false);
    }
    if(soc<25.0)
    {
        ssd1306_bitmaps(&dev, 100, 5, battery_low, 32, 17, false);
    }

    if(Disp_menu_count==1)
    {
        
        ssd1306_display_text(&dev,3, display_str,strlen(display_str),true);
        if(pos_count==1)
        {
            ssd1306_display_text(&dev,5, display_str1,strlen(display_str1),false);
        }
        if(pos_count==2)
        {
            ssd1306_display_text(&dev,5, display_str2,strlen(display_str2),false);
        }
        ssd1306_display_text(&dev, 7, " Walk        ", 16, false);
    }
    if(Disp_menu_count==2)
    {
        
        ssd1306_display_text(&dev,3, display_str,strlen(display_str),false);
        if(pos_count==1)
        {
            ssd1306_display_text(&dev,5, display_str1,strlen(display_str1),true);
        }
        if(pos_count==2)
        {
            ssd1306_display_text(&dev,5, display_str2,strlen(display_str2),true);
        }

        ssd1306_display_text(&dev, 7, " Walk        ", 16, false);
    }
    if(Disp_menu_count==3)
    {
        
        ssd1306_display_text(&dev,3, display_str,strlen(display_str),false);
        if(pos_count==1)
        {
            ssd1306_display_text(&dev,5, display_str1,strlen(display_str1),false);
        }
        if(pos_count==2)
        {
            ssd1306_display_text(&dev,5, display_str2,strlen(display_str2),false);
        }
        ssd1306_display_text(&dev, 7, " Walk        ", 16, true);
    }
    

}


void disp_setup()
{
    isDisp_menu=false;
    isDisp_walk=false;
    isDisp_setup=true;
    

    if(freq_list_index>0)
    {
            ssd1306_bitmaps(&dev, 50, 15, flash, 32, 26, false);
            ssd1306_bitmaps(&dev, 106, 35,str_num[freq_list_index], 16,9,false);
    }
    if(freq_list_index==0)
    {
        ssd1306_bitmaps(&dev, 0, 0, setup_1, 128, 64, false);
        ssd1306_bitmaps(&dev, 106, 35,str_num[freq_list_index], 16,9,false);
    }
}

void mpu6050_get_data(void *params)
{
    while (1) {
        if(isDisp_walk)
            {      
                mpu6050_read_data(&accel_x, &accel_y, &accel_z, &gyro_x, &gyro_y, &gyro_z);
                // ESP_LOGI(TAG, "Accel X: %d, Y: %d, Z: %d", accel_x, accel_y, accel_z);
                // ESP_LOGI(TAG, "Gyro X: %d, Y: %d, Z: %d", gyro_x, gyro_y, gyro_z);
                printf("2000 %d %d %d -2000\n",gyro_x,gyro_y,gyro_z);
            }

        vTaskDelay(100 / portTICK_PERIOD_MS); // Delay for 1 second
    }


}
void fire_off_callback(void* arg) {
    printf("-----Fire OFF (Timer)----\n");
    gpio_set_level(IN1, 0);
    gpio_set_level(IN2, 0);
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, 0));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL));
    printf("Stimulation off(timer)\n");
    // ssd1306_clear_screen(&dev, false);
    xQueueSend(FIRE_OFFQUEUE,&angle_roll,NULL);
    fire_on = false;
    below_neg_15 = false;
    t_detect_flag = true;
    vTaskDelay(100 / portTICK_PERIOD_MS);
}

void STIMTask(void *params)
{
    example_ledc_init();
    gpio_set_direction(IN1, GPIO_MODE_OUTPUT);
    gpio_set_direction(IN2, GPIO_MODE_OUTPUT);
    gpio_set_direction(EN1, GPIO_MODE_OUTPUT);
    gpio_set_direction(EN2, GPIO_MODE_OUTPUT);
    gpio_set_direction(nRESET, GPIO_MODE_OUTPUT);
    gpio_set_direction(nSLEEP, GPIO_MODE_OUTPUT);
    gpio_set_level(nRESET, 1);
    gpio_set_level(nSLEEP, 1);
    gpio_set_level(EN1, 1);
    gpio_set_level(EN2, 1);
    // ets_printf("STIMTask setup complete");
    ESP_LOGI("NO TAG","STIMTask setup complete");
    int btn_num=0;
    while(1)
    {
        if(isDisp_setup)
        {
                // ets_printf("Driver on, strength set : %d \n", STIMStrength[freq_list_index]);
                // ESP_LOGI("NO TAG ","Driver on, strength set : %d \n", STIMStrength[freq_list_index]);
                printf("Simulation strength: %d\n",STIMStrength[freq_list_index]);

                for(int i= 0;i<20;i++)
                {
        
                    gpio_set_level(IN1, 1);
                    gpio_set_level(IN2, 0);
                    ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, STIMStrength[freq_list_index]));
                    ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL));
                    vTaskDelay(10 / portTICK_PERIOD_MS);
                    gpio_set_level(IN1, 0);
                    gpio_set_level(IN2, 1);             
                    vTaskDelay(10 / portTICK_PERIOD_MS);
                    ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, 0));
                    ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL));
                    gpio_set_level(IN1, 0);
                    gpio_set_level(IN2, 0);
                    vTaskDelay(12 / portTICK_PERIOD_MS);
                    
    
                }
                
    
                //gpio_set_level(IN1, 1);
                //gpio_set_level(IN2, 0);
    
    
               // ets_printf("Stimulation fired : driver off for 2 sec\n");
                ESP_LOGI("NO TAG","Stimulation fired : driver off for 2 sec");
                // ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, 0));
                // ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL));
                vTaskDelay(2000 / portTICK_PERIOD_MS);

        }
        if(isDisp_setup==0){
            ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, 0));
            ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL));

        }


        if(isDisp_walk)
        {
                // gpio_set_level(IN1, 0);
                // gpio_set_level(IN2, 0);
                // ESP_LOGI("NO TAG","Stimulation fired : driver off for 2 sec");
                // ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, 0));
                // ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL));

        if(xQueueReceive(FIRE_ONQUEUE,&btn_num,portMAX_DELAY))
        {
                printf("Stimualtion on\n");
                ESP_LOGI("NO TAG ","Driver on, strength set : %d \n", STIMStrength[freq_list_index]);
                for(int i= 0;i<20;i++)
                {

                    gpio_set_level(IN1, 1);
                    gpio_set_level(IN2, 0);
                    ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL,STIMStrength[freq_list_index]));
                    ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL));
                    vTaskDelay(10/ portTICK_PERIOD_MS);
                    gpio_set_level(IN1, 0);
                    gpio_set_level(IN2, 1);
                    vTaskDelay(10/ portTICK_PERIOD_MS);
        
                    ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, 0));
                    ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL));
     
                    gpio_set_level(IN1, 0);
                    gpio_set_level(IN2, 0);
                    vTaskDelay(12.5/ portTICK_PERIOD_MS);
                    
                    // ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, STIMStrength[1]));
                    // ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL));

            }

            if(xQueueReceive(FIRE_OFFQUEUE,&btn_num,portMAX_DELAY))
                {
                    gpio_set_level(IN1, 0);
                    gpio_set_level(IN2, 0);
                    ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, 0));
                    ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL));
                    printf("Stimulation off\n");
                    xQueueReset(FIRE_OFFQUEUE);

                }

            xQueueReset(FIRE_ONQUEUE);
            
        }
                

        }
    vTaskDelay(20 / portTICK_PERIOD_MS);    
    }
}





void detect_gait(void *params)
{
   float prev_value = 0.0;
	float curr_value = 0.0;
    float fire_angle = 0;
    

    while (1) {
        // mpu6050_read_data(&accel_x, &accel_y, &accel_z, &gyro_x, &gyro_y, &gyro_z);
        if(pos_count==1)
        {
          angle_roll = atan2(accel_z, sqrt(accel_x * accel_x + accel_y * accel_y)) * 180.0 / M_PI;
        }
        else if(pos_count==2)
        {
            angle_roll = atan2(accel_y, sqrt(accel_x * accel_x + accel_z * accel_z)) * 180.0 / M_PI;
        }
        prev_value = curr_value;
		curr_value = angle_roll;
        if(curr_value < THRESH_LOW)
		{
			if(trough_detected(curr_value, prev_value) && t_detect_flag && !fire_on && angle_roll<-25)
			{
				//stim on
                below_neg_15 = true; 
				t_detect_flag = false;
                fire_angle = prev_value;
                // printf("Previous Value: %.2f   Fire angle%.2f\n",prev_value,fire_angle);

			}
		}


        if (below_neg_15 && angle_roll > fire_angle && !fire_on) {
            // ssd1306_display_text_x3(&dev, 0, "Fire", 5, false);        
            printf("-----Fire ON----\n");  
            xQueueSend(FIRE_ONQUEUE,&angle_roll,NULL);
            fire_on = true;
            below_neg_15 = false; // Reset monitoring flag after firing starts
            esp_timer_start_once(fire_timer, 750000); // 1 second in microseconds
        }

        if (angle_roll > (fire_angle+60.0) && fire_on) {
            printf("-----Fire OFF----\n");
            xQueueSend(FIRE_OFFQUEUE,&angle_roll,NULL);
            // ssd1306_clear_screen(&dev, false);
            fire_on = false;
            t_detect_flag = true;
            esp_timer_stop(fire_timer);
        }

        vTaskDelay(100 / portTICK_PERIOD_MS); // Delay for 100 milliseconds
    }
}



void disp_walk(void *params)
{
    ssd1306_clear_screen(&dev, false);
    int count = 6;
    uint8_t segs[128];


    while (1) {
        if(animation_running==1)
        {
           TickType_t startTick = xTaskGetTickCount();

  
        for (int page = 0; page < 8; page++) {
            for (int seg = 0; seg < 128; seg++) {
                segs[seg] = ssd1306_rotate_byte(walk[count][seg * 8 + page]);
            }
            ssd1306_display_image(&dev, page, 0, segs, 128);
        }

        TickType_t endTick = xTaskGetTickCount();
        ESP_LOGD(TAG, "diffTick=%"PRIu32, endTick - startTick);

        count--;
        if (count < 0) count = 6;

        vTaskDelay(6/portTICK_PERIOD_MS);
        }

        vTaskDelay(100/portTICK_PERIOD_MS);
        
    }



        

    
}


void BTN_UPTask(void *param)
{
    gpio_set_direction(BTN_UP, GPIO_MODE_INPUT);
    gpio_set_intr_type(BTN_UP, GPIO_INTR_NEGEDGE);
    int BTN_NUMBER = 0;
    while (1)
    {
        if (xQueueReceive(BTN_UPQueue, &BTN_NUMBER, portMAX_DELAY))
        {
            printf("Button up pressed\n");
            if(isDisp_menu)
            {
                printf("disp menu 1 true\n");
                // isDisp_menu1=true;
                // isDisp_menu2=false;
                if(Disp_menu_count>1){
                    Disp_menu_count--;
                }
                disp_menu();

            }
            if(isDisp_setup)
            {
                
                if(freq_list_index<8)
                {
                    freq_list_index++;
                }
                disp_setup();
            }

            if(isDisp_pos)
            {
                if(pos_count>1){
                    pos_count--;
                    if(pos_count==1)
                        {
                            ssd1306_display_text(&dev, 3, " F-Front       ", 16, true);
                            ssd1306_display_text(&dev, 5, " S-Side        ", 16, false);
    
                        }
                    if(pos_count==2)
                        {
                            ssd1306_display_text(&dev, 3, " F-Front       ", 16, false);
                            ssd1306_display_text(&dev, 5, " S-Side        ", 16, true);
                        }
                }
            }
            
            xQueueReset(BTN_UPQueue);
        }
    }
}


void BTN_DOWNTask(void *params)
{
    gpio_set_direction(BTN_DOWN, GPIO_MODE_INPUT);
    gpio_set_intr_type(BTN_DOWN, GPIO_INTR_NEGEDGE);
    int BTN_NUMBER = 0;
    while (1)
    {

        if (xQueueReceive(BTN_DOWNQueue, &BTN_NUMBER, portMAX_DELAY))
        {
            printf("Down_button_works\n");
            if(isDisp_menu)
            {
                // isDisp_menu1=false;
                // isDisp_menu2=true;
                if(Disp_menu_count<3){
                    Disp_menu_count++;
                }
                disp_menu();
            }
            if(isDisp_setup)
            {
                
                if(freq_list_index>0)
                {
                    freq_list_index--;
                }
                disp_setup();
            }

            if(isDisp_pos)
            {
                if(pos_count<2){
                    pos_count++;
                    if(pos_count==1)
                    {
                        ssd1306_display_text(&dev, 3, " F-Front       ", 16, true);
                        ssd1306_display_text(&dev, 5, " S-Side        ", 16, false);

                    }
                if(pos_count==2)
                    {
                        ssd1306_display_text(&dev, 3, " F-Front       ", 16, false);
                        ssd1306_display_text(&dev, 5, " S-Side        ", 16, true);
                    }

                }

            }

            
            xQueueReset(BTN_DOWNQueue);
        }
    }
}

void BTN_PWRTask(void *params)
{
    gpio_set_direction(BTN_OK, GPIO_MODE_INPUT);
    gpio_set_intr_type(BTN_OK, GPIO_INTR_NEGEDGE);
    int BTN_NUMBER = 0;
    while (1)
    {
        if (xQueueReceive(BTN_PWRQueue, &BTN_NUMBER, portMAX_DELAY))
        {
            ESP_LOGI(TAG, "Task BTN_PWRTask: Button power pressed (15)!");
            long_press_detected = false;

            while (gpio_get_level(BTN_PWR) == 0 && !long_press_detected)
            {
                curr_time = esp_timer_get_time();

                if (curr_time - intr_time >= 1000000) // Check for long press duration
                {
                    ESP_LOGI("NO TAG", "Long Press Detected");
                    long_press_detected = true; // Set long press flag
                    enter_deep_sleep(); // Enter deep sleep on long press
                }

                if (gpio_get_level(BTN_PWR) == 1)
                {
                    if (curr_time - intr_time < 1000000)
                    {
                        ESP_LOGI("NO TAG", "Short Press Detected");
                        long_press_detected = true;
                    }
                }
            }

            xQueueReset(BTN_PWRQueue);
        }
    }
            

}

void BTN_OKTask(void *params)
{
    gpio_set_direction(BTN_PWR, GPIO_MODE_INPUT);
    gpio_set_intr_type(BTN_PWR, GPIO_INTR_NEGEDGE);
    int BTN_NUMBER = 0;
    while (1)
    {
        if (xQueueReceive(BTN_OKQueue, &BTN_NUMBER, portMAX_DELAY))
        {
            printf("Ok button pressed\n");
            if(isDisp_menu)
            {
                if(Disp_menu_count==1)
                {
                    printf("disp setup true\n");
                    isDisp_setup=true;
                    isDisp_menu = false;
                    ssd1306_clear_screen(&dev, false);
                    ssd1306_bitmaps(&dev, 0, 0, setup_1, 128, 64, false);
                    ssd1306_bitmaps(&dev, 106, 35,str_num[0], 16,9,false);
                    disp_setup();
                } 
                if(Disp_menu_count==3)
                {
                    printf("disp walk true\n");
                    isDisp_walk=true;
                    isDisp_menu=false;
                    isDisp_setup=false;
                    animation_running = true;
                    ssd1306_clear_screen(&dev, false);
                    ssd1306_display_text_x3(&dev, 0, " Walk", 6, false);
                    printf("%d\n",animation_running);
                }

                if(Disp_menu_count==2)
                {
                    ssd1306_clear_screen(&dev, false);
                    isDisp_pos = true;
                    isDisp_walk=false;
                    isDisp_menu=false;
                    isDisp_setup=false;
                    if(pos_count==1)
                    {
                        ssd1306_display_text(&dev, 3, " F-Front       ", 16, true);
                        ssd1306_display_text(&dev, 5, " S-Side        ", 16, false);
                    }
                    if(pos_count==2)
                    {
                        ssd1306_display_text(&dev, 3, " F-Front       ", 16, false);
                        ssd1306_display_text(&dev, 5, " S-Side        ", 16, true);
                    }
                    
                }
            }

            else if(isDisp_setup)
            {

                ssd1306_clear_screen(&dev, false);
                disp_menu();
            }
            else if(isDisp_walk)
            {
                isDisp_walk=false;
                isDisp_setup=false;
                isDisp_menu=true;
                animation_running = false;
                ssd1306_clear_screen(&dev, false);
                disp_menu();
            }
            
            else if(isDisp_pos)
            {
                isDisp_pos = false;
                ssd1306_clear_screen(&dev, false);
                disp_menu();
            }

            xQueueReset(BTN_OKQueue);

        }
        vTaskDelay(10/portTICK_PERIOD_MS);
    }
}

void get_soc(void *params)
{
while(1)
{
    soc = max17260_read_soc();
    // if (soc >= 0) 
    // {
    //     ESP_LOGI(TAG, "State of Charge: %.2f%%", soc);

    // }
    vTaskDelay(1000/portTICK_PERIOD_MS);

}
}

static void IRAM_ATTR BTN_UP_interrupt_handler(void *args)
{
    
    int pinNumber = (int)args;
    if(esp_timer_get_time() - pre_time_up > 500000)
    {
        xQueueSendFromISR(BTN_UPQueue, &pinNumber, NULL);

    }
    pre_time_up= esp_timer_get_time();
}

static void IRAM_ATTR BTN_DOWN_interrupt_handler(void *args)
{
    int pinNumber = (int)args;
    if(esp_timer_get_time() - pre_time_down > 500000)
    {
        xQueueSendFromISR(BTN_DOWNQueue, &pinNumber, NULL);
    }
    pre_time_down = esp_timer_get_time();
}

static void IRAM_ATTR BTN_PWR_interrupt_handler(void *args)
{
    int pinNumber = (int)args;
    if(esp_timer_get_time() - pre_time_pwr > 500000){
    xQueueSendFromISR(BTN_PWRQueue, &pinNumber, NULL);
    }
    pre_time_pwr = esp_timer_get_time();
}

static void IRAM_ATTR BTN_OK_interrupt_handler(void *args)
{
    int pinNumber = (int)args;
    if(esp_timer_get_time() - pre_time_ok > 500000){
    xQueueSendFromISR(BTN_OKQueue, &pinNumber, NULL);
    intr_time = esp_timer_get_time();
    }
    pre_time_ok = esp_timer_get_time();
}

void app_main(void)
{
    i2c_master_init(&dev, CONFIG_SDA_GPIO, CONFIG_SCL_GPIO, CONFIG_RESET_GPIO);
    read_who_i_am(MPU6050_WHO_I_AM);
    mpu6050_init(MPU6050_PWR_MGMT_1);
    // display_logo();
    // disp_menu();
    BTN_UPQueue = xQueueCreate(10, sizeof(int));
    BTN_DOWNQueue = xQueueCreate(10, sizeof(int));
    BTN_PWRQueue = xQueueCreate(10, sizeof(int));
    BTN_OKQueue = xQueueCreate(10, sizeof(int));
    FIRE_ONQUEUE = xQueueCreate(10, sizeof(int));
    FIRE_OFFQUEUE = xQueueCreate(10, sizeof(int));

    if (BTN_UPQueue == NULL || BTN_DOWNQueue == NULL || BTN_PWRQueue == NULL || BTN_OKQueue == NULL)
    {
        ESP_LOGE(TAG, "Failed to create button queues");
        return;
    }

    float soc = max17260_read_soc();
    if (soc >= 0) 
    {
        ESP_LOGI(TAG, "State of Charge: %.2f%%", soc);
    }
    const esp_timer_create_args_t fire_timer_args = {
        .callback = &fire_off_callback,
        .name = "fire_timer"
    };

    if (esp_timer_create(&fire_timer_args, &fire_timer) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create fire timer");
        return;
    }

    gpio_install_isr_service(0);
    gpio_isr_handler_add(BTN_UP, BTN_UP_interrupt_handler, (void *)BTN_UP);
    gpio_isr_handler_add(BTN_DOWN, BTN_DOWN_interrupt_handler, (void *)BTN_DOWN);
    gpio_isr_handler_add(BTN_PWR, BTN_PWR_interrupt_handler, (void *)BTN_PWR);
    gpio_isr_handler_add(BTN_OK, BTN_OK_interrupt_handler, (void *)BTN_OK);

    xTaskCreate(BTN_UPTask, "BTN_UPTask", 2048, NULL, 1, NULL);
    xTaskCreate(BTN_DOWNTask, "BTN_DOWNTask", 2048, NULL, 1, NULL);
    xTaskCreate(BTN_PWRTask, "BTN_PWRTask", 2048, NULL, 1, NULL);
    xTaskCreate(BTN_OKTask, "BTN_OKTask", 8000, NULL, 1, NULL);
    xTaskCreate(mpu6050_get_data, "Mpu6050",8000, NULL, 1, NULL);
    xTaskCreate(detect_gait, "detect_gait",8192, NULL, 1, NULL);
    xTaskCreate(get_soc, "get soc",8000, NULL, 1, NULL);
    // xTaskCreate(disp_walk, "display walk",8000, NULL, 1, NULL);
    xTaskCreate(STIMTask, "STIMTask", 8000, NULL, 1, NULL);

    display_logo();
    disp_menu();
}
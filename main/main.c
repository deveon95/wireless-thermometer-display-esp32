#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_err.h"
#include "nvs_flash.h"
#include "esp_tls.h" 
#include "file_serving_example_common.h"

#include "driver/gpio.h"
#include "hal/gpio_ll.h"
#include "driver/timer.h"
#include "time.h"
#include "driver/uart.h"
#include "driver/adc.h"

#define SR_DELAY_US 1
#define NUM_OF_ANODES 16
#define ANODES_IN_USE 0b0011111100111111

#define pin_segAR 14
#define pin_segBR 21
#define pin_segCR 47
#define pin_segDR 48
#define pin_segER 35
#define pin_segFR 36
#define pin_segGR 37
#define pin_segDPR 38
#define pin_segAL 4
#define pin_segBL 5
#define pin_segCL 6
#define pin_segDL 7
#define pin_segEL 17
#define pin_segFL 18
#define pin_segGL 8
#define pin_segDPL 13

#define pin_SOE 9
#define pin_SLAT 10
#define pin_SDAT 11
#define pin_SCK 12

#define pin_BUTTON2 0
#define pin_BUTTON3 2
#define pin_BUTTON4 3

#define TIMER_DIVIDER         (16)  //  Hardware timer clock divider
#define TIMER_SCALE           ((ets_get_apb_freq() / 10000)/ TIMER_DIVIDER)  // convert counter value to seconds

typedef struct {
    int timer_group;
    int timer_idx;
    int alarm_interval;
    bool auto_reload;
} timer_info_t;


// brightness of the display (0 to 3)
uint8_t display_brightness = 3;

#define NUMBER_OF_BRIGHTNESS_SETTINGS 4
#define BRIGHTNESS_HYSTERESIS 100
#define ADC_FILTER_LENGTH 30
#define ADC_MAX_VALUE 4095
#define ADC_HYSTERESIS 200
static const adc_channel_t channel = ADC_CHANNEL_0;     //GPIO1 for ADC1
static const adc_bits_width_t width = ADC_WIDTH_BIT_12;
static const adc_atten_t atten = ADC_ATTEN_DB_11;

double temperatures[CONFIG_NUMBER_OF_TEMPERATURES];
// Mask 
uint32_t temperaturesReceived;

/* This example demonstrates how to create file server
 * using esp_http_server. This file has only startup code.
 * Look in file_server.c for the implementation.
 */

// Setup UART buffered IO with event queue
const uart_port_t uart_num = UART_NUM_2;
const int uart_buffer_size = (8 * 32);  // Must be greater than 128
QueueHandle_t uart_queue;
#define PATTERN_CHR_NUM    (3)         /*!< Set the number of consecutive and identical characters received by receiver which defines a UART pattern*/

static const char *TAG = "example";

/* FreeRTOS event group to signal when we are connected*/
static EventGroupHandle_t s_wifi_event_group;
/* The event group allows multiple bits for each event, but we only care about two events:
 * - we are connected to the AP with an IP
 * - we failed to connect after the maximum amount of retries */
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT BIT1



void get_display_digits(double value, uint8_t * display_digits, uint32_t * dp_pos, bool enabled)
{
    if (enabled)
    {
        int32_t value_int = (int32_t)(value * 100.0);
        if (value_int >= 100000)        // >= 1000.00 out of range
        {
            display_digits[0] = 0x0A;
            display_digits[1] = 1;
            display_digits[2] = 0x0A;
            *dp_pos = 4;     // Decimal point in 3rd position
        }
        else if (value_int >= 10000)         // 100.00 - 999.99
        {
            display_digits[0] = value_int / 10000ul % 10;
            display_digits[1] = value_int / 1000ul % 10;
            display_digits[2] = value_int / 100ul % 10;
            *dp_pos = 4;     // Decimal point in 3rd position
        }
        else if (value_int >= 0)     // 10.00 - 99.99
        {
            display_digits[0] = value_int >= 1000 ? value_int / 1000ul % 10 : 0x0A;
            display_digits[1] = value_int / 100ul % 10;
            display_digits[2] = value_int / 10ul % 10;
            *dp_pos = 2;     // Decimal point in 2nd position
        }
        /* Commented out this section to restrict display to one decimal place
        else if (value_int >= 0)        // 0.00 - 9.99
        {
            display_digits[0] = value_int / 100ul % 10;
            display_digits[1] = value_int / 10ul % 10;
            display_digits[2] = value_int % 10;
            *dp_pos = 1;     // Decimal point in 1st position
        }*/
        else if (value_int > -1000)     // -9.90 to -0.10
        {
            display_digits[0] = 0x0B;
            display_digits[1] = (value_int * -1) / 100ul % 10;
            display_digits[2] = (value_int * -1) / 10ul % 10;
            *dp_pos = 2;     // Decimal point in 2nd position
        }
        else if (value_int > -10000)    // -99.9 to -10
        {
            display_digits[0] = 0x0B;
            display_digits[1] = (value_int * -1) / 1000ul % 10;
            display_digits[2] = (value_int * -1) / 100ul % 10;
            *dp_pos = 4;     // Decimal point in 3rd position
        }
        else                                // Out of range (negative)
        {
            display_digits[0] = 0x0B;
            display_digits[1] = 1;
            display_digits[2] = 0x0A;
            *dp_pos = 0;     // Decimal point off
        }
    }
    else
    {
        display_digits[0] = 0x0A;
        display_digits[1] = 0x0A;
        display_digits[2] = 0x0A;
        *dp_pos = 0;     // Decimal point off
    }
}

// Callback for Timer Interrupt - displays the next digit on the 7-segment display on each run
static bool IRAM_ATTR timer_group_isr_callback(void *args)
{
    static uint8_t display_digits[NUM_OF_ANODES * 2] = {0,1,2,3,4,5,0,0,8,9,0,1,2,3,0,0,6,7,8,9,0,1,0,0,4,5,6,7,8,9,0,0};
    static uint8_t display_segments[NUM_OF_ANODES * 2];
    static uint32_t decimal_points = 0;
    const uint8_t segment_patterns[12] = {0b00111111, 0b00000110, 0b01011011, 0b01001111, 0b01100110, 0b01101101, 0b01111100, 0b00000111, 0b01111111, 0b01100111, 0b00000000, 0b01000000};
    static uint8_t current_disp_index = 0;
    static uint8_t dim_cycle_counter = 0;
    uint32_t dp_temp;
    //bool button2_held = !gpio_get_level(pin_BUTTON2);
    //bool button3_held = !gpio_get_level(pin_BUTTON3);
    
    // If first digit is about to be displayed, update display data
    if ((current_disp_index == 0) && (dim_cycle_counter == 0))
    {
        decimal_points = 0;
        // Right hand displays
        get_display_digits(temperatures[1], &display_digits[0], &dp_temp, temperaturesReceived & 0x01); decimal_points |= dp_temp;
        get_display_digits(temperatures[3], &display_digits[3], &dp_temp, temperaturesReceived & 0x02); decimal_points |= dp_temp << 3;
        get_display_digits(temperatures[5], &display_digits[8], &dp_temp, temperaturesReceived & 0x04); decimal_points |= dp_temp << 8;
        get_display_digits(temperatures[7], &display_digits[11], &dp_temp, temperaturesReceived & 0x08); decimal_points |= dp_temp << 11;
        
        // Left hand display
        get_display_digits(temperatures[0], &display_digits[16], &dp_temp, temperaturesReceived & 0x10); decimal_points |= dp_temp << 16;
        get_display_digits(temperatures[2], &display_digits[19], &dp_temp, temperaturesReceived & 0x20); decimal_points |= dp_temp << 19;
        get_display_digits(temperatures[4], &display_digits[24], &dp_temp, temperaturesReceived & 0x40); decimal_points |= dp_temp << 24;
        get_display_digits(temperatures[6], &display_digits[27], &dp_temp, temperaturesReceived & 0x80); decimal_points |= dp_temp << 27;
                
        
        // Get required segment patterns
        for (uint8_t i = 0; i < NUM_OF_ANODES * 2; i++)
        {
            display_segments[i] = segment_patterns[display_digits[i]];
        }
    }
    
    // Turn off all digits
    // SPI code needed here
    gpio_set_level(pin_SOE, 1);
    
    // Turn off all segments by setting specified bits high in 'Write 1 to set' register
    GPIO.out_w1ts =      ((uint32_t)1 << pin_segAL)
                       | ((uint32_t)1 << pin_segBL)
                       | ((uint32_t)1 << pin_segCL)
                       | ((uint32_t)1 << pin_segDL)
                       | ((uint32_t)1 << pin_segEL)
                       | ((uint32_t)1 << pin_segFL)
                       | ((uint32_t)1 << pin_segGL)
                       | ((uint32_t)1 << pin_segDPL)
                       | ((uint32_t)1 << pin_segAR)
                       | ((uint32_t)1 << pin_segBR);
    GPIO.out1_w1ts.val = 
                         ((uint32_t)1 << (pin_segCR - 32))
                       | ((uint32_t)1 << (pin_segDR - 32))
                       | ((uint32_t)1 << (pin_segER - 32))
                       | ((uint32_t)1 << (pin_segFR - 32))
                       | ((uint32_t)1 << (pin_segGR - 32))
                       | ((uint32_t)1 << (pin_segDPR - 32));
    
    if (dim_cycle_counter >= (NUMBER_OF_BRIGHTNESS_SETTINGS - 1 - display_brightness))
    {
        // Turn on required digit
        // SPI code needed here
        for (uint8_t i = 0; i < NUM_OF_ANODES; i++)
        {
            gpio_set_level(pin_SDAT, (NUM_OF_ANODES - 1 - current_disp_index) == i ? 0 : 1);
            esp_rom_delay_us(SR_DELAY_US);
            gpio_set_level(pin_SCK, 1);
            esp_rom_delay_us(SR_DELAY_US);
            gpio_set_level(pin_SCK, 0);
        }
        
        esp_rom_delay_us(SR_DELAY_US);
        gpio_set_level(pin_SLAT, 1);
        esp_rom_delay_us(SR_DELAY_US);
        gpio_set_level(pin_SLAT, 0);
        esp_rom_delay_us(SR_DELAY_US);
        gpio_set_level(pin_SOE, 0);
        
        // Turn on required segments
        GPIO.out_w1tc =
                             ((((display_segments[current_disp_index] & 0x01) >> 0)) << pin_segAL )
                           | ((((display_segments[current_disp_index] & 0x02) >> 1)) << pin_segBL )
                           | ((((display_segments[current_disp_index] & 0x04) >> 2)) << pin_segCL )
                           | ((((display_segments[current_disp_index] & 0x08) >> 3)) << pin_segDL )
                           | ((((display_segments[current_disp_index] & 0x10) >> 4)) << pin_segEL )
                           | ((((display_segments[current_disp_index] & 0x20) >> 5)) << pin_segFL )
                           | ((((display_segments[current_disp_index] & 0x40) >> 6)) << pin_segGL )
                           | ((((decimal_points >> current_disp_index & 0x01) >> 0)) << pin_segDPL)
                           | ((((display_segments[current_disp_index + NUM_OF_ANODES] & 0x01) >> 0)) << pin_segAR )
                           | ((((display_segments[current_disp_index + NUM_OF_ANODES] & 0x02) >> 1)) << pin_segBR )
                           ;
        GPIO.out1_w1tc.val = 
                             ((((display_segments[current_disp_index + NUM_OF_ANODES] & 0x04) >> 2)) << (pin_segCR - 32) )
                           | ((((display_segments[current_disp_index + NUM_OF_ANODES] & 0x08) >> 3)) << (pin_segDR - 32) )
                           | ((((display_segments[current_disp_index + NUM_OF_ANODES] & 0x10) >> 4)) << (pin_segER - 32) )
                           | ((((display_segments[current_disp_index + NUM_OF_ANODES] & 0x20) >> 5)) << (pin_segFR - 32) )
                           | ((((display_segments[current_disp_index + NUM_OF_ANODES] & 0x40) >> 6)) << (pin_segGR - 32) )
                           | ((((decimal_points >> (current_disp_index + NUM_OF_ANODES) & 0x01) >> 0)) << (pin_segDPR - 32) )
                           ;
    }
    
    // There are NUMBER_OF_BRIGHTNESS_SETTINGS iterations of dim_cycle_counter where the display is turned off or on
    // according to the value of display_brightness to change the brightness of the digit. After all iterations,
    // current_disp_index (the digit counter) is incremented. 
    if (dim_cycle_counter < (NUMBER_OF_BRIGHTNESS_SETTINGS - 1))
    {
        dim_cycle_counter++;
    }
    else
    {
        dim_cycle_counter = 0;
        do
        {
            if (current_disp_index < NUM_OF_ANODES - 1)
            {
                current_disp_index++;
            }
            else
            {
                current_disp_index = 0;
            }
        }
        while ((ANODES_IN_USE >> current_disp_index) == 0);
    }
    
    
    
    return true; // return whether we need to yield at the end of ISR
}
/**
 * @brief Initialize selected timer of timer group
 *
 * @param group Timer Group number, index from 0
 * @param timer timer ID, index from 0
 * @param auto_reload whether auto-reload on alarm event
 * @param timer_interval_tenthmsec interval of alarm
 */
static void example_timer_init(int group, int timer, bool auto_reload, int timer_interval_tenthmsec)
{
    /* Select and initialize basic parameters of the timer */
    timer_config_t config = {
        .divider = TIMER_DIVIDER,
        .counter_dir = TIMER_COUNT_UP,
        .counter_en = TIMER_PAUSE,
        .alarm_en = TIMER_ALARM_EN,
        .auto_reload = auto_reload,
    }; // default clock source is APB
    timer_init(group, timer, &config);

    /* Timer's counter will initially start from value below.
       Also, if auto_reload is set, this value will be automatically reload on alarm */
    timer_set_counter_value(group, timer, 0);

    /* Configure the alarm value and the interrupt on alarm. */
    timer_set_alarm_value(group, timer, timer_interval_tenthmsec * TIMER_SCALE);
    timer_enable_intr(group, timer);

    timer_info_t *timer_info = calloc(1, sizeof(timer_info_t));
    timer_info->timer_group = group;
    timer_info->timer_idx = timer;
    timer_info->auto_reload = auto_reload;
    timer_info->alarm_interval = timer_interval_tenthmsec;
    timer_isr_callback_add(group, timer, timer_group_isr_callback, timer_info, 0);

    timer_start(group, timer);
}
// Display task - sets up the Timer Interrupt on whichever core display_task is pinned to
// Timer interrupt used for updating the display to allow for faster refresh rate
void display_task(void * pvParameters)
{
    ESP_LOGI(TAG, "starting display_task on core %d", xPortGetCoreID());
    
    // Configure timer in this task to guarantee that correct core is used for ISR
    example_timer_init(TIMER_GROUP_0, TIMER_0, true, 2);
    
    while(1)
    {
        vTaskDelay(10000 / portTICK_PERIOD_MS);
    }
}

/* Read brightness sensor and update LED brightness
 * The photodiode is connected between the ADC input pin (K) and GND (A)
 * A 10k resistor is connected between the input pin and +3V3
 */
void get_light_level_task(void * pvParameters)
{
    adc1_config_width(width);
    adc1_config_channel_atten(channel, atten);
    uint16_t adc_filter[ADC_FILTER_LENGTH];
    uint32_t adc_average = 0;
    uint16_t adc_reading;
    
    while(1)
    {
        // 50ms sampling interval
        vTaskDelay(50 / portTICK_PERIOD_MS);
        
        // Move entries in the filter along
        for (uint8_t i = 0; i < ADC_FILTER_LENGTH - 1; i++)
        {
            adc_filter[i] = adc_filter[i+1];
        }
        // Get ADC reading and add to filter (invert so that higher value means brighter)
        adc_reading = adc1_get_raw((adc1_channel_t)channel);
        /*
        // Multiply by 2
        adc_reading = (adc_reading << 2);
        // Expand the upper half of the range and discard the lower half
        if (adc_reading > ((ADC_MAX_VALUE + 1) * 3))
        {
            adc_reading -= ((ADC_MAX_VALUE + 1) * 3);
        }
        else
        {
            adc_reading = 0;
        }
        */
        // Add to filter
        adc_filter[ADC_FILTER_LENGTH - 1] = ADC_MAX_VALUE - adc_reading;
        
        // Calculate average from the filter
        adc_average = 0;
        for (uint8_t i = 0; i < ADC_FILTER_LENGTH; i++)
        {
            adc_average += (uint32_t)adc_filter[i];
        }
        adc_average /= (uint32_t)ADC_FILTER_LENGTH;
        
        // Calculate LED brightness with hysteresis
        int adc_range_per_level = ADC_MAX_VALUE / NUMBER_OF_BRIGHTNESS_SETTINGS;
        int upper_limit = display_brightness * adc_range_per_level + adc_range_per_level + BRIGHTNESS_HYSTERESIS;
        int lower_limit = display_brightness * adc_range_per_level - BRIGHTNESS_HYSTERESIS;
        
        if ((int)adc_average >= upper_limit) {
            display_brightness = (display_brightness + 1) % NUMBER_OF_BRIGHTNESS_SETTINGS;
        } else if ((int)adc_average <= lower_limit) {
            display_brightness = (display_brightness + NUMBER_OF_BRIGHTNESS_SETTINGS - 1) % NUMBER_OF_BRIGHTNESS_SETTINGS;
        }
        
        // Print ADC reading
        //ESP_LOGI(TAG_ADC, "ADC value %d Brightness %d", adc_average, display_brightness);
    }
}

// static bool IRAM_ATTR uart_intr_handle(void *args)
// {
    // return true;
// }

static void uart_event_task(void *pvParameters)
{
    uart_event_t event;
    size_t buffered_size;
    uint8_t* dtmp = (uint8_t*) malloc(uart_buffer_size);
    for (;;) {
        //Waiting for UART event.
        if (xQueueReceive(uart_queue, (void *)&event, (TickType_t)portMAX_DELAY)) {
            bzero(dtmp, uart_buffer_size);
            ESP_LOGI(TAG, "uart[%d] event:", uart_num);
            switch (event.type) {
            //Event of UART receiving data
            /*We'd better handler data event fast, there would be much more data events than
            other types of events. If we take too much time on data event, the queue might
            be full.*/
            case UART_DATA:
                ESP_LOGI(TAG, "[UART DATA]: %d", event.size);
                uart_read_bytes(uart_num, dtmp, event.size, portMAX_DELAY);
                ESP_LOGI(TAG, "[DATA EVT]:");
                uart_write_bytes(uart_num, (const char*) dtmp, event.size);
                // Check packet for temperature data
                if (event.size == 8)
                {
                    // Validation: Second byte is 'C', checksum is correct, and tenths byte is less than 10
                    if (dtmp[1] == 'C' && dtmp[7] == (dtmp[2] + dtmp[3] + dtmp[4] + dtmp[5] + dtmp[6]) && dtmp[3] < 10 && dtmp[0] < CONFIG_NUMBER_OF_TEMPERATURES)
                    {
                        ESP_LOGI(TAG, "Temperature packet received: ID %d Temperature %d.%d", dtmp[0], dtmp[2], dtmp[3]);
                        bool sign = dtmp[2] >> 7;
                        // Convert magnitude to floating point
                        temperatures[dtmp[0]] = (dtmp[2] & 0x7f) + (dtmp[3] * 0.1);
                        // apply sign
                        temperatures[dtmp[0]] = temperatures[dtmp[0]] * (sign ? (-1.0) : 1.0);
                        
                        temperaturesReceived |= (1 << dtmp[0]);
                    }
                }
                break;
            //Event of HW FIFO overflow detected
            case UART_FIFO_OVF:
                ESP_LOGI(TAG, "hw fifo overflow");
                // If fifo overflow happened, you should consider adding flow control for your application.
                // The ISR has already reset the rx FIFO,
                // As an example, we directly flush the rx buffer here in order to read more data.
                uart_flush_input(uart_num);
                xQueueReset(uart_queue);
                break;
            //Event of UART ring buffer full
            case UART_BUFFER_FULL:
                ESP_LOGI(TAG, "ring buffer full");
                // If buffer full happened, you should consider increasing your buffer size
                // As an example, we directly flush the rx buffer here in order to read more data.
                uart_flush_input(uart_num);
                xQueueReset(uart_queue);
                break;
            //Event of UART RX break detected
            case UART_BREAK:
                ESP_LOGI(TAG, "uart rx break");
                break;
            //Event of UART parity check error
            case UART_PARITY_ERR:
                ESP_LOGI(TAG, "uart parity error");
                break;
            //Event of UART frame error
            case UART_FRAME_ERR:
                ESP_LOGI(TAG, "uart frame error");
                break;
            //UART_PATTERN_DET
            case UART_PATTERN_DET:
                uart_get_buffered_data_len(uart_num, &buffered_size);
                int pos = uart_pattern_pop_pos(uart_num);
                ESP_LOGI(TAG, "[UART PATTERN DETECTED] pos: %d, buffered size: %d", pos, buffered_size);
                if (pos == -1) {
                    // There used to be a UART_PATTERN_DET event, but the pattern position queue is full so that it can not
                    // record the position. We should set a larger queue size.
                    // As an example, we directly flush the rx buffer here.
                    uart_flush_input(uart_num);
                } else {
                    uart_read_bytes(uart_num, dtmp, pos, 100 / portTICK_PERIOD_MS);
                    uint8_t pat[PATTERN_CHR_NUM + 1];
                    memset(pat, 0, sizeof(pat));
                    uart_read_bytes(uart_num, pat, PATTERN_CHR_NUM, 100 / portTICK_PERIOD_MS);
                    ESP_LOGI(TAG, "read data: %s", dtmp);
                    ESP_LOGI(TAG, "read pat : %s", pat);
                }
                break;
            //Others
            default:
                ESP_LOGI(TAG, "uart event type: %d", event.type);
                break;
            }
        }
    }
    free(dtmp);
    dtmp = NULL;
    vTaskDelete(NULL);
}


// WiFi event handler
static void event_handler(void* arg, esp_event_base_t event_base,
																int32_t event_id, void* event_data)
{
    static int s_retry_num = 0;
	if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
		esp_wifi_connect();
	} else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
		if (s_retry_num < CONFIG_ESP_MAXIMUM_RETRY) {
				esp_wifi_connect();
				s_retry_num++;
				ESP_LOGI(TAG, "retry to connect to the AP");
		} else {
				xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
		}
		ESP_LOGI(TAG,"connect to the AP fail");
        //wifi_connected = false;
	} else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
		ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
		ESP_LOGI(TAG, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
		s_retry_num = 0;
		xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
        //wifi_connected = true;
	}
}

// WiFi initialisation
void wifi_init_sta(void)
{
	s_wifi_event_group = xEventGroupCreate();

	ESP_ERROR_CHECK(esp_netif_init());

	ESP_ERROR_CHECK(esp_event_loop_create_default());
	esp_netif_create_default_wifi_sta();

	wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
	ESP_ERROR_CHECK(esp_wifi_init(&cfg));

	esp_event_handler_instance_t instance_any_id;
	esp_event_handler_instance_t instance_got_ip;
	ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
									ESP_EVENT_ANY_ID,
									&event_handler,
									NULL,
									&instance_any_id));
	ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
									IP_EVENT_STA_GOT_IP,
									&event_handler,
									NULL,
									&instance_got_ip));

	wifi_config_t wifi_config = {
		.sta = {
			.ssid = CONFIG_ESP_WIFI_SSID,
			.password = CONFIG_ESP_WIFI_PASSWORD,
			/* Setting a password implies station will connect to all security modes including WEP/WPA.
			 * However these modes are deprecated and not advisable to be used. Incase your Access point
			 * doesn't support WPA2, these mode can be enabled by commenting below line */
			.threshold.authmode = WIFI_AUTH_WPA2_PSK,

			.pmf_cfg = {
				.capable = true,
				.required = false
			},
		},
	};
	ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA) );
	ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config) );
	ESP_ERROR_CHECK(esp_wifi_start() );

	ESP_LOGI(TAG, "wifi_init_sta finished.");

	/* Waiting until either the connection is established (WIFI_CONNECTED_BIT) or connection failed for the maximum
	 * number of re-tries (WIFI_FAIL_BIT). The bits are set by event_handler() (see above) */
	EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
		WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
		pdFALSE,
		pdFALSE,
		portMAX_DELAY);

	/* xEventGroupWaitBits() returns the bits before the call returned, hence we can test which event actually
	 * happened. */
	if (bits & WIFI_CONNECTED_BIT) {
		ESP_LOGI(TAG, "connected to ap SSID:%s password:%s", CONFIG_ESP_WIFI_SSID, CONFIG_ESP_WIFI_PASSWORD);
	} else if (bits & WIFI_FAIL_BIT) {
		ESP_LOGI(TAG, "Failed to connect to SSID:%s, password:%s", CONFIG_ESP_WIFI_SSID, CONFIG_ESP_WIFI_PASSWORD);
	} else {
		ESP_LOGE(TAG, "UNEXPECTED EVENT");
	}

	/* The event will not be processed after unregister */
	ESP_ERROR_CHECK(esp_event_handler_instance_unregister(IP_EVENT, IP_EVENT_STA_GOT_IP, instance_got_ip));
	ESP_ERROR_CHECK(esp_event_handler_instance_unregister(WIFI_EVENT, ESP_EVENT_ANY_ID, instance_any_id));
	vEventGroupDelete(s_wifi_event_group);
}


void app_main(void)
{
    ESP_LOGI(TAG, "Starting ESP32 Wireless Temperature Display");
	//Initialize NVS
	esp_err_t ret = nvs_flash_init();
	if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
		ESP_ERROR_CHECK(nvs_flash_erase());
		ret = nvs_flash_init();
	}
	ESP_ERROR_CHECK(ret);
    
    // Set up GPIO
    
    gpio_config_t io_conf;
    io_conf.intr_type = (gpio_int_type_t)GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = ((uint64_t)1 << pin_segAL)
                         | ((uint64_t)1 << pin_segBL)
                         | ((uint64_t)1 << pin_segCL)
                         | ((uint64_t)1 << pin_segDL)
                         | ((uint64_t)1 << pin_segEL)
                         | ((uint64_t)1 << pin_segFL)
                         | ((uint64_t)1 << pin_segGL)
                         | ((uint64_t)1 << pin_segDPL)
                         | ((uint64_t)1 << pin_segAR)
                         | ((uint64_t)1 << pin_segBR)
                         | ((uint64_t)1 << pin_segCR)
                         | ((uint64_t)1 << pin_segDR)
                         | ((uint64_t)1 << pin_segER)
                         | ((uint64_t)1 << pin_segFR)
                         | ((uint64_t)1 << pin_segGR)
                         | ((uint64_t)1 << pin_segDPR)
                         | ((uint64_t)1 << pin_SLAT)
                         | ((uint64_t)1 << pin_SOE)
                         | ((uint64_t)1 << pin_SDAT)
                         | ((uint64_t)1 << pin_SCK);
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    gpio_config(&io_conf);
    
    uart_config_t uart_config = {
        .baud_rate = 2400,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    };
    // Install UART driver using an event queue here
    ESP_ERROR_CHECK(uart_driver_install(uart_num, uart_buffer_size, \
                                            uart_buffer_size, 10, &uart_queue, 0));
    // Configure UART parameters
    ESP_ERROR_CHECK(uart_param_config(uart_num, &uart_config));
    ESP_ERROR_CHECK(uart_set_line_inverse(uart_num, UART_SIGNAL_RXD_INV | UART_SIGNAL_TXD_INV));
    ESP_ERROR_CHECK(uart_set_pin(uart_num, 43, 44, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    //Set uart pattern detect function.
    uart_enable_pattern_det_baud_intr(uart_num, '+', PATTERN_CHR_NUM, 9, 0, 0);
    //Reset the pattern queue length to record at most 20 pattern positions.
    uart_pattern_queue_reset(uart_num, 20);
    // set up UART interrupt
    // uart_intr_config_t uart_intr_config_params = {
        // .intr_enable_mask = UART_INTR_RXFIFO_FULL,
        // .rx_timeout_thresh = 4,
        // .rxfifo_full_thresh = 8,
    // };
    // ESP_ERROR_CHECK(uart_intr_config(uart_num, &uart_intr_config_params));
    // ESP_ERROR_CHECK(uart_enable_rx_intr(uart_num);
    // uart_isr_free(uart_num);
    // esp_err = uart_isr_register(uart_num,uart_intr_handle,NULL,0,NULL);

    /* Initialize file storage */
    const char* base_path = "/data";
    ESP_ERROR_CHECK(example_mount_storage(base_path));
    
    TaskHandle_t displayHandle;
    TaskHandle_t getLightLevelHandle;
    
    xTaskCreatePinnedToCore(display_task, "display_task", 2048, NULL, configMAX_PRIORITIES - 2, &displayHandle, 1);
    
    xTaskCreate(uart_event_task, "uart_event_task", 3072, NULL, 12, NULL);
    
    xTaskCreatePinnedToCore(get_light_level_task, "get_light_level_task", 2048, NULL, configMAX_PRIORITIES - 4, &getLightLevelHandle, 1);

    /* This helper function configures Wi-Fi or Ethernet, as selected in menuconfig.
     * Read "Establishing Wi-Fi or Ethernet Connection" section in
     * examples/protocols/README.md for more information about this function.
     */
    //ESP_ERROR_CHECK(example_connect());
    wifi_init_sta();

    /* Start the file server */
    ESP_ERROR_CHECK(example_start_file_server(base_path));
    ESP_LOGI(TAG, "File server started");
    
    
}

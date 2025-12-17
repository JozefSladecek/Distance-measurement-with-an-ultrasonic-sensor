/**
 * @file main.c
 * @brief Ultrasonic Distance Meter with ESP32
 * @author Jozef Sládeček (xsladej00)
 * @date 2025
 *
 * This program implements an ultrasonic distance meter using HY-SRF05 sensor
 * and 4-digit 7-segment LED display with TLC5916 driver.
 * Features: 3 display modes, brightness control, measurement history, freeze function.
 */

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "esp_task_wdt.h"

#include "driver/ledc.h"  // PWM


// === SENSOR PIN DEFINITIONS ===
#define TRIG_PIN GPIO_NUM_26
#define ECHO_PIN GPIO_NUM_25

#define SPEED_OF_SOUND 0.0343
#define TIMEOUT_US 30000


// === DISPLAY PIN DEFINITIONS ===
// SPI pins for display
#define MOSI_PIN GPIO_NUM_23
#define SCLK_PIN GPIO_NUM_18
#define XLAT_PIN GPIO_NUM_17
#define BLANK_PIN GPIO_NUM_13

// Digit select pins
#define DIG1_PIN GPIO_NUM_14
#define DIG2_PIN GPIO_NUM_27
#define DIG3_PIN GPIO_NUM_16
#define DIG4_PIN GPIO_NUM_12

// BUTONS select pins
#define SW1_PIN GPIO_NUM_4
#define SW2_PIN GPIO_NUM_5
#define SW3_PIN GPIO_NUM_33
#define SW4_PIN GPIO_NUM_32


#define DIGIT_BLANK 0b00000000

#define DIGIT_0 0b10101111
#define DIGIT_1 0b00101000
#define DIGIT_2 0b10011011
#define DIGIT_3 0b10111010
#define DIGIT_4 0b00111100
#define DIGIT_5 0b10110110
#define DIGIT_6 0b10110111
#define DIGIT_7 0b00101010
#define DIGIT_8 0b10111111
#define DIGIT_9 0b00111110

#define DIGIT_DP 0b01000000

#define BRIGHTNESS_HIGH   0     // Duty 0%      → 100% brightness
#define BRIGHTNESS_MEDIUM 128   // Duty 50%   → 100% brightness
#define BRIGHTNESS_LOW    200   // Duty 78%   → 100% brightness
#define BRIGHTNESS_MIN    230   // Duty 90%   → 100% brightness

//pwm
#define LEDC_TIMER              LEDC_TIMER_0
#define LEDC_MODE               LEDC_LOW_SPEED_MODE
#define LEDC_CHANNEL            LEDC_CHANNEL_0
#define LEDC_DUTY_RES           LEDC_TIMER_8_BIT  // 8-bit = 0-255
#define LEDC_FREQUENCY          (5000)             // 5000 Hz = 5000 cyklov/sekundu


typedef enum {
    MODE_STATIC,      // XX.X
    MODE_ADAPTIVE,
    MODE_INTEGER      // XXX
} display_mode_t;

typedef enum {
    BRIGHTNESS_LVL_HIGH,
    BRIGHTNESS_LVL_MEDIUM,
    BRIGHTNESS_LVL_LOW,
    BRIGHTNESS_LVL_MIN
} brightness_level_t;

// === GLOBAL VARIABLES (SHARED) ===

// Button state flags (set by ISR, processed by button_task)
static volatile bool mode_change_requested = false;
static volatile bool brightness_change_requested = false;
static volatile bool freeze_toggle_requested = false;

// Display settings
display_mode_t current_mode = MODE_STATIC;
brightness_level_t current_brightness = BRIGHTNESS_LVL_HIGH;

// Freeze functionality
bool is_frozen = false;              // is display frozen?
float frozen_distance = 0.0;

// Measurement history
#define HISTORY_SIZE 4
float measurement_history[HISTORY_SIZE] = {0.0, 0.0, 0.0, 0.0};
int history_view_index = -1;  // -1 = normal view, 0..3 = history view

static volatile float current_distance = 0.0;

// SW4 long/short press detection
int64_t sw4_press_time = 0;  // long press normal mode
bool sw4_was_pressed = false;

static const char *TAG = "ULTRASONIC_METER";

// === ISR HANDLERS ===

static void IRAM_ATTR button_isr_handler(void* arg) {
   // sw1
    mode_change_requested = true;
}
static void IRAM_ATTR button_sw2_isr_handler(void* arg) {
    // sw2
    brightness_change_requested = true;
}

static void IRAM_ATTR button_sw3_isr_handler(void* arg) {
    // sw3
    freeze_toggle_requested = true;
}


// === HARDWARE INIT FUNCTIONS ===

/**
 * @brief Initialize ultrasonic sensor pins
 */
void ultrasonic_init(void) {
    gpio_set_direction(TRIG_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(TRIG_PIN, 0);
    gpio_set_direction(ECHO_PIN, GPIO_MODE_INPUT);
}


/**
 * @brief Initialize PWM for display brightness control
 * Note: TLC5916 BLANK pin has inverted logic (HIGH = OFF)
 */
void pwm_init(void) {
    // set timer
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_MODE,
        .timer_num        = LEDC_TIMER,
        .duty_resolution  = LEDC_DUTY_RES,
        .freq_hz          = LEDC_FREQUENCY,
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ledc_timer_config(&ledc_timer);

    // add timer to channel
    ledc_channel_config_t ledc_channel = {
        .speed_mode     = LEDC_MODE,
        .channel        = LEDC_CHANNEL,
        .timer_sel      = LEDC_TIMER,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = BLANK_PIN,
        .duty           = BRIGHTNESS_HIGH,  // full brightness
        .hpoint         = 0
    };
    ledc_channel_config(&ledc_channel);
}

/**
 * @brief Initialize 7-segment display pins and PWM
 */
void display_init(void) {
    // MOSI as output
    gpio_set_direction(MOSI_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(MOSI_PIN, 0);

    // SCLK as output
    gpio_set_direction(SCLK_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(SCLK_PIN, 0);

    // XLAT as output
    gpio_set_direction(XLAT_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(XLAT_PIN, 0);

    // Digit select pins as output
    gpio_set_direction(DIG1_PIN, GPIO_MODE_OUTPUT);
    gpio_set_direction(DIG2_PIN, GPIO_MODE_OUTPUT);
    gpio_set_direction(DIG3_PIN, GPIO_MODE_OUTPUT);
    gpio_set_direction(DIG4_PIN, GPIO_MODE_OUTPUT);


    // All digits OFF initially
    gpio_set_level(DIG1_PIN, 1);
    gpio_set_level(DIG2_PIN, 1);
    gpio_set_level(DIG3_PIN, 1);
    gpio_set_level(DIG4_PIN, 1);

    pwm_init();

    ESP_LOGI(TAG, "Display initialized");
}

/**
 * @brief Initialize button pins with interrupts
 * SW1-SW3 use interrupts, SW4 uses polling for long/short press detection
 */
void button_init(void) {
    // config
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << SW1_PIN) | (1ULL << SW2_PIN) | (1ULL << SW3_PIN) | (1ULL << SW4_PIN),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_NEGEDGE  // Interupt on falling edge (1→0)
    };
    gpio_config(&io_conf);

    gpio_install_isr_service(0);

    // add handlers
    gpio_isr_handler_add(SW1_PIN, button_isr_handler, NULL);
    gpio_isr_handler_add(SW2_PIN, button_sw2_isr_handler, NULL);
    gpio_isr_handler_add(SW3_PIN, button_sw3_isr_handler, NULL);


    ESP_LOGI(TAG, "Buttons initialized");
}

// === HELPER FUNCTIONS ===

/**
 * @brief Switch to next display mode (cyclic)
 * STATIC → ADAPTIVE → INTEGER → STATIC
 */
void switch_mode(void) {
    current_mode = (current_mode + 1) % 3;

    const char* mode_names[] = {"STATIC", "ADAPTIVE", "INTEGER"};
    ESP_LOGI(TAG, "Switched to mode: %s", mode_names[current_mode]);
}

/**
 * @brief Save measurement to history (circular buffer)
 * @param value Distance value in centimeters
 */
void save_to_history(float value) {
    // move values one position right
    for (int i = HISTORY_SIZE - 1; i > 0; i--) {
        measurement_history[i] = measurement_history[i - 1];
    }

    // new value to the beginning
    measurement_history[0] = value;

    ESP_LOGI(TAG, "Saved to history: %.2f cm", value);
}

/**
 * @brief Set display brightness level
 * @param level Brightness level (HIGH/MEDIUM/LOW/MIN)
 */
void set_brightness(brightness_level_t level) {
    uint32_t duty;

    switch (level) {
        case BRIGHTNESS_LVL_HIGH:
            duty = BRIGHTNESS_HIGH;
            ESP_LOGI(TAG, "Brightness: HIGH (100%%)");
            break;
        case BRIGHTNESS_LVL_MEDIUM:
            duty = BRIGHTNESS_MEDIUM;
            ESP_LOGI(TAG, "Brightness: MEDIUM (50%%)");
            break;
        case BRIGHTNESS_LVL_LOW:
            duty = BRIGHTNESS_LOW;
            ESP_LOGI(TAG, "Brightness: LOW (22%%)");
            break;
        case BRIGHTNESS_LVL_MIN:
            duty = BRIGHTNESS_MIN;
            ESP_LOGI(TAG, "Brightness: MIN (10%%)");
            break;
        default:
            duty = BRIGHTNESS_HIGH;
    }

    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, duty);
    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);

    current_brightness = level;
}

/**
 * @brief Switch to next brightness level (cyclic)
 */
void switch_brightness(void) {
    ESP_LOGI(TAG, "switch_brightness() called, current level: %d", current_brightness);
    current_brightness = (current_brightness + 1) % 4;
    set_brightness(current_brightness);
}

/**
 * @brief Check if button is currently pressed
 * @param pin GPIO pin number
 * @return true if pressed (LOW), false otherwise
 */
bool is_button_pressed(gpio_num_t pin) {
    return gpio_get_level(pin) == 0;  // Active LOW
}

/**
 * @brief Send one byte to display via SPI
 * @param byte 8-bit segment pattern
 */
void send_byte(uint8_t byte) {
    for (int i = 7; i >= 0; i--) {
        // Set MOSI to bit value
        gpio_set_level(MOSI_PIN, (byte >> i) & 0x01);

        gpio_set_level(SCLK_PIN, 1);
        esp_rom_delay_us(1);
        gpio_set_level(SCLK_PIN, 0);
        esp_rom_delay_us(1);
    }
}

/**
 * @brief Latch data to display output register
 */
void latch_data(void) {
    gpio_set_level(XLAT_PIN, 1);
    esp_rom_delay_us(1);
    gpio_set_level(XLAT_PIN, 0);
}

/**
 * @brief Display one digit with multiplexing
 * @param segment Digit position (0-3)
 * @param digit_to_show 8-bit segment pattern
 */
void show_digit(int segment, uint8_t digit_to_show) {
    gpio_set_level(DIG1_PIN, 1);
    gpio_set_level(DIG2_PIN, 1);
    gpio_set_level(DIG3_PIN, 1);
    gpio_set_level(DIG4_PIN, 1);
    send_byte(digit_to_show);
    latch_data();

    switch (segment) {
        case 0: gpio_set_level(DIG1_PIN, 0); break;
        case 1: gpio_set_level(DIG2_PIN, 0); break;
        case 2: gpio_set_level(DIG3_PIN, 0); break;
        case 3: gpio_set_level(DIG4_PIN, 0); break;
    }
}

/**
 * @brief Measure distance using ultrasonic sensor HY-SRF05
 * @return Distance in centimeters, or -1.0 on error/timeout
 *
 * Sends 10us trigger pulse, measures ECHO pulse duration.
 * Uses timeout of 30ms (~5m max distance).
 */
float measure_distance(void) {
    // Send 10us trigger pulse
    gpio_set_level(TRIG_PIN, 0);
    esp_rom_delay_us(2); // stabilization
    gpio_set_level(TRIG_PIN, 1);
    esp_rom_delay_us(10);
    gpio_set_level(TRIG_PIN, 0);

    // Wait for ECHO HIGH (0->1)
    int64_t start_wait = esp_timer_get_time();
    while (gpio_get_level(ECHO_PIN) == 0) {
        if ((esp_timer_get_time() - start_wait) > TIMEOUT_US) {
            return -1; // timeout
        }
    }

    // Measure pulse duration (1->0)
    int64_t pulse_start = esp_timer_get_time();
    while (gpio_get_level(ECHO_PIN) == 1) {
        if ((esp_timer_get_time() - pulse_start) > TIMEOUT_US) {
            return -1.0;
        }
    }
    int64_t pulse_end = esp_timer_get_time();

    float duration_us = (float)(pulse_end - pulse_start);
    float distance_cm = (duration_us * SPEED_OF_SOUND) / 2.0;

    return distance_cm;

}

/**
 * @brief Extract individual digits from integer value
 * @param val Integer value to extract from
 * @param digits Output array for digit patterns
 * @param patterns Array of digit patterns (DIGIT_0 to DIGIT_9)
 * @param start_pos Starting position in output array
 * @param count Number of digits to extract
 *
 * Example: val=1234, start_pos=0, count=4 → digits[0..3] = patterns[1,2,3,4]
 */
void extract_digits(int val, uint8_t *digits, const uint8_t *patterns,
                   int start_pos, int count) {
    int divisors[] = {1000, 100, 10, 1};

    for (int i = 0; i < count; i++) {
        digits[start_pos + i] = patterns[(val / divisors[4 - count + i]) % 10];
    }
}

/**
 * @brief Display distance on 7-segment display with selected mode
 * @param distance_cm Distance in centimeters
 *
 * Supports three display modes:
 * - STATIC: Fixed decimal point (XX.X)
 * - ADAPTIVE: Adjustable decimal point (X.XX / XX.XX / XXX.X / XXXX)
 * - INTEGER: No decimal point (whole centimeters)
 * Displays "----" for invalid/out-of-range values.
 */
void display_distance(float distance_cm) {
    // Digit patterns for 0-9
    uint8_t patterns[10] = {
        DIGIT_0, DIGIT_1, DIGIT_2, DIGIT_3, DIGIT_4,
        DIGIT_5, DIGIT_6, DIGIT_7, DIGIT_8, DIGIT_9
    };

    // Error handling - display "----" for invalid values
    if (distance_cm < 0) {
        for (int i = 0; i < 4; i++) {
            show_digit(i, 0b00010000);  // Display minus sign (segment G)
            esp_rom_delay_us(2000);
        }
        return;
    }

    // Initialize digit buffer and decimal point position
    uint8_t digits[4] = {DIGIT_BLANK, DIGIT_BLANK, DIGIT_BLANK, DIGIT_BLANK};
    int dp_position = -1;  // -1 = no decimal point

    // Select display format based on current mode
    switch (current_mode) {
        case MODE_STATIC:
        {
            // Always align right with decimal at position 2
            int val = (int)(distance_cm * 10);

            if (val >= 1000) {  // >= 100.0 cm → "XXX.X" (123.4)
                extract_digits(val, digits, patterns, 0, 4);
            }
            else if (val >= 100) {  // >= 10.0 cm → " XX.X" (25.3)
                extract_digits(val, digits, patterns, 1, 3);
            }
            else {  // < 10.0 cm → "  X.X" (5.3)
                extract_digits(val, digits, patterns, 2, 2);
            }

            dp_position = 2;  // Always at position 2 in static mode!
        }
            break;

        case MODE_ADAPTIVE:
            {
                // Adaptive decimal point (changes position based on value)
                if (distance_cm >= 1000.0) {  // "XXXX" (1234)
                    int val = (int)distance_cm;
                    extract_digits(val, digits, patterns, 0, 4);
                    dp_position = -1;  // No decimal point
                }
                else if (distance_cm >= 100.0) {  // "XXX.X" (123.4)
                    int val = (int)(distance_cm * 10);
                    extract_digits(val, digits, patterns, 0, 4);
                    dp_position = 2;  // Decimal after 3rd digit
                }
                else {  // "XX.XX" (25.34) or " X.XX" (5.34)
                    int val = (int)(distance_cm * 100);

                    if (val >= 1000) {  // >= 10.00 → "XX.XX"
                        extract_digits(val, digits, patterns, 0, 4);
                    } else {  // < 10.00 → " X.XX"
                        extract_digits(val, digits, patterns, 1, 3);
                    }
                    dp_position = 1;  // Decimal after 2nd digit
                }
            }
            break;

        case MODE_INTEGER:
            {
                // Integer mode (no decimal point)
                int val = (int)distance_cm;

                if (val >= 1000) {  // "XXXX" (1234)
                    extract_digits(val, digits, patterns, 0, 4);
                }
                else if (val >= 100) {  // " XXX" (123)
                    extract_digits(val, digits, patterns, 1, 3);
                }
                else if (val >= 10) {  // "  XX" (25)
                    extract_digits(val, digits, patterns, 2, 2);
                }
                else {  // "   X" (5)
                    extract_digits(val, digits, patterns, 3, 1);
                }

                dp_position = -1;  // No decimal point
            }
            break;
    }

    // Display all 4 digits with multiplexing
    for (int i = 0; i < 4; i++) {
        uint8_t pattern = digits[i];

        // Add decimal point if needed
        if (i == dp_position) {
            pattern |= DIGIT_DP;
        }

        // Show digit and wait for persistance
        show_digit(i, pattern);
        esp_rom_delay_us(2000);  // 2ms per digit = 125hz

    }
}

// === TASK FUNCTIONS ===

/**
 * @brief Display task - continuously refreshes 7-segment display
 * @param pvParameters Unused
 *
 * Displays current/frozen/history value based on state.
 * Refresh rate: ~125Hz (8ms per full cycle).
 */
void display_task(void *pvParameters) {
    ESP_LOGI(TAG, "Display task started");

    while (true) {
        float display_value;

        // Determine what to display
        if (history_view_index == -1) {
            display_value = is_frozen ? frozen_distance : current_distance;
        } else {
            display_value = measurement_history[history_view_index];
        }

        // Refresh display
        display_distance(display_value);

        // Small delay to prevent watchdog issues
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

/**
 * @brief Measurement task - periodically measures distance
 * @param pvParameters Unused
 *
 * Measures distance every 200ms  and updates current_distance.
 * Validates measurement range (0-400cm).
 */
void measurement_task(void *pvParameters) {
    ESP_LOGI(TAG, "Measurement task started");

    int64_t last_measurement = 0;
    const int64_t MEASURE_INTERVAL_US = 200000;  // 200ms

    while (true) {
        int64_t now = esp_timer_get_time();
        // Measure only one in 200ms
        if ((now - last_measurement) >= MEASURE_INTERVAL_US) {
            last_measurement = now;

            float new_distance = measure_distance();

            if (new_distance >= 0 && new_distance < 400) {
                current_distance = new_distance;  //  Update shared variable
                ESP_LOGI(TAG, "Distance: %.2f cm", current_distance);
            } else {
                ESP_LOGW(TAG, "Out of range or error");
                current_distance = -1.0;
            }
        }

        vTaskDelay(pdMS_TO_TICKS(10));  // check every 10ms
    }
}



/**
 * @brief Button task - handles button presses with debouncing
 * @param pvParameters Unused
 *
 * Processes button flags set by ISR handlers.
 * SW1: Mode change, SW2: Brightness, SW3: Freeze, SW4: History (polling).
 * Debounce time: 200ms.
 */
void button_task(void *pvParameters) {
    ESP_LOGI(TAG, "Button task started");

    int64_t last_mode_change = 0;
    int64_t last_brightness_change = 0;
    int64_t last_freeze_toggle = 0;
    const int64_t DEBOUNCE_TIME_US = 200000;  // 200ms

    while (true) {
        int64_t now = esp_timer_get_time();

        // SW1 - Mode change
        if (mode_change_requested) {
            if ((now - last_mode_change) > DEBOUNCE_TIME_US) {
                switch_mode();
                last_mode_change = now;
                ESP_LOGI(TAG, "Button sw1 pressed! Mode changed.");
            }
            mode_change_requested = false;
        }

        // SW2 - Brightness
        if (brightness_change_requested) {
            if ((now - last_brightness_change) > DEBOUNCE_TIME_US) {
                switch_brightness();
                last_brightness_change = now;
            }
            brightness_change_requested = false;
        }

        // SW3 - Freeze toggle
        if (freeze_toggle_requested) {
            if ((now - last_freeze_toggle) > DEBOUNCE_TIME_US) {
                if (history_view_index == -1) {
                    is_frozen = !is_frozen;
                    if (is_frozen) {
                        frozen_distance = current_distance;
                        save_to_history(current_distance);
                        ESP_LOGI(TAG, "FROZEN at %.2f cm", frozen_distance);
                    } else {
                        ESP_LOGI(TAG, "UNFROZEN");
                    }
                    last_freeze_toggle = now;
                } else {
                    // history mode → ignore freeze
                    ESP_LOGI(TAG, "Cannot freeze in history view");
                }
            }
            freeze_toggle_requested = false;
        }

        // SW4 - History view (long/short press) - pooling

        bool sw4_currently_pressed = is_button_pressed(SW4_PIN);

        if (sw4_currently_pressed && !sw4_was_pressed) {
            sw4_press_time = esp_timer_get_time();
        }

        if (!sw4_currently_pressed && sw4_was_pressed) {
            int64_t press_duration = esp_timer_get_time() - sw4_press_time;

            if (press_duration > 1000000) {  // Long press (>1s)
                if (history_view_index == -1) {
                    history_view_index = 0;
                    ESP_LOGI(TAG, "ENTERING history [0]");
                } else {
                    history_view_index = -1;
                    ESP_LOGI(TAG, "EXITING history");
                }
            } else {  // Short press
                if (history_view_index != -1) {
                    // cycle history only if in history view
                    history_view_index = (history_view_index + 1) % HISTORY_SIZE;
                    ESP_LOGI(TAG, "History [%d]", history_view_index);
                }
            }
        }

        sw4_was_pressed = sw4_currently_pressed;

        vTaskDelay(pdMS_TO_TICKS(20));  // check every 20ms
    }
}


/**
 * @brief Main function - initializes hardware and creates FreeRTOS tasks
 */
void app_main(void) {
    ESP_LOGI(TAG, "=== Ultrasonic Distance Meter (Multi-Task) ===");
    esp_task_wdt_deinit();

    ultrasonic_init();
    display_init();
    button_init();

    // all task has priority 1
    xTaskCreate(display_task, "display", 4096, NULL, 1, NULL);
    xTaskCreate(measurement_task, "measurement", 4096, NULL, 1, NULL);
    xTaskCreate(button_task, "buttons", 4096, NULL, 1, NULL);

    ESP_LOGI(TAG, "All tasks created successfully");

    // Main task can end other task can run
}

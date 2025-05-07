#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "esp_err.h"
#include "driver/i2c_master.h"
#include "sdkconfig.h"

#include "freertos/event_groups.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "lwip/err.h"
#include "lwip/sys.h"
#include "esp_sntp.h"
#include "time.h"
#include <bme680.h>
#include "user_data.h"

#define LGFX_USE_V1
#include <LovyanGFX.hpp>

// Display dimension definitions
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64

#define MAX_RETRY      5

// I2C pins
#define I2C_SDA_PIN 8
#define I2C_SCL_PIN 9

// The event group allows multiple bits for each event, but we only care about two events:
// - we are connected to the AP with an IP
// - we failed to connect after the maximum amount of retries
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1

// FreeRTOS event group to signal when we are connected
static EventGroupHandle_t s_wifi_event_group;

static const char *TAG = "cyberclock";
static int s_retry_num = 0;

// Configure the display panel for SSD1309
class LGFX_SSD1309 : public lgfx::LGFX_Device {
public:
    lgfx::Panel_SSD1306 _panel_instance;
    lgfx::Bus_I2C _bus_instance;

    LGFX_SSD1309() {
        { // Configure bus control
            auto cfg = _bus_instance.config();
            cfg.i2c_port = I2C_NUM_0;        // Use I2C port 0
            cfg.freq_write = 400000;         // Set the clock to 400kHz
            cfg.pin_sda = I2C_SDA_PIN;       // SDA pin
            cfg.pin_scl = I2C_SCL_PIN;       // SCL pin
            cfg.i2c_addr = 0x3C;             // Default I2C address for SSD1309 is usually 0x3C
            _bus_instance.config(cfg);
            _panel_instance.setBus(&_bus_instance);
        }

        { // Configure display panel control
            auto cfg = _panel_instance.config();
            cfg.panel_width = SCREEN_WIDTH;    // Display width
            cfg.panel_height = SCREEN_HEIGHT;  // Display height
            cfg.offset_x = 0;                  // Offset in X direction
            cfg.offset_y = 0;                  // Offset in Y direction
            cfg.offset_rotation = 0;           // Rotation offset
            cfg.readable = false;              // Whether it can be read
            cfg.invert = false;                // Whether to invert the display
            cfg.rgb_order = false;             // Not a color display
            cfg.dlen_16bit = false;            // Not a 16-bit interface
            cfg.bus_shared = false;            // Bus is not shared with other devices
            _panel_instance.config(cfg);
        }

        setPanel(&_panel_instance);
    }
};

LGFX_SSD1309 display;

// WiFi event handler
static void event_handler(void* arg, esp_event_base_t event_base,
                          int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (s_retry_num < MAX_RETRY) {
            esp_wifi_connect();
            s_retry_num++;
            ESP_LOGI(TAG, "Retry to connect to the AP");
        } else {
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
        }
        ESP_LOGI(TAG,"Connect to the AP failed");
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "Got IP:" IPSTR, IP2STR(&event->ip_info.ip));
        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

// Initialize WiFi as station
void wifi_init_sta(void)
{
    s_wifi_event_group = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &event_handler, NULL));

    wifi_config_t wifi_config = {0};
    strcpy((char*)wifi_config.sta.ssid, WIFI_SSID);
    strcpy((char*)wifi_config.sta.password, WIFI_PASS);
    wifi_config.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "wifi_init_sta finished.");

    /* Waiting until either the connection is established (WIFI_CONNECTED_BIT) or connection failed for the maximum
     * number of re-tries (WIFI_FAIL_BIT). The bits are set by event_handler() */
    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
            WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
            pdFALSE,
            pdFALSE,
            portMAX_DELAY);

    /* xEventGroupWaitBits() returns the bits before the call returned, hence we can test which event actually
     * happened. */
    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG, "Connected to AP SSID:%s", WIFI_SSID);
    } else if (bits & WIFI_FAIL_BIT) {
        ESP_LOGI(TAG, "Failed to connect to SSID:%s", WIFI_SSID);
    } else {
        ESP_LOGE(TAG, "UNEXPECTED EVENT");
    }
}

// Initialize time sync
void initialize_sntp(void)
{
    ESP_LOGI(TAG, "Initializing SNTP");
    esp_sntp_setoperatingmode(SNTP_OPMODE_POLL);
    esp_sntp_setservername(0, "pool.ntp.org");
    esp_sntp_init();
}

char time_buffer_current[64];

// Print current time
void print_time(void)
{
    time_t now;
    struct tm timeinfo;
    
    time(&now);
    localtime_r(&now, &timeinfo);
    
    char time_buffer[64];
    strftime(time_buffer, sizeof(time_buffer), "%H:%M", &timeinfo);

    if (strcmp(time_buffer, time_buffer_current) == 0) {
      return;
    }

    strcpy(time_buffer_current, time_buffer);
    
    ESP_LOGI(TAG, "Current time: %s", time_buffer);

    // Update counter part
    // display.fillRect(50, 40, 78, 8, 0); // Clear the area for counter
    // List of built in fonts https://m5stack.lang-ship.com/howto/m5gfx/font/
    display.clear();
    display.setFont(&fonts::Orbitron_Light_32);
    display.setCursor(20, 10);
    display.printf(time_buffer);
}

// Time sync notification callback
void time_sync_notification_cb(struct timeval *tv)
{
    ESP_LOGI(TAG, "Time synchronized from NTP server!");
    print_time();
}

// Set timezone to your local timezone
void set_timezone(void)
{
    setenv("TZ", TIMEZONE, 1);
    tzset();
}

extern "C" void app_main_old(void)
{
    ESP_LOGI(TAG, "Initializing display...");
    
    // Initialize the display
    display.init();

    display.setRotation(2);
    
    // Set text color and size
    display.setTextColor(TFT_WHITE);
    display.setTextSize(1);
    
    // Clear the display
    display.clear();
    
    // Show greeting text
    display.setCursor(20, 20);
    display.println("Starting up...");
    // display.println("128x64 OLED Display");
    // display.println("");
    // display.println("Hello World!");
    // display.println("");

    time_buffer_current[0] = '\0';
    
    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_LOGI(TAG, "ESP32-C3 WiFi Time Sync Example");

    // Initialize WiFi
    wifi_init_sta();

    // Wait for WiFi connection
    vTaskDelay(2000 / portTICK_PERIOD_MS);

    // Set timezone
    set_timezone();

    // Initialize SNTP
    initialize_sntp();

    // Set time synchronization notification callback
    esp_sntp_set_time_sync_notification_cb(time_sync_notification_cb);

    // Wait for time to be set
    time_t now = 0;
    struct tm timeinfo;
    memset(&timeinfo, 0, sizeof(struct tm));
    int retry = 0;
    const int retry_count = 10;
    
    while (timeinfo.tm_year < (2020 - 1900) && ++retry < retry_count) {
        ESP_LOGI(TAG, "Waiting for system time to be set... (%d/%d)", retry, retry_count);
        vTaskDelay(2000 / portTICK_PERIOD_MS);
        time(&now);
        localtime_r(&now, &timeinfo);
    }
    
    // Main program loop
    while (1) {
        // Display everything
        display.display();

        print_time();

        // Wait 100ms
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}

static inline void print_registers(bme680_handle_t handle) {
    /* configuration registers */
    bme680_control_measurement_register_t ctrl_meas_reg;
    bme680_control_humidity_register_t    ctrl_humi_reg;
    bme680_config_register_t              config_reg;
    bme680_control_gas0_register_t        ctrl_gas0_reg;
    bme680_control_gas1_register_t        ctrl_gas1_reg;

    /* attempt to read control humidity register */
    bme680_get_control_humidity_register(handle, &ctrl_humi_reg);

    /* attempt to read control measurement register */
    bme680_get_control_measurement_register(handle, &ctrl_meas_reg);

    /* attempt to read configuration register */
    bme680_get_configuration_register(handle, &config_reg);

    /* attempt to read control gas 0 register */
    bme680_get_control_gas0_register(handle, &ctrl_gas0_reg);

    /* attempt to read control gas 1 register */
    bme680_get_control_gas1_register(handle, &ctrl_gas1_reg);

    ESP_LOGI(TAG, "Variant Id          (0x%02x): %s", handle->variant_id,uint8_to_binary(handle->variant_id));
    ESP_LOGI(TAG, "Configuration       (0x%02x): %s", config_reg.reg,    uint8_to_binary(config_reg.reg));
    ESP_LOGI(TAG, "Control Measurement (0x%02x): %s", ctrl_meas_reg.reg, uint8_to_binary(ctrl_meas_reg.reg));
    ESP_LOGI(TAG, "Control Humidity    (0x%02x): %s", ctrl_humi_reg.reg, uint8_to_binary(ctrl_humi_reg.reg));
    ESP_LOGI(TAG, "Control Gas 0       (0x%02x): %s", ctrl_gas0_reg.reg, uint8_to_binary(ctrl_gas0_reg.reg));
    ESP_LOGI(TAG, "Control Gas 1       (0x%02x): %s", ctrl_gas1_reg.reg, uint8_to_binary(ctrl_gas1_reg.reg));
}

void i2c0_bme680_task(i2c_master_bus_handle_t i2c0_bus_hdl) {
    // initialize the xLastWakeTime variable with the current time.
    TickType_t          last_wake_time  = xTaskGetTickCount ();
    //
    // initialize i2c device configuration
    bme680_config_t dev_cfg         = I2C_BME680_CONFIG_DEFAULT;
    bme680_handle_t dev_hdl;
    //
    // init device
    bme680_init(i2c0_bus_hdl, &dev_cfg, &dev_hdl);
    if (dev_hdl == NULL) {
        ESP_LOGE(TAG, "bme680 handle init failed");
        assert(dev_hdl);
    }
    
    print_registers(dev_hdl);

    // task loop entry point
    for ( ;; ) {
        ESP_LOGI(TAG, "######################## BME680 - START #########################");
        //
        // handle sensor

        bme680_data_t data;
        esp_err_t result = bme680_get_data(dev_hdl, &data);
        if(result != ESP_OK) {
            ESP_LOGE(TAG, "bme680 device read failed (%s)", esp_err_to_name(result));
        } else {
            data.barometric_pressure = data.barometric_pressure / 100;
            ESP_LOGI(TAG, "air temperature:     %.2f °C", data.air_temperature);
            ESP_LOGI(TAG, "dewpoint temperature:%.2f °C", data.dewpoint_temperature);
            ESP_LOGI(TAG, "relative humidity:   %.2f %%", data.relative_humidity);
            ESP_LOGI(TAG, "barometric pressure: %.2f hPa", data.barometric_pressure);
            ESP_LOGI(TAG, "gas resistance:      %.2f kOhms", data.gas_resistance/1000);
            // ESP_LOGI(TAG, "iaq score:           %u (%s)", data.iaq_score, bme680_iaq_air_quality_to_string(data.iaq_score));
        }
        //
        ESP_LOGI(TAG, "######################## BME680 - END ###########################");
        //
        //
        // pause the task per defined wait period
        // vTaskDelaySecUntil( &last_wake_time, I2C0_TASK_SAMPLING_RATE );
        xTaskDelayUntil( &last_wake_time, 1000 / portTICK_PERIOD_MS );
    }
    //
    // free resources
    bme680_delete( dev_hdl );
    vTaskDelete( NULL );
}


extern "C" void app_main(void) {
    vTaskDelay(1500 / portTICK_PERIOD_MS);
    ESP_LOGI(TAG, "Starting...");
    vTaskDelay(1500 / portTICK_PERIOD_MS);
    
    i2c_master_bus_config_t i2c_mst_config = {
        .i2c_port = I2C_NUM_0,
        .sda_io_num = (gpio_num_t)I2C_SDA_PIN,
        .scl_io_num = (gpio_num_t)I2C_SCL_PIN,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .intr_priority = 0,
        .trans_queue_depth = 0,
        .flags = {
            .enable_internal_pullup = true,
            .allow_pd = 0
        }
    };
    
    i2c_master_bus_handle_t bus_handle;
    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_mst_config, &bus_handle));

    i2c0_bme680_task(bus_handle);

    // uint8_t devices_found = 0;
    
    // ESP_LOGI(TAG, "Scanning I2C bus...");
    
    // for (uint8_t addr = 1; addr < 128; addr++) {
    //     esp_err_t ret = i2c_master_probe(bus_handle, addr, -1);
        
    //     if (ret == ESP_OK) {
    //         ESP_LOGI(TAG, "Device found at address 0x%02X", addr);
    //         devices_found++;
    //     }
    // }
    
    // ESP_LOGI(TAG, "I2C scan completed. Found %d devices.", devices_found);
    
}
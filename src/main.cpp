#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "driver/i2c.h"
#include "sdkconfig.h"

#include "freertos/event_groups.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "lwip/err.h"
#include "lwip/sys.h"
#include "esp_sntp.h"
#include "time.h"
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
    display.fillRect(50, 40, 78, 8, 0); // Clear the area for counter
    display.setCursor(50, 40);
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

extern "C" void app_main(void)
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
    display.setCursor(0, 0);
    display.println("ESP32-C3 with SSD1309");
    display.println("128x64 OLED Display");
    display.println("");
    display.println("Hello World!");
    display.println("");

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
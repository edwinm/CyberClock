#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "driver/i2c.h"
#include "sdkconfig.h"

#define LGFX_USE_V1
#include <LovyanGFX.hpp>

// Display dimension definitions
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64

// I2C pins
#define I2C_SDA_PIN 8
#define I2C_SCL_PIN 9

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

static const char *TAG = "SSD1309_Example";
LGFX_SSD1309 display;

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
    display.print("Uptime: ");
    
    int counter = 0;
    
    // Main program loop
    while (1) {
        // Update counter part
        display.fillRect(50, 40, 78, 8, 0); // Clear the area for counter
        display.setCursor(50, 40);
        display.printf("%d seconds", counter++);
        
        // Display everything
        display.display();
        
        // Wait 1 second
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}
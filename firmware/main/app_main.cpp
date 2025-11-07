/*
 * Death Matter Controller - ESP32-C3
 * 
 * Matter-enabled controller for Death skull with 17 state switches.
 * Communicates with ESP32-WROVER via UART.
 * 
 * Architecture:
 * - 2 trigger switches (FAR_MOTION, NEAR_MOTION)
 * - 15 state command switches (based on death-states.md)
 * - UART protocol with CRC8 for reliable communication
 * - All switches auto-reset to OFF after 500ms
 * 
 * This example code is in the Public Domain (or CC0 licensed, at your option.)
 */

#include <app/server/CommissioningWindowManager.h>
#include <app/server/Server.h>
#include <platform/CHIPDeviceLayer.h>
#include <esp_err.h>
#include <esp_log.h>
#include <esp_matter.h>
#include <esp_matter_ota.h>
#include <esp_matter_endpoint.h>
#include <nvs_flash.h>

// Include the sdkconfig.h file to access Kconfig values
#include "sdkconfig.h"

#include <stdint.h>

#include <app_openthread_config.h>
#include "app_reset.h"
#include "utils/common_macros.h"

// Button component direct include (for factory reset only)
#include "iot_button.h"
#include "button_gpio.h"

// For VID/PID and Onboarding Codes (official example method)
#include <setup_payload/OnboardingCodesUtil.h>

// drivers implemented by this example


#include <esp_matter_event.h>
#include <esp_console.h>
#include <esp_vfs_dev.h>
#include <driver/gpio.h>
#include <esp_timer.h>
#include <driver/uart.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/timers.h>

static const char *TAG = "app_main";

// Global variables - endpoint IDs for all 12 switches
static uint16_t g_endpoint_ids[12] = {0};  // Array to store all endpoint IDs

// Endpoint names for logging
static const char* death_state_names[] = {
    "FAR Motion",
    "NEAR Motion", 
    "Play Welcome",
    "Wait For Near",
    "Play Finger Prompt",
    "Mouth Open Wait Finger",
    "Finger Detected",
    "Snap With Finger",
    "Snap No Finger",
    "Fortune Flow",
    "Fortune Done",
    "Cooldown"
};

// Use the Kconfig value directly

using namespace esp_matter;
using namespace esp_matter::attribute;

// Helper function to set custom endpoint name (simplified for now)
static void set_endpoint_name(endpoint_t *endpoint, const char *name) {
    // TODO: Implement custom naming when ESP-Matter API is better understood
    ESP_LOGI(TAG, "Would set endpoint name to: %s", name);
}
using namespace esp_matter::endpoint;
using namespace chip::app::Clusters;

// Define GPIO pins
#define BUTTON_GPIO CONFIG_BSP_BUTTON_GPIO  // GPIO 9 on ESP32-C3 SuperMini
#define BSP_BUTTON_NUM 0

// UART configuration for ESP32-WROVER communication
#define UART_NUM UART_NUM_1
#define UART_TX_PIN 21
#define UART_RX_PIN 20
#define UART_BAUD 115200
#define UART_BUF_SIZE 1024

// LED for visual feedback
#define LED_GPIO (gpio_num_t)8               // Built-in LED (inverted: LOW=ON)

// UART Protocol
#define FRAME_START 0xA5
#define CRC_POLY 0x31

// Death state commands (C3 to WROVER)
#define CMD_FAR_MOTION       0x01
#define CMD_NEAR_MOTION      0x02
#define CMD_PLAY_WELCOME     0x03
#define CMD_WAIT_FOR_NEAR    0x04
#define CMD_PLAY_FINGER_PROMPT 0x05
#define CMD_MOUTH_OPEN_WAIT_FINGER 0x06
#define CMD_FINGER_DETECTED  0x07
#define CMD_SNAP_WITH_FINGER 0x08
#define CMD_SNAP_NO_FINGER   0x09
#define CMD_FORTUNE_FLOW     0x0A
#define CMD_FORTUNE_DONE     0x0B
#define CMD_COOLDOWN         0x0C
#define CMD_BOOT_HELLO       0x0D  // Repeated at boot until ACK
#define CMD_FABRIC_HELLO     0x0E  // Repeated when joining a fabric until ACK

// Commands from C3 (status notifications)
#define CMD_STATUS_PAIRED    0x10
#define CMD_STATUS_UNPAIRED  0x11

// Responses from WROVER
#define RSP_ACK      0x80
#define RSP_ERR      0x81
#define RSP_BUSY     0x82
#define RSP_DONE     0x83
#define RSP_BOOT_ACK    0x90  // ACK for CMD_BOOT_HELLO
#define RSP_FABRIC_ACK  0x91  // ACK for CMD_FABRIC_HELLO

typedef enum {
    HANDSHAKE_BOOT = 0,
    HANDSHAKE_FABRIC = 1,
} handshake_type_t;

static TimerHandle_t s_boot_handshake_timer = nullptr;
static TimerHandle_t s_fabric_handshake_timer = nullptr;
static bool s_boot_ack_received = false;
static bool s_fabric_ack_received = false;


// ===== LED Control Functions =====
static void led_on() {
    gpio_set_level(LED_GPIO, 0);  // Inverted: LOW = ON
}

static void led_off() {
    gpio_set_level(LED_GPIO, 1);  // Inverted: HIGH = OFF
}

static void led_blink(int count, int on_ms, int off_ms) {
    for (int i = 0; i < count; i++) {
        led_on();
        vTaskDelay(pdMS_TO_TICKS(on_ms));
        led_off();
        if (i < count - 1) {
            vTaskDelay(pdMS_TO_TICKS(off_ms));
        }
    }
}

// LED patterns for different events
static void led_ack() {
    led_blink(2, 100, 100);  // 2 quick blinks
}

static void led_command_sent() {
    led_blink(2, 200, 120);  // 2 medium blinks for each UART TX
}

static void led_error() {
    led_blink(5, 50, 50);  // 5 rapid blinks
}

static void led_hello() {
    led_blink(3, 80, 80);  // 3 quick blinks at boot
}

// ===== CRC8 Calculation =====
static uint8_t crc8(const uint8_t *data, size_t len) {
    uint8_t crc = 0x00;
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x80) {
                crc = (crc << 1) ^ CRC_POLY;
            } else {
                crc <<= 1;
            }
        }
    }
    return crc;
}

// ===== UART Helper Functions =====
static bool uart_send_frame(uint8_t cmd, const uint8_t *payload = nullptr, uint8_t payload_len = 0) {
    uint8_t frame[64];
    uint8_t idx = 0;
    
    // Header
    frame[idx++] = FRAME_START;
    
    // Length = CMD (1 byte) + PAYLOAD
    uint8_t len = 1 + payload_len;
    frame[idx++] = len;
    
    // Command/Response
    frame[idx++] = cmd;
    
    // Payload
    if (payload && payload_len > 0) {
        memcpy(&frame[idx], payload, payload_len);
        idx += payload_len;
    }
    
    // CRC (over LEN + CMD + PAYLOAD)
    uint8_t crc = crc8(&frame[1], idx - 1);
    frame[idx++] = crc;
    
    // Send
    int written = uart_write_bytes(UART_NUM, frame, idx);
    
    ESP_LOGI(TAG, "UART TX: %d bytes, CMD=0x%02X", idx, cmd);
    
    return (written == idx);
}

// Wrapper for responses
static bool uart_send_response(uint8_t response_cmd, const uint8_t *payload = nullptr, uint8_t payload_len = 0) {
    return uart_send_frame(response_cmd, payload, payload_len);
}

static void send_handshake_command(handshake_type_t type) {
    uint8_t cmd = (type == HANDSHAKE_BOOT) ? CMD_BOOT_HELLO : CMD_FABRIC_HELLO;
    const char *label = (type == HANDSHAKE_BOOT) ? "BOOT_HELLO" : "FABRIC_HELLO";

    if (uart_send_frame(cmd, nullptr, 0)) {
        ESP_LOGI(TAG, "UART handshake sent: %s (0x%02X)", label, cmd);
        led_command_sent();
    } else {
        ESP_LOGW(TAG, "Failed to send UART handshake: %s (0x%02X)", label, cmd);
    }
}

static void handshake_timer_callback(TimerHandle_t xTimer) {
    handshake_type_t type = static_cast<handshake_type_t>(reinterpret_cast<uintptr_t>(pvTimerGetTimerID(xTimer)));
    send_handshake_command(type);
}

static void start_handshake_sequence(handshake_type_t type) {
    TimerHandle_t *timer = (type == HANDSHAKE_BOOT) ? &s_boot_handshake_timer : &s_fabric_handshake_timer;
    bool *ack_flag = (type == HANDSHAKE_BOOT) ? &s_boot_ack_received : &s_fabric_ack_received;
    const char *name = (type == HANDSHAKE_BOOT) ? "boot_handshake" : "fabric_handshake";

    if (*timer == nullptr) {
        *timer = xTimerCreate(name, pdMS_TO_TICKS(5000), pdTRUE, reinterpret_cast<void *>(static_cast<uintptr_t>(type)),
                              handshake_timer_callback);
        if (*timer == nullptr) {
            ESP_LOGE(TAG, "Failed to create %s timer", name);
            return;
        }
    }

    *ack_flag = false;
    if (xTimerIsTimerActive(*timer) != pdFALSE) {
        xTimerStop(*timer, 0);
    }

    send_handshake_command(type);

    if (xTimerStart(*timer, 0) != pdPASS) {
        ESP_LOGE(TAG, "Failed to start %s timer", name);
    }
}

static void stop_handshake_sequence(handshake_type_t type) {
    TimerHandle_t timer = (type == HANDSHAKE_BOOT) ? s_boot_handshake_timer : s_fabric_handshake_timer;
    bool *ack_flag = (type == HANDSHAKE_BOOT) ? &s_boot_ack_received : &s_fabric_ack_received;
    const char *label = (type == HANDSHAKE_BOOT) ? "BOOT_HELLO" : "FABRIC_HELLO";

    if (timer != nullptr) {
        xTimerStop(timer, 0);
    }

    *ack_flag = true;
    ESP_LOGI(TAG, "Handshake ACK received: %s", label);
}

// ===== Command Handlers (Receive commands from WROVER) =====
// Note: WROVER can send responses back, but the primary communication
// is C3 -> WROVER commands when switches are triggered

// ===== UART RX Task =====
static void uart_rx_task(void *arg) {
    uint8_t *data = (uint8_t *)malloc(UART_BUF_SIZE);
    uint8_t frame_buf[64];
    uint8_t buf_idx = 0;
    uint8_t state = 0;  // 0=wait start, 1=len, 2=cmd+payload, 3=crc
    uint8_t frame_len = 0;
    uint8_t bytes_to_read = 0;
    uint8_t cmd = 0;
    
    ESP_LOGI(TAG, "UART RX task started");
    
    while (1) {
        int len = uart_read_bytes(UART_NUM, data, UART_BUF_SIZE, pdMS_TO_TICKS(100));
        
        for (int i = 0; i < len; i++) {
            uint8_t b = data[i];
            
            switch (state) {
                case 0:  // Wait for start byte
                    if (b == FRAME_START) {
                        frame_buf[buf_idx++] = b;
                        state = 1;
                    }
                    break;
                    
                case 1:  // Length byte
                    frame_buf[buf_idx++] = b;
                    frame_len = b;
                    if (frame_len == 0 || frame_len > 60) {
                        ESP_LOGW(TAG, "Invalid frame length: %d", frame_len);
                        state = 0;
                        buf_idx = 0;
                    } else {
                        bytes_to_read = frame_len;  // CMD + payload
                        state = 2;
                    }
                    break;
                    
                case 2:  // CMD + Payload
                    frame_buf[buf_idx++] = b;
                    if (buf_idx == 3) {  // Just read CMD byte
                        cmd = b;
                    }
                    bytes_to_read--;
                    if (bytes_to_read == 0) {
                        state = 3;  // Next is CRC
                    }
                    break;
                    
                case 3:  // CRC
                    frame_buf[buf_idx++] = b;
                    uint8_t received_crc = b;
                    uint8_t calc_crc = crc8(&frame_buf[1], buf_idx - 2);
                    
                    if (calc_crc == received_crc) {
                        // Valid frame - extract payload
                        uint8_t payload_len = frame_len - 1;  // Exclude CMD
                        uint8_t *payload = (payload_len > 0) ? &frame_buf[3] : nullptr;
                        
                        // Check if this is a response (0x80+) or command (0x01-0x7F)
                        if (cmd >= 0x80) {
                            switch (cmd) {
                                case RSP_BOOT_ACK:
                                    stop_handshake_sequence(HANDSHAKE_BOOT);
                                    break;
                                case RSP_FABRIC_ACK:
                                    stop_handshake_sequence(HANDSHAKE_FABRIC);
                                    break;
                                default:
                                    ESP_LOGI(TAG, "Received response from WROVER: 0x%02X", cmd);
                                    break;
                            }
                        } else {
                            // This is a command from WROVER - log it
                            ESP_LOGI(TAG, "Received command from WROVER: 0x%02X (not implemented)", cmd);
                        }
                    } else {
                        ESP_LOGE(TAG, "CRC error: expected 0x%02X, got 0x%02X", calc_crc, received_crc);
                        led_error();
                    }
                    
                    // Reset for next frame
                    state = 0;
                    buf_idx = 0;
                    break;
            }
        }
    }
    
    free(data);
}

static void open_commissioning_window_if_necessary()
{
    VerifyOrReturn(chip::Server::GetInstance().GetFabricTable().FabricCount() == 0);

    chip::CommissioningWindowManager & commissionMgr = chip::Server::GetInstance().GetCommissioningWindowManager();
    VerifyOrReturn(commissionMgr.IsCommissioningWindowOpen() == false);

    // After removing last fabric, this example does not remove the Wi-Fi credentials
    // and still has IP connectivity so, only advertising on DNS-SD.
    CHIP_ERROR err = commissionMgr.OpenBasicCommissioningWindow(chip::System::Clock::Seconds16(300),
                                    chip::CommissioningWindowAdvertisement::kDnssdOnly);
    if (err != CHIP_NO_ERROR)
    {
        ESP_LOGE(TAG, "Failed to open commissioning window, err:%" CHIP_ERROR_FORMAT, err.Format());
    }
}

static void app_event_cb(const ChipDeviceEvent *event, intptr_t arg)
{
    switch (event->Type) {
    case chip::DeviceLayer::DeviceEventType::kCommissioningComplete:
        ESP_LOGI(TAG, "Commissioning complete - notifying WROVER");
        // Notify WROVER that we're now paired with HomeKit
        uart_send_frame(CMD_STATUS_PAIRED, nullptr, 0);
        led_blink(5, 100, 100);  // Celebration blinks!
        start_handshake_sequence(HANDSHAKE_FABRIC);
        break;

    case chip::DeviceLayer::DeviceEventType::kFailSafeTimerExpired:
        ESP_LOGI(TAG, "Commissioning failed, fail safe timer expired");
        break;

    case chip::DeviceLayer::DeviceEventType::kFabricRemoved:
        ESP_LOGI(TAG, "Fabric removed successfully - notifying WROVER");
        // Notify WROVER that we're unpaired
        uart_send_frame(CMD_STATUS_UNPAIRED, nullptr, 0);
        open_commissioning_window_if_necessary();
        break;

    case chip::DeviceLayer::DeviceEventType::kBLEDeinitialized:
        ESP_LOGI(TAG, "BLE deinitialized and memory reclaimed");
        break;

    default:
        break;
    }
}

// This callback is invoked when clients interact with the Identify Cluster.
// In the callback implementation, an endpoint can identify itself. (e.g., by flashing a light).
static esp_err_t app_identification_cb(identification::callback_type_t type, uint16_t endpoint_id, uint8_t effect_id,
                                       uint8_t effect_variant, void *priv_data)
{
    ESP_LOGI(TAG, "Identification callback: type: %u, effect: %u, variant: %u", type, effect_id, effect_variant);
    return ESP_OK;
}





// This callback is called for every attribute update. The callback implementation shall
// handle the desired attributes and return an appropriate error code. If the attribute
// is not of your interest, please do not return an error code and strictly return ESP_OK.
// Map endpoint IDs to command codes and names
static int get_endpoint_index(uint16_t endpoint_id) {
    // Find the endpoint in our array and return its index
    for (int i = 0; i < 12; i++) {
        if (g_endpoint_ids[i] == endpoint_id) {
            return i;
        }
    }
    return -1;  // Not found
}

static uint8_t get_command_for_endpoint(uint16_t endpoint_id) {
    int idx = get_endpoint_index(endpoint_id);
    if (idx >= 0) {
        return (uint8_t)(CMD_FAR_MOTION + idx);
    }
    return 0;  // Not found
}

static const char* get_endpoint_name(uint16_t endpoint_id) {
    int idx = get_endpoint_index(endpoint_id);
    if (idx >= 0 && idx < 12) {
        return death_state_names[idx];
    }
    return "Unknown Endpoint";
}

static esp_err_t app_attribute_update_cb(attribute::callback_type_t type, uint16_t endpoint_id, uint32_t cluster_id,
                                         uint32_t attribute_id, esp_matter_attr_val_t *val, void *priv_data)
{
    if (type == PRE_UPDATE) {
        // Handle On/Off cluster commands
        if (cluster_id == OnOff::Id && attribute_id == OnOff::Attributes::OnOff::Id) {
            bool new_state = val->val.b;
            const char* endpoint_name = get_endpoint_name(endpoint_id);
            
            ESP_LOGI(TAG, "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
            ESP_LOGI(TAG, "🎯 MATTER COMMAND RECEIVED");
            ESP_LOGI(TAG, "   Endpoint ID: %d", endpoint_id);
            ESP_LOGI(TAG, "   Endpoint Name: %s", endpoint_name);
            ESP_LOGI(TAG, "   State: %s", new_state ? "ON" : "OFF");
            ESP_LOGI(TAG, "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
            
            if (new_state) {
                // Switch turned ON - send UART command
                uint8_t cmd = get_command_for_endpoint(endpoint_id);
                if (cmd != 0) {
                    ESP_LOGI(TAG, "📤 Sending UART command: 0x%02X (%s)", cmd, endpoint_name);
                    uart_send_frame(cmd, nullptr, 0);
                    led_command_sent();
                    
                    // Auto-reset to OFF after 500ms
                    vTaskDelay(pdMS_TO_TICKS(500));
                    esp_matter_attr_val_t off_val = esp_matter_bool(false);
                    attribute::report(endpoint_id, OnOff::Id, OnOff::Attributes::OnOff::Id, &off_val);
                    ESP_LOGI(TAG, "✅ Auto-reset %s to OFF", endpoint_name);
                } else {
                    ESP_LOGW(TAG, "⚠️  Unknown endpoint ID: %d (%s)", endpoint_id, endpoint_name);
                }
            }
        }
    }
    return ESP_OK;
}

static esp_err_t factory_reset_button_register()
{
    // Create button configurations
    button_config_t button_config = {
        .long_press_time = 5000,     // 5 seconds for long press
        .short_press_time = 50,      // 50ms for short press
    };
    
    button_gpio_config_t gpio_config = {
        .gpio_num = 9,               // GPIO 9 (BOOT button on ESP32-C3 SuperMini)
        .active_level = 0,           // Active low
        .enable_power_save = false,  // No power save
        .disable_pull = false,       // Use internal pull-up
    };
    
    button_handle_t push_button = NULL;
    
    // Create the GPIO button device
    esp_err_t err = iot_button_new_gpio_device(&button_config, &gpio_config, &push_button);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create button device: %s", esp_err_to_name(err));
        return err;
    }
    
    return app_reset_button_register(push_button);
}

// AFTER factory_reset_button_register(), add switch button callback registration

/* -------------------------------------------------------------------------- */
/* Generic Switch button callback                                             */
/* -------------------------------------------------------------------------- */





// Physical button handling removed - GPIO control is now via Matter commands only

// Button registration removed - GPIO control is now via Matter commands only

// Simple factory reset trigger - will reset after 10 seconds
static void trigger_factory_reset_timer(void)
{
    ESP_LOGW(TAG, "=== FACTORY RESET TRIGGERED ===");
    ESP_LOGW(TAG, "Device will reset in 10 seconds...");
    ESP_LOGW(TAG, "Unplug power now if you want to cancel!");
    
    vTaskDelay(pdMS_TO_TICKS(10000)); // Wait 10 seconds
    
    ESP_LOGI(TAG, "Starting factory reset NOW");
    esp_matter::factory_reset();
}

// Console command for factory reset
static int factory_reset_cmd(int argc, char **argv)
{
    if (argc == 2 && strcmp(argv[1], "confirm") == 0) {
        // Start the reset in a new task
        xTaskCreate([](void*){ trigger_factory_reset_timer(); vTaskDelete(NULL); }, 
                   "factory_reset", 4096, NULL, 5, NULL);
        return 0;
    } else {
        printf("Usage: factory_reset confirm\n");
        printf("WARNING: This will erase all pairing data!\n");
        return 1;
    }
}

static void register_factory_reset_console_cmd()
{
    esp_console_cmd_t cmd = {
        .command = "factory_reset",
        .help = "Perform factory reset (use 'factory_reset confirm')",
        .hint = NULL,
        .func = &factory_reset_cmd,
    };
    esp_console_cmd_register(&cmd);
}

extern "C" void app_main()
{
    /* Initialize the ESP NVS layer */
    nvs_flash_init();

    /* Initialize console for factory reset command */
    esp_console_repl_t *repl = NULL;
    esp_console_repl_config_t repl_config = ESP_CONSOLE_REPL_CONFIG_DEFAULT();
    esp_console_dev_uart_config_t uart_config = ESP_CONSOLE_DEV_UART_CONFIG_DEFAULT();
    esp_console_new_repl_uart(&uart_config, &repl_config, &repl);
    register_factory_reset_console_cmd();
    esp_console_start_repl(repl);

    /* Initialize push button on the dev-kit to reset the device */
    esp_err_t err = factory_reset_button_register();
    ABORT_APP_ON_FAILURE(ESP_OK == err, ESP_LOGE(TAG, "Failed to initialize reset button, err:%d", err));

    /* Initialize LED for visual feedback */
    gpio_config_t led_conf = {
        .pin_bit_mask = (1ULL << LED_GPIO),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    err = gpio_config(&led_conf);
    ABORT_APP_ON_FAILURE(ESP_OK == err, ESP_LOGE(TAG, "Failed to configure LED GPIO, err:%d", err));
    led_off();  // Start with LED off
    ESP_LOGI(TAG, "LED GPIO %d initialized", LED_GPIO);
    led_hello();  // Boot indicator: 3 quick flashes

    /* Initialize UART for ESP32-WROVER communication */
    uart_config_t uart_conf = {
        .baud_rate = UART_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 122,
        .source_clk = UART_SCLK_DEFAULT,
    };
    err = uart_param_config(UART_NUM, &uart_conf);
    ABORT_APP_ON_FAILURE(ESP_OK == err, ESP_LOGE(TAG, "Failed to configure UART params, err:%d", err));
    
    err = uart_set_pin(UART_NUM, UART_TX_PIN, UART_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    ABORT_APP_ON_FAILURE(ESP_OK == err, ESP_LOGE(TAG, "Failed to set UART pins, err:%d", err));
    
    err = uart_driver_install(UART_NUM, UART_BUF_SIZE, UART_BUF_SIZE, 0, NULL, 0);
    ABORT_APP_ON_FAILURE(ESP_OK == err, ESP_LOGE(TAG, "Failed to install UART driver, err:%d", err));
    
    ESP_LOGI(TAG, "UART initialized: TX=%d, RX=%d, Baud=%d", UART_TX_PIN, UART_RX_PIN, UART_BAUD);
    
    /* Start UART RX task */
    xTaskCreate(uart_rx_task, "uart_rx", 4096, NULL, 10, NULL);
    ESP_LOGI(TAG, "UART RX task created");

    /* Begin boot handshake with WROVER */
    start_handshake_sequence(HANDSHAKE_BOOT);
    
    /* Create a Matter node and add the mandatory Root Node device type on endpoint 0 */
    node::config_t node_config{}; // Explicitly zero-initialize

    // --- BEGIN CUSTOM DEVICE INFO CONFIGURATION ---
    // When 'Device Info Provider' is 'Custom' (via menuconfig),
    // core device identity (VID, PID, names, versions) is primarily expected
    // from the factory NVS partition.
    // Kconfig settings under "Device Basic Information" provide other fields.
    // We will only set the node_label here if we want to override the Kconfig value at runtime.

    // Basic Information Cluster Configuration for Root Node
    // Set a node_label (user-visible device name for this node).
    // This overrides any node_label set via Kconfig.
    strncpy(node_config.root_node.basic_information.node_label, "Death Controller", 
            sizeof(node_config.root_node.basic_information.node_label) - 1);
    node_config.root_node.basic_information.node_label[sizeof(node_config.root_node.basic_information.node_label) - 1] = '\0';
    ESP_LOGI(TAG, "Device name set to: Death Controller");

    // Identify Cluster on Root Node is mandatory and typically initialized by default by the SDK.

    // --- END CUSTOM DEVICE INFO CONFIGURATION ---

    node_t *node = node::create(&node_config, app_attribute_update_cb, app_identification_cb);
    ABORT_APP_ON_FAILURE(node != nullptr, ESP_LOGE(TAG, "Failed to create Matter node"));

    // ------------------------------------------------------------------
    // Create 12 On/Off Plugin Unit endpoints for Death state switches
    // ------------------------------------------------------------------
    
    on_off_plug_in_unit::config_t switch_cfg; // default config
    
    ESP_LOGI(TAG, "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
    ESP_LOGI(TAG, "🔧 Creating Matter Endpoints:");
    ESP_LOGI(TAG, "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
    
    for (int i = 0; i < 12; i++) {
        endpoint_t *switch_ep = on_off_plug_in_unit::create(node, &switch_cfg, ENDPOINT_FLAG_NONE, NULL);
        ABORT_APP_ON_FAILURE(switch_ep != nullptr, ESP_LOGE(TAG, "Failed to create %s plugin unit endpoint", death_state_names[i]));
        g_endpoint_ids[i] = endpoint::get_id(switch_ep);
        
        // Set custom name
        set_endpoint_name(switch_ep, death_state_names[i]);
        
        ESP_LOGI(TAG, "   ✅ Endpoint %d: %s (ID: %d)", i + 1, death_state_names[i], g_endpoint_ids[i]);
    }
    
    ESP_LOGI(TAG, "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
    ESP_LOGI(TAG, "💡 TIP: Toggle endpoints in Apple Home to see which is which!");
    ESP_LOGI(TAG, "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");

    // GPIO control is now handled via Matter commands only

#if CHIP_DEVICE_CONFIG_ENABLE_THREAD
    /* Set OpenThread platform config */
    esp_openthread_platform_config_t config = {
        .radio_config = ESP_OPENTHREAD_DEFAULT_RADIO_CONFIG(),
        .host_config = ESP_OPENTHREAD_DEFAULT_HOST_CONFIG(),
        .port_config = ESP_OPENTHREAD_DEFAULT_PORT_CONFIG(),
    };
    set_openthread_platform_config(&config);
#endif

    /* Matter start */
    err = esp_matter::start(app_event_cb);
    ABORT_APP_ON_FAILURE(err == ESP_OK, ESP_LOGE(TAG, "Failed to start Matter, err:%d", err));

    // PrintOnboardingCodes will log the necessary VID/PID and commissioning info
    chip::DeviceLayer::StackLock lock; // RAII lock for Matter stack
    PrintOnboardingCodes(chip::RendezvousInformationFlags(chip::RendezvousInformationFlag::kBLE).Set(chip::RendezvousInformationFlag::kOnNetwork));
}

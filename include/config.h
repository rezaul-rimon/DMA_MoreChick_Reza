#define DEBUG_MODE true
#define USE_Fast_LED


#define CONFIG_TASK_WDT_DEBUG 1



// Start Library Include section //
#include <Arduino.h>
#include <WiFi.h>
#include <WiFiManager.h>
#include <PubSubClient.h>
#include <HTTPClient.h>
#include <esp_task_wdt.h>
#include <Wire.h>
#include <Preferences.h>
#include <FastLED.h>
// ---------------------------- //


//Configuration Section Start
//-------------------------//

// Debug mode Config
#define DEBUG_PRINT(x)  if (DEBUG_MODE) { Serial.print(x); }
#define DEBUG_PRINTLN(x) if (DEBUG_MODE) { Serial.println(x); }
// ----------------------- //

// Sensor Config
// ----------------------- //




// ----------------------- //

// Device ID Config
#define CHANGE_DEVICE_ID 0 // Define as 1 (true) if new firmware, 0 (false) otherwise.

#if CHANGE_DEVICE_ID
    #define WORK_PACKAGE "1178"
    #define GW_TYPE "00"
    #define FIRMWARE_UPDATE_DATE "251015" // Format: yymmdd
    #define DEVICE_SERIAL "0999"
#endif

// ----------------------- //


// Heartbeat and Data send interval config (in milliseconds)
#define HB_INTERVAL 2*60*1000
#define DATA_INTERVAL 5*60*1000
// ----------------------- //

// FastLED Config
#ifdef USE_Fast_LED
    #define DATA_PIN 27
    #define NUM_LEDS 1
    CRGB leds[NUM_LEDS];
#endif
// ----------------------- //

// ----------------------- //

// WiFi and MQTT reconnection time config
#define WIFI_ATTEMPT_COUNT 60
#define WIFI_ATTEMPT_DELAY 1000
#define WIFI_WAIT_COUNT 60
#define WIFI_WAIT_DELAY 1000
#define MAX_WIFI_ATTEMPTS 2
#define MQTT_ATTEMPT_COUNT 12
#define MQTT_ATTEMPT_DELAY 5000
// ----------------------- //

// WiFi and MQTT attempt counters
int wifiAttemptCount = WIFI_ATTEMPT_COUNT;
int wifiWaitCount = WIFI_WAIT_COUNT;
int maxWifiAttempts = MAX_WIFI_ATTEMPTS;
int mqttAttemptCount = MQTT_ATTEMPT_COUNT;
// ----------------------- //

// MQTT Server Config
const char* mqtt_server = "broker2.dma-bd.com";
const char* mqtt_user = "broker2";
const char* mqtt_password = "Secret!@#$1234";
const char* mqtt_hb_topic = "DMA/MC/HB";
const char* mqtt_pub_topic = "DMA/MC/PUB";
const char* mqtt_sub_topic = "DMA/MC/SUB";
const char* ota_url = "https://raw.githubusercontent.com/rezaul-rimon/DMA_MoreChick_Reza/main/ota/firmware.bin";
// ----------------------- //

//Sensor and other global variables
//----------------------- //


const char* DEVICE_ID;
Preferences preferences;

// WiFi Reset Button
#define WIFI_RESET_BUTTON_PIN 0

//Enf of Configuration Section
//-------------------------//



//Start Making instance Section//
//-----------------------------//

//Wifi and MQTT Instance
WiFiManager wm;
WiFiClient espClient;
PubSubClient client(espClient);

//---------------------------//

//FreeRTOS Task instances
TaskHandle_t networkTaskHandle;
TaskHandle_t mainTaskHandle;
TaskHandle_t wifiResetTaskHandle = NULL;
TaskHandle_t otaTaskHandle = NULL;
//---------------------------//


//End Making instance Section//
//-----------------------------//
// Start Library Include section //
#include <Arduino.h>
#include <WiFi.h>
#include <WiFiManager.h>
#include <PubSubClient.h>
#include <HTTPClient.h>
#include <Wire.h>
#include <ArtronShop_SHT3x.h>
#include <Preferences.h>
#include <FastLED.h>
// ---------------------------- //


//Configuration Section Start
//-------------------------//

// Debug mode Config
#define DEBUG_MODE true
#define DEBUG_PRINT(x)  if (DEBUG_MODE) { Serial.print(x); }
#define DEBUG_PRINTLN(x) if (DEBUG_MODE) { Serial.println(x); }
// ----------------------- //

// Device Type Config
#define MP702 1 // Define as 1 (true) if Ammonia sensor exists, 0 (false) otherwise.
#if MP702
    // For Chicken Farm
    #define GW_TYPE "00"
#else
    // For Pharmaceuticals
    #define GW_TYPE "01"
#endif
// ----------------------- //
#if MP702
  #define SENSOR_PIN 34 // Pin for ammonia sensor
  #define RL 10.0       // Load resistance in kOhm
#endif
// ----------------------- //

// Device ID Config
#define CHANGE_DEVICE_ID 0 // Define as 1 (true) if new firmware, 0 (false) otherwise.

#if CHANGE_DEVICE_ID
    #define WORK_PACKAGE "1178"
    #define FIRMWARE_UPDATE_DATE "251015" // Format: yymmdd
    #define DEVICE_SERIAL "0009"
#endif

// ----------------------- //


// Heartbeat and Data send interval config (in milliseconds)
#define HB_INTERVAL 2*60*1000
#define DATA_INTERVAL 5*60*1000
// ----------------------- //

// FastLED Config
#define Fast_LED 1 // Define as 1 (true) if FastLED library is used, 0 (false) otherwise.
#if Fast_LED
    #define DATA_PIN 4
    #define NUM_LEDS 1
    CRGB leds[NUM_LEDS];
#endif
// ----------------------- //

// SXT sensor check config
#define SXT_ATTEMPT_EACH 5          
#define SXT_RECHECK_INTERVAL 30000
// ----------------------- //
bool sxt_available = false;
int sxt_attempt_count = 0;
unsigned long last_sxt_check_time = 0;
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

// Address for GY-302 Light Sensor
#define ADDR_GY302 0x23
// ----------------------- //

const char* DEVICE_ID;
Preferences preferences;

// WiFi Reset Button
#define WIFI_RESET_BUTTON_PIN 0

// #define LED_PIN 25

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

//SHT3x sensor instance
ArtronShop_SHT3x sht3x(0x44, &Wire); // ADDR: 0 => 0x44, ADDR: 1 => 0x45


//End Making instance Section//
//-----------------------------//
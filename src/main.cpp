// Start Library Include section //
// ---------------------------- //
#include <Arduino.h>
#include <WiFi.h>
#include <WiFiManager.h>  // WiFiManager library
#include <PubSubClient.h>
#include <Wire.h>
#include <ArtronShop_SHT3x.h>

// #include <FastLED.h>

// End Library Include section //
// ---------------------------- //


//Configuration Section Start
//-------------------------//

// Debug mode Config
#define DEBUG_MODE true
#define DEBUG_PRINT(x)  if (DEBUG_MODE) { Serial.print(x); }
#define DEBUG_PRINTLN(x) if (DEBUG_MODE) { Serial.println(x); }

// Device Config
#define WORK_PACKAGE "1178"
#define GW_TYPE "00"
#define FIRMWARE_UPDATE_DATE "241121" // Format: yymmdd
#define DEVICE_SERIAL "0002"
#define DEVICE_ID WORK_PACKAGE GW_TYPE FIRMWARE_UPDATE_DATE DEVICE_SERIAL

#define HB_INTERVAL 30*1000
#define DATA_INTERVAL 1*60*1000

// SXT sensor control
#define SXT_ATTEMPT_EACH 5          
#define SXT_RECHECK_INTERVAL 30000

// WiFi and MQTT reconnection time config
#define WIFI_ATTEMPT_COUNT 30
#define WIFI_ATTEMPT_DELAY 1000
#define WIFI_WAIT_COUNT 60
#define WIFI_WAIT_DELAY 1000
#define MAX_WIFI_ATTEMPTS 2
#define MQTT_ATTEMPT_COUNT 10
#define MQTT_ATTEMPT_DELAY 5000

// WiFi and MQTT attempt counters
int wifiAttemptCount = WIFI_ATTEMPT_COUNT;
int wifiWaitCount = WIFI_WAIT_COUNT;
int maxWifiAttempts = MAX_WIFI_ATTEMPTS;
int mqttAttemptCount = MQTT_ATTEMPT_COUNT;

// MQTT Server Config
const char* mqtt_server = "broker2.dma-bd.com";
const char* mqtt_user = "broker2";
const char* mqtt_password = "Secret!@#$1234";
const char* mqtt_topic = "DMA/MC/PUB";


//End Configuration Section//
//-------------------------//


//Start Making instance Section//
//-----------------------------//

//Wifi and MQTT Instance
WiFiClient espClient;
PubSubClient client(espClient);

//FreeRTOS Task instance
TaskHandle_t networkTaskHandle;
TaskHandle_t mainTaskHandle;
TaskHandle_t wifiResetTaskHandle;

// Initialize SHT3x sensor
ArtronShop_SHT3x sht3x(0x44, &Wire); // ADDR: 0 => 0x44, ADDR: 1 => 0x45


//End Making instance Section//
//-----------------------------//


//Start Variable declaretion Section//
//----------------------------------//

// WiFi Reset Button
#define WIFI_RESET_BUTTON_PIN 35
#define SENSOR_PIN 34 // Pin for ammonia sensor
#define LED_PIN 25 //Status LED Pin

//WiFi Reset Flag
bool wifiResetFlag = false;
bool sxt_available = false;
int sxt_attempt_count = 0;
#define RL 10.0       // Load resistance in kOhm
unsigned long last_sxt_check_time = 0;

//Heartbeat and Data send interval variable
// unsigned long hbLastTime = 0, dataLastTime = 0;



//End Variable declaretion Section//
//--------------------------------//



// Start Function Section //
//-----------------------//

// Function to reconnect to WiFi
void reconnectWiFi() {
  digitalWrite(LED_PIN, HIGH);
  if (WiFi.status() != WL_CONNECTED) {
    if (wifiAttemptCount > 0) {
      DEBUG_PRINTLN("Attempting WiFi connection...");
      WiFi.begin();  // Use saved credentials
      wifiAttemptCount--;
      DEBUG_PRINTLN("Remaining WiFi attempts: " + String(wifiAttemptCount));
      // vTaskDelay(WIFI_ATTEMPT_DELAY / portTICK_PERIOD_MS);
      vTaskDelay(pdMS_TO_TICKS(WIFI_ATTEMPT_DELAY));
    } else if (wifiWaitCount > 0) {
      wifiWaitCount--;
      DEBUG_PRINTLN("WiFi wait... retrying in a moment");
      DEBUG_PRINTLN("Remaining WiFi wait time: " + String(wifiWaitCount) + " seconds");
      vTaskDelay(pdMS_TO_TICKS(WIFI_WAIT_DELAY));
    } else {
      wifiAttemptCount = WIFI_ATTEMPT_COUNT;
      wifiWaitCount = WIFI_WAIT_COUNT;
      maxWifiAttempts--;
      if (maxWifiAttempts <= 0) {
        DEBUG_PRINTLN("Max WiFi attempt cycles exceeded, restarting...");
        ESP.restart();
      }
    }
  }
}

// Function to reconnect to MQTT with a unique client ID
void reconnectMQTT() {
  if (!client.connected()) {
    digitalWrite(LED_PIN, HIGH);
    char clientId[16];  // 1 byte for "dma_em_" + 8 bytes for random hex + null terminator
    snprintf(clientId, sizeof(clientId), "dma_mc_%04X%04X", random(0xffff), random(0xffff));

    if (mqttAttemptCount > 0) {
      DEBUG_PRINTLN("Attempting MQTT connection...");
      
      if (client.connect(clientId, mqtt_user, mqtt_password)) {  // Use the unique client ID
        DEBUG_PRINTLN("MQTT connected");
        DEBUG_PRINT("Client_ID: ");
        DEBUG_PRINTLN(clientId);
        digitalWrite(LED_PIN, LOW);

        char topic[48];
        snprintf(topic, sizeof(topic), "%s/%s", mqtt_topic, DEVICE_ID);
        client.subscribe(topic);
        
      } else {
        DEBUG_PRINTLN("MQTT connection failed");
        DEBUG_PRINTLN("Remaining MQTT attempts: " + String(mqttAttemptCount));
        mqttAttemptCount--;
        vTaskDelay(pdMS_TO_TICKS(MQTT_ATTEMPT_DELAY));
      }
    } else {
      DEBUG_PRINTLN("Max MQTT attempts exceeded, restarting...");
      ESP.restart();
    }
  }
}

void mqttCallback(char* topic, byte* payload, unsigned int length) {
  String message;
  for (unsigned int i = 0; i < length; i++) {
    message += (char)payload[i];
  }

  // Print the topic and message for debugging
  DEBUG_PRINTLN("Message arrived on topic: " + String(topic));
  DEBUG_PRINTLN("Message content: " + message);

  // Check if the message is "get_from_sd_card"
  if (message == "get_data_from_sd_card") {
    DEBUG_PRINTLN("Triggering sendToFtp()...");
   
  }

  // Check if the message is "get_from_sd_card"
  if (message == "clear_sd_card") {
    DEBUG_PRINTLN("Triggering sendToFtp()...");
  }
}

// Check SXT sensor availability
void check_sxt_sensor() {
  DEBUG_PRINTLN("Checking SXT sensor availability...");
  sxt_attempt_count = 0;
  
  while (!sht3x.begin() && sxt_attempt_count < SXT_ATTEMPT_EACH) {
    DEBUG_PRINTLN("SXT sensor not found, retrying...");
    vTaskDelay(pdMS_TO_TICKS(1000));
    sxt_attempt_count++;
  }

  if (sht3x.begin()) {
    sxt_available = true;
    DEBUG_PRINTLN("SXT sensor connected successfully.");
  } else {
    sxt_available = false;
    DEBUG_PRINTLN("SXT sensor not available, continuing without SXT data.");
  }
}


// End Function Section //
//----------------------//


// Start FreeRTOS Task Section //
//----------------------//

// Network Task for WiFi and MQTT
void networkTask(void *param) {
  WiFi.mode(WIFI_STA);
  WiFi.begin();

  for (;;) {
    // Check WiFi connection
    if (WiFi.status() == WL_CONNECTED) {
      // Check and reconnect MQTT if necessary
      if (!client.connected()) {
        reconnectMQTT();
      }
    } else {
      // Reconnect WiFi if disconnected
      reconnectWiFi();
    }

    // Loop MQTT client for processing incoming messages
    client.loop();

    // Delay for 100ms before next cycle
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}


// Task for WiFi reset and WiFiManager setup
void wifiResetTask(void *param) {
  for (;;) {
    // Check if button is pressed (LOW state)
    if (digitalRead(WIFI_RESET_BUTTON_PIN) == LOW) {
      unsigned long pressStartTime = millis(); // Record the time when the button is pressed
      DEBUG_PRINTLN("Button Pressed....");

      // Wait for at least 5 seconds to confirm long press
      while (digitalRead(WIFI_RESET_BUTTON_PIN) == LOW) {
        // Check if button is still pressed after 5 seconds
        if (millis() - pressStartTime >= 5000) {
          DEBUG_PRINTLN("5 seconds holding time reached, starting WiFiManager...");

          // Suspend other tasks to avoid conflict
          vTaskSuspend(networkTaskHandle);
          vTaskSuspend(mainTaskHandle);

          DEBUG_PRINTLN("Starting WiFiManager for new WiFi setup...");
          WiFiManager wifiManager;
          wifiManager.resetSettings();  // Clear previous settings
          wifiManager.autoConnect("DMA_MC_Setup"); // Start AP for new configuration

          DEBUG_PRINTLN("New WiFi credentials set, restarting...");
          ESP.restart();  // Restart after WiFi configuration
        }

        vTaskDelay(pdMS_TO_TICKS(100));  // Small delay to avoid overwhelming the system
      }
    }

    vTaskDelay(pdMS_TO_TICKS(100));  // Check every 100 ms
  }
}


/*********************************************************************/
/*                                  main                             */
/*********************************************************************/

void mainTask(void *param) {
  for (;;) {

    // Get the current time's epoch
    static unsigned long last_hb_send_time = 0;
    if (millis() - last_hb_send_time >= HB_INTERVAL) {
      last_hb_send_time = millis();

        if (client.connected()) {
            char hb_data[50];  // Buffer for the heartbeat data

            // Format the heartbeat message into the buffer
            snprintf(hb_data, sizeof(hb_data), "%s,wifi_connected", DEVICE_ID);

            // Publish the heartbeat message
            client.publish(mqtt_topic, hb_data);
            DEBUG_PRINTLN("Heartbeat published data to mqtt");
            digitalWrite(LED_PIN, HIGH);
            vTaskDelay(pdMS_TO_TICKS(1000));
            digitalWrite(LED_PIN, LOW);
        } else {
            DEBUG_PRINTLN("Failed to publish Heartbeat on MQTT");
        }
    }

    static unsigned long last_data_send_time = 0;
    if (millis() - last_data_send_time >= DATA_INTERVAL) {
      last_data_send_time = millis();
      // Read ammonia sensor
      int sensorValue = analogRead(SENSOR_PIN);
      float sensorVoltage = sensorValue * (3.3 / 4095.0); // ESP32 12-bit ADC
      float Rs = (3.3 - sensorVoltage) * RL / sensorVoltage;
      float ratio = Rs / RL;
      float ppm = pow(10, ((log10(ratio) - 0.0) / -0.6)); // Adjust based on sensor sensitivity curve

      String payload = String(DEVICE_ID) + ","; // Initialize payload with GWID

      // Check if SXT is available
      if (sxt_available) {
        if (sht3x.measure()) {
          // If measurement is successful, set sxt_available to true
          sxt_available = true;
          float temperature = sht3x.temperature();
          float humidity = sht3x.humidity();
          payload += String(temperature, 2) + "," + String(humidity, 2) + ",";
        } else {
          // If there's an error in measurement, mark it as unavailable and add placeholder data
          DEBUG_PRINTLN("SXT read error");
          sxt_available = false; // Set to false if measurement fails
          payload += "N/A,N/A,";
        }
      } else {
        // If SXT is not available, add placeholder values
        payload += "N/A,N/A,";
      }

      payload += String(ppm, 2); // Append ammonia data

      client.publish(mqtt_topic, payload.c_str());
      DEBUG_PRINTLN("Data sent -> ");
      DEBUG_PRINTLN(payload);
      digitalWrite(LED_PIN, HIGH);
      vTaskDelay(pdMS_TO_TICKS(1000));
      digitalWrite(LED_PIN, LOW);
    }

    // Check for SXT sensor every SXT_RECHECK_INTERVAL
    if (!sxt_available && millis() - last_sxt_check_time >= SXT_RECHECK_INTERVAL) {
      last_sxt_check_time = millis();
      check_sxt_sensor(); // Retry connecting SXT sensor
    }

    // Additional task (optional, e.g., print debug message every second)
    // DEBUG_PRINTLN(timestamp);
    // DEBUG_PRINTLN("Hello");
    vTaskDelay(pdMS_TO_TICKS(1000));  // Print "Hello" every second
  }
}


void setup() {
  // Serial Monitor buad rate
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);
  delay(2000);
  digitalWrite(LED_PIN, LOW);

  Serial.begin(115200);

  Serial.print("Device ID: ");
  Serial.println(DEVICE_ID);
  delay(1000);

  // LED setup
  // FastLED.addLeds<NEOPIXEL, DATA_PIN>(leds, NUM_LEDS);

  // timeClient.begin();

  // Button setup
  pinMode(WIFI_RESET_BUTTON_PIN, INPUT_PULLUP);

  pinMode(SENSOR_PIN, INPUT);
  Wire.begin();
  check_sxt_sensor();

  // Set up MQTT client
  client.setServer(mqtt_server, 1883);
  client.setCallback(mqttCallback);

  // Create tasks
  xTaskCreatePinnedToCore(networkTask, "Network Task", 8*1024, NULL, 1, &networkTaskHandle, 0);
  xTaskCreatePinnedToCore(mainTask, "Main Task", 16*1024, NULL, 1, &mainTaskHandle, 1);
  xTaskCreatePinnedToCore(wifiResetTask, "WiFi Reset Task", 4*1024, NULL, 1, &wifiResetTaskHandle, 1);
}

// Loop function
void loop() {
}





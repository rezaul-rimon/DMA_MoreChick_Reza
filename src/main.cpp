#include <config.h>

// Function Prototypes
void reconnectWiFi();
void reconnectMQTT();
void check_sxt_sensor();
float readGY302(uint8_t address);
void initBH1750(uint8_t address);
void mqttCallback(char* topic, byte* payload, unsigned int length);
void networkTask(void *param);
void mainTask(void *param);
void wifiResetTask(void *param);
void otaTask(void *parameter);
//-----------------------//



// Start Function Section //
//-----------------------//

// Function to reconnect to WiFi
void reconnectWiFi() {
  // digitalWrite(LED_PIN, HIGH);
  #if Fast_LED
    leds[0] = CRGB::Red;
    FastLED.show();
  #endif

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
//-----------------------//

// Function to reconnect to MQTT with a unique client ID
void reconnectMQTT() {
  if (!client.connected()) {
    // digitalWrite(LED_PIN, HIGH);
    #if Fast_LED
      leds[0] = CRGB::Yellow;
      FastLED.show();
    #endif
    char clientId[16];  // 1 byte for "dma_em_" + 8 bytes for random hex + null terminator
    snprintf(clientId, sizeof(clientId), "dma_mc_%04X%04X", random(0xffff), random(0xffff));

    if (mqttAttemptCount > 0) {
      DEBUG_PRINTLN("Attempting MQTT connection...");
      
      if (client.connect(clientId, mqtt_user, mqtt_password)) {  // Use the unique client ID
        DEBUG_PRINTLN("MQTT connected");
        DEBUG_PRINT("Client_ID: ");
        DEBUG_PRINTLN(clientId);
        // digitalWrite(LED_PIN, LOW);
        #if Fast_LED
          leds[0] = CRGB::Black;
          FastLED.show();
        #endif

        char topic[48];
        snprintf(topic, sizeof(topic), "%s/%s", mqtt_sub_topic, DEVICE_ID);
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
//-----------------------//

// MQTT message callback function
void mqttCallback(char* topic, byte* payload, unsigned int length) {
  String message;
  for (unsigned int i = 0; i < length; i++) {
    message += (char)payload[i];
  }
  #if Fast_LED
    leds[0] = CRGB::Blue;
    FastLED.show();
    vTaskDelay(pdMS_TO_TICKS(500)); // Short delay to indicate message received
    leds[0] = CRGB::Black;
    FastLED.show();
  #endif
  
  // Print the topic and message for debugging
  DEBUG_PRINTLN("Message arrived on topic: " + String(topic));
  DEBUG_PRINTLN("Message content: " + message);

  // Check if the message is "update_firmware"
  if (message == "update_firmware") {
    if (otaTaskHandle == NULL) {
      xTaskCreatePinnedToCore(otaTask, "OTA Task", 8*1024, NULL, 1, &otaTaskHandle, 1);
    } else {
      Serial.println("OTA Task already running.");
    }
  }

}
//-----------------------//

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
    #if Fast_LED
      leds[0] = CRGB::Red;
      FastLED.show();
      vTaskDelay(pdMS_TO_TICKS(500));
      leds[0] = CRGB::Black;
      FastLED.show();
    #endif
  }
}
//-----------------------//

// Initialize BH1750 / GY302 Light sensor in continuous mode
void initBH1750(uint8_t address) {
  Wire.beginTransmission(address);
  Wire.write(0x01); // Power on
  Wire.endTransmission();

  Wire.beginTransmission(address);
  Wire.write(0x10); // Continuous H-Resolution Mode (1 lx resolution, 120ms)
  Wire.endTransmission();
}
//-----------------------//

// Read lux value from the sensor
float readGY302(uint8_t address) {
  int16_t val = -1;  // Default to -1 (error)

  delay(180); // Wait for measurement to complete

  if (Wire.requestFrom(address, (uint8_t)2) == 2) {  // Ensure 2 bytes are received
    val = Wire.read();
    val <<= 8;
    val |= Wire.read();
  }

  return (val == -1) ? -1.00 : val / 1.2; // Convert to lux or return error
}
//-----------------------//


// End Function Section //
//----------------------//


// Start FreeRTOS Task Section //
//----------------------//

// Start OTA Task
void otaTask(void *parameter) {
  Serial.println("Starting OTA update...");

  #if Fast_LED
    leds[0] = CRGB::Green;
    FastLED.show();
    vTaskDelay(pdMS_TO_TICKS(250)); // Short delay to indicate status
    leds[0] = CRGB::Black;
    FastLED.show();
    vTaskDelay(pdMS_TO_TICKS(250)); // Short delay to indicate status
    leds[0] = CRGB::Green;
    FastLED.show();
    vTaskDelay(pdMS_TO_TICKS(250)); // Short delay to indicate status
    leds[0] = CRGB::Black;
    FastLED.show();
  #endif

  HTTPClient http;
  http.begin(ota_url);
  int httpCode = http.GET();

  if (httpCode == HTTP_CODE_OK) {
    int contentLength = http.getSize();
    Serial.printf("Content-Length: %d bytes\n", contentLength);
    
    if (Update.begin(contentLength)) {
      Update.writeStream(http.getStream());
      if (Update.end() && Update.isFinished()) {
        Serial.println("OTA update completed. Restarting...");
        char message[64];  
        snprintf(message, sizeof(message), "%s,OTA update successful", DEVICE_ID);  
        client.publish(mqtt_pub_topic, message);
        vTaskDelay(2000 / portTICK_PERIOD_MS);
        http.end();
        ESP.restart();
      } else {
        Serial.println("OTA update failed!");
        char message[64];  
        snprintf(message, sizeof(message), "%s,OTA Update Failed!", DEVICE_ID);  
        client.publish(mqtt_pub_topic, message);
      }
    } else {
      Serial.println("OTA begin failed!");
      char message[64];  
      snprintf(message, sizeof(message), "%s,OTA Begin Failed!", DEVICE_ID);  
      client.publish(mqtt_pub_topic, message);
    }
  } else {
    Serial.printf("HTTP request failed, error: %s\n", http.errorToString(httpCode).c_str());
    char message[64];  
    snprintf(message, sizeof(message), "%s,HTTP Request Failed", DEVICE_ID);  
    client.publish(mqtt_pub_topic, message);
  }

  http.end();
  vTaskDelay(2000 / portTICK_PERIOD_MS);
  
  ESP.restart();

  otaTaskHandle = NULL;  
  vTaskDelete(NULL);
}
//-----------------------//

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
//-----------------------//

//Start WiFi reset task
void wifiResetTask(void *param) {
  DEBUG_PRINTLN("WiFi Reset Task started, resetting WiFi settings...");
  for (;;) {
    leds[0] = CRGB::Green;
    FastLED.show();
    vTaskSuspend(networkTaskHandle);
    vTaskSuspend(mainTaskHandle);
    vTaskDelay(pdMS_TO_TICKS(100));

    wm.resetSettings();
    wm.autoConnect("DMA_MoreChick");
    ESP.restart();

    wifiResetTaskHandle = NULL;  
    vTaskDelete(NULL);
  }
}
//-----------------------//


/*********************************************************************/
/*                               Main Task                           */
/*********************************************************************/

void mainTask(void *param) {
  for (;;) {
    // Get the current time's epoch
    static unsigned long last_hb_send_time = 0;
    if (millis() - last_hb_send_time >= HB_INTERVAL) {
      last_hb_send_time = millis();

      if (client.connected()) {
        char hb_data[50];
        snprintf(hb_data, sizeof(hb_data), "%s,wifi_connected", DEVICE_ID);
        client.publish(mqtt_hb_topic, hb_data);
        DEBUG_PRINTLN("Heartbeat published to MQTT");

        // digitalWrite(LED_PIN, HIGH);
        // vTaskDelay(pdMS_TO_TICKS(1000));
        // digitalWrite(LED_PIN, LOW);
        #if Fast_LED
          leds[0] = CRGB::Blue;
          FastLED.show();
          vTaskDelay(pdMS_TO_TICKS(250));
          leds[0] = CRGB::Black;
          FastLED.show();
          vTaskDelay(pdMS_TO_TICKS(250));
          leds[0] = CRGB::Blue;
          FastLED.show();
          vTaskDelay(pdMS_TO_TICKS(250));
          leds[0] = CRGB::Black;
          FastLED.show();
        #endif
      } else {
        DEBUG_PRINTLN("Failed to publish Heartbeat on MQTT");
      }
    }

    // Send sensor data
    static unsigned long last_data_send_time = 0;
    if ((millis() - last_data_send_time >= DATA_INTERVAL) || (digitalRead(WIFI_RESET_BUTTON_PIN) == LOW)) {
      last_data_send_time = millis();

      // Read ammonia sensor
      float ppm = -1;
      #if MP702
      int sensorValue = analogRead(SENSOR_PIN);
      float sensorVoltage = sensorValue * (3.3 / 4095.0); // ESP32 12-bit ADC
      float Rs = (3.3 - sensorVoltage) * RL / sensorVoltage;
      float ratio = Rs / RL;
      ppm = pow(10, ((log10(ratio) - 0.0) / -0.6)); // Adjust based on sensor curve
      #endif

      // Read temperature & humidity sensor (SXT)
      float temperature = -1, humidity = -1;
      if (sht3x.measure()) {
        temperature = sht3x.temperature();
        humidity = sht3x.humidity();
        sxt_available = true;
      } else {
        DEBUG_PRINTLN("SXT read error");
        sxt_available = false;
      }

      // Read light sensor (GY-302)
      float luxGY302 = readGY302(ADDR_GY302);

      // Format MQTT payload efficiently
      char payload[100]; // Adjust buffer size based on expected max length
      snprintf(payload, sizeof(payload), "%s,%s,%s,%s,%s",
              DEVICE_ID,
              (temperature >= 0) ? String(temperature, 2).c_str() : "N/A",
              (humidity >= 0) ? String(humidity, 2).c_str() : "N/A",
              (ppm >= 0) ? String(ppm, 2).c_str() : "N/A",
              (luxGY302 >= 0) ? String(luxGY302, 2).c_str() : "N/A");

      if(client.connected()){
        DEBUG_PRINTLN("MQTT connected, sending data...");
        client.publish(mqtt_pub_topic, payload);
        DEBUG_PRINTLN("Data sent -> ");
        DEBUG_PRINTLN(payload);
        #if Fast_LED
        leds[0] = CRGB::Green;
        FastLED.show();
        vTaskDelay(pdMS_TO_TICKS(250));
        leds[0] = CRGB::Black;
        FastLED.show();
        vTaskDelay(pdMS_TO_TICKS(250));
        leds[0] = CRGB::Green;
        FastLED.show();
        vTaskDelay(pdMS_TO_TICKS(250));
        leds[0] = CRGB::Black;
        FastLED.show();
      #endif
      } else {
        DEBUG_PRINTLN("MQTT not connected, cannot send data.");
        DEBUG_PRINTLN("Payload was: ");
        DEBUG_PRINTLN(payload);
        #if Fast_LED
          leds[0] = CRGB::DeepPink;
          FastLED.show();
          vTaskDelay(pdMS_TO_TICKS(250));
          leds[0] = CRGB::Black;
          FastLED.show();
        #endif
        // continue; // Skip sending if not connected
      }
      

      // digitalWrite(LED_PIN, HIGH);
      // vTaskDelay(pdMS_TO_TICKS(1000));
      // digitalWrite(LED_PIN, LOW);
      
    }

    // Retry checking SXT sensor at intervals
    static unsigned long last_sxt_check_time = 0;
    if (!sxt_available && millis() - last_sxt_check_time >= SXT_RECHECK_INTERVAL) {
      last_sxt_check_time = millis();
      check_sxt_sensor();
    }

    // Check for WiFi reset button press
    if (digitalRead(WIFI_RESET_BUTTON_PIN) == LOW) {
      unsigned long pressStartTime = millis();
      DEBUG_PRINTLN("Button Pressed....");

      #if Fast_LED
        leds[0] = CRGB::Blue;
        FastLED.show();
      #endif

      while (digitalRead(WIFI_RESET_BUTTON_PIN) == LOW) {
        if (millis() - pressStartTime >= 5000) {
          DEBUG_PRINTLN("5 seconds holding time reached, starting WiFiManager...");
          
          if(wifiResetTaskHandle == NULL) {
            xTaskCreatePinnedToCore(wifiResetTask, "WiFi Reset Task", 8*1024, NULL, 1, &wifiResetTaskHandle, 1);
          }
          else{
            Serial.println("WiFi Reset Task already running.");
          }
          vTaskDelay(pdMS_TO_TICKS(100));
        }
      }
      #if Fast_LED
        leds[0] = CRGB::Black;
        FastLED.show();
      #endif
    }

    vTaskDelay(pdMS_TO_TICKS(100));  // Task delay to prevent CPU overload
  }
}
/*********************************************************************/
//End FreeRTOS Task Section //
//-------------------------//

// Setup function
void setup() {
  // Serial Monitor buad rate
  // pinMode(LED_PIN, OUTPUT);
  // digitalWrite(LED_PIN, HIGH);
  // delay(500);
  // digitalWrite(LED_PIN, LOW);
  // delay(500);
  // digitalWrite(LED_PIN, HIGH);
  // delay(500);
  // digitalWrite(LED_PIN, LOW);

  Serial.begin(115200);

  preferences.begin("device_data", false);  // Open Preferences (NVS)
  static String device_id; // Static variable to persist scope
  
  #if CHANGE_DEVICE_ID
    // Construct new device ID
    device_id = String(WORK_PACKAGE) + GW_TYPE + FIRMWARE_UPDATE_DATE + DEVICE_SERIAL;
    
    // Save device ID to Preferences
    preferences.putString("device_id", device_id);
    Serial.println("Device ID updated in Preferences: " + device_id);
  #else
    // Restore device ID from Preferences
    device_id = preferences.getString("device_id", "UNKNOWN");
    Serial.println("Restored Device ID from Preferences: " + device_id);
  #endif

  DEVICE_ID = device_id.c_str(); // Assign to global pointer

  preferences.end();

  Serial.print("Device ID: ");
  Serial.println(DEVICE_ID);
  delay(1000);

  #if Fast_LED
    FastLED.addLeds<NEOPIXEL, DATA_PIN>(leds, NUM_LEDS);
    leds[0] = CRGB::HotPink;
    FastLED.show();
    vTaskDelay(pdMS_TO_TICKS(1000));

    leds[0] = CRGB::Black;
    FastLED.show();
  #endif

  // Button setup
  pinMode(WIFI_RESET_BUTTON_PIN, INPUT_PULLUP);
  #if MP702
    pinMode(SENSOR_PIN, INPUT);
  #endif

  Wire.begin();
  check_sxt_sensor();
  initBH1750(ADDR_GY302);

  // Set up MQTT client
  client.setServer(mqtt_server, 1883);
  client.setCallback(mqttCallback);

  // Create tasks
  xTaskCreatePinnedToCore(networkTask, "Network Task", 8*1024, NULL, 1, &networkTaskHandle, 0);
  xTaskCreatePinnedToCore(mainTask, "Main Task", 16*1024, NULL, 1, &mainTaskHandle, 1);
}
//-----------------------//

// Loop function
void loop() {
}





#include <config.h>

// Function Prototypes
void reconnectWiFi();
void reconnectMQTT();
void mqttCallback(char* topic, byte* payload, unsigned int length);
void networkTask(void *param);
void mainTask(void *param);
void wifiResetTask(void *param);
void otaTask(void *parameter);
//-----------------------//

// Start Function Section //
//-----------------------//

void writeRegister(uint8_t reg) {
  Wire.beginTransmission(HDC1080_ADDR);
  Wire.write(reg);
  Wire.endTransmission();
}

float readTemperature() {
  writeRegister(0x00);  // Temperature register
  delay(20);            // Wait for conversion (~15ms)
  
  Wire.requestFrom(HDC1080_ADDR, 2);
  uint16_t raw = (Wire.read() << 8) | Wire.read();

  // Convert raw data to Celsius (from datasheet)
  return (raw / 65536.0) * 165.0 - 40.0;
}

float readHumidity() {
  writeRegister(0x01);  // Humidity register
  delay(20);            // Wait for conversion (~15ms)
  
  Wire.requestFrom(HDC1080_ADDR, 2);
  uint16_t raw = (Wire.read() << 8) | Wire.read();

  // Convert raw data to %RH (from datasheet)
  return (raw / 65536.0) * 100.0;
}

float readAmmonia(){
  int adcSum = 0;
  for(int i=0; i<10; i++){
    adcSum += analogRead(AmmoniaSensorPin);
    delay(100);
  }
  int adcValue = adcSum / 10;
  // Serial.print("Ammonia Sensor ADC Value: ");
  // Serial.println(adcValue);

  float voltageL = adcValue * (3.3 / 4095.0); // ESP32 12-bit ADC  
  // Serial.print("RL Voltage: ");
  // Serial.print(voltageL, 3);
  // Serial.println(" V");

  float voltageS = 3.3 - voltageL;
  // Serial.print("Rs Voltage: ");
  // Serial.print(voltageS, 3);
  // Serial.println(" V");

  float Rs = (voltageS * 10000.0) / voltageL; // RL = 10k Ohm
  // Serial.print("Calculated Rs: ");
  // Serial.print(Rs, 2);
  // Serial.println(" Ohm");

  float ratio = Rs / 10000.0; // RL = 10k Ohm
  float ppm = pow(10, ((log10(ratio) + 0.60) / -0.45)); // Adjust based on sensor curve
  // Serial.print("Calculated Ammonia Concentration: ");
  // Serial.print(ppm, 2);
  // Serial.println(" ppm");
  return ppm;
}

float ldrToLux(int adc) {
  // Known calibration points
  const int ADC_vals[5] = {4048, 3800, 2096, 1966, 1600};
  const float Lux_vals[5] = {961, 488, 16.67, 11.67, 10.83};
  
  // If out of range
  if(adc >= ADC_vals[0]) return Lux_vals[0];
  if(adc <= ADC_vals[4]) return Lux_vals[4];
  
  // Find which segment
  for(int i=0; i<4; i++){
    if(adc <= ADC_vals[i] && adc >= ADC_vals[i+1]){
    float log_adc1 = log(ADC_vals[i]);
    float log_adc2 = log(ADC_vals[i+1]);
    float log_lux1 = log(Lux_vals[i]);
    float log_lux2 = log(Lux_vals[i+1]);
    
    float log_adc = log(adc);
    float log_lux = log_lux1 + (log_lux2 - log_lux1) * (log_adc - log_adc1) / (log_adc2 - log_adc1);
    
    return exp(log_lux);  // return interpolated Lux
    }
  }
  return 0; // fallback
}

float readLightIntensity(){
  // Serial.print("LDR Value: ");
  int ldrValueSum = 0;
  for(int i=0; i<10; i++){
    ldrValueSum += analogRead(LDR_PIN);
    delay(100);
  }
  int ldrValue = ldrValueSum / 10;
  // Serial.print(ldrValue);
  // Serial.println();

  float lux = ldrToLux(ldrValue);
  // Serial.print("Calculated Lux: ");
  // Serial.print(lux, 2);
  // Serial.println(" lx");
  return lux;
}

void publishHeartbeat() {
  if (client.connected()) {
    char hb_data[50];
    snprintf(hb_data, sizeof(hb_data), "%s,wifi_connected", DEVICE_ID);
    client.publish(mqtt_hb_topic, hb_data);
    DEBUG_PRINTLN("Heartbeat published to MQTT");

    #ifdef USE_Fast_LED
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

void publishData() {
  float temperature = readTemperature();
  float humidity = readHumidity();
  float ammonia = readAmmonia();
  float lightIntensity = readLightIntensity();

  // Format MQTT payload efficiently
  char payload[100]; // Adjust buffer size based on expected max length
  snprintf(payload, sizeof(payload), "%s,%s,%s,%s,%s",
          DEVICE_ID,
          (temperature >= 0) ? String(temperature, 2).c_str() : "N/A",
          (humidity >= 0) ? String(humidity, 2).c_str() : "N/A",
          (ammonia >= 0) ? String(ammonia, 2).c_str() : "N/A",
          (lightIntensity >= 0) ? String(lightIntensity, 2).c_str() : "N/A");

  if(client.connected()){
    DEBUG_PRINTLN("MQTT connected, sending data...");
    client.publish(mqtt_pub_topic, payload);
    DEBUG_PRINTLN("Data sent -> ");
    DEBUG_PRINTLN(payload);
    #ifdef USE_Fast_LED
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
    #ifdef USE_Fast_LED
      leds[0] = CRGB::DeepPink;
      FastLED.show();
      vTaskDelay(pdMS_TO_TICKS(250));
      leds[0] = CRGB::Black;
      FastLED.show();
    #endif
    // continue; // Skip sending if not connected

  }
}

void publishPingResponse() {
  if (client.connected()) {
    char ping_data[50];
    snprintf(ping_data, sizeof(ping_data), "%s,%s,%s", DEVICE_ID,FIRMWARE_VERSION,FIRMWARE_RELEASE_DATE);
    client.publish(mqtt_ack_topic, ping_data);
    DEBUG_PRINTLN("Heartbeat published to MQTT");

    #ifdef USE_Fast_LED
      leds[0] = CRGB::Blue;
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
    DEBUG_PRINTLN("Failed to publish Heartbeat on MQTT");
  }
}
//-----------------------//


// Function to reconnect to WiFi
void reconnectWiFi() {
  #ifdef USE_Fast_LED
    leds[0] = CRGB::Red;
    FastLED.show();
  #endif

  if (WiFi.status() != WL_CONNECTED) {
    if (wifiAttemptCount > 0) {
      esp_task_wdt_reset();
      DEBUG_PRINTLN("Attempting WiFi connection...");
      WiFi.begin();  // Use saved credentials
      wifiAttemptCount--;
      DEBUG_PRINTLN("Remaining WiFi attempts: " + String(wifiAttemptCount));
      // vTaskDelay(WIFI_ATTEMPT_DELAY / portTICK_PERIOD_MS);
      vTaskDelay(pdMS_TO_TICKS(WIFI_ATTEMPT_DELAY));
    } else if (wifiWaitCount > 0) {
      esp_task_wdt_reset();
      wifiWaitCount--;
      DEBUG_PRINTLN("WiFi wait... retrying in a moment");
      DEBUG_PRINTLN("Remaining WiFi wait time: " + String(wifiWaitCount) + " seconds");
      vTaskDelay(pdMS_TO_TICKS(WIFI_WAIT_DELAY));
    } else {
      esp_task_wdt_reset();
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
    esp_task_wdt_reset();

    #ifdef USE_Fast_LED
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
        
        #ifdef USE_Fast_LED
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
  
  #ifdef USE_Fast_LED
    leds[0] = CRGB::Blue;
    FastLED.show();
    vTaskDelay(pdMS_TO_TICKS(300)); // Short delay to indicate message received
    leds[0] = CRGB::Black;
    FastLED.show();
    vTaskDelay(pdMS_TO_TICKS(200));
  #endif
  
  // Print the topic and message for debugging
  DEBUG_PRINTLN("Message arrived on topic: " + String(topic));
  DEBUG_PRINTLN("Message content: " + message);

  // Handle ping request
  if (message == "ping") {
    publishPingResponse();
    return;
  }

  if( message == "restart") {
    DEBUG_PRINTLN("Received reset command via MQTT, restarting...");
    vTaskDelay(pdMS_TO_TICKS(2000));
    ESP.restart();
  }

  if(message == "heartbeat") {
    publishHeartbeat();
    return;
  }

  if(message == "data") {
    publishData();
    return;
  }

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



// Start FreeRTOS Task Section //
//----------------------//

// Start OTA Task
void otaTask(void *parameter) {
  esp_task_wdt_reset();
  Serial.println("Starting OTA update...");

  #ifdef USE_Fast_LED
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
    esp_task_wdt_reset();
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
    esp_task_wdt_reset();

    leds[0] = CRGB::Green;
    FastLED.show();

    // Suspend other tasks while configuring WiFi
    vTaskSuspend(networkTaskHandle);
    vTaskSuspend(mainTaskHandle);
    vTaskDelay(pdMS_TO_TICKS(100));

    // Reset WiFi settings
    wm.resetSettings();

    // Set timeout for config portal (e.g., 3 minutes)
    wm.setConfigPortalTimeout(180);  // timeout in seconds

    // Start autoConnect with timeout
    if (!wm.autoConnect("DMA_MoreChick")) {
      DEBUG_PRINTLN("WiFi config portal timed out!");
      // Handle fallback, e.g., restart or continue offline
      ESP.restart();
    }

    // If connected successfully
    DEBUG_PRINTLN("WiFi connected!");
    // If WiFi is configured successfully
    DEBUG_PRINTLN("Restarting to apply settings...");
    delay(2000);
    ESP.restart();  // Restart ESP to use new WiFi credentials

    wifiResetTaskHandle = NULL;
    vTaskDelete(NULL); // Delete this task
  }
}
//-----------------------//


/*********************************************************************/
/*                               Main Task                           */
/*********************************************************************/

void mainTask(void *param) {
  for (;;) {
    esp_task_wdt_reset();
    // Get the current time's epoch
    static unsigned long last_hb_send_time = 0;
    if (millis() - last_hb_send_time >= HB_INTERVAL) {
      last_hb_send_time = millis();
      //----------------------------------

      // Publish heartbeat
      publishHeartbeat();

      //----------------------------------
    }

    // Send sensor data
    static unsigned long last_data_send_time = 0;
    if ((millis() - last_data_send_time >= DATA_INTERVAL) || (digitalRead(WIFI_RESET_BUTTON_PIN) == LOW)) {
      last_data_send_time = millis();
      //----------------------------------

      // Publish sensor data
      publishData();
      
      //--------------------------------------------
    }

    // Check for WiFi reset button press
    if (digitalRead(WIFI_RESET_BUTTON_PIN) == LOW) {
      unsigned long pressStartTime = millis();
      DEBUG_PRINTLN("Button Pressed....");

      #ifdef USE_Fast_LED
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
      #ifdef USE_Fast_LED
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

  Serial.begin(115200);

  // Initialize Preferences
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
  // End Preferences

  Serial.print("Device ID: ");
  Serial.println(DEVICE_ID);
  delay(1000);

  #ifdef USE_Fast_LED
    FastLED.addLeds<NEOPIXEL, DATA_PIN>(leds, NUM_LEDS);
    leds[0] = CRGB::HotPink;
    FastLED.show();
    vTaskDelay(pdMS_TO_TICKS(1000));

    leds[0] = CRGB::Black;
    FastLED.show();
  #endif

  // Initialize I2C for HDC1080
  Wire.begin(21, 22);  // SDA, SCL
  delay(100);
  Wire.beginTransmission(HDC1080_ADDR);
  Wire.write(0x02);
  Wire.write(0x10); // Bit7=0 Temp first, Bits[10:8]=000 (14-bit)
  Wire.write(0x00);
  Wire.endTransmission();
  delay(15);
  Serial.println("✅ HDC1080 Initialized!");
  // End I2C Initialization

  // Button setup
  pinMode(WIFI_RESET_BUTTON_PIN, INPUT_PULLUP);


  // Set up MQTT client
  client.setServer(mqtt_server, 1883);
  client.setCallback(mqttCallback);
  client.setKeepAlive(60);
  Serial.println("✅ MQTT Client Initialized!");

  esp_task_wdt_init(60, true);   // 🛡️ 60s timeout for all registered tasks 
  Serial.println("✅ WDT Initialized!");

  // Create tasks
  xTaskCreatePinnedToCore(networkTask, "Network Task", 8*1024, NULL, 1, &networkTaskHandle, 0);
  xTaskCreatePinnedToCore(mainTask, "Main Task", 16*1024, NULL, 1, &mainTaskHandle, 1);
}
//-----------------------//

// Loop function
void loop() {
  vTaskDelay(pdMS_TO_TICKS(100));
}





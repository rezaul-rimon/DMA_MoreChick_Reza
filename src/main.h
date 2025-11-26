#include <Arduino.h>
#include <Wire.h>

#define LDR_PIN 32 // Pin for LDR sensor
#define NTC_PIN 35 // Pin for NTC sensor
#define AmmoniaSensorPin 34 // Pin for Ammonia sensor

// HDC1080 default I2C address
#define HDC1080_ADDR 0x40

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
    int adcValue = analogRead(AmmoniaSensorPin);
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
    int ldrValue = analogRead(LDR_PIN);
    // Serial.print(ldrValue);
    // Serial.println();

    float lux = ldrToLux(ldrValue);
    // Serial.print("Calculated Lux: ");
    // Serial.print(lux, 2);
    // Serial.println(" lx");
    return lux;
}

void setup() {
    Serial.begin(115200);
    Wire.begin(21, 22);  // SDA, SCL
    delay(100);
    pinMode(LDR_PIN, INPUT);
    // pinMode(NTC_PIN, INPUT);
    // pinMode(AmmoniaSensorPin, INPUT);
    
    Serial.println("\n✅ HDC1080 Temperature & Humidity Sensor Test");

    // Configuration register: 14-bit temp + 14-bit humidity
    Wire.beginTransmission(HDC1080_ADDR);
    Wire.write(0x02);
    Wire.write(0x10); // Bit7=0 Temp first, Bits[10:8]=000 (14-bit)
    Wire.write(0x00);
    Wire.endTransmission();
    delay(15);
}

void loop() {
    float temp = readTemperature();
    float hum = readHumidity();

    Serial.print("Temperature: ");
    Serial.print(temp, 2);
    Serial.print(" °C  |  Humidity: ");
    Serial.print(hum, 2);
    Serial.println(" %");

    float lux = readLightIntensity();
    Serial.print("Light Intensity: ");
    Serial.print(lux, 2);
    Serial.println(" lx");

    float ammonia = readAmmonia();
    Serial.print("Ammonia Concentration: ");
    Serial.print(ammonia, 2);
    Serial.println(" ppm");

    Serial.println("--------------------------------------------------");

    

  delay(2000); // Read every 2 seconds
}

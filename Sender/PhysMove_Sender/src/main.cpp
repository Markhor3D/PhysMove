#include <ESP8266WiFi.h>
#include <espnow.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <EEPROM.h>

// Replace with the MAC address of the ESP32 receiver
uint8_t receiverMAC[] = {0xCC, 0xDB, 0xA7, 0x14, 0x3D, 0x48};

Adafruit_MPU6050 mpu;

void sendCallback(uint8_t *mac, uint8_t status) {
    Serial.print("Send Status: ");
    Serial.println(status == 0 ? "Success" : "Failed");
}

void setup() {
    //Serial.begin(115200, SERIAL_8N1, SerialMode::SERIAL_RX_ONLY);
    Serial.begin(115200);
    //pinMode(LED_BUILTIN, OUTPUT);
    // delay(100);
    // EEPROM.begin(64);
    // for (int i = 0; i < 6; i++){
    //   receiverMAC[i] = EEPROM.read(i);
    // }
    WiFi.mode(WIFI_STA);
    Wire.begin(0, 2);


    // Initialize MPU6050
    if (!mpu.begin()) {
        Serial.println("Failed to find MPU6050 chip");
        while(!mpu.begin()) {
          delay(1000);
          Serial.println("I2C Scan");
          for (int i = 1; i< 127; i++){
            Wire.beginTransmission(i);
            if(Wire.endTransmission()){
            }else{
              
              Serial.print("Transmitted at: ");
              Serial.println(i);
              break;
            }
          }
        }
    }
    else
      Serial.println("MPU6050 initialized successfully!");

    // Initialize ESP-NOW
    if (esp_now_init() != 0) {
        Serial.println("ESP-NOW Initialization Failed");
        return;
    }

    // Register peer
    esp_now_set_self_role(ESP_NOW_ROLE_CONTROLLER);
    esp_now_add_peer(receiverMAC, ESP_NOW_ROLE_SLAVE, 1, NULL, 0);

    // Register send callback
    esp_now_register_send_cb(sendCallback);


    // Set sensor range (optional)
    mpu.setAccelerometerRange(MPU6050_RANGE_4_G);
    mpu.setGyroRange(MPU6050_RANGE_2000_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_260_HZ);
    delay(100);

}

#define DATA_SETS 8 // Number of data sets per packet
// Structure to hold a single data set
struct DataSet {
    float accelX, accelY, accelZ;
    float gyroX, gyroY, gyroZ;
    long timestamp;
};
// Array to store multiple data sets
DataSet dataPacket[DATA_SETS];
int currentIndex = 0; // Index to track data in the packet

long lastSample = 0;
bool lastLED = 0;
long lastLEDToggleAt = 0;
void loop() {
  if (millis() - lastSample >= 3){ // a packet every 3ms
    lastSample = millis();
    // Collect sensor data
    sensors_event_t accel, gyro, temp;
    mpu.getEvent(&accel, &gyro, &temp);

    // Add data to the packet
    dataPacket[currentIndex].accelX = accel.acceleration.x;
    dataPacket[currentIndex].accelY = accel.acceleration.y;
    dataPacket[currentIndex].accelZ = accel.acceleration.z;
    dataPacket[currentIndex].gyroX = gyro.gyro.x;
    dataPacket[currentIndex].gyroY = gyro.gyro.y;
    dataPacket[currentIndex].gyroZ = gyro.gyro.z;
    dataPacket[currentIndex].timestamp = millis();
    if (isnan(accel.acceleration.x)
    ||  isnan(accel.acceleration.y)
    ||  isnan(accel.acceleration.z)
    ||  isnan(gyro.gyro.x)
    ||  isnan(gyro.gyro.y)
    ||  isnan(gyro.gyro.z))
    return;
    currentIndex++;
  }
  // Send packet when full
  if (currentIndex == DATA_SETS) {
    if(esp_now_send(NULL, (uint8_t *)dataPacket, sizeof(dataPacket)) == 0){
      //Serial.println("Packet sent!");
    }
    //else
      //Serial.println("Packet NOT sent!");
      

    // Reset index
    currentIndex = 0;
      
  }
}

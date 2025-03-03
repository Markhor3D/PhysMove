#include <WiFi.h>
#include <esp_now.h>


// Structure to match the sender's data structure
struct DataSet {
    float accelX, accelY, accelZ;
    float gyroX, gyroY, gyroZ;
    long timestamp;
};


#define BUFFER_SIZE 8 // Number of data samples in each packet
class DataSource{
    public:
    DataSet dataBuffer[BUFFER_SIZE];
    long lastDataPacketAt = 0;
    long packetTimestamp = 0; // Time when the last packet was received
    bool clientConnected = false;
    uint8_t mac [6];
    bool MacIsSame(const uint8_t* mac_){
        // Serial.print("Incoming Mac: ");
        // for (int macI = 0; macI < 6; macI ++)
        //     Serial.print(mac_[macI], 16);
        // Serial.println();

        
        // Serial.print("This Mac: ");
        // for (int macI = 0; macI < 6; macI ++)
        //     Serial.print(mac[macI], 16);
        // Serial.println();

        for (int i = 0; i < 6; i++){
            if (mac[i] != mac_[i])
                return false;
        }
        return true;
    }
    void feedData(const uint8_t* incomingData, int len){
        //Serial.println("Feed");
        // Store the data in the buffer
        memcpy(dataBuffer, incomingData, len);
    
        // Normalize timestamps to the receiver's clock
        long receiverStartTime = millis(); // Reference time on the receiver
        long senderStartTime = dataBuffer[0].timestamp; // First timestamp from the sender
    
        for (int i = 0; i < BUFFER_SIZE; i++) {
            dataBuffer[i].timestamp = receiverStartTime + (dataBuffer[i].timestamp - senderStartTime);
        }
    
        // Record the timestamp of the received packet
        packetTimestamp = receiverStartTime;
        lastDataPacketAt = millis();
    }
    
    // Function to interpolate data
    DataSet getInterpolatedData(long queryTime) {
        // Determine the buffer's time range
        long bufferStart = dataBuffer[0].timestamp;                // Oldest timestamp in the buffer
        long bufferEnd = dataBuffer[BUFFER_SIZE - 1].timestamp;    // Latest timestamp in the buffer

        // Handle query times outside the buffer range
        if (queryTime <= bufferStart) {
            //Serial.println("Query time is too old! Returning the first sample in the buffer.");
            return dataBuffer[0]; // Return the oldest data as fallback
        }
        if (queryTime >= bufferEnd) {
            //Serial.println("Query time is too new! Returning the latest sample in the buffer.");
            return dataBuffer[BUFFER_SIZE - 1]; // Return the newest data as fallback
        }

        // Find the two data points for interpolation
        int index1 = 0, index2 = 0;
        for (int i = 0; i < BUFFER_SIZE - 1; i++) {
            if (dataBuffer[i].timestamp <= queryTime && dataBuffer[i + 1].timestamp > queryTime) {
                index1 = i;
                index2 = i + 1;
                break;
            }
        }

        // Calculate the interpolation factor
        long t1 = dataBuffer[index1].timestamp;
        long t2 = dataBuffer[index2].timestamp;
        float alpha = (float)(queryTime - t1) / (float)(t2 - t1);
        if(t2 == t1){
        // Serial.println("Time same!!");
        // Serial.print("Now: ");
        // Serial.println(millis());
        // Serial.print("Index 1: ");
        // Serial.println(index1);
        // Serial.print("Index 2: ");
        // Serial.println(index2);
        for (int i = 0; i < 8; i++){
            
            //Relay interpolated data to the second protocol
            // Serial.print(" Data: ");
            // Serial.print(i);
            // Serial.print(" | Time: ");
            // Serial.print(dataBuffer[i].timestamp);
            // Serial.print(" | Accel X: ");
            // Serial.print(dataBuffer[i].accelX);
            // Serial.print(", Y: ");
            // Serial.print(dataBuffer[i].accelY);
            // Serial.print(", Z: ");
            // Serial.println(dataBuffer[i].accelZ);
        }
        }
        // Interpolate between the two data points
        DataSet interpolatedData;
        interpolatedData.accelX = (1 - alpha) * dataBuffer[index1].accelX + alpha * dataBuffer[index2].accelX;
        interpolatedData.accelY = (1 - alpha) * dataBuffer[index1].accelY + alpha * dataBuffer[index2].accelY;
        interpolatedData.accelZ = (1 - alpha) * dataBuffer[index1].accelZ + alpha * dataBuffer[index2].accelZ;
        interpolatedData.gyroX = (1 - alpha) * dataBuffer[index1].gyroX + alpha * dataBuffer[index2].gyroX;
        interpolatedData.gyroY = (1 - alpha) * dataBuffer[index1].gyroY + alpha * dataBuffer[index2].gyroY;
        interpolatedData.gyroZ = (1 - alpha) * dataBuffer[index1].gyroZ + alpha * dataBuffer[index2].gyroZ;
        interpolatedData.timestamp = queryTime;

        return interpolatedData;
    }

};
#define MaxSources 8
DataSource dataSources [MaxSources];
void onDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
     if (len != sizeof(DataSet) * BUFFER_SIZE) {
        Serial.println("Invalid packet size!");
        return;
    }
    int feedTo = -1;
    for (int i =0; i < MaxSources; i++){
        // Serial.print("Incoming Mac: ");
        // for (int macI = 0; macI < 6; macI ++)
        //     Serial.print(mac[macI], 16);
        // Serial.println();
        if (dataSources[i].MacIsSame(mac)){
            feedTo = i;
            //Serial.print("Found client to feed: ");
            //Serial.println(i);
            break;
        }
    }
    if (feedTo < 0) {// register as new client        
        Serial.print("Register new @");
        for (int i = 0; i < MaxSources; i++){
            if (!dataSources[i].clientConnected){
                dataSources[i].clientConnected = true;
                for (int j = 0; j < 6; j++)
                    dataSources[i].mac[j] = mac[j];
                feedTo = i;
                   
                Serial.println(i);
                break;
            }
        }
    }
    if (feedTo >=0) {
        dataSources[feedTo].feedData(incomingData, len);   
    }
    else{
        Serial.println("No client to feed");
    }
}

void setup() {
    Serial.begin(115200);
    Serial2.begin(115200);
    pinMode(LED_BUILTIN, OUTPUT);
    // Print MAC address of this ESP32
    Serial.print("ESP32 MAC Address: ");

    // Set device as a Wi-Fi station
    WiFi.mode(WIFI_STA);
    Serial.println(WiFi.macAddress());

    // Initialize ESP-NOW
    if (esp_now_init() != ESP_OK) {
        Serial.println("ESP-NOW Initialization Failed");
        return;
    }

    // Register callback function to receive data
    esp_now_register_recv_cb(onDataRecv);

    Serial.println("ESP-NOW Receiver Ready");
}

void loop() {
    //if (millis() - lastDataPacketAt > 1000){
        Serial2.print("mac=");
        Serial2.print(WiFi.macAddress());
        Serial2.print(":");
        delay(100);
        digitalWrite(LED_BUILTIN, millis() % 2000 > 1000);

    //}
    //else{
        
        digitalWrite(LED_BUILTIN, millis() % 50 > 25);
        // Example: Simulating a query from the second protocol
        long currentTime = millis();
        if (dataSources[0].clientConnected){
            DataSet interpolatedData = dataSources[0].getInterpolatedData(currentTime);

            // Relay interpolated data to the second protocol
            Serial.print("Interpolated Data | Time: ");
            Serial.print(interpolatedData.timestamp);
            Serial.print(" | Accel X: ");
            Serial.print(interpolatedData.accelX);
            Serial.print(", Y: ");
            Serial.print(interpolatedData.accelY);
            Serial.print(", Z: ");
            Serial.println(interpolatedData.accelZ);
        }
        delay(2); // Adjust as needed for your second protocol's query rate
    //}
}

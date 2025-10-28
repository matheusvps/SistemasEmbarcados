#include <Arduino.h>

// === Pin Definitions ===
#define RXD2 16
#define TXD2 17
#define HC12 Serial2
#define SET_PIN 5

// === Configuration ===
#define BOARD_MODE 2
#define SERIAL_BAUDRATE 115200
#define HC12_BAUDRATE 9600
#define SETUP_DELAY_MS 40
#define POST_SETUP_DELAY_MS 80

// === Data Protocol Constants ===
#define DATA_PACKET_MAGIC 0xAA55
#define DATA_PACKET_SIZE 15

// === Data Structure ===
typedef struct {
    uint16_t magic;         // Magic number for identification
    int16_t ax, ay, az;    // Accelerometer data
    int16_t gx, gy, gz;    // Gyroscope data
    uint8_t checksum;      // Simple checksum for data integrity
} mpu_data_packet_t;

// === Threshold Constants ===
#define IMPACT_THRESHOLD -1.2f
#define BRAKE_THRESHOLD -0.8f
#define TURN_THRESHOLD 200.0f

// === Function Prototypes ===
uint8_t calculateChecksum(const mpu_data_packet_t *packet);
uint8_t receiveDataPacket(mpu_data_packet_t *packet);
float mpu6050_accel_g(int16_t raw);
float mpu6050_gyro_dps(int16_t raw);
void processSensorData(const mpu_data_packet_t *packet);

void setup() {
    // Configure SET pin
    pinMode(SET_PIN, OUTPUT);
    digitalWrite(SET_PIN, LOW);         // Enter AT mode
    delay(SETUP_DELAY_MS);
    
    // Initialize serial communications
    Serial.begin(SERIAL_BAUDRATE);         // Serial port to computer
    HC12.begin(HC12_BAUDRATE, SERIAL_8N1, RXD2, TXD2); // Serial port to HC12
    delay(POST_SETUP_DELAY_MS);
    
    // Exit AT mode
    digitalWrite(SET_PIN, HIGH);        // Normal operation mode
    delay(POST_SETUP_DELAY_MS);
    
    Serial.println("ESP32 initialized - waiting for binary data packets...");
}

void loop() {
    if (BOARD_MODE == 1) {
        // Generator mode (if needed)
        unsigned long time = millis();
        char str[20];
        float val1 = sin(time * 0.1);
        float val2 = cos(time * 0.1);
        sprintf(str, "%.2f,%.2f\n", val1, val2);
        HC12.write(str);
    }
    else if (BOARD_MODE == 2) {
        // Receiver mode - process binary data packets
        mpu_data_packet_t packet;
        
        if (receiveDataPacket(&packet)) {
            // Valid packet received
            processSensorData(&packet);
        } else {
            // Invalid packet or timeout
            Serial.println("Invalid packet received");
        }
    }
}

uint8_t calculateChecksum(const mpu_data_packet_t *packet) {
    uint8_t checksum = 0;
    const uint8_t *data = (const uint8_t*)packet;
    
    // Calculate checksum for all bytes except the checksum field itself
    for (int i = 0; i < sizeof(mpu_data_packet_t) - 1; i++) {
        checksum ^= data[i];
    }
    return checksum;
}

uint8_t receiveDataPacket(mpu_data_packet_t *packet) {
    uint8_t *data = (uint8_t*)packet;
    
    // Wait for data to be available
    if (HC12.available() < sizeof(mpu_data_packet_t)) {
        return 0; // Not enough data available
    }
    
    // Receive all bytes of the packet
    for (int i = 0; i < sizeof(mpu_data_packet_t); i++) {
        data[i] = HC12.read();
    }
    
    // Verify magic number
    if (packet->magic != DATA_PACKET_MAGIC) {
        return 0; // Invalid packet
    }
    
    // Verify checksum
    uint8_t calculated_checksum = calculateChecksum(packet);
    if (packet->checksum != calculated_checksum) {
        return 0; // Checksum error
    }
    
    return 1; // Success
}

float mpu6050_accel_g(int16_t raw) {
    return raw / 16384.0f;
}

float mpu6050_gyro_dps(int16_t raw) {
    return raw / 131.0f;
}

void processSensorData(const mpu_data_packet_t *packet) {
    // Convert raw data to physical units
    float ax = mpu6050_accel_g(packet->ax);
    float ay = mpu6050_accel_g(packet->ay);
    float az = mpu6050_accel_g(packet->az);
    
    float gx = mpu6050_gyro_dps(packet->gx);
    float gy = mpu6050_gyro_dps(packet->gy);
    float gz = mpu6050_gyro_dps(packet->gz);
    
    // Output accelerometer data
    Serial.print(ax);
    Serial.print(",");
    Serial.print(ay);
    Serial.print(",");
    Serial.print(az);
    Serial.print("\n");
    
    // Detect events based on thresholds
    if (ay < IMPACT_THRESHOLD) {
        Serial.println("BATIDA");
    }
    else if (ay < BRAKE_THRESHOLD) {
        Serial.println("FREADA BRUSCA");
    }
    else if (fabs(gz) > TURN_THRESHOLD) {
        Serial.println("CURVA ACENTUADA");
    }
}

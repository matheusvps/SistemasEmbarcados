#include <Arduino.h>
#define RXD2 16	//(RX2)
#define TXD2 17	//(TX2)
#define HC12 Serial2 //Hardware serial 2 on the ESP32

#define BOARD 2

char buffer[64];

void getline(char * buffer) {
    uint8_t idx = 0;
    char c;
    do {
        while (HC12.available() == 0) ; // wait for a char this causes the blocking
        c = HC12.read();
        buffer[idx++] = c;
    } while (c != '\n' && c != '\r'); 
    buffer[idx] = 0;
}

float mpu6050_accel_g(int raw) {
    return raw / 16384.0f;
}

float mpu6050_gyro_dps(int16_t raw) {
  return raw / 131.0f;
}

void setup() {
    pinMode(5, OUTPUT);
    digitalWrite(5, LOW);         //Normally HIGH, LOW for settings
    delay(40);
    Serial.begin(115200);         // Serial port to computer
    HC12.begin(9600, SERIAL_8N1, RXD2, TXD2); // Serial port to HC12
    delay(80);
    digitalWrite(5, HIGH);        //Normally HIGH, LOW for settings
    delay(80);
}

void loop() {
    unsigned long time = 0;
    while (BOARD == 1) {
        char str[20];
        time = millis();
        float val1 = sin(time * 0.1);
        float val2 = cos(time * 0.1);
        sprintf(str, "%.2f,%.2f\n", val1, val2);
        HC12.write(str);
    }
    while (BOARD == 2) {
        getline(buffer);

        int rax = 0;
        int ray = 0;
        int raz = 0;
        int rgx = 0;
        int rgy = 0;
        int rgz = 0;

        sscanf(buffer, "%d,%d,%d,%d,%d,%d\n", &rax, &ray, &raz, &rgx, &rgy, &rgz);

        float ax = mpu6050_accel_g(rax);
        float ay = mpu6050_accel_g(ray);
        float az = mpu6050_accel_g(raz);

        float gx = mpu6050_gyro_dps(rgx);
        float gy = mpu6050_gyro_dps(rgy);
        float gz = mpu6050_gyro_dps(rgz);

        // Serial.print(ax);
        // Serial.print(",");
        // Serial.print(ay);
        // Serial.print(",");
        // Serial.print(az);
        // Serial.print(",");
        Serial.print(gx);
        Serial.print(",");
        Serial.print(gy);
        Serial.print(",");
        Serial.print(gz);
        Serial.print(",");
        Serial.print("\n");
    }
}

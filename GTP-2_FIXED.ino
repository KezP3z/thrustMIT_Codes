#include <SPI.h>
#include <SD.h>
#include <HardwareSerial.h>

HardwareSerial loraSerial(2);
const int SD_CS = 5;

float vel_x=0;
float vel_y=0;
float vel_z=0;
float smoothedValue = 0;
float smoothingFactor = 0.1;
unsigned long lastTime;
bool isApogee = false;

int d_MOS_pin = 1;
int m_MOS_pin = 2;

float pressure_initial=0;
float sum_p=0;
float sum_T=0;
int ctr = 0;

const int windowSize = 10;


File all_Data;

float seaLevelPressure = 1013.25;

#define P_CS_PIN 53  // Chip Select pin for LPS22HB

// LPS22HB Register Addresses
#define P_WHO_AM_I 0x0F
#define P_CTRL_REG1 0x10
#define PRESS_OUT_XL 0x28
#define TEMP_OUT_L 0x2B

#define IMU_CS_PIN 52

#define IMU_WHO_AM_I_REG 0x0F
#define IMU_LSM6DSO32_WHO_AM_I 0x6C

#define IMU_CTRL1_XL 0x10 //ODR selection and scaling selection for acceleration
#define IMU_CTRL2_G 0x11 //ODR selection and scaling selection for acceleration
#define IMU_CTRL6_C 0x15 //
#define IMU_CTRL7_G 0x16

#define IMU_OUTX_L_G 0x22 // gyroscope output regs
#define IMU_OUTX_H_G 0x23
#define IMU_OUTY_L_G 0x24
#define IMU_OUTY_H_G 0x25
#define IMU_OUTZ_L_G 0x26
#define IMU_OUTZ_H_G 0x27

#define IMU_OUTX_L_A 0x28 // acceleration output regs
#define IMU_OUTX_H_A 0x29
#define IMU_OUTY_L_A 0x30
#define IMU_OUTY_H_A 0x31
#define IMU_OUTZ_L_A 0x32
#define IMU_OUTZ_H_A 0x33

void setup() {
  if (readRegister(IMU_WHO_AM_I_REG) == IMU_LSM6DSO32_WHO_AM_I) {
    Serial.println("LSM6DSO32 Found!");
  } else {
    Serial.println("LSM6DSO32 not detected. Check wiring.");
    while (1);
  }

   // Check WHO_AM_I register (should return 0xB1 for LPS22HB)
  uint8_t whoami = SPI_ReadRegister(P_WHO_AM_I);
  Serial.print("WHO_AM_I: 0x");
  Serial.println(whoami, HEX);
  if (whoami != 0xB1) {
      Serial.println("LPS22HB not detected!");
      while (1);
  }

  // Configure LPS22HB (CTRL_REG1)
  writeRegister(P_CTRL_REG1, 0x50);  // Enable continuous mode, ODR = 10 Hz

  SPI.setDataMode(SPI_MODE3);  // LPS22HB works in SPI mode 3
  SPI.setClockDivider(SPI_CLOCK_DIV16);
  SPI.setBitOrder(MSBFIRST);

  pinMode(10,OUTPUT);
  
  pinMode(P_CS_PIN, OUTPUT);
  pinMode(IMU_CS_PIN, OUTPUT);
  pinMode(d_MOS_pin,OUTPUT);
  pinMode(m_MOS_pin,OUTPUT);
  digitalWrite(P_CS_PIN, HIGH);
  digitalWrite(IMU_CS_PIN, HIGH); // Deselect the sensor

  SPI.begin();
  Serial.begin(115200);

  writeRegister(IMU_CTRL7_G, 0x82); // gyroscope enable normal mode
  writeRegister(IMU_CTRL6_C, 0x90); // acceleration enable normal mode
  writeRegister(IMU_CTRL2_G, 0x4C);
  writeRegister(IMU_CTRL1_XL, 0x4C);

  Serial.begin(115200);
    loraSerial.begin(9600, SERIAL_8N1, 16, 17); // Using UART2

    delay(1000); // Allow module to initialize

    // Configure LoRa Module
    loraSerial.println("AT+ADDRESS=1");
    delay(100);
    loraSerial.println("AT+BAND=490000000");
    delay(100);
    loraSerial.println("AT+NETWORKID=18");
    delay(100);
    loraSerial.println("AT+PARAMETER=12,7,1,4");
    delay(1000);
    all_Data =  SD.open("FML.txt", FILE_WRITE);
}

void writeRegister(byte reg, byte value) {
  digitalWrite(IMU_CS_PIN, LOW); // Select module
  SPI.transfer(reg & 0x7F);         // Write command (MSB=0 for write)
  SPI.transfer(value);
  digitalWrite(IMU_CS_PIN, HIGH); // Deselect module
}

uint8_t SPI_ReadRegister(uint8_t reg) {
    digitalWrite(P_CS_PIN, LOW);
    SPI.transfer(reg | 0x80);  // Read operation (MSB = 1)
    uint8_t value = SPI.transfer(0x00);
    digitalWrite(P_CS_PIN, HIGH);
    return value;
}

void SPI_ReadRegisters(uint8_t reg, uint8_t *buffer, uint8_t len) {
    digitalWrite(P_CS_PIN, LOW);
    SPI.transfer(reg | 0x80);  // Read operation
    for (uint8_t i = 0; i < len; i++) {
        buffer[i] = SPI.transfer(0x00);
    }
    digitalWrite(P_CS_PIN, HIGH);
}


byte readRegister(byte reg) {
  digitalWrite(IMU_CS_PIN, LOW); // Select BMP388
  SPI.transfer(reg | 0x80);         // Read command (MSB=1 for read)
  byte value = SPI.transfer(0x00);  // Send dummy byte to read
  digitalWrite(IMU_CS_PIN, HIGH); // Deselect BMP388
  return value;
}

int8_t readInt8(byte reg) {
  return (int8_t)readRegister(reg);
}

int16_t readInt16(byte reg1, byte reg2) {
  int8_t data1 = readInt8(reg1) << 8; 
  int16_t data = data1 | readInt8(reg2);
  return data;
}

float smoothie(int16_t newReading) {
  smoothedValue = (smoothingFactor * newReading) + ((1 - smoothingFactor) * smoothedValue);
  return smoothedValue;
}

void loop() {
  unsigned long currentTime = 0;
  currentTime = millis(); 
  float deltaTime = (currentTime - lastTime)/1000;
  lastTime = currentTime;
  float prev_vel_y = 0;

  int16_t gyro_x = readInt16(IMU_OUTX_L_G,IMU_OUTX_H_G);
  gyro_x /= 1000.0;
  int16_t gyro_y = readInt16(IMU_OUTY_L_G,IMU_OUTY_H_G);
  gyro_y /= 1000.0;
  int16_t gyro_z = readInt16(IMU_OUTZ_L_G,IMU_OUTZ_H_G);
  gyro_z /= 1000.0;

  int16_t acc_x = readInt16(IMU_OUTX_L_A,IMU_OUTX_H_A);
  acc_x /= 1000.0;
  int16_t acc_y = readInt16(IMU_OUTY_L_A,IMU_OUTY_H_A);
  acc_y /= 1000.0;
  int16_t acc_z = readInt16(IMU_OUTZ_L_A,IMU_OUTZ_H_A);
  acc_z /= 1000.0;

  float s_gyro_x = smoothie(gyro_x);
  float s_gyro_y = smoothie(gyro_y);
  float s_gyro_z = smoothie(gyro_z);
  float s_acc_x = smoothie(acc_x);
  float s_acc_y = smoothie(acc_y);
  float s_acc_z = smoothie(acc_z);

  vel_y += s_acc_y * deltaTime;

 

  Serial.println("gyro_x: ");
  Serial.print(s_gyro_x, 2);
  Serial.println("gyro_y: ");
  Serial.print(s_gyro_y, 2);
  Serial.println("gyro_z: ");
  Serial.print(s_gyro_z, 2);
    
  Serial.println("acc_x: ");
  Serial.print(s_acc_x, 2);
  Serial.println("acc_y: ");
  Serial.print(s_acc_y, 2);
  Serial.println("acc_z: ");
  Serial.print(s_acc_z, 2);

  Serial.println("vel_y: ");
  Serial.print(vel_y, 2);

  delay(1000);

  float pavg;
  float tavg;
  uint8_t pressure_data[3];
  uint8_t temp_data[2];


  // Read pressure (3 bytes)
  SPI_ReadRegisters(PRESS_OUT_XL, pressure_data, 3);
  int32_t raw_pressure = (pressure_data[2] << 16) | (pressure_data[1] << 8) | pressure_data[0];
  float pressure_hPa = raw_pressure / 4096.0;  // Convert to hPa
  sum_p+=pressure_hPa;
  ctr++;

  if(ctr==windowSize){
    pavg=sum_p/ctr;
    sum_p=0;
  }

  // Read temperature (2 bytes)
  SPI_ReadRegisters(TEMP_OUT_L, temp_data, 2);
  int16_t raw_temp = (temp_data[1] << 8) | temp_data[0];
  float temperature_C = raw_temp / 100.0;  // Convert to °C
  sum_T+=temperature_C;
  if (ctr==windowSize){
    tavg=sum_T/ctr;
    sum_T=0;
    ctr=0;
  }

  float raw_alti = 44330 * (1.0 - pow(pavg / seaLevelPressure, 0.1903));
  float alti = 2*raw_alti*tavg/(15.15 + tavg); //https://forum.arduino.cc/t/barometric-pressure-to-altitude/297866
  float Falti = alti *3.28084;

  float sensorArray[] = {pavg, tavg, s_acc_x, s_acc_y, s_acc_z, s_gyro_x, s_gyro_y, s_gyro_z, vel_y, Falti};
  int size= sizeof(sensorArray) / sizeof(sensorArray[0]);
    for (int i = 0; i <  size; i++) {
        char dataBuffer[100];
        snprintf(dataBuffer, sizeof(dataBuffer), "%.2f",sensorArray[i]);
        // Send data via LoRa
        loraSerial.print("AT+SEND=2,");
        loraSerial.print(strlen(dataBuffer));
        loraSerial.print(",");
        loraSerial.print(dataBuffer);
        loraSerial.println();

        Serial.print("Sent: ");
        Serial.println(dataBuffer);

        // Wait for ACK (if needed)
        unsigned long startTime = millis();
        bool ackReceived = false;
        while (millis() - startTime < 2000) { // Wait for 2 seconds for an ACK
            if (loraSerial.available()) {
                String response = loraSerial.readStringUntil('\n');
                if (response.indexOf("+OK") != -1) {
                    ackReceived = true;
                    break;
                }
            }
        }

        if (!ackReceived) {
            Serial.println("ACK not received! Resending...");
            i--; // Retry the same data
        } else {
            Serial.println("ACK received!");
        }

        delay(2000); // Send next data after 2 seconds
    }

  // Print results
  Serial.print("Pressure: (in hPa) ");
  Serial.print(pavg);
  Serial.print("Temperature: (in degree C) ");
  Serial.print(tavg);

  float previousSmoothedPressure = 0;
  
  delay(500);

  all_Data.println(gyro_x);
  all_Data.print(" , ");
  all_Data.print(gyro_y);
  all_Data.print(" , ");
  all_Data.print(gyro_z);
  all_Data.print(" , ");
  all_Data.print(acc_x);
  all_Data.print(" , ");
  all_Data.print(acc_y);
  all_Data.print(" , ");
  all_Data.print(acc_z);
  all_Data.print(" , ");
  all_Data.print(vel_y);
  all_Data.print(" , ");
  all_Data.print(pavg);
  all_Data.print(" , ");
  all_Data.print(tavg);

  prev_vel_y = vel_y;

  if (previousSmoothedPressure < pavg) {
     Serial.println("Apogee detetcted at");
     Serial.print(Falti);
     digitalWrite(d_MOS_pin,HIGH);
     delay(2000);
     digitalWrite(d_MOS_pin,LOW);
     isApogee = true;
  }

  previousSmoothedPressure = pavg;

  if (Falti > 1485 && Falti < 1515 && isApogee == true ) {
    digitalWrite(m_MOS_pin,HIGH);
    delay(2000);
    digitalWrite(m_MOS_pin,LOW);
  }
}
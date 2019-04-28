#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <driver/adc.h>
#include "esp-nowmesh.h"

/*
MotionDetector
=====================================

*NOTE: The registers used in this have put my mpu6050 to a permanant interrupt
* sleep I cannot recover it from.  It will still work in interrupt mode but nothing else.

*Note2: There is a bug in the WiFi library that does not let the esp deep sleep after wifi is turned on.
* see for more details: https://esp32.com/viewtopic.php?f=19&t=8392
* " For now you can add a call 'adc_power_off();' (from driver/adc.h) right before entering deep sleep."

Wiring
======
MPU6050    GY-521(MPU6050)
3.3V       VCC ( GY-521 has onboard regulator 5v tolerant)
GND        GND
21         SDA
22         SCL
34          INT - Note below on allowed RTC IO Pins
*/


/*
Current Usage
=====================================
// 5.37mA with GY-521 LED attached
// 3.96mA with Led Broken/removed, Temp and Gyro still active, high speed parsing
// 0.71mA with Broken LED, Gyro's disabled
// 0.65mA with Broken LED, Gyro's disabled, Temp sensor disabled
*/

/*
Deep Sleep with External Wake Up
=====================================
NOTE:
======
Only RTC IO can be used as a source for external wake
source. They are pins: 0,2,4,12-15,25-27,32-39.
*/

// MPU registers
#define SIGNAL_PATH_RESET  0x68
#define ACCEL_CONFIG       0x1C
#define MOT_THR            0x1F  // Motion detection threshold bits [7:0]
#define MOT_DUR            0x20  // This seems wrong // Duration counter threshold for motion interrupt generation, 1 kHz rate, LSB = 1 ms
#define MOT_DETECT_CTRL    0x69
#define INT_PIN_CFG        0x37
#define INT_ENABLE         0x38
#define INT_STATUS         0x3A
#define PWR_MGMT_1         0x6B
#define PWR_MGMT_2         0x6C
#define INT_STATUS 0x3A
#define MPU6050_ADDRESS 0x68 //AD0 is 0
#define WHO_AM_I           0x75

/*    Example for using write byte
      Configure the accelerometer for self-test
      writeByte(MPU6050_ADDRESS, ACCEL_CONFIG, 0xF0); // Enable self test on all three axes and set accelerometer range to +/- 8 g */
void writeByte(uint8_t address, uint8_t subAddress, uint8_t data)
{
  Wire.begin(21, 22);
  Wire.beginTransmission(address);  // Initialize the Tx buffer
  Wire.write(subAddress);           // Put slave register address in Tx buffer
  Wire.write(data);                 // Put data in Tx buffer
  Wire.endTransmission();           // Send the Tx buffer
}

//example showing using readbytev   ----    readByte(MPU6050_ADDRESS, GYRO_CONFIG);
uint8_t readByte(uint8_t address, uint8_t subAddress)
{
  uint8_t data;                            // `data` will store the register data
  Wire.beginTransmission(address);         // Initialize the Tx buffer
  Wire.write(subAddress);                  // Put slave register address in Tx buffer
  Wire.endTransmission(false);             // Send the Tx buffer, but send a restart to keep connection alive
  Wire.requestFrom(address, (uint8_t) 1);  // Read one byte from slave register address
  data = Wire.read();                      // Fill Rx buffer with result
  return data;                             // Return data read from slave register
}

void setup(){
//  Serial.begin(115200);
//  delay(50);
//
//  Serial.println("UP");
  writeByte( MPU6050_ADDRESS, PWR_MGMT_1, 0b00001000); // Cycle & disable TEMP SENSOR
  writeByte( MPU6050_ADDRESS, PWR_MGMT_2, 0b11000111); // Disable Gyros, 40MHz sample rate
  writeByte( MPU6050_ADDRESS, MOT_THR, 1);  //Write the desired Motion threshold to register 0x1F (For example, write decimal 20).
  writeByte( MPU6050_ADDRESS, MOT_DUR, 1);  //Set motion detect duration to 1  ms; LSB is 1 ms @ 1 kHz rate
  writeByte( MPU6050_ADDRESS, MOT_DETECT_CTRL, 0b00000101); //to register 0x69, write the motion detection decrement and a few other settings (for example write 0x15 to set both free-fall and motion decrements to 1 and accelerometer start-up delay to 5ms total by adding 1ms. )
  writeByte( MPU6050_ADDRESS, INT_PIN_CFG, 0b00100000 ); // now INT pin is active high, cleared on read
  writeByte( MPU6050_ADDRESS, INT_ENABLE, 0b01000000 ); //write register 0x38, bit 6 (0x40), to enable motion detection interrupt.
  readByte(MPU6050_ADDRESS, INT_STATUS);

  NowMesh.begin(6, true); //Channel = 6, SendOnly = true
  NowMesh.send("FF:FF:FF:FF:FF:FF", (uint8_t*)"ALARM ACTIVATED", 16);
  delay(2);

  
//  Serial.println("SLEEPING");
  WiFi.mode(WIFI_OFF);
  adc_power_off();  // adc power off disables wifi entirely, upstream bug

  esp_sleep_enable_ext1_wakeup((uint64_t)(1<<4),ESP_EXT1_WAKEUP_ANY_HIGH);
  esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_SLOW_MEM, ESP_PD_OPTION_OFF);
  esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_FAST_MEM, ESP_PD_OPTION_OFF);
  esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_OFF);
  esp_deep_sleep_start();
  
  //unless the deep sleep code above is commented this won't run
  pinMode(2, OUTPUT);
  pinMode(0, INPUT);

  Serial.printf("0x%02X: %02X\r\n", WHO_AM_I, readByte(MPU6050_ADDRESS, WHO_AM_I));
  Serial.printf("0x%02X: %02X\r\n", INT_ENABLE, readByte(MPU6050_ADDRESS, INT_ENABLE));
  Serial.printf("0x%02X: %02X\r\n", INT_PIN_CFG, readByte(MPU6050_ADDRESS, INT_PIN_CFG));
  Serial.printf("0x%02X: %02X\r\n", MOT_THR, readByte(MPU6050_ADDRESS, MOT_THR));
  Serial.printf("0x%02X: %02X\r\n", MOT_DUR, readByte(MPU6050_ADDRESS, MOT_DUR));
  Serial.printf("0x%02X: %02X\r\n", 0x6B, readByte(MPU6050_ADDRESS, 0x6B));
  Serial.printf("0x%02X: %02X\r\n", 0x6C, readByte(MPU6050_ADDRESS, 0x6C));
  Serial.printf("0x%02X: %02X\r\n", INT_STATUS, readByte(MPU6050_ADDRESS, INT_STATUS));
}

void loop(){
  digitalWrite(2, digitalRead(4));
  if (!digitalRead(0)) readByte(MPU6050_ADDRESS, INT_STATUS);
}

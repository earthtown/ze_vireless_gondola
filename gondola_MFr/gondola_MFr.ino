/*
 * This is merely a proof of concept for the hsardware setup.
 * Protortype haredware setup that is, for the long awaited wireless gondola.
 */

///////////////////////// display bits ////////////////////
#include <U8g2lib.h>
#include <Wire.h>
U8G2_SSD1306_128X64_NONAME_1_SW_I2C u8g2(U8G2_R1, 2,   0,   U8X8_PIN_NONE);
                                      //(U8G2_R0, SCL, SDA, U8X8_PIN_NONE);

///////////////////////// mpu6050 bits ////////////////////
#include "I2Cdev.h"
#include "MPU6050.h"
MPU6050 accelgyro;
int16_t ax, ay, az;
int16_t gx, gy, gz;
#define OUTPUT_READABLE_ACCELGYRO
#define LED_PIN LED_BUILTIN
bool blinkState = false;

///////////////////////// servo bits ////////////////////
#include <Servo.h>
Servo myservo;  // create servo object to control a servo
int penPin = 14;
int penState = 0;

///////////////////////// S      T     P ////////////////////
/////////////////////////     E     U    ////////////////////
void setup() {
  myservo.attach(5);
  pinMode(penPin, INPUT);

  // gotta specificitizingness teh software serial spins
  Wire.pins(0,2);
  Wire.begin();
 
  Serial.begin(115200);

  // initialize device
  Serial.println("Initializing I2C devices...");
  accelgyro.initialize();

  // verify connection
  Serial.println("Testing device connections...");
  Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");
  pinMode(LED_PIN, OUTPUT);
  u8g2.begin();


  //// OLED ////
  u8g2.firstPage();
  do {
    u8g2.setFont(u8g2_font_ncenB10_tr);
    u8g2.drawStr(5,15,"gocupi");
    u8g2.drawStr(0,35,"gondola");
    u8g2.setFont(u8g2_font_ncenB08_tr);
    u8g2.drawStr(2,55,"DISPLAY");
  } while ( u8g2.nextPage() );  

}

void loop() {

//// SERVO ////  
  penState = digitalRead(penPin);
  if (penState == HIGH){
    myservo.write(45);
  } else {
    myservo.write(100);
  }
  
//// MPU6050 ////
  // read raw accel/gyro measurements from device
  accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  // these methods (and a few others) are also available
  //accelgyro.getAcceleration(&ax, &ay, &az);
  //accelgyro.getRotation(&gx, &gy, &gz);

  #ifdef OUTPUT_READABLE_ACCELGYRO
      // display tab-separated accel/gyro x/y/z values
      Serial.print("a/g:\t");
      Serial.print(ax); Serial.print("\t");
      Serial.print(ay); Serial.print("\t");
      Serial.print(az); Serial.print("\t");
      Serial.print(gx); Serial.print("\t");
      Serial.print(gy); Serial.print("\t");
      Serial.println(gz);
  #endif

  #ifdef OUTPUT_BINARY_ACCELGYRO
      Serial.write((uint8_t)(ax >> 8)); Serial.write((uint8_t)(ax & 0xFF));
      Serial.write((uint8_t)(ay >> 8)); Serial.write((uint8_t)(ay & 0xFF));
      Serial.write((uint8_t)(az >> 8)); Serial.write((uint8_t)(az & 0xFF));
      Serial.write((uint8_t)(gx >> 8)); Serial.write((uint8_t)(gx & 0xFF));
      Serial.write((uint8_t)(gy >> 8)); Serial.write((uint8_t)(gy & 0xFF));
      Serial.write((uint8_t)(gz >> 8)); Serial.write((uint8_t)(gz & 0xFF));
  #endif

  // blink LED to indicate activity
  blinkState = !blinkState;
  digitalWrite(LED_PIN, blinkState);

}
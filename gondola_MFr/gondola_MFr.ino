/*
 * This is merely a proof of concept for the hsardware setup.
 * Protortype haredware setup that is, for the long awaited wireless gondola.
 */
///////////////////////// wifi bits ////////////////////
#include <ESP8266WiFi.h>
const char* ssid = "SSID";
const char* password = "PASSWORD";
int penState = 0;
WiFiServer server(80);
void client_interface(void);

///////////////////////// display bits ////////////////////
#include <U8x8lib.h>
#include <Wire.h>
U8X8_SSD1306_128X64_NONAME_SW_I2C u8x8(/* clock=*/ 2, /* data=*/ 0, /* reset=*/ U8X8_PIN_NONE); 

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

///////////////////////// S      T     P ////////////////////
/////////////////////////     E     U    ////////////////////
void setup() {
  
  // connect to network
  Serial.begin(115200);
  Serial.print("Connecting to ");
  Serial.println(ssid);
 
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
  delay(500);
  Serial.print(".");
  }
  Serial.println("");
  Serial.print("WiFi connected to: ");
  Serial.println(ssid);
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());
 
  // Start the server
  server.begin();

  // assign servo
  myservo.attach(4);
  pinMode(penPin, INPUT);

  // gotta specificitizingness teh software serial spins
  Wire.pins(0,2);
  Wire.begin();

  // initialize device
  Serial.println("Initializing I2C devices...");
  accelgyro.initialize();

  // verify connection
  Serial.println("Testing device connections...");
  Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");
  pinMode(LED_PIN, OUTPUT);

  //// OLED ////
  u8x8.begin();
  u8x8.clear();
  u8x8.setFont(u8x8_font_5x7_f);
  u8x8.setCursor(0,0);
  u8x8.println("gocupi gondola");
  u8x8.println("");
  u8x8.println("Network:");
  u8x8.println(ssid);
  u8x8.println("");
  u8x8.println("IP:");
  u8x8.println(WiFi.localIP());
}

void loop() {

//// WEB CLIENT ////
  client_interface();
  
//// SERVO ////  
  //penState = digitalRead(penPin);
  if (penState == HIGH){
    myservo.write(10);
  } else {
    myservo.write(150);
  }
  
//// MPU6050 ////
  // read raw accel/gyro measurements from device
  accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  // these methods (and a few others) are also available
  //accelgyro.getAcceleration(&ax, &ay, &az);
  //accelgyro.getRotation(&gx, &gy, &gz);
/*
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
*/
}

void client_interface() {
  // Check if a client has connected
  WiFiClient client = server.available();
  if (!client) {
    return;
  }
 
  // Wait until the client sends some data
  Serial.println("new client");
  while(!client.available()){
    delay(1);
  }
 
  // Read the first line of the request
  String request = client.readStringUntil('\r');
  Serial.println(request);
  client.flush();
 
  // Match the request
 
  int value = LOW;
  if (request.indexOf("/PEN=UP") != -1)  {
    penState = 1;
    value = HIGH;
  }
  if (request.indexOf("/PEN=DOWN") != -1)  {
    penState = 0;
    value = LOW;
  }
 
// Set ledPin according to the request
//digitalWrite(ledPin, value);
 
  // Return the response
  client.println("HTTP/1.1 200 OK");
  client.println("Content-Type: text/html");
  client.println(""); //  do not forget this one
  client.println("<!DOCTYPE HTML>");
  client.println("<html>");
 
  client.print("Pen is now: ");
 
  if(value == HIGH) {
    client.print("Up");
  } else {
    client.print("Down");
  }
  client.println("<br><br>");
  client.println("<a href=\"/PEN=UP\"\"><button>Lift Pen </button></a>");
  client.println("<a href=\"/PEN=DOWN\"\"><button>Lower Pen </button></a><br />");  
  client.println("</html>");
 
  delay(1);
  Serial.println("Client disonnected");
  Serial.println("");
 
}
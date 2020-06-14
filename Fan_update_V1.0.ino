/*
Project:  Fan update with screen, 3 speed  
          BME280 Sensor data send over MQTT with a ESP8266 / NodeMCU
Author:   Thomas Edlinger for www.edistechlab.com
Date:     Created 30.05.2020
Version:  V1.0
 
- Used Aduino IDE V1.8.12
Required libraries (sketch -> include library -> manage libraries)
 - PubSubClient by Nick O'Leary V2.7.0  
 - Adafruit BME280 Library V2.0.1
 - Adafruit Unified Sensor V1.1.2
 - Adafruit SSD1306 V2.2.1
 - Adafruit GFX Library V1.8.3
 - ArduinoOTA by Juraj Andrassy V1.0.3
Required Board (Tools -> Board -> Boards Manager...)
 - Board: esp8266 by ESP8266 Community V2.7.1

Wirering for the Sensor:
-BME280     NodeMCU
SCL         D1 / GPIO5
SDA         D2 / GPIO4
-Relay      
CH1         D5 / GPIO14   
CH2         D6 / GPIO12
CH3         D7 / GPIO13
-Switch     RX / GPIO3
*/

#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <ArduinoOTA.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_GFX.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

#define wifi_ssid "Your_SSID"  //#define wifi_ssid "your_SSID"
#define wifi_password "Your_Password"  //#define wifi_password "your_Password"

#define mqtt_server "homeassistant.local"  // or your IP adress from the Home-Server
#define mqtt_user "your_User"         
#define mqtt_password "Your_Password"       

#define ESPHostname "FAN"

#define humidity_topic "fan/humidity"
#define temperature_topic "fan/temperature"
#define pressure_topic "fan/pressure"
#define fanStatus_topic "fan/status" 
#define fanSpeed_topic "fan/speed"

#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#include <Fonts/FreeSansBold9pt7b.h>
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

Adafruit_BME280  bme;  // initialize Adafruit BME280 library
float temp = 0.0;
float hum = 0.0;
float pres = 0.0;
float diff = 0.5;

//Pins for Inputs/Outputs
const int relayPinLowSpeed = 14;
const int relayPinMidSpeed = 12;
const int relayPinHighSpeed = 13;
const int inputSwitch = 3;
int switchState = 0;

WiFiClient espClient;
PubSubClient client(espClient);
long lastMsg = 0;
char msg[50];
int value = 0;

void setup() {
  Serial.begin(115200);
  setup_wifi();
  pinMode(relayPinLowSpeed, OUTPUT);
  pinMode(relayPinMidSpeed, OUTPUT);
  pinMode(relayPinHighSpeed, OUTPUT);
  pinMode(inputSwitch, INPUT);
  digitalWrite(relayPinLowSpeed, LOW);  
  digitalWrite(relayPinMidSpeed, LOW);  
  digitalWrite(relayPinHighSpeed, HIGH);  //invertiertes Signal
  ArduinoOTA.setHostname(ESPHostname);
  ArduinoOTA.setPassword("YourOTA_Password"); // Define your OTA Password
  ArduinoOTA.begin();
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C); // I2C address = 0x3C
  delay(1000);
  while(!Serial);    // time to get serial running
    unsigned status;
    status = bme.begin(0x76, &Wire);  //I2C address is either 0x76 or 0x77  
    if (!status) {
        Serial.println("Could not find a valid BME280 sensor, check wiring, address, sensor ID!");
        while (1);
    }
}

void loop() {
  if (!client.connected()) {
    reconnect();
  }
  client.loop();
  ArduinoOTA.handle(); 
  long now = millis();
  
  if (switchState == 0) {
    if (digitalRead(inputSwitch) == LOW){
      schalterAN();
      switchState = 1;
    }
  }
  if (switchState == 1) {
    if (digitalRead(inputSwitch) == HIGH){
      schalterAUS();
      switchState = 0;
    } 
  }
  if (now - lastMsg > 2000) {
    lastMsg = now;
    getBME280Values();
  }
}

void setup_wifi() {
  delay(10);
  // We start by connecting to a WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(wifi_ssid);
  WiFi.begin(wifi_ssid, wifi_password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

void callback(char* topic, byte* message, unsigned int length) {
  Serial.print("Message arrived on topic: ");
  Serial.print(topic);
  Serial.print(". Message: ");
  String messageTemp;
  
  for (int i = 0; i < length; i++) {
    Serial.print((char)message[i]);
    messageTemp += (char)message[i];
  }
  Serial.println();
  
  if (String(topic) == "fan/speed") {
    if(messageTemp == "1"){
      speed1();
    }
    else if(messageTemp == "2"){
      speed2();
    }
    else if(messageTemp == "3"){
      speed3();
    }
    else if(messageTemp == "0"){
      fanOFF();
    }
  }
}

bool checkBound(float newValue, float prevValue, float maxDiff) {
  return !isnan(newValue) &&
  (newValue < prevValue - maxDiff || newValue > prevValue + maxDiff);
}

void getBME280Values() {
  float newPres = bme.readPressure() / 100.0F;
  float newTemp = bme.readTemperature();
  float newHum = bme.readHumidity();

  if (checkBound(newTemp, temp, diff)) {
    temp = newTemp;
    Serial.print("New temperature:");
    Serial.println(String(temp).c_str());
    client.publish(temperature_topic, String(temp).c_str(), true);
    updateDisplay();
  }

  if (checkBound(newHum, hum, diff*2)) {
    hum = newHum;
    Serial.print("New Huminity:");
    Serial.println(String(hum).c_str());
    client.publish(humidity_topic, String(hum).c_str(), true);
    updateDisplay();
  }

  if (checkBound(newPres, pres, diff)) {
    pres = newPres;
    Serial.print("New Pressure:");
    Serial.println(String(pres).c_str());
    client.publish(pressure_topic, String(pres).c_str(), true);
  }  
}

void speed1() {
  Serial.println("Speed 1 (low)");
  digitalWrite(relayPinLowSpeed, HIGH); 
  digitalWrite(relayPinMidSpeed, LOW);  
  digitalWrite(relayPinHighSpeed, HIGH);   //invertiertes Signal
  client.publish(fanStatus_topic, "Run LOW speed");
  delay(200);  
}

void speed2() {
  Serial.println("Speed 2 (Mid)");
  digitalWrite(relayPinLowSpeed, LOW); 
  digitalWrite(relayPinMidSpeed, HIGH);  
  digitalWrite(relayPinHighSpeed, HIGH);   //invertiertes Signal
  client.publish(fanStatus_topic, "Run MID speed");
  delay(200); 
}

void speed3() {
  Serial.println("Speed 3 (High)");
  digitalWrite(relayPinLowSpeed, LOW); 
  digitalWrite(relayPinMidSpeed, LOW);  
  digitalWrite(relayPinHighSpeed, LOW);   //invertiertes Signal 
  client.publish(fanStatus_topic, "Run HIGH speed");
  delay(200); 
}
void fanOFF() {
  Serial.println("off");
  digitalWrite(relayPinLowSpeed, LOW);   
  digitalWrite(relayPinMidSpeed, LOW);  
  digitalWrite(relayPinHighSpeed, HIGH);   //invertiertes Signal 
  client.publish(fanStatus_topic, "Fan OFF");
  delay(200);
}

void schalterAN() {
  Serial.println("Schalter is AN");
  client.publish(fanStatus_topic, "Schalter AN");
  speed2();
}

void schalterAUS() {
  Serial.println("Schalter is AUS");
  client.publish(fanStatus_topic, "Schalter AUS");
  fanOFF();
}

void updateDisplay() {
  display.clearDisplay();
  display.setTextSize(1);   
  display.setTextColor(WHITE);
  display.setCursor(5, 8);
  display.setFont(&FreeSansBold9pt7b);
  display.print("Edi's Techlab");
  display.drawLine(0, 18, 127, 18, WHITE);
  display.setCursor(33, 41);
  display.print(temp);
  display.drawRect(79, 30, 3, 3, WHITE); // Das ° Symbol
  display.print(" C");
  display.setCursor(33, 63);
  display.print(hum);
  display.print(" %");
  display.display();
}

void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Create a random client ID
    String clientId = "ESP8266-";
    clientId += String(random(0xffff), HEX);
    // Attempt to connect
    if (client.connect(clientId.c_str(), mqtt_user, mqtt_password)) {
      Serial.println("connected");
      // Once connected, publish an announcement...
      client.publish(fanStatus_topic, "Fan alive");
      // ... and resubscribe
      client.subscribe(fanSpeed_topic);
    } else {
        Serial.print("failed, rc=");
        Serial.print(client.state());
        Serial.println(" try again in 5 seconds");
        // Wait 5 seconds before retrying
        delay(5000);
      }
  }
}

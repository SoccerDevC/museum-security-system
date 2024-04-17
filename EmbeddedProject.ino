
#include <SoftwareSerial.h>


#include "NewPing.h"
#include <dht.h>        // Include library
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define outPin 9        // Defines pin number to which the sensor is connected

#define DELAY_MS 1000
#define GSM_DELAY_MS 200
#define RESPONSE_TIMEOUT_MS 200
#define GPRS_RESPONSE_TIMEOUT_MS 3000
#define DISCONNECT_TIMEOUT_MS 4000

// Hook up HC-SR04 with Trig to Arduino Pin 9, Echo to Arduino pin 10
#define TRIGGER_PIN 6
#define ECHO_PIN 7

#define BUZZER_PIN 11  // Example pin, replace with your chosen pin

// Maximum distance we want to ping for (in centimeters).
#define MAX_DISTANCE 400	
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

// Declaration for SSD1306 display connected using I2C
#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C

//Create software serial object to communicate with SIM800L
SoftwareSerial SIM800(13, 12); //SIM800L Tx & Rx is connected to Arduino #3 & #2



Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
dht DHT;                // Creates a DHT object

// NewPing setup of pins and maximum distance.
NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE);

void initializeGSM();
void establishGPRSConnection();
boolean endGPRSConnection();
boolean isGPRSConnected();
boolean waitForExpectedResponse(const char* expectedAnswer = "OK", unsigned int timeout = RESPONSE_TIMEOUT_MS);
void connectToThingSpeak();
void sendDataToThingSpeak(float temperature, float humidity, int distance);
void disconnectFromThingSpeak();
void handleThingSpeak(float temperature, float humidity, int distance);

const char* APN = "internet";
const char* USER = "";
const char* PASS = "";
const char* THINGSPEAK_HOST = "api.thingspeak.com";
const int THINGSPEAK_PORT = 80;
const char* API_KEY = "PJKILCUJ44NZSCXD";



void setup() {
   //Begin serial communication with Arduino and Arduino IDE (Serial Monitor)
  Serial.begin(9600);
  
  //Begin serial communication with Arduino and SIM800L
  SIM800.begin(9600);

  Serial.println("Initializing...");
    Serial.println("yo...");

  delay(1000);
  initializeGSM();
  SIM800.println("AT"); //Once the handshake test is successful, i t will back to OK
  updateSerial();



  pinMode(BUZZER_PIN, OUTPUT);  // Set the buzzer pin as an output

	Serial.begin(9600);
    // initialize the OLED object
  if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
    for(;;); // Don't proceed, loop forever
  }
    display.clearDisplay();

}

void loop() {
  int distance = sonar.ping_cm();
  

  if (distance < 20) {
    digitalWrite(BUZZER_PIN, HIGH);  // Turn the buzzer ON
    
  } else {
    digitalWrite(BUZZER_PIN, LOW);  // Turn the buzzer OFF
  }
	
    int readData = DHT.read11(outPin);

    float t = DHT.temperature;        // Read temperature
    float h = DHT.humidity;           // Read humidity

    // Set text size and color
    display.setTextSize(1);
    display.setTextColor(WHITE);
    // Print temperature and humidity readings
    display.setCursor(0, 0);
    display.print("Temperature = ");
    Serial.println(t);
    display.print(t);
    display.println("°C |");
    display.print((t*9.0)/5.0+32.0);        // Convert celsius to fahrenheit
    display.println("°F ");
    display.print("Humidity = ");
    display.print(h);
    display.println("%");
    display.print("Distance = ");
    display.print(distance);
    display.println(" cm");
    // Display the content on the OLED
    display.display();

    delay(500); // wait two seconds
    display.clearDisplay();

    if (isnan(h) || isnan(t)) {
      Serial.println("Failed to read from DHT11 sensor!");
      return;
    }
    handleThingSpeak(t, h, distance);
    delay(500);
}


void updateSerial()
{
  delay(500);
  while (Serial.available()) 
  {
    SIM800.write(Serial.read());//Forward what Serial received to Software Serial Port
  }
  while(SIM800.available()) 
  {
    Serial.write(SIM800.read());//Forward what Software Serial received to Serial Port
  }
}

void initializeGSM() {
  SIM800.println("AT");
  waitForExpectedResponse();
  delay(GSM_DELAY_MS);
  SIM800.println("AT+CPIN?");
  waitForExpectedResponse("+CPIN: READY");
  delay(GSM_DELAY_MS);
  SIM800.println("AT+CFUN=1");
  waitForExpectedResponse();
  delay(GSM_DELAY_MS);
  SIM800.println("AT+CREG?");
  waitForExpectedResponse("+CREG: 0,");
  delay(GSM_DELAY_MS);
}

void establishGPRSConnection() {
  SIM800.println("AT+SAPBR=3,1,\"Contype\",\"GPRS\"");
  waitForExpectedResponse();
  delay(GSM_DELAY_MS);
  SIM800.println("AT+SAPBR=3,1,\"APN\",\"" + String(APN) + "\"");
  waitForExpectedResponse();
  delay(GSM_DELAY_MS);
  if (USER != "") {
    SIM800.println("AT+SAPBR=3,1,\"USER\",\"" + String(USER) + "\"");
    waitForExpectedResponse();
    delay(GSM_DELAY_MS);
  }
  if (PASS != "") {
    SIM800.println("AT+SAPBR=3,1,\"PWD\",\"" + String(PASS) + "\"");
    waitForExpectedResponse();
    delay(GSM_DELAY_MS);
  }
  SIM800.println("AT+SAPBR=1,1");
  waitForExpectedResponse("OK", GPRS_RESPONSE_TIMEOUT_MS);
  delay(GSM_DELAY_MS);
}

boolean isGPRSConnected() {
  SIM800.println("AT+SAPBR=2,1");
  if (waitForExpectedResponse("0.0.0.0") == 1) { return false; }
  return true;
}

boolean endGPRSConnection() {
  SIM800.println("AT+CGATT=0");
  waitForExpectedResponse("OK", DISCONNECT_TIMEOUT_MS);
  return true;
}

boolean waitForExpectedResponse(const char* expectedAnswer, unsigned int timeout) {
  unsigned long previous = millis();
  String response;
  while ((millis() - previous) < timeout) {
    while (SIM800.available()) {
      char c = SIM800.read();
      response.concat(c);
      if (response.indexOf(expectedAnswer) != -1) {
        Serial.println(response);
        return true;
      }
    }
  }
  Serial.println(response);
  return false;
}

void connectToThingSpeak() {
  Serial.println("Connecting to ThingSpeak...");
  SIM800.println("AT+CIPSTART=\"TCP\",\"" + String(THINGSPEAK_HOST) + "\"," + String(THINGSPEAK_PORT));
  if (waitForExpectedResponse("CONNECT OK")) {
    Serial.println("Connected to ThingSpeak");
  } else {
    Serial.println("Connection to ThingSpeak failed");
  }
}

void sendDataToThingSpeak(float temperature, float humidity, int distance) {
  String data = "api_key=" + String(API_KEY) + "&field1=" + String(temperature) + "&field2=" + String(humidity) + "&field3=" + String(distance);
  String postRequest = "POST /update HTTP/1.1\r\n";
  postRequest += "Host: " + String(THINGSPEAK_HOST) + "\r\n";
  postRequest += "Content-Type: application/x-www-form-urlencoded\r\n";
  postRequest += "Content-Length: " + String(data.length()) + "\r\n\r\n";
  postRequest += data;

  SIM800.println("AT+CIPSEND=" + String(postRequest.length()));
  if (waitForExpectedResponse(">")) {
    SIM800.println(postRequest);
    if (waitForExpectedResponse("OK")) {
      Serial.println("Data sent to ThingSpeak");
    } else {
      Serial.println("Failed to send data to ThingSpeak");
    }
  } else {
    Serial.println("Error in sending data to ThingSpeak");
  }
}

void disconnectFromThingSpeak() {
  SIM800.println("AT+CIPCLOSE");
  waitForExpectedResponse("CLOSE OK");
  Serial.println("Disconnected from ThingSpeak");
  delay(DELAY_MS);
}

void handleThingSpeak(float temperature, float humidity, int distance) {
  connectToThingSpeak();
  sendDataToThingSpeak(temperature, humidity, distance);
  disconnectFromThingSpeak();
}

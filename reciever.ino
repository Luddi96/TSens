#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

#include <ESP8266WiFi.h>
#include <WiFiClient.h> 
#include <ESP8266WebServer.h>
#include <ESP8266HTTPClient.h>

RF24 radio(D4, D8); // CE, CSN

uint8_t rxAddr[6] = {"XXXXX"}; //Adress of this Node
uint8_t txAddr[6] = {"XXXXX"}; //Adress of Node5

const char *ssid = "XXXX";  //ENTER YOUR WIFI SETTINGS
const char *password = "XXXX";

void setup() 
{
  Serial.begin(9600);
  radio.begin();
  radio.stopListening();
  radio.openWritingPipe(txAddr);
  radio.openReadingPipe(1,rxAddr);

  WiFi.mode(WIFI_OFF);        
  delay(1000);
  WiFi.mode(WIFI_STA);        
  
  WiFi.begin(ssid, password);     
  Serial.println("");
  Serial.print("Connecting");
  
  while (WiFi.status() != WL_CONNECTED) 
  {
    delay(500);
    Serial.print(".");
  }
}

void loop() 
{
  radio.startListening();
  if(radio.available())
  {
    char msg[300] = "";
    radio.read(&msg, sizeof(msg));
    String test = "http://address?log=";
    test += msg;
    
    WiFiClient client;
    HTTPClient http;

    http.begin(client,test);     //Specify request destination
    int httpCode = http.GET();            //Send the request
    String payload = http.getString();    //Get the response payload
  
    Serial.println(httpCode);   //Print HTTP return code
    Serial.println(payload);    //Print request response payload
  }
}

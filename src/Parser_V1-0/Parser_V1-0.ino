#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

#include <ESP8266WiFi.h>
#include <WiFiClient.h> 
#include <ESP8266WebServer.h>
#include <ESP8266HTTPClient.h>

RF24 radio(D4, D8); // CE, CSN

uint8_t rxAddr[6] = {"0KEVN"}; //Adress of this Node
uint8_t txAddr[6] = {"5KEVN"}; //Adress of Node5

//const char *ssid = "SSID";  //ENTER YOUR WIFI SETTINGS
//const char *password = "pass";

uint16_t getRemain(uint8_t mod_id);

void setup() 
{
  Serial.begin(9600);
  radio.begin();
  radio.stopListening();
  //radio.openWritingPipe(txAddr);
  radio.openReadingPipe(1,rxAddr);

  Serial.println("");
  Serial.println("");
  Serial.println("");
  Serial.println("Hallo Kevin, ich empfange jetzt");

  /*WiFi.mode(WIFI_OFF);        
  delay(1000);
  WiFi.mode(WIFI_STA);        
  
  WiFi.begin(ssid, password);     
  Serial.println("");
  Serial.print("Connecting");
  
  while (WiFi.status() != WL_CONNECTED) 
  {
    delay(500);
    Serial.print(".");
  }*/
}

int16_t zwerg = 0;
float fzwerg = 0;
uint16_t tx_remain = 4186;

int mod_id = 0;
uint8_t tx_arr[] = {0x00, 0x00, 0x0E, 0x00, 0x02, 0xFF};

void loop() 
{
  //txAddr[0] = 0xA1;
  radio.startListening();
  if(radio.available())
  {

    
    
    Serial.println("");
    Serial.println("_____________");
    Serial.println("Hex Rep:");
    
    uint8_t rcv_arr[35];
    for(int i = 0; i < 35; i++)
    {
      rcv_arr[i] = 0;
    }
    
    radio.read(&rcv_arr, sizeof(rcv_arr));
    
    for(int i = 0; i < 35; i++)
    {
      Serial.print(rcv_arr[i], HEX);
      Serial.print(", ");
    }
    
    Serial.println("");
    Serial.println("_____________");
    Serial.println("Decode:");

    for(int i = 0; i < 35; i++)
    {
      switch (rcv_arr[i])
      {
        case 0x00:  i++;
                    Serial.print("Mod ID: ");
                    Serial.println(rcv_arr[i],HEX);
                    mod_id = rcv_arr[i];
                    break;
        case 0x01:  i+=2;
                    zwerg = (rcv_arr[i] | (rcv_arr[i-1] << 8));
                    fzwerg = float(zwerg)/10.0;
                    Serial.print("Temp: ");
                    Serial.println(fzwerg,1);
                    break;
        case 0x02:  i+=2;
                    zwerg = (rcv_arr[i] | (rcv_arr[i-1] << 8));
                    fzwerg = float(zwerg)/10.0;
                    Serial.print("Hum: ");
                    Serial.println(fzwerg,1);
                    break;
        case 0x03:  i+=2;
                    zwerg = (rcv_arr[i] | (rcv_arr[i-1] << 8));
                    fzwerg = float(zwerg)/10.0;
                    Serial.print("Press: ");
                    Serial.println(fzwerg,1);
                    break;
        case 0x04:  i+=2;
                    zwerg = (rcv_arr[i] | (rcv_arr[i-1] << 8));
                    fzwerg = float(zwerg)/10.0;
                    Serial.print("Water: ");
                    Serial.println(fzwerg,1);
                    break;            
        case 0x05:  if(mod_id < 0xA0)
                    {
                      i++;
                      Serial.print("Batt: ");
                      Serial.println(rcv_arr[i]);
                    }
                    else
                    {
                      i+=2;
                      zwerg = (rcv_arr[i] | (rcv_arr[i-1] << 8));
                      fzwerg = float(zwerg)/100.0;
                      Serial.print("Batt: ");
                      Serial.println(fzwerg,2);
                    }
                    break;
        case 0x06:  i+=2;
                    zwerg = (rcv_arr[i] | (rcv_arr[i-1] << 8));
                    fzwerg = float(zwerg);
                    Serial.print("UV: ");
                    Serial.println(zwerg);
                    break;
        case 0x07:  i+=2;
                    zwerg = (rcv_arr[i] | (rcv_arr[i-1] << 8));
                    fzwerg = float(zwerg);
                    Serial.print("Rain: ");
                    Serial.println(zwerg);
                    break;
        case 0x0D:  if(mod_id < 0xA0)
                    {
                      i++;
                      Serial.print("Runtime: ");
                      Serial.println(rcv_arr[i]);
                    }
                    else
                    {
                      i+=2;
                      zwerg = (rcv_arr[i] | (rcv_arr[i-1] << 8));
                      Serial.print("Runtime: ");
                      Serial.println(zwerg);
                    }                 
                    break;
        /*case 0x0E:  //delay(100);
                    radio.stopListening();
                    tx_remain = getRemain(mod_id);
                    txAddr[0] = mod_id;
                    tx_arr[1] = mod_id;
                    tx_arr[3] = msb(tx_remain);
                    tx_arr[4] = lsb(tx_remain);
                    radio.openWritingPipe(txAddr);
                    Serial.println("Sending..");
                    if(!radio.write(&tx_arr, sizeof(tx_arr)))
                    { Serial.println("T1");
                      delay(100);
                      if(!radio.write(&tx_arr, sizeof(tx_arr)))
                      {Serial.println("T2");
                        delay(100);
                        if(!radio.write(&tx_arr, sizeof(tx_arr)))
                        {Serial.println("T3");
                          delay(100);
                        }
                      }
                    }
                    radio.startListening();
                    break;*/
        case 0x08:  Serial.println("Gauge Int!");
                    break;
        case 0xFF:  Serial.println("End");
                    i = 36;
                    break;
        default:    Serial.println("Error decoding!");
                    i=36;
      }
    }
    
    Serial.println("_____________");
    Serial.println("Finished");
    Serial.println("_____________");
    Serial.println("");
  }
}
/*uint16_t getRemain(uint8_t mod_id)
{
  WiFiClient client;
  HTTPClient http;

  String addr = "http://192.168.178.4/hs/ttn.php";
  
  http.begin(client,addr);

  int httpCode = http.GET();            //Send the request
  String payload = http.getString();

  Serial.println(payload);

  uint16_t remain = payload.toInt();
  remain += (mod_id & 0x0F) * 2;
  Serial.println(remain);
  if(remain > 0 && remain < 10000)
  {
    return remain;
  }
  else
  {
    return 4186;
  }
}*/

uint8_t msb(uint16_t input)
{
  return (input >> 8 ) & 0xFF;
}
uint8_t lsb(uint16_t input)
{
  return input & 0xFF;
}

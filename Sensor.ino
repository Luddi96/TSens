#include <Wire.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

RF24 radio(7, 8); // CE, CSN
Adafruit_BME280 bme;

uint8_t txAddr[6] = {"XXXXX"}; //Adress of Server

void setup() {

  WDTCSR = (24);//change enable and WDE - also resets
  WDTCSR = (33);//prescalers only - get rid of the WDE and WDCE bit
  WDTCSR |= (1<<6);//enable interrupt mode

  //Disable ADC - don't forget to flip back after waking up if using ADC in your application ADCSRA |= (1 << 7);
  ADCSRA &= ~(1 << 7);
  
  //ENABLE SLEEP - this enables the sleep mode
  SMCR |= (1 << 2); //power down mode
  SMCR |= 1;//enable sleep

  radio.begin();
  radio.stopListening();
  radio.openWritingPipe(txAddr);

  bme.begin(0x76, &Wire);
  bme.setSampling(Adafruit_BME280::MODE_FORCED,
                    Adafruit_BME280::SAMPLING_X1, // temperature
                    Adafruit_BME280::SAMPLING_X1, // pressure
                    Adafruit_BME280::SAMPLING_X1, // humidity
                    Adafruit_BME280::FILTER_OFF   );
  pinMode(A2, OUTPUT);
}
unsigned long after = 0;
unsigned long before = 0;
unsigned long before2 = 0;
int test = 0;
void loop() {
  
  //delay(5000);
  
  for(int i = 0; i < 7; i++)
  {
    //BOD DISABLE - this must be called right before the __asm__ sleep instruction
    MCUCR |= (3 << 5); //set both BODS and BODSE at the same time
    MCUCR = (MCUCR & ~(1 << 5)) | (1 << 6); //then set the BODS bit and clear the BODSE bit at the same time
    __asm__  __volatile__("sleep");//in line assembler to go to sleep
  }
  //Wake up here
  delay(1);
  bme.takeForcedMeasurement();
  delay(1);
  before2 = before;
  before = millis();
  ADCSRA |= (1 << 7);
  radio.powerUp();

  //Code from here
  /*********************/
  digitalWrite(A2, HIGH);
  String out = "";
  out += after;
  out += ";";
  out += after-before2;
  out += ";";
  out += round((float)analogRead(A1)/1024.0*6.6*106.8);
  digitalWrite(A2, LOW);
  out += ";";
  out += round(10.0*bme.readTemperature());
  out += ";";
  out += round(10.0*bme.readHumidity());
  out += ";";
  out += round(bme.readPressure() / 100.0F);
  out += " ";
  
  char cstr[out.length()];
  out.toCharArray(cstr, out.length());
  radio.stopListening();
  radio.write(&cstr, out.length());
  /*******************/
  //Code to here
  
  //Disable
  radio.powerDown();
  ADCSRA &= ~(1 << 7);
  after = millis();
  //Go to sleep
}

ISR(WDT_vect){
}

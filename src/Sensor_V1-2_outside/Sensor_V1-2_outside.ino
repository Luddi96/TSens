/**Includes**/
#include <Wire.h> //Needed for BME
#include <SPI.h> //Needed for NRF
#include <nRF24L01.h> //Needed for NRF
#include <RF24.h> //Needed for NRF
#include <Adafruit_Sensor.h> //Needed for BME
#include <Adafruit_BME280.h> //Needed for BME

//#define S_VERB 1 //Verbose output
//#define S_VERB2 1 //Very Verbose output
#define RAIN_GAUGE_PRESENT 1 //Rain Gauge connected to P_INT_RAIN_GAUGE and GND
#define RAIN_UV_PRESENT 1 //UV Sensor connected P_UV_RAIN_EN, P_RAIN_MEAS, P_UV_MEAS and GND
#define RAIN_TRESH 500 //Treshold when to trigger rain warning
#define RAIN_TIMEOUT 80 //How long to pause measurement afer rain warning triggered (*8s)

/**Pin Assignment**/
#define P_NRF_CE 7 //CE of NRF Module
#define P_NRF_CSN 8 //CSN of NRF Module
#define P_UBAT_EN A3 //This pin enables UBAT measurement (approx. 200uA)
#define P_UBAT_MEAS A2 //Analog pin to read UBAT via divider
#define P_SW_USER 6 //Button to GND - Short press: Reset, long press: This pin is low at startup, otherwise high
#define P_LED_USER 5 //Led for button illumination (approx. 20mA)
#define P_INT_RAIN_GAUGE 2 //Interrupt Pin for Rain Gauge
#define P_UV_RAIN_EN A0 //Enable/Supply Pin for UV + Rain Sensor
#define P_RAIN_MEAS A1 //Analog pin to measure rain value
#define P_UV_MEAS A6 //Analog pin to measure UV value

/**Adress definition**/
#define A_BME_I2C 0x76
#define A_BROADC "0KEVN"
#define MODULE_ID 0x01

/**Instance creation**/
RF24 radio(P_NRF_CE, P_NRF_CSN);
Adafruit_BME280 bme;

/**Global variables**/
uint8_t txAddr[6] = {A_BROADC};
float volt = 0;

#ifdef RAIN_GAUGE_PRESENT
bool rainRisingDetect = 0;
bool waitForRainComplete = 0;
int rainTimeout = 0;
unsigned long rainFallStamp = 0;

void rainInterruptTrigger();
#endif

#ifdef RAIN_UV_PRESENT
int uvRainTimeout = 0;
void measUVRain(uint16_t *uv, uint16_t *rain);
void rainUvInterruptTrigger(uint16_t rain);
#endif

/**Functions**/
void initReg();
void initPins();
void checkButton();
void initNRF();
void initBME();
void deepSleep(int duration);
uint8_t measUBat();
uint8_t msb(uint16_t input);
uint8_t lsb(uint16_t input);

void setup() 
{
  #ifdef S_VERB
  Serial.begin(9600);
  while (!Serial) //Wait for serial
  {
  }
  Serial.println("\n\n\nReset!");
  #endif
  initReg();
  initPins();
  checkButton();
  initNRF();
  initBME();
}



unsigned long after = 0;
unsigned long before = 0;
unsigned long before2 = 0;
int test = 0;

void loop() {
  
  
  before2 = before;
  before = millis();

  //Code from here
  bme.takeForcedMeasurement();
  /*int uv = 0;
  int rain = 0;
  measUVRain(&uv, &rain);*/

  uint16_t tx_temp = round(10.0*bme.readTemperature());
  uint16_t tx_hum = round(10.0*bme.readHumidity());
  uint16_t tx_press = round(bme.readPressure() / 10.0F); //This means 10*hpa
  uint8_t tx_batt = measUBat();
  uint16_t tx_uv = 0;
  uint16_t tx_rain = 0;
  measUVRain(&tx_uv, &tx_rain);
  uint8_t tx_runtime = (after-before2 ) & 0xFF;
  
  uint8_t tx_arr[] = {0x00, MODULE_ID, 0x01, msb(tx_temp), lsb(tx_temp), 0x02, msb(tx_hum), lsb(tx_hum), 0x03, msb(tx_press), lsb(tx_press), 0x05, tx_batt, 0x06, msb(tx_uv), lsb(tx_uv), 0x07, msb(tx_rain), lsb(tx_rain), 0x0D, tx_runtime, 0xFF};
  
  radio.stopListening();
  radio.write(&tx_arr, sizeof(tx_arr));
  /*
  String out = "";
  out += "M3;";
  out += after-before2;
  out += ";";
  out += round(100*volt);
  out += ";";
  out += round(10.0*bme.readTemperature());
  out += ";";
  out += round(10.0*bme.readHumidity());
  out += ";";
  out += round(bme.readPressure() / 100.0F);
  out += ";";
  out += uv;
  out += ";";
  out += rain;
  out += " ";
  char cstr[out.length()];
  out.toCharArray(cstr, out.length());
  radio.stopListening();
  radio.write(&cstr, out.length());
  //Serial.println(out);*/
  //Code to here
  
  after = millis();
  //Go to sleep
  deepSleep(8);
}

/** Functions **/

/* Set all registers for operation */
void initReg()
{
  #ifdef S_VERB
  Serial.print("Setting register...");
  #endif
  WDTCSR = (24); //change enable and WDE - also resets
  WDTCSR = (33); //prescalers only - get rid of the WDE and WDCE bit
  WDTCSR |= (1<<6); //enable interrupt mode

  //Disable ADC - don't forget to flip back after waking up if using ADC in your application ADCSRA |= (1 << 7);
  //ADCSRA &= ~(1 << 7);
  
  //ENABLE SLEEP - this enables the sleep mode
  SMCR |= (1 << 2); //power down mode
  SMCR |= 1;//enable sleep
  #ifdef S_VERB
  Serial.println("Done");
  #endif
}

/* Init all Pins */
void initPins()
{
  #ifdef S_VERB
  Serial.print("Pin init...");
  #endif
  pinMode(P_UBAT_EN, OUTPUT);
  pinMode(P_SW_USER, INPUT);
  pinMode(P_LED_USER, OUTPUT);
  
  #ifdef RAIN_GAUGE_PRESENT
    #ifdef S_VERB
    Serial.print("Using Rain Gauge, setting Interrupt...");
    #endif
    attachInterrupt(digitalPinToInterrupt(P_INT_RAIN_GAUGE), rainFalling, CHANGE); //interrupt for waking up
  #endif
  #ifdef RAIN_UV_PRESENT
    #ifdef S_VERB
    Serial.print("Using Rain UV Sensor, setting Pin...");
    #endif
    pinMode(A0, OUTPUT);
  #endif
}

/* Check if user presses button and wait till let go */
void checkButton()
{
  #ifdef S_VERB
  Serial.println("Checking for button press...");
  #endif
  if(!digitalRead(P_SW_USER))
  {
    delay(100);
    if(!digitalRead(P_SW_USER))
    {
      delay(1000);
      if(!digitalRead(P_SW_USER))
      {
        #ifdef S_VERB
        Serial.println("Button pressed");
        #endif
        while(!digitalRead(P_SW_USER))
        {
          delay(10);
        }
        #ifdef S_VERB
        Serial.println("Button released");
        #endif
      }
    }
  }
  else
  {
    #ifdef S_VERB
    Serial.println("No press detected...");
    #endif    
  }
}

/*Init NRF Module */
void initNRF()
{
  #ifdef S_VERB
  Serial.print("Enabling NRF...");
  #endif
  radio.begin();
  radio.stopListening();
  radio.openWritingPipe(txAddr);
  #ifdef S_VERB
  Serial.println("Done");
  #endif
}

/* Init BME Module */
void initBME()
{
  #ifdef S_VERB
  Serial.print("Enabling BME...");
  #endif
  bme.begin(A_BME_I2C, &Wire);
  bme.setSampling(Adafruit_BME280::MODE_FORCED,
                    Adafruit_BME280::SAMPLING_X1, // temperature
                    Adafruit_BME280::SAMPLING_X1, // pressure
                    Adafruit_BME280::SAMPLING_X1, // humidity
                    Adafruit_BME280::FILTER_OFF   );
  #ifdef S_VERB
  Serial.println("Done");
  #endif
}

/* Deep sleep for approx. (duration*9) seconds */
void deepSleep(int duration)
{
  #ifdef S_VERB
  Serial.println("Going to sleep...");
  delay(200);
  Serial.end();
  #endif
  //digitalWrite(P_UBAT_EN, LOW); //Diable UBAT Meas
  radio.powerDown(); //Disable NRF
  ADCSRA &= ~(1 << 7); //Disable ADC
  //Actual sleep
  for(int i = 0; i < duration; i++)
  {
    #ifdef RAIN_GAUGE_PRESENT
    rainInterruptTrigger();
    #endif
    
    #ifdef RAIN_UV_PRESENT
    if(uvRainTimeout == 0)
    {
      uint16_t uv = 0;
      uint16_t rain = 0;
      ADCSRA |= (1 << 7); //Enable ADC
      measUVRain(&uv, &rain);
      ADCSRA &= ~(1 << 7); //Disable ADC
      if(rain < RAIN_TRESH)
      {
        rainUvInterruptTrigger(rain);
      }
    }
    if(uvRainTimeout > 0) uvRainTimeout --;
    if(uvRainTimeout < 0) uvRainTimeout = 0;
    #endif
    //BOD DISABLE - this must be called right before the __asm__ sleep instruction
    MCUCR |= (3 << 5); //set both BODS and BODSE at the same time
    MCUCR = (MCUCR & ~(1 << 5)) | (1 << 6); //then set the BODS bit and clear the BODSE bit at the same time
    __asm__  __volatile__("sleep");//in line assembler to go to sleep
  }
  //Actual wake up
  delay(1);
  #ifdef S_VERB
  Serial.begin(9600);
  while (!Serial) //Wait for serial
  {
    delay(1);
  }
  Serial.println("Woke up...");
  #endif
  ADCSRA |= (1 << 7); //Enable ADC
  radio.powerUp(); //Enable NRF
  //digitalWrite(P_UBAT_EN, HIGH); //Enable UBAT Meas
}

/* Measure UBat and give back percent */
uint8_t measUBat()
{
  digitalWrite(P_UBAT_EN, HIGH); //Enable UBAT Meas
  delay(5);
  #ifdef S_VERB2
  Serial.println("Measuring UBat...");
  #endif
  float rawValue = 0;
  for(int i = 0; i < 3; i++)
  {
    delay(1);
    rawValue += analogRead(P_UBAT_MEAS);
  }
  digitalWrite(P_UBAT_EN, LOW); //Diable UBAT Meas
  volt = rawValue / 3072.0 * 6.6 * 1.025586;
  float offsVolt = volt - 3.3;
  if(offsVolt > 1.2) offsVolt = 1.2;
  if(offsVolt < 0.0) offsVolt = 0.0;
  float fPercent = -(13.27 * pow(offsVolt,5)) - (37.49 * pow(offsVolt,4)) + (57.53 * pow(offsVolt,3)) + (42.27 * pow(offsVolt,2)) + (44.17 * offsVolt) - 1.9;
  if(fPercent > 100.0) fPercent = 100.0;
  if(fPercent < 0.00) fPercent = 0.0;
  uint8_t percent = round(fPercent);
  
  #ifdef S_VERB2
  Serial.print("UBat: ");
  Serial.print(volt);
  Serial.print("V, Percent: ");
  Serial.print(percent);
  Serial.println("%\n");
  #endif

  return percent;
}

uint8_t msb(uint16_t input)
{
  return (input >> 8 ) & 0xFF;
}
uint8_t lsb(uint16_t input)
{
  return input & 0xFF;
}

/* Leave this untouched */
ISR(WDT_vect) 
{
  #ifdef RAIN_GAUGE_PRESENT
  if(rainTimeout) rainTimeout --;
  #endif
}


#ifdef RAIN_GAUGE_PRESENT
//Check for Rain Interrupt and send message
void rainInterruptTrigger()
{
  while(waitForRainComplete && !rainRisingDetect && millis()-rainFallStamp < 150 && !rainTimeout)
  {
    delay(1);
  }
  waitForRainComplete = 0;
  if(rainRisingDetect && !rainTimeout)
  {
    #ifdef S_VERB
    Serial.println("Rain detected by Wippe");
    delay(100);
    #endif

    radio.powerUp();
    uint8_t tx_arr[] = {0x00, MODULE_ID, 0x08, 0xFF};
    radio.stopListening();
    if(radio.write(&tx_arr, sizeof(tx_arr)))
    {
      rainTimeout = 2;
      rainRisingDetect = 0;
    }
    radio.powerDown();  
  }
}

//ISR for Rain Gauge
void rainFalling(){
  if(digitalRead(2))
  {
    if((millis() - rainFallStamp) > 30 && (millis() - rainFallStamp) < 150 && waitForRainComplete)
    {
      waitForRainComplete = 0;
      rainRisingDetect = 1;
    }
  }
  else
  {
    if(!waitForRainComplete)
    {
      waitForRainComplete = 1;
      rainFallStamp = millis();
    }
  }
}
#endif

#ifdef RAIN_UV_PRESENT
void measUVRain(uint16_t *uv, uint16_t *rain)
{
  digitalWrite(P_UV_RAIN_EN, HIGH);
  delay(3);
  *uv = analogRead(P_UV_MEAS);
  *rain = analogRead(P_RAIN_MEAS);
  digitalWrite(P_UV_RAIN_EN, LOW);
}
void rainUvInterruptTrigger(uint16_t rain)
{
  #ifdef S_VERB
  Serial.println("Rain detected by UV");
  delay(100);
  #endif

  radio.powerUp();
  uint8_t tx_arr[] = {0x00, MODULE_ID, 0x07, msb(rain), lsb(rain), 0xFF};
  radio.stopListening();
  if(radio.write(&tx_arr, sizeof(tx_arr)))
  {
    uvRainTimeout = RAIN_TIMEOUT;
  }
  radio.powerDown();
}
#endif


#include <EEPROM.h>
#include "Nextion.h"
#include <PID_v1.h>
#include <PID_AutoTune_v0.h>
#include <Adafruit_MAX31865.h>
#include <PWM.h>

const int f_low = 6;     //relay pin for fan speed
const int f_medium = 7;
const int f_high = 8;

int valve = 9;  //0-10V output pin (PWM output which will be converted to 0-10V after opamp)
int32_t frequency = 10000;  //pwm frequency
int temper = 30; //default set temperature

double Setpoint, Input, Output;  //variables for PID class

byte ATuneModeRemember = 2; //variable for a PID object


double kp = 2, ki = 0.5, kd = 2; // default tuning for PID algorithm

double kpmodel = 1.5, taup = 100, theta[50]; //unnecessary
double outputStart = 5; //unnecessary
double aTuneStep = 50, aTuneNoise = 1, aTuneStartValue = 100; //auto tuning variables
unsigned int aTuneLookBack = 20;

boolean tuning = true;
unsigned long  modelTime, serialTime;  //unnecessary

PID myPID(&Input, &Output, &Setpoint, kp, ki, kd, DIRECT); //PID object
PID_ATune aTune(&Input, &Output);  //Auto tuning function

boolean useSimulation = false; //unnecessary



//PID myPID(&Input, &Output, &Setpoint, consKp, consKi, consKd, DIRECT);

uint32_t set_point = 20; //default set_point

unsigned long previousMillis = 0;        // used for counting time
unsigned long currentMillis; // used for counting time

// constants won't change:
const long interval = 1000;  //interval preiod for calling a function repeatedly
// Use software SPI: CS, DI, DO, CLK
Adafruit_MAX31865 max = Adafruit_MAX31865(5, 4, 3, 2);  //PT100/PT1000 sensor interface board pin configuration
// use hardware SPI, just pass in the CS pin
//Adafruit_MAX31865 max = Adafruit_MAX31865(10);
char buffer1[4];  //used for integar to array conversion
// The value of the Rref resistor. Use 430.0 for PT100 and 4300.0 for PT1000
#define RREF      430.0 // The 'nominal' 0-degrees-C resistance of the sensor
// 100.0 for PT100, 1000.0 for PT1000
#define RNOMINAL  100.0



//Touch screen objects
// Declare your Nextion objects - Example (page id = 0, component id = 1, component name = "b0")
NexText rt = NexText(1, 5, "t0"); //use for setting text of an item in touchscreen
NexText fspeed = NexText(1, 10, "t4"); //use for setting text of an item in touchscreen
NexButton bHigh = NexButton(1, 1, "bHigh"); //used to identify which button of touchscreen was pressed
NexButton bMedium = NexButton(1, 2, "bMedium"); //used to identify which button of touchscreen was pressed
NexButton bLow = NexButton(1, 10, "bLow"); //used to identify which button of touchscreen was pressed
NexSlider h0 = NexSlider(1, 6, "h0"); //used to identify which slider touchscreen was moved
//NexText tSlider = NexText(0, 6, "tSlider");
//NexText tTempC = NexText(1, 5, "tTempC");
//NexText tTempF = NexText(1, 4, "tTempF");
//NexProgressBar jHumidity = NexProgressBar(1, 8, "jHumidity");
//NexText tHumidity = NexText(1, 9, "tHumidity");
//NexButton bUpdate = NexButton(1,10, "bUpdate");

// this function needs to be called repeatedly as fast as possible to detect touch events from touchscreen.
//Events that will be monitored needs to be added to this list
NexTouch *nex_listen_list[] = {
  &bHigh,  //button named High in Touchscreen
  &bMedium, //button named Medium in Touchscreen
  &bLow, //button named Low in Touchscreen
  &h0, //name of slider which will be monitored
  NULL
};


void bHighPopCallback(void *ptr)  //this function which is tied to bHigh button will be called whenever the high button in touchscreen was pressed
{

  fspeed.setText("High");

  digitalWrite(f_low, LOW);
  digitalWrite(f_medium, LOW);
  digitalWrite(f_high, HIGH);
  //Serial.println("Fan speed set to High");
  EEPROM.update(1, 3);

}


void bMediumPopCallback(void *ptr)  //this function which is tied to bHigh button will be called whenever the medium button in touchscreen was pressed
{

  fspeed.setText("Medium");

  digitalWrite(f_low, LOW); //setting relay f_low to off
  digitalWrite(f_high, LOW);  //setting relay f_medium to off
  digitalWrite(f_medium, HIGH); //setting relay f_high on
  //Serial.println("Fan speed set to Medium");
  EEPROM.update(1, 2);
}

void bLowPopCallback(void *ptr)  //this function which is tied to bHigh button will be called whenever the low button in touchscreen was pressed
{

  fspeed.setText("Low");
  digitalWrite(f_medium, LOW);
  digitalWrite(f_high, LOW);
  digitalWrite(f_low, HIGH);
  //Serial.println("Fan speed set to Low");
  EEPROM.update(1, 1);
}

/*
   Slider h0 component pop callback function.
   When the slider is released, the LED brightness changes and the slider text changes.
*/
void h0PopCallback(void *ptr) { //this function which is tied to h0 slider will be called whenever the slider in touchscreen was moved



  //calling the touchscreen to report the value of the slider
  //needs to be called repeatedly for a few times because touchscreen does not respond properly due to a bug
  h0.getValue(&set_point);  //get value from slider in touchscreen and place it into set_point
  h0.getValue(&set_point);
  h0.getValue(&set_point);
  h0.getValue(&set_point);
  h0.getValue(&set_point);
  h0.getValue(&set_point);
  h0.getValue(&set_point);
  h0.getValue(&set_point);
  h0.getValue(&set_point);
  h0.getValue(&set_point);
  h0.getValue(&set_point);
  h0.getValue(&set_point);
  h0.getValue(&set_point);
  h0.getValue(&set_point);
  h0.getValue(&set_point);
  h0.getValue(&set_point);
  h0.getValue(&set_point);
  h0.getValue(&set_point);
  h0.getValue(&set_point);
  h0.getValue(&set_point);
  h0.getValue(&set_point);
  h0.getValue(&set_point);
  h0.getValue(&set_point);
  h0.getValue(&set_point);
  h0.getValue(&set_point);



  //the ASCII of the integer will be stored in this char array
  utoa(set_point, buffer1, 10);
  //Serial.write(0xff);
  //Serial.write(0xff);
  //Serial.write(0xff);
  Serial.print("n0.val=");
  Serial.print(buffer1);
  Serial.write(0xff);  //send these characters to touchscreen to notify end of instruction three times
  Serial.write(0xff);
  Serial.write(0xff);
  Setpoint = set_point;  //keeping the variable in another variable which will be used in PID algorithm
  EEPROM.update(0, set_point); //store value of set_point in eeprom for device restart
  // change LED brightness
  //analogWrite(led2, number);
}

void read_sensor() //getting pt100 temperature
{

  uint16_t rtd = max.readRTD();
  nexLoop(nex_listen_list);  //calling this function checks any touch event from touchscreen, not calling this function repeatedly can cause the microcontroller to miss a touch event. therefore it is called here in between another work.
  //Serial.print("RTD value: "); Serial.println(rtd);
  float ratio = rtd;
  nexLoop(nex_listen_list);
  ratio /= 32768;
  //Serial.print("Ratio = "); Serial.println(ratio,8);
  //Serial.print("Resistance = "); Serial.println(RREF*ratio,8);
  //Serial.print("Temperature = "); Serial.println(max.temperature(RNOMINAL, RREF));
  temper = max.temperature(RNOMINAL, RREF);
  nexLoop(nex_listen_list); //calling this function checks any touch event from touchscreen, not calling this function repeatedly can cause the microcontroller to miss a touch event. therefore it is called here in between another work.
  char buffer2[4];         //the ASCII of the integer will be stored in this char array
  itoa(temper, buffer2, 10);
  // Serial.print("t0.txt=");
  //Serial.print(buffer);
  //Serial.write(0xff);
  //Serial.write(0xff);
  //Serial.write(0xff);
  rt.setText(buffer2); //showing the current temperature in a text field insider touchscreen
  utoa(set_point, buffer1, 10); //unsigned integer to array conversion
  //delay(6000);
  //Serial.write(0xff);
  //Serial.write(0xff);
  //Serial.write(0xff);
  // Serial.print("n0.val="); //setting value of n0. field in touchscreen which shows current set temperature point
  // Serial.print(buffer1);
  // Serial.write(0xff);
  // Serial.write(0xff);
  //Serial.write(0xff);
  // Check and print any faults
  uint8_t fault = max.readFault();
  if (fault) {
    Serial.print("Fault 0x"); Serial.println(fault, HEX);
    if (fault & MAX31865_FAULT_HIGHTHRESH) {
      Serial.println("RTD High Threshold");
    }
    if (fault & MAX31865_FAULT_LOWTHRESH) {
      Serial.println("RTD Low Threshold");
    }
    if (fault & MAX31865_FAULT_REFINLOW) {
      Serial.println("REFIN- > 0.85 x Bias");
    }
    if (fault & MAX31865_FAULT_REFINHIGH) {
      Serial.println("REFIN- < 0.85 x Bias - FORCE- open");
    }
    if (fault & MAX31865_FAULT_RTDINLOW) {
      Serial.println("RTDIN- < 0.85 x Bias - FORCE- open");
    }
    if (fault & MAX31865_FAULT_OVUV) {
      Serial.println("Under/Over voltage");
    }
    max.clearFault();
  }
  //Serial.println();
  //delay(1000);
}


void changeAutoTune()
{
  if (!tuning)
  {
    //Set the output to the desired starting frequency.
    Output = aTuneStartValue;
    aTune.SetNoiseBand(aTuneNoise);
    aTune.SetOutputStep(aTuneStep);
    aTune.SetLookbackSec((int)aTuneLookBack);
    AutoTuneHelper(true);
    tuning = true;
  }
  else
  { //cancel autotune
    aTune.Cancel();
    tuning = false;
    AutoTuneHelper(false);
  }
}

void AutoTuneHelper(boolean start)
{
  if (start)
    ATuneModeRemember = myPID.GetMode();
  else
    myPID.SetMode(ATuneModeRemember);
}



void setup(void) {   //arduino initialization
  //  dht.begin();
  InitTimersSafe();
  set_point = EEPROM.read(0);
  

  Serial.begin(9600);
  delay(10000);  // This dalay is just in case the nextion display didn't start yet, to be sure it will receive the following command.
  // Serial.print("baud=115200");
  Serial.write(0xff);  //end of instruction flag for touchscreen
  Serial.write(0xff);
  Serial.write(0xff);
  Serial.print("thsp=30"); //nextion instruction to go to sleep after no touch
  Serial.write(0xff); //end of instruction
  Serial.write(0xff);
  Serial.write(0xff);
  Serial.print("thup=1"); //nextion instruction to set "wake-up on touch"
  Serial.write(0xff);
  Serial.write(0xff);
  Serial.write(0xff);
  //Serial.end();
  //Serial.flush();
  //Serial.begin(115200);

  max.begin(MAX31865_3WIRE);  // set to 2WIRE or 4WIRE as necessary
  // You might need to change NexConfig.h file in your ITEADLIB_Arduino_Nextion folder
  // Set the baudrate which is for debug and communicate with Nextion screen
  nexInit();
  SetPinFrequencySafe(valve, frequency); //pwm output pin initialization

  // Register the pop event callback function of the components for touchscreen
  bHigh.attachPop(bHighPopCallback, &bHigh);
  bMedium.attachPop(bMediumPopCallback, &bMedium);
  bLow.attachPop(bLowPopCallback, &bLow);
  h0.attachPop(h0PopCallback);
  // bUpdate.attachPop(bUpdatePopCallback, &bUpdate);

  
  pinMode(f_low, OUTPUT);
  pinMode(f_medium, OUTPUT);
  pinMode(f_high, OUTPUT);
  utoa(set_point, buffer1, 10);
  delay(6000); //another delay to allow display to ready up
  Serial.write(0xff);
  Serial.write(0xff);
  Serial.write(0xff);
  Serial.print("n0.val=");
  Serial.print(buffer1);
  Serial.write(0xff);
  Serial.write(0xff);
  Serial.write(0xff);


  Setpoint = set_point;

  //turn the PID on
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(0, 200);
  if (tuning)
  {
    tuning = false;
    changeAutoTune();
    tuning = true;
  }
  int set_speed = EEPROM.read(1);

  switch (set_speed)
  {
    case 1:
      digitalWrite(f_medium, LOW);
      digitalWrite(f_high, LOW);
      digitalWrite(f_low, HIGH);
      fspeed.setText("Low");
      break;
    case 2:
      digitalWrite(f_low, LOW); //setting relay f_low to off
      digitalWrite(f_high, LOW);  //setting relay f_medium to off
      digitalWrite(f_medium, HIGH); //setting relay f_high on
      fspeed.setText("Medium");
      break;
    case 3:
      digitalWrite(f_low, LOW);
      digitalWrite(f_medium, LOW);
      digitalWrite(f_high, HIGH);
      fspeed.setText("High");
      break;
    default:
      ;
  }
}

void loop(void) //main loop
{

  /*
     When a pop or push event occured every time,
     the corresponding component[right page id and component id] in touch event list will be asked.
  */
  nexLoop(nex_listen_list); //calling this function checks any touch event from touchscreen, not calling this function repeatedly can cause the microcontroller to miss a touch event. therefore it is called here in between another work.
  currentMillis = millis();
  if (currentMillis - previousMillis >= interval) //execture this if block after every one second to avoid getting pt100 sensor value repeatedly
  {
    // save the last time you blinked the LED
    previousMillis = currentMillis;
    nexLoop(nex_listen_list);
    read_sensor();
    char buffer3[4];         //the ASCII of the integer will be stored in this char array
    int gauge = map(Output, 0, 255, 0, 90); //mapping pwm duty cycle from 0 - 255 to 0-90
    itoa(gauge, buffer3, 10);
    //Serial.write(0xff);
    //Serial.write(0xff);
    //Serial.write(0xff);
    nexLoop(nex_listen_list);
    Serial.print("z0.val="); //set gauge pointer in touchscreen
    Serial.print(buffer3);
    Serial.write(0xff);
    Serial.write(0xff);
    Serial.write(0xff);
  }



  Input = temper;  //putting pt100 sensor temperature into Input variable which is used in PID algorithm to calculate duty cycle

  if (tuning)
  {
    byte val = (aTune.Runtime());
    if (val != 0)
    {
      tuning = false;
      nexLoop(nex_listen_list);
    }
    if (!tuning)
    { //we're done, set the tuning parameters
      kp = aTune.GetKp();
      ki = aTune.GetKi();
      kd = aTune.GetKd();
      myPID.SetTunings(kp, ki, kd);
      AutoTuneHelper(false);
      nexLoop(nex_listen_list);
    }
  }
  else myPID.Compute();

  myPID.Compute();
  //analogWrite(valve, Output);
  pwmWrite(valve, Output);





  nexLoop(nex_listen_list);

}

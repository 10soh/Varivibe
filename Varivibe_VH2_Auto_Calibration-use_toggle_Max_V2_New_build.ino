#include "VectorHaptics.h"
#include "Esp32PicoMini.h"
#include <movingAvgFloat.h>
// Importing Required header files
#include <VHChannelList.h>
#include <VHCore.h>
VectorHaptics<Esp32PicoMini> vh;
VHCore core;

VHChannel chnl1(1, 25,{"Left channel", "Channel 1", "Left", "Finger"});
VHChannel chnl2(2, 26,{"Right channel", "Channel 2", "Right", "Finger"});
VHChannelList list({&chnl1,&chnl2});

#include <Wire.h>
#include <SparkFun_MMC5983MA_Arduino_Library.h> //Click here to get the library: http://librarymanager/All#SparkFun_MMC5983MA
SFE_MMC5983MA myMag;

double findZ = -1000;
double maxVal = 0.0;
double minVal = 0.0;
double avgVal;
movingAvgFloat avg(20);

int fMax = 300;
int fMin = 40;
int freqVal = 0.5 * (fMax + fMin); //holds the local max/min frequency toggled by fTransient
int freqLock;

float intMax = 1.0;
float intMin = 0.1;
float intVal = 0.50;
float intLock;

bool isOn = false; //false = off; true = on;
int buttonPinState;
int lastbuttonPinState;
unsigned long sleepTimer = 0;
unsigned long awakeTimer = 0;
int timeOnOff = 700; //0.7 sec to consider starting up/shutting down
unsigned long timePressed = 0;


//DEEP SLEEP/////////
#include "driver/rtc_io.h"
#define BUTTON_PIN_BITMASK 0x200000000 // 2^33 in hex
const int buttonPin = 4;
bool buttonPinStateFromOn = true;

bool freqMode = true;//determines current mode: true = frequency sweep, false = intensity sweep
bool modeBeeped = false;//indicates if the mode indication beep has been performed (once)
float dutyCycle = 0.5;
float thresh = 0.12;
uint32_t offsetZ = 147500;
double adjustedZ;
double prevZ = -2;
bool calibrated = false; 
double zStart;
uint32_t currentZ;
double scaledZ;
bool rising = false;
bool prevRising = false;
bool falling = false;
bool prevFalling = false;

void setup(){
  Serial.begin(115200);
  print_wakeup_reason(); //Print the wakeup reason for ESP32
  esp_sleep_enable_ext0_wakeup(GPIO_NUM_4, 0); //1 = High, 0 = Low
  Wire.begin();
  vh.Init({&list,&core});
  pinMode(buttonPin, INPUT_PULLUP);
  avg.begin();
  myMag.begin();
  myMag.softReset();
  myMag.disableXChannel();
  updateOffset(&offsetZ);
  zSetup();
}

//For ESP Deep Sleep Functionality
void print_wakeup_reason() {
  esp_sleep_wakeup_cause_t wakeup_reason;
  wakeup_reason = esp_sleep_get_wakeup_cause();

  switch (wakeup_reason){
    case ESP_SLEEP_WAKEUP_EXT0 : Serial.println("Wakeup caused by external signal using RTC_IO"); break;
    default : Serial.printf("Wakeup was not caused by deep sleep: %d\n", wakeup_reason); break;
  }
}

double map(double x, double in_min, double in_max, double out_min, double out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

double getZ(){
  currentZ = myMag.getMeasurementZ();
  scaledZ = (double)currentZ - (double)offsetZ; // Convert to double _before_ subtracting
  scaledZ /= 131072.0;  
  return scaledZ; 
}

void updateOffset(uint32_t *offsetZ){
  bool success = true; // Use AND (&=) to record if any one command fails
  success &= myMag.performSetOperation(); // Perform the SET operation
  uint32_t setZ = 131072;
  setZ = myMag.getMeasurementZ(); // Read all three channels
  setZ = myMag.getMeasurementZ(); // Read all three channels
  success &= myMag.performResetOperation(); // Perform the RESET operation

  uint32_t resetZ = 131072;
  resetZ = myMag.getMeasurementZ(); // Read all three channels
  resetZ = myMag.getMeasurementZ(); // Do it twice - just in case there is noise on the first
  if (success)  {
    *offsetZ = (setZ + resetZ) / 2;
  }
}

void zSetup(){
  for(int i = 0 ; i < 20; i ++){
      double z = getZ();
      avg.reading(z);
  }
  avgVal = avg.getAvg();
  zStart = getZ();
}

void varivibeMain(){
  
  double z = getZ();
  double diffZ = (double)currentZ - (double)offsetZ; 
  if (calibrated == false &&  abs(diffZ) > 150){
    Serial.println("?????????");
    calibrated = true;
  }
  maxVal = (z > maxVal) ? z: maxVal;
  minVal = (z < minVal) ? z: minVal;
  adjustedZ = ( z < avgVal) ? map(z, minVal, avgVal, -1.0, 0.0) : map(z, avgVal, maxVal, 0.0, 1.0);
  
  
  if(prevZ < adjustedZ && adjustedZ > (avgVal + thresh) && calibrated){ //only works for rising edge

//      adjustedZ = pow(2000, (adjustedZ - 1));
      if(freqMode){
        freqVal = (int) map(adjustedZ, 0.0, 1.0, freqLock, fMax);
      }
      else{
        intVal = map(adjustedZ, avgVal, 1.0, intLock, intMin);
      }
  }
  else if(prevZ > adjustedZ && adjustedZ < (avgVal - thresh) && calibrated) {//falling edge

//      adjustedZ = -1.0*pow(2000, ((-1.0)*adjustedZ) - 1));
      if(freqMode){
        freqVal = (int) map(adjustedZ, 0.0, -1.0, freqLock, fMin);
      }
      else{
        intVal = map(adjustedZ, avgVal, -1.0, intLock, intMax);
      }
  }
  else if (abs(adjustedZ) < thresh && calibrated){
    Serial.println("Here!!!!!!!!!!!!!");
    freqLock = freqVal;
    intLock = intVal;
  }
  
//  if( (!prevFalling && falling) || (!prevRising) && rising){
//    Serial.println("Here!!!!!!!!!!!!!");
//        freqLock = freqVal;
//        intLock = intVal;
//   }
//   
//   prevRising = rising;
//   prevFalling = falling;
  
  prevZ = adjustedZ;
  vh.play({VHVIBRATE(freqVal, intVal, (2000.0/freqVal), dutyCycle, 0)}, "Finger");
}

void turnPinOff(){
    Serial.println("Disable motor and haptic drivers");
    digitalWrite(32, LOW); //driver sleep pin
    digitalWrite(15, LOW);
    // Make sure the LEDs are off
    digitalWrite(13, LOW); //RED
    digitalWrite(14, LOW); //GREEN
    digitalWrite(12, LOW); //YELLOW
  
    gpio_hold_en((gpio_num_t) 15);
    gpio_hold_en((gpio_num_t) 32);
    gpio_hold_en((gpio_num_t) 13);
    gpio_hold_en((gpio_num_t) 14);
    gpio_hold_en((gpio_num_t) 12);
}

void turnPinOn(){
    Serial.println("Enable motor and haptic drivers");
    gpio_hold_dis((gpio_num_t) 15);
    gpio_hold_dis((gpio_num_t) 32);
    gpio_hold_dis((gpio_num_t) 13);
    gpio_hold_dis((gpio_num_t) 14);
    gpio_hold_dis((gpio_num_t) 12);
    
    digitalWrite(32, HIGH); 
    digitalWrite(15, HIGH);
    digitalWrite(13, HIGH ); //LED red
    digitalWrite(14, HIGH); //green
    digitalWrite(12, HIGH); //yellow

}

void turnOFF() { //"turn off" effect
  if (isOn == true){
    for (int i = 200; i > 0; i -= 30) {
      vh.play({VHVIBRATE(i, 0.5, 2000 / i, dutyCycle, 0)}, "Finger");
    }
    vh.play({VHPAUSE(150), VHPULSE(300,12)}, "Finger");
  }
  deepSleep();
}

void deepSleep(){ //only call deepSleep by itself when it's awaken but device not On
    turnPinOff();
    while(buttonPinState == LOW && millis()- sleepTimer < 15000){
      buttonPinState =  digitalRead(buttonPin);
    }
    esp_deep_sleep_start(); //put into deep sleep
}

void modeSwitchBeep(){
  if (modeBeeped == false && freqMode) {      //beeping indicating the mode
    for (int i = 40; i < 300; i = i + 30) {
        vh.play({VHVIBRATE(i, 0.5, (2000.0 / i), dutyCycle, 0)}, "Finger");
    }
      modeBeeped = true;
      delay(190);
  }
  else if (modeBeeped == false && !freqMode) {    
    delay(50);
    vh.play({VHVIBRATE(100, 0.5, (2000.0/100), dutyCycle, 0)}, "Finger");
    delay(50);
    vh.play({VHVIBRATE(100, 0.25, (2000.0/100), dutyCycle, 0)}, "Finger");
    modeBeeped = true;
    delay(190);
  }
}

void buttonMode(){
  unsigned long totalTime = 0;
  timePressed = millis();
  while (buttonPinState == LOW && totalTime < timeOnOff){
    Serial.println("Button pressing"); //DONT DELETE
    buttonPinState = digitalRead(buttonPin);
    totalTime = millis() - timePressed;
    if(isOn){
      varivibeMain(); 
    }
  }

  if (totalTime < timeOnOff && isOn) { //include debounce period

    modeSwitchBeep();
    freqMode = !freqMode;
    modeBeeped = false;
  }
  else if(totalTime > timeOnOff){  
    isOn  = !isOn;
    if(isOn){//if On;
      buttonPinStateFromOn = false; 
      turnPinOn();
    }
    else{
      sleepTimer = millis();
      turnOFF();
    }
  }
}

void loop() {
  
//    findZ = (currentZ >  findZ) ? currentZ :  findZ;
    
//    Serial.print("calibrated: ");
//    Serial.println(calibrated);
  buttonPinState = digitalRead(buttonPin);
  if (!buttonPinStateFromOn && buttonPinState ==HIGH){
    buttonPinStateFromOn = true;
  }
  
  if (buttonPinState == LOW && buttonPinStateFromOn){
    buttonMode();
    
  }
  if(isOn){
    awakeTimer = millis();
    varivibeMain(); 
//    Serial.print("freqMode: ");
//    Serial.println(freqMode);
//    Serial.print("intVal: ");
//    Serial.println(intVal);
//    Serial.print("freqVal: ");
//    Serial.println(freqVal);
//
//
//    Serial.print("currentZ: ");
//    Serial.println(currentZ);    
      Serial.print(freqVal);
      Serial.print(",");
      Serial.println(adjustedZ*300.0);
//    Serial.print("freqVal: ");
//    Serial.println(freqVal);
//    Serial.print("adjustedZ: ");
//    Serial.println(adjustedZ*300.0);
    
  }
  if(isOn == false &&  millis() - awakeTimer > 2500){ //leave on for 2.5 before going into deepsleep
    deepSleep();
  }
}

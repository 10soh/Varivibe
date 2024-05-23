#include "VectorHaptics.h"
#include "Esp32PicoMini.h"
#include <movingAvgFloat.h>
#include <VHChannels.h>
#include <VHBasePrimitives.h>
VectorHaptics<Esp32PicoMini> vh;
VHBasePrimitives core;

VHChannel chnl1(1, 25,{"Left channel", "Channel 1", "Left", "Finger"});
VHChannel chnl2(2, 26,{"Right channel", "Channel 2", "Right", "Finger"});
VHChannels list({&chnl1,&chnl2});

#include <Wire.h>
#include <SparkFun_MMC5983MA_Arduino_Library.h> //Click here to get the library: http://librarymanager/All#SparkFun_MMC5983MA
SFE_MMC5983MA myMag;
#include <Preferences.h>
Preferences preferences;

double maxVal;
double minVal;
double avgVal = 0;

int fMax = 300;
int fMin = 40;
int freqVal = 0.5 * (fMax + fMin); //holds the local max/min frequency toggled by fTransient
int freqLock;

float intMax = 1.0;
float intMin = 0.1;
float intVal = 0.50;
float intLock;

bool isOn = false; //false = off; true = on;
int buttonState = LOW;
int lastButtonState;
unsigned long sleepTimer = 0;
unsigned long awakeTimer = 0;
int timeOnOff = 700; //0.7 sec to consider starting up/shutting down
unsigned long timePressed = 0;
bool buttonFromOn = false;

//DEEP SLEEP/////////
#include "driver/rtc_io.h"
#define BUTTON_PIN_BITMASK 0x200000000 // 2^33 in hex
const int buttonPin = 4;
bool buttonPinStateFromOn = true;
long lastDebounceTime = 0;  // the last time the output pin was toggled
long debounceDelay = 50;

bool freqMode = true;//determines current mode: true = frequency sweep, false = intensity sweep
bool modeBeeped = false;//indicates if the mode indication beep has been performed (once)
float dutyCycle = 0.5;
float thresh = 0.20;
float fallingThresh = 0.30;
uint32_t offsetZ = 147500;
double adjustedZ;
double prevZ = -2;
uint32_t currentZ;
double scaledZ;

void setup(){
  Serial.begin(115200);
  print_wakeup_reason(); //Print the wakeup reason for ESP32
  esp_sleep_enable_ext0_wakeup(GPIO_NUM_4, 0); //1 = High, 0 = Low
  Wire.begin();
  vh.Init({&list,&core});
  pinMode(buttonPin, INPUT_PULLUP);
  myMag.begin();
  myMag.softReset();
  myMag.disableXChannel();
  updateOffset(&offsetZ);
  preferenceSetup();
}

void preferenceSetup(){
  preferences.begin("calibrationVal", false);//begins a storage space in memory to permenantely store calibrated toggle range
  minVal = preferences.getDouble("minVal", 0.0);
  minVal *= 0.9;
  maxVal = preferences.getDouble("maxVal", 0.0);
  maxVal *= 0.9;
  preferences.end();
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

void varivibeMain(){
  
  double z = getZ();
  maxVal = (z > maxVal) ? z: maxVal;
  minVal = (z < minVal) ? z: minVal;
  adjustedZ = ( z < avgVal) ? map(z, minVal, avgVal, -1.0, 0.0) : map(z, avgVal, maxVal, 0.0, 1.0);
  
  if(prevZ < adjustedZ && adjustedZ > (avgVal + thresh)){ //only works for rising edge
      Serial.println("Rising Edge");
      if(freqMode){
        freqVal = (int) map(adjustedZ, thresh, 1.0, freqLock, fMax);
      }
      else{
        intVal = map(adjustedZ, thresh, 1.0, intLock, intMin);
      }
  }
  else if(prevZ > adjustedZ && adjustedZ < (avgVal - fallingThresh)) {
      Serial.println("Falling Edge");
      if(freqMode){
        freqVal = (int) map(adjustedZ, -fallingThresh, -1.0, freqLock, fMin);
      }
      else{
        intVal = map(adjustedZ, -fallingThresh, -1.0, intLock, intMax);
      }
  }
  else if (abs(adjustedZ) < thresh ){
    Serial.println("-----Resting State-----");
    freqLock = freqVal;
    intLock = intVal;
  }

  prevZ = adjustedZ;
  vh.play({VIBRATE(freqVal, intVal, (2000.0/freqVal), dutyCycle, 0)}, "Finger");
}

void turnPinOff(){
    Serial.println("Disable motor and haptic drivers");
    digitalWrite(32, LOW); //driver sleep pin
    digitalWrite(15, LOW);
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
  //save values
  preferences.begin("calibrationVal", false);
  preferences.putDouble("minVal", minVal);
  preferences.putDouble("maxVal", maxVal);
  preferences.end();
  if (isOn == true){
    for (int i = 200; i > 0; i -= 30) {
      vh.play({VIBRATE(i, 0.5, 2000 / i, dutyCycle, 0)}, "Finger");
    }
    vh.play({PAUSE(150), PULSE(300,12)}, "Finger");
  }
  deepSleep();
}

void deepSleep(){ //only call deepSleep by itself when it's awaken but device not On
    turnPinOff();
    while(buttonState == LOW && millis()- sleepTimer < 15000){
      buttonState =  digitalRead(buttonPin);
    }
    esp_deep_sleep_start(); //put into deep sleep
}

void modeSwitchBeep(){
  if (modeBeeped == false && freqMode) {      //beeping indicating the mode
    for (int i = 40; i < 300; i = i + 30) {
      vh.play({VIBRATE(i, 0.5, (2000.0 / i), dutyCycle, 0)}, "Finger");
    }
      modeBeeped = true;
      delay(190);
  }
  else if (modeBeeped == false && !freqMode) {    
    delay(50);
    vh.play({VIBRATE(100, 0.5, (2000.0/100), dutyCycle, 0)}, "Finger");
    delay(50);
    vh.play({VIBRATE(100, 0.25, (2000.0/100), dutyCycle, 0)}, "Finger");
    modeBeeped = true;
    delay(190);
  }
}

void turnOnOff(){  
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

void buttonMode(){
  Serial.println("Button Mode -----");
  if (buttonState == LOW){ //when pressed, set off timer
    timePressed = millis();
  }
  else if(isOn && !buttonFromOn){
      modeSwitchBeep();
      freqMode = !freqMode;
      modeBeeped = false;
  }
}

void loop() {
  buttonState = digitalRead(buttonPin);
  unsigned long totalTime = millis()- timePressed;
  if ((millis() - lastDebounceTime) > debounceDelay && buttonState!=lastButtonState) {
     //trig when button state changes + debounce consideration
     buttonMode();    
     lastDebounceTime = millis();
  }
  if(totalTime > timeOnOff){
    buttonFromOn = true;
    turnOnOff();
    timePressed = millis();
  }
  if(buttonState == HIGH){
    timePressed = millis();  
    buttonFromOn = false;
  }
  if(isOn){
    awakeTimer = millis();
    varivibeMain(); 
  }
  if(isOn == false &&  millis() - awakeTimer > 2500){ //leave on for 2.5 before going into deepsleep
    deepSleep();
  }
  lastButtonState = buttonState;
}

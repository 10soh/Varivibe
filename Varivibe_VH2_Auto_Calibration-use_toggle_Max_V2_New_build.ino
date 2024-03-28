#include "VectorHaptics.h"
#include "Esp32PicoMini.h"

// Importing Required header files
#include <VHChannelList.h>
#include <VHCore.h>

// Creating API object
VectorHaptics<Esp32PicoMini> vh;
//Creating object of core
VHCore core;

// Creating mono channel with channel number, gpio pin, channel tags
VHChannel chnl1(1, 25,{"Left channel", "Channel 1", "Left", "Finger"});
// Creating mono channel with channel number, gpio pin, channel tags
VHChannel chnl2(2, 26,{"Right channel", "Channel 2", "Right", "Finger"});
VHChannelList list({&chnl1,&chnl2});

#include <CircularBuffer.h>
#include <Preferences.h> //the library used to save data to memory
Preferences preferences;

TaskHandle_t handle = NULL;


#include <Wire.h>
#include <SparkFun_MMC5983MA_Arduino_Library.h> //Click here to get the library: http://librarymanager/All#SparkFun_MMC5983MA
SFE_MMC5983MA myMag1;
int currentX1 = 0;
int currentY1 = 0;
int currentZ1 = 0;

int rockerHigh; //the upper limit value that the rocker switch reads (neutral at 27224)
int rockerLow;//the lower limit value that the rocker switch reads
int rockerMid;
int togglePos = 0.0;
int toggleMax;//the local max value to temporarily store max mag reading
int toggleMin;//the local min value to temporarily store max mag reading

float fMax = 300;
float fMin = 40;
float fHold1 = 0.5 * (fMax + fMin); //holds the local max/min frequency toggled by fTransient
float fHold2 = fHold1; //holds the previously reached min/max value as a starting point
float fTransient = fHold1; //the transient frequency value to temporarily set fHold1 value

float intMax = 1.0;
float intMin = 0.1;
float intHold1 = (intMax + intMin) * 0.5;
float intHold2 = intHold1;
float intTransient = intHold1;

int calibrationVal; //the raw mag reading everytime the motor turns on (while the rocker is at neutral)
bool deviceOn = false; //indicates if the device is currently on

const int buttonPin = 4;
bool buttonPressed = false;
unsigned long buttonTimer = 0;
unsigned long sleepTimer = 0;
unsigned long awakeTimer = 0;
int buttonHoldDur = 700;
int buttonState; 

bool freqSweepMode = true;//determines current mode: true = frequency sweep, false = intensity sweep
bool modeBeeped = true;//indicates if the mode indication beep has been performed (once)

float dutyCycle = 0.5;
float thresholdPercentage = 0.12;

//Auto Calibration Function Parameters
#define BUFFER_SIZE 5
CircularBuffer<int, BUFFER_SIZE> buffer1;//buffer array for rockerLow averaging
CircularBuffer<int, BUFFER_SIZE> buffer2;//buffer array for rockerHigh averaging
CircularBuffer<int, BUFFER_SIZE> buffer3; //buffer array for rockerMid averaging
int rockerHighAvg; //the starting average value of rockerHigh array
int rockerLowAvg; //the starting average value of rockerLow array
int rockerMidAvg;//the global average value of rockerMid, stored to memory;
int pushCounterHigh = 0;
int pushCounterLow = 0;
int pushCounterLimit = 10;

int upperRange;//the permanent range value -distance between rockerMid and rockerHigh/rockerLow
int lowerRange;

int incThres;
int decThres;//the threshold values that triggers the increase/decrease in fTransient

//DEEP SLEEP/////////
#include "driver/rtc_io.h"
#define BUTTON_PIN_BITMASK 0x200000000 // 2^33 in hex

void setup(){
  vh.Init({&list,&core});
  preferences.begin("toggleRange", false);//begins a storage space in memory to permenantely store calibrated toggle range
  upperRange = preferences.getInt("upperRange", 5500);
  lowerRange = preferences.getInt("lowerRange", 2500);
  preferences.end();

  pinMode(buttonPin, INPUT_PULLUP);
  Serial.begin(115200);
  Wire.begin();

  myMag1.begin();
  if (myMag1.begin() == false){
    Serial.println("MMC5983MA did not respond - check your wiring. Freezing.");
    while (true);
  }

  myMag1.softReset();
  myMag1.disableXChannel();//disable X channel because it's not being used

  // For more info about deep sleep: https://randomnerdtutorials.com/esp32-external-wake-up-deep-sleep/
  print_wakeup_reason(); //Print the wakeup reason for ESP32
  esp_sleep_enable_ext0_wakeup(GPIO_NUM_4, 0); //1 = High, 0 = Low
}


// Function to calculate the average of an array
int calculateAverage(CircularBuffer<int, BUFFER_SIZE> &buffer) {
  int sum = 0;
  int count = buffer.size();

  //summing elements
  for (int i = 0; i <  buffer.size(); i++) {
    sum += buffer[i];
  }

  // Calculate and return the average
  if (count > 0) { return sum / count; }
  return 0; // Return 0 if the buffer is empty
  
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

//Re-maps a number from one range to another
float map(float x, float in_min, float in_max, float out_min, float out_max){
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void deepSleep(){
    rampDown();
    deviceOn = false;
    fHold1 = 0.5 * (fMax + fMin);//when device is turned off using the button, the frequency returns back to mid point the next time it turns on
    freqSweepMode = false;//always have freqSweepMode set to false while turning off, so that it turns on with the same mode everytime
    modeBeeped = false;
    delay(50);
    myMag1.disableYZChannels();//disable YZ channels to save power while esp goes to sleep 
    Serial.println(myMag1.areYZChannelsEnabled());
    Serial.println("Going to sleep now");
    turnPinOff();
    while(buttonState == LOW && millis()- sleepTimer < 15000){
      buttonState =  digitalRead(buttonPin);
    }
    esp_deep_sleep_start(); //put into deep sleep
}

void turnPinOff(){
    Serial.println("Disable motor and haptic drivers");
    digitalWrite(32, LOW); //driver sleep pin
    digitalWrite(15, LOW);
    // Make sure the LEDs are off
    digitalWrite(13, LOW); //RED
    digitalWrite(14, LOW); //GREEN
    digitalWrite(2, LOW); //YELLOW
  
    gpio_hold_en((gpio_num_t) 15);
    gpio_hold_en((gpio_num_t) 32);
    gpio_hold_en((gpio_num_t) 13);
    gpio_hold_en((gpio_num_t) 14);
    gpio_hold_en((gpio_num_t) 2);
}

void turnPinOn(){
    Serial.println("Enable motor and haptic drivers");
    gpio_hold_dis((gpio_num_t) 15);
    gpio_hold_dis((gpio_num_t) 32);
    gpio_hold_dis((gpio_num_t) 13);
    gpio_hold_dis((gpio_num_t) 14);
    gpio_hold_dis((gpio_num_t) 2);
    
    digitalWrite(32, HIGH); 
    digitalWrite(15, HIGH);
    digitalWrite(13, HIGH ); //LED red
    digitalWrite(14, HIGH); //green
    digitalWrite(2, HIGH); //yellow

}

void rampDown(){
  if (deviceOn == true){
    for (int i = 200; i > 0; i -= 30) {
      vh.play({VHVIBRATE(i, 0.5, 2000 / i, dutyCycle, 0)}, "Finger");
//      vh.vibrate(i, 0.5, 2000 / i, dutyCycle, 0, 0);
    }
    vh.play({VHPAUSE(150), VHPULSE(300,12)}, "Finger");
//    vh.pause(150);
//    vh.pulse(300, 12);
  }
}

void buttonMode(){
  buttonTimer = millis();
  buttonPressed = true;
  if (deviceOn == true) { //if device on; either switch mode or go to sleep
     Serial.println("22222222222222222222");
    while(buttonState != HIGH){ //not released 
      Serial.print("-------------------------fHOLD1: ");
      Serial.println(fHold1);
      vh.play({VHVIBRATE(fHold1, intHold1, (2000.0/fHold1), dutyCycle, 0)}, "Finger");
//      vh.vibrate(fHold1, intHold1, 50, dutyCycle, 0, 0);
      if (millis() - buttonTimer > buttonHoldDur){ //Go to deepsleep
        Serial.println("In DEEP SLEEP MODE");
        sleepTimer = millis();
        deepSleep();
      }
      buttonState =  digitalRead(buttonPin);
    }
    modeSwitchBeep();
    freqSweepMode = !freqSweepMode;
    modeBeeped = false;
    buttonPressed = false;
  }
  else{ //device is off; want to turn it on
    //wait until button is released
     Serial.println("3333333333333333333333333");
    while (buttonState != HIGH) {
      if (millis() - buttonTimer > buttonHoldDur){ 
        deviceOn = true;
        turnPinOn();
        myMag1.enableYZChannels();//enable magnotometer
        delay(50);
        rockerCalibration();
        break;
      }
      buttonState = digitalRead(buttonPin);
    }
  }
}

void rockerCalibration(){
    rockerMid = myMag1.getMeasurementZ();//set middle rocker reading when system starts
    buffer3.push(rockerMid); //push rockerMid value into buffer3 to get averaged rockMid - rockerMidAvg
    rockerMid = calculateAverage(buffer3);
    rockerHigh = rockerMid + upperRange; //the starting upper limit value that will be updated through auto calibration (about 50% of the approximate real avlue)
    rockerLow = rockerMid - lowerRange; //the starting lower limit value that will be updated through auto calibration (about 50% of the approximate real avlue)
    buffer1.push(rockerLow);
    buffer2.push(rockerHigh);
    rockerHighAvg = rockerHigh;
    rockerLowAvg = rockerLow;
    thresholdAssign();
}

void thresholdAssign(){
    incThres = rockerMid + thresholdPercentage * (rockerHigh - rockerMid); //the threshold values that triggers the increment in fTransient
    decThres = rockerMid - thresholdPercentage * (rockerMid - rockerLow); //the threshold values that triggers the decrement in fTransient
    toggleMax = incThres; //set initial toggle max value at incThres
    toggleMin = decThres; //set initial toggle min value at decThres
}

void modeSwitchBeep(){
  if (modeBeeped == false && freqSweepMode) {      //beeping indicating the mode
    for (int i = 40; i < 300; i = i + 30) {
        vh.play({VHVIBRATE(i, 0.5, (2000.0 / i), dutyCycle, 0)}, "Finger");
//      vh.vibrate(i, 0.5, 2000 / i, dutyCycle, 0, 0);
    }
      modeBeeped = true;
      delay(190);
  }
  else if (modeBeeped == false && !freqSweepMode) {    
    delay(50);
    vh.play({VHVIBRATE(100, 0.5, (2000.0/100), dutyCycle, 0)}, "Finger");
//    vh.vibrate(100, 0.5, 80, dutyCycle, 0, 0);
    delay(50);
    vh.play({VHVIBRATE(100, 0.25, (2000.0/100), dutyCycle, 0)}, "Finger");
//    vh.vibrate(100, 0.25, 80, dutyCycle, 0, 0);
      modeBeeped = true;
    delay(190);
  }
}

void storeRange(){
    upperRange = rockerHigh - rockerMid;//stores updated upper and lower range into the memory so next time the latest value is being read
    lowerRange = rockerMid - rockerLow;
    preferences.begin("toggleRange", false);
    preferences.putInt("upperRange", upperRange);
    preferences.putInt("lowerRange", lowerRange);
    preferences.end();
}


void freqMode(){
    if (togglePos > incThres)  { //linearly increase toggleMax to a new value if togglePos goes larger
      Serial.println("F: toggleMax");
      if (togglePos > toggleMax) {
        toggleMax = togglePos;
        fTransient = map(toggleMax, incThres, rockerHigh, fHold2, fMax);
        fTransient =  fTransient > fMax? fMax: fTransient;
        Serial.println(fTransient);
        fHold1 = (0.00345 * sq(fTransient) + 9.66);
      }
      fHold1 =  fHold1 > fMax ? fMax: fHold1;//curve equation to replace the linear mapping
      //2000/fHold1 (in millisecond) represents the duration that is 2 full cycles of vibration based on the input frequency
    }
    else if (togglePos < decThres)  {//decrease toggleMin to a new value if togglePos goes smaller
      Serial.println("F: toggleMin"); 
      if (togglePos < toggleMin) {
        toggleMin = togglePos;
        fTransient = map(toggleMin, decThres, rockerLow, fHold2, fMin) ;
        fTransient = fTransient < fMin ?  fMin: fTransient;
        Serial.println(fTransient);
        fHold1 = (0.00345 * sq(fTransient) + 9.66);
      }
      fHold1 = fHold1 < fMin ? fMin: fHold1;//curve equation to replace the linear mapping
    }
    else if (togglePos > decThres && togglePos < incThres) { //reset function: when toggle moves back within the thresholds, reset
      Serial.println("F: In Between Values"); 
      fHold2 = sqrt((fHold1 - 9.66) / 0.00345);//curve equation (reset) to replace the linear mapping
      if (toggleMax > rockerHighAvg && pushCounterHigh <= pushCounterLimit) { //push new max value to rockerHigh buffer if it is higher than average
        buffer2.push(toggleMax);
        rockerHighAvg = calculateAverage(buffer2);
        rockerHigh = rockerHighAvg;
        pushCounterHigh = pushCounterHigh + 1;
      }
      else if (pushCounterHigh > pushCounterLimit && toggleMax > rockerMid + 0.95 * (rockerHigh - rockerMid)) { //after pushCounterLimit times, only push value within 95%
        buffer2.push(toggleMax);
        rockerHighAvg = calculateAverage(buffer2);
        rockerHigh = rockerHighAvg;
      }

      if (toggleMin < rockerLowAvg &&  pushCounterLow <= pushCounterLimit) {//push new min value to rockerLow buffer if it is lower than average
        buffer1.push(toggleMin);
        rockerLowAvg = calculateAverage(buffer1);
        rockerLow = rockerLowAvg;
        pushCounterLow = pushCounterLow + 1;
      }
      else if (pushCounterHigh > pushCounterLimit && toggleMin < rockerMid - 0.95 * (rockerMid - rockerLow)) {//after pushCounterLimit times, only push value within 95%
        buffer1.push(toggleMin);
        rockerLowAvg = calculateAverage(buffer1);
        rockerLow = rockerLowAvg;
      }
      
      thresholdAssign();
    }
    storeRange();
}

void intensityMode(){
    if (togglePos > incThres){ //linearly increase toggleMax to a new value if togglePos goes larger
      Serial.println("I: toggleMax"); 
      if (togglePos > toggleMax) {
        toggleMax = togglePos;
        intTransient = map(toggleMax, incThres, rockerHigh, intHold2, intMin) ;
        intTransient = intTransient > intMax ? intMax: intTransient;
      }
      intHold1 = intTransient  > intMax ? intMax : intTransient;
      //2000/fHold1 (in millisecond) represents the duration that is 2 full cycles of vibration based on the input frequency
    }
    else if (togglePos < decThres){//decrease toggleMin to a new value if togglePos goes smaller
      Serial.println("I: toggleMin"); 
      if (togglePos < toggleMin) {
        toggleMin = togglePos;
        intTransient = map(toggleMin, decThres, rockerLow, intHold2, intMax);
        intTransient = intTransient < intMin ? intMin : intTransient;
      }
      intHold1 = intTransient > intMax ? intMax : intTransient;
    }
    else if (togglePos > decThres && togglePos < incThres) { //reset function: when toggle moves back within the thresholds, reset
      Serial.println("I: In Between Values"); 
      intHold2 = intHold1;
      thresholdAssign();
    }
    storeRange();
}

void loop(){
  buttonState = digitalRead(buttonPin);
  Serial.println("RUNNING_----------------------------------");
  if (buttonState == LOW && buttonPressed == false){//button pressed detected
    Serial.println("11111111111111111");
    buttonMode();  
  }
  else if(buttonState == HIGH){ //caused from opening
    buttonPressed = false;
  }

  if(deviceOn == false &&  millis() -awakeTimer > 2500){
    deepSleep();
  }
  if (deviceOn == true){
    awakeTimer = millis();
    togglePos = myMag1.getMeasurementZ();// raw mag reading 
    if(freqSweepMode){
      Serial.println("4444444444444444444444");
      freqMode();
    }
    else{
      Serial.println("55555555555555555555555");
      intensityMode();
    }
    vh.play({VHVIBRATE(fHold1, intHold1, (2000.0/fHold1), dutyCycle, 0)}, "Finger");
//    vh.endVibration(); 
//    vh.vibrate(fHold1, intHold1, 25, dutyCycle, 0, 0);
    Serial.print(">>>>>>>>>>>>>>>>>>>>>>>>fHOLD1: ");
    Serial.println(fHold1);
    Serial.print(">>>>>>>>>>>>>>>>>>>>>>>>intHOLD1: ");
    Serial.println(intHold1);
  }
}

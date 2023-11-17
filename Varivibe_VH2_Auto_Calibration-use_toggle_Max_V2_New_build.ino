//12345
//#include "VHMasterPack.h"
#include "VHBasicHapticCmds.h"
#include "VHBasicFuncGenerator.h"
#include "VHHeaders.h" // import this header at last
#include "BluetoothAudioDev.h"
#define VH_NAME "VHDevice"
#define LOG_SERIAL true
Pam8403 DrvPam8403;
VectorHaptics<Esp32PicoMini> vh(&DrvPam8403);
#define PIN1 21
#define PIN2 19
#define MASTER_PIN 22

#include <CircularBuffer.h>
#include <Preferences.h> //the library used to save data to memory
Preferences preferences;

BluetoothAudioDev btDev(VH_NAME);
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

float intMax = 1;
float intMin = 0.3;
float intHold1 = (intMax + intMin) * 0.5;
float intHold2 = intHold1;
float intTransient = intHold1;

int calibrationVal; //the raw mag reading everytime the motor turns on (while the rocker is at neutral)
bool deviceOn = false; //indicates if the device is currently on
bool lastDeviceOn = false;//indicates the last device status when the button is being released (to avoid mode switch during launching)

const int buttonPin = 4;
bool buttonPressed = false;
unsigned long buttonTimer = 0;
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

int threshold1;
int threshold2;//the threshold values that triggers the increase/decrease in fTransient
//DEEP SLEEP/////////
#include "driver/rtc_io.h"
#define BUTTON_PIN_BITMASK 0x200000000 // 2^33 in hex

void setup(){
  R_CHANNEL rightChannel;
  rightChannel.ENABLE_ALL = true;
  rightChannel.OUT_PIN = "R+/R-";
  rightChannel.CHANNEL_DESC = "Right";

  L_CHANNEL leftChannel;
  leftChannel.ENABLE_ALL = true;
  leftChannel.OUT_PIN = "L+/L-";
  leftChannel.CHANNEL_DESC = "Left";

  M_CHANNEL motorChannel;
  motorChannel.ENABLE_ALL = true;
  motorChannel.OUT_PIN = "M+/M-";
  motorChannel.CHANNEL_DESC = "Hand";

  BOARD_DESC info;
  info.pR_CHANNEL = &rightChannel;
  info.pL_CHANNEL = &leftChannel;
  info.pM_CHANNEL = &motorChannel;

  const int INFO_SIZE = 3;
  ADDITIONAL_INFO addInfo[INFO_SIZE];
  addInfo[0] = {"Motor Type", "Tack Hammer"};
  addInfo[1] = {"Resonant Frequency", "1kHz"};
  addInfo[2] = {"Gmin / Gmax", "1.234"};

  info.pADDITIONAL_INFO = addInfo;
  info.SIZE_OF_ADDITIONAL_INFO = INFO_SIZE;

  DeviceInfo devInfo;
  devInfo.setDeviceName(VH_NAME);
  devInfo.setManufacturer("TitanHaptics");
  devInfo.setSerialNumber("1234567890");
  devInfo.setModelNumber("1234567890");
  devInfo.setDeviceMode(MCU_MODE::DEVELOPMENT);
  devInfo.setConnType(ConnType::USB_CONN, CMD_MODE::STRING_MODE);

  preferences.begin("toggleRange", false);//begins a storage space in memory to permenantely store calibrated toggle range
  upperRange = preferences.getInt("upperRange", 5500);
  lowerRange = preferences.getInt("lowerRange", 2500);
  preferences.end();

  vh.Init(&info, &devInfo);
  vh.setF0(50); // Sets the default resonant frequency of the haptic actuator.  Affects VH primitives. Default is 100Hz
  vh.setMinMax(0, 1);    // setMinMax(float min, float max) values between 0 and 1
  vh.adjustTiming(1.16); // multiplier to correct timing.  Values < 1 speed up timing, and > 1 slow it down.  Depending on your clock sources and MCU, you may need to adjust this parameter to correct for deviations.  NOTE: May not impact all frequencies equally!
  vh.turnOnBuiltinLED(true);   // to make sure the new OS is in effect turn on the builtin LED
  vh.EnableDac(); // Enable the DAC
  vh.TurnOnDrv();
  vh.TurnOnPam(); // turn on the Pam amplifier
  vh.setWaveCalibFactor(0.02);

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
  vh.logAllMessages(false);

  // For more info about deep sleep: https://randomnerdtutorials.com/esp32-external-wake-up-deep-sleep/
  print_wakeup_reason(); //Print the wakeup reason for ESP32

//-------------------THIS NEEDS TO BE CHECKED IF ITS NEEDED!!----------------------------- if yes, remove the if - endif
  turnPinOff();
//-------------------THIS NEEDS TO BE CHECKED IF ITS NEEDED!!----------------------------- if yes, remove the if - endif
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
  fHold1 = 0.5 * (fMax + fMin);//when device is turned off using the button, the frequency returns back to mid point the next time it turns on
  freqSweepMode = false;//always have freqSweepMode set to false while turning off, so that it turns on with the same mode everytime
  modeBeeped = false;
  delay(50);
  myMag1.disableYZChannels();//disable YZ channels to save power while esp goes to sleep 
  Serial.println(myMag1.areYZChannelsEnabled());
  Serial.println("Going to sleep now");
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

    //enable magnotometer
    myMag1.enableYZChannels();
    Serial.println(myMag1.areYZChannelsEnabled());
}

void buttonMode(){
  buttonTimer = millis();
  buttonPressed = true;
  if (deviceOn == true) { //if device on; either switch mode or go to sleep
    while(buttonState != HIGH){ //not released  
      if (millis() - buttonTimer > buttonHoldDur){ //Go to deepsleep
        Serial.println("In DEEP SLEEP MODE");
        deepSleep();
      }
      buttonState =  digitalRead(buttonPin);
    }
    freqSweepMode = !freqSweepMode;
  }
  else{ //device is off; want to turn it on
    //wait until button is released
    while (buttonState != HIGH) {
      if (millis() - buttonTimer > buttonHoldDur){ 
        deviceOn = true;
        turnPinOn();
        delay(50);
        rockerCalibration();
      }
      buttonState = digitalRead(buttonPin);
    } 
    if (millis() - buttonTimer < buttonHoldDur){
      deepSleep();
    }
  }
  
  buttonPressed = false;
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
    threshold1 = rockerMid + thresholdPercentage * (rockerHigh - rockerMid); //the threshold values that triggers the increment in fTransient
    threshold2 = rockerMid - thresholdPercentage * (rockerMid - rockerLow); //the threshold values that triggers the decrement in fTransient
    toggleMax = threshold1; //set initial toggle max value at threshold1
    toggleMin = threshold2; //set initial toggle min value at threshold2
}
void modeSwitchBeep(){
  if (modeBeeped == false) {      //beeping indicating the mode

    for (int i = 40; i < 300; i = i + 30) {
      vh.vibrate(i, 0.5, 2000 / i, dutyCycle, 0, 0);
    }
  
    modeBeeped = true;
    delay(190);
  }
}

void storeRange(){
//    Serial.println(fHold1);
//    Serial.println(intHold1);

    upperRange = rockerHigh - rockerMid;//stores updated upper and lower range into the memory so next time the latest value is being read
    lowerRange = rockerMid - rockerLow;
    preferences.begin("toggleRange", false);
    preferences.putInt("upperRange", upperRange);
    preferences.putInt("lowerRange", lowerRange);
    preferences.end();
}


void freqMode(){
    modeSwitchBeep();

    if (togglePos > threshold1)  { //linearly increase toggleMax to a new value if togglePos goes larger
      Serial.println("F: toggleMax");
      if (togglePos > toggleMax) {
        toggleMax = togglePos;
        fTransient = map(toggleMax, threshold1, rockerHigh, fHold2, fMax);
        fTransient =  fTransient > fMax? fMax: fTransient;
        Serial.println(fTransient);
        fHold1 = (0.00345 * sq(fTransient) + 9.66);
      }
      fHold1 =  fHold1 > fMax ? fMax: fHold1;//curve equation to replace the linear mapping
      //2000/fHold1 (in millisecond) represents the duration that is 2 full cycles of vibration based on the input frequency
      vh.vibrate(fHold1, intHold1, 2000 / fHold1, dutyCycle, 0, 0); //vibrate using the recent fHold1 value
    }
    else if (togglePos < threshold2)  {//decrease toggleMin to a new value if togglePos goes smaller
      Serial.println("F: toggleMin"); 
      if (togglePos < toggleMin) {
        toggleMin = togglePos;
        fTransient = map(toggleMin, threshold2, rockerLow, fHold2, fMin) ;
        fTransient = fTransient < fMin ?  fMin: fTransient;
        Serial.println(fTransient);
        fHold1 = (0.00345 * sq(fTransient) + 9.66);
      }
      fHold1 = fHold1 < fMin ? fMin: fHold1;//curve equation to replace the linear mapping
      vh.vibrate(fHold1, intHold1, 2000 / fHold1, dutyCycle, 0, 0); //vibrate using the recent fHold1 value
    }
    else if (togglePos > threshold2 && togglePos < threshold1) { //reset function: when toggle moves back within the thresholds, reset
      Serial.println("F: In Between Values"); 
      vh.vibrate(fHold1, intHold1, 2000 / fHold1, dutyCycle, 0, 0); // vibration output when toggle switch reset
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

}

void intensityMode(){
    modeSwitchBeep();

    if (togglePos > threshold1){ //linearly increase toggleMax to a new value if togglePos goes larger
      Serial.println("I: toggleMax"); 
      if (togglePos > toggleMax) {
        toggleMax = togglePos;
        intTransient = map(toggleMax, threshold1, rockerHigh, intHold2, intMin) ;
        intTransient = intTransient > intMax ? intMax: intTransient;
      }
      intHold1 = intTransient  > intMax ? intMax : intTransient;
      //2000/fHold1 (in millisecond) represents the duration that is 2 full cycles of vibration based on the input frequency
      vh.vibrate(fHold1, intHold1, 2000 / fHold1, dutyCycle, 0, 0); //vibrate using the recent fHold1 value
    }
    else if (togglePos < threshold2){//decrease toggleMin to a new value if togglePos goes smaller
      Serial.println("I: toggleMin"); 
      if (togglePos < toggleMin) {
        toggleMin = togglePos;
        intTransient = map(toggleMin, threshold2, rockerLow, intHold2, intMax);
        intTransient = intTransient < intMin ? intMin : intTransient;
      }
      intHold1 = intTransient > intMax ? intMax : intTransient;
      vh.vibrate(fHold1, intHold1, 2000 / fHold1, dutyCycle, 0, 0); //vibrate using the recent intHold1 value   
    }
    else if (togglePos > threshold2 && togglePos < threshold1) { //reset function: when toggle moves back within the thresholds, reset
      Serial.println("I: In Between Values"); 
      vh.vibrate(fHold1, intHold1, 2000 / fHold1, dutyCycle, 0, 0); // vibration output when toggle switch reset
      intHold2 = intHold1;
      thresholdAssign();
    }

    
}

void loop(){
  buttonState = digitalRead(buttonPin);
  
  if (buttonState == LOW && buttonPressed == false){//button pressed detected
    buttonMode();  
  }
  
  if (deviceOn == true){
    togglePos = myMag1.getMeasurementZ();// raw mag reading 
    //Serial.println(togglePos);
    if(freqSweepMode){
      //freqModeZeli();
      freqMode();
    }
    else{
      intensityMode();
    }
    storeRange();
  }
}


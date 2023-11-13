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



///////////////////////////////////////////
#include <Wire.h>

#include <SparkFun_MMC5983MA_Arduino_Library.h> //Click here to get the library: http://librarymanager/All#SparkFun_MMC5983MA

SFE_MMC5983MA myMag1;
int currentX1 = 0;
int currentY1 = 0;
int currentZ1 = 0;

int rockerVal; //the raw reading from mag
int rockerHigh; //the upper limit value that the rocker switch reads (neutral at 27224)
int rockerLow;//the lower limit value that the rocker switch reads
int rockerMid;
int togglePos = 0.0;
//the frequency value that the rocker toggles to;it's a averaged value based on rockerVal
//it should be between rockerHigh and rockerLow

int toggleMax;//the local max value to temporarily store max mag reading
int toggleMin;//the local min value to temporarily store max mag reading
float fMax = 300;
float fMin = 40;
float fHold1 = 0.5 * (fMax + fMin); //holds the local max/min frequency toggled by fTransient
float fHold2 = fHold1; //holds the previously reached min/max value as a starting point
float fTransient = fHold1; //the transient frequency value to temporarily set fHold1 value


int calibrationVal; //the raw mag reading everytime the motor turns on (while the rocker is at neutral)
const int buttonPin = 4;
bool buttonStat = false;
bool buttonStat2 = false;

float dutyCycle = 1;
float intensity = 1;

int threshold1;
int threshold2;//the threshold values that triggers the increase/decrease in fTransient

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
/////////////////////////////////////

void setup()
{
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

  vh.Init(&info, &devInfo);

  vh.setF0(50); // Sets the default resonant frequency of the haptic actuator.  Affects VH primitives. Default is 100Hz

  vh.setMinMax(0, 1);    // setMinMax(float min, float max) values between 0 and 1
  vh.adjustTiming(1.16); // multiplier to correct timing.  Values < 1 speed up timing, and > 1 slow it down.  Depending on your clock sources and MCU, you may need to adjust this parameter to correct for deviations.  NOTE: May not impact all frequencies equally!
  // to make sure the new OS is in effect turn on the builtin LED:
  vh.turnOnBuiltinLED(true);
  //vh.SetWaveResolution(40);
  vh.EnableDac(); // Enable the DAC
  vh.TurnOnDrv();
  vh.TurnOnPam(); // turn on the Pam amplifier
  vh.setWaveCalibFactor(0.02);
  // pinMode(PIN1, INPUT_PULLDOWN); // 32
  // pinMode(PIN2, INPUT_PULLDOWN); // 33
  // pinMode(MASTER_PIN, OUTPUT);
  //digitalWrite(MASTER_PIN, HIGH);
  //vh.configSerialBluetoothWifi(true, false, false);
  //  xTaskCreate(PinListenerFuncThread, "PinListenerFuncThread", 8000, NULL, 1, NULL);
  pinMode(buttonPin, INPUT_PULLUP);
  Serial.begin(115200);
  Wire.begin();

  myMag1.begin();

  if (myMag1.begin() == false)
  {
    Serial.println("MMC5983MA did not respond - check your wiring. Freezing.");
    while (true)
      ;
  }

  myMag1.softReset();
  vh.logAllMessages(false);

  preferences.begin("toggleRange", false);//clear memory at reset (for testing only) - this function will be removed for the final version
  preferences.clear();
  preferences.end();

}




// Function to calculate the average of a CircularBuffer
int calculateAverage(CircularBuffer<int, BUFFER_SIZE> &buffer) {
  int sum = 0;
  int count = buffer.size();

  // Iterate through the buffer and calculate the sum of elements
  for (int i = 0; i < count; i++) {
    sum += buffer[i];
  }

  // Calculate and return the average
  if (count > 0) {
    return sum / count;
  } else {
    return 0; // Return 0 if the buffer is empty
  }
}

void loop()
{
  //vh.MainLoop();
  //MOTOR TUNR-ON FUNCTION that is controlled by the push button; when button is pushed,
  //motor turns on and runs the main fucntion block
  if (buttonStat == false) {
    if (digitalRead(buttonPin) == LOW) {
      buttonStat = true;
      while (digitalRead(buttonPin) != HIGH) {
      }
      delay(50);

      preferences.begin("toggleRange", false);//begins a storage space in memory to permenantely store calibrated toggle range
      upperRange = preferences.getInt("upperRange", 5500);
      lowerRange = preferences.getInt("lowerRange", 2500);
      preferences.end();

      //CALIBRATION//
      rockerMid = myMag1.getMeasurementZ();//set middle rocker reading when system starts
      buffer3.push(rockerMid); //push rockerMid value into buffer3 to get averaged rockMid - rockerMidAvg
      rockerMid = calculateAverage(buffer3);
      rockerHigh = rockerMid + upperRange; //the starting upper limit value that will be updated through auto calibration (about 50% of the approximate real avlue)
      rockerLow = rockerMid - lowerRange; //the starting lower limit value that will be updated through auto calibration (about 50% of the approximate real avlue)
      buffer1.push(rockerLow);
      buffer2.push(rockerHigh);
      rockerHighAvg = rockerHigh;
      rockerLowAvg = rockerLow;
      threshold1 = rockerMid + thresholdPercentage * (rockerHigh - rockerMid); //the threshold values that triggers the increment in fTransient
      threshold2 = rockerMid - thresholdPercentage * (rockerMid - rockerLow); //the threshold values that triggers the decrement in fTransient
      toggleMax = threshold1; //set initial toggle max value at threshold1
      toggleMin = threshold2; //set initial toggle min value at threshold2
    }
  }


  /////////////////////MAIN FUNCTION BLOCK//////////////

  rockerVal = myMag1.getMeasurementZ();// raw mag reading
  // Serial.println(rockerVal);
  if (buttonStat == true) {     //when button is pushed, start pushing raw mag reading into the buffer

    togglePos = rockerVal;
    //now we have a buffered mag reading named as togglePos, which is the main input in the following code

    // some useful prints for troubleshooting:


    if (togglePos > threshold1)  { //linearly increase toggleMax to a new value if togglePos goes larger
      if (togglePos > toggleMax) {
        toggleMax = togglePos;
        fTransient = map(toggleMax, threshold1, rockerHigh, fHold2, fMin);
        if (fTransient < fMin) {
          fTransient = fMin;
        }
        //fHold1 = exp((fTransient + 176.2951) / 81.0085); //set new fHold1 position to the recent fTransient
        fHold1 = 0.00345 * sq(fTransient) + 9.66;//curve equation to replace the linear mapping
      }
      if (fHold1 < fMin) {
        fHold1 = fMin;
      }

      vh.vibrate(fHold1, intensity, 2000 / fHold1, dutyCycle, 0, 0); //vibrate using the recent fHold1 value
      //2000/fHold1 (in millisecond) represents the duration that is 2 full cycles of vibration based on the input frequency

    }


    if (togglePos < threshold2)  {//decrease toggleMin to a new value if togglePos goes smaller
      if (togglePos < toggleMin) {
        toggleMin = togglePos;
        fTransient = map(toggleMin, threshold2, rockerLow, fHold2, fMax);
        if (fTransient > fMax) {
          fTransient = fMax;
        }
        //fHold1 = exp((fTransient + 176.2951) / 81.0085); //set new fHold1 position to the recent fTransient
        fHold1 = 0.00345 * sq(fTransient) + 9.66;//curve equation to replace the linear mapping
      }
      if (fHold1 > fMax) {
        fHold1 = fMax;
      }


      vh.vibrate(fHold1, intensity, 2000 / fHold1, dutyCycle, 0, 0); //vibrate using the recent fHold1 value
      //2000/fHold1 (in millisecond) represents the duration that is 2 full cycles of vibration based on the input frequency
    }




    if (togglePos > threshold2 && togglePos < threshold1) { //reset function: when toggle moves back within the thresholds, reset
      vh.vibrate(fHold1, intensity, 2000 / fHold1, dutyCycle, 0, 0); // vibration output when toggle switch reset
      //2000/fHold1 (in millisecond) represents the duration that is 2 full cycles of vibration based on the input frequency
      //fHold2 = 81.0085 * log(fHold1) - 176.2951; //when togglePos returns, update fHold2 with the latest fHold1 for next the next mapping to be valid
      fHold2 = sqrt((fHold1 - 9.66) / 0.00345);//curve equation (reset) to replace the linear mapping
      if (toggleMax > rockerHighAvg && pushCounterHigh <= pushCounterLimit) { //push new max value to rockerHigh buffer if it is higher than average
        buffer2.push(toggleMax);
        rockerHighAvg = calculateAverage(buffer2);
        rockerHigh = rockerHighAvg;
        pushCounterHigh = pushCounterHigh + 1;
      }
      if (pushCounterHigh > pushCounterLimit && toggleMax > rockerMid + 0.95 * (rockerHigh - rockerMid)) { //after pushCounterLimit times, only push value within 95%
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
      if (pushCounterHigh > pushCounterLimit && toggleMin < rockerMid - 0.95 * (rockerMid - rockerLow)) {//after pushCounterLimit times, only push value within 95%
        buffer1.push(toggleMin);
        rockerLowAvg = calculateAverage(buffer1);
        rockerLow = rockerLowAvg;
      }

      threshold1 = rockerMid + thresholdPercentage * (rockerHigh - rockerMid); //the threshold values that triggers the increment in fTransient
      threshold2 = rockerMid - thresholdPercentage * (rockerMid - rockerLow); //the threshold values that triggers the decrement in fTransient

      toggleMax = threshold1;//reset toggleMax and toggleMin back to thresholds
      toggleMin = threshold2;
      // Serial.println(fTransient);
    }

//USEFUL PRINTS FOR DEBUGGING/////////////
    // Serial.println(fHold1);
    //        Serial.print("fholdMap: ");
    //        Serial.println(map(fHold1, fMin, fMax, rockerLow, rockerHigh));
    //


        Serial.print("rocker Mid: ");
        Serial.println(rockerMid);
        Serial.print("rockerHigh: ");
        Serial.println(rockerHigh);
        Serial.print("rockerLow: ");
        Serial.println(rockerLow);
    
        Serial.print("toggle pos: ");
        Serial.println(togglePos);
//////////////////////////////////////////

    upperRange = rockerHigh - rockerMid;//stores updated upper and lower range into the memory so next time the latest value is being read
    lowerRange = rockerMid - rockerLow;
    preferences.begin("toggleRange", false);
    preferences.putInt("upperRange", upperRange);
    preferences.putInt("lowerRange", lowerRange);
    preferences.end();
    //^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^MAIN FUNCTION BLOCK^^^^^^^^^^^^^^^^^^^^^^^^^^

    //MOTOR TURN OFF FUNCTION: when button pushed during main function running, the motor turns off
    if (digitalRead(buttonPin) == LOW) {
      buttonStat = false;
      fHold1 = 0.5 * (fMax + fMin);//when device is turned off using the button, the frequency returns back to mid point the next time it turns on
      while (digitalRead(buttonPin) != HIGH) {
      }
      delay(50);
    }

  }
}

float map(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

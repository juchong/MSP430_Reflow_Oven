/***************************************************************************************
*  Reflow Oven Controller
*  Version: 1.0
*  Date: 8-8-2013
*  Code Author: Kristen Villemez
*  Hardware Design: Juan Chong
*  Website: www.juanjchong.com
*
*  Overview
*  =========
*  This code is used to control a reflow oven. It allows for the option to change
*  solder types (lead vs. lead-free) before each time the reflow process is started.
*  Definitions for both profiles are included in this code. This code is designed to
*  work with the Reflow Oven Controller BoosterPack designed by Juan Chong. It uses
*  the MAX31855 thermocouple chip and library and an on-board LCD screen for status
*  display on the fly. A PID controller library is used to control the system.
*
*  Basic Usage
*  ============
*  When the unit is first started, it will display the splash screen and prompt the
*  user to select a solder type - lead or lead-free. The default is lead solder.
*  If the user wishes to run the lead-free profile, press and hold the Solder Select
*  button on the BoosterPack or Shield (depending on if you have a Launchpad or
*  Arduino.
*
*  Attributions
*  =============
* + Lim Phang Moh - http://www.rocketscream.com
*  Author of MAX31855 library and original code off which this program is based
* + Brett Beauregard - http://www.brettbeauregard.com
*  Author of Arduino PID Library used in this program
* + Bill Earl - http://www.adafruit.com
*  Author of Sous Vide, an excellent self-tuning PID controller example
*
*  DISCLAIMERS!!!
*  ===============
*  HIGH VOLTAGES ARE DANGEROUS! PLEASE UNDERSTAND THE RISKS YOU ARE DEALING WITH
*  WHEN DEALING WITH THIS HARDWARE BEFORE YOU BEGIN! USE COMMON SENSE WHEN WORKING
*  WITH THIS PROJECT. USE OF THIS HARDWARE AND SOFTWARE IS AT YOUR OWN RISK, AND
*  WE ARE NOT RESPONSIBLE OR LIABLE FOR ANY DAMAGE TO YOU OR YOUR SURROUNDINGS THAT
*  MAY OCCUR OUT OF USE OF THIS AND RELATED MATERIALS!
*
*  THIS SOFTWARE IS MADE AVAILABLE "AS IS" WITHOUT WARRANTY OF ANY KIND, EITHER
*  EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION ANY IMPLIED WARRANTIES OF
*  CONDITION, UNINTERRUPTED USE, MERCHANTABILITY, FITNESS FOR A PARTICULAR USE, OR
*  NON-INFRINGEMENT.
*
*  Licenses
*  =========
*  This hardware and software is released under...
*  Creative Commons Share Alike v3.0 License
*  http://creativecommons.org/licenses/by-sa/3.0/
*  You are free to use this code and/or modify it. All we ask is an attribution,
*  including supporting libraries and their respective authors used in this
*  software.
*
*  Required Libraries
*  ===================
* - Arduino PID Library: (cross-platform compatible)
*    >> https://github.com/b3rttb/Arduino-PID-Library
* - MAX31855 Library (cross-platform compatible)
*    >> https://github.com/rocketscream/MAX31855
* - TwoMsTimer Library (use this if you are using the MSP430 Launchpad and Boosterpack)
*    >> https://github.com/freemansoft/build-monitor-devices/tree/master/ti_launchpad_rgb
* - Timer1 Library (use this if you are using the Arduino and Shield)
*    >> http://playground.arduino.cc/code/timer1
* - LiquidCrystal Library (included in both development environments)
*
*  REVISION HISTORY
*  =================
*  1.0 - Initial release
*  1.1 - Fix bug where oven does not go into ERROR state when communication is lost
*
***************************************************************************************/

//////////////////////////////////////////////
//   BEGIN INSTANTIATIONS AND DEFINITIONS   //
//////////////////////////////////////////////

/* LIBRARY INCLUSIONS */
#include <LiquidCrystal.h>
#include <MAX31855.h>
#include <PID_v1.h>
#include <TwoMsTimer.h>

/* PID CONTROLLER PARAMETERS - CONSTANTS */
#define SAMPLE_TIME 1000

#define KP_PREHEAT 100
#define KI_PREHEAT 0.025
#define KD_PREHEAT 20

#define KP_SOAK 300
#define KI_SOAK 0.05
#define KD_SOAK 250

#define KP_REFLOW 350 // changed from 300
#define KI_REFLOW 0.13 // changed from 0.05
#define KD_REFLOW 400 // changed from 350

/* REFLOW STAGE DEFINITIONS */
typedef enum REFLOW_STAGE
{
  IDLE_STAGE,
  PREHEAT_STAGE,
  SOAK_STAGE,
  REFLOW_STAGE,
  COOL_STAGE,
  COMPLETE_STAGE,
  ERROR_PRESENT
};

/* PID CONTROLLER PARAMETERS - VARIABLES */
//  If your oven happens to be overshooting,
//  increase the windowSize variable below.
//  If it's undershooting, decrease it.
double setpoint;
double input;
double output;
int windowSize = 1500;
unsigned long windowStartTime;
double kp;
double ki;
double kd;
unsigned long timerSoak;

/* LCD MESSAGES */
const char* lcdStageMessages[] = {
  "Select Solder",
  "PRE-HEAT",
  "SOAK",
  "REFLOW",
  "COOLING",
  "COMPLETE",
  "SENSOR ERROR"
};

/* DEGREE SYMBOL DEF */
byte degree[8] = {
  B001100,
  B010010,
  B010010,
  B001100,
  B000000,
  B000000,
  B000000,
  B000000
};

/* CONSTANT INSTANTIATIONS */
int ROOM;
int SOAK_MIN;
int SOAK_MAX;
int SOAK_STEP;
int SOAK_MICRO_PERIOD;
int REFLOW_MAX;
int COOL_MIN;
int SAMPLING_TIME;

/* PIN ASSIGNMENT */
int relayPin = 2;
int ledPin = 13;
int thermoSO = 15;
int thermoCS = 8;
int thermoCLK = 7;
int lcdRs = 18;
int lcdE = 19;
int lcdD4 = 9;
int lcdD5 = 10;
int lcdD6 = 11;
int lcdD7 = 12;
int typeBttn = 5;
int startstopBttn = 6;

/* STATE VARIABLE INSTANTIATIONS */
enum REFLOW_STAGE reflowStage = IDLE_STAGE;
boolean ovenState = false;

/* INSTANTIATE PID CONTROLLER */
PID ovenPID(&input, &output, &setpoint, kp, ki, kd, DIRECT);
/* INSTANTIATE LCD INTERFACE */
LiquidCrystal lcd(lcdRs, lcdE, lcdD4, lcdD5, lcdD6, lcdD7);
/* INSTANTIATE THERMOCOUPLE INTERFACE */
MAX31855 thermo(thermoSO, thermoCS, thermoCLK);

/* DEFINING SOLDER TYPE VARIABLE*/
/* true means lead, false means lead-free */
boolean solderType = true;

//////////////////////////////////////////////
//         BEGIN CODE METHOD BLOCKS         //
//////////////////////////////////////////////

//////////////////////////////////////////////
// setup Function:
//    Sets pin modes and plays splash screen
void setup()
{
  // Ensure oven relay is off
  digitalWrite(relayPin, LOW);
  pinMode(relayPin, OUTPUT);
  
  // Start Serial monitor, for debugging
  Serial.begin(9600);
  
  // Setting control button modes
  pinMode(typeBttn, INPUT_PULLUP);
  pinMode(startstopBttn, INPUT_PULLUP);
  
  // Setting LED Pin mode
  pinMode(ledPin, OUTPUT);
  // pin is active LOW
  digitalWrite(ledPin, HIGH);
  
  // Start-up Splash Screen
  lcd.begin(8, 2);
  lcd.createChar(1, degree);
  lcd.clear();
  lcd.print("Reflow Oven 1.0");
  lcd.setCursor(0,1);
  lcd.print("MSP430-TI In It");
  delay(2500);
  lcd.clear();
  
  // Begin Time Keeping
  //   Replace these with the relevant commands from Timer1 for Arduino
  //   to interrupt every 500 ms
  TwoMsTimer::set(500, InterruptHandler);
  TwoMsTimer::start();
  
  // Attach START/STOP interrupt to the button
  attachInterrupt(startstopBttn, StartStop, FALLING);
}

///////////////////////////////////////////////////
// loop Function:
//    Basic control of moving from stage to stage
//    of the reflow process. Sets up PID control
//    variables based on which stage is currently
//    active.
void loop()
{
  // The following if statement code ensures that the
  //  oven is in the IDLE_STAGE if set to off. It will
  //  also shut the oven off if it reaches 265C in error
  //  to prevent internal damage.
  if (!ovenState || input >= 265)
  {
    ovenState = false;
    reflowStage = IDLE_STAGE;
  }
  DoControl();
  switch (reflowStage)
  {
    case IDLE_STAGE:
      Idle();
      break;
    case PREHEAT_STAGE:
      Preheat();
      break;
    case SOAK_STAGE:
      Soak();
      break;
    case REFLOW_STAGE:
      Reflow();
      break;
    case COOL_STAGE:
      Cool();
      break;
    case COMPLETE_STAGE:
      Complete();
      break;
    case ERROR_PRESENT:
      Error();
      break;
  }
  
}

//////////////////////////////////////////////
// Interrupt Handler
//    This simple function decides whether to
//    make the next computation (if oven state
//    is true) or to ensure oven is switched
//    off. It is called every 15ms as called
//    by the watchdog timer in TwoMsTimer.
void InterruptHandler()
{
  lcd.clear();
  lcd.print(lcdStageMessages[reflowStage]);
  lcd.setCursor(0,1);
  lcd.print(input);
  lcd.write(1);
  lcd.print("C ");
  if(solderType)
  {
    lcd.print("Lead");
  }
  else
  {
    lcd.print("No Lead");
  }
  if (ovenState)
  {
    DriveOutput();
  }
  else
  {
    digitalWrite(relayPin, LOW);
  }
}

//////////////////////////////////////////////
// Drive Output
//    This function simply checks the output
//    value of the PID controller to see if
//    relay should be turned on. If it is,
//    this turns the relay on/off
//    appropriately. It also prints out the
//    current temperature and reflow stage.
void DriveOutput()
{
  Serial.println(output);
  long now = millis();
  if(now - windowStartTime > windowSize)
  { //time to shift the Relay Window
    windowStartTime += windowSize;
  }
  if(output > (now - windowStartTime))
  {
    digitalWrite(relayPin,HIGH);
  }
  else
  {
    digitalWrite(relayPin,LOW);
  }
}

//////////////////////////////////////////////
// Do Control
//    This function performs all the basic PID
//    function calls needed to control the
//    oven properly.
void DoControl()
{
  input = thermo.readThermocouple(CELSIUS);
  ovenPID.Compute();
}

//////////////////////////////////////////////
// Idle
//    This function is the default reflow
//    stage. It is where the temperature
//    profile is chosen.
void Idle()
{  
  DoControl();
  if((input == FAULT_OPEN) || (input == FAULT_SHORT_GND) || (input == FAULT_SHORT_VCC) || (input < 5))
  {
    reflowStage = ERROR_PRESENT;
    return;
  }
  while (!ovenState)
  {
    // Setting up temporary variable for debouncing
    int pressConfLvl = 0;

    DoControl();
    while (!digitalRead(typeBttn))
    {
      // Increment button confidence counter
      pressConfLvl++;
      //  If by now the counter has reached 32000, button
      //  should be pressed and not just bouncing
      if (pressConfLvl > 32000)
      {
        lcd.clear();
        solderType = !solderType;
        // Reset the counter for the next button press
        pressConfLvl = 0;
      }
    }
  }
  if(ovenState)
  {
    if (solderType)
    {
      ROOM = 50;
      SOAK_MIN = 135;
      SOAK_MAX = 155;
      REFLOW_MAX = 225;
      COOL_MIN = 50;
      SAMPLING_TIME = 1000;
      SOAK_STEP = 6;
      SOAK_MICRO_PERIOD = 9000;
    }
    else
    {
      ROOM = 50;
      SOAK_MIN = 150;
      SOAK_MAX = 200;
      REFLOW_MAX = 250;
      COOL_MIN = 50;
      SAMPLING_TIME = 1000;
      SOAK_STEP = 5;
      SOAK_MICRO_PERIOD = 9000;
    }
    windowStartTime = millis();
    setpoint = SOAK_MIN;
    ovenPID.SetOutputLimits(0, windowSize);
    ovenPID.SetSampleTime(SAMPLING_TIME);
    ovenPID.SetTunings(KP_PREHEAT, KI_PREHEAT, KD_PREHEAT);
    ovenPID.SetMode(AUTOMATIC);
    DoControl();
    digitalWrite(ledPin,LOW);
    reflowStage = PREHEAT_STAGE;
  }
}

//////////////////////////////////////////////
// Preheat
//    This function handles the setting of the
//    new setpoint once the temperature has
//    reached SOAK_MIN and the oven is ready
//    to move into the soaking stage.
void Preheat()
{
  DoControl();
  if((input == FAULT_OPEN) || (input == FAULT_SHORT_GND) || (input == FAULT_SHORT_VCC) || (input < 5))
  {
    reflowStage = ERROR_PRESENT;
    ovenState = false;
    return;
  }
  if (((SOAK_MIN + 5) > input) && (input >= SOAK_MIN))
  {
    timerSoak = millis() + SOAK_MICRO_PERIOD;
    setpoint = SOAK_MIN + SOAK_STEP;
    ovenPID.SetTunings(KP_SOAK, KI_SOAK, KD_SOAK);
    reflowStage = SOAK_STAGE;
  }
}

//////////////////////////////////////////////
// Soak
//    The SOAK stage is broken up into mini
//    time frames to better control the
//    temperature increase. All infrastructure
//    for this frame break-up was set up at
//    the end of the PREHEAT stage. Once
//    SOAK_MAX has been reached, tunings and
//    setpoints are updated for REFLOW stage.
void Soak()
{
  DoControl();
  if((input == FAULT_OPEN) || (input == FAULT_SHORT_GND) || (input == FAULT_SHORT_VCC) || (input < 5))
  {
    reflowStage = ERROR_PRESENT;
    ovenState = false;
    return;
  }

  if (millis() > timerSoak)
  {
    timerSoak = millis() + SOAK_MICRO_PERIOD;
    setpoint += SOAK_STEP;
    if (((SOAK_MAX + 5) > input) && (input > SOAK_MAX))
    {
      ovenPID.SetTunings(KP_REFLOW, KI_REFLOW, KD_REFLOW);
      setpoint = REFLOW_MAX;
      reflowStage = REFLOW_STAGE;
    }
  }
}

//////////////////////////////////////////////
// Reflow
//    This is where the magic happens. (In
//    the oven silly!) Tunings have been set
//    very aggressive and paste melts here
//    before cooling back down in the next
//    stage.
void Reflow()
{
  DoControl();
  if((input == FAULT_OPEN) || (input == FAULT_SHORT_GND) || (input == FAULT_SHORT_VCC) || (input < 5))
  {
    reflowStage = ERROR_PRESENT;
    ovenState = false;
    return;
  }
  if (((REFLOW_MAX + 5) > input) && (input >= (REFLOW_MAX)))
  {
    setpoint = COOL_MIN;
    reflowStage = COOL_STAGE;
  }
}

//////////////////////////////////////////////
// Cool
//    This essentially just waits for the
//    oven temperature to reach a level safe
//    enough to open the door.
void Cool()
{
  DoControl();
  digitalWrite(ledPin, HIGH);
  if ((input <= 60) && (input > 50))
  {
    reflowStage = COMPLETE_STAGE;
  }
}

//////////////////////////////////////////////
// Complete
//    This is just a place-holder forced wait
//    time to display the fact that the
//    process is complete so the user knows
//    they can take the soldered boards out
//    of the oven.
void Complete()
{
  delay(5000);
  reflowStage = IDLE_STAGE;
  ovenState = false;
}

//////////////////////////////////////////////
// Error
//    This function alerts the user that there
//    was an error in reading the thermocouple
//    temperature. Either the thermocouple is
//    disconnected or something else is wrong.
void Error()
{
  detachInterrupt(startstopBttn);
  DoControl();
  lcd.clear();
  lcd.println("Sensor Error!!!");
  lcd.println("Reset Launchpad");
  while(1);
}

//////////////////////////////////////////////
// StartStop
//    This function handles the software
//    debouncing of the Start/Stop button, as
//    well as handling the setting of the
//    ovenState variable. If toggled off during
//    the reflow process, it will also set the
//    stage to IDLE_STAGE.
void StartStop()
{
  delayMicroseconds(40000);
  if(!digitalRead(startstopBttn))
  {  
    ovenState = !ovenState;
    if(!ovenState)
    {
      reflowStage = IDLE_STAGE;
      digitalWrite(ledPin, HIGH);
    }
  }
}

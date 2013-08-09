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
*  Attributions
*  =============
* + Lim Phang Moh - http://www.rocketscream.com
*  Original code off which this program is based
* + Brett Beauregard - http://www.brettbeauregard.com
*  Author of Arduino PID Library used in this program
* + Limor Fried - http://www.adafruit.com
*  Author of MAX31855 Library
*
*  DISCLAIMER!!!
*  ==============
*  HIGH VOLTAGES ARE DANGEROUS! PLEASE UNDERSTAND THE RISKS YOU ARE DEALING WITH
*  WHEN DEALING WITH THIS HARDWARE BEFORE YOU BEGIN! USE COMMON SENSE WHEN WORKING
*  WITH THIS PROJECT. USE OF THIS HARDWARE AND SOFTWARE IS AT YOUR OWN RISK, AND
*  WE ARE NOT RESPONSIBLE OR LIABLE FOR ANY DAMAGE TO YOU OR YOUR SURROUNDINGS THAT
*  MAY OCCUR OUT OF USE OF THIS AND RELATED MATERIALS!
*  THERE IS NO GUARANTEE OF FUNCTION OF THIS CODE IN YOUR PARTICULAR APPLICATION.
*  NO WARRANTY ON THIS SOFTWARE IS GUARANTEED NOR IMPLIED.
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
* - Arduino PID Library: (works as is in Energia)
*    >> https://github.com/b3rttb/Arduino-PID-Library
* - MAX31855 Library (works as is in Energia)
*    >> https://github.com/rocketscream/MAX31855
* - FlexiTimer2 Library (works as is in Energia)
*    >> http://www.pjrc.com/teensy/td_libs_MsTimer2.html
* - LiquidCrystal Library (included in Energia)
*
*    REVISION HISTORY
*  Revision  Description
*  ========  ===========
*  1.0       Initial release
*
***************************************************************************************/



/* LIBRARY INCLUSIONS */
#include <LiquidCrystal.h>
#include <MAX31855.h>
#include <PID_v1.h>
#include <FlexiTimer2.h>

/* OVEN STATE DEFINITIONS */
enum REFLOW_STAGE
{
  IDLE_STAGE,
  PREHEAT_STAGE,
  SOAK_STAGE,
  REFLOW_STAGE,
  COOL_STAGE,
  COMPLETE_STAGE,
  ERROR_PRESENT,
};

/* PID CONTROLLER PARAMETERS - CONSTANTS */
#define SAMPLE_TIME 1000

#define KP_PREHEAT 100
#define KI_PREHEAT 0.025
#define KD_PREHEAT 20

#define KP_SOAK 300
#define KI_SOAK 0.05
#define KD_SOAK 250

#define KP_REFLOW 300
#define KI_REFLOW 0.05
#define KD_REFLOW 350

/* PID CONTROLLER PARAMETERS - VARIABLES */
double setpoint;
double input;
double output;
int windowSize;
unsigned long windowStartTime;
volatile long onTime = 0;
double kp;
double ki;
double kd;
unsigned long timerSoak;

/* LCD MESSAGES */
const char* lcdStageMessages[] = {
  "READY",
  "PRE-HEAT",
  "SOAK",
  "REFLOW",
  "COOL",
  "COMPLETE",
  "ERROR"
};

/* DEGREE SYMBOL DEF */
unsigned byte degree[8] = {
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
int relayPin = 19;
int thermoSO = 15;
int thermoCS = 8;
int thermoCLK = 7;
int lcdRs = 18;
int lcdE = 4;
int lcdD4 = 9;
int lcdD5 = 10;
int lcdD6 = 11;
int lcdD7 = 12;
int typeBttn = 5;
int startstopBttn = 6;

/* STATE VARIABLE INSTANTIATIONS */
REFLOW_STAGE reflowStage = IDLE_STAGE;
boolean ovenState = false;

/* INSTANTIATE PID CONTROLLER */
PID ovenPID(&input, &output, &setpoint, kp, ki, kd, DIRECT);
/* INSTANTIATE LCD INTERFACE */
LiquidCrystal lcd(lcdRs, lcdE, lcdD4, lcdD5, lcdD6, lcdD7);
/* INSTANTIATE THERMOCOUPLE INTERFACE */
MAX31855 thermo(thermoSO, thermoCS, thermoCLK);

/* DEFINING BUTTON INTERRUPT VARIABLES */
boolean solderType = true;
long debouncingTime = 30; // Software debouncing time in ms
volatile unsigned long lastMicros;

//////////////////////////////////////////////
//        BEGIN CODE METHOD BLOCKS          //
//////////////////////////////////////////////

//////////////////////////////////////////////
// setup Function:
//    Sets pin modes and plays splash screen
void setup()
{
  // Ensure oven relay is off
  digitalWrite(relayPin, LOW);
  pinMode(relayPin, OUTPUT);
  
  // Setting control button modes
  pinMode(typeBttn, INPUT);
  pinMode(startstopBttn, INPUT);
  
  // Start-up Splash Screen
  lcd.begin(8, 2);
  lcd.createChar(0, degree);
  lcd.clear();
  lcd.print("Reflow Oven 1.0");
  lcd.setCursor(0,1);
  lcd.print("MSP430-TI In it");
  delay(2500);
  lcd.clear();
  
  // Begin Time Keeping
  FlexiTimer2::set(500, InterruptHandler);
  FlexiTimer2::start();
}

///////////////////////////////////////////////////
// loop Function:
//    Basic control of moving from stage to stage
//    of the reflow process. Sets up PID control
//    variables based on which stage is currently
//    active.
void loop()
{
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
//    by the watchdog timer in FlexiTimer2
void InterruptHandler()
{
  if (!ovenState)
  {
    digitalWrite(relayPin, LOW);
  }
  else
  {
    DriveOutput();
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
  long now = millis();
  // Set the output
  // "on time" is proportional to the PID output
  if(now - windowStartTime>WindowSize)
  { //time to shift the Relay Window
    windowStartTime += WindowSize;
  }
  if((onTime > 100) && (onTime > (now - windowStartTime)))
  {
    digitalWrite(RelayPin,HIGH);
  }
  else
  {
    digitalWrite(RelayPin,LOW);
  }
  if (reflowStage != IDLE_STAGE)
  {
    lcd.clear();
    lcd.print(lcdStageMessages[reflowStage];
    lcd.setCursor(0,1);
    lcd.print(input);
    lcd.print(0, BYTE);
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
  onTime = output;
}

//////////////////////////////////////////////
// Idle
//    This function is the default reflow
//    stage. It is where the temperature
//    profile is chosen.
void Idle()
{
  lcd.clear();
  lcd.print("Select Solder");
  attachInterrupt(typeBttn, SolderSelect, FALLING);
  attachInterrupt(startstopBttn, StartStop, FALLING);
  while(!ovenState)
  {
    if((input == FAULT_OPEN) || (input == FAULT_SHORT_GND) || (input == FAULT_SHORT_VCC))
      reflowStage = ERROR_PRESENT;
  }
  if (solderType)
  {
    lcd.clear();
    lcd.print("Lead Set");
    ROOM = 50;
    SOAK_MIN = 135;
    SOAK_MAX = 155;
    REFLOW_MAX = 225;
    COOL_MIN = 100;
    SAMPLING_TIME = 1000;
    SOAK_STEP = 6;
    SOAK_MICRO_PERIOD = 9000;
  }
  else
  {
    lcd.clear();
    lcd.print("Lead-Free Set");
    ROOM = 50;
    SOAK_MIN = 150;
    SOAK_MAX = 200;
    REFLOW_MAX = 250;
    COOL_MIN = 100;
    SAMPLING_TIME = 1000;
    SOAK_STEP = 5;
    SOAK_MICRO_PERIOD = 9000;
  }
  delay(3000);
  windowStartTime = millis();
  setpoint = SOAK_MIN;
  ovenPID.SetOutputLimits(0, windowSize);
  ovenPID.SetSampleTime(SAMPLING_TIME);
  ovenPID.SetTunings(KP_PREHEAT, KI_PREHEAT, KD_PREHEAT);
  ovenPID.SetMode(AUTOMATIC);
  ovenState = true;
  reflowStage = PREHEAT_STAGE;
}

//////////////////////////////////////////////
// Preheat
//    This function handles the setting of the
//    new setpoint once the temperature has
//    reached SOAK_MIN and the oven is ready
//    to move into the soaking stage.
void Preheat()
{
  if (input >= SOAK_MIN)
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
  if (millis() > timerSoak)
  {
    timerSoak += SOAK_MICRO_PERIOD;
    setpoint += SOAK_STEP;
    if (setpoint > SOAK_MAX)
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
  // Need to avoid hovering at the peak temp for too long...
  // Crude method but works and is safe for components
  if (input >= (REFLOW_MAX - 5)
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
  if (input <= COOL_MIN)
  {
    ovenState = false;
    reflowStage = COMPLETE_STAGE;
  }
}

//////////////////////////////////////////////
// Complete
//    This is really just a place-holder
//    forced wait time to display the fact
//    that the process is complete so the user
//    knows they can take the soldered boards
//    out of the oven.
void Complete()
{
  delay(10000);
  reflowStage = IDLE_STAGE;
}

//////////////////////////////////////////////
// Error
//    This function alerts the user that there
//    was an error in reading the thermocouple
//    temperature. Either the thermocouple is
//    disconnected or something else is wrong.
void Error()
{
  if((input == FAULT_OPEN) || (input == FAULT_SHORT_GND) || (input == FAULT_SHORT_VCC))
    reflowStage = ERROR_PRESENT;
  else
    reflowStage = IDLE_STAGE;
}


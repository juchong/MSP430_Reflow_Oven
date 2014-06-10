/***************************************************************************************
*  Reflow Oven Controller for the TI MSP430 Launchpad
*  Built for the TI MSP430G2553 Microcontroller
*  Software Version: 1.4
*  Date: 06-09-2014
*  Code Author: Kristen Villemez, Juan Chong
*  Hardware Design: Juan Chong
*  Website: http://www.juanjchong.com/
*  Last Updated By: Juan Chong
*
*  Overview
*  =========
*  This code is used to control a reflow oven. It allows for the option to change
*  solder types (lead vs. lead-free) before the reflow process is started.
*  Definitions for both profiles are included in this code. This code is designed to
*  work with the Reflow Oven Controller BoosterPack designed by Juan Chong. It uses
*  the MAX31855 thermocouple amplifier breakout board and library, and an on-board LCD 
*  screen for status display. A PID controller library is used to control a solid state
*  relay which drives the heater elements in a toaster oven.
*
*  DISCLAIMERS!!!
*  ===============
*  HIGH VOLTAGES AND CURRENTS ARE DANGEROUS! PLEASE UNDERSTAND THE RISKS OF WORKING 
*  WITH DANGEROUS HARDWARE BEFORE YOU BEGIN! USE COMMON SENSE WHEN WORKING
*  WITH THIS PROJECT. USE OF THIS HARDWARE AND SOFTWARE IS AT YOUR OWN RISK, AND
*  WE ARE NOT RESPONSIBLE OR LIABLE FOR ANY DAMAGE TO YOU OR YOUR SURROUNDINGS THAT
*  MAY OCCUR OUT OF USE OF THIS AND RELATED MATERIALS!
*
*  THIS SOFTWARE IS MADE AVAILABLE "AS IS" WITHOUT WARRANTY OF ANY KIND, EITHER
*  EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION ANY IMPLIED WARRANTIES OF
*  CONDITION, UNINTERRUPTED USE, MERCHANTABILITY, FITNESS FOR A PARTICULAR USE, OR
*  NON-INFRINGEMENT.
*
*  Basic Use
*  ============
*  When the controller is first started, it will display the splash screen and prompt the
*  user to select a solder type - lead (Pb) or lead-free (NoPb). The default is lead solder.
*  If the user wishes to run the lead-free profile, press the Solder Select
*  button on the BoosterPack/Shield. Once you are ready to start the reflow process, press the
*  Start Reflow button and wait for the system to run through the reflow profile.
*
*  Required Libraries
*  ===================
* - Arduino PID Library: (Energia or Arduino Compatible)
*    >> http://playground.arduino.cc/Code/PIDLibrary
* - MAX31855 Library (Energia or Arduino Compatible)
*    >> https://github.com/rocketscream/MAX31855
* - TwoMsTimer Library (Energia Compatible - Only Use If Programming An MSP430)
*    >> https://github.com/freemansoft/build-monitor-devices/tree/master/ti_launchpad_rgb
* - Timer1 Library (Arduino Compatible - Only Use If Programming An Arduino)
*    >> http://playground.arduino.cc/code/timer1
* - LiquidCrystal Library (Included By Default In Both Development Environments)
*
*  Attributions
*  =============
* + Lim Phang Moh - http://www.rocketscream.com/
*  Author of MAX31855 library and original Arduino code from which this program is based 
* + Brett Beauregard - http://www.brettbeauregard.com/
*  Author of Arduino PID Library used in this program
* + Bill Earl - http://www.adafruit.com/
*  Author of Sous Vide, an excellent self-tuning PID controller example
*
*  Licenses
*  =========
*  This hardware and software is released under:
*  Creative Commons Share Alike v3.0 License
*  http://creativecommons.org/licenses/by-sa/3.0/
*  You are free to use this code and/or modify it. All we ask is an attribution,
*  including supporting libraries and their respective authors used in this
*  software. If you would like to use this software and hardware for commercial
*  purposes, please contact the author using the website listed above.
*
*  REVISION HISTORY
*  =================
*  1.0 - Initial release
*  1.1 - Fix bug where oven does not go into ERROR state when thermocouple communication is lost.
*  1.2 - Optimized ISR tasks and LCD updating to avoid timing and memory issues.
*  1.3 - Added a step to ask the user whether the thermocouple has been placed on the PCB.
*  1.4 - Added a countdown to start reflow, fixed a bug where the oven keeps wanting to 
          approach 50C after reflow, adjusted PID values, added asthetics to LCD, improved 
          push button response, and cleaned up comments.
***************************************************************************************

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

#define KP_REFLOW 140 
#define KI_REFLOW 0.025 
#define KD_REFLOW 100 

/* REFLOW STAGE DEFINITIONS */
typedef enum REFLOW_STAGE
{
  IDLE_STAGE,
  PROBE_CHECK,
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

/* LCD MESSAGES CORRESPONDING TO STAGE DEFINITIONS ABOVE*/
const char* lcdStageMessages[] = {
  "Solder Type:",
  "Probe On PCB?   ",
  "PRE-HEATING",
  "SOAKING",
  "REFLOWING",
  "COOLING",
  "COMPLETE",
  "SENSOR ERROR!"
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
/* Do not change if using the Launchpad BoosterPack!! */
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
enum REFLOW_STAGE stoppedStage = IDLE_STAGE;
int countdown = 0;
int dot = 0;
boolean killReflow = false;
boolean ovenState = false;
boolean doUpdate = false;
boolean probeState = false;
boolean probeOnBoard = false;
boolean continueWithError = false;
boolean continueState = false;
boolean asked = false;

/* INSTANTIATE PID CONTROLLER */
PID ovenPID(&input, &output, &setpoint, kp, ki, kd, DIRECT);
/* INSTANTIATE LCD INTERFACE */
LiquidCrystal lcd(lcdRs, lcdE, lcdD4, lcdD5, lcdD6, lcdD7);
/* INSTANTIATE THERMOCOUPLE INTERFACE */
MAX31855 thermo(thermoSO, thermoCS, thermoCLK);

/* DEFINING SOLDER TYPE VARIABLE*/
/* True = Lead, False = Lead-Free */
boolean solderType = true;

//////////////////////////////////////////////
// Sets pin modes and plays splash screen
void setup()
{
  // Ensure oven relay is off when starting the controller
  digitalWrite(relayPin, LOW);
  pinMode(relayPin, OUTPUT);
  
  // Start Serial monitor, for debugging
  //Serial.begin(9600);
  
  // Setting control button modes
  pinMode(typeBttn, INPUT_PULLUP);
  pinMode(startstopBttn, INPUT_PULLUP);
  
  // Setting LED Pin mode. Pin is active-high.
  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, HIGH);
  
  // Start-up Splash Screen
  lcd.begin(8, 2);
  lcd.createChar(1, degree);
  lcd.clear();
  lcd.print("Reflow Oven 1.4");
  lcd.setCursor(0,1);
  lcd.print("juanjchong.com");
  delay(2500);
  lcd.clear();
  
  // Begin Time Keeping
  //  Replace these with the relevant commands from Timer1 
  //  for arduino to interrupt every 200 ms.
  TwoMsTimer::set(200, InterruptHandler);
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
  // Main switch statement to handle mode selection.  
  switch (reflowStage)
  {
    case IDLE_STAGE:
      Idle();
      break;
    case PROBE_CHECK:
      Probe();
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
  // Call the global update if needed.
  if (doUpdate)
    Update();
}

// This function will clear the LCD.
void CleanLCD()
{
  lcd.clear();
}

// This function will print reflow stages, temperatures, and reflow modes
// periodically when the timer interrupt is triggered. This function does not 
// clear the LCD, but instead overwrites data present on the display previously.
void UpdateLCD()
{   
  lcd.setCursor(0,0);
  lcd.print(lcdStageMessages[reflowStage]);
  if (reflowStage == IDLE_STAGE)
  {
    if(solderType)
    {
      lcd.print("Pb  ");
    }
    else
    {
      lcd.print("NoPb");
    }
  }
//////////////////////////////////////
//  Set up LCD messages for the Probe Check
//  mode.
  if (reflowStage == PROBE_CHECK)
  {
    if (probeOnBoard)
    {
      lcd.setCursor(0,1);
      lcd.print("Yes?");
    }
    else
    {
      lcd.setCursor(0,1);
      lcd.print("No? ");
    }
    if (countdown == 3)
    {
      lcd.setCursor(0,0);
      lcd.print("Starting in 3...");
      lcd.setCursor(0,1);
      lcd.print("                ");
    }
    if (countdown == 2)
    {
      lcd.setCursor(0,0);
      lcd.print("Starting in 2...");
      lcd.setCursor(0,1);
      lcd.print("                ");
    }
    if (countdown == 1)
    {
      lcd.setCursor(0,0);
      lcd.print("Starting in 1...");
      lcd.setCursor(0,1);
      lcd.print("                ");
    }
  }
  if (reflowStage == ERROR_PRESENT)
  {
    lcd.setCursor(0,1);
    lcd.write("Continue? ");
    if (continueWithError)
      lcd.write("Yes");
    else
      lcd.write("No");
  }
//////////////////////////////////////////
//  Set up LCD status dots and manage a counter 
//  to time and display them.
  if (reflowStage != PROBE_CHECK)
  {
    if (reflowStage != IDLE_STAGE)
    {
      dot ++;
      lcd.setCursor(0,0);
      lcd.print(lcdStageMessages[reflowStage]);
      if (dot == 4)
      {
        lcd.print(".  ");
      }
      if (dot == 8)
      {
        lcd.print(".. ");
      }
      if (dot == 12)
      {
        lcd.print("...");
      }
      if (dot == 16)
      {
        lcd.print("   ");
        dot = 0;
      }
    }
  // Display the temperature, degree symbol, and unit.
    lcd.setCursor(0,1);
    lcd.print(input);
    lcd.write(1);
    lcd.print("C         ");
  }
}

//////////////////////////////////////////////
// Interrupt Handler
//  This simple function decides whether to
//  make the next computation (if oven state
//  is true) or to ensure oven is switched
//  off. It is called every 100ms as called
//  by the TwoMsTimer.
//  
//  If anything else needs to go in the ISR place it here
//  else just call Update() from Loop.
void InterruptHandler()
{
  doUpdate = true;
}

void Update()
{
  UpdateLCD();
  doUpdate = false;
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
  //Serial.println(output);
  if ((reflowStage != PROBE_CHECK) || (reflowStage != ERROR_PRESENT))
  {
    long now = millis();
    if(now - windowStartTime > windowSize)
     //Time to shift the Relay Window
      windowStartTime += windowSize;
    if(output > (now - windowStartTime))
      digitalWrite(relayPin,HIGH);
    else
      digitalWrite(relayPin,LOW);
  }
  else
      digitalWrite(relayPin,LOW);
}

//////////////////////////////////////////////
// ReadTemp
//    This function reads the temperature of the 
//    sensor and places the value into the "input"
//    variable.
void ReadTemp()
{
  input = thermo.readThermocouple(CELSIUS);
}

//////////////////////////////////////////////
// Do Control
//    This function performs all the basic PID
//    function calls needed to control the
//    oven properly. It also decides whether to
//    enable/disable the relay and checks for errors
void DoControl()
{
  if (reflowStage != PROBE_CHECK)
  {
    ReadTemp();
    ErrorCheck();
    ovenPID.Compute();
    DriveOutput();
  }
}

void ErrorCheck()
{
  if (reflowStage != ERROR_PRESENT)
  {
    if ((input < 0) || (input >= 280))
    {
      stoppedStage = reflowStage;
      reflowStage = ERROR_PRESENT;
      detachInterrupt(startstopBttn);
      attachInterrupt(startstopBttn, ErrorChoice, FALLING);
    }
  }
}

//////////////////////////////////////////////
// Idle
//    This function is the default reflow
//    stage. It is where the temperature
//    profile is chosen.
void Idle()
{ 
  if (killReflow = true)
  {
    digitalWrite(relayPin,LOW);
    digitalWrite(ledPin,HIGH);
    killReflow = false;
  }
  ReadTemp();
  while (!ovenState)
  {
    // Setting up temporary variable for debouncing
    int pressConfLvl = 0;
    ReadTemp();
    while (!digitalRead(typeBttn))
    {
      // Increment button confidence counter
      pressConfLvl++;
      //  If by now the counter has reached 20000, button
      //  should be pressed and not just bouncing
      if (pressConfLvl > 20000)
      {
        solderType = !solderType;
        // Reset the counter for the next button press
        pressConfLvl = 0;
      }
    }
    if (doUpdate)
      Update();
  }
// Set the variables for the various set points of the 
// program according to the mode selected.
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

  // Monitor the variable in case a cycle was stopped
  // mid-reflow. In case it was, skip asking whether the
  // temperature probe is on the PCB.
    if (asked)
    {
      DoControl();
      reflowStage = PREHEAT_STAGE;
    }
    else
    {
      reflowStage = PROBE_CHECK;
      detachInterrupt(startstopBttn);
      attachInterrupt(startstopBttn, ProbeSet, FALLING);
    }
    CleanLCD();
    digitalWrite(ledPin,LOW);
  }
}

///////////////////////////////////
// Probe
//  Ask the user whether the probe is on the PCB or not.
//  IF IT IS NOT, YOU WILL DAMAGE YOUR PCB!!
void Probe()
{
  // Setting up temporary variable for debouncing
  ReadTemp();
  int pressConfLvl = 0;
  while (!digitalRead(typeBttn))
  {
    // Increment button confidence counter
    pressConfLvl++;
    //  If by now the counter has reached 20000, button
    //  should be pressed and not just bouncing
    if (pressConfLvl > 20000)
    {
      probeOnBoard = !probeOnBoard;
      // Reset the counter for the next button press
      pressConfLvl = 0;
    }
  }

  // If the probe is on the PCB and the start button have both been pressed,
  // proceed with the reflow cycle.
  if(probeState && probeOnBoard)
  {
    CleanLCD();
    detachInterrupt(startstopBttn);
    attachInterrupt(startstopBttn, StartStop, FALLING);
    probeState = false;
    asked = true;
    //Begin countdown after the probe is set and cycle started.
    //This in part is to avoid rushing through the menu.
    countdown = 3;
    UpdateLCD();
    delay(1000);
    countdown = 2;
    UpdateLCD();
    delay(1000);
    countdown = 1;
    UpdateLCD();
    delay(1000);
    CleanLCD();
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
  asked = true;
  countdown = 0;
  if (((SOAK_MIN + 5) > input) && (input >= SOAK_MIN))
  {
    timerSoak = millis() + SOAK_MICRO_PERIOD;
    setpoint = SOAK_MIN + SOAK_STEP;
    ovenPID.SetTunings(KP_SOAK, KI_SOAK, KD_SOAK);
    reflowStage = SOAK_STAGE;
    CleanLCD();
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
  asked = true;
  if (millis() > timerSoak)
  {
    timerSoak = millis() + SOAK_MICRO_PERIOD;
    setpoint += SOAK_STEP;
    if (((SOAK_MAX + 5) > input) && (input > SOAK_MAX))
    {
      ovenPID.SetTunings(KP_REFLOW, KI_REFLOW, KD_REFLOW);
      setpoint = REFLOW_MAX;
      reflowStage = REFLOW_STAGE;
      CleanLCD();
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
  asked = true;
  if (((REFLOW_MAX + 5) > input) && (input >= (REFLOW_MAX)))
  {
    setpoint = COOL_MIN;
    reflowStage = COOL_STAGE;
    CleanLCD();
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
    CleanLCD();
    asked = true;
  }
}

//////////////////////////////////////////////
// Complete
//    This is just a place-holder forced wait
//    time to display the fact that the
//    process is complete so the user knows
//    they can take the soldered boards out
//    of the oven. Maybe in the next revision
//    of the hardware I'll add a buzzer?
void Complete()
{
  ovenState = false;
  delay(2500);
  reflowStage = IDLE_STAGE;
  CleanLCD();
  asked = false;
  doUpdate = false;
  probeState = false;
  probeOnBoard = false;
  asked = false;
}

//////////////////////////////////////////////
// Error
//    This function alerts the user that there
//    was an error in reading the thermocouple
//    temperature. Either the thermocouple is
//    disconnected or something else is wrong.
void Error()
{
  digitalWrite(relayPin,LOW);
  digitalWrite(ledPin,HIGH);
    // Setting up temporary variable for debouncing
  int pressConfLvl = 0;
  while (!digitalRead(typeBttn))
  {
    // Increment button confidence counter
    pressConfLvl++;
    //  If by now the counter has reached 20000, button
    //  should be pressed and not just bouncing
    if (pressConfLvl > 20000)
    {
      continueWithError = !continueWithError;
      // Reset the counter for the next button press
      pressConfLvl = 0;
    }
  }
  if (continueState && continueWithError)
  {
    CleanLCD();
    delay(100);
    detachInterrupt(startstopBttn);
    attachInterrupt(startstopBttn, StartStop, FALLING);
    reflowStage = stoppedStage;
    continueState = !continueState;
    continueWithError = !continueWithError;
  }
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
  delayMicroseconds(20000);
  if (!digitalRead(startstopBttn))
    ovenState = !ovenState;
  if (reflowStage != IDLE_STAGE)
    reflowStage = IDLE_STAGE;
    killReflow = true;
}
//////////////////////////////////////////////
// ProbeSet
//    This function handles the software
//    debouncing of the Start/Stop button, as
//    well as handling the setting of the
//    probeState variable.
void ProbeSet()
{
  delayMicroseconds(20000);
  if (!digitalRead(startstopBttn))
    probeState = !probeState;
}

void ErrorChoice()
{
  delayMicroseconds(20000);
  if (!digitalRead(startstopBttn))
    continueState = !continueState;
}

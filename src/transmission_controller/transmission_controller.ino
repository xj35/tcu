//Transmission Controller
//Carter Moursund
//June, 2021

// This version uses interrupts to measure speed and RPM, because the previous polled version was gimpy and random

// 4L60E technical manual: http://jd.offroadtb.com/trans-docs/4l60e-electronic-controls.pdf

#include <ArduinoBLE.h>
#include <assert.h>

// Define Pins
#define SPEED_IN    12   // Line to speed sensor interface board

#define K_LINE_TX   11   // RX/TX to K_LINE on OBD-2 Connector Honda Accord
#define K_LINE_RX   10   // This doesn't work - can't get a UART to compile

#define TR_SOL_A    2    // 4L60E transmission control solenoids
#define TR_SOL_B    3

#define MLPS_A      9    // Pins to transmission gear selector switch
#define MLPS_B      8
#define MLPS_C      7

#define SHIFT_UP_IN 5    // Connected to gear up/down joystick (or buttons or whatever)
#define SHIFT_DN_IN 4

#define RPM_IN      A0
#define RPM_ON      1
#define RPM_OFF     0

#define FET_0       A1   // Transmission Pressure Control Solenoid, OFF is MAXIMUM PRESSURE
#define PCS         A1   // Transmission Pressure Control Solenoid, OFF is MAXIMUM PRESSURE

#define FET_1       A2   // 3-2 Shift solenoid valve control Off in first gear, 90% on in all other gears
#define SH_SOL_32   A2   // 3-2 Shift solenoid valve control Off in first gear, 90% on in all other gears

#define FET_2       A3   // Random PWM outputs connected to FETs for future expansion
#define FET_3       A4   // Random PWM outputs connected to FETs for future expansion

#define FET_4       A5   // Variable Frequency Speed Output to PS Pump
#define VS_PS       A5   // Variable Frequency Speed Output to PS Pump

#define FET_5       A6   // Random PWM outputs connected to FETs for future expansion

#define RELAY_0     6    // Random digital outputs connected to relays for future expansion
#define RELAY_1     13


// Define Pin States
#define TR_SOL_ON   1    // This will turn on the relay that will GROUND the solenoid line and turn it ON
#define TR_SOL_OFF  0    // This will turn off the relay that will allow the solenoid line to float and turn if OFF
#define SPEED_ON    1    // The Speed IO is a pull down; This doesn't really matter as it is a 50% duty cycle square wave
#define SPEED_OFF   0    // It is configured with a pull up

#define SH_SOL_VAL  230  // 90% duty cycle times 255 = 230

// Define Timeouts
#define SPEED_TIMEOUT_MS       (MPH_US/1000)  // if we're going really slow, we may need to wait a while
#define SHIFT_DEB_PERIOD       20000            // 20 milliseconds for gear shift to stabliize
#define IDLE_TIME              250              // 250 milliseconds in idle before we'll look for the next shift
#define RPM_POLL_TIME          250              // 250 milliseconds between reading RPM from the engine
#define SPEED_POLL_TIME        100              // 100 milliseconds between calculating speed
#define REDOWN_SHIFT_TIME      1000             // 1000 milliseconds before a follow on hold the button downshift, even if the engine is OK
#define UPDATE_TIME            500              // 100 miliseconds between serial and BLE updates
#define MONEY_TIME             1000             // 1000 milliseconds of inidcating a failed money shift
#define RPM_TIMEOUT            40000           // 40000 microseconds before we give up on seeing a motor pulse edge (equivalent to 250 RPM)

// Define Constants

#define TIRE_DIA_IN           ((unsigned long) 26)
#define REV_PER_MILE          ((unsigned long) (12 * 5280 * 10000) / (TIRE_DIA_IN * 31415))
#define FINAL_DRIVE_RATIO_K   3730  // Final back end drive ratio x1000
#define RED_LINE              5000  // The maximum RPM we'll allow a downshift in to
#define PULSES_PER_REV        40    // 4L60E's claim to have 40 pulses per revolution of the drive shaft
// #define INCHES_PER_PULSE      5.907  // PI * TIRE_DIA_IN * FINAL_DRIVE_RATIO / PULSES_PER_REV  (milli-inches per pulse) using 23, 3.27, 40
// #define MILES_PER_PULSE       9.322E-5  // INCHES_PER_PULSE_K/1000 / (12*5280) carrying on using 23, 3.27, 40
// #define MPH_US                ((unsigned long) 14592 * TIRE_DIA_IN)  // MPH * microseconds per pulse (9.322E-5 * 1000000 us/sec * 3600 sec/hr)
#define MPH_US                 (((unsigned long) 179 * TIRE_DIA_IN * (unsigned long) FINAL_DRIVE_RATIO_K) / (unsigned long) PULSES_PER_REV)
                            // 179 =~ PI / 1000 * 1000000* 3600 / (5280 * 12) so these other variables get us the right number


// Drive State Machine Names

#define DRIVE_IDLE          0   // No switches, waiting for inputs
#define DEBOUNCE_SHIFT_UP   1   // Saw the up switch, waiting to make sure it is stable
#define DEBOUNCE_SHIFT_DOWN 2   // Saw the down switch, waiting to make sure it is stable
#define WAIT_FOR_IDLE       3   // Just changed gears, waiting for both switches to return to idle
#define WAIT_FOR_REDOWN     4   // Special mode, hold the downshift and it'll wait for REDOWN_SHIFT_TIME ms and then try to downshift


// Global Variables

#include "ble.h"


BLEService carService(BLE_UUID_TEST_SERVICE); // create service

BLEByteCharacteristic GearBLEChar(BLE_UUID_GEAR, BLERead | BLENotify  );       // Current gear selected by solenoids
BLEByteCharacteristic SpeedBLEChar(BLE_UUID_SPEED, BLERead | BLENotify  );     // Current speed of the car
BLEByteCharacteristic ShifterBLEChar(BLE_UUID_SHIFTER, BLERead | BLENotify );  // Current position of the Shift Lever
BLEIntCharacteristic RPMBLEInt(BLE_UUID_RPM, BLERead | BLENotify );           // Current RPM of the engine
BLEByteCharacteristic MoneyShiftBLEChar(BLE_UUID_MONEY, BLERead | BLENotify ); // Indicates excess downshift for 1 second

// Uart dlcSerial (&sercom0,K_LINE_RX,K_LINE_TX,SERCOM_RX_PAD_1, UART_TX_PAD_0); // Set up the bit bang serial port, full duplex on the selected pins - Doesn't work won't compile


int BLEstatus;     // Bluetooth Status
char BLEconstate;  // Bluetooth Connection status for Serial Debugging

// Speed Sensor
int Speed = -1;                // Most recent Speed calculated, -1 to begin with
volatile int New_Speed = 0;    // Indicates a new time has been stored
volatile unsigned long Speed_Time = 0;       // Time in microseconds since last speed edge
volatile unsigned long Last_Speed_Time = 0;  // Time in microsends when the last Speed edge was received

const unsigned long TrannyGearRatio_K[4] = {4027, 2364, 1532, 1152};  // 4L60E gear ratios x1000


// Gear information
int Gear;                      // Currently Gear Selected by Solenoids

unsigned long RPM_time_ms;        // Time in milliseconds to next read RPM;
unsigned long Speed_time_ms;      // Time in milliseconds to next read Speed;
unsigned long Shift_Deb_time_us;  // Time in microseconds when we started debouncing the current switch
unsigned long Update_time_ms;     // Time in milliseconds to next update display

int RPM = -1;                     // Most recent RPM reading
volatile int New_RPM = 0;                  // Check that the signal has been triggered
volatile unsigned long RPM_Time = 0;       // Time in microseconds since the last RPM edge
volatile unsigned long Last_RPM_Time = 0;  // Time in microseconds when the last RPM edge was received

int DriveState = DRIVE_IDLE;      // Driving State Machine - start in IDLE
char ShiftFail = 0;               // Have we tried a failed shift due to RPM
unsigned long ShiftFailTime_ms;   // Hold the shift fail signal for 1 second
unsigned long IdleTime_ms;        // How long we have to wait before doing another shift

unsigned long VS_PS_Switch_us = 0;    // How long to keep the VSPS line in current state for 50% duty cycle variable frequency output
int VS_PS_Pin = 0;                    // State of the pin, starts out off
  

/*****************************************************************************
* IsSafeToShiftDown
* Parameters: CurrentGear (car's current gear), CurrentRPM (car's current RPK)
* Return Value: bool (true if we can shift down without exceeding red line)
* Globals: None
*
* Determines whether or not a down shift is possible without exceeding
* red line at the current gear and RPM.
******************************************************************************/
bool IsSafeToShiftDown(int CurrentGear, int CurrentRPM)
{
  assert(0 < CurrentGear && CurrentGear < 5);
  if (CurrentGear == 1)
  {
    // can't downshift from 1
    return false;
  }
  int CurrentGearRatio = TrannyGearRatio_K[CurrentGear - 1];
  int NewGearRatio = TrannyGearRatio_K[CurrentGear - 2];
  int TargetRPM = (CurrentRPM * NewGearRatio) / CurrentGearRatio;

  char buf[128];
  sprintf(buf, "MONEY Gear %d RPM %d NEW RPM %d", CurrentGear, CurrentRPM, TargetRPM);
  Serial.write(buf);
  
  return TargetRPM <= RED_LINE;
}


/*****************************************************************************
* RPM_ISR
* Parameters: none
* Return Value: none
* Globals: RPM_Time, Last_RPM_Time, New_RPM
* 
* Stores the time in microseconds since the last RPM edge
******************************************************************************/
void RPM_ISR()
{
  unsigned long t;
  unsigned long h;

  t = micros();

  // Getting some quantization noise in this ISR, so added a light averaging function
  // New Avg = ((Old Value * 7) + New Value) / 8
  // Seems to have knocked down the error quite a bit on a static function generator
  
  RPM_Time = RPM_Time * 7;    
  RPM_Time += (t - Last_RPM_Time);
  RPM_Time = RPM_Time >> 3;
  New_RPM = 1;

  Last_RPM_Time = t;
}

/*****************************************************************************
* Speed_ISR
* Parameters: none
* Return Value: none
* Globals: Speed_Time, Last_Speed_Time, New_Speed
* 
* Stores the time in microseconds since the last Speed sensor edge
******************************************************************************/
void Speed_ISR()
{
  unsigned long time;

  time = micros();

  Speed_Time = time - Last_Speed_Time;
  New_Speed = 1;

  Last_Speed_Time = time;
}

/*****************************************************************************
* setup
* Parameters: None
* Return Value: None
* Globals: All of them are initialized here
* Arduino start point. Initializes all HW, turns on BLE, gets Honda computer
* talking to us. Initializes internal serial port over USB for debugging 
* messages
******************************************************************************/
void setup() {
  char GearHnd;                // Where the gear selector handle is
  unsigned long PickGearTime;  // When we started looking for speed
  unsigned long PickRPMTime;   // When we started looking for RPMs
  
  pinMode(TR_SOL_A, OUTPUT);  // Solenoids are outputs and we want to get this thing into a reasonable gear immediately if we reset randomly
  pinMode(TR_SOL_B, OUTPUT);
  
  Serial.begin(9600);  // Debugging port
  Serial.print("Start\n");
  
  pinMode(SPEED_IN, INPUT);        // Speed sensor chip has its own pullup
  pinMode(RPM_IN, INPUT_PULLUP);   // RPM clipped pulses will come in here when grounded, otherwise pull up
  pinMode(MLPS_A, INPUT_PULLUP);   // These switches just ground the line, so we use internal pull ups
  pinMode(MLPS_B, INPUT_PULLUP);
  pinMode(MLPS_C, INPUT_PULLUP);

  pinMode(SHIFT_UP_IN, INPUT_PULLUP);  // This switch just grounds the joystick, so we use internal pull ups
  pinMode(SHIFT_DN_IN, INPUT_PULLUP);
  
  pinMode(K_LINE_RX, INPUT_PULLUP);    // Bit banged serial port to K-Line Transceiver
  pinMode(K_LINE_TX, OUTPUT);
  digitalWrite(K_LINE_TX, 1);          // Idle state for transceiver

  analogWrite(FET_0,0);  // Turn off pressure control valve for maximum pressure
  analogWrite(SH_SOL_32,0);  // Start off, will get reset shortly based on actual gear
  analogWrite(FET_2,0);  
  analogWrite(FET_3,0);  // Can only easily do 4 PWM outputs, turn them all off
  
  pinMode(VS_PS, OUTPUT);  // Just make the last 2 FETs digital ON/OFF
  pinMode(FET_5, OUTPUT);
  
  pinMode(RELAY_0, OUTPUT);   // Extra relays to someday turn things on/off
  pinMode(RELAY_1, OUTPUT);
  
  digitalWrite(VS_PS,0);      // Do not ground VS_PS so it is on to maximum
  digitalWrite(FET_5,0);      // Turn off all the extra outputs
  digitalWrite(RELAY_0, 0);
  digitalWrite(RELAY_1, 0);
  
  // Figure out our starting gear
  GearHnd = GetGearHandle();  // See what gear the gear handle is set to

  attachInterrupt(digitalPinToInterrupt(SPEED_IN), Speed_ISR, RISING);  // Make rising edges of Speed line trigger ISR, do this here so we can see the speed
  attachInterrupt(digitalPinToInterrupt(RPM_IN), RPM_ISR, RISING);      // Make rising edges of RPM line trigger ISR
  
  switch (GearHnd)  
  {
    case 'P':
    case 'R':
    case 'N':
      GearSelect(1);  // 4L60E manual says to put solenoids in ON,ON state if in P,N, or R
      break;
    case 'D':
    case '3':
    case '2':
    case '1':
    case 'X':
         // The transmission controller has restarted and we're in some flavor of drive; we don't know the speed, so we need to sense speed
         // Or the gear handle switch is giving us nonsense and we don't know which gear we've selected

      // Determine Speed
      
      PickGearTime = millis();
      while (Speed == -1)
      {
        if (Speed_Time > 0)
        {
          Speed = MPH_US / Speed_Time;
          New_Speed = 0;
        }
        else
        {
          if ((millis() - PickGearTime) > SPEED_TIMEOUT_MS)
          {
            Speed = 0;
          }
        }
      }
    
      if (Speed > 0)   // We got a good speed, automatically pick a good gear
      {
        GearSelect(GearSelectBySpeed(Speed));
      }
      else
      {
        GearSelect(3);   // Speed sensor is hosed or we're stopped, pick 3rd to be safe
      }
      break;
    default:
      GearSelect(3);    // Gear handle function giving us nonsense
      break;
  }

  // Determine RPM
  
  PickRPMTime = micros();   // Get a start time so we can exit if the engine is stopped
  
  while (RPM == -1)
  {
    if (RPM_Time > 0)
    {
      RPM = (unsigned long) 20000000 / RPM_Time;  // 1E6 microsec/sec * 60 sec/min / 3 fires/rev = 2E7 microsec RPMs
      New_RPM = 0;
    }
    else
    {
      if ((micros() - PickRPMTime) > RPM_TIMEOUT)
      {
        RPM = 0;      // We waited a good amount of time and there were no pulses, the engine is stopped
      }
    }
  }
  
  
  // START UP THE BLE COMMUNICATION STREAM
  
  BLEstatus = BLE.begin();

  BLE.setDeviceName("CarData");  // This is what we will call ourselves, look for this name when Bluetooth connecting
  BLE.setLocalName("CarData");
  BLE.setAdvertisedService(carService);
  
  carService.addCharacteristic(GearBLEChar);   // Add all the characteristics we want to share
  carService.addCharacteristic(SpeedBLEChar);
  carService.addCharacteristic(ShifterBLEChar);
  carService.addCharacteristic(RPMBLEInt);
  carService.addCharacteristic(MoneyShiftBLEChar);

  BLE.addService(carService);  // Make the service available on the device

  GearBLEChar.writeValue(Gear);  // Set the characteristics to their initial values we just determined
  SpeedBLEChar.writeValue(Speed);
  ShifterBLEChar.writeValue(GearHnd);
  RPMBLEInt.writeValue(RPM);
  MoneyShiftBLEChar.writeValue(0);

  BLE.advertise();  // Let the world know we are here

  Serial.print("BLE started! Waiting for connections\n");

  Update_time_ms = RPM_time_ms = millis();
  
  Serial.print("Out of Start!\n");

}

/*****************************************************************************
* loop
* Parameters: None
* Return Value: None
* Globals: RPM and Speed are used
* This routine is repeatedly called by the arduino core code
* This code cannot block, because other functions are getting called for 
* updates outside of this code.
* This code does the following:
*   Make calls to update speed and RPM state machines and values
*   Check the gear handle
*   While in the drive gears, run a state machine on the gear up/dn shifter
*   Execute those shifts
*   Update status via debugging serial and BLE tags
*****************************************************************************/
void loop() {
  // put your main code here, to run repeatedly: NOTE DO NOT GET STUCK HERE OR BLE WILL FAIL WEIRDLY
  char GearHnd;  // Current position of the gear selector handle
  char SDS[40];  // Serial Debug String
    
  GearHnd = GetGearHandle();   // See where the gear handle is

  if ((millis() - Speed_time_ms) >= SPEED_POLL_TIME)  // Periodically calculate the speed - it isn't going to change that much
  {
    if (Speed_Time > 0)
    {
      if (New_Speed)   // Speed was updated since the last time we checked
      {
        Speed = MPH_US / Speed_Time;
        New_Speed = 0;
      }
      else
      {
        Speed = 0;    // It has been a really long time, so speed is zero or sensor is hosed
      }
    }
    else
    {
      Speed = 0;  // Still waiting for first pulse
    }
    
    Speed_time_ms = millis();
  }

  UpdateVSPS();             // Update the frequency output to the Vehicle Speed to Power Steering Pin

  if ((millis() - RPM_time_ms) >= RPM_POLL_TIME)  // Periodically calculate the RPM, it changes often, but it doesn't matter much here
  {
    if ((RPM_Time > 0) && (RPM_Time < 100000))   // Only calculate reasonable RPMs (100 to 10000000)
    {
      if (New_RPM)
      {
        RPM = (unsigned long) 20000000 / RPM_Time;  // 1E6 microsec/sec * 60 sec/min / 3 fires/rev = 2E7 microsec RPMs
        New_RPM = 0;
      }
      else
      {
        RPM = 0;   // We haven't gotten a new RPM pulse since the last update time, engine is dead
      }
    }
    else
    {
      RPM = 0;   // We're still waiting for the engine to start, or we got the first pulse after a really long time
    }
    
    RPM_time_ms = millis();
  }
  
  

  if ((GearHnd == 'P') || (GearHnd == 'R'))  // 4L60E technical manual says 1st gear for P or R
  {
    if (Speed < 30)
    {   // Check that we're going slow and we're not just dealing with a loose connection before slamming it into 1st gear
      GearSelect(1);
    }
  }

  if (GearHnd == 'N')  // 4L60E technical manual says 1st gear, but I want the transmission to be ready to go back into gear
  {
    GearSelect(GearSelectBySpeed(Speed));
  }

  if ((GearHnd == 'D') || (GearHnd == '3') || (GearHnd == '2') || (GearHnd == '1'))   // This is the normal driving, with manual shifting
  {
    switch (DriveState)
    {
      case DRIVE_IDLE:
        if (digitalRead(SHIFT_UP_IN) == LOW)
        {
          Shift_Deb_time_us = micros();
          DriveState = DEBOUNCE_SHIFT_UP;
        }
        else
        if (digitalRead(SHIFT_DN_IN) == LOW)
        {
          Shift_Deb_time_us = micros();
          DriveState = DEBOUNCE_SHIFT_DOWN;
        }
        break;
      case DEBOUNCE_SHIFT_UP:
        if (digitalRead(SHIFT_UP_IN) == LOW)
        {
          if ((micros() - Shift_Deb_time_us) >= SHIFT_DEB_PERIOD)  // We have waited long enough
          {
            switch (GearHnd)
            {
              case 'D':                  // We're in full drive, select any gear
                GearSelect(Gear + 1);    // GearSelect will wrap this back to 4 as needed
                break;
              case '3':
              case '2':
              case '1':
                if ((Gear + 1) <= (GearHnd - '0'))
                  GearSelect(Gear + 1);   // We always allow shift up if the mechanical gear select handle is in the right gear
                break;
              default:
                break;
            }
            
            DriveState = WAIT_FOR_IDLE;    // Wait until that switch goes back idle before processing another shift
            IdleTime_ms = millis();           // How long we have to wait before we shift again
          }
        }
        else
        {
          DriveState = DRIVE_IDLE; // Restart the count, the switch didn't stay in state
        }
        break;
      case DEBOUNCE_SHIFT_DOWN:
        if (digitalRead(SHIFT_DN_IN) == LOW)
        {
          if ((micros() - Shift_Deb_time_us) >= SHIFT_DEB_PERIOD)  // We have waited long enough
          {
            if (IsSafeToShiftDown(Gear, RPM))
            {                
              GearSelect(Gear - 1);            // We won't blow up the engine: do the downshift
              // DriveState = WAIT_FOR_IDLE;   // We're not doing this, because you can hold the down shift and it'll keep downshifting until you let up
              // IdleTime_ms = millis();          // If we decide to force a return to the middle
              // We still need to wait a while, so we'll put that code in here
              DriveState = WAIT_FOR_REDOWN;    // Even if you hold the handle down and the engine is OK, we need to let the transmission catch up
              IdleTime_ms = millis();
            }
            else
            {
              ShiftFail = 1;
              ShiftFailTime_ms = millis();   // So we can turn on an indicator for a period of time
            }
          }
        }
        else
        {
          DriveState = DRIVE_IDLE; // Restart the count, the switch didn't stay in state
        }
        break;
      case WAIT_FOR_IDLE:
        if ((digitalRead(SHIFT_DN_IN) == HIGH) && (digitalRead(SHIFT_UP_IN) == HIGH))
        {
          if ((millis() - IdleTime_ms) >= IDLE_TIME)
          {
            DriveState = DRIVE_IDLE;
          }
        }
        else
        {
          IdleTime_ms = millis();    // Reset the idle time until the button is in idle for the right time
        }
        break;
      case WAIT_FOR_REDOWN:
        if (digitalRead(SHIFT_DN_IN) == LOW)
        {
          if ((millis() - IdleTime_ms) >= REDOWN_SHIFT_TIME)
          {
            DriveState = DEBOUNCE_SHIFT_DOWN;  // It's already debounced, so this will trigger
          }
        }
        else
        {
          IdleTime_ms = millis();
          DriveState = WAIT_FOR_IDLE;   // We let up on the button, so we're going to wait for it to really idle
        }
        break;
      default:  // Something went very wrong, start over and try again
        DriveState = DRIVE_IDLE;
    }
  }

  // This is where we would display things if we had a display, so we update the BLE for our display
  if (millis() - Update_time_ms >= UPDATE_TIME)
  {
    BLEDevice central = BLE.central();
  
    // Update BLE
    if (central == false)
    {
      BLEconstate = 'X';
    }
    else
    {
      BLEconstate = 'C';

      ShifterBLEChar.writeValue(GearHnd);
      SpeedBLEChar.writeValue(Speed);
      GearBLEChar.writeValue(Gear);
      RPMBLEInt.writeValue(RPM);

      BLE.advertise();
    }

    Update_time_ms = millis();

    sprintf(SDS,"%4d %9lu %3d %c %d %d %c",RPM,RPM_Time,Speed,GearHnd,DriveState,Gear,BLEconstate);
    Serial.write(SDS);

    if (ShiftFail)
    {
      MoneyShiftBLEChar.writeValue(1);
      Serial.write("S!\n");

      if ((millis() - ShiftFailTime_ms) > 1000)
      {
        ShiftFail = 0;
        MoneyShiftBLEChar.writeValue(0);
      }
    }
    else
    {
      Serial.write("\n");
    }
  }
}

/*****************************************************************************
* GearSelect
* Parameters: GearIn - the gear we would like to try to shift in to
* Return Value: None
* Globals: Gear - actual current gear is set here
*          Arduino Output Pins for gear select relays are set
*          Arduino Output Pin for 3-2 Shift solenoid is set
* This code takes in a requested gear shift, checks if it is in range
* and then sets the solenoids can the current Gear value appropriately
* It does not check of the gear selector handle is in a particular position
* so other code should do that before sending a shift.
******************************************************************************/

void GearSelect(int GearIn)
{
  if (GearIn < 1)
  {
    GearIn = 1;
  }

  if (GearIn > 4)
  {
    GearIn = 4;
  }

  switch (GearIn)
  {
    case 1:
      digitalWrite(TR_SOL_A, TR_SOL_ON);
      digitalWrite(TR_SOL_B, TR_SOL_ON);
      analogWrite(SH_SOL_32,0);  // 3-2 Shift solenoid is off in first gear
      Gear = 1;
      break;
    case 2:
      digitalWrite(TR_SOL_A, TR_SOL_OFF);
      digitalWrite(TR_SOL_B, TR_SOL_ON);
      analogWrite(SH_SOL_32, SH_SOL_VAL);  // 3-2 Shift solenoid is on a set amount in all other gears
      Gear = 2;
      break;
    case 3:
      digitalWrite(TR_SOL_A, TR_SOL_OFF);
      digitalWrite(TR_SOL_B, TR_SOL_OFF);
      Gear = 3;
      analogWrite(SH_SOL_32, SH_SOL_VAL);  // 3-2 Shift solenoid is on a set amount in all other gears
      break;
    case 4:
      digitalWrite(TR_SOL_A, TR_SOL_ON);
      digitalWrite(TR_SOL_B, TR_SOL_OFF);
      analogWrite(SH_SOL_32, SH_SOL_VAL);  // 3-2 Shift solenoid is on a set amount in all other gears
      Gear = 4;
      break;
    default:
      digitalWrite(TR_SOL_A, TR_SOL_OFF);
      digitalWrite(TR_SOL_B, TR_SOL_OFF);
      analogWrite(SH_SOL_32, SH_SOL_VAL);  // 3-2 Shift solenoid is on a set amount in all other gears
      Gear = 3;
      break;
  }
}

/*****************************************************************************
* GetGearHandle
* Parameters: None
* Return Value: char - a letter representing the current gear handle position
                'P' - Park, 'R' - Reverse, 'N' - Neutral, 'D' - Drive (OD/4)
                '3' - 3rd, '2' - 2nd, '1' - 1st, 'X' - Invalid/Error
* Globals: none
*          Arduino Input Pins for MLPS are read
* This code looks at the 3 MLPS bits that are set by the switch controlled
* by the gear handle. Based on those 3 bits, the actual gear is decoded by
* a logic table, where L is grounded and H is internal pull up (not connected)
*
*             A   B   C   P(not used)
* Park        L   H   H   L
* Reverse     L   L   H   H
* Neutral     H   L   H   L
* OD / 4 / D  H   L   L   H
* D 3         L   L   L   L
* D 2         L   H   L   H
* D 1         H   H   L   L
*
******************************************************************************/

char GetGearHandle()
{
  char MLPS_Switches = 0;

  if (digitalRead(MLPS_A) == HIGH)
  {
    MLPS_Switches += 4;
  }

  if (digitalRead(MLPS_B) == HIGH)
  {
    MLPS_Switches += 2;
  }

  if (digitalRead(MLPS_C) == HIGH)
  {
    MLPS_Switches += 1;
  }

  switch (MLPS_Switches)
  {
    case 0: return '3';
    case 1: return 'R';
    case 2: return '2';
    case 3: return 'P';
    case 4: return 'D';
    case 5: return 'N';
    case 6: return '1';
    default: return 'X';
  }

  return 'X';
}

/*****************************************************************************
* GearSelectBySpeed
* Parameters: InputSpeed - integer speed you want to use to select a gear
* Return Value: An appropriate gear for that speed
* Globals: none
* This code takes in a speed and picks a safe gear for that speed
* It isn't particularly aggressive, and will put the engine in its midband
* for the speed. This function should only get used if something bad happened
* to the transmission controller and we reset while driving and we need to
* pick a good gear.
******************************************************************************/


int GearSelectBySpeed(int InputSpeed)
{
  int Select;
  
  Select = (InputSpeed / 15) + 1;    // This formula needs to be confirmed by math; puts the motor at around 2000-3000 RPM before picking a higher gear

  if (Select >= 4)
    Select = 4;

  return Select;
}

/*****************************************************************************
* RPMLookUp
* Parameters: InputSpeed - the speed you want to turn into engine RPMs
              InputGear - the gear you want to check for that RPM calculation
* Return Value: RPM as unsigned long
* Globals: none
* This code takes in a speed and a gear and calculates approximately what the
* engine RPMs will be. This is the key code to avoid a money shift. The 
* calculation below has been integer optimized, but the comments show the
* mathematical translation to get these values.
* Changes in tire diameter, final drive ratio, or transmission will cause
* the 47 value below to need to be recalculated
******************************************************************************/

unsigned long RPMLookUp(int InputSpeed, int InputGear)
{
  unsigned long RetRPM = 20000;

  // (RPM / TrannyGearRatio[InputGear] / FinalDriveRatio) * TireCircumference (miles) * 60 = SpeedInMPH
  // RPM = SpeedInMPH * (TrannyGearRatio[InputGear] * FinalDriveRatio) / (TireCircumerence (miles) * 60)
  // RPM = SpeedInMPH * RevolutionsPerMile * (TrannyGearRatio[InputGear] * FinalDriveRatio / 60
  // If we want this to stay integer math, then we need to multiply by the inverse of TireCircumerence
  // We also put all the ratios times 1000 to get 3 digits of precision that we divide out

  if (InputGear < 1)
  {
    InputGear = 1;
  }
  if (InputGear > 4)
  {
    InputGear = 4;
  }
  
  RetRPM = (InputSpeed * TrannyGearRatio_K[InputGear-1] * 47) / 1000;

  return RetRPM;
}

/*****************************************************************************
* UpdateVSPS
* Parameters: None
* Return Value: None
* Globals: Speed - int speed of vehicle in MPH
*          VS_PS_Switch_us - unsigned long time to next switch in pin state
*          VS_PS - Digital output pin tied to power steering pump
*          VS_PS_Pin - int stores current state of VS_PS pin, starts off
* This code computes a transfer function from the speed of the vehicle to
* a 50% duty cycle square wave of variable frequency sent to the electric
* power steering pump to control power steering assist
******************************************************************************/
void UpdateVSPS()             // Update the frequency output to the Vehicle Speed to Power Steering Pin
{
  if (Speed == 0)
  {
    digitalWrite(VS_PS,0);    // Turn off the FET and let the VSPS line float
    VS_PS_Pin = 0;
    VS_PS_Switch_us = micros();
  }
  else
  {
    if (micros() >= VS_PS_Switch_us)
    {
      if (VS_PS_Pin == 0)
      {
        digitalWrite(VS_PS,1);
        VS_PS_Pin = 1;
      }
      else
      {
        digitalWrite(VS_PS,0);
        VS_PS_Pin = 0;
      }
      
      // Calculate period for next pin state based on speed
      // First try transfer function is 5 Hz at 1 MPH to 200 Hz at 40+ MPH
      // Frequency = Speed * 5
      // Period in us = 1000000 / Frequency
      // 50% duty cycle = Period in us / 2
      // Increment period in us = 100000 / Speed
      // Note we break for zero speed above, so no DIV/0 problem here
      
      if (Speed < 40)
      {
        VS_PS_Switch_us = micros() + ((unsigned long) 100000 / Speed);
      }
      else
      {
        VS_PS_Switch_us = micros() + 2500;
      }
    }
  }
}

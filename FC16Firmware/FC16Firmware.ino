// Fox Cub 16 Firmware for Atmega328P-PU
//
// Copyright: Jetty 2012, V0.9Alpha
// License: Creative Commons - Non Commercial - Attribution - ShareAlike
// http://www.creativecommons.org/
//
// DISCLAIMER: USE THIS SOFTWARE AT YOUR OWN RISK AND YOUR OWN LIABILITIY.
// IF YOU DON'T UNDERSTAND WHAT YOU'RE DOING WITH ELECTRONICS AND POWER, GET PROFESSIONAL ADVICE.
// IF YOU'RE USING 240V, YOU WILL NEED TO CHANGE THE CIRCUIT, FIRMWARE AND APPROACH.  240V CONTAINS
// CONSIDERABLY MORE ENERGY THAN 110V.
//
// Note 1: Assumes a 16MHz Crystal, will require adjustments to FREQ_XXHZ_TIMER below if you use 20MHz.
//
// Note 2: Programming jumper must be removed whilst uploading firmwre, and in place during operation
//
// Built on Arduino IDE 1.0.1


#include <EEPROM.h>
#include "FC16Firmware.h"



// Set to FREQ_50HZ_TIMER or FREQ_60HZ_TIMER depending on your power frequency
#define POWER_FREQUENCY  FREQ_60HZ_TIMER

// Timeout in seconds, after which if there has been no serial communication, and a channel
// is still on, the channel is automatically switched off.  Prevents lights being stuck on if
// your computer crashes or dies.
#define INACTIVITY_TIMEOUT (1 * 60 * 120 )  //1 Min * 60 Seconds * (60Hz * 2)

// Zero crossing detection pin - Must be a pin change interrupt (INT0)
// Goes HIGH on zero crossing
#define ZERO_CROSSING_PIN  D2

// Config switch pin - Must be a pin change interrupt (INT1)
// Goes LOW when pressed
#define CONFIG_SWITCH_PIN  D3

// Timing for switch debouncing
#define DEBOUNCE_CONFIG_SWITCH_DELAY  50  //milliseconds

// The total number of channels
#define CHANNELS 16

// We access the hardware ports for the channels instead of using digitalWrite
// to save cycles.  ssrPortIndex and ssrBit should all match to identify a particular
// Arduino pin.  Reference the Atmega328 datasheet for pin to hardware port mapping

// An index number identifying the port
// 0 = PORTB, 1 = PORTC, 2 = PORTD
uint8_t ssrPortIndex[CHANNELS] = {
  2, 2, 2, 2,
  0, 0, 0, 0,
  0, 0, 1, 1,
  1, 1, 1, 1
};

// The bit of the port we need to change for a particular channel
uint8_t ssrBit[CHANNELS] = {
  4, 5, 6, 7,
  0, 1, 2, 3,
  4, 5, 0, 1,
  2, 3, 4, 5
};

// Duration of the config switch short hold (in milliseconds)
#define CONFIG_SWITCH_SHORT_HOLD_DURATION  5000L

// Duration of the config switch long hold (in milliseconds)
#define CONFIG_SWITCH_MEDIUM_HOLD_DURATION   10000L

// Duration of the config switch long hold (in milliseconds)
#define CONFIG_SWITCH_LONG_HOLD_DURATION   15000L

//
// NO USER TUNABLE STUFF BEYOND HERE
//

// Starting channel number
// Note this the channel number as it's known to vixen, i.e. indexed from 1
// So, this number should be in units of 16 channels, i.e.
// 1, 17, 33, 49, 65
uint8_t startChannel;  //Units of 16 + 1, i.e. 1, 17, 33, 49 65 etc.

#define START_CHANNEL_EEPROM_ADDRESS 1  //We don't use 0, as 0 is often subject to corruption on a crash

// The renard protocol is processed by a finite state machine, we define the states here
enum RenardState {
  RS_WAIT_SYNC = 0,
  RS_WAIT_CMD_ADDRESS,
  RS_WAIT_ESCAPE,
  RS_WAIT_DATA,
};

enum RenardState renardState = RS_WAIT_SYNC;

// Current mode
enum RunningMode {
  RM_LISTEN_RENARD = 0,     // Listen to renard input
  RM_TEST_ALL_ON,           // All channels on
  RM_TEST_ALL_DIMMING,      // All channels dimming in unison
  RM_TEST_ON_SEQUENCE,      // Channels switched on in sequence
  RM_TEST_LARSEN,           // Larsen scanner
  RM_TEST_ZERO_CROSS,       // Zero crossing test
  RM_CONFIG_START_ADDRESS,  // Configure the channel start address
  RM_CONFIG_BAUD_RATE,      // Configure the baud rate
};

enum RunningMode runningState = RM_LISTEN_RENARD;
//enum RunningMode runningState = RM_TEST_ALL_DIMMING;

// Stores the PWM value for each channel's output
uint8_t pwm[CHANNELS];

// Used internally to index channels
uint8_t protocolChannelCount;

// Used for the Renard Protocol and escape characters
bool ignoreEscape = false;

// Used to store the byte read from renard
unsigned char b;

// Used to backwards count position in the AC half cycle (255-0, 0 is the begining of the cycle)
// Volatile has a performance penalty here, but it's likely needed
volatile uint8_t pwmSliceCounter = 0;

// Used to switch off lights when renard has been inactive for a while
volatile uint16_t inactivityTimer = 0;

// For sequencing channels in test mode
uint8_t testChannel = 0;

// For testing dimming in test mode
uint8_t testDimming = 0;

// Direction of the larsen scanner, +1 is forward, -1 is backwards
int8_t testLarsenDirection = 1;

// Indicates which pair in the zero cross detection tester is lit
bool testZeroCrossOutsidePair = true;

// Indicates the type of switch press.  The interrupt sets it, the main loop clears it when read
enum ConfigSwitchPressType 
{
  CSPT_NONE = 0,
  CSPT_PRESS,       //Momentary press
  CSPT_SHORT_HOLD,  //Hold for 5 seconds
  CSPT_MEDIUM_HOLD, //Hold for 10 seconds
  CSPT_LONG_HOLD,   //Hold for 15 seconds
};

volatile enum ConfigSwitchPressType configSwitchPressType = CSPT_NONE;

// The time when the switch was pressed down, used to time the duration of config switch presses
volatile unsigned long timeSinceConfigSwitchDepressed = 0;

// Used to provide a flash of all channels at short hold and long hold times
volatile uint8_t configSwitchPressHoldFlash = 0;

volatile uint8_t zeroCrossCounter = 0;

// Start and end of the channel, do not change
uint8_t internalStartChannel;
uint8_t internalEndChannel;


#define BAUD_RATE_EEPROM_ADDRESS 2

#define DEFAULT_BAUD_RATE_INDEX 8  //57600
uint8_t baudRateIndex;

//Available baud rates
long baudRates[] = {1200, 2400, 4800, 9600, 14400, 19200, 28800, 38400, 57600, 115200};
#define LENGTH_BAUD_RATES 10


// Setup various stuff on boot

void setup()
{
  // Read the starting channel
  startChannel = EEPROM.read(START_CHANNEL_EEPROM_ADDRESS);
  
  // If the starting channel has not been set or is garbage, set it to 1
  if (( startChannel == 255 ) || ((( startChannel - 1) % CHANNELS) != 0 ))
  {
    startChannel = 1;
    EEPROM.write(START_CHANNEL_EEPROM_ADDRESS, startChannel);
  }
  
  internalStartChannel = startChannel - 1;
  internalEndChannel   =  (internalStartChannel + 16 - 1);

  // Setup serial comms.  Note we use RX (D0), we don't use TX (D1)
  // although we have to leave TX disconnected, as the serial library will 
  // want to have it available
  baudRateIndex = EEPROM.read(BAUD_RATE_EEPROM_ADDRESS);
  if (( baudRateIndex == 255 ) || ( baudRateIndex >= LENGTH_BAUD_RATES ))
  {
    baudRateIndex = DEFAULT_BAUD_RATE_INDEX;
    EEPROM.write(BAUD_RATE_EEPROM_ADDRESS, baudRateIndex);
  }
  
  Serial.begin(baudRates[baudRateIndex]);
  

  // For all channels, set to lights out initially
  // We handle the hardware ports, enabling outputs at the same time
  uint8_t ddrBCD[3];  //Store input/output configuration ports
  ddrBCD[0] = DDRB;
  ddrBCD[1] = DDRC;
  ddrBCD[2] = DDRD;

  for ( uint8_t i = 0; i < CHANNELS; i ++ )
  {
    pwm[i] = 0;
    
    // Set pin to output
    ddrBCD[ssrPortIndex[i]] |= _BV(ssrBit[i]);    
  }
  
  // Write input / output pins
  DDRB = ddrBCD[0];
  DDRC = ddrBCD[1];
  DDRD = ddrBCD[2];

  // Setup the zero crossing pin as an input
  pinMode(ZERO_CROSSING_PIN, INPUT);
  
  // Setup an internal pullup resistor on the config switch
  pinMode(CONFIG_SWITCH_PIN, INPUT);
  digitalWrite(CONFIG_SWITCH_PIN, HIGH);
  
  // Stop interrupts temporarily so we can setup a new timer
  cli();
  
  // Setup power frequency PWM timer on Timer 1
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1 = 0;                //Initialize ocunter value to 0
  OCR1A = POWER_FREQUENCY;  //Setup OCR1 for the power frequency
  TCCR1B |= _BV(WGM12);     //Turn on CTC Mode
  TCCR1B |= _BV(CS10);      //No prescaler, Freq = 16Mhz
  TIMSK1 |= _BV(OCIE1A);    //Enable timer compare interrupt
  
  //Attach the INT0 interrupt for zero crossing (zero cross = HIGH)
  attachInterrupt(0, int0_ISR_resetZeroCrossing, RISING);
  
  // Attach the INT1 interrupt for the config switch changes
  attachInterrupt(1, int1_ISR_configSwitchPressed, CHANGE);

  // Enable interrupts again 
  sei();
}



//Process 1 byte of renard serial data using a finite state machine

void processRenardSerialByte(void)
{
      //Reset the inactivity timer as we just received data
    inactivityTimer = 0;

    if      ( b == 0x7D )  ;                              //Do nothing - This is used for periodic syncing, we drop it as it's done it's job
    else if ( b == 0x7E )  renardState = RS_WAIT_CMD_ADDRESS;   //We're starting a packet, process it
    else
    {
      //Finite state machine to handle the renard protocol
      switch(renardState)
      {
        case RS_WAIT_SYNC:
          break;

        case RS_WAIT_CMD_ADDRESS:
          if      ( b < 0x80 )  renardState = RS_WAIT_SYNC;
          else if ( b == 0x80 )
          {
            //b = b - 0x80;
  
            renardState = RS_WAIT_DATA;
            protocolChannelCount = 0;
          }
          else                  renardState = RS_WAIT_SYNC; 
          break;

        case RS_WAIT_ESCAPE:
          if ( b == 0x30 )  b = 0x7E;  
          if ( b == 0x31 )  {
            b = 0x7F; 
            ignoreEscape = true;
          }
          if ( b == 0x2F )  b = 0x7D;

        case RS_WAIT_DATA:
          //Handle escape character
          if ( ignoreEscape )  ignoreEscape = false;
          else if ( b == 0x7F )
          {
            renardState = RS_WAIT_ESCAPE;
            break;
          }

          if (( protocolChannelCount >= internalStartChannel) && ( protocolChannelCount <= internalEndChannel) )
            pwm[protocolChannelCount - internalStartChannel] = b;

          protocolChannelCount ++;

          if ( protocolChannelCount > internalEndChannel )  renardState = RS_WAIT_SYNC;
          break;

        default:
          break;
      }
    }
}



// Flashes all channels momentarily.  Used to indicate movement between test mode and config mode

void flashAllChannels(void)
{
  // All channels on
  for ( uint8_t i = 0; i < CHANNELS; i ++)
    pwm[i] = 255;
  
  delay(100);

  // All channels off
  for ( uint8_t i = 0; i < CHANNELS; i ++)
    pwm[i] = 0;
}



// Switch all channels off

void allChannelsOff(void)
{
  for ( uint8_t i = 0; i < CHANNELS; i ++ )  pwm[i] = 0;
}


//Displays a byte as binary on the LEDs

void byteToBinary(uint8_t b)
{
  for ( uint8_t i = 0; i < CHANNELS; i ++ )
  {
    if ( b & 128 )  pwm[i] = 255;
    else            pwm[i] = 0;
    
    b <<= 1;
  }
}



//Resets the arduino

void softwareReset()
{
  asm volatile ("  jmp 0");  
}



// Main loop

void loop()
{  
#ifdef CONFIG_SWITCH_DEBUG
  // Switch press debugging
  if ( configSwitchPressType == CSPT_PRESS )
  {
      if ( pwm[0] == 255 )  pwm[0] = 0;
      else                  pwm[0] = 255;
      configSwitchPressType = CSPT_NONE;
  }
  if ( configSwitchPressType == CSPT_SHORT_HOLD )
  {
      if ( pwm[1] == 255 )  pwm[1] = 0;
      else                  pwm[1] = 255;
      configSwitchPressType = CSPT_NONE;
  }
  if ( configSwitchPressType == CSPT_MEDIUM_HOLD )
  {
      if ( pwm[2] == 255 )  pwm[2] = 0;
      else                  pwm[2] = 255;
      configSwitchPressType = CSPT_NONE;
  }
  if ( configSwitchPressType == CSPT_LONG_HOLD )
  {
      if ( pwm[3] == 255 )  pwm[3] = 0;
      else                  pwm[3] = 255;
      configSwitchPressType = CSPT_NONE;
  }
  return;
#endif
   
   // Handle flashing at short hold and long hold intervals during a switch hold   
  if ( configSwitchPressHoldFlash >= 1 )
  {
      unsigned long diff = millis() - timeSinceConfigSwitchDepressed;

     if (( configSwitchPressHoldFlash == 1 ) && ( diff >= CONFIG_SWITCH_SHORT_HOLD_DURATION ))
     {
       flashAllChannels();
       configSwitchPressHoldFlash ++;
     }
     if (( configSwitchPressHoldFlash == 2 ) && ( diff >= CONFIG_SWITCH_MEDIUM_HOLD_DURATION ))
     {
       flashAllChannels();
       configSwitchPressHoldFlash ++;
     }
     if (( configSwitchPressHoldFlash == 3 ) && ( diff >= CONFIG_SWITCH_LONG_HOLD_DURATION ))
     {
       flashAllChannels();
       configSwitchPressHoldFlash ++;
     }

  }
  
  
  if ( runningState != RM_LISTEN_RENARD )  inactivityTimer = 0;
  
  switch ( runningState )
  {
    case RM_LISTEN_RENARD:
                            // Do we have incoming renard serial data?  If we do, lets process it
                            if ( Serial.available() )
                            {
                              // Read the next byte of serial data
                              b = Serial.read();
    
                              // Process the serial data
                              processRenardSerialByte();
                            }

                            if ( configSwitchPressType == CSPT_SHORT_HOLD )
                            {
                              configSwitchPressType = CSPT_NONE;
                              runningState = RM_TEST_ALL_ON;
                              allChannelsOff();
                            }
                            
                            if ( configSwitchPressType == CSPT_MEDIUM_HOLD )
                            {
                              configSwitchPressType = CSPT_NONE;
                              runningState = RM_CONFIG_START_ADDRESS;
                              allChannelsOff();
                            }

                            if ( configSwitchPressType == CSPT_LONG_HOLD )
                            {
                              configSwitchPressType = CSPT_NONE;
                              runningState = RM_CONFIG_BAUD_RATE;
                              allChannelsOff();
                            }
                            break; 

    case RM_TEST_ALL_ON:    // All channels on
                            for ( uint8_t i = 0; i < CHANNELS; i ++ )
                              pwm[i] = 255;

                            if ( configSwitchPressType == CSPT_PRESS )
                            {
                              configSwitchPressType = CSPT_NONE;
                              runningState = RM_TEST_ALL_DIMMING;
                              allChannelsOff();
                            }
                            else delay(1000);
                            break;

    case RM_TEST_ALL_DIMMING:
                            // All channels dimming in unison
                            for ( uint8_t i = 0; i < CHANNELS; i ++ )
                            {
                                if ( testDimming >= 128 )  pwm[i] = (255 - testDimming) << 1;
                                else                       pwm[i] = testDimming << 1;
                            }
                            testDimming ++;
                            
                            if ( configSwitchPressType == CSPT_PRESS )
                            {
                              configSwitchPressType = CSPT_NONE;
                              runningState = RM_TEST_ON_SEQUENCE;
                              allChannelsOff();
                            }
                            else delay(20);
                            break;

    case RM_TEST_ON_SEQUENCE:
                            // Channels switched on in sequence
                            pwm[testChannel] = 255;
                            if ( testChannel == 0 )   pwm[CHANNELS-1]     = 0;
                            else                      pwm[testChannel-1]  = 0;
                              
                            //Increment testChannel and wrap around
                            testChannel ++;                            
                            testChannel %= CHANNELS;
                            
                            if ( configSwitchPressType == CSPT_PRESS )
                            {
                              configSwitchPressType = CSPT_NONE;
                              runningState = RM_TEST_LARSEN;
                              allChannelsOff();
                            }
                            else  delay(20);
                            break;

    case RM_TEST_LARSEN:    
                            // Larsen scanner
                            pwm[testChannel] = 255;
                            
                            //Decay/Fade the values
                            for ( uint8_t i = 0; i < CHANNELS; i ++ )
                            {
                                if ( i == testChannel )  continue;
                                 pwm[i] >>= 2;
                            }
                                                           
                            //Increment testChannel and wrap around
                            testChannel += testLarsenDirection;
                            if ( testChannel >= CHANNELS ) 
                            {
                                testLarsenDirection = -1;  
                                testChannel += testLarsenDirection; 
                            }                         
                            
                            if ( testChannel == 0 )  testLarsenDirection = 1;
                            
                            if ( configSwitchPressType == CSPT_PRESS )
                            {
                              configSwitchPressType = CSPT_NONE;
                              runningState = RM_TEST_ZERO_CROSS;
                              testZeroCrossOutsidePair = true;
                              zeroCrossCounter = 0;
                              allChannelsOff();
                            }
                            else delay(75);
                            break;

    case RM_TEST_ZERO_CROSS:
                            // Zero crossing test
                            // Flip between channel 1/2 and 15/16
                            // If the flip happens every second, then zero cross detection is working,
                            // if it's static, then either it's not plugged into 110V or it's broken
                            if ( zeroCrossCounter >= 60 )
                            {
                              zeroCrossCounter = 0;
                              if ( testZeroCrossOutsidePair )  testZeroCrossOutsidePair = false;
                              else                             testZeroCrossOutsidePair = true;
                            }
                            
                            if ( testZeroCrossOutsidePair )
                            {
                              pwm[0] = pwm[15] = 255;
                              pwm[1] = pwm[14] = 0;
                            }
                            else
                            {
                              pwm[0] = pwm[15] = 0;
                              pwm[1] = pwm[14] = 255;
                            }
                            
                            if ( configSwitchPressType == CSPT_PRESS )
                            {
                              configSwitchPressType = CSPT_NONE;
                              runningState = RM_LISTEN_RENARD;
                              allChannelsOff();
                            }
                            break;
                            
    case RM_CONFIG_START_ADDRESS:
                            // Configure the channel start address
                            byteToBinary(startChannel);
                            if ( configSwitchPressType == CSPT_PRESS )
                            {
                              configSwitchPressType = CSPT_NONE;

                              // Increment address
                              if ((255 - startChannel ) < CHANNELS )  startChannel = 1;
                              else                                    startChannel += CHANNELS;
                            }
                            if ( configSwitchPressType == CSPT_SHORT_HOLD )
                            {
                              configSwitchPressType = CSPT_NONE;
                              runningState = RM_LISTEN_RENARD;
                              allChannelsOff();
                              
                              //Reset to reread the starting address
                              softwareReset();
                            }
                            if ( configSwitchPressType == CSPT_MEDIUM_HOLD )
                            {
                              configSwitchPressType = CSPT_NONE;

                              //Store the starting channel in eeprom
                              EEPROM.write(START_CHANNEL_EEPROM_ADDRESS, startChannel);

                              //Reset to reread the original starting address
                              softwareReset();
                            }
                            break;
                            
    case RM_CONFIG_BAUD_RATE:
                            // Configure the baud rate
                            byteToBinary(baudRateIndex + 1);
                            if ( configSwitchPressType == CSPT_PRESS )
                            {
                              configSwitchPressType = CSPT_NONE;

                              // Increment address
                              baudRateIndex ++;
                              if ( baudRateIndex >= LENGTH_BAUD_RATES )  baudRateIndex = 0;
                            }
                            if ( configSwitchPressType == CSPT_SHORT_HOLD )
                            {
                              configSwitchPressType = CSPT_NONE;
                              runningState = RM_LISTEN_RENARD;
                              allChannelsOff();

                              //Reset to reread the baud rate
                              softwareReset();
                            }
                            if ( configSwitchPressType == CSPT_MEDIUM_HOLD )
                            {
                              configSwitchPressType = CSPT_NONE;

                              //Store thebaud rate in eeprom
                              EEPROM.write(BAUD_RATE_EEPROM_ADDRESS, baudRateIndex);

                              //Reset to reread the baud rate
                              softwareReset();
                            }

                            break;

    default:     
                            break;
  }
  
  //If we're not listening to renard, we dump all incoming serial
  if (( runningState != RM_LISTEN_RENARD ) && ( Serial.available() ))  Serial.read();
}



//Config switch has been pressed, handle it

void int1_ISR_configSwitchPressed(void)
{  
  uint8_t firstRead = digitalRead(CONFIG_SWITCH_PIN);
  
  // Debounce the switch
  delay(DEBOUNCE_CONFIG_SWITCH_DELAY);
  
  if ( digitalRead(CONFIG_SWITCH_PIN) == LOW )
  {
    // If switch is now LOW, it's been pressed, so we must have been previously unpressed
    // Store the time of the start of our press
    if ( firstRead != LOW )  return;
    
    timeSinceConfigSwitchDepressed = millis();  
    
    configSwitchPressHoldFlash = 1;
  }
  else 
  {

    //If switch is now HIGH, it's been released, so we must have been previously pressed
    //We can now figure out which press we have
    if ( timeSinceConfigSwitchDepressed == 0 )  return;  //Safeguard against triggering on startup

    if ( firstRead != HIGH )  return;
    
    unsigned long diff = millis() - timeSinceConfigSwitchDepressed;
    
    if      ( diff >= CONFIG_SWITCH_LONG_HOLD_DURATION )     configSwitchPressType = CSPT_LONG_HOLD;
    else if ( diff >= CONFIG_SWITCH_MEDIUM_HOLD_DURATION )   configSwitchPressType = CSPT_MEDIUM_HOLD;
    else if ( diff >= CONFIG_SWITCH_SHORT_HOLD_DURATION )    configSwitchPressType = CSPT_SHORT_HOLD;
    else                                                     configSwitchPressType = CSPT_PRESS;
    
    configSwitchPressHoldFlash = 0;
  }
}



//We've crossed zero in the AC cycle, reset the pwmSliceCounter to align the timing with the start of the cycle

void int0_ISR_resetZeroCrossing(void)
{
  pwmSliceCounter = 0;
  zeroCrossCounter ++;
}



// Interrupt to implement Software Pulse Width Modulation for 16 channels and to set the output on
// the channels.  Timer 1 - Compare Match A Interrupt
//
// Note: This interrupt is fairly time sensitive and does a lot of work at relatively
// high frequency for an Arduino.  If you alter this, it's recommended that you test
// with an oscilloscope to make sure you're maintaining the cycle frequency and haven't
// impacted zero crossing by making code changes.

// Temporary containers for PORTB, PORTC, PORTD whilst we manipulate them
//#ifdef FRED
uint8_t portBCD[3];
uint8_t portBCD_last[3];

ISR(TIMER1_COMPA_vect)
{
  //Store the current port values so that we can manipulate them
  //on mass without having to do a port read/write each time  
  //We also store a backup (portBCD_last) so that we can identify changes later.
  portBCD[0] = PORTB;
  portBCD[1] = PORTC;
  portBCD[2] = PORTD;
  portBCD_last[0] = portBCD[0];
  portBCD_last[1] = portBCD[1];
  portBCD_last[2] = portBCD[2];
  
  // pwmSliceCounter = 0 is the start of the half wave AC cycle (i.e. zero crossing)
  // note, we count backwards from 0, so that we can match pwm[i] without requiring an
  // extra subtraction
  if ( pwmSliceCounter == 0 )
  {
    // Safety against broken serial comms leaving lights on
    // Switch off if the inactivity timer has timed out
    inactivityTimer ++;
    
    if ( inactivityTimer > INACTIVITY_TIMEOUT )
    {
      for ( uint8_t i = 0; i < CHANNELS; i ++ )
         pwm[i] = 0;
     
      inactivityTimer = 0;
    } 

    // Set channel trigger to true if pwm[i] = 255, i.e. we're starting at the beginning of the AC Cycle
    for ( uint8_t i = 0; i < CHANNELS; i ++ )
    {
      if ( pwm[i] == 255 )
      {
        // Set output ON
         portBCD[ssrPortIndex[i]] &= ~_BV(ssrBit[i]);
      }
      else
      {
         // set channel OFF
         portBCD[ssrPortIndex[i]]  |= _BV(ssrBit[i]);
      }
    }   
  }
  else
  {
    // Trigger a channel if pwm[i] and pwmSliceCounter match
    for ( uint8_t i = 0; i < CHANNELS; i ++ )
    {
      if ( pwmSliceCounter == pwm[i] )
      {
        // set channel ON
        portBCD[ssrPortIndex[i]] &= ~_BV(ssrBit[i]);
      }
    }
  }
    
  //Set the ports to the new values we've defined, but only
  //if we've changed them.  Although we could just set them anyway and
  //ignore the test, we don't because most of the times, channels don't change
  //and by doing this, we save some cycles.
  if ( portBCD[0] != portBCD_last[0] )  PORTB = portBCD[0];
  if ( portBCD[1] != portBCD_last[1] )  PORTC = portBCD[1];
  if ( portBCD[2] != portBCD_last[2] )  PORTD = portBCD[2];

  //If we've wrapped around, advance to the next, as we have N-1, because pwmSliceCounter = 255 = 0
  if ( pwmSliceCounter == 0 )  pwmSliceCounter = 254;
  else                         pwmSliceCounter --;
}
//#endif



#ifdef ALTERNATE_ISR

uint8_t ssrPins[CHANNELS] = {
  D4, D5, D6, D7,
  D8, D9, D10, D11,
  D12, D13, A0, A1,
  A2, A3, A4, A5
};

// Interrupt to handle Pulse Width Modulation for 16 channels and to set the output on
// the channels.  Timer 1 - Compare Match A Interrupt

ISR(TIMER1_COMPA_vect) {
  // Each half cycle (120Hz for 60HZ NorthAmerica), the start of the half cycle
  // is pwmSliceCounter = 255, and the end of the half cycle is pwmSliceCounter = 0
  //
  // If we're at the start of the cycle, reset the pulse to low if we're not full on
  if ( pwmSliceCounter == 0 ) 
  {
    inactivityTimer ++;
    
    // Safety against broken serial comms leaving lights on
    // Switch off if the inactivity timer has timed out
    if ( inactivityTimer > INACTIVITY_TIMEOUT )
    {
       for ( uint8_t i = 0; i < CHANNELS; i ++ )
         pwm[i] = 0;
       
       inactivityTimer = 0;
    }
    
    // Set each channel off at the start of the half cycle, unless
    // a channel is full on.
    // We use outputPins as a holding buffer so we don't waste cycles doing port writes twice
    for ( uint8_t i = 0; i < CHANNELS; i ++ )
    {
      if ( pwm[i] == 255 )  digitalWrite(ssrPins[i], LOW);      
      else                  digitalWrite(ssrPins[i], HIGH);     
    }
  }
  else
  {
    // Switch the channel on when we reach the appropriate part in the cycle.
    for ( uint8_t i = 0; i < CHANNELS; i ++ )
    {
      if ( pwmSliceCounter == pwm[i] )
      {
          digitalWrite(ssrPins[i], LOW);
      }
    }
  }
  
  if ( pwmSliceCounter == 0 )  pwmSliceCounter = 254;
  else                         pwmSliceCounter --;
}

#endif

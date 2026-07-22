/*
 *   Program:      Euclidean Rhythm Generator Kosmo/Eurorack
 *   Author:       L J Brackney,
 *   Organization: Suncoast Polytechnical High School
 *   Version:      1.0
 *   Date:         11/21/2021
 *
 *   Adapted by Jos Bouten, Jan ... Apr 2022
 *   Added support for shift button, clock out, 4 function inputs
 *   - Added variable pattern length
 *   - To change the tempo the shift/tap tempo button needs to be pressed while the rotary encoder is rotated.
 *   - Changed all channel dependent variables into arrays.
 *   - Used capitals for constants, camel case for variables and function calls
 *   - Setting the speed depends on pressing the tap tempo button.
 *   - While setting the speed, the pattern leds show a white background pattern.
 *   - Added storing, recalling and clearing of patches in EEPROM.
 *   - The channel chosen last will be recalled on startup.
 * 
 * 2022-04-27: Whenever you change the pattern length, or change the number of Euclidean Rhythm pulses,
 *             a new euclidean pattern is generated and this pattern is started from its starting position for the active pattern.
 * 2022-04-29: The mode from which a jump into program mode is made is restored when leaving program mode.
 * 2022-05-05: Code adapted to function inputs using negative logic.
 *
 *  Compile, upload and run the code one time with the INITIALIZE flag set.
 *  This will result in the nano writing 16 patches to its EEPROM.
 *  One patch will contain a pattern, the others are empty patches.
 *  Next comment out the line "#define INITIALIZE 1" and compile, upload and run the code.
 *  This will program the Euclidean sequencer code into the nano which will be run each time you start up the nano.
 *  The nano will read the patches from memory each time it starts and chooses the patch you saved last.
 *
 *  Upon startup the sequencer is in 'Pattern Length / Tempo Mode'.
 *  When pressing the rotary encoder you can switch to the second mode, 'Num Pulse Mode'.
 *  Press it again and the mode is 'Rotate Mode'.
 *  Press again and the mode returns to 'Pattern Length / Tempo Mode'.
 *
 *  In 'Pattern Mode' rotating the encoder will increase or decrease the pattern length of the chosen trigger channel.
 *  Each time you change the length of the pattern a new fitting Euclidean pattern is generated depending on the number of pulses chosen (see below) for this pattern.
 *  Rotating the dial with the tap tempo / shift button pressed at the same time, will allow for changing tempo of the sequencer's internal clock.
 *  In 'Num Pulse Mode' the encoder can be used to set the number of trigger pulses for the chosen trigger channel.
 *  The pulses will follow a Euclidean pattern.
 *  In 'Rotate Mode' it is possible to shift the trigger pulse pattern accross the total length of the pattern.
 *
 *  Pressing the shift button and the rotary switch at the same time will select the 'Program Mode'.
 *  Using the rotary encoder's knob 1 of 16 memory cells can be selected.
 *  Pressing the trigger channel D button will store the current pattern of the selected trigger channel to the corresponding memory cell.
 *  Pressing the trigger channel A button will recall the patch pattern stored in the EEPROM memory cell and store it in
 *    the sequencer's active memory at the patch number selected with the rotary dial.
 *  Pressing the trigger channel B button will clear the patch in the eeprom the rotary encoder is set to.
 *  The pattern already in the working memory however will not be changed. Don't forget to save it is you like it.
 *
 *  The LEDs will light up red for the locations containing a patch and green for the locations that are cleared.
 *  When in 'Program Mode' pressing the rotary switch will switch to 'Pattern Length / Tempo Mode'.
 *
 * 2022-02-27: some changes to hardware and software
 *  - deactivated pattern length change menu because this is hardly ever used and is a nuisance to have to click through.
 *  - added tempo change to rotate and pulse number menu.
 *  - adapted ext clock in and func in to positive logic after hardware change.
 *  - changed the range of the pulse width knob so that the output pulses range from about 10% to 99% of the clock cyle time.
 *  - changed the duty cycle of the clock out to be fixed at 50%.
 *
 * 2026-07-19: software changes -- dlakwi Ensign / donald johnson
 *  - reactivated pattern length change
 *  - added encoder 'accelaration' to internal tempo change for setting tempo quicker
 *  - changed length colour to cyan since channel 4 colour is yellow
 *  - added NeoPixel brightness set
 *  - removed code for Euclid-O-Matic hardware
 */

#include <Adafruit_NeoPixel.h>    // Include Adafruit_NeoPixel library
#include <Encoder.h>              // Include rotary encoder library
#include <EEPROM.h>

// Run program once using INITIALIZE to set some initial values in the EEPROM
//#define INITIALIZE 1

// pins

#define PIXEL_PIN             2  // Attach NeoPixel ring to digital pin 2
#define PROG_PIN              5  // Attach rotary encoder switch to digital pin 5
#define CLOCK_SOURCE_PIN      6  // Attach external clock select switch to digital pin 6
#define TRIG_A_PIN           10  //   swap pins 7 and 10 than to resolder things. :)
#define TRIG_B_PIN            9  //   assembling the modules, and it was easier to
#define TRIG_C_PIN            8  //   digital pins 7-10.  I mixed up two wires when
#define TRIG_D_PIN            7  // Attach the four drum/voice trigger output pins to 
#define BUTTON_PIN           A0  // The trigger select buttons are on a voltage divider attached to A0
#define PWM_POT_PIN          A1  // The pulse width potentiometer is attached to analog pin A1
#define EXT_CLOCK_IN_PIN     A2  // Ext Clock in
#define ENC_A_PIN             3
#define ENC_B_PIN             4

#define SHIFT_PIN            11  // Shift Function button on D11 
#define ON  LOW
#define OFF HIGH

// defines

// Show some debug info on the serial port.
//#define DEBUG 1

#define NUM_CHANNELS           4

// array indexes step[], pulses[], patternLength[], channelPattern[]  --  not 0 1 2 3
#define CH_A_1                 3  // Pattern/trigger A/1 == red
#define CH_B_2                 2  // Pattern/trigger B/2 == green
#define CH_C_3                 1  // Pattern/trigger C/3 == blue
#define CH_D_4                 0  // Pattern/trigger D/4 == yellow

char CHANNEL_NAMES[NUM_CHANNELS] = { 'D', 'C', 'B', 'A' };

// Modes you can choose by pressing the rotary encoder's button.
#define PATTERN_LENGTH_MODE    1  // also used as "pulse length mode"
#define NUM_PULSE_MODE         2
#define ROTATE_MODE            3
#define PROGRAM_MODE           4  // Allows to store patterns in EEPROM or to recall them.
                                  // This mode can only be reached by pressing the shift button AND the rotary encoder button.
#define MAX_MODE               3  // Note, we do not count PROGRAM_MODE here.

// Activate this if you allow pattern length change
#define ALLOW_PATTERN_LENGTH_CHANGE 1

#define MIN_DELAY_TIME        10  // Limit the fastest loop time to 10 mS per step -- Change if you want faster tempos.
#define MAX_DELAY_TIME      2000  // Limit the slowest loop time to 1 sec per step -- Change if you want slower tempos.
#define INIT_DELAY_TIME      500

#define MAX_ADC_VALUE       1000  // This number was determined empirically.
#define HALF_DAC_RANGE       500

#define BRIGHTNESS           192
#define NUM_NEOP_LEDS         16  // The number of LEDs in the neopixel wheel.
#define NR_OF_MEMORY_CELLS     NUM_NEOP_LEDS
// Use the switches below to make the brightness of the neopixel oscillate.
  #define LED_FX               1  // Oscillate LEDs in all modes except PROGRAM_MODE.
//#define LED_FX_BLACK         1  // When set the intensity will fall back to 0, otherwise to 1.
  #define PATCH_MEMORY_LED_FX  1  // Also oscillate LED intensity in PROGRAM_MODE

#define EEPROM_BASE_ADDRESS    0

#define CHECK_BIT(var, pos) ((var) & (1 << (pos)))

#ifdef DEBUG
char msg[255];
#endif

// ----------------

static int        step          [NUM_CHANNELS];  // The active step for each trigger channel.

struct            Patch
{
  int             pulses        [NUM_CHANNELS];  // The number of pulses to be generated for each of the triggers.
  int             patternLength [NUM_CHANNELS];  // The pattern length for each trigger output.
  unsigned int    channelPattern[NUM_CHANNELS];  // The pattern of Euclidean distributed pulses,
};

Patch             emptyPatch;
Patch             currentPatch;       // The patch the sequencer is playing.

Patch             patches[NR_OF_MEMORY_CELLS];
int               chosenPatchNumber      = 0;
int               candidatePatchNumber   = 0;
int               selectedTriggerChannel = -1;

int               delayTime              = INIT_DELAY_TIME;  // The time between steps persists in each loop.
unsigned int      memoryCellsInUse       = 0;


Encoder           myEnc( ENC_A_PIN, ENC_B_PIN );  // Attach rotary encoder - swap if encoder is 'backwards'

Adafruit_NeoPixel pixels( NUM_NEOP_LEDS, PIXEL_PIN, NEO_GRB + NEO_KHZ800 );  // Set up the NeoPixel ring

// ----------------

void setup( void )
{
#ifdef DEBUG
  // Set up the serial console for debugging messages
  Serial.begin( 115200 );
#endif

#ifdef INITIALIZE
  initializePatchInEeprom();
  pinMode( 13, OUTPUT );  // Nano LED
  digitalWrite( 13, 0 );
#ifdef DEBUG
  Serial.println( "Successfuly written 16 default patches to EEPROM." );
#endif

#else
  pinMode( PROG_PIN,          INPUT_PULLUP );  // Assign the encoder switch which closes to ground and uses an internal pullup resistor.
  pinMode( CLOCK_SOURCE_PIN,  INPUT );         // Assign the external clock selector switch.
  pinMode( TRIG_D_PIN,        OUTPUT );        // Assign trigger 1 out.
  pinMode( TRIG_C_PIN,        OUTPUT );        // Assign trigger 2 out.
  pinMode( TRIG_B_PIN,        OUTPUT );        // Assign trigger 3 out.
  pinMode( TRIG_A_PIN,        OUTPUT );        // Assign trigger 4 out.
  pinMode( EXT_CLOCK_IN_PIN,  INPUT );         // Assign external clock input.
  pinMode( SHIFT_PIN,         INPUT );         // Assign "tap"/shift button.

  pixels.begin();                              // Start the NeoPixel
  pixels.setBrightness( BRIGHTNESS );
  pixels.clear();

  testLedsAndPixels();                         // Show that leds and pixels are working.

  // Read all 16 stored (empty or not) patches from EEPROM.
  chosenPatchNumber    = readPatchesFromEEPROM( patches, memoryCellsInUse, delayTime, selectedTriggerChannel );
  candidatePatchNumber = chosenPatchNumber;

  // Load the current patch with the stored information.
  copyPatch( patches[chosenPatchNumber], currentPatch );

#ifdef DEBUG
  sprintf( msg, "Reading patch info from EEPROM." );        Serial.println( msg );
  sprintf( msg, "Restoring patch: %d", chosenPatchNumber ); Serial.println( msg );
  displayPatch( currentPatch );
  sprintf( msg, "Setting speed to: %d", delayTime );                              Serial.println( msg );
  sprintf( msg, "Selecting channel: %c", CHANNEL_NAMES[selectedTriggerChannel] ); Serial.println( msg );
#endif

#endif

  // Initialize step count for each trigger channel.
  for ( int triggerChannel = 0; triggerChannel < NUM_CHANNELS; triggerChannel++ )
  {
    step[triggerChannel] = 0;
  }

  createEmptyPatch( emptyPatch );
}

#ifdef INITIALIZE

void loop( void )
{
  static byte led = 0;
#ifdef DEBUG
  Serial.println( "Now recompile and upload with #define INITIALIZE commented out" );
  Serial.println( "to run Euclid-O-Matic (a Euclidean sequencer / rhythm generator." );
#endif
  led = !led;
  digitalWrite( 13, led );  // blink LED
  delay( 1000 );
}

#else

static long oldPosition;  // Previous encoder reading.
static long newPosition;  // Current  encoder reading.

void loop( void )
{
#ifdef ALLOW_PATTERN_LENGTH_CHANGE
  static int           mode                   = PATTERN_LENGTH_MODE;
#else
  static int           mode                   = NUM_PULSE_MODE;
#endif
  static int           prevMode               = 0;
  static long          mode1Pos               = 0;                                // The last encoder position in mode 1 is stored for a future return to that mode.
  static int           prevExtClk             = 0;                                // Store the previous External Clock Pulse value for transition checking.
  static unsigned long thisTime;                                                  // Prepare to query the Nano's clock.
  static unsigned long prevTime               = 0;                                // Keep track of how much time elapses between loops.
  static bool          prevProg;                                                  // Keep track of the previous encoder switch state.
  static bool          prog;                                                      // Prepare to read the current encoder switch state.

  bool                 triggered              = false;                            // Assume that no new step is going to occur this loop.
                       newPosition            = myEnc.read();                     // Read the current number of encoder counts.
  int                  triggerWidthPotValue   = analogRead( PWM_POT_PIN );        // Read the trigger pulse width potentiometer value.
  bool                 extClk                 = digitalRead( CLOCK_SOURCE_PIN );  //
  int                  extClkV                = analogRead( EXT_CLOCK_IN_PIN );   // Read the external clock input.
  int                  shiftButton            = digitalRead( SHIFT_PIN );         // Read the shift button (will be HIGH when pressed);

  int                  programButtonName      = -1;
  int                  programButtonValue;
  static int           prevProgramButtonValue = 0;
  boolean              change                 = false;
  int                  R = 0, G = 0, B = 0;

  if ( mode == PROGRAM_MODE )
  {
    programButtonValue = checkProgramButtons( programButtonName );         // Find out whether Trig A, B or D was pressed.
    if ( programButtonValue == 0 )                                         // No button was pressed.
    {
      prevProgramButtonValue = 0;
    }
  }
  else
  {
    change = checkButtons( selectedTriggerChannel, shiftButton );          // Query the trigger buttons and update the active trigger if needed.
    if ( change )                                                          // Remember the channel chosen (this will be recalled when powering up).
    {
      writeTriggerChannelToEEPROM( selectedTriggerChannel );
    }
  }

  prevProg = prog;                                                         // Store the last encoder switch position.
  prog     = digitalRead( PROG_PIN );                                      // Get the current encoder switch position.

  thisTime = millis();                                                     // Get the current Nano clock value in msec.

  if ( ( prog == true ) && ( prevProg == false ) )                         // If the encoder switch was just pressed, we need to switch modes.
  {
    if ( mode == 1 ) mode1Pos = newPosition;                               // Store the current encoder position for a future return to that mode.

    if ( shiftButton )                                                     // If the shift button is pressed as well, then we start the program mode.
    {
      prevMode = mode;
      mode     = PROGRAM_MODE;
    }
    else
    {
      if ( mode == PROGRAM_MODE )
      {
        mode = prevMode;                                                   // Return to mode from which we jumped into program mode
      }
      else
      {
        mode++;                                                            // Cycle to the next mode.
        if ( mode > MAX_MODE )
#ifdef ALLOW_PATTERN_LENGTH_CHANGE
          mode = PATTERN_LENGTH_MODE;
#else
          mode = NUM_PULSE_MODE;
#endif
        if ( mode == PATTERN_LENGTH_MODE )                                 // Restore the encoder position when last in the current mode.
        {
          myEnc.write( mode1Pos );
          newPosition = mode1Pos;
        }
      }
    }

    oldPosition = newPosition;
  }

  ClearNeoPixelPattern();                                                  // Clear the NeoPixel ring in preparation for any changes to the pattern.

  switch ( mode )
  {

#ifdef ALLOW_PATTERN_LENGTH_CHANGE
    // Mode 1 - Change the length of the active pattern.
    case PATTERN_LENGTH_MODE:
      if ( shiftButton && !extClk )                                        // If the shift button is pressed at the same, change the tempo
      {
        setTempo();
      }
      else                                                                 // Increase/decrease the pattern length for the active pattern.
      {
        if ( newPosition > ( oldPosition + 3 ) )                           // CW?
        {
          currentPatch.patternLength[selectedTriggerChannel]++;            // Increase the pattern length by 1 for the active pattern.
          if ( currentPatch.patternLength[selectedTriggerChannel] > NUM_NEOP_LEDS )
          {
            currentPatch.patternLength[selectedTriggerChannel] = NUM_NEOP_LEDS;
          }
          oldPosition = newPosition;
          // Compute a Euclidean Rhythm for the active pattern.
          currentPatch.channelPattern[selectedTriggerChannel] = euclid( currentPatch, selectedTriggerChannel );
          // Restart all patterns
          resetSteps( step );
        }
        else if ( newPosition < ( oldPosition - 3 ) )                      // CCW?
        {
          currentPatch.patternLength[selectedTriggerChannel]--;            // Decrease the pattern length by 1 for the active pattern.
          if ( currentPatch.patternLength[selectedTriggerChannel] < 0 )
          {
            currentPatch.patternLength[selectedTriggerChannel] = 0;
          }
          oldPosition = newPosition;
          // Compute a Euclidean Rhythm for the active pattern.
          currentPatch.channelPattern[selectedTriggerChannel] = euclid( currentPatch, selectedTriggerChannel );
          // Restart all patterns
          resetSteps( step );
        }
      }
      break;
#endif

    // Mode 2 - Change the number of pulses of the active pattern.
    case NUM_PULSE_MODE:
      if ( shiftButton && !extClk )                                        // If the shift button is pressed at the same, change the tempo
      {
        setTempo();
      }
      else
      {
        if ( newPosition > ( oldPosition + 3 ) )                           // CW?
        {
          currentPatch.pulses[selectedTriggerChannel]++;                   // Increase the number of pulses in the active pattern.
          if ( currentPatch.pulses[selectedTriggerChannel] > currentPatch.patternLength[selectedTriggerChannel] )
          {
            currentPatch.pulses[selectedTriggerChannel] = currentPatch.patternLength[selectedTriggerChannel];  // Limit the largest number of pulses to NumSteps.
          }
          oldPosition = newPosition;
          // Compute a Euclidean Rhythm for the active pattern.
          currentPatch.channelPattern[selectedTriggerChannel] = euclid( currentPatch, selectedTriggerChannel );
          // Restart all patterns
          resetSteps( step );
        }
        else if ( newPosition < ( oldPosition - 3 ) )                      // CCW?
        {
          currentPatch.pulses[selectedTriggerChannel]--;                   // Decrease the number of pulses in the active pattern.
          if ( currentPatch.pulses[selectedTriggerChannel] < 0 )
          {
            currentPatch.pulses[selectedTriggerChannel] = 0;               // Limit the smallest number of pulses to zero.
          }
          oldPosition = newPosition;
          // Compute a Euclidean Rhythm for the active pattern.
          currentPatch.channelPattern[selectedTriggerChannel] = euclid( currentPatch, selectedTriggerChannel );
          // Restart all patterns
          resetSteps( step );
        }
      }
      break;

    // Mode 3 - Rotate the active pattern.
    case ROTATE_MODE:
      if ( shiftButton && !extClk )                                        // If the shift button is pressed at the same, change the tempo
      {
        setTempo();
      }
      else
      {
        if ( newPosition > ( oldPosition + 3 ) )                           // CW?
        {
          oldPosition = newPosition;
          rotateLeft( currentPatch, selectedTriggerChannel );              // Rotate the active pattern clockwise
        }
        else if ( newPosition < ( oldPosition - 3 ) )                      // CCW?
        {
          oldPosition = newPosition;
          rotateRight( currentPatch, selectedTriggerChannel );             // Rotate the active pattern counter-clockwise
        }
      }
      break;

    // Mode 4 - EEPROM operations: copy, clear, write the current patch
    case PROGRAM_MODE:                                                     
      if ( newPosition > ( oldPosition + 3 ) )                             // CW?
      {
        oldPosition = newPosition;
        candidatePatchNumber += 1;
        if ( candidatePatchNumber > NR_OF_MEMORY_CELLS - 1 )
        {
          candidatePatchNumber = 0;
        }
      }
      else if ( newPosition < ( oldPosition - 3 ) )                        // CCW?
      {
        oldPosition = newPosition;
        candidatePatchNumber -= 1;
        if ( candidatePatchNumber < 0 )
        {
          candidatePatchNumber = NR_OF_MEMORY_CELLS - 1;
        }
      }

      switch ( programButtonName )
      {
        case CH_A_1:  // Copy the stored pattern from EEPROM and put it in the current pattern.
          if ( prevProgramButtonValue == 0 )
          {
            prevProgramButtonValue = 1;
            chosenPatchNumber      = candidatePatchNumber;
            copyPatch( patches[chosenPatchNumber], currentPatch );
            // The pattern lengths may have changed, so we want to sync the chosen patterns and restart their step counters.
            // Therefore, we initialize the step count for each trigger channel.
            for ( int triggerChannel = 0; triggerChannel < NUM_CHANNELS; triggerChannel++ )
            {
              step[triggerChannel] = 0;
            }
            writePatchNumberToEEPROM( chosenPatchNumber );                 // Write the current patch number to EEPROM.
#ifdef DEBUG
            displayPatch( chosenPatchNumber );
#endif
          }
          break;

        case CH_B_2:  // Wipe the memory cell chosen clean. Note, this does not affect the current patch.
          if ( prevProgramButtonValue == 0 )
          {
            prevProgramButtonValue = 1;
            memoryCellsInUse       = memoryCellsInUse & ~( 1 << candidatePatchNumber );
            copyPatch( emptyPatch, patches[candidatePatchNumber] );
            writePatchToEEPROM( emptyPatch, memoryCellsInUse, candidatePatchNumber, delayTime, selectedTriggerChannel );
          }
          break;

        case CH_C_3:  // no op
          break;

        case CH_D_4:  // Write the current patch to the EEPROM at the chosen patch location.
          if ( prevProgramButtonValue == 0 )
          {
            prevProgramButtonValue = 1;
            memoryCellsInUse       = memoryCellsInUse | ( 1 << candidatePatchNumber );
            // Write chosen patch to candidate location in EEPROM.
            writePatchToEEPROM( currentPatch, memoryCellsInUse, candidatePatchNumber, delayTime, selectedTriggerChannel );
            // Either read the memory cell back to sync patches (slow because of EEPROM) OR copy the patch directly (fast because of in memory copy).
            copyPatch( currentPatch, patches[candidatePatchNumber] );
          }
          break;
      }
      break;
  } // end swith(mode)

  if ( mode == PROGRAM_MODE )
  {
    showPatchMemory( candidatePatchNumber, memoryCellsInUse );             // Show selected memory location.
  }
  else
  {
    showBitPattern( selectedTriggerChannel,                                // Display the active pattern.
                    currentPatch.channelPattern[selectedTriggerChannel],
                    currentPatch.patternLength [selectedTriggerChannel],
                    mode );
  }

  // with an external clock, the delayTime is set based on consecutive clock pulses -- the internally set value is ignored

  if ( extClk )                                                             // If we're using an external clock pulse.
  {
    if ( ( extClkV > HALF_DAC_RANGE ) && ( prevExtClk < HALF_DAC_RANGE ) )  // Check to see if the pulse just went high.
    {
      delayTime = thisTime - prevTime;                                      // If so, then capture the msec delayTime associated with the external pulse.
      triggered = true;                                                     // Note that we've just triggered a new step in the sequence.
    }
    prevExtClk = extClkV;                                                   // The current external clock input will be the previous input for the next loop.
  }
  else                                                                      // We're using the internal clock to generate pulses.
  {
    if ( ( thisTime - prevTime ) > delayTime )                              // Check to see if we've waited long enough for the next step in the sequence.
    {
      triggered = true;                                                     // Note that we've triggered a new step in the sequence.
    }
  }

  // Calculate the pulse width based on the potentiometer setting and clock.
  unsigned int trigWidth = map( triggerWidthPotValue, 0, 1023, delayTime * 99 / 100, delayTime / 10 );
  if ( ( thisTime - prevTime ) > trigWidth )  // Turn off the trigger outputs if the pulse width has elapsed.
  {
    digitalWrite( TRIG_A_PIN, LOW );
    digitalWrite( TRIG_B_PIN, LOW );
    digitalWrite( TRIG_C_PIN, LOW );
    digitalWrite( TRIG_D_PIN, LOW );
  }

  // For each trigger channel we keep track of at which step we are.
  if ( triggered )                          // If we're triggering a new step in the sequence.
  {
    prevTime = thisTime;                    // The current time will be the previous time in the next loop.
    for ( int triggerChannel = 0; triggerChannel < NUM_CHANNELS; triggerChannel++ )
    {
      step[triggerChannel]++;               // Increment the step counter.
      if ( step[triggerChannel] > currentPatch.patternLength[triggerChannel] - 1 )
      {
        step[triggerChannel] = 0;           // Reset the step counter if we've gone through all the beats for this trigger channel.
      }
    }

    // Show a cursor by lighting up each pixel and using color to show what mode we are in.
    getCursorColor( mode, R, G, B );

    // Light up the current step pixel with a color to indicate the current mode.
    pixels.setPixelColor( step[selectedTriggerChannel], pixels.Color( 4 * R, 4 * G, 4 * B ) );

    // Show/refresh all pixels.
    pixels.show();

    // Turn on trigger output + LED A..D if the current step is in the Euclidean Rhythm.
    if ( currentPatch.channelPattern[CH_A_1] & ( 1 << step[CH_A_1] ) ) digitalWrite( TRIG_A_PIN, HIGH );
    if ( currentPatch.channelPattern[CH_B_2] & ( 1 << step[CH_B_2] ) ) digitalWrite( TRIG_B_PIN, HIGH );
    if ( currentPatch.channelPattern[CH_C_3] & ( 1 << step[CH_C_3] ) ) digitalWrite( TRIG_C_PIN, HIGH );
    if ( currentPatch.channelPattern[CH_D_4] & ( 1 << step[CH_D_4] ) ) digitalWrite( TRIG_D_PIN, HIGH );
  }
}

void setTempo( void )
{
  if ( newPosition > ( oldPosition + 3 ) )                           // CW?
  {
    delayTime += accelStep( -1 );
    if ( delayTime < MIN_DELAY_TIME ) delayTime = MIN_DELAY_TIME;    // Limit the fastest loop time
    oldPosition = newPosition;
  }
  else if ( newPosition < ( oldPosition - 3 ) )                      // CCW?
  {
    delayTime += accelStep( +1 );
    if ( delayTime > MAX_DELAY_TIME ) delayTime = MAX_DELAY_TIME;    // Limit the slowest loop time
    oldPosition = newPosition;
  }
}

// << ClockForge == https://github.com/VoltageFoundryMod/ForgeSeries-CLK
// 
// Calculate the speed of the encoder rotation.
// Resets to 1 when direction reverses so the first detent after a turn-around is always a single step.
// Avoids the "skips 2" artifact on BPM decrease.

unsigned long lastEncoderTime = 0;
int           lastEncoderDir  = 0;  // +1 or -1

int accelStep( int dir )
{
  int           step               = 1;
  unsigned long currentEncoderTime = millis();
  unsigned long timeDiff           = currentEncoderTime - lastEncoderTime;
  lastEncoderTime                  = currentEncoderTime;

  if ( ( lastEncoderDir != 0 ) && ( dir != lastEncoderDir ) )
  {
    // Direction changed - set step to 1 for first step of new direction
    step           = 1;
    lastEncoderDir = dir;
    return dir;
  }
  lastEncoderDir = dir;

  if      ( timeDiff <  30 ) { step = 8; }  // Very fast spin
  else if ( timeDiff <  60 ) { step = 4; }  // Fast spin
  else if ( timeDiff < 120 ) { step = 2; }  // Moderate spin
  else                       { step = 1; }  // Normal

  return dir * step;
}

#endif  // normal loop()

// ----------------

void resetSteps( int step[] )
{
  for ( int triggerChannel = 0; triggerChannel < NUM_CHANNELS; triggerChannel++ )
  {
    step[triggerChannel] = 0;
  }
}

// The Euclid function returns an unsigned integer whose bits reflect a Euclidean rhythm of "pulses" spread across "patternLength[]."
// There are several ways to implement this, but here an adapted version of algorithm described at:
//   https://www.computermusicdesign.com/simplest-euclidean-rhythm-algorithm-explained/
// is used.

unsigned int euclid( Patch thisPatch, int channelNumber )
{
  int          bucket = 0;
  unsigned int number = 0;                                           // Number is the number of pulses that have been allocated so far.

  for ( int i = 0; i < thisPatch.patternLength[channelNumber]; i++ ) // Cycle through all of the possible steps in the sequence.
  {
    bucket = bucket + thisPatch.pulses[channelNumber];               // Fill a "bucket" with the number of pulses to be allocated.
    if ( bucket >= thisPatch.patternLength[channelNumber] )          // If the bucket exceeds the maximum number of steps then "empty
    {
      bucket = bucket - thisPatch.patternLength[channelNumber];      // the bucket" by setting the i-th bit in the sequence and
      number |= 1 << i;                                              // Refill the now empty bucket with the remainder.
    }
  }

  return number; // Return the sequence encoded as the bits set in Number.
}

// The rotateLeft function shifts the bits in an unsigned integer by one place.
// Any bit that falls off to the left (of the most significant bit) is shifted around to become the new least significant bit.
// For our purposes, this means that we can rotate the Euclidean Rhythm clockwise on the ring.

void rotateLeft( Patch& thisPatch, int channelNumber )
{
  int DROPPED_MSB;                                                                                                  // Need to keep track of any bits dropped off the left
  DROPPED_MSB = ( thisPatch.channelPattern[channelNumber] >> ( thisPatch.patternLength[channelNumber] - 1 ) ) & 1;  // aka the old most significant bit
  // Shift all the bits in Pattern to the left by 1 and add back in the new least significant bit if needed.
  thisPatch.channelPattern[channelNumber] = ( thisPatch.channelPattern[channelNumber] << 1 ) | DROPPED_MSB;
}

// The rotateRight function shifts the bits in an unsigned integer by one place.
// Any bit that falls off to the right (of the least significant bit) is shifted around to become the new most significant bit.
// For our purposes, this means that we can rotate the Euclidean Rhythm counter-clockwise on the ring.

void rotateRight( Patch& thisPatch, int channelNumber )
{
  int DROPPED_LSB;                                            // Need to keep track of any bits dropped off the right
  DROPPED_LSB = thisPatch.channelPattern[channelNumber] & 1;  // aka the old least signficant bit.
  thisPatch.channelPattern[channelNumber] = ( thisPatch.channelPattern[channelNumber] >> 1 ) & ( ~( 1 << ( thisPatch.patternLength[channelNumber] - 1 ) ) );  // Shift all the bits in Pattern to the right by 1.
  thisPatch.channelPattern[channelNumber] =   thisPatch.channelPattern[channelNumber] | ( DROPPED_LSB << ( thisPatch.patternLength[channelNumber] - 1 ) );    // Tack the old LSB onto the new MSB if needed
}

// checkButtons and checkProgramButtons decode the voltage values from the button voltage divider circuit.
// 
// The values here assume that 1k, 2.2k, 4.7k and 10k resistors are switched in line with a 5V source and
//   dropped across a 10k resistor before going into the analog input channel.
// The ranges should be big enough to accommodate resistor variation, but if a button doesn't switch to the trigger as expected,
//   you should Serial.println( Buttons ) and adjust the ranges as needed for your circuit.

int checkProgramButtons( int& selectedProgramButtonName )
{
  // Note: the selectedProgramButtonName will only change if a button is pressed.
  // Read the voltage divider output.
  int buttonValue = analogRead( BUTTON_PIN );

  //  Determine which button was pressed.

  if ( ( buttonValue > 0.40 * MAX_ADC_VALUE ) && ( buttonValue <= 0.60 * MAX_ADC_VALUE ) ) { selectedProgramButtonName = CH_D_4; return 1; }  // Button D is pressed.
  if ( ( buttonValue > 0.60 * MAX_ADC_VALUE ) && ( buttonValue <= 0.75 * MAX_ADC_VALUE ) ) { selectedProgramButtonName = CH_C_3; return 1; }  // Button C is pressed.
  if ( ( buttonValue > 0.75 * MAX_ADC_VALUE ) && ( buttonValue <= 0.85 * MAX_ADC_VALUE ) ) { selectedProgramButtonName = CH_B_2; return 1; }  // Button B is pressed.
  if (   buttonValue > 0.85 * MAX_ADC_VALUE )                                              { selectedProgramButtonName = CH_A_1; return 1; }  // Button A is pressed.

  selectedProgramButtonName = -1;
  return 0;
}

// If no button was pressed, return the input parameter unchanged.

bool checkButtons( int& selectedChannel, int shiftButton )
{
  // Note: the selectedChannel will only change if a button is pressed.
  static int  previousSelectedChannel = -1;
  static bool change                  = false;

  int buttonValue = analogRead( BUTTON_PIN );

  // Read the voltage divider output and only change the selectedChannel if shiftButton is not pressed.
  if ( shiftButton == 0 )
  {
    if ( ( buttonValue > 0.40 * MAX_ADC_VALUE ) && ( buttonValue <= 0.60 * MAX_ADC_VALUE ) ) selectedChannel = CH_D_4;  // pattern/trigger D
    if ( ( buttonValue > 0.60 * MAX_ADC_VALUE ) && ( buttonValue <= 0.75 * MAX_ADC_VALUE ) ) selectedChannel = CH_C_3;  // pattern/trigger C
    if ( ( buttonValue > 0.75 * MAX_ADC_VALUE ) && ( buttonValue <= 0.85 * MAX_ADC_VALUE ) ) selectedChannel = CH_B_2;  // pattern/trigger B
    if (   buttonValue > 0.85 * MAX_ADC_VALUE )                                              selectedChannel = CH_A_1;  // pattern/trigger A

    if ( previousSelectedChannel != selectedChannel )
    {
      previousSelectedChannel = selectedChannel;
      change                  = true;
    }
    else
    {
      change = false;
    }
  }
  return change;
}

// ----------------

void testLedsAndPixels( void )
{
  testAllLeds( 10 );  // Show that leds are working.
  testNeoPixel();     // Show that neopixels are working.
  pixels.clear();     // And clear it
  testAllLeds( 5 );
}

void testAllLeds( int delayTime )
{
  int LEDS[] = { TRIG_A_PIN, TRIG_B_PIN, TRIG_C_PIN, TRIG_D_PIN };

  for ( int j = 0; j < NUM_CHANNELS+1; j++ )
  {
    for ( int i = 0; i < NUM_CHANNELS; i++ )
    {
      digitalWrite( LEDS[i], ON );  delay( delayTime );
      digitalWrite( LEDS[i], OFF ); delay( delayTime );
    }
  }
}

// ----------------

// NeoPixels

// Fill the dots one after the other with a color.

void colorWipe( uint32_t c, uint8_t wait, int direction = 0 )
{
  if ( direction == 0 )
  {
    for ( uint16_t i = 0; i < pixels.numPixels(); i++ )
    {
      pixels.setPixelColor( i, c );
      pixels.show();
      delay( wait );
    }
  }
  else
  {
    for ( uint16_t i = pixels.numPixels() - 1; i > 0; i-- )
    {
      pixels.setPixelColor( i, c );
      pixels.show();
      delay( wait );
    }
  }
}

void testNeoPixel( void )
{
  colorWipe( pixels.Color( 5, 0, 0 ), 25 );    // Red
  colorWipe( pixels.Color( 0, 5, 0 ), 20, 1 ); // Green
  colorWipe( pixels.Color( 0, 0, 5 ), 15 );    // Blue
}

// Erases all lights on the NeoPixel in preparation for an updated pattern to be displayed.

void ClearNeoPixelPattern( void )
{
  for ( int i = 0; i < NUM_NEOP_LEDS; i++ )              // Step through each pixel in the ring and
  {
    pixels.setPixelColor( i, pixels.Color( 0, 0, 0 ) );  // turn each pixel off.
  }
  pixels.show();                                         // Update the pixels displayed on the ring.
}

void getCursorColor( int mode, int& R, int& G, int& B )
{
  switch ( mode )
  {
    case PATTERN_LENGTH_MODE: R = 0; G = 4; B = 4; break;  // Cyan   = Length
    case NUM_PULSE_MODE:      R = 5; G = 0; B = 5; break;  // Violet = Pulses
    case ROTATE_MODE:         R = 5; G = 5; B = 5; break;  // White  = Rotation
  }
}

// Calculates the amplitude value of the oscillation used to modulate the brightness of the NeoPixels.

void updateCnt( float& cnt, float& delta, float minDelta )
{
#ifdef LED_FX
  cnt += delta;
  if ( cnt > 30 ) delta = -delta;
#ifdef LED_FX_BLACK
  if ( cnt < 0 ) delta = -delta;
#else
  if ( cnt < minDelta ) delta = -delta;
#endif
#endif
}

// ----------------

// In program mode, Show whether there are patches stored in the 16 memory cells or not.
//   Red   = memory cell is in use.
//   Green = memory cell is empty.

void showPatchMemory( int selectedPatch, unsigned int memoryCellsInUse )
{
  int R = 0;
  int G = 0;
  int B = 0;

#ifdef PATCH_MEMORY_LED_FX
  const  float DELTA = 0.04F;
  static float delta = DELTA;
#endif
  static float cnt   = 1.0;

  for ( int i = 0; i < NR_OF_MEMORY_CELLS; i++ )  // Step through each pixel in the ring.
  {
    int cellIsInUse = ( CHECK_BIT( memoryCellsInUse, i ) > 0 );

    if      ( cellIsInUse && ( i == selectedPatch ) ) { R = (int)( 15.0 * cnt ); G = 0;                   B = 0; }
    else if ( i == selectedPatch )                    { R = 0;                   G = (int)( 15.0 * cnt ); B = 0; }  // Show position of rotary encoder is on current Patch.
    else if ( cellIsInUse )                           { R = 1;                   G = 0;                   B = 0; }
    else                                              { R = 0;                   G = 1;                   B = 0; }  // Show position of rotary encoder when in other positions.

    pixels.setPixelColor( i, pixels.Color( R, G, B ) );
  }

  pixels.show();  // Update the pixels displayed on the NeoPixel ring.

#ifdef PATCH_MEMORY_LED_FX
  updateCnt( cnt, delta, DELTA );
#endif
}

// showBitPattern displays a Euclidean pattern associated with the bits set in Pattern.
// The color of the pattern is dictated by channel number "channelNumber".
// You can change the ring colors to match the buttonValue and output LEDs you use in your build here.

void showBitPattern( int channelNumber, unsigned int pattern, int patternLength, int mode )
{
  const  float DELTA = 0.05;
  static float delta = DELTA;
  static float cnt   = 30.0;

  int R = 0;
  int G = 0;
  int B = 0;

  // Get the cursor colors to denote the beginning and end of the pattern.
  // We use low intensity here, to contrast with the actual pattern.
  getCursorColor( mode, R, G, B );

  // Show the pattern length by highlighting the 1st and last led.
  // Choose the color of the 'mode' for this. This makes it easier to see in what mode the sequencer is.

  pixels.setPixelColor( 0,                                             pixels.Color( R, G, B ) );
  pixels.setPixelColor( currentPatch.patternLength[channelNumber] - 1, pixels.Color( R, G, B ) );

  switch ( channelNumber )
  {
    case CH_A_1: R =(int)cnt;  G =  0;       B =  0;       break;  // Pattern/trigger A/1 will be shown in red.
    case CH_B_2: R =  0;       G = (int)cnt; B =  0;       break;  // Pattern/trigger B/2 will be shown in green.
    case CH_C_3: R =  0;       G =  0;       B = (int)cnt; break;  // Pattern/trigger C/3 will be shown in blue.
    case CH_D_4: R = (int)cnt; G = (int)cnt; B =  0;       break;  // Pattern/trigger D/4 will be shown in yellow.
  }

  for ( int i = 0; i < patternLength; i++ )  // step through each pixel in the ring
  {
    // If the i-th bit is set, then set that pixel to the appropriate color.
    if ( pattern & ( 1 << i ) )
    {
      pixels.setPixelColor( i, pixels.Color( R, G, B ) );
    }
  }

  pixels.show();  // Update the pixels displayed on the NeoPixel led-ring.
  updateCnt( cnt, delta, 1 );
}

// ----------------

// EEPROM

// EEPROM memory layout:
//   0: selectedTriggerChannel
//   1: default programNumber
//   2: memoryCellsInUse
//   3: delayTime                  * Note that delayTime is the same for all patches being played
//   4: 16 patches:
//      - pulses[4]
//      - channelPattern[4]
//      - patternLength[4]

// Write all patches to EEPROM.

void writePatchesToEEPROM( Patch patches[], unsigned int memoryCellsInUse, int patchNumber, int delayTime, int selectedTriggerChannel )
{
  int address = EEPROM_BASE_ADDRESS;
  EEPROM.put( address, selectedTriggerChannel ); address += sizeof( int );  // currently selected trigger channel number.
  EEPROM.put( address, patchNumber );            address += sizeof( int );  // number indicating the currently loaded program.
  EEPROM.put( address, memoryCellsInUse );       address += sizeof( int );  // which memory cells are in use.
  EEPROM.put( address, delayTime );              address += sizeof( int );  // tempo of the patches.

  for ( int i = 0; i < NR_OF_MEMORY_CELLS; i++ )
  {
    EEPROM.put( address, patches[i] );                                      // pulses, patterns, lengths
    address += sizeof( Patch );
  }
}

// Write one particular patch to EEPROM.

void writePatchToEEPROM( Patch thisPatch, unsigned int memoryCellsInUse, int patchNumber, int delayTime, int selectedTriggerChannel )
{
  int address = EEPROM_BASE_ADDRESS;
  EEPROM.put( address, selectedTriggerChannel ); address += sizeof( int );
  EEPROM.put( address, patchNumber );            address += sizeof( int );
  EEPROM.put( address, memoryCellsInUse );       address += sizeof( int );
  EEPROM.put( address, delayTime );              address += sizeof( int );

  for ( int i = 0; i < NR_OF_MEMORY_CELLS; i++ )
  {
    if ( i == patchNumber )  // Only write/replace chosen patch to/in EEPROM.
    {
      EEPROM.put( address, thisPatch );
      break;
    }
    address += sizeof( Patch );
  }
}

// Read all patches from EEPROM

int readPatchesFromEEPROM( Patch patches[], unsigned int& memoryCellsInUse, int& delayTime, int& selectedTriggerChannel )
{
  int chosenPatchNumber = 0;

  int address = EEPROM_BASE_ADDRESS;
  EEPROM.get( address, selectedTriggerChannel ); address += sizeof( int );
  EEPROM.get( address, chosenPatchNumber );      address += sizeof( int );
  EEPROM.get( address, memoryCellsInUse );       address += sizeof( int );
  EEPROM.get( address, delayTime );              address += sizeof( int );

  for ( int i = 0; i < NR_OF_MEMORY_CELLS; i++ )
  {
    if ( memoryCellsInUse & ( 1 << i ) )
    {
      EEPROM.get( address, patches[i] );
      address += sizeof( Patch );
    }
    else
    {
      createEmptyPatch( patches[i] );  // If a cell is not in use, create and load an empty patch.
    }
  }

  return ( chosenPatchNumber );
}

// Read one particular patch from the EEPROM.
// Only the patch itself is read.

void readPatchFromEEPROM( Patch patches[], int chosenPatchNumber )
{
  int address = EEPROM_BASE_ADDRESS;
  address += sizeof( int );          // selectedTriggerChannel
  address += sizeof( int );          // patchNumber
  address += sizeof( int );          // memoryCellsInUse
  address += sizeof( int );          // delayTime

  for ( int i = 0; i < NR_OF_MEMORY_CELLS; i++ )
  {
    if ( i == chosenPatchNumber )
    {
      EEPROM.get( address, patches[i] );  // Read only the content of the patches[i].
#ifdef DEBUG
      displayPatch( patches[i] );
#endif
      break;
    }

    address += sizeof( Patch );
  }
}

void initializePatchInEeprom( void )
{
  // We write one predefined patch to EEPROM. This leaves 15 free empty patch memory cells.
  unsigned int memoryCellsInUse       = 0;
  int          selectedTriggerChannel = CH_A_1;
  int          delayTime              = INIT_DELAY_TIME;

  int          pulses        [NUM_CHANNELS];  // The number of pulses to be generated for each of the 4 triggers.
  int          patternLength [NUM_CHANNELS];  // The pattern length for each trigger output.
  unsigned int channelPattern[NUM_CHANNELS];
  int          patchNr        = 0;
  int          initialPatchNr = 0;

  // Start up the generator with default bit patterns for triggers A/1 B/2 C/3 D/4.
  // Each number/pattern persists between loop iterations.

  pulses[CH_A_1] = 4; channelPattern[CH_A_1] = 0b0000100100100100; /*  2340 */ patternLength[CH_A_1] = NUM_NEOP_LEDS;
  pulses[CH_B_2] = 3; channelPattern[CH_B_2] = 0b0100001000010000; /* 16912 */ patternLength[CH_B_2] = NUM_NEOP_LEDS;
  pulses[CH_C_3] = 5; channelPattern[CH_C_3] = 0b0100100100100100; /* 18724 */ patternLength[CH_C_3] = NUM_NEOP_LEDS;
  pulses[CH_D_4] = 5; channelPattern[CH_D_4] = 0b0010100100100100; /* 10532 */ patternLength[CH_D_4] = NUM_NEOP_LEDS;

  for ( int j = 0; j < NUM_CHANNELS; j++ )
  {
    patches[initialPatchNr].pulses[j]         = pulses[j];
    patches[initialPatchNr].channelPattern[j] = channelPattern[j];
    patches[initialPatchNr].patternLength[j]  = patternLength[j];
  }

  memoryCellsInUse = memoryCellsInUse | ( 1 << initialPatchNr );

#ifdef DEBUG
  displayPatch( initialPatchNr );
#endif

  // The rest of the 16 patterns are empty.
  for ( patchNr = 1; patchNr < NR_OF_MEMORY_CELLS; patchNr++ )
  {
    for ( int i = 0; i < NUM_CHANNELS; i++ )
    {
      patches[patchNr].pulses        [i] = 0;
      patches[patchNr].channelPattern[i] = 0;
      patches[patchNr].patternLength [i] = NUM_NEOP_LEDS;;
    }
  }

  writePatchesToEEPROM( patches, memoryCellsInUse, initialPatchNr, delayTime, selectedTriggerChannel );
}

// Store this program number as the current program number.

void writePatchNumberToEEPROM( int patchNumber )
{
  int address = EEPROM_BASE_ADDRESS + sizeof( int );
  EEPROM.put( address, patchNumber );
}

void writeTriggerChannelToEEPROM( int triggerChannel )
{
  int address = EEPROM_BASE_ADDRESS;
  EEPROM.put( address, triggerChannel );
}

void copyPatch( Patch srcPatch, Patch& dstPatch )
{
  for ( int i = 0; i < NUM_CHANNELS; i++ )
  {
    dstPatch.pulses        [i] = srcPatch.pulses[i];
    dstPatch.patternLength [i] = srcPatch.patternLength[i];
    dstPatch.channelPattern[i] = srcPatch.channelPattern[i];
  }
}

void createEmptyPatch( Patch& dstPatch )
{
  for ( int i = 0; i < NUM_CHANNELS; i++ )
  {
    dstPatch.pulses        [i] = 0;
    dstPatch.patternLength [i] = NUM_NEOP_LEDS;
    dstPatch.channelPattern[i] = 0;
  }
}

// ----------------

#ifdef DEBUG

void displayPatch( int patchNumber )
{
  sprintf( msg, "patchNumber: %d", patchNumber ); Serial.println( msg );

  for ( int j = 0; j < NUM_CHANNELS; j++ ) { sprintf( msg, "pulses[%d]  = %d", j, patches[patchNumber].pulses[j] );         Serial.println( msg ); }
  for ( int j = 0; j < NUM_CHANNELS; j++ ) { sprintf( msg, "pattern[%d] = %d", j, patches[patchNumber].channelPattern[j] ); Serial.println( msg ); }
  for ( int j = 0; j < NUM_CHANNELS; j++ ) { sprintf( msg, "length[%d]  = %d", j, patches[patchNumber].patternLength[j] );  Serial.println( msg ); }

  Serial.print( "\n" );
}

// For debug purposes.

void displayPatch( Patch thisPatch )
{
  for ( int j = 0; j < NUM_CHANNELS; j++ ) { sprintf( msg, "pulses[%d]  = %d", j, thisPatch.pulses[j] );         Serial.println( msg ); }
  for ( int j = 0; j < NUM_CHANNELS; j++ ) { sprintf( msg, "pattern[%d] = %d", j, thisPatch.channelPattern[j] ); Serial.println( msg ); }
  for ( int j = 0; j < NUM_CHANNELS; j++ ) { sprintf( msg, "length[%d]  = %d", j, thisPatch.patternLength[j] );  Serial.println( msg ); }
  Serial.print( "\n" );
}

#endif

// ----

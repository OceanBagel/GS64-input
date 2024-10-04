/*
 * Copyright (c) 2009 Andrew Brown
 * Permission is hereby granted, free of charge, to any person
 * obtaining a copy of this software and associated documentation
 * files (the "Software"), to deal in the Software without
 * restriction, including without limitation the rights to use,
 * copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following
 * conditions:
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
 * OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
 * HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
 * WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * OTHER DEALINGS IN THE SOFTWARE.
 */

/*
 * Some of the code was copied from rcombs's code:
 * https://github.com/rcombs/tas-playback/blob/master/serial-n64/serial-n64.ino
 * 
 * HOW TO USE:
 * Obtain an Arduino Micro. Put a button between pin 3 and GND (optional).
 * Connect two wires to GND and the pin labeled SS (by the reset button).
 * These two wires will connect to the controller port (don't connect them yet).
 * Find the Gameshark code you want to enter. Make sure separate codes are separated by at least one empty line.
 * Copy the Gameshark code and paste it here, making sure to uncheck "split output into multiple lines":
 * https://tomeko.net/online_tools/cpp_text_escape.php?lang=en
 * Take the resulting string and put it into the code on the line that starts with #define PAYLOAD. (Replace the placeholder codes.)
 * Upload the sketch to the Arduino and leave the Arduino plugged in. Optionally open the serial monitor to monitor progress.
 * If the sketck exceeds the Arduino's memory, you will need to split the code into smaller segments.
 * With a normal controller plugged in, start the N64 with the Gameshark and game inserted.
 * Go to Options and ensure Menu Scrolling is OFF. (This is crucial for timing.)
 * Navigate to the new code window, optionally give the code a name, and then navigate to the data box.
 * The cursor should be on the top left digit, and 0 should be selected on the numpad.
 * Note that each code can store 62 lines, so new codes will be created as needed and saved without names.
 * Unplug the controller, connect the ground wire to the right controller pin, and connect the SS pin wire to the center controller pin.
 * WARNING: Do not plug the wires into the wrong controller port pins or else you may damage your N64 or Arduino.
 * When you're ready to start, connect pin 3 to ground, either using the button or by bridging the two pins with a wire.
 * The code will be entered slowly due to Gameshark input restrictions (a little over 1 input per second).
 */

#include "pins_arduino.h"

// Payload definition for the Gameshark codes to enter
// Codes must be separated by \n, and codes meant to be in separate entries must be separated by at least two \n
// Spaces are optional and are ignored
// Use a C/C++ string converter if using codes from pastebin: https://tomeko.net/online_tools/cpp_text_escape.php?lang=en
#define PAYLOAD "810743F0 4459\n810743F2 8F5E\n\n803DE387 0000\n\nD03DE37B 0001\n803DE37B 0002\n\nD1074006 0002\n803DE38B 0001\nD1074006 0002\n803DE37B 0000\nD1074006 0002\n803DE38F 0000\n\nD1074006 0001\n803DE38F 0001\nD1074006 0001\n803DE37B 0000\n\nD1074006 0004\n803DE37B 0004\nD1074006 0004\n803DE383 0006\n \nD1074006 0010\n810743F2 0001\n\n800DBEDA 0042\n\nD1074006 0020\n8007408B 0004\nD1074006 0020\n81074090 0013\nD1074006 0020\n80074093 0001"

#define STATUS_PIN 13

#define SERIAL_BAUD_RATE 9600

#define N64_PIN 17
#define N64_HIGH DDRB &= ~0x01
#define N64_LOW DDRB |= 0x01
#define N64_QUERY (PINB & 0x01)

#define LED_HIGH DDRB &= ~0x20
#define LED_LOW DDRB |= 0x20

#define AA_PRESS 0b10000000
#define BB_PRESS 0b01000000
#define AB_PRESS 0b11000000 // A and B together, only used for saving the code
#define ZZ_PRESS 0b00100000
#define DU_PRESS 0b00001000
#define DD_PRESS 0b00000100
#define DL_PRESS 0b00000010
#define DR_PRESS 0b00000001
#define NO_PRESS 0b00000000

#define INPUT_BUFFER_UPDATE_TIMEOUT 10 // 10 ms

// Payload is stored in program memory which can hold more data
const char payload[] PROGMEM = PAYLOAD;

static char n64_raw_dump[38]; // maximum recv is 1+2+32 bytes + 1 bit
// n64_raw_dump does /not/ include the command byte. That gets pushed into
// n64_command:
static unsigned char n64_command;
// bytes to send to the 64
// maximum we'll need to send is 33, 32 for a read request and 1 CRC byte
static unsigned char n64_buffer[33];
static void get_n64_command();
static void n64_send();

static void updateInputBuffer();

static bool finished = false;

int payloadIndex = 0;
int currentDigitX = 0;
int currentDigitY = 0;
int inputIndex = 0;
int delayWait = 0;
int lastInputInSet = 0;
int newLines = 0;
static unsigned char inputSet[7] = {NO_PRESS, NO_PRESS, NO_PRESS, NO_PRESS, NO_PRESS, NO_PRESS, NO_PRESS};
// Must use strlen_P because payload is in progmem
static int payloadSize = strlen_P(payload);
static void calculateNextInputs();

// This controls how many frames happen between each input. Set it as low as Gameshark can respond to it.
static int numDelayFrames = 27;

// This pin can be changed to whatever is most convenient
const int buttonPin = 3;
int previousButtonState = HIGH;

void setup()
{
  //Don't begin main loop execution until the button is pressed
  pinMode(buttonPin, INPUT_PULLUP);
  int buttonState = digitalRead(buttonPin);


  Serial.begin(SERIAL_BAUD_RATE);
  while (!Serial) { ; } // wait for serial port to connect. Needed for native USB port only
  Serial.println("Starting up");

  // Status LED
  digitalWrite(STATUS_PIN, LOW);
  pinMode(STATUS_PIN, OUTPUT);

  // Communication with the N64 on this pin
  digitalWrite(N64_PIN, LOW);
  pinMode(N64_PIN, INPUT);

  // Pause until pin 3 is set to low (with either a push button or a jumper wire)
  while (!(buttonState == LOW && previousButtonState == HIGH)) {
    buttonState = digitalRead(buttonPin);
  }
  updateInputBuffer();

  Serial.println(F("Initialization done."));
}

void loop()
{
    //Serial.println(F("Entering main loop"));
    unsigned char data, addr;
    unsigned long updateTime;

    // wait to make sure the line is idle before
    // we begin listening
    
    for (int idle_wait=32; idle_wait>0; --idle_wait) {
        if (!N64_QUERY) {
            idle_wait = 32;
        }
    }
    
    noInterrupts();
    // Wait for incomming 64 command
    // this will block until the N64 sends us a command
    get_n64_command();

    
    // 0x00 is identify command
    // 0x01 is status
    // 0x02 is read
    // 0x03 is write
    switch (n64_command)
    {
        case 0x00:
        case 0xFF:
            // identify
            // mutilate the n64_buffer array with our status
            // we return 0x050001 to indicate we have a rumble pack
            // or 0x050002 to indicate the expansion slot is empty
            //
            // 0xFF I've seen sent from Mario 64 and Shadows of the Empire.
            // I don't know why it's different, but the controllers seem to
            // send a set of status bytes afterwards the same as 0x00, and
            // it won't work without it.
            n64_buffer[0] = 0x05;
            n64_buffer[1] = 0x00;
            n64_buffer[2] = 0x01;

            n64_send(n64_buffer, 3, 0);
            interrupts();
            Serial.println(F("Controller identified"));
            break;
        case 0x01:            
            // If the TAS is finished, there's nothing left to do.
            if (finished)
                break;
        
            n64_buffer[0] = inputSet[inputIndex];
            n64_buffer[1] = 0x00;
            n64_buffer[2] = 0x00;
            // blast out the pre-assembled array in n64_buffer
            n64_send(n64_buffer, 4, 1);
            interrupts();
            //Serial.println(F("Received an input command"));

            // update input buffer and make sure it doesn't take too long
            updateTime = micros();

            

            // Serial.print(F("Pos: "));
            // Serial.print(inputIndex);
            // Serial.print(F(" Data: "));
            // Serial.println(inputSet[inputIndex]);
            
            updateInputBuffer();
            
            // Record if it took longer than expected
            updateTime = micros() - updateTime;
            if (updateTime > INPUT_BUFFER_UPDATE_TIMEOUT * 1000) {
                Serial.print(F("Input buffer update took to long ("));
                Serial.print(updateTime / 1000);
                Serial.println(F(" ms)"));
            }
            break;
        // The rest of the commands will not be encountered.
        case 0x02:
        case 0x03:
        default:
            interrupts();
            Serial.print(F("Invalid command: 0x"));
            Serial.println(n64_command, HEX);
            break;
    }
}

static void updateInputBuffer() {
    if (delayWait == 0) {
      delayWait = numDelayFrames;
      if (inputSet[inputIndex] == AB_PRESS) {
        delayWait = 599; //Delay for longer due to saving the code
      }
      // Determine whether to calculate the next input set or return an existing input
      if (inputIndex == lastInputInSet || inputIndex == 6) {
        if (payloadIndex == payloadSize) {
          finished = true;
        }
        inputIndex = 0;
        calculateNextInputs();
      }
      // Otherwise, advance to the next input
      else {
        inputIndex += 1;
      }      

    }
    else {
      // Serial.print(F("Delay wait is at "));
      // Serial.println(delayWait);
      inputSet[inputIndex] = NO_PRESS;
      delayWait -= 1;
    }
}

/**
 * Complete copy and paste of gc_send, but with the N64
 * pin being manipulated instead.
 */
static void n64_send(unsigned char *buffer, char length, bool wide_stop)
{
    // The original code here was written for the Arduino Nano.
    // I modified it for the Arduino Micro, so some comments may no longer apply.
    asm volatile (";Starting N64 Send Routine");
    // Send these bytes
    char bits;

    // This routine is very carefully timed by examining the assembly output.
    // Do not change any statements, it could throw the timings off
    //
    // We get 16 cycles per microsecond, which should be plenty, but we need to
    // be conservative. Most assembly ops take 1 cycle, but a few take 2
    //
    // I use manually constructed for-loops out of gotos so I have more control
    // over the outputted assembly. I can insert nops where it was impossible
    // with a for loop

    asm volatile (";Starting outer for loop");
outer_loop:
    {
        asm volatile (";Starting inner for loop");
        bits=8;
inner_loop:
        {
            // Starting a bit, set the line low
            asm volatile (";Setting line to low");
            N64_LOW; // 1 op, 2 cycles

            asm volatile (";branching");
            if (*buffer >> 7) {
                asm volatile (";Bit is a 1");
                // 1 bit
                // remain low for 1us, then go high for 3us
                // nop block 1
                asm volatile ("nop\nnop\n");

                asm volatile (";Setting line to high");
                N64_HIGH;

                // nop block 2
                // we'll wait only 2us to sync up with both conditions
                // at the bottom of the if statement
                asm volatile ("nop\nnop\nnop\nnop\nnop\n"
                              "nop\nnop\nnop\nnop\nnop\n"
                              "nop\nnop\nnop\nnop\nnop\n"
                              "nop\nnop\nnop\nnop\nnop\n"
                              "nop\nnop\nnop\nnop\nnop\n"
                              "nop\nnop\nnop\nnop\nnop\n"
                              "nop\nnop\nnop\nnop\n"
                              );
            } else {
                asm volatile (";Bit is a 0");
                // 0 bit
                // remain low for 3us, then go high for 1us
                // nop block 3
                asm volatile ("nop\nnop\nnop\nnop\nnop\n"
                              "nop\nnop\nnop\nnop\nnop\n"
                              "nop\nnop\nnop\nnop\nnop\n"
                              "nop\nnop\nnop\nnop\nnop\n"
                              "nop\nnop\nnop\nnop\nnop\n"
                              "nop\nnop\nnop\nnop\nnop\n"
                              "nop\nnop\nnop\nnop\n");

                asm volatile (";Setting line to high");
                N64_HIGH;

                // wait for 1us
                asm volatile ("; end of conditional branch, need to wait 1us more before next bit");
            }
            // end of the if, the line is high and needs to remain
            // high for exactly 16 more cycles, regardless of the previous
            // branch path

            asm volatile (";finishing inner loop body");
            --bits;
            if (bits != 0) {
                // nop block 4
                // this block is why a for loop was impossible
                asm volatile ("nop\nnop\nnop\n");
                // rotate bits
                asm volatile (";rotating out bits");
                *buffer <<= 1;

                goto inner_loop;
            } // fall out of inner loop
        }
        asm volatile (";continuing outer loop");
        // In this case: the inner loop exits and the outer loop iterates,
        // there are /exactly/ 16 cycles taken up by the necessary operations.
        // So no nops are needed here (that was lucky!)
        asm volatile ("nop\nnop\nnop\n");
        --length;
        if (length != 0) {
            ++buffer;
            goto outer_loop;
        } // fall out of outer loop
    }

    // send a single stop (1) bit
    // nop block 5
    asm volatile ("nop\nnop\nnop\nnop\n");
    N64_LOW;
    // wait 1 us, 16 cycles, then raise the line
    // take another 3 off for the wide_stop check
    // 16-2-3=11
    // nop block 6
    asm volatile ("nop\nnop\nnop\nnop\nnop\n"
                  "nop\nnop\nnop\nnop\nnop\n");
    if (wide_stop) {
        asm volatile (";another 1us for extra wide stop bit\n"
                      "nop\nnop\nnop\nnop\nnop\n"
                      "nop\nnop\nnop\nnop\nnop\n"
                      "nop\nnop\nnop\nnop\nnop\n"
                      "nop\nnop\nnop\n");
    }

    N64_HIGH;
}

/**
  * Waits for an incomming signal on the N64 pin and reads the command,
  * and if necessary, any trailing bytes.
  * 0x00 is an identify request
  * 0x01 is a status request
  * 0x02 is a controller pack read (unused here)
  * 0x03 is a controller pack write (unused here)
  *
  * All data is raw dumped to the n64_raw_dump array, 1 bit per byte, except
  * for the command byte, which is placed all packed into n64_command
  */
static void get_n64_command()
{
  // Unlike n64_send, this code was not updated for the Arduino Micro. However, it still seems to work fine.
    int bitcount;
    char *bitbin = n64_raw_dump;

    n64_command = 0;

    bitcount = 8;

read_loop:
        // wait for the line to go low
        while (N64_QUERY){}

        // wait approx 2us and poll the line
        asm volatile (
                      "nop\nnop\nnop\nnop\nnop\n"
                      "nop\nnop\nnop\nnop\nnop\n"
                      "nop\nnop\nnop\nnop\nnop\n"
                      "nop\nnop\nnop\nnop\nnop\n"
                      "nop\nnop\nnop\nnop\nnop\n"
                      "nop\nnop\nnop\nnop\nnop\n"
                );
        if (N64_QUERY)
            n64_command |= 0x01;

        --bitcount;
        if (bitcount == 0)
            goto read_more;

        n64_command <<= 1;

        // wait for line to go high again
        // I don't want this to execute if the loop is exiting, so
        // I couldn't use a traditional for-loop
        while (!N64_QUERY) {}
        goto read_loop;

read_more:
        switch (n64_command)
        {
            case (0x03):
                // write command
                // we expect a 2 byte address and 32 bytes of data
                bitcount = 272 + 1; // 34 bytes * 8 bits per byte
                //Serial.println("command is 0x03, write");
                break;
            case (0x02):
                // read command 0x02
                // we expect a 2 byte address
                bitcount = 16 + 1;
                //Serial.println("command is 0x02, read");
                break;
            case (0x00):
            case (0x01):
            default:
                // get the last (stop) bit
                bitcount = 1;
                break;
        }

        // make sure the line is high. Hopefully we didn't already
        // miss the high-to-low transition
        while (!N64_QUERY) {}
read_loop2:
        // wait for the line to go low
        while (N64_QUERY){}

        // wait approx 2us and poll the line
        asm volatile (
                      "nop\nnop\nnop\nnop\nnop\n"
                      "nop\nnop\nnop\nnop\nnop\n"
                      "nop\nnop\nnop\nnop\nnop\n"
                      "nop\nnop\nnop\nnop\nnop\n"
                      "nop\nnop\nnop\nnop\nnop\n"
                      "nop\nnop\nnop\nnop\nnop\n"
                );
        *bitbin = N64_QUERY;
        ++bitbin;
        --bitcount;
        if (bitcount == 0)
            return;

        // wait for line to go high again
        while (!N64_QUERY) {}
        goto read_loop2;
}

static void calculateNextInputs() { // This function calculates what the next set of inputs should be based on the next digit
  // First check if the next read will be out of bounds
  if (payloadIndex + 1 >= payloadSize) {
    // End of payload
    unsigned char newInputSet[7] = {BB_PRESS, BB_PRESS, AA_PRESS, NO_PRESS, NO_PRESS, NO_PRESS, NO_PRESS};
    lastInputInSet = 2;
    memcpy(inputSet, newInputSet, sizeof(newInputSet));
    payloadIndex++;
    Serial.println(F("Reached the end."));
    return;
  }

  // Serial.println(F("Reading next digit"));
  
  // Next read the next digit
  char codeDigit = pgm_read_byte_near(payload + payloadIndex++);
  // Skip any spaces
  while (codeDigit == ' ') {
    codeDigit = pgm_read_byte_near(payload + payloadIndex++);
  }

  // Next establish whether it's a digit or a newline
  if (codeDigit == '\n') {
    //Check for multiple newlines in a row, indicating a new code should be started
    int newLinesInARow = 1;
    while ((payloadIndex + newLinesInARow - 1 < payloadSize) && pgm_read_byte_near(payload + payloadIndex + newLinesInARow - 1) == '\n') {
      newLinesInARow++;
    }
    if (payloadIndex + newLinesInARow - 1 == payloadSize) {
      // End of payload, ended with one or multiple newlines
      unsigned char newInputSet[7] = {BB_PRESS, BB_PRESS, AA_PRESS, NO_PRESS, NO_PRESS, NO_PRESS, NO_PRESS};
      lastInputInSet = 2;
      memcpy(inputSet, newInputSet, sizeof(newInputSet));
      payloadIndex++;
      Serial.println(F("Reached the end.."));
      return;
    }
    else { // Set the payload index to the next non-newline value
      payloadIndex = payloadIndex + newLinesInARow - 1;
    }
    if (newLinesInARow == 1) { // Only one newline, continue on the current code
      newLines++;
      if (newLines < 62) {
        // newline so do the newline inputs
        unsigned char newInputSet[7] = {BB_PRESS, DD_PRESS, AA_PRESS, NO_PRESS, NO_PRESS, NO_PRESS, NO_PRESS};
        lastInputInSet = 2;
        memcpy(inputSet, newInputSet, sizeof(newInputSet));
        // also update X and Y to 0, 0 (the digit 0)
        currentDigitX = 0;
        currentDigitY = 0;
      }
      else {
        newLines = 0;
        // Out of lines, need to make a new code
        unsigned char newInputSet[7] = {BB_PRESS, AB_PRESS, BB_PRESS, AA_PRESS, ZZ_PRESS, DD_PRESS, AA_PRESS};
        lastInputInSet = 6;
        memcpy(inputSet, newInputSet, sizeof(newInputSet));
        // also update X and Y to 0, 0 (the digit 0)
        currentDigitX = 0;
        currentDigitY = 0;
      }
      return;
    }
    else { // Multiple newlines, need to make a new code
      newLines = 0;
      unsigned char newInputSet[7] = {BB_PRESS, AB_PRESS, BB_PRESS, AA_PRESS, ZZ_PRESS, DD_PRESS, AA_PRESS};
      lastInputInSet = 6;
      memcpy(inputSet, newInputSet, sizeof(newInputSet));
      // also update X and Y to 0, 0 (the digit 0)
      currentDigitX = 0;
      currentDigitY = 0;
      return;
    }
  }

  Serial.print(F("Parsed digit: "));
  Serial.print(codeDigit);
  Serial.println(F(""));
  
  // It's a digit, so do a switch case to find target X and Y
  int targetX = 0;
  int targetY = 0;
  switch (codeDigit) {
    default:
    case '0': // it's already 0 so just break
      break;
    case '1':
      targetX = 1;
      targetY = 0;
      break;
    case '2':
      targetX = 2;
      targetY = 0;
      break;
    case '3':
      targetX = 3;
      targetY = 0;
      break;
    case '4':
      targetX = 0;
      targetY = 1;
      break;
    case '5':
      targetX = 1;
      targetY = 1;
      break;
    case '6':
      targetX = 2;
      targetY = 1;
      break;
    case '7':
      targetX = 3;
      targetY = 1;
      break;
    case '8':
      targetX = 0;
      targetY = 2;
      break;
    case '9':
      targetX = 1;
      targetY = 2;
      break;
    case 'A':
    case 'a':
      targetX = 2;
      targetY = 2;
      break;
    case 'B':
    case 'b':
      targetX = 3;
      targetY = 2;
      break;
    case 'C':
    case 'c':
      targetX = 0;
      targetY = 3;
      break;
    case 'D':
    case 'd':
      targetX = 1;
      targetY = 3;
      break;
    case 'E':
    case 'e':
      targetX = 2;
      targetY = 3;
      break;
    case 'F':
    case 'f':
      targetX = 3;
      targetY = 3;
      break;
  }

  // Now calculate how far we need to move
  int deltaX = targetX - currentDigitX;
  int deltaY = targetY - currentDigitY;

  // Variables to track progress of the cursor
  int movedX = 0;
  int movedY = 0;

  bool pressedA = false;

  for (int i = 0; i < 7; i++) {
    if (movedX != deltaX) {
      if (deltaX < 0) {
        inputSet[i] = DL_PRESS;
        movedX--;
      }
      else {
        inputSet[i] = DR_PRESS;
        movedX++;
      }
    }
    else if (movedY != deltaY) {
      if (deltaY < 0) {
        inputSet[i] = DU_PRESS;
        movedY--;
      }
      else {
        inputSet[i] = DD_PRESS;
        movedY++;
      }
    }
    else if (!pressedA) {
      inputSet[i] = AA_PRESS;
      pressedA = true;
      lastInputInSet = i;
    }
    else {
      inputSet[i] = NO_PRESS;
    }
  }

  // Now set the digits
  currentDigitX = targetX;
  currentDigitY = targetY;
  return;
}
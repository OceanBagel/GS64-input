# GS64-input
## A program for entering N64 GameShark codes using an Arduino Micro as a replay device

This is a program written for an Arduino Micro to be able to input GameShark codes automatically by pasting the text of the codes into the script.

Some of the code was copied from rcombs's code here: https://github.com/rcombs/tas-playback/blob/master/serial-n64/serial-n64.ino

 ### HOW TO USE:
 * Obtain an Arduino Micro. Put a button between pin 3 and GND (optional).
 * Connect two wires to GND and the pin labeled SS (by the reset button). These two wires will connect to the controller port (don't connect them yet).
 * Find the Gameshark code you want to enter. Make sure separate codes are separated by at least one empty line.
 * Copy the Gameshark code and paste it here, making sure to uncheck "split output into multiple lines": https://tomeko.net/online_tools/cpp_text_escape.php?lang=en
 * Take the resulting string and put it into the code on the line that starts with `#define PAYLOAD`. (Replace the placeholder codes.)
 * Upload the sketch to the Arduino and leave the Arduino plugged in. Optionally open the serial monitor to monitor progress.
   * If the sketch exceeds the Arduino's memory, you will need to split the code into smaller segments.
 * With a normal controller plugged in, start the N64 with the Gameshark and game inserted.
 * Go to Options and ensure Menu Scrolling is OFF. This is crucial for timing.
 * Navigate to the new code window, optionally give the code a name, and then navigate to the data box.
 * The cursor should be on the top left digit, and 0 should be selected on the numpad.
   * Note that each code can store 62 lines, so new codes will be created as needed and saved without names.
 * Unplug the controller, connect the ground wire to the **right** controller port pin, and connect the SS pin wire to the **center** controller port pin.
 * When you're ready to start, connect pin 3 to ground, either using the button or by bridging the two pins with a wire.

> [!WARNING]
> Do not plug the wires into the wrong controller port pins or else you may damage your N64 or Arduino.
 
 The code will be entered slowly due to Gameshark input restrictions (a little over 1 input per second). There is a potentially faster way to enter codes by using fast scrolling (holding down directions on the dpad), but this would be much more complicated and so wasn't implemented.

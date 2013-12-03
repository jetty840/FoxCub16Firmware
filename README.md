Firmware for the FoxCub16 controller, please see forum for details.

Built using Arduino IDE 1.0.5.

There is only one firmware for both regular operation and testing, you don't need to change firmware
afterwards unless you want to upgrade to a newer firmware.

Remember to remove the programming jumper during firmware programming, and remember
to put it back for regular operation.  The jumper connects the incoming RS485 to the Atmega328,
and as the same pin is used for the RS232 when programming, you need to set the jumper correctly.

When programming, disconnect the incoming / outgoing RS485 too, so that you're not powering from
2 different power supplies at the same time (USB and external 5V).

Baud rate, Renard starting address can be configured via the config button in the field
if required, settings are stored / remembered in eeeprom.

Defaults for a blank Atmega328 are Starting Address = 1 and Baud Rate = 57600.


Config Button Operation
=======================

There are 4 types of press:

1. Press (hold for about 0.25 - 1 second)
2. Short Press (hold for 5 seconds)
3. Medium Press (hold for 10 seconds)
4. Long Press (hold for 15 seconds)

When holding, all channels will flash at the 5, 10 and 15 second intervals to tell you
which press is active.

So for example, to enter the testing mode, you need a Short Press, so
you hold until you see the first flash (about 5 seconds), then release and you've
entered testing mode, then you press to cycle through the various tests.

If you wanted to config the starting address, then you need a Medium Press, so
you hold and wait for 2 flashes (about 10 seconds), then release and you've entered
address setting mode.

By default after power on, you're in the Renard Listening State, this is your regular
operation mode.

Test Mode
---------

Short Press, followed by Press to cycle through all of the test modes.  The modes
are:
All Channels On -> All Channels Dimming -> Channel Sequencing -> Larsen Scanner -> Zero Cross

On the final press, it returns to the Renard Listening State, and all lights go off, waiting
for Renard data.

Zero Cross alternates between the outside 2 channels when a zero cross signal is detected every second.
If there is no power, or a problem with the circuit, the outside 2 channels are static.

Renard Starting Address Mode
----------------------------

Medium Press to enter the mode, followed by a Press to increment to the next bank of 16 
addresses.

Starting address is indicated on the channels 1-8 as binary.  MSB is channel 1.  So for example, a
channel starting address of 17 would be indicated as  OOOXOOOX  where O is off, and X is on.

Valid starting addresses start from channel 1 and are in blocks of 16, for example, 1, 17, 33, 49,
65 etc. are all valid.  When address 241 is reached, the address cycles back to 1.  For addresses
larger than 241, minor code changes will be required.

After the address has been chosen, a Medium Press will store the new address and return to regular operation.
A Short Press will cancel your changes and not store them.

Baud Rate Mode
--------------

Long Press to enter the mode, followed by a Press to increment to the next Baud Rate.

Baud Rate is indicated on channels 1-8 as binary.  MSB is channel 1.  So for example, a baud rate of 2400 baud,
would be indicated as OOOOOOXO where O is off and X is on.

1  = 1200
2  = 2400
3  = 4800
4  = 9600
5  = 14400
6  = 19200
7  = 28800
8  = 38400
9  = 57600
10 = 115200

After the baud rate has been chosen, a Medium Press will store the new baud rate and return to regular operation.
A Short Press will cancel your changes and not store them.

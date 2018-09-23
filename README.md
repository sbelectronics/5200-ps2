# Atari 5200 Sony PS2 Controller Adapter
Scott Baker
smbaker@sb-software.com
http://www.smbaker.com/

## Purpose

This adapter allows you to connect a Sony PS2 "DualShock" controller to an atari 5200, using it instead of the Atari 5200's stock controller. The PS2 controller has two analog sticks. You can use them in ambidextrous mode where either stick may be used to control a single controller, or in a dual stick mode where one stick serves as controller 1 on the 5200 and the other stick serves as controller 2. Dual-stick mode is used for games like Robotron.

## Caution

Use at your own risk.

Current boards do not implement level-shifting on the PS2 controller. This is probably fine for official Sony controllers, but it is unknown whether third-party controllers tolerate the 5V logic levels and/or could be damaged.

## Usage

The controller defaults to ambidextrous mode, whether either thumbstick may be used to play your game.

Left-dual-stick mode may be entered by holding down *select* and *square* at the same time. Right-dual-stick mode may be entered by holding down *select* and *circle* at the same time. Ambidextrous mode may be re-entered by holding down *select* and *triangle*. The two dual-stick modes are useful for games like Robotron.

*L1*, *R1*, and *Triangle* may all be used interchangeably as trigger 0.

*L2*, *R2*, and *X* may all be used interchangeably as trigger 1.

Please connect the ps2 controller to the board *before* connecting the board to your atari 5200.

## Programming

This project uses a microcontroller, and the microcontroller must be programmed prior to use.

### Programming with minipro on a TL866

This is the workflow I used when programming from windows, using a TL866 programmer. The TL866 is the style programmer where you remove the chip from the pcboard and insert it into the programmer. Make sure to program both fuses.hex to the fuses, and 5200ps2.hex to the program memory.

NOTE: The Windows minipro software does not have the ATTINY861 in its chip database. Until this is fixed this makes programming using minipro on Windows impossible. I do have a patch for the opensource Ubuntu minipro software.

### Programming ICSP with usbasp/avrdude from ubuntu

This is the workflow I used to do in-circuit serial programming from Ubuntu, using a usbasp programmer. The advantage of this workflow is that no ICs need to be removed from the pcboard. Only revision 0.9 and above of the pcboard have the ICSP header. You may also be able to use a windows version of avrdude instead of using ubuntu. I have not tried windows. The following two commands will program the uses and the flash:

    make fuses
    make flash
This directory contains files that may be written using the *opensource* minipro. This is not the same minipro that runs under windows. This software runs on Linux and MacOS.

Find my fork at https://github.com/sbelectronics/minipro, with ATTINY861 support...

to program the flash:

    minipro -p attiny861 -w 5200ps2.bin

to program the fuses

    minipro -p attiny861 -c config -w fuses.txt
    
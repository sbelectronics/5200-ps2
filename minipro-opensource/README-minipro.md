This directory contains files that may be written using the *opensource* minipro. This is not the same minipro that runs under windows. This software runs on Linux and MacOS.

Use radiomanV's fork of minipro at https://gitlab.com/radiomanV/minipro

to program the flash:

    minipro -p attiny861 -w 5200ps2.bin

to program the fuses

    minipro -p attiny861 -e -c config -w fuses.txt

Program the flash first, then the fuses.

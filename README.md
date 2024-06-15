# PHA
Firmware for the LASP Teensy-based Pulse Height Analyzer

This repository was initially started with version 5.1 of the Pulse Height Analyzer 
Teensy code.

A feature was added where a few operational parameters can be
adjusted via the Serial connection that the PHA transmits data
to the LPC mainboard. This will allow the PHA to be adjusted during
flight.

The PHA has three parameters that will be adjustable via the serial line:

* Hi Gain Threshold - a 10 bit number that is the threshold level for detecting we have a pulse.
* Hi gain baseline offset - a 12 bit number that adjusts the 'zero' offset for the hi gain stage on the PHA
* Lo gain baseline offset - a 12 bit number that adjusts the 'zero' offset for the lo gain stage on the PHA

## Installation

Clone from GitHub into your Arduino IDE *libraries/* folder::

```sh
cd Documents/arduino/libraries # Or wherever your Arduino libraries are
git clone https://github.com/MisterMartin/PHA   # PHA application
```

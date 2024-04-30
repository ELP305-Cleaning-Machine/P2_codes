# README



## Introduction

Welcome to our innovative bus tracking system designed to enhance the Bus transportation experience in IIT Delhi. Our system utilizes ESP32 technology to provide real-time updates and improve service reliability. Commuters at each stop are informed via a display unit, which indicates the **anticipated arrival time** of the next bus. Our system comprises two integral components: **units installed on the buses** and **display units at the stops**

## Out of Service Indicator

- **Button Activation**: When the out-of-service button (boot push button on the bus unit ESP32) is pressed and the bus is within range of any bus stop, the display will show **"OOS"**.

## Passenger Waiting Detection

- **Sensor Activation**: If the PIR sensor detects a passenger waiting (emulated by the boot push button pressed on the bus stop ESP32) and the bus is within the range of any stop, the bus unit LED will blink twice, indicating that a passenger has arrived at the stop.

## Bus Halt Timers

- **Timer start:** All timers show 0 before the bus arrives at the first stop. The bus starts it's routine from the first stop, and all timers are updated once the shift starts (from the first stop).
- **Bus Stop Halt**: If the bus halts within a bus stop region, this will be recognized as a bus stop, and all timers will be updated accordingly.
- **Near Bus Stop Halt**: If the bus halts in the range of a bus stop but not within the bus stop region, all timers will pause.
- **Outside Bus Stop Range**: If the bus stops outside the range of all bus stops, the timers will continue but will pause after reaching a lower limit of **1**.

## Codes

The codes used during the demo of project-2 can be found [here](https://github.com/ELP305-Cleaning-Machine/P2_codes):

- The BUS UNIT's code is [BUS.ino](BUS/BUS.ino)
- BUS STOP A's code is [BusStopA.ino](BusStopA/BusStopA.ino)
- BUS STOP B's code is [BusStopB.ino](BusStopB/BusStopB.ino)
- Hungarian Notations are used in the codes
- Comments are mostly Doxygen compliant
- Doxygen HTML files are available [here](htmloutput/html/index.html)

For further assistance, please refer to the full project report [here](https://2nav.github.io/TribeC/).

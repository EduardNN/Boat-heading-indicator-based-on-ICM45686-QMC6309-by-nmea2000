# Boat-heading-indicator-based-on-ICM45686-QMC6309-by-nmea2000
An attempt to adapt the BerndCirotzki compass https://github.com/BerndCirotzki/ESP32_Precision-9_compass_BNO055 to the ICM45686+QMC6309 module and WiFi interface, using AI...


BerndCirotzki's compass https://github.com/BerndCirotzki/ESP32_Precision-9_compass_BNO055 is a good project for those
who want to build a boat compass that uses nmea2000. However, the BNO055's flaws and poorly assembled modules affect its performance. The discontinuation of this sensor also does not favor its use.
The ICM45686+QMC6309 modules, which are now commercially available, have proven themselves to be quite effective in VR trackers.
Therefore, I'm trying to adapt this module to BerndCirotzki's project.
Since I'm not a programmer, I asked Deepseek AI to help me with this.
I'm posting the results for your review.

The sketch compiles with the latest libraries as of March 2026

ArduinoIDE 2.3.8

Arduino ESP32 board 2.0.18

I haven't tested the compiled code.

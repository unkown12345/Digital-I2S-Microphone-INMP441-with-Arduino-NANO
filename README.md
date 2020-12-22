Digital I2S Microphone INMP441 with Arduino NANO
Most people means UNO/Nano is to slow for I2S_MIC INMP441 but there is a Trick toggle PB3 with Timer2 and Timer 0 to save CPU-Time ->Sampling 25Khz

In this example I take a NANO Board modified with 3.3V to connect INMP441 directly.

After reading the 24Bit-INMP441-Data there is 12µs to work with it in Realtime.

In this example I take a 12-Bit-DAC MCP4921 to show the 24Bit-INMP441-Data in Realtime.



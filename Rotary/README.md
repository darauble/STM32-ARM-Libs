# Rotary Encoder Library

This is universal rotary encoder library independent from any microcontroller
(hardware agnostic). Reading GPIOs is a responsibility of the developer, and
this library can only be used to interpret their values. Use interrupts or busy
scanning, it's your choice.

Rotary encoders usually have "A" and "B" pins and a push button ("P" pin).
Read these values and pass them to encoder's functions to get interpretation.

See the comments for details in the source files.


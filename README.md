# Tempereature-Control
This repository contains the code for a temperature control project, implemented on an STM32 Nucleo L476RG.  An analog temperature sensor and PWM outputs were used to control a lamp and a fan.

Control is done with pins configured as PWM output that activates the actuator to generate control through the fan or lamp.

The temperature is read from the previously conditioned LM35 analog sensor and a conditioning with a gain of 5.2 is used.

Used Pins:

PWM outputs:
-PC7(Lamp)
-PA8(Fan)

Sensor Input (analog input):
-PC3

# ADCS-Controls

This is a tool designed as a part of the AE-2111-I course at TU Delft.
It is used for simulating the response of the reaction wheels and reaction control thursters to achieve certain desired
angular velocity or acceleration of a spacecraft.

The program contains a very crude text-based interface, where commands are selected by typing appropriate letters
and numerical value inputs are interpreted as floats. There is no protection against invalid inputs implemented.

main.py contains all the simulation functions and text interface

properties.py stores hard-coded parameters of the spacecraft, reaction wheels and thrusters required for the simulation


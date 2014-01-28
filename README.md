Line Follower '13
=================

**A Basic PID-Control Line Follower Source Code.**

ABOUT
-----

A Line Follower Bot is an autonomous embedded system which recognizes a line/path on a contrasting background and moves along it. In this implementation it is treated as a control system. A PID Control has been used as the primary algorithm for self-correction of path based on input the bot receives through its sensors.
The source code is in Embedded C and has been documented considerably.

The entire project is divided into multiple source and header files. It also contains a shell script that will condense all the code from the files into a single file that can be compiled easily using an external compiler. A Makefile has also been included.

INSTALLATION
------------
Things you might need:-
* You might need `make`tool to use the Makefile.
* You might need **UNIX Tools** to use the *Source Code Condenser*.
* You might need `avr-gcc` (in our case) or a relevant compiler.

###Make Targets
1. **Binaries** - Creates the final binary file.
2. **clean** - Removes .obj files.
3. **cleanall** - Removes all .bin, .obj and .so files.

Just copy the source code as it is to your project directory and compile either using the **Makefile** or use the **Source Code Condenser** script to get all the source code into a single file to easily feed to your own compiler.

HOW TO?
-------
We used a **8-bit ATMEL AVR ATmega16 Microcontroller**. The pin/port usage will be apparent from the source code. You can easily change that as the source code is modular in design.

###Files
####Header Files
* **inc/adci.h** - Contains some initialization for the A/DC and its function prototype.
* **inc/lineFollwerMain.h** - Contains some global level initialization for the bot and upper-level function prototypes.
* **inc/normalizeAlgorithm.h** - Contains function prototypes for the input normalization algorithm.
* **inc/moveAlgorithm.h** - Contains initializations for the main PID-Control Algorithm and bot movement function prototypes.

####Source Files
* **src/lineFollower.c** - Contains the `main()` routine.
* **src/initializeAll.c** - Bot initialization code. Initializes A/DC, PWM units, Data Direction registers and PORT registers
* **src/inputSensorArray.c** - Code to take input from the sensor array and store them.
* **src/isRunningFeasible.c** - Code to check if bot movement is feasible or required.
* **src/normalizeSensorArray.c** - Code to normalize the sensor array input received. Used in special situations/cases.
* **src/moveLineFollower.c** - Contains the main PID-Control algorithm and functions that assign values to ports responsible for movement.

Documentation of the microcontroller is available online. Ports can be changed according to the design easily.

###Note
If you have any query, you can mail me.
If you wish to contribute, feel free to **Fork** this repository and send me a **Pull Request**!
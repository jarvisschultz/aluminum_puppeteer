About the aluminum_puppeteer Package
====================================

This is a repository that contains all of the PIC code that runs on the mobile
robots that are part of the robotic marionette project at Northwestern
University.  The project is part of Todd Murphey's research group which is
housed within the Neuroscience and Robotics Laboratory (NxR lab).

Each robot has a PIC32 processor (either a PIC32MX460F512L or a
PIC32MX795F512L), and is controlled by a master computer via an XBee network.
Each robot is responsible for controlling 4 separate DC motors via PID loops, as
well as tracking odometry, and providing a serial interface.  


Compiling and loading onto the PIC
==================================

Currently, the PIC processors are attached to UBW32 boards which are then
plugged into separate breakout boards that connect to the motors, and serial
communication circuit.  On my system, the code is compiled into a *.hex file,
and then loaded onto the UBW32 via the
[ubw32](http://www.paintyourdragon.com/uc/ubw32/index.html) command line program.  The
UBW32's have the standard bootloader onboard.  Note that with very little
modification, this code could be loaded with a PIC programmer (such as the
PICKit3).

Assuming you have make installed, and the xc32-gcc compiler (from Microchip) is
available on your system's path, the code can be compiled for the 795 and loaded
onto the UBW32 using

    make PIC=795 write

Replacing the '795' with '460' would compile for the PIC32MX460F512L.  


More Information
================

More information about the project, the NxR lab, and videos of the system
working can be found at

http://lims.mech.northwestern.edu/RESEARCH/current_projects/Puppets/Marionette_homepage.html

http://nxr.mech.northwestern.edu

http://vimeo.com/channels/numarionette


More information about the bootloading program and the UBW32 itself
can be found at

www.schmalzhaus.com/UBW32/ 

and 

www.paintyourdragon.com/uc/ubw32/index.html

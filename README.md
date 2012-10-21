FlyMaple
========

Information
-----------

This project is released under the GNU GPLv3 (see LICENSE file). Some code may have been copied from external projects. When it is the case the license is detailed in the file.
Please see the [wiki](http://wiki.open-drone.org/doku.php?id=flymaple) for more information.

How to use
----------

**Software**
Download Maple IDE:
http://leaflabs.com/docs/maple-ide-install.html

Get the code in your sketchs folder. The following command can help you. Execute it from your sketchbook.

    git clone https://github.com/opendrone/flymaple-ide.git

Open FlyMaple.pde, and the other files should appear on the tabs.

**Hardware**

You should take care of:
* Keeping the center of gravity centered <!-- be more precise here -->
* We are using Flymaple board, if you want to try, you can get one here:
http://www.dfrobot.com.cn/index.php?route=product/product&product_id=583

Files
-----

* *FlyMaple1_0.pde* − Main file with the setup() and loop() functions
* *AHRS.pde* − Functions to manage the [Altitude and Heading Reference System](http://en.wikipedia.org/wiki/Attitude_and_heading_reference_system), including the PID error correction algorithm.
* *MOTOR.pde* − 
* *CapturePPM.pde* − Radio Frequency controller functions
* *Communication.pde* − Utility functions to help debugging
* *ADXL345.pde* − Librairy file for [ADXL](https://www.sparkfun.com/products/9836) 345 3-axis digital accelerometer
* *BMP085.pde* − Librairy file for [BMP085](https://www.sparkfun.com/products/9694) barometric pressure sensor
* *HMC5883.pde* − Librairy file for [HMC5883](https://www.sparkfun.com/products/10426) 3-axis digital compass
* *ITG3205.pde* − Librairy file for [ITG3205](http://www.flytron.com/sensors/160-itg3205-digital-gyro-breakout.html) digital gyroscope

Todo
----

* integrate PID and calibrate values

Resources
---------

* http://www.mstarlabs.com/control/znrule.html
* http://aeroquad.com/showthread.php?1167-Stable-Mode-Explained

Some examples
-------------

* [PID library for the arduino](https://github.com/mwoodward/Arduino-PID-Library/blob/master/PID_v1/Examples/PID_AdaptiveTunings/PID_AdaptiveTunings.pde) by [mwoodward](https://github.com/mwoodward)

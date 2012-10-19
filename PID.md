How the PID works?
==================

What is PID?
------------

PID means Proportional, Integral, Derivative. It is the common name for a regulator using these 3 operations to stabilize a system.
It is very commonly used for multirotors.

A regulator is a function applied to the error a system detects (the difference between the expected and the measured values) to stabilize the result when there is some noise (there is ALWAYS some noise).

* **Proportional** -> Error is multiplied by a gain G
* **Integral** -> The integral of the error is multiplied by a gain Ti
* **Derivative** -> The derivate of the error is divided by a gain Td

More information on the PID on [wikipedia](http://en.wikipedia.org/wiki/PID_controller)

How does it apply to the open drone?
------------------------------------

A Kalman filter designed to run on an STM32F429 dev board.
Simulates a rocket altitude estimator given accelerometer
and altimeter measurements. The accelerometer serves as 
a control input to the Kalman filter, while the altimeter
is used for the correction step. We assume constant
acceleration dynamics.

Expected output:
```
Q Matrix:
[[        0.00000,         0.00000],
 [        0.00000,         0.00001]]
Kalman initialization status: 0
xHat Matrix:
[[        0.30625],
 [        2.45000]]
P Matrix:
[[      531.25000,       125.00000],
 [      125.00000,       500.00000]]
Step 1 predict status: 0
xHat Matrix:
[[      -18.35168],
 [       -1.94010]]
P Matrix:
[[      228.18793,        53.69128],
 [       53.69128,       483.22147]]
Step 1 correct status: 0
xHat Matrix:
[[      -17.90171],
 [        5.53990]]
P Matrix:
[[      285.23492,       174.49664],
 [      174.49664,       483.22147]]
Step 2 predict status: 0
xHat Matrix:
[[      -15.07044],
 [        7.27197]]
P Matrix:
[[      166.50345,       101.86092],
 [      101.86092,       438.78549]]
Step 2 correct status: 0
```

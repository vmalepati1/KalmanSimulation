A Kalman filter designed to run on an STM32F429 dev board.
Simulates a rocket altitude estimator given accelerometer
and altimeter measurements. The accelerometer serves as 
a control input to the Kalman filter, while the altimeter
is used for the correction step. We assume constant
acceleration dynamics.
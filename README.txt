This flight controller is inspired by Carbon Aeronautics. 
The main difference is the stability improvement by correcting the coordinate transformation to comply with the definition of Euler angles. 
The transformation is non-linear, so in theory I should have used the Extended Kalman Filter (EKF). However, during testing I came to the conclusion that using the estimate of the linear KF 
without bothering with the Jacobian still delivers very stable control. 
If I ever get over being lazy I'll get to the EKF. 
# 3D Quadcopter Estimator 

## Project Instruction

This project is the extended project of the previous project which only incorporates the controls. In this project, I've learned and implemented the 3D estimator for quadcopters with extended Kalman Filter (EKF). The development setup and the project instruction are provided in the [instruction](./Instruction.md).

So, if the control of a quadcopter exists, why do we bother about the estimator? We need the 3D estimator for these two issues:

1. to deal with the localization or the exact position of the quadcopter since without the estimator we will not get a reliable localization
2. to deal with the sensor measurement values which taken from the quadcopter (GPS, Accelerometer, magnetometer). 

A summary of the mathematical calculation of the EKF estimator can be found in [https://www.overleaf.com/read/vymfngphcccj#/54894644/](https://www.overleaf.com/read/vymfngphcccj#/54894644/).

## Implementation Details

In addition to the previous project, Udacity added more 6 scenarios, where I can test my estimator. Thus, I'll describe only these six new scenario

### Scenario 6: Sensor Noise

In scenario 6, I have to calculate the standard deviation of the given GPS and accelerometer sensor on the quadcopter. During the quadcopter stays on the air, the sensor values are recorded. For this scenario I wrote a Python script to calculate the standard deviation of the both sensors. The script is located in [config/task1.py](./config/task1.py). I utilize the Python csv and numpy libraries which have been provided in the Conda starter-kit. The results are:

    MeasuredStdDev_GPSPosXY = 0.71325
    MeasuredStdDev_AccelXY = 0.497225


## Scenario 7: Attitude Estimation

In this scenario, I need to implement the function `UpdateFromIMU()` which contains a complementary filter-type attitude filter. First, I take the Euler angles and convert them to quartenions to get the quadcopter representation in intertial frame. Then, I integrate the body rate from the gyro in the quartenions to get the updated pitch, roll, and yaw values.

These are the implemented code section:

```
void QuadEstimatorEKF::UpdateFromIMU(V3F accel, V3F gyro)
{
  ...
  Quaternion<float> attitude_quat = Quaternion<float>::FromEuler123_RPY(rollEst, pitchEst, ekfState(6));

  attitude_quat.IntegrateBodyRate(gyro, dtIMU);

  float predictedPitch = attitude_quat.Pitch();
  float predictedRoll = attitude_quat.Roll();
  ekfState(6) = attitude_quat.Yaw();

  // normalize yaw to -pi .. pi
  if (ekfState(6) > F_PI) ekfState(6) -= 2.f * F_PI;
  if (ekfState(6) < -F_PI) ekfState(6) += 2.f * F_PI;
  ...
}
```
![scenario7][scenario7]

## Scenario 8: Prediction Step

The next step is to implement the prediction step of the EKF estimator. For this prediction step we need to calculate the transitional function as `PredictState()`, the partial derivative of the rotation matrix from body frame to global frame (`Rbg`) with respect to yaw, called `RbgPrime`, and the EKF prediction as `Predict()`.

The transitional function calculate the first six states which are the pose and the velocities as described in [Eq. 36 in the overleaf summary](https://www.overleaf.com/read/vymfngphcccj#/54894644/):

```
  predictedState(0) = predictedState(0) + curState(3) * dt;
  predictedState(1) = predictedState(1) + curState(4) * dt;
  predictedState(2) = predictedState(2) + curState(5) * dt;

  // find the global acceleration to predict the next velocity value
  V3F accel_global = attitude.Rotate_BtoI(accel);
  accel_global.z -= static_cast<float>(CONST_GRAVITY);

  predictedState(3) = predictedState(3) + accel_global.x * dt;
  predictedState(4) = predictedState(4) + accel_global.y * dt;
  predictedState(5) = predictedState(5) - accel_global.z * dt;
```
Since the yaw integral is already done in the IMU update, I don't need to update the yaw again.

Next, I implement the Eq. 52 for the the partial derivative of the rotation matrix. Finally, I need to implement the Jacobian matrix from the Eq. 51 in the `Predict()` function.

Below is the result of the simulation

![scenario8][scenario8]

## Scenario 9: Covariance Tuning

In this scenario I need to tune the standard deviation of the noise covariance matrix. My result is:

    QPosXYStd = .001
    QPosZStd = .05

![scenario9][scenario9]

## Scenario 10: Magnetometer Update

From the initial scenario, we can see that the estimate yaw is drifting away from the real value, especially in case where the quadcopter does a turn right or left movement. Below figure shows this drifting and the increasing error of the estimate yaw:

![drift][drift]

In order to avoid this drift, we need to update the yaw from the magnetometer measurement value. The section 7.3.2 of the mention summary provides the mathematical formula to implement the manetometer. Based on the Eq. 56, 57, 58, I implement those equations like below:

```
hPrime(0,6) = 1.;  //eq. 58

zFromX(0) = ekfState(6);
float dyaw = z(0) - zFromX(0);
// normalize the yaw angle
if (dyaw > F_PI)
{
zFromX(0) = zFromX(0) + 2. * F_PI;
}
else if (dyaw < -F_PI)
{
zFromX(0) = zFromX(0) - 2. * F_PI;
}

```

![scenario10][scenario10]



## Scenario 11 GPS Update with Custom Controller

This scenario allows me to test the localization with the GPS update, which is described in the setion 7.3.1 of [the summary handout](https://www.overleaf.com/read/vymfngphcccj#/54894644/). In the code, I implement the partial derivative of the measurement model `h(x)`, called `hPrime`. `hPrime` is a 6x7 matrix which contains 5x5 identity matrix.

These are the implemented code in the function `void QuadEstimatorEKF::UpdateFromGPS(V3F pos, V3F vel)`:

```
  ...
  hPrime(0,0) = 1.;
  hPrime(1,1) = 1.;
  hPrime(2,2) = 1.;
  hPrime(3,3) = 1.;
  hPrime(4,4) = 1.;
  hPrime(5,5) = 1.;

  // Assign the EKF state to zFromX without the yaw
  for(auto idx = 0; idx < 6; idx++)
  {
    zFromX(idx) = ekfState(idx);
  }

  Update(z, hPrime, R_GPS, zFromX);
```


![scenario11][scenario11]


[//]: # (References)
[drift]: ./images/task4_drift.png
[scenario7]: ./images/scenario7.gif
[scenario8]: ./images/scenario8.gif
[scenario9]: ./images/scenario9.gif
[scenario10]: ./images/scenario10.gif
[scenario11]: ./images/scenario11.gif


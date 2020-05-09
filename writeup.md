# Writeup - Project 3, Building a Controller
## Udacity Flying Car Nanodegree

## Overview
The project involved implementing a cascaded flight controller for a quadcopter drone.

<image src="assets/cascaded_controller_detailed.png" height="250">  

*[Image from [Udacity](www.udacity.com)]*

## Imlementation
### Generate motor commands
Tasks to implement the `QuadControl::GenerateMotorCommands` class method in `QuadCOntrol.cpp`.

The known rotor physics equations are:  

<img src="https://latex.codecogs.com/gif.latex?\begin{align*}&space;\tau_x&space;&=&space;(F_1-F_2-F_3&plus;F_4)l&space;\\&space;\tau_y&space;&=&space;(F_1&plus;F_2-F_3-F_4)l&space;\\&space;\tau_z&space;&=&space;\tau_1&plus;\tau_2&plus;\tau_3&plus;\tau_4&space;\\&space;F_c&space;&=&space;F_1&space;&plus;&space;F_2&space;&plus;&space;F_3&space;&plus;&space;F_4&space;\\&space;\end{align*}" title="\begin{align*} \tau_x &= (F_1-F_2-F_3+F_4)l \\ \tau_y &= (F_1+F_2-F_3-F_4)l \\ \tau_z &= \tau_1+\tau_2+\tau_3+\tau_4 \\ F_c &= F_1 + F_2 + F_3 + F_4 \\ \end{align*}" />

The thrust, F, can be written in terms of the torque, tau based upon an empirical relationship:

<img src="https://latex.codecogs.com/svg.latex?\tau&space;=&space;\kappa&space;F" title="\tau = \kappa F" />

The equations can then be written in matrix form:

<img src="https://latex.codecogs.com/svg.latex?\begin{bmatrix}&space;\tau_{x}&space;\\&space;\tau_{y}&space;\\&space;\tau_{z}&space;\\&space;F_{c}&space;\end{bmatrix}&space;=&space;\begin{bmatrix}&space;l&space;&&space;-l&space;&&space;-l&space;&&space;l&space;\\&space;l&space;&&space;l&space;&&space;-l&space;&&space;-l&space;\\&space;-\kappa&space;&&space;\kappa&space;&&space;-\kappa&space;&&space;\kappa&space;\\&space;1&space;&&space;1&space;&&space;1&space;&&space;1&space;\\&space;\end{bmatrix}&space;\begin{bmatrix}&space;F_1&space;\\&space;F_2&space;\\&space;F_3&space;\\&space;F_4&space;\end{bmatrix}" title="\begin{bmatrix} \tau_{x} \\ \tau_{y} \\ \tau_{z} \\ F_{c} \end{bmatrix} = \begin{bmatrix} l & -l & -l & l \\ l & l & -l & -l \\ -\kappa & \kappa & -\kappa & \kappa \\ 1 & 1 & 1 & 1 \\ \end{bmatrix} \begin{bmatrix} F_1 \\ F_2 \\ F_3 \\ F_4 \end{bmatrix}" />

Solving for the individual thrusts we get:

<img src="https://latex.codecogs.com/svg.latex?\begin{bmatrix}&space;F_1&space;\\&space;F_2&space;\\&space;F_3&space;\\&space;F_4&space;\end{bmatrix}&space;=&space;\begin{bmatrix}\frac{F_{c}}{4}&space;&plus;&space;\frac{\tau_{x}}{4&space;l}&space;&plus;&space;\frac{\tau_{y}}{4&space;l}&space;-&space;\frac{\tau_{z}}{4&space;\kappa}\\\frac{F_{c}}{4}&space;-&space;\frac{\tau_{x}}{4&space;l}&space;&plus;&space;\frac{\tau_{y}}{4&space;l}&space;&plus;&space;\frac{\tau_{z}}{4&space;\kappa}\\\frac{F_{c}}{4}&space;-&space;\frac{\tau_{x}}{4&space;l}&space;-&space;\frac{\tau_{y}}{4&space;l}&space;-&space;\frac{\tau_{z}}{4&space;\kappa}\\\frac{F_{c}}{4}&space;&plus;&space;\frac{\tau_{x}}{4&space;l}&space;-&space;\frac{\tau_{y}}{4&space;l}&space;&plus;&space;\frac{\tau_{z}}{4&space;\kappa}\end{bmatrix}" title="\begin{bmatrix} F_1 \\ F_2 \\ F_3 \\ F_4 \end{bmatrix} = \begin{bmatrix}\frac{F_{c}}{4} + \frac{\tau_{x}}{4 l} + \frac{\tau_{y}}{4 l} - \frac{\tau_{z}}{4 \kappa}\\\frac{F_{c}}{4} - \frac{\tau_{x}}{4 l} + \frac{\tau_{y}}{4 l} + \frac{\tau_{z}}{4 \kappa}\\\frac{F_{c}}{4} - \frac{\tau_{x}}{4 l} - \frac{\tau_{y}}{4 l} - \frac{\tau_{z}}{4 \kappa}\\\frac{F_{c}}{4} + \frac{\tau_{x}}{4 l} - \frac{\tau_{y}}{4 l} + \frac{\tau_{z}}{4 \kappa}\end{bmatrix}" />

The code then for the motor commands is written as follows noting the ordering of the motors index 2 & 3 are different from the derived formulas:

```cpp
VehicleCommand QuadControl::GenerateMotorCommands(float collThrustCmd, V3F momentCmd)
{
    // Moment distance is L / srt(2) as L is distance from centre to motor
    float d = L / 1.4142135623730951;
    // Calculate repeated terms for speed
    float x_term = momentCmd.x / d;
    float y_term = momentCmd.y / d;
    float z_term = - momentCmd.z / kappa;
    cmd.desiredThrustsN[0] = (collThrustCmd + x_term + y_term + z_term) / 4.f; // front left
    cmd.desiredThrustsN[1] = (collThrustCmd - x_term + y_term - z_term) / 4.f; // front right
    cmd.desiredThrustsN[3] = (collThrustCmd - x_term - y_term + z_term) / 4.f; // rear left
    cmd.desiredThrustsN[2] = (collThrustCmd + x_term - y_term - z_term) / 4.f; // rear right

    return cmd;
}
```

### Body rate control
Tasks to implement the `QuadControl::BodyRateControl` class method in `QuadCOntrol.cpp`.

For the body rate control a simple proportional controller is implemented to calculate an angular accelertion command `pqr_dot`. To convert this into the required moment command we use the relationship:

<img src="https://latex.codecogs.com/svg.latex?\tau&space;=&space;I&space;\alpha" title="\tau = I \alpha" />

```cpp
V3F QuadControl::BodyRateControl(V3F pqrCmd, V3F pqr)
{
    V3F momentCmd;
    V3F pqrError = pqrCmd - pqr;
    V3F pqr_dot = kpPQR * pqrError;
    V3F moi = V3F {Ixx, Iyy, Izz};
    momentCmd = moi * pqr_dot;

    return momentCmd;
}
```

### Roll and pitch control
Tasks to implement the `QuadControl::RollPitchControl` class method in `QuadCOntrol.cpp`.

Given a rotation matrix, R, to transofrm from the body frame to the world/inertial frame the accelerations in the body frame are given as (noting that f is collective thrust divided by mass):

<img src="https://latex.codecogs.com/svg.latex?\begin{bmatrix}&space;\ddot{x}_x&space;\\&space;\ddot{x}_y&space;\\&space;\ddot{x}_z&space;\end{bmatrix}&space;=&space;\begin{bmatrix}&space;R_{00}&space;&&space;R_{01}&space;&&space;R_{02}&space;\\&space;R_{10}&space;&&space;R_{11}&space;&&space;R_{12}&space;\\&space;R_{20}&space;&&space;R_{21}&space;&&space;R_{22}&space;\\&space;\end{bmatrix}&space;\begin{bmatrix}&space;0&space;\\&space;0&space;\\&space;f_c&space;\end{bmatrix}&space;-&space;\begin{bmatrix}&space;0&space;\\&space;0&space;\\&space;g&space;\end{bmatrix}" title="\begin{bmatrix} \ddot{x}_x \\ \ddot{x}_y \\ \ddot{x}_z \end{bmatrix} = \begin{bmatrix} R_{00} & R_{01} & R_{02} \\ R_{10} & R_{11} & R_{12} \\ R_{20} & R_{21} & R_{22} \\ \end{bmatrix} \begin{bmatrix} 0 \\ 0 \\ f_c \end{bmatrix} - \begin{bmatrix} 0 \\ 0 \\ g \end{bmatrix}" />

Multiplying this out we see which values of the rotation matrix affect the body acceleration. We can define these rotation matrix elements by a value, b:

<img src="https://latex.codecogs.com/svg.latex?\begin{bmatrix}&space;\ddot{x}_x&space;\\&space;\ddot{x}_y&space;\\&space;\ddot{x}_z&space;\end{bmatrix}&space;=&space;\begin{bmatrix}R_{02}&space;f_{c}\\R_{12}&space;f_{c}\\R_{22}&space;f_{c}&space;-&space;g&space;\end{bmatrix}=&space;\begin{bmatrix}b^x&space;f_{c}&space;\\&space;b^y&space;f_{c}&space;\\&space;b^z&space;f_{c}&space;-&space;g&space;\end{bmatrix}" title="\begin{bmatrix} \ddot{x}_x \\ \ddot{x}_y \\ \ddot{x}_z \end{bmatrix} = \begin{bmatrix}R_{02} f_{c}\\R_{12} f_{c}\\R_{22} f_{c} - g \end{bmatrix}= \begin{bmatrix}b^x f_{c} \\ b^y f_{c} \\ b^z f_{c} - g \end{bmatrix}" />

These "control knobs" can then be adjusted:

<img src="https://latex.codecogs.com/svg.latex?\begin{bmatrix}&space;\dot{b}^x_c&space;\\&space;\dot{b}^y_c&space;\end{bmatrix}&space;=&space;\begin{bmatrix}k_p(b^x_c&space;-&space;b^x_a)&space;\\&space;k_p(b^y_c&space;-&space;b^y_a)&space;\end{bmatrix}" title="\begin{bmatrix} \dot{b}^x_c \\ \dot{b}^y_c \end{bmatrix} = \begin{bmatrix}k_p(b^x_c - b^x_a) \\ k_p(b^y_c - b^y_a) \end{bmatrix}" />

The body rates, p & q are calculated by commanding vaues of the rotation matrix:

<img src="https://latex.codecogs.com/svg.latex?\begin{bmatrix}&space;p_c&space;\\&space;q_c&space;\\&space;\end{bmatrix}&space;=&space;\frac{1}{R_{22}}\begin{bmatrix}&space;R_{10}&space;&&space;-R_{00}&space;\\&space;R_{11}&space;&&space;-R_{01}&space;\end{bmatrix}&space;\begin{bmatrix}&space;\dot{b}^x_c&space;\\&space;\dot{b}^y_c&space;\end{bmatrix}" title="\begin{bmatrix} p_c \\ q_c \\ \end{bmatrix} = \frac{1}{R_{22}}\begin{bmatrix} R_{10} & -R_{00} \\ R_{11} & -R_{01} \end{bmatrix} \begin{bmatrix} \dot{b}^x_c \\ \dot{b}^y_c \end{bmatrix}" />

The C++ implementation is then given as:

```cpp
V3F QuadControl::RollPitchControl(V3F accelCmd, Quaternion<float> attitude, float collThrustCmd)
{
    V3F pqrCmd;
    Mat3x3F R = attitude.RotationMatrix_IwrtB();

    if (collThrustCmd > 0) {
        float R02_target = CONSTRAIN(- accelCmd.x * mass / collThrustCmd, -maxTiltAngle, maxTiltAngle);
        float R12_target = CONSTRAIN(- accelCmd.y * mass / collThrustCmd, -maxTiltAngle, maxTiltAngle);

        float b_dot_c_x = kpBank * (R02_target - R(0, 2));
        float b_dot_c_y = kpBank * (R12_target - R(1, 2));
        pqrCmd.x = (R(1,0) * b_dot_c_x - R(0,0) * b_dot_c_y) / R(2,2);
        pqrCmd.y = (R(1,1) * b_dot_c_x - R(0,1) * b_dot_c_y) / R(2,2);
    }

    return pqrCmd;
}
```

### Altitude control
Tasks to implement the `QuadControl::AltitiudeControl` class method in `QuadCOntrol.cpp`.

Altitude control is implemented using:
* PI control using postion
* P control using velocity
* Feedforward control using z acceleration

From above we had defined the body accelerations as follows:

<img src="https://latex.codecogs.com/svg.latex?\begin{bmatrix}&space;\ddot{x}_x&space;\\&space;\ddot{x}_y&space;\\&space;\ddot{x}_z&space;\end{bmatrix}&space;=&space;\begin{bmatrix}R_{02}&space;f_{c}\\R_{12}&space;f_{c}\\R_{22}&space;f_{c}&space;-&space;g&space;\end{bmatrix}" title="\begin{bmatrix} \ddot{x}_x \\ \ddot{x}_y \\ \ddot{x}_z \end{bmatrix} = \begin{bmatrix}R_{02} f_{c}\\R_{12} f_{c}\\R_{22} f_{c} - g \end{bmatrix}" />

This allows us to convert our acceleration to a thrust command.

```cpp
float QuadControl::AltitudeControl(float posZCmd, float velZCmd, float posZ, float velZ, Quaternion<float> attitude, float accelZCmd, float dt)
{
    Mat3x3F R = attitude.RotationMatrix_IwrtB();
  
    integratedAltitudeError += (posZCmd - posZ) * dt;

    float accelZCtrl = kpPosZ * (posZCmd - posZ) + \
                       kpVelZ * (velZCmd - velZ) + \
                       KiPosZ * integratedAltitudeError + \
                       accelZCmd;
    // Limit acceleration control to within limits
    accelZCtrl = CONSTRAIN(accelZCtrl, -maxAscentRate/dt, maxDescentRate/dt);
    // Convert to thrust in body frame (transform and multiply by mass)
    float thrust = (-accelZCtrl + 9.81f) * mass / R(2,2);  // -ve accel for +ve thrust

    return thrust;
}
```

### Lateral position control
Tasks to implement the `QuadControl::LateralPositionControl` class method in `QuadCOntrol.cpp`.

Lateral position control is implemented using:
* P control using postion
* P control using velocity
* Feedforward control using z acceleration

The commanded speed and accelerations are limited by `maxSpeedXY` and `maxAccelXY` by normalising the vectors. The return value is just this acceleration.

```cpp
V3F QuadControl::LateralPositionControl(V3F posCmd, V3F velCmd, V3F pos, V3F vel, V3F accelCmdFF)
{
    // make sure we don't have any incoming z-component
    accelCmdFF.z = 0;
    velCmd.z = 0;
    posCmd.z = pos.z;

    // Limit velocity command
    float mag_velCmd = velCmd.magXY();
    if (mag_velCmd > maxSpeedXY) {
        velCmd = velCmd * maxSpeedXY / mag_velCmd;  // Create unit vector then multiply by max magnitude 
    }

    V3F accelCmd = kpPosXY * (posCmd - pos) + kpVelXY * (velCmd - vel) + accelCmdFF;
    // Limit output
    float mag_accelCmd = accelCmd.magXY();
    if (mag_accelCmd > maxAccelXY) {
        accelCmd = accelCmd * maxAccelXY / mag_accelCmd;
    }
    accelCmd.z = 0;
    
    return accelCmd;
}
```

### Yaw control
Tasks to implement the `QuadControl::YawControl` class method in `QuadCOntrol.cpp`.

Yaw control is implemented using a simple proportional controller. The only compleity comes from calculating the correct magnitue and direction of yaw based upon supposed possible input angles between -&#8734; and &#8734;. To do this all the angles are converted to the range 0 - &pi; and then if the difference between the angles is greater than &pi; the direction is changed.

```cpp
float QuadControl::YawControl(float yawCmd, float yaw)
{
    if (yawCmd < 0) {  // Convert -ve angle to +ve between 0 & 2pi
        yawCmd  = fmodf(-yawCmd, 2 * F_PI);
        yawCmd = 2 * F_PI - yawCmd;
    } else {  // Constrain +ve inputs to -0 & 2pi
        yawCmd  = fmodf(yawCmd, 2 * F_PI);
    }
    if (yaw < 0) {
        yaw  = fmodf(-yaw, 2 * F_PI);
        yaw = 2 * F_PI - yaw;
    } else {
        yaw  = fmodf(yaw, 2 * F_PI);
    }
    // If angle difference is greater than pi then go in the other direction (short way round)
    float delta_yaw = yawCmd - yaw;
    if (delta_yaw > F_PI) {
        delta_yaw -= F_PI;
    } else if (delta_yaw < -F_PI) {
        delta_yaw += 2 * F_PI;
    }
    // std::cout << delta_yaw << std::endl;
    float yawRateCmd = kpYaw * delta_yaw;

    return yawRateCmd;
}
```
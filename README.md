# **Model Predictive Control**

---


**Model Predictive Control Project**

The goals / steps of this project are the following:

* Implementation of the Model Predictive Control method in C++ to control the actuator inputs of a car.
* Application of the method on the term-2 simulator -complete a whole lap successfully.

[//]: # (Image References)

[image1]: ./outputs/RMSE_Unscented.png "Undistorted"
[image2]: ./outputs/YawAngle.png "Undistorted"
[image3]: ./outputs/YawRate.png "Undistorted"
[image4]: ./outputs/Lidar_NIS.png "Undistorted"
[image5]: ./outputs/Radar_NIS.png "Undistorted"

## Rubric Points

Here I will consider the [rubric points](https://review.udacity.com/#!/rubrics/896/view) individually and describe how I addressed each point in my implementation.

---


#### 1. Algorithm structure and processing flow

The algorithm is divided in the following source code files located in `/src`:

`main.cpp` -> Main routine to read the simulator data and call the MPC `Solve` function, which represents the routine where the best trajectory is calculated.

`MPC.cpp` -> Contains the class MPC. It defines the routine `Solve`, from where ipopt runs to find locally optimal values. Additionally, the `FG_eval` function is defined. It includes the `operator` routine, which implements the cost function and sets the model constraints.

#### 2. Model

The model consists of an optimizer with a set of six equations to update the vehicle state as well as the Cross Track and orientation errors. Four equations correspond to the state (`x` and `y` positions, velocity and orientation) and the other two to both errors. The equations use as inputs the current state, the current actuator inputs and the polynomial that represents the reference trajectory.

The model sets actuator input constraints. In this case, the default constraints have been selected. This means that the steering angle is within a range of `[-25, 25]°` and the acceleration within `[-1, 1] m/s2`.

Finally it is necessary to specify a cost function that defines how each state component affects the optimization calculation. The magnitude of CTE, orientation, velocity and actuator inputs penalize the calculation. Additionally, the difference between consecutive cycles regarding steering angle and acceleration has been also considered. In order to avoid significant turns, it has been decided that the steering angle difference between cycles is the component that penalizes most the cost function (factor `50000000` applied).

#### 3. Timestep Length and Elapsed Duration

The optimization is performed along a prediction horizon modelled through the number of timesteps `N` and the elapsed duration `dt`. It has been found that an approximate horizon of `N * dt = 0.85s` works well for the prediction of the trajectory. It was also experienced that the higher the velocity, the fewer number of timesteps are needed for a good optimization. The following values were found to give a good performance:

| Velocity (mph)  | N             | dt (s)  |
| -------------   | ------------- | -----   |
| 40              | 25            | 0.0340  |
| 50              | 25            | 0.0340  |
| 60              | 15            | 0.0583  |
| 70              | 14            | 0.0602  |
| 80              | 14            | 0.0602  |

#### 4. Polynomial fitting and MPC preprocessing

As previously mentioned, a polynomial modeling the reference trajectory acts as input of the model. In order to calculate it, the simulator provides six points of the trajectory. These are introduced to `polyfit`, that calculates the coefficients. In this case it is possible to tune the polynomial degree, so a third-degree polynomial has been finally chosen, since it offered the best results.

It is important to mention that, as specified in the project Q&A, in order to get the method working with the simulator, it is necessary to convert these points to the vehicle coordinate system. The conversion is done through the equations implemented in the `for` loop of the line `96` of `main.cpp`. After the conversion, the `x` and `y` coordinates as well as the orientation should be considered as `0` for the calculation of `epsi`, and also the vehicle coordinate system values are introduced as state components to the MPC.

#### 5. MPC with latency

The model should handle a latency of 100ms. In this case, its application is achieved with the `sleep_for` method used in the initial code version. This instruction is located in the line `167` of the final version of `main.cpp`. It is also necessary to make predictions of the state 100ms in the future and pass it to solver. For the prediction the kinematic equations are used in the line `122` of `main.cpp`

#### 6. Results

The car drives successfully and safely during a whole lap with a velocity of `60mph`, with `N` equal to `15` and `dt` to `0.0583s`. Higher speeds (`70mph`, and eventually `80mph` when the acceleration magnitude is not included in the cost function and the penalization of steering angle difference between cycles is reduced) have also been achieved, but with a more abrupt driving and popping up onto ledges.

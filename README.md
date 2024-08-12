# MATLAB_Centre_Back_Steering_Wheel
Centre Back Steering Wheel Behaviour using Proportional Controller modeled in MATLAB. The steering wheel is assumed to be a rotational mass spring damper system.

# What is Centre - back positioning of a steering wheel?
The "center-back position" of a steering wheel refers to a feature where the steering wheel automatically returns to its neutral or centered position after the driver releases their grip on it. This feature is essential for maintaining vehicle stability and control, especially during turns or maneuvers.

In traditional mechanical steering systems, the center-back position is achieved through the geometry of the steering linkage and the steering column. When the driver turns the wheel, forces are applied to the steering mechanism, causing the wheels to rotate. Once the driver releases the steering wheel, the system's geometry and alignment naturally return the wheels and the steering wheel itself to the center position.

In modern vehicles equipped with power steering, the center-back position is often assisted by hydraulic or electrically controlled systems. These systems apply a force to assist the driver in returning the steering wheel to the center position after turning. This assistance not only enhances driver comfort but also contributes to vehicle stability and control.

The center-back position of the steering wheel as seen in figure 2.3 is crucial for maintaining predictable handling characteristics and ensuring that the vehicle responds predictably to driver inputs. It plays a significant role in vehicle safety and is an essential aspect of steering system design and calibration.

![image](https://github.com/user-attachments/assets/a69ea844-210f-4134-864f-4be12b272c07)


# Simulation of a rotational mass-spring-damper system with a proportional controller
1. System Parameters Definition: Define the parameters of the rotational mass-spring damper system such as moment of inertia (J), damping coefficient (b), spring constant (K), and desired position (setpoint).


2. Controller Parameters Definition: Define the parameters of the proportional controller, in this case, only the proportional gain (Kp).

  
3. Simulation Parameters Definition: Set the parameters for simulation such as time step (timestep) and total simulation time (total_time).

  
4. Initialization: Initialize arrays to store the angular position (theta) and angular velocity (omega) over time.

   
5. Simulation Loop: Iterate over each time step from 0 to total_time with a time step of timestep. Within each iteration:
   
a) Compute Error: Calculate the error by subtracting the current position (theta) from the desired position (setpoint).


b) Compute Control Action: Compute the control action using proportional control, which is simply the proportional gain (Kp) multiplied by the error calculated in the previous step.


c) Compute Acceleration: Calculate the angular acceleration (alpha) using the equation of motion for the system, which involves the control action, damping, spring, and inertia.


d) Update Velocity and Position: Update the angular velocity (omega) and angular position (theta) using Euler's method, which is a simple numerical integration method.


6. Plotting: After the simulation loop, plot the results of the simulation, showing how the angular position and angular velocity change over time


This algorithm essentially simulates the behaviour of the rotational mass-spring-damper system with a proportional controller by iteratively updating the system's state (angular position and velocity) over time based on the defined parameters and the control action computed at each time step.


# Proportional Controller for a DC Motor

1. Motor Parameters Definition: Define the parameters of the DC motor such as armature resistance (R), armature inductance (L), motor constant (K), moment of inertia of the rotor (J), damping coefficient (b), and motor constant related to back electromotive force (Ke).

   
2. Controller Parameters Definition: Set the proportional gain (Kp) for the proportional controller.

   
3. Setpoint Definition: Define the desired position (theta_setpoint) for the motor's angular position.

   
4. Simulation Parameters Definition: Set parameters for simulation including the time step (timestep) and total simulation time (total_time).

   
5. Initialization: Initialize arrays to store the angular position (theta), angular velocity (omega), and applied voltage (V) over time.

    
6. Simulation Loop: Iterate over each time step from 0 to total_time with a time step of timestep. Within each iteration:

a) Compute Error: Calculate the error by subtracting the current position (theta) from the desired position (theta_setpoint).


b) Compute Control Action: Compute the control action using proportional control, which is the proportional gain (Kp) multiplied by the error calculated in the previous step.


c) Compute Back EMF: Calculate the back electromotive force (EMF) based on the motor's angular velocity (omega) and the motor constant related to back electromotive force (Ke).


d) Compute Applied Voltage: Compute the applied voltage to the motor by adding the control action to the back EMF.


e) Compute Torque: Calculate the torque generated by the motor using the applied voltage and the motor constant (K).


f) Compute Acceleration: Calculate the angular acceleration (alpha) using the equation of motion for the motor, which involves torque, damping, and inertia.


g) Update Velocity and Position: Update the angular velocity (omega) and angular position (theta) using Euler's method.


7. Plotting: Plot the results of the simulation, showing the motor's angular position, angular velocity, and applied voltage over time.


# Centre - back Steering wheel behavior in MATLAB

1. System Parameters Definition: Define parameters for the rotational system such as moment of inertia (J), damping coefficient (b), spring constant (K), and desired position (setpoint).

   
2. DC Motor Parameters Definition: Define parameters for the DC motor such as voltage rating (V_rating), power rating (P_rating), and maximum speed (RPM_max). Then, calculate motor constants including the back electromotive force constant (Km), voltage constant (Ke), and armature resistance (R) based on the provided ratings.


3. Initial Condition: Set initial conditions for the rotational system including initial position (initial_position) and initial velocity (initial_velocity).

   
4. Simulation Parameters Definition: Set parameters for simulation such as the time step (timestep) and total simulation time (total_time).

   
5. Transfer Function Definition: Define the transfer function (sys_tf) of the combined system using the provided parameters.

  
6. Plotting Impulse Response: Plot the impulse response of the system using the defined transfer function.


7. Plotting Step Response: Plot the step response of the system using the defined transfer function.

    
8. Compute Step Response Metrics: Compute various metrics such as settling time, overshoot, and rise time from the step response.

    
9. Plotting Bode Plot: Plot the Bode plot of the system using the defined transfer function.

    
10. Plotting Pole-Zero Plot: Plot the pole-zero plot of the system using the defined transfer function.

    
11. Proportional Controller Definition: Define the proportional gain (Kp) for the proportional controller.

    
12. Initialization: Initialize arrays to store the angular position (theta), angular velocity (omega), and applied voltage (V) over time.

    
13. Initial Condition Setup: Set the initial position and velocity of the rotational system.

14. Simulation Loop: Iterate over each time step from 0 to total_time with a time step of timestep. Within each iteration:
    
    
a) Compute Error: Calculate the error by subtracting the current position (theta) from the desired position (setpoint).


b) Compute Control Action: Compute the control action using proportional control, which is the proportional gain (Kp) multiplied by the error calculated in the previous step.


c) Compute Applied Voltage: Compute the applied voltage to the motor based on the control action.


d) Compute Torque: Calculate the torque generated by the motor using the applied voltage and the motor constant (Km).


e) Compute Acceleration: Calculate the angular acceleration (alpha) using the equation of motion for the motor.


f) Update Velocity and Position: Update the angular velocity (omega) and angular position (theta) using Euler's method.


15. Plotting Results: Plot the simulation results including angular position, angular velocity, and applied voltage over time.


![Poster idea 1](https://github.com/user-attachments/assets/780701a4-5571-42e9-8977-30e8a1bde958)


# Results of Centre - back Steering wheel behavior in MATLAB

![image](https://github.com/user-attachments/assets/c81d6a8d-6f1e-4c3f-b915-7cb080eb024a)


![image](https://github.com/user-attachments/assets/c218429c-ee19-4ce8-91b8-6296507a30df)




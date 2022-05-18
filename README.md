# APL-for-Velocity-Optimization-Sealing-task-
Creation of a varying velocity reference profile of different paths for the sealing task. The optimal velocity value at each point of the path is created with the help of Active Preference Learning Algorithm and this data is used to create velocity profile for different paths of execution.

Experimental Setup-Flange for the Franka Emika Panda Robot and the arduino code from:https://github.com/emarescotti/VelocityPlanning_DMP_FL

APL algorithm from:   (XXXX)

Instructions:

-APL task execution: Use APL algorithm to generate execution velocity values. Use this value in the file curve_execution(forAPL).py to execute in robot. Then the user judges the output by comparison and with the help of the APL Algorithm should enter a new value of velocity to curve_execution(forAPL).py file. In this loop,  for each radius curve, velocity can be optimized.
(the already created APL data is in the file curv_vel_data(APL).py).

-Acceleration_constraint1: Variable Velocity generation folder has  different paths to choose from. Execute a path from this and with the created .npz file run Curvature_analysis_constraints2.py file to run the final execution

-Non_linear_optimization: with the velocity values(in non_linear_optimization_velocity_values folder) created with help of (XXXX), run the new_optimization_acc_constarint2.py.

(based on the master thesis work,attached is a pdf of the thesis)

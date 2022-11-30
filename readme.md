## Trajectory Tracking PD controller with quaternion based dynamics
### Author:
[Shubham Wani](https://www.linkedin.com/in/shubhamwani/) Nikunj Sanghai

## Implementation
This project implements the following features:
1. Quintic polynomial trajectory generation based on start, waypoint and goal
2. State visualisation based on 3D matplotlib plots
3. PD control on position and attitude error and their relevant graphs
4. Quaternion based dynamics instead of rotation/euler angles

## Video
![Video](https://github.com/shubhamwani376/MPC_Quadcopter/blob/main/Quadrotor.gif)

## Dependencies
```
pip install numpy
pip install matplotlib
pip install opencv-python # only for creating video
```
## Reference
Class notes MAE271D @ UCLA, Fall 22
Relevant files added to folder [Reference](https://github.com/shubhamwani376/MPC_Quadcopter/tree/main/Reference)

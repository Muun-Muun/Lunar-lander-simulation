# Lunar-lander-simulation

We design soft-landing system for lunar-lader to achieve trageted point with minimized fuel.

This system is consist of four sections:

>1. Guidance
>2. Attitude controller
>3. 6-dof simulation

## Guidance
We solve trajectory optimization problem using gradient descent convex optimization method.

<p align="center">
  <img src="https://user-images.githubusercontent.com/52774019/176707984-337374f8-72ea-47f7-8311-5c2f4c63dfef.png" width="30%" height="30%" /><br>
  Convexed-optimized trajectory<br></p> 
<p align="center">
  <img src="https://user-images.githubusercontent.com/52774019/176713087-6f43fd3c-372d-4d3a-a4a8-dd3d009c8416.png" width="30%" height="30%" /><br>
  Comparison of position and velocity with cvxpy<br></p> 
<p align="center">  
  <img src="https://user-images.githubusercontent.com/52774019/176713255-9b1cc19d-c9fe-4a32-830b-cdd55427347f.png" width="30%" height="30%" /><br>
  Thrust magnitude per unit mass<br></p> 

This plot was generated in Python. If you want to check the python code, go to Choi.J.W's Github. @j9bobotw 

## Attitude controller
Our concept lunar-lander's threster is fixed to it's body. so, the body must be aligned with the thrust vector.
To be precise, the attitude should be controlled in the opposite direction of the acceleration calculated in the guidance section.

## 6-dof simulation
Guidance and Attitude controller were verified through 6dof simulation.

<p align="center">
  <img src="https://user-images.githubusercontent.com/52774019/176716171-0bf138fe-0171-4213-9ce8-ce0e8e74b055.png" width="40%" height="40%" /><br>
  Lunar-lander soft landing 6-dof simulation<br></p> 
<p align="center">
  <img src="https://user-images.githubusercontent.com/52774019/176716297-656b8fd0-d3c2-4c6c-a9d3-a30d90b566b6.png" width="40%" height="40%" /><br>
  states of 6-dof simulation<br></p> 

All this processes were done by MATLAB & Python.

#
Thanks for visiting my Github.<br>
If you have any questions, please e-mail me.  
:point_right: yunjung.kim181@gmail.com

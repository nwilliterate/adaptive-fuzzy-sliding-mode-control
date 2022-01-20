# adaptive fuzzy sliding mode control

> Authors:	Seonghyeon Jo(cpsc.seonghyeon@gmail.com)
> 
> Date:		 Jan  20, 2022

This repository is a MATLAB simulation of  **adaptive fuzzy sliding mode control** for robot manipulator.  The robot manipulator uses sawyer 4-dof manipulator and prototype 3-dof manipulator. we compare to simple PID Controller, Sliding Mode Control(SMC), Fuzzy Silding Mode Control(FSMC), Adaptive Fuzzy Sliding Mode Control(AFSMC).

"Adaptive fuzzy sliding mode control using supervisory fuzzy control for
3 DOF planar robot manipulators."

#### Description of robot manipulator model

   In general, the dynamic of a robotic manipulator can be expressed as follows:
<img src=
"https://render.githubusercontent.com/render/math?math=%5Cdisplaystyle+%5CM%28%5Cq%29%5Cddot%7B%5Cq%7D+%2B+%5CC%28%5Cq%2C%5Cdot%7B%5Cq%7D%29%5Cdot%7B%5Cq%7D%2B%5CG%28%5Cq%29%2B%5CF%28%5Cdot%7B%5Cq%7D%29+%3D%5Cboldsymbol%5Ctau+%2B+%5Cboldsymbol%5Ctau_d" 
alt="\M(\q)\ddot{\q} + \C(\q,\dot{\q})\dot{\q}+\G(\q)+\F(\dot{\q}) =\boldsymbol\tau + \boldsymbol\tau_d">
where $q, \dot{q}, \ddot{q}, \tau, \tau_d \in \mathcal{R}^{n}$ denote the joint position vector, velocity vector, acceleration, the vector of control torques, and the vector of disturbance torques, respectively, and $M\in\mathcal{R}^{n\times n}$ denotes the symmetric positive definite inertia matrix, $C \in \mathcal{R}^{n\times n}$ denotes the Coriolis matrix, $G \in \mathcal{R}^{n}$ denotes the gravity vector and $F$ is the friction matrix. 

   Multiplying $M^{-1}(q)$ by both sides in Equation (1), we have 

<img src=
"https://render.githubusercontent.com/render/math?math=%5Cdisplaystyle+%5Cddot%7B%5Cq%7D%3D-%5CM%5E%7B-1%7D%28%5Cq%29%28%5CC%28%5Cq%2C%5Cdot%7B%5Cq%7D%29%5Cdot%7B%5Cq%7D%2B%5CG%28%5Cq%29%2B%5CF%28%5Cdot%7B%5Cq%7D%29-%5Cboldsymbol%5Ctau-%5Cboldsymbol%5Ctau_d+%29" 
alt="\ddot{\q}=-\M^{-1}(\q)(\C(\q,\dot{\q})\dot{\q}+\G(\q)+\F(\dot{\q})-\boldsymbol\tau-\boldsymbol\tau_d )">
ddd
<img src=
"https://render.githubusercontent.com/render/math?math=%5Cdisplaystyle+%5Cbegin%7Balign%2A%7D%0A%5Cs%28t%29%26%3DK_%7Bp%7D%5Ce%28t%29%2BK_%7Bi%7D%5Cint%7B%5Ce%7D%28%5Ctau%29d%5Ctau%2BK_%7Bd%7D%5Cdot%7B%5Ce%7D%28t%29%0A%5Cend%7Balign%2A%7D" 
alt="\begin{align*}
\s(t)&=K_{p}\e(t)+K_{i}\int{\e}(\tau)d\tau+K_{d}\dot{\e}(t)
\end{align*}">

dddd
<img src=
"https://render.githubusercontent.com/render/math?math=%5Cdisplaystyle+%5Cbegin%7Balign%2A%7D%0A%5Cdot%7B%5Cs%7D%28t%29+%26%3D+K_p+%5Cdot%7B%5Ce%7D%28t%29+%2B+K_i+%5Ce%28t%29%2B+K_%7Bd%7D%5Cddot%7B%5Ce%7D%28t%29%5C%5C%0A%26%3D+K_p+%5Cdot%7B%5Ce%7D%28t%29+%2B+K_i+%5Ce%28t%29%2BK_d+%28%5Cddot%7B%5Cq%7D_d%2B%5CM%5E%7B-1%7D%28%5Cq%29%28%5CC%28%5Cq%2C%5Cdot%7B%5Cq%7D%29%5Cdot%7B%5Cq%7D%2B%5CG%28%5Cq%29%2B%5CF%28%5Cdot%7B%5Cq%7D%29-%5Cboldsymbol%5Ctau-%5Cboldsymbol%5Ctau_d%29%5C%5C%0A%5Cend%7Balign%2A%7D" 
alt="\begin{align*}
\dot{\s}(t) &= K_p \dot{\e}(t) + K_i \e(t)+ K_{d}\ddot{\e}(t)\\
&= K_p \dot{\e}(t) + K_i \e(t)+K_d (\ddot{\q}_d+\M^{-1}(\q)(\C(\q,\dot{\q})\dot{\q}+\G(\q)+\F(\dot{\q})-\boldsymbol\tau-\boldsymbol\tau_d)\\
\end{align*}">

Control effort  equivalent control effort
<img src=
"https://render.githubusercontent.com/render/math?math=%5Cdisplaystyle+%5Cbegin%7Balign%2A%7D%0A%5Cu_%7Beq%7D%3D+%28K_d%5CM%5E%7B-1%7D%28%5Cq%29%29%5E%7B-1%7D%28K_p+%5Cdot%7B%5Ce%7D+%2B+K_i+%5Ce%2BK_d+%28%5Cddot%7B%5Cq%7D_d%2B%5CM%5E%7B-1%7D%28%5Cq%29%28%5CC%28%5Cq%2C%5Cdot%7B%5Cq%7D%29%5Cdot%7B%5Cq%7D%2B%5CG%28%5Cq%29%2B%5CF%28%5Cdot%7B%5Cq%7D%29%29%29%0A%5Cend%7Balign%2A%7D" 
alt="\begin{align*}
\u_{eq}= (K_d\M^{-1}(\q))^{-1}(K_p \dot{\e} + K_i \e+K_d (\ddot{\q}_d+\M^{-1}(\q)(\C(\q,\dot{\q})\dot{\q}+\G(\q)+\F(\dot{\q})))
\end{align*}">

## sawyer 4-dof manipulator

### RMSE 

|  Method  | joint 1 | joint 2 | joint 3 | joint 4 |  sum   |
| :------: | :-----: | :-----: | :-----: | :-----: | :----: |
|   PID    | 0.0394  | 0.0339  | 0.0044  | 0.0083  | 0.0860 |
| SMC-SIGN | 0.0078  | 0.0349  | 0.0003  | 0.0039  | 0.0469 |
| SMC-SAT  | 0.0078  | 0.0358  | 0.0004  | 0.0019  | 0.0460 |
|   FSMC   | 0.0111  | 0.0146  | 0.0017  | 0.0043  | 0.0317 |
|  AFSMC   | 0.0105  | 0.0141  | 0.0013  | 0.0042  | 0.0301 |


### Figure

#### pid controller
<img src="./sawyer-4dof-manipulator/fig/joint_position_result_1.png" alt="joint_input_result_1"  width="375" />
<img src="./sawyer-4dof-manipulator/fig/joint_input_result_1.png" alt="joint_input_result_1"  width="375" />
<img src="./sawyer-4dof-manipulator/fig/position_error_result_1.png" alt="joint_input_result_1"  width="375" />

#### smc using sign function
<img src="./sawyer-4dof-manipulator/fig/joint_position_result_2.png" alt="joint_input_result_1"  width="375" />
<img src="./sawyer-4dof-manipulator/fig/joint_input_result_2.png" alt="joint_input_result_1"  width="375" />
<img src="./sawyer-4dof-manipulator/fig/position_error_result_2.png" alt="joint_input_result_1"  width="375" />


#### smc using sat function
<img src="./sawyer-4dof-manipulator/fig/joint_position_result_2.png" alt="joint_input_result_1"  width="375" />
<img src="./sawyer-4dof-manipulator/fig/joint_input_result_2.png" alt="joint_input_result_1"  width="375" />
<img src="./sawyer-4dof-manipulator/fig/position_error_result_2.png" alt="joint_input_result_1"  width="375" />

#### smc using sat function
<img src="./sawyer-4dof-manipulator/fig/joint_position_result_3.png" alt="joint_input_result_1"  width="375" />
<img src="./sawyer-4dof-manipulator/fig/joint_input_result_3.png" alt="joint_input_result_1"  width="375" />
<img src="./sawyer-4dof-manipulator/fig/position_error_result_3.png" alt="joint_input_result_1"  width="375" />


#### fsmc
<img src="./sawyer-4dof-manipulator/fig/joint_position_result_4.png" alt="joint_input_result_1"  width="375" />
<img src="./sawyer-4dof-manipulator/fig/joint_input_result_4.png" alt="joint_input_result_1"  width="375" />
<img src="./sawyer-4dof-manipulator/fig/position_error_result_4.png" alt="joint_input_result_1"  width="375" />

####  afsmc
<img src="./sawyer-4dof-manipulator/fig/joint_position_result_5.png" alt="joint_input_result_1"  width="375" />
<img src="./sawyer-4dof-manipulator/fig/joint_input_result_5.png" alt="joint_input_result_1"  width="375" />
<img src="./sawyer-4dof-manipulator/fig/position_error_result_5.png" alt="joint_input_result_1"  width="375" />








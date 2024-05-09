# Task 3: 6D Pose Estimation

The goal of this task is get the **6D Pose** of the `inboard` that is attached to the gripper, i.e:
- **Translation vector** in X, Y, Z directions.
- **Rotation Matrix**.

---
$$
\mathbf{H} = \begin{bmatrix}
r_{11} & r_{12} & r_{13} & t_x \\
r_{21} & r_{22} & r_{23} & t_y \\
r_{31} & r_{32} & r_{33} & t_z \\
0 & 0 & 0 & 1
\end{bmatrix}
$$
---

The image below shows the origin of the object as seen on the given mesh file:
![Pose 6D](imgs/inboard_1.png)

Instruction
---
Implement a method that estimates the 6D pose of the attached object.

Please validate your method using the given RGD-D images or ROS bag file. It is further recommended to visualise the estimated pose overlayed on the input data for visual comparison.

> Tip: Use visualisation tools provided in RViz or Open3D to show your results 

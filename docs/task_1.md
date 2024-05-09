# Task 1: Object Isolation

The goal of this task is to detect the `inboard` that is attached to the gripper, and to isolate it from other objects in the scene by creating a **mask**. The attached object could be found to be in various orientations on the magnetic gripper.

The following object is called `inboard`.

<img src="imgs/inboard_2.png" alt="drawing" width="400"/>
<img src="imgs/inboard_1.png" alt="drawing" width="200"/>


Bin picking scenario:

<img src="imgs/binpick_background.png" alt="object in hand" width="400"/>

Camera view / Input data:

<img src="imgs/Pose_Corrector_Challenge.drawio.png" alt="object in hand" width="400"/>

Instruction
---
Implement a method that is able to isolate the attached object from the surrounding pixels for the camera view RGB-D data provided. An example masked 'inboard' image: 

<img src="imgs/object_mask.png" alt="object in hand" width="400"/>


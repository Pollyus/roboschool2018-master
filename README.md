# Line Follower Lab

## Description

Move a six-wheeled differential drive mobile robot along the colored line using **"Proportional-Differential-Integral" (PID)** controller or its modification. The mobile robot is equipped with an RGB monocular camera and wheel encoders.

## Tasks

1. Setup image processing pipeline to detect the colored line.
2. Move along the colored line using P controller.
3. Move along the colored line using PI/PID controller.
4. (*) Move along the colored line using other modification of PID controller (e.g. PIID).
5. Stop at the red traffic lights.
6. Turn to the right on the crossroad.
7. [Change camera parameters](car_description/urdf/camera.xacro): add noise, decrease resolution, add radial distortion.

## Build and Run

1. Create a catkin workspace:

   ```bash
   mkdir -p catkin_ws/src
   cd catkin_ws/src
   
   catkin_init_workspace
   ```

2. Clone the project into the workspace (**fork the repository beforehand**):

   ```bash
   # replace the <user> with your login
   git clone https://github.com/<user>/roboschool2018.git
   ```

3. Build the packages:

   ```bash
   cd .. && catkin_make
   
   # setup Gazebo model path (in ~/.bashrc)
   export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:<path>/catkin_ws/src/roboschool2018/car_gazebo/models
   
   source ~/.bashrc
   source devel/setup.bash
   ```

4. Run the simulation nodes:

   ```bash
   # RViz
   roslaunch car_description view_model.launch
   
   # Gazebo simulation
   roslaunch car_gazebo track.launch
   
   # image processing pipeline demo (Python, line detection)
   rosrun car_hackathon line_detection_example.py
   
   # line follower (C++)
   rosrun car_hackathon line_follower_cpp
   
   # control the traffic lights
   rosrun car_gazebo light_controller.py
   ```

## Useful Resources

Presentations:

- [ROS. Моделирование роботов в среде Gazebo](ROS_Gazebo.pdf)
 - [Использование ROS, Gazebo и OpenCV для распознавания дорожной разметки](ROS_Gazebo_OpenCV.pdf)


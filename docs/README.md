# Simulated Lane Detection

Lane detection package that runs on the Remote PC receives camera images either from TurtleBot3 or Gazebo simulation to detect driving lanes and to drive the Turtlebot3 along them.
The following instructions describe how to use and calibrate the lane detection feature via rqt.

1. Place the TurtleBot3 inbetween yellow and white lanes.

    - NOTE: The lane detection filters yellow on the left side while filters white on the right side. Be sure that the yellow lane is on the left side of the robot.


2. `$ roslaunch tb3_gazebo turtlebot3_autorace.launch`
3. `$ roslaunch tb3_camera intrinsic_camera_calibration.launch`
4. `$ roslaunch tb3_camera extrinsic_camera_calibration.launch`
5. `$ roslaunch tb3_detect detect_lane.launch mode:=calibration`
6. `$ rqt`

7. Launch the rqt image viewer by selecting Plugins > Cisualization > Image view.
Multiple rqt plugins can be run.
8. Display 3 topics at each image viewer
    - /detect/image_lane/compressed
    - /detect/image_yellow_lane_marker/compressed : a yellow range color filtered image.
    - /detect/image_white_lane_marker/compressed : a white range color filtered image.

9. `$ rosrun rqt_reconfigure rqt_reconfigure`
10. Click detect_lane then adjust parameters so that yellow and white colors can be filtered properly.

11. Open lane.yaml file located in tb3_detect/param/lane/. 
    - You need to write modified values to the file. This will make the camera set its parameters as you set here from next launching.

12. Close the terminal or terminate with Ctrl + C on rqt_reconfigure and detect_lane terminals.

13. Open a new terminal and launch the lane detect node without the calibration option.
    - `$ roslaunch tb3_detect detect_lane.launch`

14. Open a new terminal and launch the node below to start the lane following operation.
    - `$ roslaunch tb3_driving tb3_control_lane.launch`


## alias simulated lane follow
- `rltba`  -roslaunch tb3_gazebo turtlebot3_autorace.launch
- `icam`   -roslaunch tb3_camera intrinsic_camera_calibration.launch
- `ecam`   -roslaunch tb3_camera extrinsic_camera_calibration.launch
- `detlnc` -roslaunch tb3_detect detect_lane.launch mode:=calibration
- `rqt`
- `rqtconfig` -rosrun rqt_reconfigure rqt_reconfigure
- close rqt_reconfigure and detect lane
- `detln`   -roslaunch tb3_detect detect_lane.launch
- `drln`    -roslaunch tb3_driving tb3_control_lane.launch



# Lane Detection

1. Place TurtleBot3 between yellow and white lanes.
    - NOTE: Be sure that yellow lane is placed left side of the robot and White lane is placed right side of the robot.
2. `roscore`
3. `tb3cam`
4. `icama`
5. `ecama`
6. `detlnc`
7. `rqt`
8. Click plugins > visualization > Image view; Multiple windows will be present
9. Select three topics at each image view: 
    - /detect/image_yellow_lane_marker/compressed
    - /detect/image_lane/compressed
    - /detect/image_white_lane_marker/compressed
10. `rqtconfig`
11. Click Detect Lane then adjust parameters to do line color filtering.
12. Open lane.yaml file located in turtlebot3_autorace_detect/param/lane/. 
    - You need to write modified values to the file. This will make the camera set its parameters as you set here from next launching.
13. Close both rqt_rconfigure and turtlebot3_autorace_detect_lane.
14. `detlna`
15. Check if the results come out correctly.
    - `drln`
    - `tb3lnch`
# Simulated Lane Detection

Lane detection package that runs on the Remote PC receives camera images either from TurtleBot3 or Gazebo simulation to detect driving lanes and to drive the Turtlebot3 along them.
The following instructions describe how to use and calibrate the lane detection feature via rqt.

1. Place the TurtleBot3 inbetween yellow and white lanes.

    - NOTE: The lane detection filters yellow on the left side while filters white on the right side. Be sure that the yellow lane is on the left side of the robot.


2. `$ roslaunch turtlebot3_gazebo tb3_autonomous_op.launch`
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
- `rltba`
- `icam`
- `ecam`
- `detlnc`
- `rqt`
- `rqtconfig`
- close rqt_reconfigure and detect lane
- `detln`
- `drln`

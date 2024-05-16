## Turtlebot Challenge - ENPM673

### Team Members - Group 19

- Ruthwik Dasyam
- Md Zahiruddin
- Shivam Dhakad
- Abhey Sharma 


### Instructions

1. Download and setup the turtlebot3 ROS humble package from turtlebot documnetation

2. Copy paste the 'perception' package and 'enpm673_final_proj' package in the src folder along with other turtlebot3 packages

3. Launch the simulation using the command
    ```
    ros2 launch enpm673_final_proj enpm673_world.launch.py 
    ```

4. We did not use chessboard for horizon line detection and it messes up the path planning a little bit (makes it run slow till it passes chessboard), Use teleop and pass over the chessboard and align properly with the 3rd paper to detect the horizon line

5. Launch the script using the following command
    ```
    ros2 run perception percept
    ```


### Hardware
To run on hardware change the _image topic_ in '/perception/perception/percept.py' to get feed from camera

Simulation Video Link

https://drive.google.com/file/d/1KqcUp2-_mwKfzmICVlzJcGbCpeXmtNB0/view?usp=drive_link
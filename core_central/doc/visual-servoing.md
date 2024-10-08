# Task1  circle visual servoing

1. take off 
2. BoatLanding: 
    - frame_id:world
    - velocity: 3
    - max_acceleration: 0.3
    - height: 1
    - width: 28
    - activate_wrench: 0
    - force: 0
3. rosrun feature_extractor circle_extractor.py
    - the script will mute pose controller and send vel target to velocity controller
    - debug/visualize topic: /image_color_processed
    - change visual target: /visual_target
    rostopic pub /visual_target std_msgs/Float64MultiArray "layout:
  dim:
  \- label: ' '
    size: 2
    stride: 0
  data_offset: 0
data: [650.0, 350.0, 300.0, 300.0]" 
    
        data: rect (x,y,w,h)

# TODO: 

1. add a GUI section for visual-servoing activation

2. add a script that automate the whole process



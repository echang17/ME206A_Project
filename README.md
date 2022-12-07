# ME206A_Project
Planning and Control with Human Collaborators

## Running Vision Node(s) (subscribes to /ar_pose_marker and publishes each marker's information in reference/base frame to /tag_info, along with object dimensions):
- update ~/.bashrc file to correct robot and workstation -> source ~/.bashrc
- source workspace: cd ME206A/Project, source devel/setup.bash
- connect to robot: ./intera.sh
- enable robot: rosrun intera_interface enable_robot.py -e
- display live view of Sawyer's head cam (sometimes rviz doesn't detect camera topic otherwise): rosrun intera_examples camera_display.py
- run launch file for AR tag tracking: roslaunch vision ar_track.launch
- (in a new terminal, with workspace sourced, connected to robot) open RVIZ: rosrun rviz rviz
  - set fixed frame: head_camera
  - add Camera display, set topic: /io/internal_camera/head_camera/image_raw (image with AR tag frame overlay)
  - add TF display to see AR tag frames (x: red, y: green, z: blue)
- (in a new terminal, with workspace sourced, connected to robot) run sawyer_vision_calib.py: rosrun vision sawyer_vision_calib.py (this prints object length and width)
- run sawyer_vision3.py with these dimensions: rosrun vision sawyer_vision3.py [object length] [object width]
  - may need to cd src/vision/src, chmod +x *.py to enable python files as executables first
- (in a new terminal, with workspace sourced, connected to robot) see what's being published to /tag_info topic: rostopic echo /tag_info
  - see what's being published to /ar_pose_marker topic: rostopic echo /ar_pose_marker

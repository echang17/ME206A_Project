# ME206A_Project
Planning and Control with Human Collaborators

## Running Vision Node (subscribes to /ar_pose_marker and publishes each marker's information to /tag_info):
- update ~/.bashrc file to correct robot and workstation -> source ~/.bashrc
- source workspace: cd ME206A/Project, source devel/setup.bash
- connect to robot: ./intera.sh
- enable robot: rosrun intera_interface enable_robot.py -e
- display live view of Sawyer's head cam (sometimes rviz doesn't detect camera topic otherwise): rosrun intera_examples camera_display.py
- run launch file for AR tag tracking: roslaunch vision ar_track.launch
- (in a new terminal, with workspace sourced, connected to robot) open RVIZ: rosrun rviz rviz
  - set fixed frame: head_camera
  - add Image display, set topic: /io/internal_camera/head_camera/image_raw (image only)
  - add Camera display, set topic: /io/internal_camera/head_camera/image_raw (image with AR tag frame overlay)
  - add TF display to see AR tag frames (x: red, y: green, z: blue)
- (in a new terminal, with workspace sourced, connected to robot) run sawyer_vision.py: rosrun vision sawyer_vision.py
  - may need to cd src/vision/src, chmod +x *.py to enable python files as executables first
- (in a new terminal, with workspace sourced, connected to robot) see what's being published to /tag_info topic: rostopic echo \tag_info
  - see what's being published to /ar_pose_marker topic: rostopic echo /ar_pose_marker

# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "vision: 2 messages, 0 services")

set(MSG_I_FLAGS "-Ivision:/home/cc/ee106a/fa22/class/ee106a-adg/ME206A_Project/src/vision/msg;-Isensor_msgs:/opt/ros/noetic/share/sensor_msgs/cmake/../msg;-Igeometry_msgs:/opt/ros/noetic/share/geometry_msgs/cmake/../msg;-Istd_msgs:/opt/ros/noetic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(vision_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/cc/ee106a/fa22/class/ee106a-adg/ME206A_Project/src/vision/msg/VisualData.msg" NAME_WE)
add_custom_target(_vision_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "vision" "/home/cc/ee106a/fa22/class/ee106a-adg/ME206A_Project/src/vision/msg/VisualData.msg" "geometry_msgs/Pose:geometry_msgs/Point:geometry_msgs/Quaternion:geometry_msgs/PoseStamped:std_msgs/Header"
)

get_filename_component(_filename "/home/cc/ee106a/fa22/class/ee106a-adg/ME206A_Project/src/vision/msg/SawyerCog.msg" NAME_WE)
add_custom_target(_vision_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "vision" "/home/cc/ee106a/fa22/class/ee106a-adg/ME206A_Project/src/vision/msg/SawyerCog.msg" "geometry_msgs/Quaternion:geometry_msgs/Pose:geometry_msgs/Point"
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(vision
  "/home/cc/ee106a/fa22/class/ee106a-adg/ME206A_Project/src/vision/msg/VisualData.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/PoseStamped.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/vision
)
_generate_msg_cpp(vision
  "/home/cc/ee106a/fa22/class/ee106a-adg/ME206A_Project/src/vision/msg/SawyerCog.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/vision
)

### Generating Services

### Generating Module File
_generate_module_cpp(vision
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/vision
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(vision_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(vision_generate_messages vision_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/cc/ee106a/fa22/class/ee106a-adg/ME206A_Project/src/vision/msg/VisualData.msg" NAME_WE)
add_dependencies(vision_generate_messages_cpp _vision_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/cc/ee106a/fa22/class/ee106a-adg/ME206A_Project/src/vision/msg/SawyerCog.msg" NAME_WE)
add_dependencies(vision_generate_messages_cpp _vision_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(vision_gencpp)
add_dependencies(vision_gencpp vision_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS vision_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(vision
  "/home/cc/ee106a/fa22/class/ee106a-adg/ME206A_Project/src/vision/msg/VisualData.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/PoseStamped.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/vision
)
_generate_msg_eus(vision
  "/home/cc/ee106a/fa22/class/ee106a-adg/ME206A_Project/src/vision/msg/SawyerCog.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/vision
)

### Generating Services

### Generating Module File
_generate_module_eus(vision
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/vision
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(vision_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(vision_generate_messages vision_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/cc/ee106a/fa22/class/ee106a-adg/ME206A_Project/src/vision/msg/VisualData.msg" NAME_WE)
add_dependencies(vision_generate_messages_eus _vision_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/cc/ee106a/fa22/class/ee106a-adg/ME206A_Project/src/vision/msg/SawyerCog.msg" NAME_WE)
add_dependencies(vision_generate_messages_eus _vision_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(vision_geneus)
add_dependencies(vision_geneus vision_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS vision_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(vision
  "/home/cc/ee106a/fa22/class/ee106a-adg/ME206A_Project/src/vision/msg/VisualData.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/PoseStamped.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/vision
)
_generate_msg_lisp(vision
  "/home/cc/ee106a/fa22/class/ee106a-adg/ME206A_Project/src/vision/msg/SawyerCog.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/vision
)

### Generating Services

### Generating Module File
_generate_module_lisp(vision
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/vision
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(vision_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(vision_generate_messages vision_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/cc/ee106a/fa22/class/ee106a-adg/ME206A_Project/src/vision/msg/VisualData.msg" NAME_WE)
add_dependencies(vision_generate_messages_lisp _vision_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/cc/ee106a/fa22/class/ee106a-adg/ME206A_Project/src/vision/msg/SawyerCog.msg" NAME_WE)
add_dependencies(vision_generate_messages_lisp _vision_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(vision_genlisp)
add_dependencies(vision_genlisp vision_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS vision_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(vision
  "/home/cc/ee106a/fa22/class/ee106a-adg/ME206A_Project/src/vision/msg/VisualData.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/PoseStamped.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/vision
)
_generate_msg_nodejs(vision
  "/home/cc/ee106a/fa22/class/ee106a-adg/ME206A_Project/src/vision/msg/SawyerCog.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/vision
)

### Generating Services

### Generating Module File
_generate_module_nodejs(vision
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/vision
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(vision_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(vision_generate_messages vision_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/cc/ee106a/fa22/class/ee106a-adg/ME206A_Project/src/vision/msg/VisualData.msg" NAME_WE)
add_dependencies(vision_generate_messages_nodejs _vision_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/cc/ee106a/fa22/class/ee106a-adg/ME206A_Project/src/vision/msg/SawyerCog.msg" NAME_WE)
add_dependencies(vision_generate_messages_nodejs _vision_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(vision_gennodejs)
add_dependencies(vision_gennodejs vision_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS vision_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(vision
  "/home/cc/ee106a/fa22/class/ee106a-adg/ME206A_Project/src/vision/msg/VisualData.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/PoseStamped.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/vision
)
_generate_msg_py(vision
  "/home/cc/ee106a/fa22/class/ee106a-adg/ME206A_Project/src/vision/msg/SawyerCog.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/vision
)

### Generating Services

### Generating Module File
_generate_module_py(vision
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/vision
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(vision_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(vision_generate_messages vision_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/cc/ee106a/fa22/class/ee106a-adg/ME206A_Project/src/vision/msg/VisualData.msg" NAME_WE)
add_dependencies(vision_generate_messages_py _vision_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/cc/ee106a/fa22/class/ee106a-adg/ME206A_Project/src/vision/msg/SawyerCog.msg" NAME_WE)
add_dependencies(vision_generate_messages_py _vision_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(vision_genpy)
add_dependencies(vision_genpy vision_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS vision_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/vision)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/vision
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET sensor_msgs_generate_messages_cpp)
  add_dependencies(vision_generate_messages_cpp sensor_msgs_generate_messages_cpp)
endif()
if(TARGET geometry_msgs_generate_messages_cpp)
  add_dependencies(vision_generate_messages_cpp geometry_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/vision)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/vision
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET sensor_msgs_generate_messages_eus)
  add_dependencies(vision_generate_messages_eus sensor_msgs_generate_messages_eus)
endif()
if(TARGET geometry_msgs_generate_messages_eus)
  add_dependencies(vision_generate_messages_eus geometry_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/vision)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/vision
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET sensor_msgs_generate_messages_lisp)
  add_dependencies(vision_generate_messages_lisp sensor_msgs_generate_messages_lisp)
endif()
if(TARGET geometry_msgs_generate_messages_lisp)
  add_dependencies(vision_generate_messages_lisp geometry_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/vision)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/vision
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET sensor_msgs_generate_messages_nodejs)
  add_dependencies(vision_generate_messages_nodejs sensor_msgs_generate_messages_nodejs)
endif()
if(TARGET geometry_msgs_generate_messages_nodejs)
  add_dependencies(vision_generate_messages_nodejs geometry_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/vision)
  install(CODE "execute_process(COMMAND \"/usr/bin/python3\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/vision\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/vision
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET sensor_msgs_generate_messages_py)
  add_dependencies(vision_generate_messages_py sensor_msgs_generate_messages_py)
endif()
if(TARGET geometry_msgs_generate_messages_py)
  add_dependencies(vision_generate_messages_py geometry_msgs_generate_messages_py)
endif()

# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "marker_controller: 2 messages, 0 services")

set(MSG_I_FLAGS "-Imarker_controller:/home/alexj/Dropbox/UFMG/04-2-2016/JornalPaper/02-code/03-ros_ws_latpop_new/src/marker_controller/msg;-Igeometry_msgs:/opt/ros/kinetic/share/geometry_msgs/cmake/../msg;-Istd_msgs:/opt/ros/kinetic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(marker_controller_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/alexj/Dropbox/UFMG/04-2-2016/JornalPaper/02-code/03-ros_ws_latpop_new/src/marker_controller/msg/TargetPose.msg" NAME_WE)
add_custom_target(_marker_controller_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "marker_controller" "/home/alexj/Dropbox/UFMG/04-2-2016/JornalPaper/02-code/03-ros_ws_latpop_new/src/marker_controller/msg/TargetPose.msg" "geometry_msgs/Quaternion:geometry_msgs/PoseStamped:geometry_msgs/Pose:std_msgs/Header:geometry_msgs/Point"
)

get_filename_component(_filename "/home/alexj/Dropbox/UFMG/04-2-2016/JornalPaper/02-code/03-ros_ws_latpop_new/src/marker_controller/msg/TargetPoses.msg" NAME_WE)
add_custom_target(_marker_controller_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "marker_controller" "/home/alexj/Dropbox/UFMG/04-2-2016/JornalPaper/02-code/03-ros_ws_latpop_new/src/marker_controller/msg/TargetPoses.msg" "geometry_msgs/PoseStamped:std_msgs/Header:geometry_msgs/Quaternion:geometry_msgs/Point:marker_controller/TargetPose:geometry_msgs/Pose"
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(marker_controller
  "/home/alexj/Dropbox/UFMG/04-2-2016/JornalPaper/02-code/03-ros_ws_latpop_new/src/marker_controller/msg/TargetPose.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/PoseStamped.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/marker_controller
)
_generate_msg_cpp(marker_controller
  "/home/alexj/Dropbox/UFMG/04-2-2016/JornalPaper/02-code/03-ros_ws_latpop_new/src/marker_controller/msg/TargetPoses.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/PoseStamped.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg;/home/alexj/Dropbox/UFMG/04-2-2016/JornalPaper/02-code/03-ros_ws_latpop_new/src/marker_controller/msg/TargetPose.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Pose.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/marker_controller
)

### Generating Services

### Generating Module File
_generate_module_cpp(marker_controller
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/marker_controller
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(marker_controller_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(marker_controller_generate_messages marker_controller_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/alexj/Dropbox/UFMG/04-2-2016/JornalPaper/02-code/03-ros_ws_latpop_new/src/marker_controller/msg/TargetPose.msg" NAME_WE)
add_dependencies(marker_controller_generate_messages_cpp _marker_controller_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/alexj/Dropbox/UFMG/04-2-2016/JornalPaper/02-code/03-ros_ws_latpop_new/src/marker_controller/msg/TargetPoses.msg" NAME_WE)
add_dependencies(marker_controller_generate_messages_cpp _marker_controller_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(marker_controller_gencpp)
add_dependencies(marker_controller_gencpp marker_controller_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS marker_controller_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(marker_controller
  "/home/alexj/Dropbox/UFMG/04-2-2016/JornalPaper/02-code/03-ros_ws_latpop_new/src/marker_controller/msg/TargetPose.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/PoseStamped.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/marker_controller
)
_generate_msg_eus(marker_controller
  "/home/alexj/Dropbox/UFMG/04-2-2016/JornalPaper/02-code/03-ros_ws_latpop_new/src/marker_controller/msg/TargetPoses.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/PoseStamped.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg;/home/alexj/Dropbox/UFMG/04-2-2016/JornalPaper/02-code/03-ros_ws_latpop_new/src/marker_controller/msg/TargetPose.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Pose.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/marker_controller
)

### Generating Services

### Generating Module File
_generate_module_eus(marker_controller
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/marker_controller
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(marker_controller_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(marker_controller_generate_messages marker_controller_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/alexj/Dropbox/UFMG/04-2-2016/JornalPaper/02-code/03-ros_ws_latpop_new/src/marker_controller/msg/TargetPose.msg" NAME_WE)
add_dependencies(marker_controller_generate_messages_eus _marker_controller_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/alexj/Dropbox/UFMG/04-2-2016/JornalPaper/02-code/03-ros_ws_latpop_new/src/marker_controller/msg/TargetPoses.msg" NAME_WE)
add_dependencies(marker_controller_generate_messages_eus _marker_controller_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(marker_controller_geneus)
add_dependencies(marker_controller_geneus marker_controller_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS marker_controller_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(marker_controller
  "/home/alexj/Dropbox/UFMG/04-2-2016/JornalPaper/02-code/03-ros_ws_latpop_new/src/marker_controller/msg/TargetPose.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/PoseStamped.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/marker_controller
)
_generate_msg_lisp(marker_controller
  "/home/alexj/Dropbox/UFMG/04-2-2016/JornalPaper/02-code/03-ros_ws_latpop_new/src/marker_controller/msg/TargetPoses.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/PoseStamped.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg;/home/alexj/Dropbox/UFMG/04-2-2016/JornalPaper/02-code/03-ros_ws_latpop_new/src/marker_controller/msg/TargetPose.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Pose.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/marker_controller
)

### Generating Services

### Generating Module File
_generate_module_lisp(marker_controller
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/marker_controller
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(marker_controller_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(marker_controller_generate_messages marker_controller_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/alexj/Dropbox/UFMG/04-2-2016/JornalPaper/02-code/03-ros_ws_latpop_new/src/marker_controller/msg/TargetPose.msg" NAME_WE)
add_dependencies(marker_controller_generate_messages_lisp _marker_controller_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/alexj/Dropbox/UFMG/04-2-2016/JornalPaper/02-code/03-ros_ws_latpop_new/src/marker_controller/msg/TargetPoses.msg" NAME_WE)
add_dependencies(marker_controller_generate_messages_lisp _marker_controller_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(marker_controller_genlisp)
add_dependencies(marker_controller_genlisp marker_controller_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS marker_controller_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(marker_controller
  "/home/alexj/Dropbox/UFMG/04-2-2016/JornalPaper/02-code/03-ros_ws_latpop_new/src/marker_controller/msg/TargetPose.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/PoseStamped.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/marker_controller
)
_generate_msg_nodejs(marker_controller
  "/home/alexj/Dropbox/UFMG/04-2-2016/JornalPaper/02-code/03-ros_ws_latpop_new/src/marker_controller/msg/TargetPoses.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/PoseStamped.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg;/home/alexj/Dropbox/UFMG/04-2-2016/JornalPaper/02-code/03-ros_ws_latpop_new/src/marker_controller/msg/TargetPose.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Pose.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/marker_controller
)

### Generating Services

### Generating Module File
_generate_module_nodejs(marker_controller
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/marker_controller
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(marker_controller_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(marker_controller_generate_messages marker_controller_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/alexj/Dropbox/UFMG/04-2-2016/JornalPaper/02-code/03-ros_ws_latpop_new/src/marker_controller/msg/TargetPose.msg" NAME_WE)
add_dependencies(marker_controller_generate_messages_nodejs _marker_controller_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/alexj/Dropbox/UFMG/04-2-2016/JornalPaper/02-code/03-ros_ws_latpop_new/src/marker_controller/msg/TargetPoses.msg" NAME_WE)
add_dependencies(marker_controller_generate_messages_nodejs _marker_controller_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(marker_controller_gennodejs)
add_dependencies(marker_controller_gennodejs marker_controller_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS marker_controller_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(marker_controller
  "/home/alexj/Dropbox/UFMG/04-2-2016/JornalPaper/02-code/03-ros_ws_latpop_new/src/marker_controller/msg/TargetPose.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/PoseStamped.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/marker_controller
)
_generate_msg_py(marker_controller
  "/home/alexj/Dropbox/UFMG/04-2-2016/JornalPaper/02-code/03-ros_ws_latpop_new/src/marker_controller/msg/TargetPoses.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/PoseStamped.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg;/home/alexj/Dropbox/UFMG/04-2-2016/JornalPaper/02-code/03-ros_ws_latpop_new/src/marker_controller/msg/TargetPose.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Pose.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/marker_controller
)

### Generating Services

### Generating Module File
_generate_module_py(marker_controller
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/marker_controller
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(marker_controller_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(marker_controller_generate_messages marker_controller_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/alexj/Dropbox/UFMG/04-2-2016/JornalPaper/02-code/03-ros_ws_latpop_new/src/marker_controller/msg/TargetPose.msg" NAME_WE)
add_dependencies(marker_controller_generate_messages_py _marker_controller_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/alexj/Dropbox/UFMG/04-2-2016/JornalPaper/02-code/03-ros_ws_latpop_new/src/marker_controller/msg/TargetPoses.msg" NAME_WE)
add_dependencies(marker_controller_generate_messages_py _marker_controller_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(marker_controller_genpy)
add_dependencies(marker_controller_genpy marker_controller_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS marker_controller_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/marker_controller)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/marker_controller
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_cpp)
  add_dependencies(marker_controller_generate_messages_cpp geometry_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/marker_controller)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/marker_controller
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_eus)
  add_dependencies(marker_controller_generate_messages_eus geometry_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/marker_controller)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/marker_controller
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_lisp)
  add_dependencies(marker_controller_generate_messages_lisp geometry_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/marker_controller)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/marker_controller
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_nodejs)
  add_dependencies(marker_controller_generate_messages_nodejs geometry_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/marker_controller)
  install(CODE "execute_process(COMMAND \"/usr/bin/python\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/marker_controller\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/marker_controller
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_py)
  add_dependencies(marker_controller_generate_messages_py geometry_msgs_generate_messages_py)
endif()

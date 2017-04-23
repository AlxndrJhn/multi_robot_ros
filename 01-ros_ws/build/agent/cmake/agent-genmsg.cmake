# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "agent: 1 messages, 0 services")

set(MSG_I_FLAGS "-Iagent:/home/alexj/Dropbox/UFMG/04-2-2016/JornalPaper/02-code/01-ros_ws/src/agent/msg;-Istd_msgs:/opt/ros/indigo/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(genlisp REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(agent_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/alexj/Dropbox/UFMG/04-2-2016/JornalPaper/02-code/01-ros_ws/src/agent/msg/bid.msg" NAME_WE)
add_custom_target(_agent_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "agent" "/home/alexj/Dropbox/UFMG/04-2-2016/JornalPaper/02-code/01-ros_ws/src/agent/msg/bid.msg" ""
)

#
#  langs = gencpp;genlisp;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(agent
  "/home/alexj/Dropbox/UFMG/04-2-2016/JornalPaper/02-code/01-ros_ws/src/agent/msg/bid.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/agent
)

### Generating Services

### Generating Module File
_generate_module_cpp(agent
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/agent
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(agent_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(agent_generate_messages agent_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/alexj/Dropbox/UFMG/04-2-2016/JornalPaper/02-code/01-ros_ws/src/agent/msg/bid.msg" NAME_WE)
add_dependencies(agent_generate_messages_cpp _agent_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(agent_gencpp)
add_dependencies(agent_gencpp agent_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS agent_generate_messages_cpp)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(agent
  "/home/alexj/Dropbox/UFMG/04-2-2016/JornalPaper/02-code/01-ros_ws/src/agent/msg/bid.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/agent
)

### Generating Services

### Generating Module File
_generate_module_lisp(agent
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/agent
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(agent_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(agent_generate_messages agent_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/alexj/Dropbox/UFMG/04-2-2016/JornalPaper/02-code/01-ros_ws/src/agent/msg/bid.msg" NAME_WE)
add_dependencies(agent_generate_messages_lisp _agent_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(agent_genlisp)
add_dependencies(agent_genlisp agent_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS agent_generate_messages_lisp)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(agent
  "/home/alexj/Dropbox/UFMG/04-2-2016/JornalPaper/02-code/01-ros_ws/src/agent/msg/bid.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/agent
)

### Generating Services

### Generating Module File
_generate_module_py(agent
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/agent
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(agent_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(agent_generate_messages agent_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/alexj/Dropbox/UFMG/04-2-2016/JornalPaper/02-code/01-ros_ws/src/agent/msg/bid.msg" NAME_WE)
add_dependencies(agent_generate_messages_py _agent_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(agent_genpy)
add_dependencies(agent_genpy agent_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS agent_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/agent)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/agent
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
add_dependencies(agent_generate_messages_cpp std_msgs_generate_messages_cpp)

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/agent)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/agent
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
add_dependencies(agent_generate_messages_lisp std_msgs_generate_messages_lisp)

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/agent)
  install(CODE "execute_process(COMMAND \"/usr/bin/python\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/agent\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/agent
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
add_dependencies(agent_generate_messages_py std_msgs_generate_messages_py)

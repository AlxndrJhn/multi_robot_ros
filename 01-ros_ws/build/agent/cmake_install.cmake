# Install script for directory: /home/alexj/Dropbox/UFMG/04-2-2016/JornalPaper/02-code/01-ros_ws/src/agent

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/alexj/Dropbox/UFMG/04-2-2016/JornalPaper/02-code/01-ros_ws/install")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/agent/msg" TYPE FILE FILES "/home/alexj/Dropbox/UFMG/04-2-2016/JornalPaper/02-code/01-ros_ws/src/agent/msg/bid.msg")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/agent/cmake" TYPE FILE FILES "/home/alexj/Dropbox/UFMG/04-2-2016/JornalPaper/02-code/01-ros_ws/build/agent/catkin_generated/installspace/agent-msg-paths.cmake")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE DIRECTORY FILES "/home/alexj/Dropbox/UFMG/04-2-2016/JornalPaper/02-code/01-ros_ws/devel/include/agent")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/common-lisp/ros" TYPE DIRECTORY FILES "/home/alexj/Dropbox/UFMG/04-2-2016/JornalPaper/02-code/01-ros_ws/devel/share/common-lisp/ros/agent")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  execute_process(COMMAND "/usr/bin/python" -m compileall "/home/alexj/Dropbox/UFMG/04-2-2016/JornalPaper/02-code/01-ros_ws/devel/lib/python2.7/dist-packages/agent")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python2.7/dist-packages" TYPE DIRECTORY FILES "/home/alexj/Dropbox/UFMG/04-2-2016/JornalPaper/02-code/01-ros_ws/devel/lib/python2.7/dist-packages/agent")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/alexj/Dropbox/UFMG/04-2-2016/JornalPaper/02-code/01-ros_ws/build/agent/catkin_generated/installspace/agent.pc")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/agent/cmake" TYPE FILE FILES "/home/alexj/Dropbox/UFMG/04-2-2016/JornalPaper/02-code/01-ros_ws/build/agent/catkin_generated/installspace/agent-msg-extras.cmake")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/agent/cmake" TYPE FILE FILES
    "/home/alexj/Dropbox/UFMG/04-2-2016/JornalPaper/02-code/01-ros_ws/build/agent/catkin_generated/installspace/agentConfig.cmake"
    "/home/alexj/Dropbox/UFMG/04-2-2016/JornalPaper/02-code/01-ros_ws/build/agent/catkin_generated/installspace/agentConfig-version.cmake"
    )
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/agent" TYPE FILE FILES "/home/alexj/Dropbox/UFMG/04-2-2016/JornalPaper/02-code/01-ros_ws/src/agent/package.xml")
endif()


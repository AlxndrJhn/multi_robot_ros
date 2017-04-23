# Install script for directory: /home/alexj/Dropbox/UFMG/04-2-2016/JornalPaper/02-code/03-ros_ws_latpop_new/src/marker_controller

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/alexj/Dropbox/UFMG/04-2-2016/JornalPaper/02-code/03-ros_ws_latpop_new/install")
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
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/marker_controller/msg" TYPE FILE FILES
    "/home/alexj/Dropbox/UFMG/04-2-2016/JornalPaper/02-code/03-ros_ws_latpop_new/src/marker_controller/msg/TargetPose.msg"
    "/home/alexj/Dropbox/UFMG/04-2-2016/JornalPaper/02-code/03-ros_ws_latpop_new/src/marker_controller/msg/TargetPoses.msg"
    )
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/marker_controller/cmake" TYPE FILE FILES "/home/alexj/Dropbox/UFMG/04-2-2016/JornalPaper/02-code/03-ros_ws_latpop_new/build/marker_controller/catkin_generated/installspace/marker_controller-msg-paths.cmake")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE DIRECTORY FILES "/home/alexj/Dropbox/UFMG/04-2-2016/JornalPaper/02-code/03-ros_ws_latpop_new/devel/include/marker_controller")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/roseus/ros" TYPE DIRECTORY FILES "/home/alexj/Dropbox/UFMG/04-2-2016/JornalPaper/02-code/03-ros_ws_latpop_new/devel/share/roseus/ros/marker_controller")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/common-lisp/ros" TYPE DIRECTORY FILES "/home/alexj/Dropbox/UFMG/04-2-2016/JornalPaper/02-code/03-ros_ws_latpop_new/devel/share/common-lisp/ros/marker_controller")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/gennodejs/ros" TYPE DIRECTORY FILES "/home/alexj/Dropbox/UFMG/04-2-2016/JornalPaper/02-code/03-ros_ws_latpop_new/devel/share/gennodejs/ros/marker_controller")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  execute_process(COMMAND "/usr/bin/python" -m compileall "/home/alexj/Dropbox/UFMG/04-2-2016/JornalPaper/02-code/03-ros_ws_latpop_new/devel/lib/python2.7/dist-packages/marker_controller")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python2.7/dist-packages" TYPE DIRECTORY FILES "/home/alexj/Dropbox/UFMG/04-2-2016/JornalPaper/02-code/03-ros_ws_latpop_new/devel/lib/python2.7/dist-packages/marker_controller")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/alexj/Dropbox/UFMG/04-2-2016/JornalPaper/02-code/03-ros_ws_latpop_new/build/marker_controller/catkin_generated/installspace/marker_controller.pc")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/marker_controller/cmake" TYPE FILE FILES "/home/alexj/Dropbox/UFMG/04-2-2016/JornalPaper/02-code/03-ros_ws_latpop_new/build/marker_controller/catkin_generated/installspace/marker_controller-msg-extras.cmake")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/marker_controller/cmake" TYPE FILE FILES
    "/home/alexj/Dropbox/UFMG/04-2-2016/JornalPaper/02-code/03-ros_ws_latpop_new/build/marker_controller/catkin_generated/installspace/marker_controllerConfig.cmake"
    "/home/alexj/Dropbox/UFMG/04-2-2016/JornalPaper/02-code/03-ros_ws_latpop_new/build/marker_controller/catkin_generated/installspace/marker_controllerConfig-version.cmake"
    )
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/marker_controller" TYPE FILE FILES "/home/alexj/Dropbox/UFMG/04-2-2016/JornalPaper/02-code/03-ros_ws_latpop_new/src/marker_controller/package.xml")
endif()


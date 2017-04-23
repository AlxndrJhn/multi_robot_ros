# Install script for directory: /home/alexj/Dropbox/UFMG/04-2-2016/JornalPaper/02-code/03-ros_ws_latpop_new/src/laptop_node

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
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/alexj/Dropbox/UFMG/04-2-2016/JornalPaper/02-code/03-ros_ws_latpop_new/build/laptop_node/catkin_generated/installspace/laptop_node.pc")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/laptop_node/cmake" TYPE FILE FILES
    "/home/alexj/Dropbox/UFMG/04-2-2016/JornalPaper/02-code/03-ros_ws_latpop_new/build/laptop_node/catkin_generated/installspace/laptop_nodeConfig.cmake"
    "/home/alexj/Dropbox/UFMG/04-2-2016/JornalPaper/02-code/03-ros_ws_latpop_new/build/laptop_node/catkin_generated/installspace/laptop_nodeConfig-version.cmake"
    )
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/laptop_node" TYPE FILE FILES "/home/alexj/Dropbox/UFMG/04-2-2016/JornalPaper/02-code/03-ros_ws_latpop_new/src/laptop_node/package.xml")
endif()


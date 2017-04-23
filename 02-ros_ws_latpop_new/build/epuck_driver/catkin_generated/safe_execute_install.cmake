execute_process(COMMAND "/home/alexj/Dropbox/UFMG/04-2-2016/JornalPaper/02-code/03-ros_ws_latpop_new/build/epuck_driver/catkin_generated/python_distutils_install.sh" RESULT_VARIABLE res)

if(NOT res EQUAL 0)
  message(FATAL_ERROR "execute_process(/home/alexj/Dropbox/UFMG/04-2-2016/JornalPaper/02-code/03-ros_ws_latpop_new/build/epuck_driver/catkin_generated/python_distutils_install.sh) returned error code ")
endif()

# Install script for directory:  ~/Documents/FRI/3-letnik/RINS/DN3/Robotki/workspace/src/yujin_ocs/yocs_keyop

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX " ~/Documents/FRI/3-letnik/RINS/DN3/Robotki/workspace/install")
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

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES " ~/Documents/FRI/3-letnik/RINS/DN3/Robotki/workspace/build/yujin_ocs/yocs_keyop/catkin_generated/installspace/yocs_keyop.pc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/yocs_keyop/cmake" TYPE FILE FILES
    " ~/Documents/FRI/3-letnik/RINS/DN3/Robotki/workspace/build/yujin_ocs/yocs_keyop/catkin_generated/installspace/yocs_keyopConfig.cmake"
    " ~/Documents/FRI/3-letnik/RINS/DN3/Robotki/workspace/build/yujin_ocs/yocs_keyop/catkin_generated/installspace/yocs_keyopConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/yocs_keyop" TYPE FILE FILES " ~/Documents/FRI/3-letnik/RINS/DN3/Robotki/workspace/src/yujin_ocs/yocs_keyop/package.xml")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/yocs_keyop" TYPE DIRECTORY FILES " ~/Documents/FRI/3-letnik/RINS/DN3/Robotki/workspace/src/yujin_ocs/yocs_keyop/launch")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  include(" ~/Documents/FRI/3-letnik/RINS/DN3/Robotki/workspace/build/yujin_ocs/yocs_keyop/src/cmake_install.cmake")

endif()


cmake_minimum_required(VERSION 3.15)

project(irsl_dynamixel_hardware_shm
  VERSION 0.99.1
  LANGUAGES CXX
)

set(CMAKE_BUILD_TYPE RelWithDebInfo)

find_package(irsl_shm_libs       REQUIRED)
find_package(irsl_realtime_utils REQUIRED)
find_package(irsl_common_utils   REQUIRED)
#find_package(catkin REQUIRED COMPONENTS
#  dynamixel_workbench_toolbox
#)
find_package(dynamixel_workbench_toolbox REQUIRED)

### debug print variables
##get_cmake_property(_variableNames CACHE_VARIABLES)
#get_cmake_property(_variableNames VARIABLES)
#message("VARIABLES>>>>")
#list (SORT _variableNames)
#foreach (_variableName ${_variableNames})
#  message("${_variableName}=${${_variableName}}")
#endforeach()
#message("VARIABLES<<<<")


add_library(dynamixel_hw_shm SHARED src/dynamixel_hw_shm_lib.cpp src/DynamixelInterface.cpp)

target_include_directories(dynamixel_hw_shm PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${dynamixel_workbench_toolbox_INCLUDE_DIRS}
)

## one-make
add_executable(dynamixel_hardware_shm src/dynamixel_hardware_shm_main.cpp src/DynamixelInterface.cpp)
target_include_directories(dynamixel_hardware_shm PRIVATE
  include
  ${dynamixel_workbench_toolbox_INCLUDE_DIRS}
)
target_link_libraries(dynamixel_hardware_shm
  irsl_shm_libs::irsl_shm_libs
  irsl_realtime_utils::irsl_realtime_utils
  irsl_common_utils::irsl_common_utils
  ${dynamixel_workbench_toolbox_LIBRARIES}
)
install(TARGETS dynamixel_hardware_shm
  # EXPORT ${targetname}Targets
  LIBRARY       DESTINATION lib COMPONENT Runtime
  ARCHIVE       DESTINATION lib COMPONENT Development
  RUNTIME       DESTINATION bin COMPONENT Runtime
  PUBLIC_HEADER DESTINATION include COMPONENT Development
  BUNDLE        DESTINATION bin COMPONENT Runtime
)
## use shared-lib
add_executable(dynamixel_hardware_shm_wlib src/dynamixel_hardware_shm_wlib.cpp)
target_include_directories(dynamixel_hardware_shm_wlib PRIVATE
  include
)
target_link_libraries(dynamixel_hardware_shm_wlib
  dynamixel_hw_shm
  irsl_shm_libs::irsl_shm_libs
  irsl_realtime_utils::irsl_realtime_utils
  irsl_common_utils::irsl_common_utils
)
install(TARGETS dynamixel_hardware_shm_wlib
  # EXPORT ${targetname}Targets
  LIBRARY       DESTINATION lib COMPONENT Runtime
  ARCHIVE       DESTINATION lib COMPONENT Development
  RUNTIME       DESTINATION bin COMPONENT Runtime
  PUBLIC_HEADER DESTINATION include COMPONENT Development
  BUNDLE        DESTINATION bin COMPONENT Runtime
)

## install/inteface
set(targetname dynamixel_hardware_shm)
set(irsl_public_headers
  include/irsl_dynamixel_hardware_shm/DynamixelShmLib.h
)
set_target_properties(dynamixel_hw_shm PROPERTIES
  PUBLIC_HEADER "${irsl_public_headers}"
)

install(TARGETS dynamixel_hw_shm
  EXPORT irsl-dx-hw-export
  LIBRARY       DESTINATION lib COMPONENT Runtime
  ARCHIVE       DESTINATION lib COMPONENT Development
  RUNTIME       DESTINATION bin COMPONENT Runtime
  PUBLIC_HEADER DESTINATION include COMPONENT Development
  BUNDLE        DESTINATION bin COMPONENT Runtime
)

install(EXPORT irsl-dx-hw-export
  FILE ${targetname}Config.cmake
  DESTINATION lib/cmake/${targetname}
  NAMESPACE ${targetname}::
  EXPORT_LINK_INTERFACE_LIBRARIES
)

include(CMakePackageConfigHelpers)
write_basic_package_version_file(
  "${PROJECT_BINARY_DIR}/${targetname}ConfigVersion.cmake"
  # VERSION 1.0.1
  COMPATIBILITY AnyNewerVersion
)
install(FILES "${PROJECT_BINARY_DIR}/${targetname}ConfigVersion.cmake"
  DESTINATION lib/cmake/${targetname}
)

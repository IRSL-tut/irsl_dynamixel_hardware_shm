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

include_directories(
  include
  ${dynamixel_workbench_toolbox_INCLUDE_DIRS}
)

add_executable(dynamixel_hardware_shm src/DynamixelInterface.cpp src/dynamixel_hardware_shm.cpp)

target_link_libraries(dynamixel_hardware_shm
  #  ${YAML_CPP_LIBRARIES}
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

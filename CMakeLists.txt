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

set(lib_target dynamixel_hardware_shm)
add_library(${lib_target} SHARED src/dynamixel_hardware_shm_lib.cpp src/DynamixelInterface.cpp)
target_include_directories(${lib_target} PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${dynamixel_workbench_toolbox_INCLUDE_DIRS}
)
target_link_libraries(${lib_target} PRIVATE
  irsl_shm_libs::irsl_shm_libs
  irsl_realtime_utils::irsl_realtime_utils
  irsl_common_utils::irsl_common_utils
  ${dynamixel_workbench_toolbox_LIBRARIES}
)

### one-make
add_executable(one_new_exec src/dynamixel_hardware_shm_main_org.cpp src/DynamixelInterface.cpp)
set_target_properties(one_new_exec PROPERTIES
    OUTPUT_NAME "dynamixel_hardware_shm_org"
)
target_include_directories(one_new_exec PRIVATE
  include
  ${dynamixel_workbench_toolbox_INCLUDE_DIRS}
)
target_link_libraries(one_new_exec
  irsl_shm_libs::irsl_shm_libs
  irsl_realtime_utils::irsl_realtime_utils
  irsl_common_utils::irsl_common_utils
  ${dynamixel_workbench_toolbox_LIBRARIES}
)
# install executable
install(TARGETS one_new_exec
  # EXPORT ${lib_target}Targets
  LIBRARY       DESTINATION lib COMPONENT Runtime
  ARCHIVE       DESTINATION lib COMPONENT Development
  RUNTIME       DESTINATION bin COMPONENT Runtime
  PUBLIC_HEADER DESTINATION include COMPONENT Development
  BUNDLE        DESTINATION bin COMPONENT Runtime
)
### use shared-lib
add_executable(wlib_new_exec src/dynamixel_hardware_shm_main.cpp)
set_target_properties(wlib_new_exec PROPERTIES
    OUTPUT_NAME "dynamixel_hardware_shm"
)
target_include_directories(wlib_new_exec PRIVATE
  include
)
target_link_libraries(wlib_new_exec
  ${lib_target}
  irsl_shm_libs::irsl_shm_libs
  irsl_realtime_utils::irsl_realtime_utils
  irsl_common_utils::irsl_common_utils
)
# install executable
install(TARGETS wlib_new_exec
  # EXPORT ${lib_target}Targets
  LIBRARY       DESTINATION lib COMPONENT Runtime
  ARCHIVE       DESTINATION lib COMPONENT Development
  RUNTIME       DESTINATION bin COMPONENT Runtime
  PUBLIC_HEADER DESTINATION include COMPONENT Development
  BUNDLE        DESTINATION bin COMPONENT Runtime
)

## install/inteface
set(irsl_public_headers
  include/irsl_dynamixel_hardware_shm/DynamixelShmLib.h
)
set_target_properties(${lib_target} PROPERTIES
  PUBLIC_HEADER "${irsl_public_headers}"
)
install(TARGETS ${lib_target}
  EXPORT irsl-dx-hw-export
  LIBRARY       DESTINATION lib COMPONENT Runtime
  ARCHIVE       DESTINATION lib COMPONENT Development
  RUNTIME       DESTINATION bin COMPONENT Runtime
  PUBLIC_HEADER DESTINATION include/irsl_dynamixel_hardware_shm COMPONENT Development
  BUNDLE        DESTINATION bin COMPONENT Runtime
)
install(EXPORT irsl-dx-hw-export
  FILE ${lib_target}Config.cmake
  DESTINATION lib/cmake/${lib_target}
  NAMESPACE ${lib_target}::
  EXPORT_LINK_INTERFACE_LIBRARIES
)

include(CMakePackageConfigHelpers)
write_basic_package_version_file(
  "${PROJECT_BINARY_DIR}/${lib_target}ConfigVersion.cmake"
  # VERSION 1.0.1
  COMPATIBILITY AnyNewerVersion
)
install(FILES "${PROJECT_BINARY_DIR}/${lib_target}ConfigVersion.cmake"
  DESTINATION lib/cmake/${lib_target}
)

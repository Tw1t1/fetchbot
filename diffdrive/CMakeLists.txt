cmake_minimum_required(VERSION 3.16)
project(diffdrive LANGUAGES CXX)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra)
endif()

# set the same behavior for windows as it is on linux
set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)

# find dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

# find dependencies
find_package(backward_ros REQUIRED)
find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

# Specify the bcm2835 include and library directories
set(BCM2835_INCLUDE_DIR "/usr/local/include")
set(BCM2835_LIBRARY "/usr/local/lib/libbcm2835.a")

## COMPILE
add_library(
  diffdrive
  SHARED
  hardware/diffbot_system.cpp
)
target_compile_features(diffdrive PUBLIC cxx_std_17)
target_include_directories(diffdrive PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/hardware/include>
  $<INSTALL_INTERFACE:include/diffdrive>
  ${BCM2835_INCLUDE_DIR}  # Include bcm2835 headers
)
target_link_libraries(diffdrive PUBLIC
  ${BCM2835_LIBRARY}  # Link against bcm2835 library
)

ament_target_dependencies(
  diffdrive PUBLIC
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

# Export hardware plugins
pluginlib_export_plugin_description_file(hardware_interface diffdrive.xml)

# INSTALL
install(
  DIRECTORY hardware/include/
  DESTINATION include/diffdrive
)


install(TARGETS diffdrive
  EXPORT export_diffdrive
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
endif()

## EXPORTS
ament_export_targets(export_diffdrive HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()

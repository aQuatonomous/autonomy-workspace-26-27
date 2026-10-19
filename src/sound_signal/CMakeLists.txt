cmake_minimum_required(VERSION 3.8)
project(sound_signal)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(audio_common REQUIRED)
find_package(audio_common_msgs REQUIRED)
find_package(sound_signal_msgs REQUIRED)

add_executable(sound_signal src/sound_signal.cpp)
target_link_libraries(sound_signal fftw3)

install(TARGETS	
	sound_signal
	DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_target_dependencies(sound_signal rclcpp std_msgs audio_common audio_common_msgs sound_signal_msgs)

ament_package()

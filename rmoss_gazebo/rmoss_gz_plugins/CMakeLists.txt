cmake_minimum_required(VERSION 3.5)
project(rmoss_gz_plugins)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
# find package
find_package(ament_cmake REQUIRED)
find_package(gz-msgs10 REQUIRED)
find_package(gz-transport13 REQUIRED)
find_package(gz-sim8 REQUIRED)


# for plugins MecanumDrive2
add_library(MecanumDrive2 SHARED plugins/mecanum_drive2/MecanumDrive2.cc)

target_link_libraries(MecanumDrive2
  gz-sim8::gz-sim8
)

# for plugins ProjectileShooter
add_library(ProjectileShooter SHARED plugins/projectile_shooter/ProjectileShooter.cc)

target_link_libraries(ProjectileShooter
  gz-sim8::gz-sim8
)

# for plugins LightBarController
add_library(LightBarController SHARED plugins/light_bar_controller/LightBarController.cc)
target_link_libraries(LightBarController
  gz-sim8::gz-sim8
)


# Install plugins
install(TARGETS MecanumDrive2 ProjectileShooter LightBarController
    DESTINATION plugins
)

#environment
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/gazebo.dsv.in")

if(BUILD_TESTING)
  # no lint test
  # find_package(ament_lint_auto REQUIRED)
  # ament_lint_auto_find_test_dependencies()
endif()

ament_package()

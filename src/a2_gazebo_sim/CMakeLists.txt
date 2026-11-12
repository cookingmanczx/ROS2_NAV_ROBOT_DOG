cmake_minimum_required(VERSION 3.8)
project(a2_gazebo_sim)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)

install(DIRECTORY urdf worlds launch config
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY maps
  DESTINATION share/${PROJECT_NAME}
)

file(GLOB SCRIPT_FILES "${CMAKE_CURRENT_SOURCE_DIR}/scripts/*.py")
if(SCRIPT_FILES)
  install(PROGRAMS ${SCRIPT_FILES}
    DESTINATION lib/${PROJECT_NAME}
  )
endif()

ament_package()

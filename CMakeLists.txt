cmake_minimum_required(VERSION 3.5)
project(ouxt_behavior_descriptor_v1)

find_package(ament_cmake_auto REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(Lua REQUIRED)
include_directories(/usr/local/include ${LUA_INCLUDE_DIR})
ament_auto_find_build_dependencies()

#ament_auto_add_library(${PROJECT_NAME} SHARED src/main.cpp)
ament_auto_add_executable(${PROJECT_NAME}_node src/main.cpp)
message(${LUA_LIBRARIES})
target_link_libraries(${PROJECT_NAME}_node ${YAML_CPP_LIBRARIES} ${LUA_LIBRARIES})
install(DIRECTORY launch test
        DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package()
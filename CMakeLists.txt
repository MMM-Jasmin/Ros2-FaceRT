cmake_minimum_required(VERSION 3.5)
project(rtface_pkg)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()
# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-O3)
#  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
#set(TENSORRT_ROOT /home/dave/TensorRT-8.0.1.6/)

include_directories(include/${PROJECT_NAME})

# > --- find dependencies ---
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
find_package(rosidl_default_generators REQUIRED)
# find_package(<dependency> REQUIRED)
find_package(image_tools REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_geometry)
find_package(vision_opencv)
find_package(sensor_msgs REQUIRED)
#find_package(more_interfaces)
find_package(std_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(OpenCV REQUIRED)
find_package(message_filters REQUIRED)
find_package(sm_interfaces REQUIRED)
find_package(nlohmann_json REQUIRED)

set(msg_files
      "msg/IdImage.msg"
      "msg/ListIdImage.msg"
      "msg/IdName.msg"
      "msg/ListIdName.msg"
      "msg/IdMask.msg"
      "msg/ListIdMask.msg"
      "msg/IdEmotion.msg"
      "msg/ListIdEmotion.msg"
      "msg/NameListImage.msg"
      )

rosidl_generate_interfaces(${PROJECT_NAME}
      ${msg_files} DEPENDENCIES sensor_msgs
      )

ament_export_dependencies(rosidl_default_runtime)

option(CUDA_USE_STATIC_CUDA_RUNTIME OFF)

find_package(CUDA REQUIRED)

if (CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
  message("embed_platform on")
  include_directories(/usr/local/cuda/targets/aarch64-linux/include)
  link_directories(/usr/local/cuda/targets/aarch64-linux/lib)
else()
  message("embed_platform off")
  include_directories(/usr/local/cuda/include)
  link_directories(/usr/local/cuda/lib64)
endif()

if (CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
  message("Xavier installation path for face_RT set.")
  set(face_RT_DIR "/opt/dev/dependencies/RTface_lib/install/face_RT/")
  set(CMAKE_MODULE_PATH "/opt/dev/dependencies/RTface_lib/install/face_RT/cmake/")
  list(APPEND CMAKE_PREFIX_PATH "/opt/dev/dependencies/RTface_lib/install/face_RT/")
  find_library(DECODE_PLUGIN decodeplugin REQUIRED
        HINTS "/opt/dev/dependencies/RTface_lib/install/face_RT/lib/"
        PATH_SUFFIXES lib)
else()
  message("Home installation path for face_RT set.")
  set(face_RT_DIR "/opt/dev/dependencies/RTface_lib/install/face_RT/")
  set(CMAKE_MODULE_PATH "/opt/dev/dependencies/RTface_lib/install/face_RT/cmake/")
  find_library(DECODE_PLUGIN decodeplugin REQUIRED
        HINTS "/opt/dev/dependencies/RTface_lib/install/face_RT/lib/"
        PATH_SUFFIXES lib)
endif()


# /home/dave/uni/ma-drichter/face_RT/install/face_RT/cmake

# get_filename_component(INSTALL_PATH "${CMAKE_CURRENT_LIST_DIR}/../../install" ABSOLUTE)
#list(APPEND CMAKE_MODULE_PATH "/opt/dev/ma-drichter/face_RT/install/face_RT/cmake/")

# project (minimal_ros_package)
set(PROJECT_BINARY ${PROJECT_NAME})
#set(CMAKE_CXX_STANDARD 11)

find_library(TENSORRT_LIBRARY_INFER nvinfer REQUIRED
        HINTS ${TENSORRT_BUILD} ${CUDA_TOOLKIT_ROOT_DIR}
        PATH_SUFFIXES lib lib64 lib/x64 lib/aarch64-linux-gnu)
find_path(TENSORRT_INCLUDE_DIR NvInferPlugin.h
        HINTS  ${CUDA_TOOLKIT_ROOT_DIR}
        PATH_SUFFIXES include)
find_library(TENSORRT_LIBRARY_INFER_PLUGIN nvinfer_plugin
        HINTS   ${TENSORRT_BUILD} ${CUDA_TOOLKIT_ROOT_DIR}
        PATH_SUFFIXES lib lib64 lib/x64 lib/aarch64-linux-gnu)
find_library(TENSORRT_LIBRARY_PARSER nvparsers
      HINTS ${TENSORRT_BUILD} ${CUDA_TOOLKIT_ROOT_DIR}
      PATH_SUFFIXES lib lib64 lib/x64)
find_library(TENSORRT_LIB_ONNXPARSER nvonnxparser
        HINTS  ${TENSORRT_BUILD} ${CUDA_TOOLKIT_ROOT_DIR}
        PATH_SUFFIXES lib lib64 lib/x64)
find_library(TENSORRT_LIBRARY_INFER_PLUGIN onnxparser
        HINTS ${TENSORRT_BUILD} ${CUDA_TOOLKIT_ROOT_DIR}
        PATH_SUFFIXES lib lib64 lib/x64 lib/aarch64-linux-gnu)

#find_library(DECODE_PLUGIN decodeplugin
#      HINTS "/opt/dev/dependencies/RTface_lib/install/face_RT/lib/"
#      PATH_SUFFIXES lib)

set(TENSORRT_LIBRARY 
  ${TENSORRT_LIBRARY_INFER}
  ${TENSORRT_LIBRARY_INFER_PLUGIN}
  ${TENSORRT_LIBRARY_PARSER} 
  ${TENSORRT_LIB_ONNXPARSER}
)
# < --- includes for tensorRT ---

# > --- include implemented face-analysis-modules ---
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")
#find_package(rtface_pkg)
#find_package(RT REQUIRED)
#include_directories(${RT_INCLUDE_DIRS})
message("module_path1: " ${CMAKE_MODULE_PATH})

file(GLOB_RECURSE SOURCE_FILES ${PROJECT_SOURCE_DIR}/src/*.cpp)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/src)
#add_library(${PROJECT_BINARY} ${SOURCE_FILES})
#add_executable(${PROJECT_BINARY} ${SOURCE_FILES})

add_executable(detect_node src/detect_node.cpp)
add_executable(display_node src/display_node.cpp)
add_executable(recognize_node src/recognize_node.cpp)
add_executable(emotion_node src/emotion_node.cpp)
add_executable(mask_node src/mask_node.cpp)
add_executable(addPerson_node src/addPerson_node.cpp)
add_executable(cam_node src/cam_node.cpp)

#list(APPEND CMAKE_MODULE_PATH "/opt/dev/ma-drichter/face_RT/install/face_RT/cmake/")

message("module_path: " ${CMAKE_MODULE_PATH})
#set(mylib_DIR "${INSTALL_PATH}/mylib/cmake")
find_package(Boost COMPONENTS program_options REQUIRED)
find_package(face_RT)
if (face_RT_FOUND)
  message(#[=[STATUS]=] "face_RT_DIR = " ${face_RT_DIR})
  message(#[=[STATUS]=] "face_RT_INCLUDE_DIR = " ${face_RT_INCLUDE_DIR})
  message(#[=[STATUS]=] "face_RT_LIBRARY = " ${face_RT_LIBRARY})
  include_directories(${face_RT_INCLUDE_DIR})
  target_link_libraries(emotion_node
        ${Boost_LIBRARIES}
        ${face_RT_LIBRARY}
        ${TENSORRT_LIBRARY}
        ${OpenCV_LIBS}
        cudart
        stdc++fs)
  target_link_libraries(mask_node
        ${Boost_LIBRARIES}
        ${face_RT_LIBRARY}
        ${TENSORRT_LIBRARY}
        ${OpenCV_LIBS}
        cudart
        stdc++fs)
  target_link_libraries(recognize_node
        ${Boost_LIBRARIES}
        ${face_RT_LIBRARY}
        ${TENSORRT_LIBRARY}
        ${OpenCV_LIBS}
        cudart
        stdc++fs)
  target_link_libraries(detect_node
        ${Boost_LIBRARIES}
        ${face_RT_LIBRARY}
        ${TENSORRT_LIBRARY}
        ${DECODE_PLUGIN}
        ${OpenCV_LIBS}
        cudart
        stdc++fs)
  target_link_libraries(display_node
        ${OpenCV_LIBS})
  target_link_libraries(cam_node
        ${OpenCV_LIBS})
  target_link_libraries(addPerson_node
        ${OpenCV_LIBS})
endif (face_RT_FOUND)


# < --- include implemented face-analysis-modules ---

#include_directories(/opt/dev/ros2_ws/install/more_interfaces/include)


#find_package(more_interfaces REQUIRED)
ament_target_dependencies(
      emotion_node
      "rclcpp"
      "std_msgs"
      "sensor_msgs"
      "cv_bridge"
)
ament_target_dependencies(
      recognize_node
      "rclcpp"
      "std_msgs"
      "sensor_msgs"
      "cv_bridge"
)
ament_target_dependencies(
      mask_node
      "rclcpp"
      "std_msgs"
      "sensor_msgs"
      "cv_bridge"
)
ament_target_dependencies(
      detect_node
      "rclcpp"
      "std_msgs"
      "message_filters"
      "sm_interfaces"
      "sensor_msgs"
      "cv_bridge"
)
ament_target_dependencies(
      display_node
      "rclcpp"
      "std_msgs"
      "sensor_msgs"
      "cv_bridge"
)
ament_target_dependencies(
      addPerson_node
      "rclcpp"
      "std_msgs"
      "sensor_msgs"
      "cv_bridge"
)
ament_target_dependencies(
      cam_node
      "rclcpp"
      "std_msgs"
      "sensor_msgs"
      "cv_bridge"
)
#ament_target_dependencies(
#        ${PROJECT_BINARY}
#        "rclcpp"
#        "std_msgs"
#        "sensor_msgs"
#        "cv_bridge"
#)
#rosidl_generate_interfaces(${PROJECT_NAME}
#      #[[${msg_files}
#      DEPENDENCIES]] more_interfaces
#      )
#rosidl_generate_interfaces(detect_node
#       more_interfaces
#      )
#rosidl_generate_interfaces(recognize_node
#      more_interfaces
#      )

#rosidl_target_interfaces(detect_node "more_interfaces" "rosidl_typesupport_cpp")
#rosidl_target_interfaces(recognize_node "more_interfaces" "rosidl_typesupport_cpp")
#rosidl_target_interfaces(${PROJECT_BINARY} "more_interfaces" "rosidl_typesupport_cpp")

#install(
#  TARGETS ${PROJECT_BINARY}
#  DESTINATION lib/${PROJECT_NAME}
#)

install(
      TARGETS detect_node
      DESTINATION lib/${PROJECT_NAME}
)
install(
      TARGETS display_node
      DESTINATION lib/${PROJECT_NAME}
)
install(
      TARGETS cam_node
      DESTINATION lib/${PROJECT_NAME}
)
install(
      TARGETS recognize_node
      DESTINATION lib/${PROJECT_NAME}
)
install(
      TARGETS emotion_node
      DESTINATION lib/${PROJECT_NAME}
)
install(
      TARGETS mask_node
      DESTINATION lib/${PROJECT_NAME}
)
install(
      TARGETS addPerson_node
      DESTINATION lib/${PROJECT_NAME}
)
install(
      DIRECTORY launch
      DESTINATION share/${PROJECT_NAME}
)

rosidl_target_interfaces(detect_node
      ${PROJECT_NAME} "rosidl_typesupport_cpp")
rosidl_target_interfaces(recognize_node
      ${PROJECT_NAME} "rosidl_typesupport_cpp")
rosidl_target_interfaces(mask_node
      ${PROJECT_NAME} "rosidl_typesupport_cpp")
rosidl_target_interfaces(display_node
      ${PROJECT_NAME} "rosidl_typesupport_cpp")
rosidl_target_interfaces(cam_node
      ${PROJECT_NAME} "rosidl_typesupport_cpp")
rosidl_target_interfaces(emotion_node
      ${PROJECT_NAME} "rosidl_typesupport_cpp")
rosidl_target_interfaces(addPerson_node
      ${PROJECT_NAME} "rosidl_typesupport_cpp")

ament_package()

message("cmake_CXX_flags standard: " ${CMAKE_CXX_FLAGS})



#[=[
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()



# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

add_executable(my_node src/my_node.cpp)
target_include_directories(my_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

install(TARGETS my_node
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
#]=] 

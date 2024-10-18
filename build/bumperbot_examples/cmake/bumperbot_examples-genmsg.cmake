# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "bumperbot_examples: 0 messages, 2 services")

set(MSG_I_FLAGS "-Istd_msgs:/opt/ros/noetic/share/std_msgs/cmake/../msg;-Igeometry_msgs:/opt/ros/noetic/share/geometry_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(bumperbot_examples_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/keith/Documents/Capstone/Robot_Kepstone/Test_Robot/src/bumperbot_examples/srv/AddTwoInts.srv" NAME_WE)
add_custom_target(_bumperbot_examples_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "bumperbot_examples" "/home/keith/Documents/Capstone/Robot_Kepstone/Test_Robot/src/bumperbot_examples/srv/AddTwoInts.srv" ""
)

get_filename_component(_filename "/home/keith/Documents/Capstone/Robot_Kepstone/Test_Robot/src/bumperbot_examples/srv/GetTransform.srv" NAME_WE)
add_custom_target(_bumperbot_examples_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "bumperbot_examples" "/home/keith/Documents/Capstone/Robot_Kepstone/Test_Robot/src/bumperbot_examples/srv/GetTransform.srv" "geometry_msgs/Vector3:geometry_msgs/TransformStamped:geometry_msgs/Quaternion:std_msgs/Header:geometry_msgs/Transform"
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages

### Generating Services
_generate_srv_cpp(bumperbot_examples
  "/home/keith/Documents/Capstone/Robot_Kepstone/Test_Robot/src/bumperbot_examples/srv/AddTwoInts.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/bumperbot_examples
)
_generate_srv_cpp(bumperbot_examples
  "/home/keith/Documents/Capstone/Robot_Kepstone/Test_Robot/src/bumperbot_examples/srv/GetTransform.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/TransformStamped.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Transform.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/bumperbot_examples
)

### Generating Module File
_generate_module_cpp(bumperbot_examples
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/bumperbot_examples
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(bumperbot_examples_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(bumperbot_examples_generate_messages bumperbot_examples_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/keith/Documents/Capstone/Robot_Kepstone/Test_Robot/src/bumperbot_examples/srv/AddTwoInts.srv" NAME_WE)
add_dependencies(bumperbot_examples_generate_messages_cpp _bumperbot_examples_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/keith/Documents/Capstone/Robot_Kepstone/Test_Robot/src/bumperbot_examples/srv/GetTransform.srv" NAME_WE)
add_dependencies(bumperbot_examples_generate_messages_cpp _bumperbot_examples_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(bumperbot_examples_gencpp)
add_dependencies(bumperbot_examples_gencpp bumperbot_examples_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS bumperbot_examples_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages

### Generating Services
_generate_srv_eus(bumperbot_examples
  "/home/keith/Documents/Capstone/Robot_Kepstone/Test_Robot/src/bumperbot_examples/srv/AddTwoInts.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/bumperbot_examples
)
_generate_srv_eus(bumperbot_examples
  "/home/keith/Documents/Capstone/Robot_Kepstone/Test_Robot/src/bumperbot_examples/srv/GetTransform.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/TransformStamped.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Transform.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/bumperbot_examples
)

### Generating Module File
_generate_module_eus(bumperbot_examples
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/bumperbot_examples
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(bumperbot_examples_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(bumperbot_examples_generate_messages bumperbot_examples_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/keith/Documents/Capstone/Robot_Kepstone/Test_Robot/src/bumperbot_examples/srv/AddTwoInts.srv" NAME_WE)
add_dependencies(bumperbot_examples_generate_messages_eus _bumperbot_examples_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/keith/Documents/Capstone/Robot_Kepstone/Test_Robot/src/bumperbot_examples/srv/GetTransform.srv" NAME_WE)
add_dependencies(bumperbot_examples_generate_messages_eus _bumperbot_examples_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(bumperbot_examples_geneus)
add_dependencies(bumperbot_examples_geneus bumperbot_examples_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS bumperbot_examples_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages

### Generating Services
_generate_srv_lisp(bumperbot_examples
  "/home/keith/Documents/Capstone/Robot_Kepstone/Test_Robot/src/bumperbot_examples/srv/AddTwoInts.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/bumperbot_examples
)
_generate_srv_lisp(bumperbot_examples
  "/home/keith/Documents/Capstone/Robot_Kepstone/Test_Robot/src/bumperbot_examples/srv/GetTransform.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/TransformStamped.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Transform.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/bumperbot_examples
)

### Generating Module File
_generate_module_lisp(bumperbot_examples
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/bumperbot_examples
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(bumperbot_examples_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(bumperbot_examples_generate_messages bumperbot_examples_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/keith/Documents/Capstone/Robot_Kepstone/Test_Robot/src/bumperbot_examples/srv/AddTwoInts.srv" NAME_WE)
add_dependencies(bumperbot_examples_generate_messages_lisp _bumperbot_examples_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/keith/Documents/Capstone/Robot_Kepstone/Test_Robot/src/bumperbot_examples/srv/GetTransform.srv" NAME_WE)
add_dependencies(bumperbot_examples_generate_messages_lisp _bumperbot_examples_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(bumperbot_examples_genlisp)
add_dependencies(bumperbot_examples_genlisp bumperbot_examples_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS bumperbot_examples_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages

### Generating Services
_generate_srv_nodejs(bumperbot_examples
  "/home/keith/Documents/Capstone/Robot_Kepstone/Test_Robot/src/bumperbot_examples/srv/AddTwoInts.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/bumperbot_examples
)
_generate_srv_nodejs(bumperbot_examples
  "/home/keith/Documents/Capstone/Robot_Kepstone/Test_Robot/src/bumperbot_examples/srv/GetTransform.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/TransformStamped.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Transform.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/bumperbot_examples
)

### Generating Module File
_generate_module_nodejs(bumperbot_examples
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/bumperbot_examples
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(bumperbot_examples_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(bumperbot_examples_generate_messages bumperbot_examples_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/keith/Documents/Capstone/Robot_Kepstone/Test_Robot/src/bumperbot_examples/srv/AddTwoInts.srv" NAME_WE)
add_dependencies(bumperbot_examples_generate_messages_nodejs _bumperbot_examples_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/keith/Documents/Capstone/Robot_Kepstone/Test_Robot/src/bumperbot_examples/srv/GetTransform.srv" NAME_WE)
add_dependencies(bumperbot_examples_generate_messages_nodejs _bumperbot_examples_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(bumperbot_examples_gennodejs)
add_dependencies(bumperbot_examples_gennodejs bumperbot_examples_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS bumperbot_examples_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages

### Generating Services
_generate_srv_py(bumperbot_examples
  "/home/keith/Documents/Capstone/Robot_Kepstone/Test_Robot/src/bumperbot_examples/srv/AddTwoInts.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/bumperbot_examples
)
_generate_srv_py(bumperbot_examples
  "/home/keith/Documents/Capstone/Robot_Kepstone/Test_Robot/src/bumperbot_examples/srv/GetTransform.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/TransformStamped.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Transform.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/bumperbot_examples
)

### Generating Module File
_generate_module_py(bumperbot_examples
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/bumperbot_examples
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(bumperbot_examples_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(bumperbot_examples_generate_messages bumperbot_examples_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/keith/Documents/Capstone/Robot_Kepstone/Test_Robot/src/bumperbot_examples/srv/AddTwoInts.srv" NAME_WE)
add_dependencies(bumperbot_examples_generate_messages_py _bumperbot_examples_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/keith/Documents/Capstone/Robot_Kepstone/Test_Robot/src/bumperbot_examples/srv/GetTransform.srv" NAME_WE)
add_dependencies(bumperbot_examples_generate_messages_py _bumperbot_examples_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(bumperbot_examples_genpy)
add_dependencies(bumperbot_examples_genpy bumperbot_examples_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS bumperbot_examples_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/bumperbot_examples)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/bumperbot_examples
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(bumperbot_examples_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()
if(TARGET geometry_msgs_generate_messages_cpp)
  add_dependencies(bumperbot_examples_generate_messages_cpp geometry_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/bumperbot_examples)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/bumperbot_examples
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(bumperbot_examples_generate_messages_eus std_msgs_generate_messages_eus)
endif()
if(TARGET geometry_msgs_generate_messages_eus)
  add_dependencies(bumperbot_examples_generate_messages_eus geometry_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/bumperbot_examples)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/bumperbot_examples
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(bumperbot_examples_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()
if(TARGET geometry_msgs_generate_messages_lisp)
  add_dependencies(bumperbot_examples_generate_messages_lisp geometry_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/bumperbot_examples)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/bumperbot_examples
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(bumperbot_examples_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()
if(TARGET geometry_msgs_generate_messages_nodejs)
  add_dependencies(bumperbot_examples_generate_messages_nodejs geometry_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/bumperbot_examples)
  install(CODE "execute_process(COMMAND \"/usr/bin/python3\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/bumperbot_examples\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/bumperbot_examples
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(bumperbot_examples_generate_messages_py std_msgs_generate_messages_py)
endif()
if(TARGET geometry_msgs_generate_messages_py)
  add_dependencies(bumperbot_examples_generate_messages_py geometry_msgs_generate_messages_py)
endif()

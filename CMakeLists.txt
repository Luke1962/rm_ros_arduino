cmake_minimum_required(VERSION 2.8.3)
project(rm_ros_arduino)

#set (CMAKE_BUILD_TYPE Debug)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages



## System dependencies are found with CMake's conventions
find_package(catkin REQUIRED COMPONENTS roscpp  std_msgs
	serial 
	
	)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a exec_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a exec_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
catkin_package(
  CATKIN_DEPENDS
    serial
    std_msgs
)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs  # Or other packages containing msgs
# )

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES rm_ros_arduino
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
	CATKIN_DEPENDS serial 
	
)

###########
## Build ##
###########

include_directories(${CMAKE_SOURCE_DIR}/yaml-cpp/include)




## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include ${catkin_INCLUDE_DIRS}
  /usr/local/include/yaml-cpp
# include
# ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/rm_ros_arduino.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
# add_executable(${PROJECT_NAME}_node src/rm_ros_arduino_node.cpp)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
# )

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
# install(TARGETS ${PROJECT_NAME}_node
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark libraries for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
# install(TARGETS ${PROJECT_NAME}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_rm_ros_arduino.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)


add_executable(ros_arduino_node
  src/ros_arduino_node.cpp
)
add_dependencies(ros_arduino_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(ros_arduino_node
  ${catkin_LIBRARIES}
  yaml-cpp
  ${mraa_LIBRARIES}
)


# -- yaml-cpp Install configuration: ""
# -- Installing: /usr/local/lib/libyaml-cpp.a
# -- Up-to-date: /usr/local/include
# -- Installing: /usr/local/include/yaml-cpp
# -- Installing: /usr/local/include/yaml-cpp/eventhandler.h
# -- Installing: /usr/local/include/yaml-cpp/stlemitter.h
# -- Installing: /usr/local/include/yaml-cpp/mark.h
# -- Installing: /usr/local/include/yaml-cpp/exceptions.h
# -- Installing: /usr/local/include/yaml-cpp/binary.h
# -- Installing: /usr/local/include/yaml-cpp/emitterdef.h
# -- Installing: /usr/local/include/yaml-cpp/node
# -- Installing: /usr/local/include/yaml-cpp/node/node.h
# -- Installing: /usr/local/include/yaml-cpp/node/convert.h
# -- Installing: /usr/local/include/yaml-cpp/node/detail
# -- Installing: /usr/local/include/yaml-cpp/node/detail/iterator_fwd.h
# -- Installing: /usr/local/include/yaml-cpp/node/detail/memory.h
# -- Installing: /usr/local/include/yaml-cpp/node/detail/node.h
# -- Installing: /usr/local/include/yaml-cpp/node/detail/node_iterator.h
# -- Installing: /usr/local/include/yaml-cpp/node/detail/impl.h
# -- Installing: /usr/local/include/yaml-cpp/node/detail/iterator.h
# -- Installing: /usr/local/include/yaml-cpp/node/detail/node_data.h
# -- Installing: /usr/local/include/yaml-cpp/node/detail/node_ref.h
# -- Installing: /usr/local/include/yaml-cpp/node/emit.h
# -- Installing: /usr/local/include/yaml-cpp/node/impl.h
# -- Installing: /usr/local/include/yaml-cpp/node/iterator.h
# -- Installing: /usr/local/include/yaml-cpp/node/ptr.h
# -- Installing: /usr/local/include/yaml-cpp/node/type.h
# -- Installing: /usr/local/include/yaml-cpp/node/parse.h
# -- Installing: /usr/local/include/yaml-cpp/emitfromevents.h
# -- Installing: /usr/local/include/yaml-cpp/null.h
# -- Installing: /usr/local/include/yaml-cpp/yaml.h
# -- Installing: /usr/local/include/yaml-cpp/parser.h
# -- Installing: /usr/local/include/yaml-cpp/emitter.h
# -- Installing: /usr/local/include/yaml-cpp/anchor.h
# -- Installing: /usr/local/include/yaml-cpp/emittermanip.h
# -- Installing: /usr/local/include/yaml-cpp/contrib
# -- Installing: /usr/local/include/yaml-cpp/contrib/graphbuilder.h
# -- Installing: /usr/local/include/yaml-cpp/contrib/anchordict.h
# -- Installing: /usr/local/include/yaml-cpp/emitterstyle.h
# -- Installing: /usr/local/include/yaml-cpp/traits.h
# -- Installing: /usr/local/include/yaml-cpp/dll.h
# -- Installing: /usr/local/include/yaml-cpp/ostream_wrapper.h
# -- Installing: /usr/local/share/cmake/yaml-cpp/yaml-cpp-targets.cmake
# -- Installing: /usr/local/share/cmake/yaml-cpp/yaml-cpp-targets-noconfig.cmake
# -- Installing: /usr/local/share/cmake/yaml-cpp/yaml-cpp-config.cmake
# -- Installing: /usr/local/share/cmake/yaml-cpp/yaml-cpp-config-version.cmake
# -- Installing: /usr/local/share/pkgconfig/yaml-cpp.pc
# -- Installing: /usr/local/lib/libgmock.a
# -- Installing: /usr/local/lib/libgmock_main.a
# -- Installing: /usr/local/include/gmock
# -- Installing: /usr/local/include/gmock/gmock-generated-nice-strict.h
# -- Installing: /usr/local/include/gmock/gmock-more-matchers.h
# -- Installing: /usr/local/include/gmock/internal
# -- Installing: /usr/local/include/gmock/internal/gmock-generated-internal-utils.h
# -- Installing: /usr/local/include/gmock/internal/gmock-generated-internal-utils.h.pump
# -- Installing: /usr/local/include/gmock/internal/gmock-port.h
# -- Installing: /usr/local/include/gmock/internal/custom
# -- Installing: /usr/local/include/gmock/internal/custom/gmock-generated-actions.h.pump
# -- Installing: /usr/local/include/gmock/internal/custom/gmock-matchers.h
# -- Installing: /usr/local/include/gmock/internal/custom/gmock-port.h
# -- Installing: /usr/local/include/gmock/internal/custom/gmock-generated-actions.h
# -- Installing: /usr/local/include/gmock/internal/gmock-internal-utils.h
# -- Installing: /usr/local/include/gmock/gmock-more-actions.h
# -- Installing: /usr/local/include/gmock/gmock-generated-function-mockers.h
# -- Installing: /usr/local/include/gmock/gmock-spec-builders.h
# -- Installing: /usr/local/include/gmock/gmock-generated-actions.h.pump
# -- Installing: /usr/local/include/gmock/gmock-generated-matchers.h.pump
# -- Installing: /usr/local/include/gmock/gmock-cardinalities.h
# -- Installing: /usr/local/include/gmock/gmock-matchers.h
# -- Installing: /usr/local/include/gmock/gmock-generated-nice-strict.h.pump
# -- Installing: /usr/local/include/gmock/gmock-generated-function-mockers.h.pump
# -- Installing: /usr/local/include/gmock/gmock-generated-matchers.h
# -- Installing: /usr/local/include/gmock/gmock.h
# -- Installing: /usr/local/include/gmock/gmock-generated-actions.h
# -- Installing: /usr/local/include/gmock/gmock-actions.h
# -- Installing: /usr/local/lib/libgtest.a
# -- Installing: /usr/local/lib/libgtest_main.a
# -- Installing: /usr/local/include/gtest
# -- Installing: /usr/local/include/gtest/gtest_pred_impl.h
# -- Installing: /usr/local/include/gtest/gtest_prod.h
# -- Installing: /usr/local/include/gtest/gtest-param-test.h
# -- Installing: /usr/local/include/gtest/gtest-message.h
# -- Installing: /usr/local/include/gtest/internal
# -- Installing: /usr/local/include/gtest/internal/gtest-port.h
# -- Installing: /usr/local/include/gtest/internal/gtest-type-util.h
# -- Installing: /usr/local/include/gtest/internal/gtest-type-util.h.pump
# -- Installing: /usr/local/include/gtest/internal/gtest-death-test-internal.h
# -- Installing: /usr/local/include/gtest/internal/gtest-string.h
# -- Installing: /usr/local/include/gtest/internal/gtest-port-arch.h
# -- Installing: /usr/local/include/gtest/internal/gtest-param-util.h
# -- Installing: /usr/local/include/gtest/internal/gtest-filepath.h
# -- Installing: /usr/local/include/gtest/internal/gtest-internal.h
# -- Installing: /usr/local/include/gtest/internal/gtest-param-util-generated.h
# -- Installing: /usr/local/include/gtest/internal/gtest-tuple.h
# -- Installing: /usr/local/include/gtest/internal/gtest-param-util-generated.h.pump
# -- Installing: /usr/local/include/gtest/internal/gtest-tuple.h.pump
# -- Installing: /usr/local/include/gtest/internal/custom
# -- Installing: /usr/local/include/gtest/internal/custom/gtest-port.h
# -- Installing: /usr/local/include/gtest/internal/custom/gtest.h
# -- Installing: /usr/local/include/gtest/internal/custom/gtest-printers.h
# -- Installing: /usr/local/include/gtest/internal/gtest-linked_ptr.h
# -- Installing: /usr/local/include/gtest/gtest-death-test.h
# -- Installing: /usr/local/include/gtest/gtest-test-part.h
# -- Installing: /usr/local/include/gtest/gtest-typed-test.h
# -- Installing: /usr/local/include/gtest/gtest-param-test.h.pump
# -- Installing: /usr/local/include/gtest/gtest-spi.h
# -- Installing: /usr/local/include/gtest/gtest.h
# -- Installing: /usr/local/include/gtest/gtest-printers.h


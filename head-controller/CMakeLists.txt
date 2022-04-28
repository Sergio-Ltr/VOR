# set up the program
add_executable(head-controller)

# declare source files
target_sources(head-controller PRIVATE main.cpp)

# link with the YARP libraries
target_link_libraries(head-controller PRIVATE YARP::YARP_os
                                          YARP::YARP_init
                                          YARP::YARP_sig)

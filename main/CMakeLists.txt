add_executable(postEcl main.cpp csvparser.c)

target_link_libraries(postEcl ecl_EKF)
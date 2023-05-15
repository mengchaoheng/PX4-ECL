add_executable(postEcl main.cpp csvparser.cpp PostEkf.cpp PreFlightChecker.cpp)

target_link_libraries(postEcl ecl_EKF)
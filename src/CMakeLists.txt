rock_library(motors_elmo_ds402
    SOURCES Dummy.cpp
    HEADERS Dummy.hpp)

rock_executable(motors_elmo_ds402_bin Main.cpp
    DEPS motors_elmo_ds402)


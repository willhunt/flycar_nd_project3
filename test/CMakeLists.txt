cmake_minimum_required(VERSION 3.5)

set (This TestQuadControl)

set(CMAKE_CXX_STANDARD 11)

set(SOURCES
    TestQuadControl.cpp
)

set(FILES
    ../src/QuadControl.h
    ../src/QuadControl.cpp
)

find_package(GTest REQUIRED)

add_executable(${This} ${SOURCES} ${FILES})
target_link_libraries(${This} PUBLIC    
                    CPPSim          
                    #   ${GTEST_LIBRARIES}
                    #   pthread
                    GTest::Main
                    )



# include_directories(${GTEST_INCLUDE_DIRS})


add_test(
        NAME ${This}
        COMMAND ${This}
)
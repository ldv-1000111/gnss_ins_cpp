find_package(GTest REQUIRED)

add_executable(test_io test_io.cpp)
target_link_libraries(test_io PRIVATE gnss_ins GTest::gtest_main)
target_compile_options(test_io PRIVATE
    $<$<CXX_COMPILER_ID:GNU,Clang>:-Wall -Wextra>
)
add_test(NAME ProfileIO COMMAND test_io
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR})

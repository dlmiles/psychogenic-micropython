#
# This build cmake CMakeLists.txt file is for desktop debugging only, not to
# be used for bulding firmware image.
#
# Taken from GoogleTest QuickStart https://google.github.io/googletest/quickstart-cmake.html
#
#
#
#  TODO
#   options to build on Linux PC in 32bit mode (which better represents target platform)
#
#  MPY need to exclude ./build-gtest/**
#
#
cmake_minimum_required(VERSION 3.14)

# CMake diagnostics
#set(CMAKE_VERBOSE_MAKEFILE TRUE)
#set(CMAKE_RULE_MESSAGES ON)

# GoogleTest requires at least C++17
# we use: C++ designated initializers from c++20
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# -Wno-unused
# We don't care about the test C++ code having unused things
set(GCC_CXX_FLAGS_WARNS     "-Wall -Wextra -Wpedantic -Werror -Wno-unused")
# We care about the C driver having unused things, as that could be a bug
set(GCC_C_FLAGS_WARNS       "-Wall -Wextra -Wpedantic -Werror")

set(CMAKE_CXX_FLAGS         ${GCC_CXX_FLAGS_WARNS})
set(CMAKE_CXX_FLAGS_DEBUG   "-g3")
set(CMAKE_CXX_FLAGS_RELEASE "-O2")

# C23 try for constexpr
set(CMAKE_C_STANDARD 23)
set(CMAKE_C_STANDARD_REQUIRED ON)

set(CMAKE_C_FLAGS           ${GCC_C_FLAGS_WARNS})
set(CMAKE_C_FLAGS_DEBUG     "-g3")
set(CMAKE_C_FLAGS_RELEASE   "-O2")

if(NOT DEFINED CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE DEBUG)
endif()

###
###
###

include(FetchContent)

FetchContent_Declare(
  googletest
  URL https://github.com/google/googletest/archive/03597a01ee50ed33e9dfd640b249b4be3799d395.zip
)
# For Windows: Prevent overriding the parent project's compiler/linker settings
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)


FetchContent_Declare(fmt
  GIT_REPOSITORY https://github.com/fmtlib/fmt.git
  GIT_TAG master
)
FetchContent_MakeAvailable(fmt)


#set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

if(CMAKE_BUILD_TYPE STREQUAL "Coverage")
   message("Coverage enabled")
   #include(CodeCoverage)
   #setup_target_for_coverage(${PROJECT_NAME}_coverage ${TEST_TARGET} coverage)

   #set(CMAKE_CXX_FLAGS  "${GCC_CXX_FLAGS_WARNS} -g -O0 -fprofile-arcs -ftest-coverage")
   #set(CMAKE_C_FLAGS    "${GCC_C_FLAGS_WARNS} -g -O0 -fprofile-arcs -ftest-coverage")
endif()

###
###
###
###
###

project(test_i2cslave VERSION 1 LANGUAGES C CXX)

find_package(Threads REQUIRED) # for pthread

enable_testing()

message("CMAKE_BUILD_TYPE = ${CMAKE_BUILD_TYPE}")
message("PROJECT_NAME = ${PROJECT_NAME}")

add_executable(${PROJECT_NAME}
  fake_rp2040_api.cpp
  test_debuglog.cpp
  test_i2cslave_lib.cpp
  lib/i2cslave_sock_seqpacket.c
)

if(CMAKE_BUILD_TYPE STREQUAL "Coverage")
   # https://gcc.gnu.org/bugzilla/show_bug.cgi?id=68080 -fprofile-update=atomic
   set(CMAKE_CXX_FLAGS  "${GCC_CXX_FLAGS_WARNS} -g -O0 -fprofile-arcs -ftest-coverage -fprofile-update=atomic")
   set(CMAKE_C_FLAGS    "${GCC_C_FLAGS_WARNS} -g -O0 -fprofile-arcs -ftest-coverage -fprofile-update=atomic")

   target_compile_options(${PROJECT_NAME} PRIVATE --coverage)
   target_link_options(${PROJECT_NAME} PRIVATE --coverage)
   set(GCOV_LINK_LIBRARIES gcov)
endif()

# This enabled the subject module being tested include faked API
target_compile_definitions(${PROJECT_NAME} PRIVATE TEST_WITH_FAKE_RP2040)

target_include_directories(${PROJECT_NAME}
  PRIVATE
    .
)
target_link_libraries(${PROJECT_NAME}
  GTest::gtest_main
  fmt::fmt
  Threads::Threads
  ${GCOV_LINK_LIBRARIES}
)

include(GoogleTest)
gtest_discover_tests(${PROJECT_NAME})

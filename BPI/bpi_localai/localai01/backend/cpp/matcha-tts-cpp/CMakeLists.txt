cmake_minimum_required(VERSION 3.16)
project(matcha_tts_cpp VERSION 1.0.0)
set(_PROTOBUF_LIBPROTOBUF libprotobuf)
set(_REFLECTION grpc++_reflection)

# Use GCC-14 for better RISC-V support if available
find_program(GCC14 gcc-14)
find_program(GXX14 g++-14)

if(GCC14 AND GXX14)
    set(CMAKE_C_COMPILER gcc-14)
    set(CMAKE_CXX_COMPILER g++-14)
    message(STATUS "Using GCC-14 for better RISC-V support")
else()
    message(STATUS "GCC-14 not found, using default compiler")
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Set default build type to Release if not specified
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Choose the type of build (Debug/Release/RelWithDebInfo/MinSizeRel)" FORCE)
endif()

# Ensure Release mode has proper optimization flags
if(CMAKE_BUILD_TYPE STREQUAL "Release")
    set(CMAKE_CXX_FLAGS_RELEASE "-O3 -DNDEBUG" CACHE STRING "" FORCE)
endif()


find_package(absl CONFIG REQUIRED)
find_package(Protobuf CONFIG REQUIRED)
find_package(gRPC CONFIG REQUIRED)

find_program(_PROTOBUF_PROTOC protoc)
set(_GRPC_GRPCPP grpc++)
find_program(_GRPC_CPP_PLUGIN_EXECUTABLE grpc_cpp_plugin)

include_directories(${CMAKE_CURRENT_BINARY_DIR})
include_directories(${Protobuf_INCLUDE_DIRS})

# Proto file
get_filename_component(hw_proto "../../../backend/backend.proto" ABSOLUTE)
get_filename_component(hw_proto_path "${hw_proto}" PATH)

# Generated sources
set(hw_proto_srcs "${CMAKE_CURRENT_BINARY_DIR}/backend.pb.cc")
set(hw_proto_hdrs "${CMAKE_CURRENT_BINARY_DIR}/backend.pb.h")
set(hw_grpc_srcs "${CMAKE_CURRENT_BINARY_DIR}/backend.grpc.pb.cc")
set(hw_grpc_hdrs "${CMAKE_CURRENT_BINARY_DIR}/backend.grpc.pb.h")

add_custom_command(
      OUTPUT "${hw_proto_srcs}" "${hw_proto_hdrs}" "${hw_grpc_srcs}" "${hw_grpc_hdrs}"
      COMMAND ${_PROTOBUF_PROTOC}
      ARGS --grpc_out "${CMAKE_CURRENT_BINARY_DIR}"
        --cpp_out "${CMAKE_CURRENT_BINARY_DIR}"
        -I "${hw_proto_path}"
        --plugin=protoc-gen-grpc="${_GRPC_CPP_PLUGIN_EXECUTABLE}"
        "${hw_proto}"
      DEPENDS "${hw_proto}")

# hw_grpc_proto
add_library(hw_grpc_proto
  ${hw_grpc_srcs}
  ${hw_grpc_hdrs}
  ${hw_proto_srcs}
  ${hw_proto_hdrs} )


# RISC-V specific flags for memory alignment
if(CMAKE_SYSTEM_PROCESSOR MATCHES "riscv|RISCV")
    message(STATUS "Detected RISC-V architecture, adding alignment flags")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -mstrict-align -fno-aggressive-loop-optimizations")
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -mstrict-align -fno-aggressive-loop-optimizations")
    # Disable SIMD optimizations that may not be available on RISC-V
    add_definitions(-DFFTW_NO_SIMD)
endif()



# Find required packages
find_package(PkgConfig REQUIRED)


# Find libsndfile
pkg_check_modules(SNDFILE REQUIRED sndfile)

# Find ONNX Runtime
find_path(ONNXRUNTIME_INCLUDE_DIR
    NAMES onnxruntime_cxx_api.h
    PATHS 
    /usr/local/include/onnxruntime
    /usr/include/onnxruntime
    /opt/homebrew/include/onnxruntime
)

find_library(ONNXRUNTIME_LIB
    NAMES onnxruntime
    PATHS 
    /usr/local/lib
    /usr/lib
    /opt/homebrew/lib
)

if(NOT ONNXRUNTIME_INCLUDE_DIR OR NOT ONNXRUNTIME_LIB)
    message(FATAL_ERROR "ONNX Runtime not found. Please install ONNX Runtime C++ library.")
endif()

# Find cURL for downloading models
find_package(CURL REQUIRED)

# Find FFTW3
pkg_check_modules(FFTW3 REQUIRED fftw3f)


# Find libarchive for tar.gz extraction (optional)
find_package(LibArchive QUIET)
if(NOT LibArchive_FOUND)
    message(WARNING "libarchive not found. Using system tar command for extraction.")
endif()

# Use bundled cppjieba mock for now
set(JIEBA_INCLUDE_DIR ${CMAKE_SOURCE_DIR}/cppjieba/include)
set(JIEBA_DEPS_DIR ${CMAKE_SOURCE_DIR}/cppjieba/deps)
set(JIEBA_DICT_DIR ${CMAKE_SOURCE_DIR}/cppjieba/dict)


# Include directories
include_directories(${CMAKE_SOURCE_DIR}/include)
include_directories(${ONNXRUNTIME_INCLUDE_DIR})
include_directories(${SNDFILE_INCLUDE_DIRS})
include_directories(${FFTW3_INCLUDE_DIRS})

if(LibArchive_FOUND)
    include_directories(${LibArchive_INCLUDE_DIRS})
endif()

if(JIEBA_INCLUDE_DIR)
    include_directories(${JIEBA_INCLUDE_DIR})
    include_directories(${JIEBA_DEPS_DIR}/limonp/include)
endif()


# TTS source files
set(TTS_SOURCES
    tts_model.cpp
    tts_model_downloader.cpp
    tts_demo.cpp
)


# TTS executable (standalone)
set(TTS_STANDALONE_SOURCES ${TTS_SOURCES} grpc-server.cpp)
add_executable(grpc-server ${TTS_STANDALONE_SOURCES})

# Common libraries
set(COMMON_LIBRARIES
    ${ONNXRUNTIME_LIB}
    ${PORTAUDIO_LIBRARIES}
    ${SNDFILE_LIBRARIES}
    ${CURL_LIBRARIES}
    ${FFTW3_LIBRARIES}
    pthread
)


target_link_libraries(grpc-server ${COMMON_LIBRARIES})
target_link_libraries(grpc-server hw_grpc_proto)
target_link_libraries(grpc-server ${CMAKE_THREAD_LIBS_INIT} absl::flags hw_grpc_proto
  absl::flags_parse
  gRPC::${_REFLECTION}
  gRPC::${_GRPC_GRPCPP}
  protobuf::${_PROTOBUF_LIBPROTOBUF})


target_link_directories(grpc-server PRIVATE ${SNDFILE_LIBRARY_DIRS})


# Compiler flags for all executables
target_compile_options(grpc-server PRIVATE ${SNDFILE_CFLAGS_OTHER})


# Set output directory for all executables
set_target_properties(grpc-server PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin
)

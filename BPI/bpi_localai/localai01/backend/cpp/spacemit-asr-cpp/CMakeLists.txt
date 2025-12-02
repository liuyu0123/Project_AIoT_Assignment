cmake_minimum_required(VERSION 3.16)
project(spacemit_asr_cpp VERSION 1.0.0)
set(_PROTOBUF_LIBPROTOBUF libprotobuf)
set(_REFLECTION grpc++_reflection)

# Use available GCC compiler
set(CMAKE_C_COMPILER gcc-14)
set(CMAKE_CXX_COMPILER g++-14)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Set default build type to Release if not specified
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Choose the type of build (Debug/Release/RelWithDebInfo/MinSizeRel)" FORCE)
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

# Ensure Release mode has proper optimization flags
if(CMAKE_BUILD_TYPE STREQUAL "Release")
    set(CMAKE_CXX_FLAGS_RELEASE "-O3 -DNDEBUG" CACHE STRING "" FORCE)
endif()

# Find required packages
find_package(PkgConfig REQUIRED)


# ==== 关键修改 1: 添加 FFmpeg 支持 ====
# 使用 pkg-config 查找 FFmpeg 组件
pkg_check_modules(AVCODEC REQUIRED libavcodec)
pkg_check_modules(AVFORMAT REQUIRED libavformat)
pkg_check_modules(AVUTIL REQUIRED libavutil)
pkg_check_modules(SWRESAMPLE REQUIRED libswresample)

# 合并 FFmpeg 库
set(FFMPEG_LIBRARIES
    ${AVCODEC_LIBRARIES}
    ${AVFORMAT_LIBRARIES}
    ${AVUTIL_LIBRARIES}
    ${SWRESAMPLE_LIBRARIES}
)

# 合并 FFmpeg 包含目录
set(FFMPEG_INCLUDE_DIRS
    ${AVCODEC_INCLUDE_DIRS}
    ${AVFORMAT_INCLUDE_DIRS}
    ${AVUTIL_INCLUDE_DIRS}
    ${SWRESAMPLE_INCLUDE_DIRS}
)


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

# Include directories
include_directories(${CMAKE_SOURCE_DIR}/include)
include_directories(${ONNXRUNTIME_INCLUDE_DIR})
include_directories(${SNDFILE_INCLUDE_DIRS})
include_directories(${FFTW3_INCLUDE_DIRS})
include_directories(${FFMPEG_INCLUDE_DIRS})  # 添加 FFmpeg 头文件路径

# Common source files (shared between executables)
set(COMMON_SOURCES
    asr_model.cpp
    audio_processor.cpp
    tokenizer.cpp
    model_downloader.cpp
)

# ASE-specific source files (no audio_recorder, vad_detector needed)
set(ASE_SOURCES
    asr_model.cpp
    audio_processor.cpp
    tokenizer.cpp
    model_downloader.cpp
    ase.cpp
    grpc-server.cpp
)

# ASR-only executable
set(ASR_SOURCES ${COMMON_SOURCES} grpc-server.cpp)
add_executable(spacemit-asr-cpp ${ASR_SOURCES})

# Common libraries
set(COMMON_LIBRARIES
    ${ONNXRUNTIME_LIB}
    ${SNDFILE_LIBRARIES}
    ${CURL_LIBRARIES}
    ${FFTW3_LIBRARIES}
    ${FFMPEG_LIBRARIES}  # 添加 FFmpeg 库
    pthread
)

# ASE libraries (no PortAudio needed)
set(ASE_LIBRARIES
    ${ONNXRUNTIME_LIB}
    ${SNDFILE_LIBRARIES}
    ${CURL_LIBRARIES}
    ${FFTW3_LIBRARIES}
    ${FFMPEG_LIBRARIES}  # 添加 FFmpeg 库
    pthread
)

# Link libraries for ASR executable
target_link_libraries(spacemit-asr-cpp ${COMMON_LIBRARIES})
target_link_libraries(spacemit-asr-cpp hw_grpc_proto)

target_link_libraries(spacemit-asr-cpp ${CMAKE_THREAD_LIBS_INIT} absl::flags hw_grpc_proto
  absl::flags_parse
  gRPC::${_REFLECTION}
  gRPC::${_GRPC_GRPCPP}
  protobuf::${_PROTOBUF_LIBPROTOBUF})

# Add library search paths for both executables
target_link_directories(spacemit-asr-cpp PRIVATE
    ${SNDFILE_LIBRARY_DIRS}
    ${AVCODEC_LIBRARY_DIRS}
    ${AVFORMAT_LIBRARY_DIRS}
    ${AVUTIL_LIBRARY_DIRS}
    ${SWRESAMPLE_LIBRARY_DIRS}
)


# Compiler flags for both executables
target_compile_options(spacemit-asr-cpp PRIVATE 
    ${SNDFILE_CFLAGS_OTHER}
    ${AVCODEC_CFLAGS_OTHER}
    ${AVFORMAT_CFLAGS_OTHER}
)


# Set output directory for all executables
set_target_properties(spacemit-asr-cpp PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin
)

# Copy models directory if it exists
if(EXISTS ${CMAKE_SOURCE_DIR}/models)
    file(COPY ${CMAKE_SOURCE_DIR}/models DESTINATION ${CMAKE_BINARY_DIR})
endif()

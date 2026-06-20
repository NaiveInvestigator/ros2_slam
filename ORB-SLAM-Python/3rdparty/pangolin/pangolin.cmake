# MIT License
#
# Copyright (c) 2025  Sumanth Nagulavancha, Ignacio Vizzo, Tiziano Guadanino,
# Saurabh Gupta Permission is hereby granted, free of charge, to any person
# obtaining a copy of this software and associated documentation files (the
# "Software"), to deal in the Software without restriction, including without
# limitation the rights to use, copy, modify, merge, publish, distribute,
# sublicense, and/or sell copies of the Software, and to permit persons to whom
# the Software is furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.
option(BUILD_SHARED_LIBS OFF)
option(BUILD_TOOLS OFF)
option(BUILD_EXAMPLES OFF)
option(BUILD_PANGOLIN_PYTHON "Build support for Pangolin Interactive Console"
       OFF)
option(BUILD_PANGOLIN_LIBDC1394 "Build support for libdc1394 video input" OFF)
option(BUILD_PANGOLIN_V4L "Build support for V4L video input" OFF)
option(BUILD_PANGOLIN_FFMPEG "Build support for ffmpeg video input" OFF)
option(BUILD_PANGOLIN_REALSENSE "Build support for RealSense video input" OFF)
option(BUILD_PANGOLIN_REALSENSE2 "Build support for RealSense2 video input" OFF)
option(BUILD_PANGOLIN_OPENNI "Build support for OpenNI video input" OFF)
option(BUILD_PANGOLIN_OPENNI2 "Build support for OpenNI2 video input" OFF)
option(BUILD_PANGOLIN_LIBUVC "Build support for libuvc video input" OFF)
option(BUILD_PANGOLIN_UVC_MEDIAFOUNDATION
       "Build support for MediaFoundation UVC input" OFF)
option(BUILD_PANGOLIN_DEPTHSENSE "Build support for DepthSense video input" OFF)
option(BUILD_PANGOLIN_TELICAM "Build support for TeliCam video input" OFF)
option(BUILD_PANGOLIN_PLEORA "Build support for Pleora video input" OFF)
option(BUILD_PANGOLIN_LIBPNG "Build support for libpng image input" OFF)
option(BUILD_PANGOLIN_LIBJPEG "Build support for libjpeg image input" OFF)
option(BUILD_PANGOLIN_LIBTIFF "Build support for libtiff image input" OFF)
option(BUILD_PANGOLIN_LIBOPENEXR "Build support for libopenexr image input" OFF)
option(BUILD_PANGOLIN_LZ4 "Build support for liblz4 compression" OFF)
option(BUILD_PANGOLIN_ZSTD "Build support for libzstd compression" OFF)
option(BUILD_PANGOLIN_LIBRAW "Build support for raw images (libraw)" OFF)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

include(FetchContent)
FetchContent_Declare(
  pangolin
  SYSTEM
  URL https://github.com/stevenlovegrove/Pangolin/archive/refs/tags/v0.9.tar.gz)
if(NOT pangolin_POPULATED)
  FetchContent_Populate(pangolin)
  add_subdirectory(${pangolin_SOURCE_DIR} ${pangolin_BINARY_DIR} SYSTEM
                   EXCLUDE_FROM_ALL)
endif()

if(${CMAKE_VERSION} VERSION_LESS 3.25)
  get_target_property(pangolin_include_dirs pangolin
                      INTERFACE_INCLUDE_DIRECTORIES)
  set_target_properties(pangolin PROPERTIES INTERFACE_SYSTEM_INCLUDE_DIRECTORIES
                                            "${pangolin_include_dirs}")
endif()

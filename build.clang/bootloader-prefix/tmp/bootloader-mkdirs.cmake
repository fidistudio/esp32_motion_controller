# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "/home/fidel/esp/esp-idf/components/bootloader/subproject"
  "/home/fidel/esp/HallEncoder/build.clang/bootloader"
  "/home/fidel/esp/HallEncoder/build.clang/bootloader-prefix"
  "/home/fidel/esp/HallEncoder/build.clang/bootloader-prefix/tmp"
  "/home/fidel/esp/HallEncoder/build.clang/bootloader-prefix/src/bootloader-stamp"
  "/home/fidel/esp/HallEncoder/build.clang/bootloader-prefix/src"
  "/home/fidel/esp/HallEncoder/build.clang/bootloader-prefix/src/bootloader-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "/home/fidel/esp/HallEncoder/build.clang/bootloader-prefix/src/bootloader-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "/home/fidel/esp/HallEncoder/build.clang/bootloader-prefix/src/bootloader-stamp${cfgdir}") # cfgdir has leading slash
endif()

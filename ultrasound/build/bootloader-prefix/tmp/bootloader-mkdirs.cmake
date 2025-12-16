# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "/home/jozef/esp/esp-idf/components/bootloader/subproject"
  "/home/jozef/Desktop/SKOLA/5.semester/IMP/Projekt/ultrasound/build/bootloader"
  "/home/jozef/Desktop/SKOLA/5.semester/IMP/Projekt/ultrasound/build/bootloader-prefix"
  "/home/jozef/Desktop/SKOLA/5.semester/IMP/Projekt/ultrasound/build/bootloader-prefix/tmp"
  "/home/jozef/Desktop/SKOLA/5.semester/IMP/Projekt/ultrasound/build/bootloader-prefix/src/bootloader-stamp"
  "/home/jozef/Desktop/SKOLA/5.semester/IMP/Projekt/ultrasound/build/bootloader-prefix/src"
  "/home/jozef/Desktop/SKOLA/5.semester/IMP/Projekt/ultrasound/build/bootloader-prefix/src/bootloader-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "/home/jozef/Desktop/SKOLA/5.semester/IMP/Projekt/ultrasound/build/bootloader-prefix/src/bootloader-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "/home/jozef/Desktop/SKOLA/5.semester/IMP/Projekt/ultrasound/build/bootloader-prefix/src/bootloader-stamp${cfgdir}") # cfgdir has leading slash
endif()

# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "C:/Users/athar/Desktop/W/targets/esp32/setup/esp_idf/components/bootloader/subproject"
  "C:/Users/athar/Desktop/SideProjects/HACKED-2025-Integrating-Factor/MatlabNew/TestLED_Model_esp32_build_system/build/bootloader"
  "C:/Users/athar/Desktop/SideProjects/HACKED-2025-Integrating-Factor/MatlabNew/TestLED_Model_esp32_build_system/build/bootloader-prefix"
  "C:/Users/athar/Desktop/SideProjects/HACKED-2025-Integrating-Factor/MatlabNew/TestLED_Model_esp32_build_system/build/bootloader-prefix/tmp"
  "C:/Users/athar/Desktop/SideProjects/HACKED-2025-Integrating-Factor/MatlabNew/TestLED_Model_esp32_build_system/build/bootloader-prefix/src/bootloader-stamp"
  "C:/Users/athar/Desktop/SideProjects/HACKED-2025-Integrating-Factor/MatlabNew/TestLED_Model_esp32_build_system/build/bootloader-prefix/src"
  "C:/Users/athar/Desktop/SideProjects/HACKED-2025-Integrating-Factor/MatlabNew/TestLED_Model_esp32_build_system/build/bootloader-prefix/src/bootloader-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "C:/Users/athar/Desktop/SideProjects/HACKED-2025-Integrating-Factor/MatlabNew/TestLED_Model_esp32_build_system/build/bootloader-prefix/src/bootloader-stamp/${subDir}")
endforeach()

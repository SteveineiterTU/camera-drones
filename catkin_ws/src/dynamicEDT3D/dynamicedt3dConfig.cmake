# - Config file for the dynamicEDT3D package
#
#  Usage from an external project:
#    In your CMakeLists.txt, add these lines:
#
#    FIND_PACKAGE(dynamicedt3d REQUIRED )
#    INCLUDE_DIRECTORIES(${DYNAMICEDT3D_INCLUDE_DIRS})
#    TARGET_LINK_LIBRARIES(MY_TARGET_NAME ${DYNAMICEDT3D_LIBRARIES})
#
# It defines the following variables
#  DYNAMICEDT3D_INCLUDE_DIRS  - include directories for dynamicEDT3D
#  DYNAMICEDT3D_LIBRARY_DIRS  - library directories for dynamicEDT3D (normally not used!)
#  DYNAMICEDT3D_LIBRARIES     - libraries to link against
#  DYNAMICEDT3D_MAJOR_VERSION - major version
#  DYNAMICEDT3D_MINOR_VERSION - minor version
#  DYNAMICEDT3D_PATCH_VERSION - patch version
#  DYNAMICEDT3D_VERSION       - major.minor.patch version

@PACKAGE_INIT@

set(DYNAMICEDT3D_MAJOR_VERSION "@DYNAMICEDT3D_MAJOR_VERSION@")
set(DYNAMICEDT3D_MINOR_VERSION "@DYNAMICEDT3D_MINOR_VERSION@")
set(DYNAMICEDT3D_PATCH_VERSION "@DYNAMICEDT3D_PATCH_VERSION@")
set(DYNAMICEDT3D_VERSION "@DYNAMICEDT3D_VERSION@")

# Tell the user project where to find our headers and libraries
set_and_check(DYNAMICEDT3D_INCLUDE_DIRS "@PACKAGE_DYNAMICEDT3D_INCLUDE_DIRS@")
set_and_check(DYNAMICEDT3D_LIBRARY_DIRS "@PACKAGE_DYNAMICEDT3D_LIB_DIR@")

set(DYNAMICEDT3D_LIBRARIES "@PACKAGE_DYNAMICEDT3D_LIB_DIR@/@DYNAMICEDT3D_LIBRARY@")

@DYNAMICEDT3D_INCLUDE_TARGETS@
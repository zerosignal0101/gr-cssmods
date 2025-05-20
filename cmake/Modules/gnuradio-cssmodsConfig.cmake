find_package(PkgConfig)

PKG_CHECK_MODULES(PC_GR_CSSMODS gnuradio-cssmods)

FIND_PATH(
    GR_CSSMODS_INCLUDE_DIRS
    NAMES gnuradio/cssmods/api.h
    HINTS $ENV{CSSMODS_DIR}/include
        ${PC_CSSMODS_INCLUDEDIR}
    PATHS ${CMAKE_INSTALL_PREFIX}/include
          /usr/local/include
          /usr/include
)

FIND_LIBRARY(
    GR_CSSMODS_LIBRARIES
    NAMES gnuradio-cssmods
    HINTS $ENV{CSSMODS_DIR}/lib
        ${PC_CSSMODS_LIBDIR}
    PATHS ${CMAKE_INSTALL_PREFIX}/lib
          ${CMAKE_INSTALL_PREFIX}/lib64
          /usr/local/lib
          /usr/local/lib64
          /usr/lib
          /usr/lib64
          )

include("${CMAKE_CURRENT_LIST_DIR}/gnuradio-cssmodsTarget.cmake")

INCLUDE(FindPackageHandleStandardArgs)
FIND_PACKAGE_HANDLE_STANDARD_ARGS(GR_CSSMODS DEFAULT_MSG GR_CSSMODS_LIBRARIES GR_CSSMODS_INCLUDE_DIRS)
MARK_AS_ADVANCED(GR_CSSMODS_LIBRARIES GR_CSSMODS_INCLUDE_DIRS)

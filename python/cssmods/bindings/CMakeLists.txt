# Copyright 2020 Free Software Foundation, Inc.
#
# This file is part of GNU Radio
#
# SPDX-License-Identifier: GPL-3.0-or-later
#

########################################################################
# Check if there is C++ code at all
########################################################################
if(NOT cssmods_sources)
    MESSAGE(STATUS "No C++ sources... skipping python bindings")
    return()
endif(NOT cssmods_sources)

########################################################################
# Check for pygccxml
########################################################################
GR_PYTHON_CHECK_MODULE_RAW(
    "pygccxml"
    "import pygccxml"
    PYGCCXML_FOUND
    )

include(GrPybind)

########################################################################
# Python Bindings
########################################################################
list(APPEND cssmods_python_files
    css_modulate_python.cc
    css_symbols_python.cc
    css_demodulate_symbols_python.cc
    css_symbols_decode_python.cc python_bindings.cc)

GR_PYBIND_MAKE_OOT(cssmods
   ../../..
   gr::cssmods
   "${cssmods_python_files}")

# copy bindings extension for use in QA test module
add_custom_command(TARGET cssmods_python POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy $<TARGET_FILE:cssmods_python>
    ${PROJECT_BINARY_DIR}/test_modules/gnuradio/cssmods/
)

install(TARGETS cssmods_python DESTINATION ${GR_PYTHON_DIR}/gnuradio/cssmods COMPONENT pythonapi)

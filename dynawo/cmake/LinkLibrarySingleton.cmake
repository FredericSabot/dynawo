# Copyright (c) 2020, RTE (http://www.rte-france.com)
# See AUTHORS.txt
# All rights reserved.
# This Source Code Form is subject to the terms of the Mozilla Public
# License, v. 2.0. If a copy of the MPL was not distributed with this
# file, you can obtain one at http://mozilla.org/MPL/2.0/.
# SPDX-License-Identifier: MPL-2.0
#
# This file is part of Dynawo, an hybrid C++/Modelica open source time domain simulation tool for power systems.

macro(link_library_singleton _target)
  if(LIBRARY_TYPE STREQUAL "STATIC")
    target_include_directories(${_target} PRIVATE $<TARGET_PROPERTY:dynawo_Util,INTERFACE_INCLUDE_DIRECTORIES>)
    if(MSVC OR APPLE)
      # Link to executable
      target_link_libraries(${_target} PRIVATE dynawo)
    endif()
  else()
    target_link_libraries(${_target} PRIVATE dynawo_Util)
  endif()
endmacro()

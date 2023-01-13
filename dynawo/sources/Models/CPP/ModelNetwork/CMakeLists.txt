# Copyright (c) 2015-2019, RTE (http://www.rte-france.com)
# See AUTHORS.txt
# All rights reserved.
# This Source Code Form is subject to the terms of the Mozilla Public
# License, v. 2.0. If a copy of the MPL was not distributed with this
# file, you can obtain one at http://mozilla.org/MPL/2.0/.
# SPDX-License-Identifier: MPL-2.0
#
# This file is part of Dynawo, an hybrid C++/Modelica open source time domain simulation tool for power systems.

#################################
#         ModelNetwork          #
#################################

set(NETWORK_CPP_MODELS_SOURCES
  DYNDerivative.cpp
  DYNNetworkComponent.cpp
  DYNModelBus.cpp
  DYNModelGenerator.cpp
  DYNModelSwitch.cpp
  DYNModelLine.cpp
  DYNModelLoad.cpp
  DYNModelShuntCompensator.cpp
  DYNModelStaticVarCompensator.cpp
  DYNModelThreeWindingsTransformer.cpp
  DYNModelTwoWindingsTransformer.cpp
  DYNModelDanglingLine.cpp
  DYNModelNetwork.cpp
  DYNModelCurrentLimits.cpp
  DYNModelRatioTapChanger.cpp
  DYNModelPhaseTapChanger.cpp
  DYNModelHvdcLink.cpp
  DYNModelVoltageLevel.cpp
  )

set(NETWORK_CPP_MODELS_HEADERS
  DYNDerivative.h
  DYNNetworkComponent.h
  DYNModelBus.h
  DYNModelGenerator.h
  DYNModelSwitch.h
  DYNModelLine.h
  DYNModelLoad.h
  DYNModelShuntCompensator.h
  DYNModelStaticVarCompensator.h
  DYNModelThreeWindingsTransformer.h
  DYNModelTwoWindingsTransformer.h
  DYNModelDanglingLine.h
  DYNModelNetwork.h
  DYNModelCurrentLimits.h
  DYNModelRatioTapChanger.h
  DYNModelPhaseTapChanger.h
  DYNModelHvdcLink.h
  DYNModelVoltageLevel.h
  )

add_library(dynawo_CPPModelsNetwork SHARED ${NETWORK_CPP_MODELS_SOURCES})

target_include_directories(dynawo_CPPModelsNetwork
  INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
  PUBLIC
    $<TARGET_PROPERTY:dynawo_CPPModelsCommon,INTERFACE_INCLUDE_DIRECTORIES>
  )
target_include_directories(dynawo_CPPModelsNetwork SYSTEM
  PRIVATE
  $<TARGET_PROPERTY:Boost::boost,INTERFACE_INCLUDE_DIRECTORIES>
  )

target_compile_definitions(dynawo_CPPModelsNetwork
  INTERFACE $<$<BOOL:${MSVC}>:_USE_MATH_DEFINES>
  )

target_link_libraries(dynawo_CPPModelsNetwork
  PRIVATE
    dynawo_DataInterface
  PUBLIC
    dynawo_SolverKINSubModel
  )

if(BUILD_TESTS OR BUILD_TESTS_COVERAGE)
  add_subdirectory(test)
endif()

install(TARGETS dynawo_CPPModelsNetwork EXPORT dynawo-targets DESTINATION ${LIBDIR_NAME})
install(FILES ${NETWORK_CPP_MODELS_HEADERS} DESTINATION ${INCLUDEDIR_NAME})

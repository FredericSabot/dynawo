# Copyright (c) 2015-2021, RTE (http://www.rte-france.com)
# See AUTHORS.txt
# All rights reserved.
# This Source Code Form is subject to the terms of the Mozilla Public
# License, v. 2.0. If a copy of the MPL was not distributed with this
# file, you can obtain one at http://mozilla.org/MPL/2.0/.
# SPDX-License-Identifier: MPL-2.0
#
# This file is part of Dynawo, an hybrid C++/Modelica open source time domain simulation tool for power systems.

# Preassembled models

set(MODELICA_PREASSEMBLED_MODELS
  BooleanTable.xml
  Bus.xml
  CombiTimeTable.xml
  CurrentLimitAutomaton.xml
  DiscretePI.xml
  DoubleStep.xml
  EventConnectedStatus.xml
  EventQuadripoleConnection.xml
  EventQuadripoleDisconnection.xml
  EventSetPointBoolean.xml
  EventSetPointReal.xml
  EventSetPointDouble.xml
  EventSetPointDoubleReal.xml
  GeneratorFictitious.xml
  GeneratorPQ.xml
  GeneratorPQPropSignalN.xml
  GeneratorPQPropSignalNSFR.xml
  GeneratorPQPropDiagramPQSignalN.xml
  GeneratorPQPropDiagramPQSignalNSFR.xml
  GeneratorPV.xml
  GeneratorPVFixed.xml
  GeneratorPVDiagramPQ.xml
  GeneratorPVSignalN.xml
  GeneratorPVRpclSignalN.xml
  GeneratorPVRpcl2SignalN.xml
  GeneratorPVSignalNSFR.xml
  GeneratorPVTfoSignalN.xml
  GeneratorPVTfoDiagramPQSignalN.xml
  GeneratorPVTfoRpclSignalN.xml
  GeneratorPVTfoRpcl2SignalN.xml
  GeneratorPVTfoDiagramPQRpclSignalN.xml
  GeneratorPVTfoDiagramPQRpcl2SignalN.xml
  GeneratorPVDiagramPQSignalN.xml
  GeneratorPVDiagramPQRpclSignalN.xml
  GeneratorPVDiagramPQRpcl2SignalN.xml
  GeneratorPVDiagramPQSignalNSFR.xml
  GeneratorPVPropSignalN.xml
  GeneratorPVPropSignalNSFR.xml
  GeneratorPVRemoteSignalN.xml
  GeneratorPVRemoteSignalNSFR.xml
  GeneratorPVRemoteDiagramPQSignalN.xml
  GeneratorPVRemoteDiagramPQSignalNSFR.xml
  GeneratorSynchronousFourWindings.xml
  GeneratorSynchronousThreeWindings.xml
  GeneratorSynchronousFourWindingsProportionalRegulations.xml
  GeneratorSynchronousThreeWindingsProportionalRegulations.xml
  GeneratorSynchronousFourWindingsGoverPropVRPropInt.xml
  GeneratorSynchronousThreeWindingsGoverPropVRPropInt.xml
  GeneratorSynchronousFourWindingsTGov1Sexs.xml
  GeneratorSynchronousFourWindingsTGov1SexsPss2A.xml
  GeneratorSynchronousFourWindingsVRKundur.xml
  GeneratorSynchronousFourWindingsVRKundurPssKundur.xml
  GeneratorSynchronousFourWindingsGovSteam1ExcIEEEST4B.xml
  GeneratorSynchronousFourWindingsGovSteam1ExcIEEEST4BPssIEEE2B.xml
  GeneratorSynchronousFourWindingsHydroG_IEEET1VR.xml
  GeneratorSynchronousFourWindingsGovernorBPAGG_IEEET1VR.xml
  GeneratorSynchronousProportionalRegulationsInternalParameters.xml
  GenericAutomaton.xml
  GridFormingConverterDroopControl.xml
  GridFormingConverterMatchingControl.xml
  GridFormingConverterDispatchableVirtualOscillatorControl.xml
  HvdcPQProp.xml
  HvdcPQPropEmulation.xml
  HvdcPQPropEmulationSet.xml
  HvdcPQPropEmulationVariableK.xml
  HvdcPQPropDiagramPQ.xml
  HvdcPQPropDiagramPQEmulation.xml
  HvdcPQPropDiagramPQEmulationSet.xml
  HvdcPQPropDiagramPQEmulationVariableK.xml
  HvdcPQPropDangling.xml
  HvdcPQPropDanglingDiagramPQ.xml
  HvdcPV.xml
  HvdcPVEmulation.xml
  HvdcPVEmulationSet.xml
  HvdcPVEmulationVariableK.xml
  HvdcPVDiagramPQ.xml
  HvdcPVDiagramPQEmulation.xml
  HvdcPVDiagramPQEmulationSet.xml
  HvdcPVDiagramPQEmulationVariableK.xml
  HvdcPVDangling.xml
  HvdcPVDanglingDiagramPQ.xml
  HvdcPTanPhi.xml
  HvdcPTanPhiDiagramPQ.xml
  HvdcPTanPhiDangling.xml
  HvdcPTanPhiDanglingDiagramPQ.xml
  HvdcVSC.xml
  HvdcVSCDanglingP.xml
  HvdcVSCDanglingUdc.xml
  HvdcVSCEmulation.xml
  IECWT4ACurrentSource.xml
  IECWT4BCurrentSource.xml
  InfiniteBus.xml
  InfiniteBusFromTable.xml
  InfiniteBusWithImpedance.xml
  InfiniteBusWithVariations.xml
  InjectorBG.xml
  IntegerDoubleStep.xml
  Line.xml
  LoadAlphaBeta.xml
  LoadAlphaBetaRestorative.xml
  LoadAlphaBetaRestorativeLimitsRecalc.xml
  LoadAlphaBetaMotor.xml
  LoadOneTransformer.xml
  LoadOneTransformerTapChanger.xml
  LoadPQ.xml
  LoadShedding.xml
  LoadTwoTransformers.xml
  LoadTwoTransformersTapChangers.xml
  LoadZIP.xml
  Measurements.xml
  NodeFault.xml
  PhaseShifterBlockingI.xml
  PhaseShifterI.xml
  PhaseShifterP.xml
  PhotovoltaicsWeccCurrentSource.xml
  PhotovoltaicsWeccVoltageSource.xml
  PowerTransferHVDCEmulation.xml
  PIAntiWindUpTable.xml
  ShuntBWithSections.xml
  SecondaryFrequencyRegulation.xml
  SecondaryVoltageControlSimp.xml
  SetPoint.xml
  ShuntB.xml
  SignalN.xml
  SineWave.xml
  StaticVarCompensator.xml
  StaticVarCompensatorPV.xml
  StaticVarCompensatorPVModeHandling.xml
  StaticVarCompensatorPVRemote.xml
  StaticVarCompensatorPVRemoteModeHandling.xml
  StaticVarCompensatorPVProp.xml
  StaticVarCompensatorPVPropModeHandling.xml
  StaticVarCompensatorPVPropRemote.xml
  StaticVarCompensatorPVPropRemoteModeHandling.xml
  Step.xml
  Switch.xml
  TapChangerAutomaton.xml
  TapChangerBlockingAutomaton1.xml
  TapChangerBlockingAutomaton2.xml
  TapChangerBlockingAutomaton3.xml
  TapChangerBlockingAutomaton4.xml
  TapChangerBlockingAutomaton5.xml
  TransformerFixedRatio.xml
  UnderVoltageAutomaton.xml
  VRRemote.xml
  WeightedDifference.xml
  WindTurbine4AWeccCurrentSource.xml
  WindTurbine4BWeccCurrentSource.xml
  )

# PYTHON scripts preassembled models
set(PYTHON_SCRIPTS_PREASSEMBLED_MODELS
  buildChecker.py
  )

install(PROGRAMS ${PYTHON_SCRIPTS_PREASSEMBLED_MODELS} DESTINATION ${SBINDIR_NAME})

foreach( FILE ${MODELICA_PREASSEMBLED_MODELS} )
  compile_preassembled_model(${FILE})
endforeach( FILE )

include(CMakeListsModelsTargets.txt)

if(BUILD_TESTS OR BUILD_TESTS_COVERAGE)
  add_subdirectory(test)
endif()

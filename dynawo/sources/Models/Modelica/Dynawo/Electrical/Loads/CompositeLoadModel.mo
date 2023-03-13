within Dynawo.Electrical.Loads;

model CompositeLoadModel "WECC composite load model"
  /*
  * Copyright (c) 2023, RTE (http://www.rte-france.com)
  * See AUTHORS.txt
  * All rights reserved.
  * This Source Code Form is subject to the terms of the Mozilla Public
  * License, v. 2.0. If a copy of the MPL was not distributed with this
  * file, you can obtain one at http://mozilla.org/MPL/2.0/.
  * SPDX-License-Identifier: MPL-2.0
  *
  * This file is part of Dynawo, an hybrid C++/Modelica open source suite of simulation tools for power systems.
  */
  extends BaseClasses.BaseLoad;
  extends AdditionalIcons.Load;
  import Dynawo;
  import Dynawo.Electrical.SystemBase;

  parameter Types.ApparentPowerModule SNom = s0Pu.re "Nominal apparent power of the load in MVA";
  parameter Types.ApparentPowerModule SNomTransfo = SNom "Nominal apparent power of the transformer in MVA";
  parameter Real ElectronicShare "Share of electronic loads";
  // Motor A, B and C parameters
  parameter Real MotorABCShare[3] "Share of motor loads (for each of the motors A B, and C)";
  parameter Types.ApparentPowerModule MotorABCSNom[3] = MotorABCShare * s0Pu.re * SystemBase.SnRef "Nominal apparent power of a single motor in MVA";
  parameter Types.PerUnit MotorABCRsPu[3] "Stator resistance in pu (base SNom, UNom)";
  parameter Types.PerUnit MotorABCLsPu[3] "Synchronous reactance in pu (base SNom, UNom)";
  parameter Types.PerUnit MotorABCLPPu[3] "Transient reactance in pu (base SNom, UNom)";
  parameter Types.PerUnit MotorABCLPPPu[3] "Subtransient reactance in pu (base SNom, UNom)";
  parameter Types.Time MotorABCtP0[3] "Transient open circuit time constant in s";
  parameter Types.Time MotorABCtPP0[3] "Subtransient open circuit time constant in s";
  parameter Real MotorABCH[3] "Inertia constant (s, base UNom, SNom)";
  parameter Real MotorABCtorqueExponent[3] "Exponent of the torque speed dependancy";
  // Motor D parameters
  parameter Real MotorDShare "Share of motor D loads";
  parameter Real MotorDShareRestarting "Share of motors that can restart after stalling";
  parameter Real MotorDPF "Load factor of the motor";
  parameter Types.VoltageModulePu MotorDUStallPu "Voltage at which the motor stalls in pu (base UNom)";
  parameter Types.VoltageModulePu MotorDUBreakPu = 0.86 "Voltage below which the active power consumed by the motor increases in pu (base UNom)";
  parameter Types.PerUnit MotorDRStallPu "Equivalent resistance of the motor when stalled in pu (base SNom, UNom)";
  parameter Types.PerUnit MotorDXStallPu "Equivalent impedance of the motor when stalled in pu (base SNom, UNom)";
  parameter Types.Time MotorDtRestart "Time needed to restart the motor in s";
  parameter Types.Time MotorDtStall "Time after which the motor stalls if the voltage stays below UStallPu in s";
  parameter Real MotorDKP1 = 0 "Proportional factor of active power voltage dependance when UPu < UBreakPu";
  parameter Real MotorDKP2 = 12 "Proportional factor of active power voltage dependance when UPu > UBreakPu";
  parameter Real MotorDNP1 = 0 "Exponent of the active power voltage dependance when UPu < UBreakPu";
  parameter Real MotorDNP2 = 3.2 "Exponent of the active power voltage dependance when UPu > UBreakPu";
  parameter Real MotorDKQ1 = 6 "Proportional factor of reactive power voltage dependance when UPu > UBreakPu";
  parameter Real MotorDKQ2 = 11 "Proportional factor of reactive power voltage dependance when UPu < UBreakPu";
  parameter Real MotorDNQ1 = 2 "Exponent of the reactive power voltage dependance when UPu > UBreakPu";
  parameter Real MotorDNQ2 = 2.5 "Exponent of the reactive power voltage dependance when UPu < UBreakPu";
  parameter Types.VoltageModulePu MotorDUMinPu[2] "Voltage threshold under which the automaton is activated in pu (base UNom) (for each UV relay)";
  parameter Types.Time MotorDtLagAction[2] "Time-lag due to the actual trip action in s (for each UV relay)";
  parameter Real MotorDShareToDisconnect "Share of motors that are disconnected by UV relays (not cumulative)";
  // Electronic load parameters
  parameter Types.VoltageModulePu ElectronicUd1Pu "Voltage at which the load starts to disconnect in pu (base UNom)";
  parameter Types.VoltageModulePu ElectronicUd2Pu "Voltage at which the load is completely disconnected in pu (base UNom)";
  parameter Real ElectronicrecoveringShare "Share of the load that recovers from low voltage trip";
  parameter Types.Time ElectronictFilter "Time constant for estimation of UMinPu";
  // Static load parameters
  parameter Real StaticAlpha "Active load sensitivity to voltage";
  parameter Real StaticBeta "Reactive load sensitivity to voltage";
  // Line parameters
  parameter Types.PerUnit LineRPu = 0 "Line resistance in pu (base SNom)";
  parameter Types.PerUnit LineXPu "Line reactance in pu (base SNom)";
  parameter Types.PerUnit LineGPu = 0 "Line half-conductance in pu (base SNom)";
  parameter Types.PerUnit LineBPu = 0 "Line half-susceptance in pu (base SNom)";
  // Feeder parameters
  parameter Types.PerUnit FeederRPu "Feeder resistance in pu (base SNom)";
  parameter Types.PerUnit FeederXPu "Feeder reactance in pu (base SNom)";
  parameter Types.PerUnit FeederGPu = 0 "Feeder half-conductance in pu (base SNom)";
  parameter Types.PerUnit FeederBPu = 0 "Feeder half-susceptance in pu (base SNom)";
  // Transformer parameters
  parameter Types.PerUnit TfoMinTapPu "Transformer minimum transformation ratio in pu: U2/U1 in no load conditions";
  parameter Types.PerUnit TfoMaxTapPu "Transformer maximum transformation ratio in pu: U2/U1 in no load conditions";
  parameter Integer TfoNbTap "Transformer number of taps";
  parameter Types.VoltageModule TapChangerUTarget "Target voltage at the secondary side of the transformer in kV";
  parameter Types.VoltageModule TapChangerUDeadBand(min = 0) "Voltage dead-band in kV";
  parameter Types.Time TapChangert1st(min = 0) "TapChanger time lag before changing the first tap";
  parameter Types.Time TapChangertNext(min = 0) "TapChanger time lag before changing subsequent taps";
  parameter Boolean OLTCLocked "True if the on-load tap changer is locked";
  // Shunt parameters
  parameter Types.PerUnit ShuntLowSideBPu "Shunt on low-side susceptance in pu (base SNom), negative values for capacitive consumption (over-excited), positive values for inductive consumption (under-excited)";

  // Init values
  parameter Types.ComplexVoltagePu uLowSide0Pu "Start value of complex voltage at low-side terminal in pu (base UNom)";
  parameter Types.ComplexVoltagePu uLoadSide0Pu "Start value of complex voltage at load-side terminal in pu (base UNom)";
  // Motor A, B, and C initial values
  parameter Types.PerUnit MotorABCEdP0Pu[3];
  parameter Types.PerUnit MotorABCEqP0Pu[3];
  parameter Types.PerUnit MotorABCEdPP0Pu[3];
  parameter Types.PerUnit MotorABCEqPP0Pu[3];
  parameter Types.PerUnit MotorABCUd0Pu[3] "Start value of voltage of direct axis in pu";
  parameter Types.PerUnit MotorABCUq0Pu[3] "Start value of voltage of quadrature axis in pu";
  parameter Types.PerUnit MotorABCid0Pu[3] "Start value of current of direct axis in pu";
  parameter Types.PerUnit MotorABCiq0Pu[3] "Start value of current of quadrature axis in pu";
  parameter Types.PerUnit MotorABCce0Pu[3] "Start value of the electrical torque in pu (SNom base)";
  parameter Real MotorABCs0[3] "Start value of the slip of the motor";
  parameter Types.AngularVelocity MotorABComegaR0Pu[3] "Start value of the angular velocity of the motor";
  parameter Types.ComplexCurrentPu MotorABCi0Pu[3] "Start value of complex current at load terminal in pu (base UNom, SnRef) (receptor convention)";
  parameter Types.ComplexApparentPowerPu MotorABCs0Pu[3] "Start value of complex apparent power in pu (base SnRef) (receptor convention)";
  // Motor D initial values
  parameter Types.ComplexCurrentPu MotorDi0Pu "Start value of complex current at load terminal in pu (base UNom, SnRef) (receptor convention)";
  parameter Types.ComplexApparentPowerPu MotorDs0Pu "Start value of complex apparent power in pu (base SnRef) (receptor convention)";
  parameter Types.VoltageModulePu MotorDUStallBreakPu "Voltage where the stalling and non-stalling characteristics cross in pu (base UNom)";
  // Electronic load initial values
  parameter Types.ComplexCurrentPu Electronici0Pu "Start value of complex current at load terminal in pu (base UNom, SnRef) (receptor convention)";
  parameter Types.ComplexApparentPowerPu Electronics0Pu "Start value of complex apparent power in pu (base SnRef) (receptor convention)";
  // Static load initial values
  parameter Types.ComplexCurrentPu Statici0Pu "Start value of complex current at load terminal in pu (base UNom, SnRef) (receptor convention)";
  parameter Types.ComplexApparentPowerPu Statics0Pu "Start value of complex apparent power in pu (base SnRef) (receptor convention)";
  // Transformer initial values
  parameter Types.ComplexVoltagePu Tfou10Pu "Transformer start value of complex voltage at terminal 1 in pu (base U1Nom)";
  parameter Types.ComplexCurrentPu Tfoi10Pu "Transformer start value of complex current at terminal 1 in pu (base U1Nom, SNom) (receptor convention)";
  parameter Types.ComplexVoltagePu Tfou20Pu "Transformer start value of complex voltage at terminal 2 in pu (base U2Nom)";
  parameter Types.ComplexCurrentPu Tfoi20Pu "Transformer start value of complex current at terminal 2 in pu (base U2Nom, SNom) (receptor convention)";
  parameter Types.VoltageModulePu TfoU10Pu "Transformer start value of voltage amplitude at terminal 1 in pu (base U1Nom)";
  parameter Types.VoltageModulePu TfoU20Pu "Transformer start value of voltage amplitude at terminal 2 in pu (base U2Nom)";
  parameter Types.ActivePowerPu TfoP10Pu "Transformer start value of active power at terminal 1 in pu (base SNom) (receptor convention)";
  parameter Types.ReactivePowerPu TfoQ10Pu "Transformer start value of reactive power at terminal 1 in pu (base SNom) (receptor convention)";
  parameter Integer TfoTap0 "Transformer start value of transformer tap";
  parameter Types.PerUnit TforTfo0Pu "Transformer start value of transformer ratio";
  // Shunt low side initial values
  parameter Types.ComplexApparentPowerPu ShuntLowSides0Pu "Shunt on low-side start value of apparent power at shunt terminal in pu (base SNom, receptor convention)";
  parameter Types.ComplexCurrentPu ShuntLowSidei0Pu "Shunt on low-side start value of complex current at shunt terminal in pu (base UNom, SNom, receptor convention)";
  // Reactive compensation low side initial values
  parameter Types.PerUnit ReactiveCompensationLowSideB0Pu "Reactive compensation on low-side susceptance in pu (base SNom), negative values for capacitive consumption (over-excited), positive values for inductive consumption (under-excited)";
  parameter Types.ComplexApparentPowerPu ReactiveCompensationLowSides0Pu "Reactive compensation on low-side start value of apparent power at shunt terminal in pu (base SNom, receptor convention)";
  parameter Types.ComplexCurrentPu ReactiveCompensationLowSidei0Pu "Reactive compensation on low-side start value of complex current at shunt terminal in pu (base UNom, SNom, receptor convention)";
  // Reactive compensation load side initial values
  parameter Types.PerUnit ReactiveCompensationLoadSideB0Pu "Reactive compensation on load-side susceptance in pu (base SNom), negative values for capacitive consumption (over-excited), positive values for inductive consumption (under-excited)";
  parameter Types.ComplexApparentPowerPu ReactiveCompensationLoadSides0Pu "Reactive compensation on load-side start value of apparent power at shunt terminal in pu (base SNom, receptor convention)";
  parameter Types.ComplexCurrentPu ReactiveCompensationLoadSidei0Pu "Reactive compensation on load-side start value of complex current at shunt terminal in pu (base UNom, SNom, receptor convention)";

protected
  final parameter Real StaticShare = 1 - ElectronicShare - sum(MotorABCShare) - MotorDShare "Share of static loads";

  Connectors.ImPin omegaRefPu(value(start = SystemBase.omegaRef0Pu)) "Network angular reference frequency in pu (base omegaNom)";
  Dynawo.Electrical.Lines.Line line(BPu = LineGPu, GPu = LineGPu, RPu = LineRPu, XPu = LineXPu) annotation(
    Placement(visible = true, transformation(origin = {-70, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Dynawo.Electrical.Transformers.IdealTransformerVariableTap transformerVariableTap( NbTap = TfoNbTap, P10Pu = TfoP10Pu, Q10Pu = TfoQ10Pu, Tap0 = TfoTap0, U10Pu = ComplexMath.'abs'(Tfou10Pu), U20Pu = ComplexMath.'abs'(Tfou20Pu), i10Pu = Tfoi10Pu, i20Pu = Tfoi20Pu, rTfo0Pu = TforTfo0Pu, rTfoMaxPu = TfoMaxTapPu, rTfoMinPu = TfoMinTapPu, u10Pu = Tfou10Pu, u20Pu = Tfou20Pu)  annotation(
    Placement(visible = true, transformation(origin = {-30, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Dynawo.Electrical.Buses.Bus lowSideBus annotation(
    Placement(visible = true, transformation(origin = {0, 0}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
  Dynawo.Electrical.Shunts.ShuntB shuntLowSide(BPu = ShuntLowSideBPu, i0Pu = ShuntLowSidei0Pu, s0Pu = ShuntLowSides0Pu, u0Pu = uLowSide0Pu)  annotation(
    Placement(visible = true, transformation(origin = {-8, -20}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Dynawo.Electrical.Lines.Line feeder(BPu = FeederBPu, GPu = FeederGPu, RPu = FeederRPu, XPu = FeederXPu)  annotation(
    Placement(visible = true, transformation(origin = {30, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Dynawo.Electrical.Buses.Bus loadBus annotation(
    Placement(visible = true, transformation(origin = {60, 0}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
  Dynawo.Electrical.Loads.LoadAlphaBeta staticLoad(alpha = StaticAlpha, beta = StaticBeta,i0Pu = Statici0Pu, s0Pu = Statics0Pu, u0Pu = uLoadSide0Pu)  annotation(
    Placement(visible = true, transformation(origin = {90, -50}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Dynawo.Electrical.Shunts.ShuntB reactiveCompensationLowSide(BPu = ReactiveCompensationLowSideB0Pu, i0Pu = ReactiveCompensationLowSidei0Pu, s0Pu = ReactiveCompensationLowSides0Pu, u0Pu = uLowSide0Pu)  annotation(
    Placement(visible = true, transformation(origin = {20, -20}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Dynawo.Electrical.Shunts.ShuntB reactiveCompensationLoadSide(BPu = ReactiveCompensationLoadSideB0Pu, i0Pu = ReactiveCompensationLoadSidei0Pu, s0Pu = ReactiveCompensationLoadSides0Pu, u0Pu = uLoadSide0Pu)  annotation(
    Placement(visible = true, transformation(origin = {40, -20}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Dynawo.Electrical.Controls.Transformers.TapChanger tapChanger(U0 = TapChangerUTarget, UDeadBand = TapChangerUDeadBand, UTarget = TapChangerUTarget,increaseTapToIncreaseValue = true, regulating0 = true, state0 = Dynawo.Electrical.Controls.Transformers.BaseClasses.TapChangerPhaseShifterParams.State.Standard, t1st = TapChangert1st, tNext = TapChangertNext, tap0 = TfoTap0, tapMax = TfoNbTap - 1, tapMin = 0, valueToMonitor0 = TapChangerUTarget) annotation(
    Placement(visible = true, transformation(origin = {-30, 30}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Dynawo.Electrical.Loads.ElectronicLoad electronicLoad(Ud1Pu = ElectronicUd1Pu, Ud2Pu = ElectronicUd2Pu, i0Pu = Electronici0Pu, recoveringShare = ElectronicrecoveringShare, s0Pu = Electronics0Pu, tFilter = ElectronictFilter, u0Pu = uLoadSide0Pu)  annotation(
    Placement(visible = true, transformation(origin = {90, -30}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Machines.Motors.MotorFifthOrder motorA(EdP0Pu = MotorABCEdP0Pu[1], EdPP0Pu = MotorABCEdPP0Pu[1], EqP0Pu = MotorABCEqP0Pu[1], EqPP0Pu = MotorABCEqPP0Pu[1],H = MotorABCH[1], LPPPu = MotorABCLPPPu[1], LPPu = MotorABCLPPu[1], LsPu = MotorABCLsPu[1],NbSwitchOffSignals = 2, RsPu = MotorABCRsPu[1], SNom = SNom*MotorABCShare[1], Ud0Pu = MotorABCUd0Pu[1], Uq0Pu = MotorABCUq0Pu[1], ce0Pu = MotorABCce0Pu[1], i0Pu = MotorABCi0Pu[1], id0Pu = MotorABCid0Pu[1], iq0Pu = MotorABCiq0Pu[1], omegaR0Pu = MotorABComegaR0Pu[1], s0 = MotorABCs0[1], s0Pu = MotorABCs0Pu[1], tP0 = MotorABCtP0[1], tPP0 = MotorABCtPP0[1], torqueExponent = MotorABCtorqueExponent[1], u0Pu = uLoadSide0Pu)  annotation(
    Placement(visible = true, transformation(origin = {90, 50}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Machines.Motors.MotorFifthOrder motorB(EdP0Pu = MotorABCEdP0Pu[2], EdPP0Pu = MotorABCEdPP0Pu[2], EqP0Pu = MotorABCEqP0Pu[2], EqPP0Pu = MotorABCEqPP0Pu[2],H = MotorABCH[2], LPPPu = MotorABCLPPPu[2], LPPu = MotorABCLPPu[2], LsPu = MotorABCLsPu[2],NbSwitchOffSignals = 2, RsPu = MotorABCRsPu[2], SNom = SNom*MotorABCShare[2], Ud0Pu = MotorABCUd0Pu[2], Uq0Pu = MotorABCUq0Pu[2], ce0Pu = MotorABCce0Pu[2], i0Pu = MotorABCi0Pu[2], id0Pu = MotorABCid0Pu[2], iq0Pu = MotorABCiq0Pu[2], omegaR0Pu = MotorABComegaR0Pu[2], s0 = MotorABCs0[2], s0Pu = MotorABCs0Pu[2], tP0 = MotorABCtP0[2], tPP0 = MotorABCtPP0[2], torqueExponent = MotorABCtorqueExponent[2], u0Pu = uLoadSide0Pu)  annotation(
    Placement(visible = true, transformation(origin = {90, 30}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Machines.Motors.MotorFifthOrder motorC(EdP0Pu = MotorABCEdP0Pu[3], EdPP0Pu = MotorABCEdPP0Pu[3], EqP0Pu = MotorABCEqP0Pu[3], EqPP0Pu = MotorABCEqPP0Pu[3],H = MotorABCH[3], LPPPu = MotorABCLPPPu[3], LPPu = MotorABCLPPu[3], LsPu = MotorABCLsPu[3],NbSwitchOffSignals = 2, RsPu = MotorABCRsPu[3], SNom = SNom*MotorABCShare[3], Ud0Pu = MotorABCUd0Pu[3], Uq0Pu = MotorABCUq0Pu[3], ce0Pu = MotorABCce0Pu[3], i0Pu = MotorABCi0Pu[3], id0Pu = MotorABCid0Pu[3], iq0Pu = MotorABCiq0Pu[3], omegaR0Pu = MotorABComegaR0Pu[3], s0 = MotorABCs0[3], s0Pu = MotorABCs0Pu[3], tP0 = MotorABCtP0[3], tPP0 = MotorABCtPP0[3], torqueExponent = MotorABCtorqueExponent[3], u0Pu = uLoadSide0Pu)  annotation(
    Placement(visible = true, transformation(origin = {90, 10}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Dynawo.Electrical.Machines.Motors.MotorD motorD(KP1 = MotorDKP1, KP2 = MotorDKP2, KQ1 = MotorDKQ1, KQ2 = MotorDKQ2, PF = MotorDPF, NP1 = MotorDNP1, NP2 = MotorDNP2, NQ1 = MotorDNQ1, NQ2 = MotorDNQ2, NbSwitchOffSignals = 2, RStallPu = MotorDRStallPu, SNom = SNom*MotorDShare, ShareRestarting = MotorDShareRestarting, ShareToDisconnect = MotorDShareToDisconnect, UBreakPu = MotorDUBreakPu, UMinPu = MotorDUMinPu, UStallBreakPu = MotorDUStallBreakPu, UStallPu = MotorDUStallPu, XStallPu = MotorDXStallPu, i0Pu = MotorDi0Pu, s0Pu = MotorDs0Pu, tLagAction = MotorDtLagAction, tRestart = MotorDtRestart, tStall = MotorDtStall, u0Pu = uLoadSide0Pu)  annotation(
    Placement(visible = true, transformation(origin = {90, -10}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
equation
  terminal.V = line.terminal1.V;
  terminal.i = line.terminal1.i * (SNom / SystemBase.SnRef);
  connect(omegaRefPu, motorA.omegaRefPu);
  connect(omegaRefPu, motorB.omegaRefPu);
  connect(omegaRefPu, motorC.omegaRefPu);
  connect(omegaRefPu, motorD.omegaRefPu);
  connect(tapChanger.UMonitored, transformerVariableTap.U2Pu);
  connect(tapChanger.tap, transformerVariableTap.tap);
  tapChanger.locked = OLTCLocked;
  connect(switchOffSignal1, motorA.switchOffSignal1);
  connect(switchOffSignal1, motorB.switchOffSignal1);
  connect(switchOffSignal1, motorC.switchOffSignal1);
  connect(switchOffSignal1, motorD.switchOffSignal1);
  connect(switchOffSignal1, electronicLoad.switchOffSignal1);
  connect(switchOffSignal1, staticLoad.switchOffSignal1);
  connect(switchOffSignal1, shuntLowSide.switchOffSignal1);
  connect(switchOffSignal1, reactiveCompensationLowSide.switchOffSignal1);
  connect(switchOffSignal1, reactiveCompensationLoadSide.switchOffSignal1);
  connect(switchOffSignal1, line.switchOffSignal1);
  connect(switchOffSignal1, feeder.switchOffSignal1);
  connect(switchOffSignal1, transformerVariableTap.switchOffSignal1);
  connect(switchOffSignal1, tapChanger.switchOffSignal1);
  connect(switchOffSignal2, motorA.switchOffSignal2);
  connect(switchOffSignal2, motorB.switchOffSignal2);
  connect(switchOffSignal2, motorC.switchOffSignal2);
  connect(switchOffSignal2, motorD.switchOffSignal2);
  connect(switchOffSignal2, electronicLoad.switchOffSignal2);
  connect(switchOffSignal2, staticLoad.switchOffSignal2);
  connect(switchOffSignal2, shuntLowSide.switchOffSignal2);
  connect(switchOffSignal2, reactiveCompensationLowSide.switchOffSignal2);
  connect(switchOffSignal2, reactiveCompensationLoadSide.switchOffSignal2);
  connect(switchOffSignal2, line.switchOffSignal2);
  connect(switchOffSignal2, feeder.switchOffSignal2);
  connect(switchOffSignal2, transformerVariableTap.switchOffSignal2);
  connect(switchOffSignal2, tapChanger.switchOffSignal2);
  connect(deltaP, electronicLoad.deltaP);
  connect(deltaP, staticLoad.deltaP);
  connect(deltaQ, electronicLoad.deltaQ);
  connect(deltaQ, staticLoad.deltaQ);
  connect(PRefPu, electronicLoad.PRefPu);
  connect(PRefPu, staticLoad.PRefPu);
  connect(QRefPu, electronicLoad.QRefPu);
  connect(QRefPu, staticLoad.QRefPu);
  connect(line.terminal2, transformerVariableTap.terminal1) annotation(
    Line(points = {{-60, 0}, {-40, 0}}, color = {0, 0, 255}));
  connect(transformerVariableTap.terminal2, lowSideBus.terminal) annotation(
    Line(points = {{-20, 0}, {0, 0}}, color = {0, 0, 255}));
  connect(shuntLowSide.terminal, lowSideBus.terminal) annotation(
    Line(points = {{-8, -20}, {-8, -6}, {0, -6}, {0, 0}}, color = {0, 0, 255}));
  connect(lowSideBus.terminal, feeder.terminal1) annotation(
    Line(points = {{0, 0}, {20, 0}}, color = {0, 0, 255}));
  connect(feeder.terminal2, loadBus.terminal) annotation(
    Line(points = {{40, 0}, {60, 0}}, color = {0, 0, 255}));
  connect(reactiveCompensationLowSide.terminal, feeder.terminal1) annotation(
    Line(points = {{20, -20}, {20, 0}}, color = {0, 0, 255}));
  connect(reactiveCompensationLoadSide.terminal, loadBus.terminal) annotation(
    Line(points = {{40, -20}, {40, -8}, {60, -8}, {60, 0}}, color = {0, 0, 255}));
  connect(staticLoad.terminal, loadBus.terminal) annotation(
    Line(points = {{90, -50}, {68, -50}, {68, -8}, {60, -8}, {60, 0}}, color = {0, 0, 255}));
  connect(electronicLoad.terminal, loadBus.terminal) annotation(
    Line(points = {{90, -30}, {70, -30}, {70, -6}, {60, -6}, {60, 0}}, color = {0, 0, 255}));
  connect(motorC.terminal, loadBus.terminal) annotation(
    Line(points = {{90, 10}, {72, 10}, {72, 2}, {60, 2}, {60, 0}}, color = {0, 0, 255}));
  connect(motorB.terminal, loadBus.terminal) annotation(
    Line(points = {{90, 30}, {70, 30}, {70, 4}, {60, 4}, {60, 0}}, color = {0, 0, 255}));
  connect(motorA.terminal, loadBus.terminal) annotation(
    Line(points = {{92, 50}, {68, 50}, {68, 6}, {60, 6}, {60, 0}}, color = {0, 0, 255}));
  connect(motorD.terminal, loadBus.terminal) annotation(
    Line(points = {{90, -10}, {72, -10}, {72, -4}, {60, -4}, {60, 0}}, color = {0, 0, 255}));
end CompositeLoadModel;

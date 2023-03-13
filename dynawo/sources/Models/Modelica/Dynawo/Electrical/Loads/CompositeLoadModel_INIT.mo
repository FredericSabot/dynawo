within Dynawo.Electrical.Loads;

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

model CompositeLoadModel_INIT
  extends Load_INIT;
  import Dynawo;

  parameter Types.ApparentPowerModule SNom = P0Pu "Nominal apparent power of the load in MVA";
  parameter Types.ApparentPowerModule SNomTransfo = SNom "Nominal apparent power of the transformer in MVA";
  parameter Real ElectronicShare "Share of electronic loads";
  // Motor A, B and C parameters
  parameter Real MotorABCShare[3] "Share of motor loads (for each of the motors A, B, and C)";
  parameter Types.ApparentPowerModule MotorABCSNom[3] = MotorABCShare * P0Pu * SystemBase.SnRef "Nominal apparent power of a single motor in MVA";
  parameter Types.PerUnit MotorABCRsPu[3] "Stator resistance in pu (base SNom, UNom)";
  parameter Types.PerUnit MotorABCLsPu[3] "Synchronous reactance in pu (base SNom, UNom)";
  parameter Types.PerUnit MotorABCLPPu[3] "Transient reactance in pu (base SNom, UNom)";
  parameter Types.PerUnit MotorABCLPPPu[3] "Subtransient reactance in pu (base SNom, UNom)";
  parameter Types.Time MotorABCtP0[3] "Transient open circuit time constant in s";
  parameter Types.Time MotorABCtPP0[3] "Subtransient open circuit time constant in s";
  parameter Real MotorABCPF[3] "Power factor of the motors (for each of the motors A, B, and C";
  // Motor D parameters
  parameter Real MotorDShare "Share of motor D loads";
  parameter Real MotorDPF "Power factor of the motor";
  parameter Types.VoltageModulePu MotorDUBreakPu = 0.86 "Voltage below which the active power consumed by the motor increases in pu (base UNom)";
  parameter Real MotorDKQ1 = 6 "Proportional factor of reactive power voltage dependance when UPu > UBreakPu";
  parameter Real MotorDNQ1 = 2 "Exponent of the reactive power voltage dependance when UPu > UBreakPu";
  parameter Real MotorDKP2 = 12 "Proportional factor of active power voltage dependance when UPu > UBreakPu";
  parameter Real MotorDNP2 = 3.2 "Exponent of the active power voltage dependance when UPu > UBreakPu";
  parameter Types.PerUnit MotorDXStallPu "Equivalent impedance of the motor when stalled in pu (base SNom, UNom)";
  // Electronic load parameters
  parameter Real ElectronicPF "Power factor of electronic loads";
  // Static load parameters
  parameter Real StaticPF "Power factor of static loads";
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
  // Tap changer parameters
  parameter Types.VoltageModule TapChangerUTarget "Target voltage at the secondary side of the transformer in kV";
  parameter Types.VoltageModule TapChangerUDeadBand(min = 0) "Voltage dead-band in kV";
  parameter Boolean OLTCLocked "True if the on-load tap changer is locked";
  parameter Types.PerUnit ShuntLowSideBPu "Shunt on low-side susceptance in pu (base SNom), negative values for capacitive consumption (over-excited), positive values for inductive consumption (under-excited)";
  parameter Real ShareLowSideFeederCompensation "Share of the feeder compensation that is located in the low-side bus";

protected
  final parameter Real StaticShare = 1 - ElectronicShare - sum(MotorABCShare) - MotorDShare "Share of static loads";

  // The voltage at all the buses of the model are computed manually before being injected to the initiating models of the components;
  final parameter Types.ComplexApparentPowerPu s0Pu_ = Complex(P0Pu, Q0Pu);
  final parameter Types.ComplexVoltagePu u0Pu_ = ComplexMath.fromPolar(U0Pu, UPhase0);
  final parameter Types.ComplexCurrentPu i0Pu_ = ComplexMath.conj(s0Pu_ / u0Pu_);
  final parameter Types.ComplexVoltagePu Tfoi10Pu_ = Lines.GetOppositeLineCurrent(LineRPu, LineXPu, LineGPu, LineBPu, i0Pu_ * SNom / SystemBase.SnRef, u0Pu_);
  final parameter Types.ComplexCurrentPu Tfou10Pu_ = Lines.GetOppositeLineVoltage(LineRPu, LineXPu, LineGPu, LineBPu, i0Pu_ * SNom / SystemBase.SnRef, u0Pu_);
  final parameter Integer TfoTap0_ = Transformers.BaseClasses_INIT.IdealTransformerTapEstimation(TfoMinTapPu, TfoMaxTapPu, TfoNbTap, Tfou10Pu_, TapChangerUTarget);
  final parameter Types.PerUnit TforTfo0Pu_ = TfoMinTapPu + (TfoMaxTapPu - TfoMinTapPu) * (TfoTap0_ / (TfoNbTap - 1));
  final parameter Types.ComplexVoltagePu Tfou20Pu_ = Tfou10Pu_ * TforTfo0Pu_;
  final parameter Types.ComplexCurrentPu Tfoi20Pu_ = - Tfoi10Pu_ / TforTfo0Pu_;  // Receptor convention on both sides of the transformer
  final parameter Types.ComplexCurrentPu iFeederPu = -Tfoi20Pu_ - Tfou20Pu_ * ShuntLowSideBPu;
  final parameter Types.ReactivePowerPu QTotal0Pu = sum(P0Pu * MotorABCShare .* MotorABCPF) + P0Pu * MotorDShare * MotorDPF + P0Pu * ElectronicShare * ElectronicPF + P0Pu * StaticShare * StaticPF "Total reactive power consumed by loads";
  final parameter Types.PerUnit FeederReactiveCompensation = estimateFeederReactiveCompensation(iFeederPu, Tfou10Pu_, ShareLowSideFeederCompensation, FeederRPu, FeederXPu, FeederGPu, FeederBPu, P0Pu, QTotal0Pu);
  final parameter Types.ComplexVoltagePu uLoadSide0Pu_ = Lines.GetOppositeLineVoltage(FeederRPu, FeederXPu, FeederGPu, FeederBPu, iFeederPu - FeederReactiveCompensation * ShareLowSideFeederCompensation * Tfou20Pu_, Tfou20Pu_);
  final parameter Types.VoltageModulePu ULoadSide0Pu_ = ComplexMath.'abs'(uLoadSide0Pu_);
  final parameter Types.Angle UPhaseLoadSide0Pu_ = ComplexMath.arg(uLoadSide0Pu_);

  function estimateFeederReactiveCompensation
    extends Icons.Function;

    input Types.ComplexCurrentPu i1FeederPu "Complex current at side 1 of the feeder in pu (base UNom, SNom)";
    input Types.ComplexVoltagePu u1FeederPu "Complex voltage at side 1 of the feeder in pu (base UNom)";
    input Real ShareLowSideFeederCompensation "Share of the feeder compensation that is located in the low-side bus";
    input Types.PerUnit FeederRPu "Feeder resistance in pu (base SNom)";
    input Types.PerUnit FeederXPu "Feeder reactance in pu (base SNom)";
    input Types.PerUnit FeederGPu "Feeder half-conductance in pu (base SNom)";
    input Types.PerUnit FeederBPu "Feeder half-susceptance in pu (base SNom)";
    input Types.ActivePowerPu LoadBusPPu "Total active power consumption on the load bus in pu (base SNom)";
    input Types.ActivePowerPu LoadBusQPu "Total reactive power consumption on the load bus in pu (base SNom)";

    output Types.PerUnit BPu "Reactance for the reactive compensation";

  protected
    Types.PerUnit BPuMax "Max bound for BPu";
    Types.PerUnit BPuMin "Min bound for BPu";
    Types.ComplexCurrentPu iLoadBusPu "Complex current at the laod bus (base UNom, SNom)";
    Types.ComplexCurrentPu uLoadBusPu "Complex voltage at the laod bus (base UNom)";
    Types.ActivePowerPu estimatedLoadBusQPu "Currently estimated reactive power consumption on the load bus in pu (base SNom)";
    Integer nbIterations "Current number of iterations in the dichotomic search algorithm";
    Boolean iterate "Indicates if the while loop should continue";

  algorithm
    BPuMin := -max(LoadBusPPu, LoadBusQPu);
    BPuMax := max(LoadBusPPu, LoadBusQPu);
    nbIterations := 0;

    while iterate loop

      BPu := (BPuMin + BPuMax) / 2;

      iLoadBusPu := Lines.GetOppositeLineCurrent(FeederRPu, FeederXPu, FeederGPu, FeederBPu, i1FeederPu - u1FeederPu * BPu * ShareLowSideFeederCompensation, u1FeederPu);
      uLoadBusPu := Lines.GetOppositeLineVoltage(FeederRPu, FeederXPu, FeederGPu, FeederBPu, i1FeederPu - u1FeederPu * BPu * ShareLowSideFeederCompensation, u1FeederPu);
      estimatedLoadBusQPu := ComplexMath.imag(uLoadBusPu * ComplexMath.conj(iLoadBusPu)) + BPu * ComplexMath.'abs'(uLoadBusPu)^2 * (1 - ShareLowSideFeederCompensation);

      if estimatedLoadBusQPu < LoadBusQPu - 1e-3 then
        BPuMin := BPu;
      elseif estimatedLoadBusQPu > LoadBusQPu + 1e-3 then
        BPuMax := BPu;
      else
        return;
      end if;
      nbIterations := nbIterations + 1;
      iterate := nbIterations < 10;
    end while;

  end estimateFeederReactiveCompensation;

public
  Types.ComplexVoltagePu uLowSide0Pu "Start value of complex voltage at low-side terminal in pu (base UNom)";
  Types.ComplexVoltagePu uLoadSide0Pu "Start value of complex voltage at load-side terminal in pu (base UNom)";
  // Motor A, B, and C initial values
  Types.PerUnit MotorABCEdP0Pu[3];
  Types.PerUnit MotorABCEqP0Pu[3];
  Types.PerUnit MotorABCEdPP0Pu[3];
  Types.PerUnit MotorABCEqPP0Pu[3];
  Types.PerUnit MotorABCUd0Pu[3] "Start value of voltage of direct axis in pu";
  Types.PerUnit MotorABCUq0Pu[3] "Start value of voltage of quadrature axis in pu";
  Types.PerUnit MotorABCid0Pu[3] "Start value of current of direct axis in pu";
  Types.PerUnit MotorABCiq0Pu[3] "Start value of current of quadrature axis in pu";
  Types.PerUnit MotorABCce0Pu[3] "Start value of the electrical torque in pu (SNom base)";
  Real MotorABCs0[3] "Start value of the slip of the motor";
  Types.AngularVelocity MotorABComegaR0Pu[3] "Start value of the angular velocity of the motor";
  Types.ComplexCurrentPu MotorABCi0Pu[3] "Start value of complex current at load terminal in pu (base UNom, SnRef) (receptor convention)";
  Types.ComplexApparentPowerPu MotorABCs0Pu[3] "Start value of complex apparent power in pu (base SnRef) (receptor convention)";
  // Motor D initial values
  Types.ComplexCurrentPu MotorDi0Pu "Start value of complex current at load terminal in pu (base UNom, SnRef) (receptor convention)";
  Types.ComplexApparentPowerPu MotorDs0Pu "Start value of complex apparent power in pu (base SnRef) (receptor convention)";
  Types.VoltageModulePu MotorDUStallBreakPu "Voltage where the stalling and non-stalling characteristics cross in pu (base UNom)";
  // Electronic load initial values
  Types.ComplexCurrentPu Electronici0Pu "Start value of complex current at load terminal in pu (base UNom, SnRef) (receptor convention)";
  Types.ComplexApparentPowerPu Electronics0Pu "Start value of complex apparent power in pu (base SnRef) (receptor convention)";
  // Static load initial values
  Types.ComplexCurrentPu Statici0Pu "Start value of complex current at load terminal in pu (base UNom, SnRef) (receptor convention)";
  Types.ComplexApparentPowerPu Statics0Pu "Start value of complex apparent power in pu (base SnRef) (receptor convention)";
  // Transformer initial values
  Types.ComplexVoltagePu Tfou10Pu "Transformer start value of complex voltage at terminal 1 in pu (base U1Nom)";
  Types.ComplexCurrentPu Tfoi10Pu "Transformer start value of complex current at terminal 1 in pu (base U1Nom, SNom) (receptor convention)";
  Types.ComplexVoltagePu Tfou20Pu "Transformer start value of complex voltage at terminal 2 in pu (base U2Nom)";
  Types.ComplexCurrentPu Tfoi20Pu "Transformer start value of complex current at terminal 2 in pu (base U2Nom, SNom) (receptor convention)";
  Types.VoltageModulePu TfoU10Pu "Transformer start value of voltage amplitude at terminal 1 in pu (base U1Nom)";
  Types.VoltageModulePu TfoU20Pu "Transformer start value of voltage amplitude at terminal 2 in pu (base U2Nom)";
  Types.ActivePowerPu TfoP10Pu "Transformer start value of active power at terminal 1 in pu (base SNom) (receptor convention)";
  Types.ReactivePowerPu TfoQ10Pu "Transformer start value of reactive power at terminal 1 in pu (base SNom) (receptor convention)";
  Integer TfoTap0 "Transformer start value of transformer tap";
  Types.PerUnit TforTfo0Pu "Transformer start value of transformer ratio";
  // Shunt low side initial values
  Types.ComplexApparentPowerPu ShuntLowSides0Pu "Shunt on low-side start value of apparent power at shunt terminal in pu (base SNom, receptor convention)";
  Types.ComplexCurrentPu ShuntLowSidei0Pu "Shunt on low-side start value of complex current at shunt terminal in pu (base UNom, SNom, receptor convention)";
  // Reactive compensation low side initial values
  Types.PerUnit ReactiveCompensationLowSideB0Pu "Reactive compensation on low-side susceptance in pu (base SNom), negative values for capacitive consumption (over-excited), positive values for inductive consumption (under-excited)";
  Types.ComplexApparentPowerPu ReactiveCompensationLowSides0Pu "Reactive compensation on low-side start value of apparent power at shunt terminal in pu (base SNom, receptor convention)";
  Types.ComplexCurrentPu ReactiveCompensationLowSidei0Pu "Reactive compensation on low-side start value of complex current at shunt terminal in pu (base UNom, SNom, receptor convention)";
  // Reactive compensation load side initial values
  Types.PerUnit ReactiveCompensationLoadSideB0Pu "Reactive compensation on load-side susceptance in pu (base SNom), negative values for capacitive consumption (over-excited), positive values for inductive consumption (under-excited)";
  Types.ComplexApparentPowerPu ReactiveCompensationLoadSides0Pu "Reactive compensation on load-side start value of apparent power at shunt terminal in pu (base SNom, receptor convention)";
  Types.ComplexCurrentPu ReactiveCompensationLoadSidei0Pu "Reactive compensation on load-side start value of complex current at shunt terminal in pu (base UNom, SNom, receptor convention)";

  Dynawo.Electrical.Machines.Motors.MotorFifthOrder_INIT motors_INIT[3](SNom=MotorABCSNom, RsPu=MotorABCRsPu, LsPu=MotorABCLsPu, LPPu=MotorABCLPPu, LPPPu=MotorABCLPPPu, tP0=MotorABCtP0, tPP0=MotorABCtPP0, P0Pu=MotorABCShare*P0Pu, each U0Pu=ULoadSide0Pu_, each UPhase0=UPhaseLoadSide0Pu_);
  Dynawo.Electrical.Machines.Motors.MotorD_INIT motorD_INIT(P0Pu=MotorDShare*P0Pu, PF=MotorDPF, UBreakPu=MotorDUBreakPu, KQ1=MotorDKQ1, NQ1=MotorDNQ1, XStallPu=MotorDXStallPu, U0Pu=ULoadSide0Pu_, UPhase0=UPhaseLoadSide0Pu_);
  Dynawo.Electrical.Loads.LoadPF_INIT electronic_INIT(P0Pu=ElectronicShare*P0Pu, PF=ElectronicPF, U0Pu=ULoadSide0Pu_, UPhase0=UPhaseLoadSide0Pu_);
  Dynawo.Electrical.Loads.LoadPF_INIT static_INIT(P0Pu=StaticShare*P0Pu, PF=StaticPF, U0Pu=ULoadSide0Pu_, UPhase0=UPhaseLoadSide0Pu_);

equation
  Tfoi10Pu = Tfoi10Pu_;
  Tfou10Pu = Tfou10Pu_;
  TfoTap0 = TfoTap0_;
  TforTfo0Pu = TforTfo0Pu_;
  Tfou20Pu = Tfou20Pu_;
  uLowSide0Pu = Tfou20Pu;
  Tfoi20Pu = Tfoi20Pu_;
  uLoadSide0Pu = uLoadSide0Pu_;

  TfoU10Pu = ComplexMath.'abs'(Tfou10Pu);
  TfoU20Pu = ComplexMath.'abs'(Tfou20Pu);
  Complex(TfoP10Pu, TfoQ10Pu) = Tfou10Pu * ComplexMath.conj(Tfou10Pu);

  MotorABCEdP0Pu = motors_INIT.EdP0Pu;
  MotorABCEqP0Pu = motors_INIT.EqP0Pu;
  MotorABCEdPP0Pu = motors_INIT.EdPP0Pu;
  MotorABCEqPP0Pu = motors_INIT.EqPP0Pu;
  MotorABCUd0Pu = motors_INIT.Ud0Pu;
  MotorABCUq0Pu = motors_INIT.Uq0Pu;
  MotorABCid0Pu = motors_INIT.id0Pu;
  MotorABCiq0Pu = motors_INIT.iq0Pu;
  MotorABCce0Pu = motors_INIT.ce0Pu;
  MotorABCs0 = motors_INIT.s0;
  MotorABComegaR0Pu = motors_INIT.omegaR0Pu;
  MotorABCi0Pu = motors_INIT.i0Pu;
  MotorABCs0Pu = motors_INIT.s0Pu;

  MotorDi0Pu = motorD_INIT.i0Pu;
  MotorDs0Pu = motorD_INIT.s0Pu;
  MotorDUStallBreakPu = motorD_INIT.UStallBreakPu;

  Electronici0Pu = electronic_INIT.i0Pu;
  Electronics0Pu = electronic_INIT.s0Pu;

  Statici0Pu = static_INIT.i0Pu;
  Statics0Pu = static_INIT.s0Pu;

  ReactiveCompensationLowSideB0Pu = FeederReactiveCompensation * ShareLowSideFeederCompensation;
  ReactiveCompensationLowSidei0Pu = ReactiveCompensationLowSideB0Pu * Tfou20Pu_;
  ReactiveCompensationLowSides0Pu = Complex(0, ReactiveCompensationLowSideB0Pu * ComplexMath.'abs'(uLowSide0Pu)^2);

  ReactiveCompensationLoadSideB0Pu = FeederReactiveCompensation * (1-ShareLowSideFeederCompensation);
  ReactiveCompensationLoadSidei0Pu = ReactiveCompensationLoadSideB0Pu * Tfou20Pu_;
  ReactiveCompensationLoadSides0Pu = Complex(0, ReactiveCompensationLoadSideB0Pu * ULoadSide0Pu_^2);

  ShuntLowSidei0Pu = ShuntLowSideBPu * uLowSide0Pu;
  ShuntLowSides0Pu = Complex(0, ShuntLowSideBPu * ComplexMath.'abs'(uLowSide0Pu)^2);

end CompositeLoadModel_INIT;

within Dynawo.Electrical.Machines.Motors;

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

model MotorD_INIT
  extends AdditionalIcons.Init;

  parameter Types.ActivePowerPu P0Pu "Start value of active power in pu (base SnRef) (receptor convention)";
  parameter Types.VoltageModulePu U0Pu "Start value of voltage amplitude at load terminal in pu (base UNom)";
  parameter Types.Angle UPhase0 "Start value of voltage angle at load terminal (in rad)";
  parameter Real PF "Power factor of the motor";
  parameter Types.VoltageModulePu UBreakPu = 0.86 "Voltage below which the active power consumed by the motor increases in pu (base UNom)";
  parameter Real KQ1 = 6 "Proportional factor of reactive power voltage dependance when UPu > UBreakPu";
  parameter Real KP2 = 12 "Proportional factor of active power voltage dependance when UPu > UBreakPu";
  parameter Real NQ1 = 2 "Exponent of the reactive power voltage dependance when UPu > UBreakPu";
  parameter Real NP2 = 3.2 "Exponent of the active power voltage dependance when UPu > UBreakPu";
  parameter Types.PerUnit XStallPu "Equivalent impedance of the motor when stalled in pu (base SNom, UNom)";

  Types.ReactivePowerPu Q0Pu "Start value of reactive power in pu (base SnRef) (receptor convention)";
  Types.ComplexVoltagePu u0Pu "Start value of complex voltage at load terminal in pu (base UNom)";
  Types.ComplexApparentPowerPu s0Pu "Start value of complex apparent power in pu (base SnRef) (receptor convention)";
  flow Types.ComplexCurrentPu i0Pu "Start value of complex current at load terminal in pu (base UNom, SnRef) (receptor convention)";
  Types.VoltageModulePu UStallBreakPu "Voltage where the stalling and non-stalling characteristics cross in pu (base UNom)";

equation
  Q0Pu = P0Pu * tan(acos(PF)) - KQ1 * (U0Pu - UBreakPu)^NQ1;
  s0Pu = Complex(P0Pu, Q0Pu);
  s0Pu = u0Pu * ComplexMath.conj(i0Pu);
  u0Pu = ComplexMath.fromPolar(U0Pu, UPhase0);

  UStallBreakPu^2 / XStallPu = P0Pu + KP2 * (UBreakPu - UStallBreakPu)^NP2;

  annotation(preferredView = "text");
end MotorD_INIT;

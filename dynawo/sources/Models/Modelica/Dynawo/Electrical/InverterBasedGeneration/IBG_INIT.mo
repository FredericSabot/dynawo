within Dynawo.Electrical.InverterBasedGeneration;

/*
* Copyright (c) 2021, RTE (http://www.rte-france.com)
* See AUTHORS.txt
* All rights reserved.
* This Source Code Form is subject to the terms of the Mozilla Public
* License, v. 2.0. If a copy of the MPL was not distributed with this
* file, you can obtain one at http://mozilla.org/MPL/2.0/.
* SPDX-License-Identifier: MPL-2.0
*
* This file is part of Dynawo, an hybrid C++/Modelica open source suite of simulation tools for power systems.
*/

model GenericIBG_INIT "Initialization model for genericIBG"
  import Modelica;
  import Modelica.ComplexMath;
  import Dynawo.Types;
  import Dynawo.Electrical.SystemBase;

  extends AdditionalIcons.Init;

  // Rating
  parameter Types.ApparentPowerModule SNom "Nominal apparent power of the injector (in MVA)";
  parameter Types.CurrentModulePu IMaxPu "Maximum current of the injector in pu (base UNom, SNom)";

  parameter Types.PerUnit P0Pu "Start value of active power at regulated bus in pu (generator convention) (base SnRef)";
  parameter Types.PerUnit Q0Pu "Start value of reactive power at regulated bus in pu (generator convention) (base SnRef)";
  parameter Types.PerUnit U0Pu "Start value of voltage magnitude at regulated bus in pu (bae UNom)";
  parameter Types.Angle UPhase0 "Start value of voltage phase angle at regulated bus in rad";

  // Voltage support
  parameter Types.VoltageModulePu US1 "Lower voltage limit of deadband in pu (base UNom)";
  parameter Types.VoltageModulePu US2 "Higher voltage limit of deadband in pu (base UNom)";
  parameter Real kRCI "Slope of reactive current increase for low voltages";
  parameter Real kRCA "Slope of reactive current decrease for high voltages";
  parameter Real m "Current injection just outside of lower deadband in pu (base IMaxPu)";
  parameter Real n "Current injection just outside of lower deadband in pu (base IMaxPu)";

protected
  Types.ComplexPerUnit u0Pu "Start value of complex voltage at terminal in pu (base UNom)";
  Types.ComplexPerUnit s0Pu "Start value of complex apparent power at terminal in pu (base SnRef) (generator convention)";
  Types.ComplexPerUnit i0Pu "Start value of complex current at terminal in pu (base UNom, SnRef) (generator convention)";

  Types.PerUnit Id0Pu "Start value of d-axis current at injector in pu (base UNom, SNom) (generator convention)";
  Types.PerUnit Iq0Pu "Start value of q-axis current at injector in pu (base UNom, SNom) (generator convention)";

  Types.PerUnit IqRef0Pu "Start value of the reference q-axis current at injector in pu (base UNom, SNom) (generator convention)";
  Types.PerUnit IqSup0Pu "Start value of the reactive current support (base Unom, SNom) (generator convention)";

equation
  s0Pu = Complex(P0Pu, Q0Pu);
  u0Pu = ComplexMath.fromPolar(U0Pu, UPhase0);
  s0Pu = u0Pu * ComplexMath.conj(i0Pu);

  Id0Pu = Modelica.Math.cos(UPhase0) * i0Pu.re + Modelica.Math.sin(UPhase0) * i0Pu.im;
  Iq0Pu = Modelica.Math.sin(UPhase0) * i0Pu.re - Modelica.Math.cos(UPhase0) * i0Pu.im;

  if U0Pu < US1 then
    IqSup0Pu = m * IMaxPu + kRCI * (US1 - U0Pu);
  elseif U0Pu < US2 then
    IqSup0Pu = 0;
  else
    IqSup0Pu = -n * IMaxPu + kRCA * (U0Pu - US2);
  end if;
  Iq0Pu = IqRef0Pu + IqSup0Pu;

  annotation(Documentation(preferredView = "text"));
end GenericIBG_INIT;

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

  parameter Types.ApparentPowerModule SNom "Nominal apparent power in MVA";

  parameter Types.PerUnit P0Pu "Start value of active power at regulated bus in pu (receptor convention) (base SnRef)";
  parameter Types.PerUnit Q0Pu "Start value of reactive power at regulated bus in pu (receptor convention) (base SnRef)";
  parameter Types.PerUnit U0Pu "Start value of voltage magnitude at regulated bus in pu (bae UNom)";
  parameter Types.Angle UPhase0 "Start value of voltage phase angle at regulated bus in rad";

  // Voltage support
  parameter Real VS1 "Lower voltage limit of deadband";
  parameter Real VS2 "Higher voltage limit of deadband";

  parameter Real kRCI "Slope of reactive current increase for low voltages";
  parameter Real kRCA "Slope of reactive current decrease for high voltages";

  parameter Real m "Current injection just outside of lower deadband (in pu of Inom)";
  parameter Real n "Current injection just outside of lower deadband (in pu of Inom)";

  parameter Real Inom "Nominal current of injector";

protected
  Types.ComplexPerUnit u0Pu "Start value of complex voltage at terminal in pu (base UNom)";
  Types.ComplexPerUnit s0Pu "Start value of complex apparent power at terminal in pu (base SnRef) (receptor convention)";
  Types.ComplexPerUnit i0Pu "Start value of complex current at terminal in pu (base UNom, SnRef) (receptor convention)";

  Types.PerUnit Id0Pu "Start value of d-axs current at injector in pu (base UNom, SNom) (generator convention)";
  Types.PerUnit Iq0Pu "Start value of q-axis current at injector in pu (base UNom, SNom) (generator convention)";

  Types.PerUnit iQref0Pu "Start value of the reference q-axis current at injector in pu (base UNom, SNom) (generator convention)";
  Types.PerUnit iQsup0Pu "Start value of the reactive current support (base Unom, SNom) (generator convention)";

equation
  u0Pu = ComplexMath.fromPolar(U0Pu, UPhase0);
  s0Pu = Complex(P0Pu, Q0Pu);
  i0Pu = ComplexMath.conj(s0Pu / u0Pu);

  Id0Pu = -1 * (Modelica.Math.cos(UPhase0) * i0Pu.re + Modelica.Math.sin(UPhase0) * i0Pu.im) * (SystemBase.SnRef/SNom);
  Iq0Pu = -1 * (Modelica.Math.sin(UPhase0) * i0Pu.re - Modelica.Math.cos(UPhase0) * i0Pu.im) * (SystemBase.SnRef/SNom);

  if U0Pu < VS1 then
    iQsup0Pu = m * Inom + kRCI * (VS1 - U0Pu);
  elseif U0Pu < VS2 then
    iQsup0Pu = 0;
  else
    iQsup0Pu = -n * Inom + kRCA * (U0Pu - VS2);
  end if;
  Iq0Pu = iQref0Pu + iQsup0Pu;

  annotation(Documentation(preferredView = "text"));
end GenericIBG_INIT;

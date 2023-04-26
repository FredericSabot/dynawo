within Dynawo.Electrical.Buses;

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

model InfiniteBusWithProgressiveVariations "Infinite bus with configurable variations on the voltage module and on the frequency. Voltage variations has a slight ramp for numerical stability"
  import Dynawo.Connectors;
  import Dynawo.Types;
  import Dynawo.Electrical.SystemBase;
  import Modelica.ComplexMath;

  extends AdditionalIcons.Bus;

  Connectors.ACPower terminal annotation(
    Placement(visible = true, transformation(origin = {-1.42109e-14, 98}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {-1.42109e-14, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));

  parameter Types.PerUnit U0Pu "Infinite bus voltage module before and after event in pu (base UNom)";
  parameter Types.PerUnit UEvtPu "Infinite bus voltage module during event in pu (base UNom)";
  parameter Types.PerUnit omega0Pu "Infinite bus angular frequency before and after event in pu (base OmegaNom)";
  parameter Types.PerUnit omegaEvtPu "Infinite bus angular frequency during event in pu (base OmegaNom)";
  parameter Types.Angle UPhase "Infinite bus voltage angle before event in rad";
  parameter Types.Time tUEvtStart "Start time of voltage event in s";
  parameter Types.Time tUEvtEnd "Ending time of voltage event in s";
  parameter Types.Time tEvtRamp = 1e-3 "Duration of variation ramps in s";
  parameter Types.Time tOmegaEvtStart "Start time of frequency event in s";
  parameter Types.Time tOmegaEvtEnd "Ending time of frequency event in s";

  Types.PerUnit UPu "Infinite bus voltage module in pu (base UNom)";
  Types.PerUnit PPu "Infinite bus active power in pu (base SnRef) (receptor convention)";
  Types.PerUnit QPu "Infinite bus reactive power in pu (base SnRef) (receptor convention)";
  Types.PerUnit omegaPu "Infinite bus angular frequency in pu (base OmegaNom)";
  Types.Angle UPhaseOffs "Infinite bus voltage phase shift in rad";

equation

  // Infinite bus equation
  terminal.V = UPu * ComplexMath.exp(ComplexMath.j * (UPhase - UPhaseOffs));

  // Voltage amplitude variation
  if time < tUEvtStart or time >= tUEvtEnd + tEvtRamp then
    UPu = U0Pu;
  elseif time < tUEvtStart + tEvtRamp then
    UPu = U0Pu + (UEvtPu-U0Pu) * (time - tUEvtStart)/tEvtRamp;
  elseif time < tUEvtEnd then
    UPu = UEvtPu;
  else
    UPu = UEvtPu + (U0Pu-UEvtPu) * (time - tUEvtEnd)/tEvtRamp;
  end if;

  // Frequency variation
  UPhaseOffs = 0;
  if time < tOmegaEvtStart or time >= tOmegaEvtEnd + tEvtRamp then
    omegaPu = omega0Pu;
  elseif time < tOmegaEvtStart + tEvtRamp then
    omegaPu = omega0Pu + (omegaEvtPu - omega0Pu) * (time - tOmegaEvtStart) / tEvtRamp;
  elseif time < tOmegaEvtEnd then
    omegaPu = omegaEvtPu;
  else
    omegaPu = omegaEvtPu + (omega0Pu - omegaEvtPu) * (time - tOmegaEvtEnd) / tEvtRamp;
  end if;

  // Outputs signals
  PPu = ComplexMath.real(terminal.V * ComplexMath.conj(terminal.i));
  QPu = ComplexMath.imag(terminal.V * ComplexMath.conj(terminal.i));

  annotation(preferredView = "text",
Documentation(info="<html>
<p> Infinite bus extended with step disturbance in voltage and frequency and measurement signals as output signals </p>
</html>"));
end InfiniteBusWithProgressiveVariations;

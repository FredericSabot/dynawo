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

package BaseClasses_INIT
  extends Icons.BasesPackage;

  partial model BaseMotor_INIT
    extends AdditionalIcons.Init;

    extends AdditionalIcons.Init;

    parameter Types.ApparentPowerModule SNom "Nominal apparent power of the motor in MVA";
    parameter Types.ActivePowerPu P0Pu "Start value of active power in pu (base SnRef) (receptor convention)";
    parameter Types.VoltageModulePu U0Pu "Start value of voltage amplitude at load terminal in pu (base UNom)";
    parameter Types.Angle UPhase0 "Start value of voltage angle at load terminal (in rad)";

    Types.ComplexVoltagePu u0Pu "Start value of complex voltage at load terminal in pu (base UNom)";
    Types.ReactivePowerPu Q0Pu "Start value of reactive power in pu (base SnRef) (receptor convention)";
    Types.ComplexApparentPowerPu s0Pu "Start value of complex apparent power in pu (base SnRef) (receptor convention)";
    flow Types.ComplexCurrentPu i0Pu "Start value of complex current at load terminal in pu (base UNom, SnRef) (receptor convention)";

  equation
    s0Pu = Complex(P0Pu, Q0Pu);
    s0Pu = u0Pu * ComplexMath.conj(i0Pu);
    u0Pu = ComplexMath.fromPolar(U0Pu, UPhase0);

  annotation(preferredView = "text");
  end BaseMotor_INIT;
equation

end BaseClasses_INIT;

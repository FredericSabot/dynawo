within Dynawo.Electrical.Machines.Motors;

/*
* Copyright (c) 2015-2019, RTE (http://www.rte-france.com)
* See AUTHORS.txt
* All rights reserved.
* This Source Code Form is subject to the terms of the Mozilla Public
* License, v. 2.0. If a copy of the MPL was not distributed with this
* file, you can obtain one at http://mozilla.org/MPL/2.0/.
* SPDX-License-Identifier: MPL-2.0
*
* This file is part of Dynawo, an hybrid C++/Modelica open source time domain simulation tool for power systems.
*/

package BaseClasses

  extends Icons.BasesPackage;

  partial model BaseMotor "Base model for motors"
    import Modelica;
    import Dynawo.Connectors;
    import Dynawo.Electrical.Controls.Basics.SwitchOff;
    import Dynawo.Electrical.SystemBase;

    extends SwitchOff.SwitchOffMotor;

    parameter Types.ApparentPowerModule SNom "Nominal apparent power of the motor in MVA";

    parameter Types.ComplexVoltagePu u0Pu "Start value of complex voltage at load terminal in pu (base UNom)";
    parameter Types.ComplexApparentPowerPu s0Pu "Start value of apparent power at load terminal in pu (base SnRef) (receptor convention)";
    parameter Types.ComplexCurrentPu i0Pu "Start value of complex current at load terminal in pu (base UNom, SnRef) (receptor convention)";

    Connectors.ACPower terminal(V(re(start = u0Pu.re), im(start = u0Pu.im)), i(re(start = i0Pu.re), im(start = i0Pu.im))) "Connector used to connect the load to the grid" annotation(
      Placement(visible = true, transformation(origin = {-1.42109e-14, 98}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {-1.42109e-14, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Connectors.ImPin omegaRefPu(value(start = SystemBase.omegaRef0Pu)) "Network angular reference frequency in pu (base omegaNom)";

    Types.ActivePowerPu PPu(start = s0Pu.re) "Active power at load terminal in pu (base SnRef) (receptor convention)";
    Types.ReactivePowerPu QPu(start = s0Pu.im) "Reactive power at load terminal in pu (base SnRef) (receptor convention)";
    Types.ComplexApparentPowerPu SPu(re(start = s0Pu.re), im(start = s0Pu.im)) "Apparent power at load terminal in pu (base SnRef) (receptor convention)";

  equation
    SPu = Complex(PPu, QPu);
    SPu = terminal.V * ComplexMath.conj(terminal.i);

    annotation(preferredView = "text");
  end BaseMotor;
end BaseClasses;

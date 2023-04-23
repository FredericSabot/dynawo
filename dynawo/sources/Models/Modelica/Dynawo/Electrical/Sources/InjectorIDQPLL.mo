within Dynawo.Electrical.Sources;

model InjectorIDQPLL "Injector controlled by d and q current components idPu and iqPu, and with angle given by an external PLL"
  /*
  * Copyright (c) 2022, RTE (http://www.rte-france.com)
  * See AUTHORS.txt
  * All rights reserved.
  * This Source Code Form is subject to the terms of the Mozilla Public
  * License, v. 2.0. If a copy of the MPL was not distributed with this
  * file, you can obtain one at http://mozilla.org/MPL/2.0/.
  * SPDX-License-Identifier: MPL-2.0
  *
  * This file is part of Dynawo, an hybrid C++/Modelica open source suite of simulation tools for power systems.
  */
  import Modelica;
  import Modelica.ComplexMath;
  import Dynawo.Connectors;
  import Dynawo.Electrical.SystemBase;
  import Dynawo.Types;
  import Dynawo.Electrical.Controls.Basics.SwitchOff;

  extends SwitchOff.SwitchOffInjector;

  // Terminal connection
  Connectors.ACPower terminal(V(re(start = u0Pu.re), im(start = u0Pu.im)), i(re(start = i0Pu.re), im(start = i0Pu.im))) "Connector used to connect the injector to the grid" annotation(
    Placement(visible = true, transformation(extent = {{0, 0}, {0, 0}}, rotation = 0), iconTransformation(origin = {115, -79}, extent = {{-15, -15}, {15, 15}}, rotation = 0)));

  parameter Types.ApparentPowerModule SNom "Nominal apparent power in MVA";

  // Inputs: d-q axis pu variables (base UNom, SNom)
  Modelica.Blocks.Interfaces.RealInput idPu(start = Id0Pu) "Injected d-axis current in pu (base SNom, UNom)" annotation(
    Placement(visible = true, transformation(origin = {0, 0},extent = {{0, 0}, {0, 0}}, rotation = 0), iconTransformation(origin = {-115, 80}, extent = {{-15, -15}, {15, 15}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealInput iqPu(start = Iq0Pu) "Injected q-axis current in pu (base SNom, UNom)" annotation(
    Placement(visible = true, transformation(origin = {0, 0},extent = {{0, 0}, {0, 0}}, rotation = 0), iconTransformation(origin = {-115, 40}, extent = {{-15, -15}, {15, 15}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealInput cosPhi(start = cos(UPhase0)) "Cos phi given by the PLL" annotation(
    Placement(visible = true, transformation(origin = {0, 0},extent = {{0, 0}, {0, 0}}, rotation = 0), iconTransformation(origin = {-115, -40}, extent = {{-15, -15}, {15, 15}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealInput sinPhi(start = sin(UPhase0)) "Sin phi given by the PLL" annotation(
    Placement(visible = true, transformation(origin = {0, 0},extent = {{0, 0}, {0, 0}}, rotation = 0), iconTransformation(origin = {-115, -80}, extent = {{-15, -15}, {15, 15}}, rotation = 0)));

  // Outputs:
  Modelica.Blocks.Interfaces.RealOutput UPu(start = U0Pu) "Magnitude voltage at inverter terminal in pu (base UNom)" annotation(
    Placement(visible = true, transformation(extent = {{0, 0}, {0, 0}}, rotation = 0), iconTransformation(origin = {115, 81}, extent = {{-15, -15}, {15, 15}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealOutput QInjPuSn(start = -Q0Pu * SystemBase.SnRef /SNom) "Injected reactive power in pu (base SNom)" annotation(
    Placement(visible = true, transformation(extent = {{0, 0}, {0, 0}}, rotation = 0), iconTransformation(origin = {115, 5}, extent = {{-15, -15}, {15, 15}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealOutput PInjPuSn(start = -P0Pu * SystemBase.SnRef /SNom) "Injected active power in pu (base SNom)" annotation(
    Placement(visible = true, transformation(extent = {{0, 0}, {0, 0}}, rotation = 0), iconTransformation(origin = {115, 43}, extent = {{-15, -15}, {15, 15}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealOutput QInjPu(start = -Q0Pu) "Injected reactive power in pu (base SnRef)";
  Modelica.Blocks.Interfaces.RealOutput PInjPu(start = -P0Pu) "Injected active power in pu (base SnRef)";
  Modelica.ComplexBlocks.Interfaces.ComplexOutput uPu(re(start = u0Pu.re), im(start=u0Pu.im)) "Complex inverter terminal voltage, used as complex connector instead of terminal connector, terminal only used for physical connection, in pu (base UNom)" annotation(
    Placement(visible = true, transformation(extent = {{0, 0}, {0, 0}}, rotation = 0), iconTransformation(origin = {115, -33}, extent = {{-15, -15}, {15, 15}}, rotation = 0)));

  parameter Types.VoltageModulePu U0Pu "Start value of voltage amplitude at injector terminal in pu (base UNom)";
  parameter Types.Angle UPhase0 "Start value of voltage angle at injector terminal in rad";
  parameter Types.ActivePowerPu P0Pu "Start value of active power in pu (base SnRef) (receptor convention)";
  parameter Types.ReactivePowerPu Q0Pu "Start value of reactive power in pu (base SnRef) (receptor convention)";
  parameter Types.ComplexVoltagePu u0Pu "Start value of complex voltage at injector terminal in pu (base UNom)";
  parameter Types.ComplexApparentPowerPu s0Pu "Start value of apparent power at injector terminal in pu (base SnRef) (receptor convention)";
  parameter Types.ComplexCurrentPu i0Pu "Start value of complex current at injector terminal in pu (base UNom, SnRef) (receptor convention)";
  parameter Types.CurrentModulePu Id0Pu "Start value of idPu in pu (base SNom, UNom)";
  parameter Types.CurrentModulePu Iq0Pu "Start value of iqPu in pu (base SNom, UNom)";
equation
  if running.value then
    // Park's transformations dq-currents in generator convention, -> receptor convention for terminal
    terminal.i.re = -1 * (cosPhi * idPu - sinPhi * iqPu) * (SNom/SystemBase.SnRef);
    terminal.i.im = -1 * (sinPhi * idPu + cosPhi * iqPu) * (SNom/SystemBase.SnRef);
    UPu = ComplexMath.'abs'(terminal.V);
    uPu = terminal.V;
    // Active and reactive power in generator convention and SNom base from terminal in receptor base in SnRef
    QInjPuSn = -1 * ComplexMath.imag(terminal.V * ComplexMath.conj(terminal.i))*SystemBase.SnRef/SNom;
    PInjPuSn = -1 * ComplexMath.real(terminal.V * ComplexMath.conj(terminal.i))*SystemBase.SnRef/SNom;
    QInjPu = -1 * ComplexMath.imag(terminal.V * ComplexMath.conj(terminal.i));
    PInjPu = -1 * ComplexMath.real(terminal.V * ComplexMath.conj(terminal.i));
  else
    terminal.i = Complex(0);
    der(UPu) = 0;
    der(uPu.re) = 0;
    der(uPu.im) = 0;
    QInjPuSn = 0;
    PInjPuSn = 0;
    QInjPu = 0;
    PInjPu = 0;
  end if;

  annotation(preferredView = "text",
    Documentation(info="<html> <p> This block calculates the current references for terminal connection based on d-q-frame setpoints from generator control  </p> </html>"),
    Icon(coordinateSystem(initialScale = 0.1), graphics = {Rectangle(extent = {{-100, 100}, {100, -100}}), Text(origin = {-33, 34}, extent = {{-59, 22}, {129, -88}}, textString = "Injector"), Text(origin = {-140, 74}, extent = {{-32, 12}, {4, -4}}, textString = "idPu"), Text(origin = {-142, 36}, extent = {{-32, 12}, {4, -4}}, textString = "iqPu"), Text(origin = {170, -70}, extent = {{-32, 12}, {4, -4}}, textString = "ACPower"), Text(origin = {158, 92}, extent = {{-32, 12}, {4, -4}}, textString = "UPu"), Text(origin = {158, 54}, extent = {{-32, 12}, {24, -8}}, textString = "PInjPuSn"), Text(origin = {158, 24}, extent = {{-32, 12}, {24, -20}}, textString = "QInjPuSn"), Text(origin = {164, -20}, extent = {{-32, 12}, {4, -4}}, textString = "uPu"), Text(origin = {-144, -44}, extent = {{-32, 12}, {4, -4}}, textString = "cosPhi"), Text(origin = {-142, -82}, extent = {{-32, 12}, {4, -4}}, textString = "sinPhi")}));
end InjectorIDQPLL;

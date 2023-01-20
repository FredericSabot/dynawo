within Dynawo.Electrical.Controls.PLL;

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

model PLLFreeze "Phase-Locked Loop that freezes when V < Vm"

  import Modelica;
  import Dynawo.Types;
  import Dynawo.Electrical.SystemBase;

  parameter Types.PerUnit Kp "PLL proportional gain";
  parameter Types.PerUnit Ki "PLL integrator gain";
  parameter Types.PerUnit OmegaMinPu "Lower frequency limit in pu (base OmegaNom)";
  parameter Types.PerUnit OmegaMaxPu "Upper frequency limit in pu (base OmegaNom)";

  Modelica.ComplexBlocks.Interfaces.ComplexInput uPu(re(start = u0Pu.re), im(start = u0Pu.im)) "Complex voltage at PCC in pu (base UNom)" annotation(
    Placement(visible = true, transformation(origin = {-150, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {-110, 60}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealInput omegaRefPu(start = SystemBase.omegaRef0Pu) "Reference frequency of the system in pu (base OmegaNom)" annotation(
    Placement(visible = true, transformation(origin = {-150, -90}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {-110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));

  Modelica.Blocks.Interfaces.RealOutput omegaPLLPu(start = SystemBase.omegaRef0Pu) "Measured frequency in pu (base OmegaNom)" annotation(
    Placement(visible = true, transformation(origin = {190, -84}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110, 50}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealOutput sinPhi(start = Modelica.Math.sin(Modelica.ComplexMath.arg(u0Pu))) "sin(phi) aligned with terminal voltage phasor" annotation(
    Placement(visible = true, transformation(origin = {190, 20}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110, -70}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealOutput cosPhi(start = Modelica.Math.cos(Modelica.ComplexMath.arg(u0Pu))) "cos(phi) aligned with terminal voltage phasor" annotation(
    Placement(visible = true, transformation(origin = {190, -20}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110, -30}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));

  Modelica.Blocks.Math.Product product annotation(
    Placement(visible = true, transformation(origin = {-70, 40}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Math.Product product1 annotation(
    Placement(visible = true, transformation(origin = {-70, -40}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Math.Add add(k1 = -1, k2 = +1) annotation(
    Placement(visible = true, transformation(origin = {-40, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Math.Gain gain(k = Kp) annotation(
    Placement(visible = true, transformation(origin = {0, 20}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Continuous.LimIntegrator limIntegrator(k = Ki, outMax = OmegaMaxPu - SystemBase.omegaRef0Pu, outMin = SystemBase.omegaRef0Pu - OmegaMinPu, y_start = 0) annotation(
    Placement(visible = true, transformation(origin = {0, -20}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Math.Add add1(k1 = +1, k2 = +1) annotation(
    Placement(visible = true, transformation(origin = {40, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Continuous.Integrator integrator(y_start = Modelica.ComplexMath.arg(u0Pu), k = SystemBase.omegaNom) annotation(
    Placement(visible = true, transformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Math.Sin sinPhi1 annotation(
    Placement(visible = true, transformation(origin = {150, 20}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Math.Cos cosPhi1 annotation(
    Placement(visible = true, transformation(origin = {150, -20}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Math.Add add2(k1 = +1, k2 = +1) annotation(
    Placement(visible = true, transformation(origin = {110, -84}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.ComplexBlocks.ComplexMath.ComplexToReal complexToReal annotation(
    Placement(visible = true, transformation(origin = {-120, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Logical.Switch switch1 annotation(
    Placement(visible = true, transformation(origin = {80, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Sources.Constant const(k = 0)  annotation(
    Placement(visible = true, transformation(origin = {50, -41}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Interfaces.BooleanInput freeze annotation(
    Placement(visible = true, transformation(origin = {-150.5, -59.5}, extent = {{-10.5, -10.5}, {10.5, 10.5}}, rotation = 0), iconTransformation(origin = {-110, -80}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
protected
  parameter Types.ComplexVoltagePu u0Pu "Start value of complex voltage at PCC in pu (base UNom)";
equation
  connect(sinPhi1.y, sinPhi) annotation(
    Line(points = {{161, 20}, {190, 20}}, color = {0, 0, 127}));
  connect(cosPhi1.y, cosPhi) annotation(
    Line(points = {{161, -20}, {190, -20}}, color = {0, 0, 127}));
  connect(add2.y, omegaPLLPu) annotation(
    Line(points = {{121, -84}, {190, -84}}, color = {0, 0, 127}));
  connect(uPu, complexToReal.u) annotation(
    Line(points = {{-150, 0}, {-132, 0}}, color = {85, 170, 255}));
  connect(product.y, add.u1) annotation(
    Line(points = {{-59, 40}, {-56, 40}, {-56, 6}, {-52, 6}}, color = {0, 0, 127}));
  connect(product1.y, add.u2) annotation(
    Line(points = {{-59, -40}, {-56, -40}, {-56, -6}, {-52, -6}}, color = {0, 0, 127}));
  connect(complexToReal.re, product.u2) annotation(
    Line(points = {{-108, 6}, {-90, 6}, {-90, 34}, {-82, 34}}, color = {0, 0, 127}));
  connect(complexToReal.im, product1.u1) annotation(
    Line(points = {{-108, -6}, {-90, -6}, {-90, -34}, {-82, -34}}, color = {0, 0, 127}));
  connect(add.y, gain.u) annotation(
    Line(points = {{-29, 0}, {-20, 0}, {-20, 20}, {-12, 20}}, color = {0, 0, 127}));
  connect(add.y, limIntegrator.u) annotation(
    Line(points = {{-29, 0}, {-20, 0}, {-20, -20}, {-12, -20}}, color = {0, 0, 127}));
  connect(gain.y, add1.u1) annotation(
    Line(points = {{11, 20}, {20, 20}, {20, 6}, {28, 6}}, color = {0, 0, 127}));
  connect(limIntegrator.y, add1.u2) annotation(
    Line(points = {{11, -20}, {20, -20}, {20, -6}, {28, -6}}, color = {0, 0, 127}));
  connect(integrator.y, cosPhi1.u) annotation(
    Line(points = {{121, 0}, {130, 0}, {130, -20}, {138, -20}}, color = {0, 0, 127}));
  connect(integrator.y, sinPhi1.u) annotation(
    Line(points = {{121, 0}, {130, 0}, {130, 20}, {138, 20}}, color = {0, 0, 127}));
  connect(sinPhi1.y, product.u1) annotation(
    Line(points = {{161, 20}, {161, 60}, {-90, 60}, {-90, 46}, {-82, 46}}, color = {0, 0, 127}));
  connect(cosPhi1.y, product1.u2) annotation(
    Line(points = {{161, -20}, {161, -60}, {-90, -60}, {-90, -46}, {-82, -46}}, color = {0, 0, 127}));
  connect(omegaRefPu, add2.u2) annotation(
    Line(points = {{-150, -90}, {98, -90}}, color = {0, 0, 127}));
  connect(switch1.y, integrator.u) annotation(
    Line(points = {{91, 0}, {98, 0}}, color = {0, 0, 127}));
  connect(switch1.y, add2.u1) annotation(
    Line(points = {{91, 0}, {93, 0}, {93, -78}, {98, -78}}, color = {0, 0, 127}));
  connect(add1.y, switch1.u1) annotation(
    Line(points = {{51, 0}, {55, 0}, {55, 8}, {68, 8}}, color = {0, 0, 127}));
  connect(const.y, switch1.u3) annotation(
    Line(points = {{61, -41}, {64, -41}, {64, -8}, {68, -8}}, color = {0, 0, 127}));
  connect(freeze, switch1.u2) annotation(
    Line(points = {{-150, -59}, {-100, -59}, {-100, -70}, {30, -70}, {30, -20}, {60, -20}, {60, 0}, {68, 0}}, color = {255, 0, 255}));
  annotation(preferredView = "diagram",
    Documentation(info = "<html><head></head><body><p> The PLL calculates the frequency of the grid voltage by synchronizing the internal phase angle with measured voltage phasor. q-component of internal voltage phasor is therefore controlled to be zero. </p>

<p> Following relationship is used to calculate internal voltage phasor q-component: </p>
<pre>   uqPu = uiPu * cos(phi) - urPu * sin(phi);
</pre>

<p> If uqPu is zero, the internal phasor is locked with the measured phasor and rotates with the same frequency.</p>

</body></html>"),
    Diagram(coordinateSystem(extent = {{-140, -100}, {180, 100}}, initialScale = 1, grid = {1, 1})),
    __OpenModelica_commandLineOptions = "",
    Icon(graphics = {Rectangle(extent = {{-100, 100}, {100, -100}}), Text(origin = {-31, 8}, extent = {{-49, 72}, {111, -88}}, textString = "PLL"), Text(origin = {131, 71}, extent = {{-23, 13}, {49, -25}}, textString = "omegaPLLPu"), Text(origin = {135, -11}, extent = {{-23, 13}, {37, -19}}, textString = "cos(phi)"), Text(origin = {135, -53}, extent = {{-23, 13}, {37, -19}}, textString = "sin(phi)"), Text(origin = {-139, 21}, extent = {{-31, 17}, {37, -19}}, textString = "omegaRefPu"), Text(origin = {-141, 89}, extent = {{3, -3}, {37, -19}}, textString = "uPu"), Text(origin = {-139, 21}, extent = {{-31, 17}, {37, -19}}, textString = "omegaRefPu"), Text(origin = {-137, -55}, extent = {{-31, 17}, {37, -19}}, textString = "freeze")}, coordinateSystem(initialScale = 0.1)));
end PLLFreeze;

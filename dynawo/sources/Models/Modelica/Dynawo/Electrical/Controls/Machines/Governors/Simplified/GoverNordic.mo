within Dynawo.Electrical.Controls.Machines.Governors.Simplified;

/*
* Copyright (c) 2022, RTE (http://www.rte-france.com)
* See AUTHORS.txt
* All rights reserved.
* This Source Code Form is subject to the terms of the Mozilla Public
* License, v. 2.0. If a copy of the MPL was not distributed with this
* file, you can obtain one at http://mozilla.org/MPL/2.0/.
* SPDX-License-Identifier: MPL-2.0
*
* This file is part of Dynawo, an hybrid C++/Modelica open source suite
* of simulation tools for power systems.
*/

model GoverNordic "Governor model for the Nordic 32 test system used for voltage stability studies"
  import Modelica;
  import Dynawo;
  import Dynawo.Electrical.SystemBase;
  import Dynawo.Types;

  parameter Types.PerUnit Ki "Integral gain of PI controller";
  parameter Types.PerUnit Kp "Proportional gain of PI controller";
  parameter Types.PerUnit KSigma "Frequency droop";

  //Nominal parameter
  parameter Types.ActivePower PNom "Nominal active power in MW";

  //Input variables
  Modelica.Blocks.Interfaces.RealInput omegaPu(start = SystemBase.omega0Pu) "Angular frequency in pu (base omegaNom)" annotation(
    Placement(visible = true, transformation(origin = {-300, 60}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-120, 60}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealInput PGenPu(start = Pm0Pu) "Active power generated by the synchronous machine in pu (base SnRef) (generator convention)" annotation(
    Placement(visible = true, transformation(origin = {-300, -20}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-120, -60}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));

  //Output variable
  Modelica.Blocks.Interfaces.RealOutput PmPu(start = Pm0Pu) "Mechanical power in pu (base PNom)" annotation(
    Placement(visible = true, transformation(origin = {290, -100}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110, 2}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));

  Dynawo.Connectors.ImPin PmPuPin(value(start = Pm0Pu)) "Pin for connecting PmPu to the generator";

  Modelica.Blocks.Continuous.Integrator waterFlow(y_start = Pm0Pu) annotation(
    Placement(visible = true, transformation(origin = {190, -40}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Math.Feedback dH annotation(
    Placement(visible = true, transformation(origin = {140, -40}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Sources.Constant const2(k = 1) annotation(
    Placement(visible = true, transformation(origin = {90, -40}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Math.Division flowDivGateOpening annotation(
    Placement(visible = true, transformation(origin = {30, -80}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Math.Feedback feedback annotation(
    Placement(visible = true, transformation(origin = {80, 60}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Math.Gain gain(k = 5) annotation(
    Placement(visible = true, transformation(origin = {130, 60}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Nonlinear.Limiter limiter(uMax = 0.1) annotation(
    Placement(visible = true, transformation(origin = {170, 60}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Continuous.LimIntegrator limIntegrator(outMax = 1, outMin = 0, y_start = Pm0Pu) annotation(
    Placement(visible = true, transformation(origin = {210, 60}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Math.Add govOut annotation(
    Placement(visible = true, transformation(origin = {30, 60}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Math.Gain govKp(k = Kp) annotation(
    Placement(visible = true, transformation(origin = {-50, 80}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Math.Gain govKi(k = Ki) annotation(
    Placement(visible = true, transformation(origin = {-70, 40}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Continuous.Integrator govInt(y_start = Pm0Pu) annotation(
    Placement(visible = true, transformation(origin = {-30, 40}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Math.Add dOmegaPlusDroop(k2 = KSigma) annotation(
    Placement(visible = true, transformation(origin = {-130, 40}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Sources.Constant const1(k = SystemBase.omega0Pu) annotation(
    Placement(visible = true, transformation(origin = {-250, 100}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Dynawo.NonElectrical.Blocks.Continuous.Power headWater(N = 2) annotation(
    Placement(visible = true, transformation(origin = {90, -80}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Math.Product product annotation(
    Placement(visible = true, transformation(origin = {250, -100}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Math.Feedback dOmega annotation(
    Placement(visible = true, transformation(origin = {-180, 100}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Sources.Constant const(k = Pm0Pu) annotation(
    Placement(visible = true, transformation(origin = {-250, 20}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Continuous.FirstOrder firstOrder1(T = 2, y_start = Pm0Pu) annotation(
    Placement(visible = true, transformation(origin = {-210, -20}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Math.Feedback dP annotation(
    Placement(visible = true, transformation(origin = {-180, 20}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Math.Gain perUnitP(k = SystemBase.SnRef / PNom) annotation(
    Placement(visible = true, transformation(origin = {-250, -20}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));

  parameter Types.ActivePowerPu Pm0Pu "Initial mechanical power in pu (base PNom)";

equation
  PmPuPin.value = PmPu;

  connect(waterFlow.y, flowDivGateOpening.u1) annotation(
    Line(points = {{201, -40}, {220, -40}, {220, -20}, {0, -20}, {0, -74}, {18, -74}}, color = {0, 0, 127}));
  connect(govInt.y, govOut.u2) annotation(
    Line(points = {{-19, 40}, {0, 40}, {0, 54}, {18, 54}}, color = {0, 0, 127}));
  connect(govKi.y, govInt.u) annotation(
    Line(points = {{-59, 40}, {-42, 40}}, color = {0, 0, 127}));
  connect(limiter.y, limIntegrator.u) annotation(
    Line(points = {{181, 60}, {198, 60}}, color = {0, 0, 127}));
  connect(gain.y, limiter.u) annotation(
    Line(points = {{141, 60}, {158, 60}}, color = {0, 0, 127}));
  connect(limIntegrator.y, flowDivGateOpening.u2) annotation(
    Line(points = {{221, 60}, {240, 60}, {240, 0}, {-20, 0}, {-20, -86}, {18, -86}}, color = {0, 0, 127}));
  connect(product.y, PmPu) annotation(
    Line(points = {{261, -100}, {290, -100}}, color = {0, 0, 127}));
  connect(feedback.y, gain.u) annotation(
    Line(points = {{89, 60}, {118, 60}}, color = {0, 0, 127}));
  connect(govKp.y, govOut.u1) annotation(
    Line(points = {{-39, 80}, {0, 80}, {0, 66}, {18, 66}}, color = {0, 0, 127}));
  connect(dOmegaPlusDroop.y, govKp.u) annotation(
    Line(points = {{-119, 40}, {-100, 40}, {-100, 80}, {-62, 80}}, color = {0, 0, 127}));
  connect(dOmegaPlusDroop.y, govKi.u) annotation(
    Line(points = {{-119, 40}, {-82, 40}}, color = {0, 0, 127}));
  connect(flowDivGateOpening.y, headWater.u) annotation(
    Line(points = {{41, -80}, {78, -80}}, color = {0, 0, 127}));
  connect(dOmega.y, dOmegaPlusDroop.u1) annotation(
    Line(points = {{-171, 100}, {-160, 100}, {-160, 46}, {-142, 46}}, color = {0, 0, 127}));
  connect(const1.y, dOmega.u1) annotation(
    Line(points = {{-239, 100}, {-188, 100}}, color = {0, 0, 127}));
  connect(omegaPu, dOmega.u2) annotation(
    Line(points = {{-300, 60}, {-180, 60}, {-180, 92}}, color = {0, 0, 127}));
  connect(const.y, dP.u1) annotation(
    Line(points = {{-239, 20}, {-188, 20}}, color = {0, 0, 127}));
  connect(firstOrder1.y, dP.u2) annotation(
    Line(points = {{-199, -20}, {-180, -20}, {-180, 12}}, color = {0, 0, 127}));
  connect(dP.y, dOmegaPlusDroop.u2) annotation(
    Line(points = {{-171, 20}, {-160, 20}, {-160, 34}, {-142, 34}}, color = {0, 0, 127}));
  connect(govOut.y, feedback.u1) annotation(
    Line(points = {{42, 60}, {72, 60}}, color = {0, 0, 127}));
  connect(limIntegrator.y, feedback.u2) annotation(
    Line(points = {{221, 60}, {240, 60}, {240, 20}, {80, 20}, {80, 52}}, color = {0, 0, 127}));
  connect(dH.y, waterFlow.u) annotation(
    Line(points = {{149, -40}, {177, -40}}, color = {0, 0, 127}));
  connect(waterFlow.y, product.u1) annotation(
    Line(points = {{201, -40}, {220, -40}, {220, -94}, {238, -94}}, color = {0, 0, 127}));
  connect(headWater.y, product.u2) annotation(
    Line(points = {{102, -80}, {140, -80}, {140, -106}, {238, -106}}, color = {0, 0, 127}));
  connect(const2.y, dH.u1) annotation(
    Line(points = {{102, -40}, {132, -40}}, color = {0, 0, 127}));
  connect(headWater.y, dH.u2) annotation(
    Line(points = {{102, -80}, {140, -80}, {140, -48}}, color = {0, 0, 127}));
  connect(PGenPu, perUnitP.u) annotation(
    Line(points = {{-300, -20}, {-262, -20}}, color = {0, 0, 127}));
  connect(perUnitP.y, firstOrder1.u) annotation(
    Line(points = {{-238, -20}, {-222, -20}}, color = {0, 0, 127}));

  annotation(preferredView = "diagram",
    Icon(graphics = {Rectangle(extent = {{-100, 100}, {100, -100}}), Text(origin = {0, 120}, lineColor = {0, 0, 255}, fillColor = {0, 0, 255}, extent = {{-60, 20}, {60, -20}}, textString = "%name"), Text(extent = {{-68, 66}, {68, -66}}, textString = "GOV")}, coordinateSystem(initialScale = 0.1)),
    Documentation(info = "<html><head></head><body>This model implements the speed control of the generator frames in the Nordic 32 test system used for voltage stability studies.<div>It consists of a turbine model (lower part) and a speed controller (upper part).&nbsp;</div></body></html>"),
    Diagram(coordinateSystem(extent = {{-280, -140}, {280, 140}})));
end GoverNordic;

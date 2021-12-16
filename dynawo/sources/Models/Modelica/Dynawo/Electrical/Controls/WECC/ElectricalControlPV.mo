within Dynawo.Electrical.Controls.WECC;

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

model ElectricalControlPV "WECC PV Electrical Control REEC"
  import Modelica;
  import Dynawo;
  import Dynawo.Types;
  import Dynawo.Electrical.Controls.WECC.Parameters;

  extends Dynawo.Electrical.Controls.WECC.ElectricalControlCommon;

  Dynawo.Electrical.Controls.WECC.BaseControls.CurrentLimitsCalculation currentLimitsCalculation1(IMaxPu = IMaxPu, PPriority = PPriority) annotation(
    Placement(visible = true, transformation(origin = {410, 29}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
  Dynawo.NonElectrical.Blocks.Continuous.RateLimFirstOrderFreeze rateLimFirstOrderFreeze(T = tPord, k = 1, UseFreeze = true, UseRateLim = true, Y0 = PInj0Pu) annotation(
    Placement(visible = true, transformation(origin = {10, 190}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Nonlinear.Limiter limiter3(uMax = PMaxPu, uMin = PMinPu) annotation(
    Placement(visible = true, transformation(origin = {50, 190}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Sources.Constant DPMax0(k = DPMax) annotation(
    Placement(visible = true, transformation(origin = {-40, 220}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Sources.Constant DPMin0(k = DPMin) annotation(
    Placement(visible = true, transformation(origin = {-40, 160}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Continuous.FirstOrder firstOrder2(T = 0.01, k = 1, y_start = Iq0Pu) annotation(
    Placement(visible = true, transformation(origin = {450, 9}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
  Modelica.Blocks.Continuous.FirstOrder firstOrder3(T = 0.01, k = 1, y_start = Id0Pu) annotation(
    Placement(visible = true, transformation(origin = {450, 49}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
  Modelica.Blocks.Sources.BooleanExpression FRTOn2(y = frtOn)  annotation(
    Placement(visible = true, transformation(origin = {4, 160}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
  Modelica.Blocks.Sources.RealExpression IqMax(y = currentLimitsCalculation1.iqMaxPu)  annotation(
    Placement(visible = true, transformation(origin = {130, -6}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Sources.RealExpression IqMin(y = currentLimitsCalculation1.iqMinPu)  annotation(
    Placement(visible = true, transformation(origin = {130, -33}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Sources.RealExpression UFilteredPu4(y = UFilteredPu)  annotation(
    Placement(visible = true, transformation(origin = {50, 24}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Math.Max max annotation(
    Placement(visible = true, transformation(origin = {50, 73}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Sources.Constant constant1(k = 0.0001)  annotation(
    Placement(visible = true, transformation(origin = {-20, 100}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Math.Max max1 annotation(
    Placement(visible = true, transformation(origin = {50, -90}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Sources.Constant constant2(k = 0.0001) annotation(
    Placement(visible = true, transformation(origin = {-20, -110}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));

protected
  parameter Types.PerUnit PInj0Pu "Start value of active power at injector terminal in p.u (generator convention) (base SNom)";
  parameter Types.PerUnit QInj0Pu "Start value of reactive power at injector terminal in p.u (generator convention) (base SNom)";
  parameter Types.PerUnit UInj0Pu "Start value of voltage magnitude at injector terminal in p.u (base UNom)";
  parameter Types.PerUnit PF0 "Start value of powerfactor";
  parameter Types.CurrentModulePu Id0Pu "Start value of d-component current at injector terminal in p.u (generator convention) (base SNom, UNom)";
  parameter Types.CurrentModulePu Iq0Pu "Start value of q-component current at injector terminal in p.u (generator convention) (base SNom, UNom)";

equation
  connect(limiter1.y, add1.u1) annotation(
    Line(points = {{291, 30}, {300, 30}, {300, -15}, {318, -15}}, color = {0, 0, 127}));
  connect(PInjRefPu, rateLimFirstOrderFreeze.u) annotation(
    Line(points = {{-270, 190}, {-2, 190}}, color = {0, 0, 127}));
  connect(rateLimFirstOrderFreeze.y, limiter3.u) annotation(
    Line(points = {{21, 190}, {38, 190}}, color = {0, 0, 127}));
  connect(currentLimitsCalculation1.ipMaxPu, variableLimiter1.limit1) annotation(
    Line(points = {{399, 31}, {360, 31}, {360, 87}, {444, 87}}, color = {0, 0, 127}));
  connect(currentLimitsCalculation1.iqMinPu, variableLimiter.limit2) annotation(
    Line(points = {{399, 27}, {360, 27}, {360, -29}, {444, -29}}, color = {0, 0, 127}));
  connect(currentLimitsCalculation1.iqMaxPu, variableLimiter.limit1) annotation(
    Line(points = {{399, 23}, {380, 23}, {380, -13}, {444, -13}}, color = {0, 0, 127}));
  connect(firstOrder3.y, currentLimitsCalculation1.ipCmdPu) annotation(
    Line(points = {{439, 49}, {430, 49}, {430, 33}, {421, 33}}, color = {0, 0, 127}));
  connect(firstOrder2.y, currentLimitsCalculation1.iqCmdPu) annotation(
    Line(points = {{439, 9}, {430, 9}, {430, 25}, {421, 25}}, color = {0, 0, 127}));
  connect(variableLimiter.y, firstOrder2.u) annotation(
    Line(points = {{421, -21}, {480, -21}, {480, 9}, {462, 9}}, color = {0, 0, 127}));
  connect(currentLimitsCalculation1.ipMinPu, variableLimiter1.limit2) annotation(
    Line(points = {{399, 35}, {380, 35}, {380, 71}, {444, 71}}, color = {0, 0, 127}));
  connect(FRTOn2.y, rateLimFirstOrderFreeze.freeze) annotation(
    Line(points = {{4, 171}, {4, 178}}, color = {255, 0, 255}));
  connect(IqMax.y, varLimPIDFreeze.yMax) annotation(
    Line(points = {{141, -6}, {150, -6}, {150, -13}, {168, -13}, {168, -13}}, color = {0, 0, 127}));
  connect(IqMin.y, varLimPIDFreeze.yMin) annotation(
    Line(points = {{141, -33}, {150, -33}, {150, -25}, {168, -25}}, color = {0, 0, 127}));
  connect(variableLimiter1.y, firstOrder3.u) annotation(
    Line(points = {{421, 79}, {480, 79}, {480, 49}, {462, 49}, {462, 49}}, color = {0, 0, 127}));
  connect(switch1.y, switch.u3) annotation(
    Line(points = {{-109, 20}, {-100, 20}, {-100, -40}, {-20, -40}, {-20, -27}, {-2, -27}, {-2, -27}}, color = {0, 0, 127}));
  connect(limiter3.y, division1.u1) annotation(
    Line(points = {{61, 190}, {70, 190}, {70, 85}, {78, 85}, {78, 85}}, color = {0, 0, 127}));
  connect(DPMin0.y, rateLimFirstOrderFreeze.dyMin) annotation(
    Line(points = {{-29, 160}, {-20, 160}, {-20, 184}, {-1, 184}}, color = {0, 0, 127}));
  connect(DPMax0.y, rateLimFirstOrderFreeze.dyMax) annotation(
    Line(points = {{-29, 220}, {-20, 220}, {-20, 197}, {-1, 197}, {-1, 197}}, color = {0, 0, 127}));
  annotation(preferredView = "diagram",
    Documentation(info = "<html>
    <p> This block contains the electrical inverter control of the generic WECC PV model according to (in case page cannot be found, copy link in browser): <a href='https://www.wecc.biz/Reliability/WECC%20Solar%20Plant%20Dynamic%20Modeling%20Guidelines.pdf/'>https://www.wecc.biz/Reliability/WECC%20Solar%20Plant%20Dynamic%20Modeling%20Guidelines.pdf </a> </p>
<p> Following control modes can be activated:
<li> local coordinated V/Q control: QFlag = true, VFlag = true </li>
<li> only plant level control active: QFlag = false, VFlag = false</li>
<li> if plant level control not connected: local powerfactor control: PfFlag = true, otherwise PfFlag = false.</li>
<p> The block calculates the idCmdPu and iqCmdPu setpoint values for the generator control based on the selected control algorithm.
</ul> </p></html>"),
    Diagram(coordinateSystem(initialScale = 0.2)),
  Icon(graphics = {Rectangle(extent = {{-100, 100}, {100, -100}}), Text(origin = {137, 79}, extent = {{-23, 13}, {35, -21}}, textString = "idCmdPu"), Text(origin = {139, -41}, extent = {{-23, 13}, {35, -21}}, textString = "iqCmdPu"), Text(origin = {141, 13}, extent = {{-23, 13}, {17, -11}}, textString = "frtOn"), Text(origin = {89, -113}, extent = {{-23, 13}, {9, -3}}, textString = "UPu"), Text(origin = {-19, -117}, extent = {{-33, 21}, {9, -3}}, textString = "QInjPu"), Text(origin = {41, -117}, extent = {{-33, 21}, {9, -3}}, textString = "PInjPu"), Text(origin = {-135, 79}, extent = {{-23, 13}, {35, -21}}, textString = "PInjRefPu"), Text(origin = {-135, -41}, extent = {{-23, 13}, {35, -21}}, textString = "QInjRefPu"), Text(origin = {-135, 21}, extent = {{-23, 13}, {35, -21}}, textString = "UFilteredPu")}, coordinateSystem(initialScale = 0.1, preserveAspectRatio = false)));

end ElectricalControlPV;

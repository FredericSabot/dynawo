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

model ElectricalControlWind "WECC Wind Electrical Control REEC"
  import Modelica;
  import Dynawo;
  import Dynawo.Types;
  import Modelica.SIunits;
  import Dynawo.Electrical.Controls.WECC.Parameters;

  extends Dynawo.Electrical.Controls.WECC.ElectricalControlCommon;

  parameter Real[:, :] VDLIpPoints "Pair of points for voltage dependent active current limitation piecewise linear curve [u1,y1; u2,y2;...]";
  parameter Real[:, :] VDLIqPoints "Pair of points for voltage dependent reactive current limitation piecewise linear curve [u1,y1; u2,y2;...]";
  parameter SIunits.Time tIpMax "Time delay for which the active current limit (ipMaxPu) is held after voltage dip vDip returns to zero for tIpMax seconds at its value during the voltage dip";
  parameter Real tIq "Absolute value of tIq defines seconds to hold current injection after voltage dip ended. tIq < 0 for constant, 0 for no injection after voltage dip, tIq > 0 for voltage dependent injection (typical: -1 .. 1 s)";
  parameter SIunits.PerUnit IqFrzPu "Constant reactive current injection value (typical: -0.1 .. 0.1 p.u.)";
  parameter Boolean PFlag "Power reference flag: const. Pref (0) or consider generator speed (1)";

  Modelica.Blocks.Interfaces.RealInput omegaGPu "Generator frequency from drive train control in p.u (base omegaNom)" annotation(
    Placement(visible = true, transformation(origin = {-269, 134}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(extent = {{-121, -106}, {-101, -86}}, rotation = 0)));

  Modelica.Blocks.Sources.RealExpression UFilteredPu5(y = UFilteredPu) annotation(
    Placement(visible = true, transformation(origin = {526, 30}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
  Modelica.Blocks.Sources.RealExpression IqMax(y = currentLimitsCalculation1.iqMaxPu)  annotation(
    Placement(visible = true, transformation(origin = {130, -6}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Sources.RealExpression IqMin(y = currentLimitsCalculation1.iqMinPu)  annotation(
    Placement(visible = true, transformation(origin = {130, -33}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Sources.BooleanExpression FRTOn2(y = frtOn)  annotation(
    Placement(visible = true, transformation(origin = {4, 145}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
  Modelica.Blocks.Sources.BooleanExpression FRTOn4(y = frtOn) annotation(
    Placement(visible = true, transformation(origin = {450, 29}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
  Modelica.Blocks.Sources.BooleanExpression FRTOn5(y = frtOn) annotation(
    Placement(visible = true, transformation(origin = {292, 60}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Sources.Constant constant1(k = 0.0001)  annotation(
    Placement(visible = true, transformation(origin = {-20, 100}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Sources.Constant constant2(k = 0.0001) annotation(
    Placement(visible = true, transformation(origin = {-20, -110}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Sources.Constant Vref(k = if VRef0Pu < 0.5 then UInj0Pu else VRef0Pu) annotation(
    Placement(visible = true, transformation(origin = {-80, -37}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Sources.BooleanConstant PflagConst(k = PFlag)  annotation(
    Placement(visible = true, transformation(origin = {-193, 174}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));

  Modelica.Blocks.Nonlinear.Limiter limiter3(uMax = PMaxPu, uMin = PMinPu) annotation(
    Placement(visible = true, transformation(origin = {50, 175}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Nonlinear.SlewRateLimiter slewRateLimiter(Falling = DPMin, Rising = DPMax, initType = Modelica.Blocks.Types.Init.SteadyState, y_start = PInj0Pu)  annotation(
    Placement(visible = true, transformation(origin = {-229, 190}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Continuous.FirstOrder firstOrder2(T = 0.01, k = 1, y_start = Iq0Pu) annotation(
    Placement(visible = true, transformation(origin = {450, -4}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
  Modelica.Blocks.Continuous.FirstOrder firstOrder3(T = 0.01, k = 1, y_start = Id0Pu) annotation(
    Placement(visible = true, transformation(origin = {450, 64}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
  Modelica.Blocks.Logical.Switch pflagswitch annotation(
    Placement(visible = true, transformation(origin = {-136, 175}, extent = {{-10, 10}, {10, -10}}, rotation = 0)));
  Modelica.Blocks.Math.Max max annotation(
    Placement(visible = true, transformation(origin = {50, 73}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Math.Max max1 annotation(
    Placement(visible = true, transformation(origin = {50, -90}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Math.Add add(k1 = +1, k2 = -1) annotation(
    Placement(visible = true, transformation(origin = {-39, -51}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Math.Product product2 annotation(
    Placement(visible = true, transformation(origin = {-193, 140}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Tables.CombiTable1Ds IpmaxFromUPu(extrapolation = Modelica.Blocks.Types.Extrapolation.HoldLastPoint, smoothness = Modelica.Blocks.Types.Smoothness.LinearSegments, table = VDLIpPoints, tableOnFile = false, verboseExtrapolation = false, verboseRead = false)  annotation(
    Placement(visible = true, transformation(origin = {483, 42}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
  Modelica.Blocks.Tables.CombiTable1Ds IqmaxFromUPu(extrapolation = Modelica.Blocks.Types.Extrapolation.HoldLastPoint, smoothness = Modelica.Blocks.Types.Smoothness.LinearSegments, table = VDLIqPoints, tableOnFile = false, verboseExtrapolation = false, verboseRead = false)  annotation(
    Placement(visible = true, transformation(origin = {482, 14}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));

  Dynawo.Electrical.Controls.WECC.Utilities.IqInjectionLogic iqInjectionLogic(IqFrzPu = IqFrzPu, tIq = tIq)  annotation(
    Placement(visible = true, transformation(origin = {311, 12}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
  Dynawo.Electrical.Controls.WECC.BaseControls.CurrentLimitsCalculationWind currentLimitsCalculation1(IMaxPu = IMaxPu, PPriority = PPriority, tIpMax = tIpMax) annotation(
    Placement(visible = true, transformation(origin = {410, 29}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
  Dynawo.NonElectrical.Blocks.Continuous.RateLimFirstOrderFreeze rateLimFirstOrderFreeze(T = tPord, k = 1, UseFreeze = true, UseRateLim = true, Y0 = PInj0Pu) annotation(
    Placement(visible = true, transformation(origin = {10, 175}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));

protected
  parameter Types.PerUnit PInj0Pu "Start value of active power at injector terminal in p.u (generator convention) (base SNom)";
  parameter Types.PerUnit QInj0Pu "Start value of reactive power at injector terminal in p.u (generator convention) (base SNom)";
  parameter Types.PerUnit UInj0Pu "Start value of voltage magnitude at injector terminal in p.u (base UNom)";
  parameter Types.PerUnit PF0 "Start value of powerfactor";
  parameter Types.CurrentModulePu Id0Pu "Start value of d-component current at injector terminal in p.u (generator convention) (base SNom, UNom)";
  parameter Types.CurrentModulePu Iq0Pu "Start value of q-component current at injector terminal in p.u (generator convention) (base SNom, UNom)";
equation
  connect(rateLimFirstOrderFreeze.y, limiter3.u) annotation(
    Line(points = {{21, 175}, {38, 175}}, color = {0, 0, 127}));
  connect(currentLimitsCalculation1.ipMaxPu, variableLimiter1.limit1) annotation(
    Line(points = {{399, 31}, {360, 31}, {360, 87}, {398, 87}}, color = {0, 0, 127}));
  connect(currentLimitsCalculation1.iqMinPu, variableLimiter.limit2) annotation(
    Line(points = {{399, 27}, {360, 27}, {360, -29}, {398, -29}}, color = {0, 0, 127}));
  connect(currentLimitsCalculation1.iqMaxPu, variableLimiter.limit1) annotation(
    Line(points = {{399, 23}, {380, 23}, {380, -13}, {398, -13}}, color = {0, 0, 127}));
  connect(firstOrder3.y, currentLimitsCalculation1.ipCmdPu) annotation(
    Line(points = {{439, 64}, {430, 64}, {430, 33}, {421, 33}}, color = {0, 0, 127}));
  connect(firstOrder2.y, currentLimitsCalculation1.iqCmdPu) annotation(
    Line(points = {{439, -4}, {430, -4}, {430, 25}, {421, 25}}, color = {0, 0, 127}));
  connect(variableLimiter.y, firstOrder2.u) annotation(
    Line(points = {{421, -21}, {480, -21}, {480, -4}, {462, -4}}, color = {0, 0, 127}));
  connect(currentLimitsCalculation1.ipMinPu, variableLimiter1.limit2) annotation(
    Line(points = {{399, 35}, {380, 35}, {380, 71}, {398, 71}}, color = {0, 0, 127}));
  connect(FRTOn2.y, rateLimFirstOrderFreeze.freeze) annotation(
    Line(points = {{4, 156}, {4, 163}}, color = {255, 0, 255}));
  connect(IqMax.y, varLimPIDFreeze.yMax) annotation(
    Line(points = {{141, -6}, {150, -6}, {150, -13}, {168, -13}, {168, -13}}, color = {0, 0, 127}));
  connect(IqMin.y, varLimPIDFreeze.yMin) annotation(
    Line(points = {{141, -33}, {150, -33}, {150, -25}, {168, -25}}, color = {0, 0, 127}));
  connect(variableLimiter1.y, firstOrder3.u) annotation(
    Line(points = {{421, 79}, {480, 79}, {480, 64}, {462, 64}}, color = {0, 0, 127}));
  connect(limiter3.y, division1.u1) annotation(
    Line(points = {{61, 175}, {70, 175}, {70, 85}, {78, 85}}, color = {0, 0, 127}));
  connect(IpmaxFromUPu.y, currentLimitsCalculation1.ipVdlPu) annotation(
    Line(points = {{472, 42}, {427, 42}, {427, 37}, {421, 37}}, color = {0, 0, 127}));
  connect(IqmaxFromUPu.y, currentLimitsCalculation1.iqVdlPu) annotation(
    Line(points = {{471, 14}, {426, 14}, {426, 21}, {421, 21}}, color = {0, 0, 127}));
  connect(limiter1.y, iqInjectionLogic.iqV) annotation(
    Line(points = {{291, 30}, {311, 30}, {311, 22}}, color = {0, 0, 127}));
  connect(FRTOn5.y, iqInjectionLogic.vDip) annotation(
    Line(points = {{303, 60}, {319, 60}, {319, 22}}, color = {255, 0, 255}));
  connect(iqInjectionLogic.iqInjPu, add1.u1) annotation(
    Line(points = {{311, 2}, {314.5, 2}, {314.5, -7}, {318, -7}, {318, -15}}, color = {0, 0, 127}));
  connect(Vref.y, add.u1) annotation(
    Line(points = {{-69, -37}, {-60, -37}, {-60, -45}, {-51, -45}}, color = {0, 0, 127}));
  connect(switch1.y, add.u2) annotation(
    Line(points = {{-109, 20}, {-100, 20}, {-100, -57}, {-51, -57}}, color = {0, 0, 127}));
  connect(add.y, switch.u3) annotation(
    Line(points = {{-28, -51}, {-2, -51}, {-2, -27}}, color = {0, 0, 127}));
  connect(FRTOn4.y, currentLimitsCalculation1.vDip) annotation(
    Line(points = {{439, 29}, {421, 29}}, color = {255, 0, 255}));
  connect(FRTOn5.y, iqInjectionLogic.vDip) annotation(
    Line(points = {{303, 60}, {319, 60}, {319, 22}}, color = {255, 0, 255}));
  connect(UFilteredPu5.y, IpmaxFromUPu.u) annotation(
    Line(points = {{515, 30}, {503, 30}, {503, 42}, {495, 42}}, color = {0, 0, 127}));
  connect(UFilteredPu5.y, IqmaxFromUPu.u) annotation(
    Line(points = {{515, 30}, {503, 30}, {503, 14}, {494, 14}}, color = {0, 0, 127}));
  connect(product2.y, pflagswitch.u1) annotation(
    Line(points = {{-182, 140}, {-160, 140}, {-160, 167}, {-148, 167}}, color = {0, 0, 127}));
  connect(omegaGPu, product2.u2) annotation(
    Line(points = {{-269, 134}, {-205, 134}}, color = {0, 0, 127}));
  connect(PInjRefPu, slewRateLimiter.u) annotation(
    Line(points = {{-270, 190}, {-241, 190}}, color = {0, 0, 127}));
  connect(slewRateLimiter.y, product2.u1) annotation(
    Line(points = {{-218, 190}, {-209, 190}, {-209, 146}, {-200, 146}}, color = {0, 0, 127}));
  connect(slewRateLimiter.y, pflagswitch.u3) annotation(
    Line(points = {{-218, 190}, {-160, 190}, {-160, 183}, {-148, 183}}, color = {0, 0, 127}));
  connect(pflagswitch.y, rateLimFirstOrderFreeze.u) annotation(
    Line(points = {{-125, 175}, {-2, 175}}, color = {0, 0, 127}));
  connect(PflagConst.y, pflagswitch.u2) annotation(
    Line(points = {{-182, 174}, {-148, 174}, {-148, 175}}, color = {255, 0, 255}));
  annotation(preferredView = "diagram",
    Documentation(info = "<html>
    <p> This block contains the electrical inverter control of the generic WECC PV model according to (in case page cannot be found, copy link in browser): <a href='https://www.wecc.biz/Reliability/WECC%20Solar%20Plant%20Dynamic%20Modeling%20Guidelines.pdf/'>https://www.wecc.biz/Reliability/WECC%20Solar%20Plant%20Dynamic%20Modeling%20Guidelines.pdf </a> </p>
<p> Following control modes can be activated:
<li> local coordinated V/Q control: QFlag = true, VFlag = true </li>
<li> only plant level control active: QFlag = false, VFlag = false</li>
<li> if plant level control not connected: local powerfactor control: PfFlag = true, otherwise PfFlag = false.</li>
<p> The block calculates the idCmdPu and iqCmdPu setpoint values for the generator control based on the selected control algorithm.
</ul> </p></html>"),
    Diagram(coordinateSystem(extent = {{-260, -130}, {540, 250}}, grid = {1, 1})),
  Icon(graphics = {Rectangle(extent = {{-100, 100}, {100, -100}}), Text(origin = {-23, 22}, extent = {{-57, 58}, {103, -102}}, textString = "Electrical Control"), Text(origin = {-157.362, -90.7059}, extent = {{-18.6379, 10.7059}, {28.3621, -17.2941}}, textString = "omegaG"), Text(origin = {139, -41}, extent = {{-23, 13}, {35, -21}}, textString = "iqCmdPu"), Text(origin = {141, 13}, extent = {{-23, 13}, {17, -11}}, textString = "frtOn"), Text(origin = {89, -113}, extent = {{-23, 13}, {9, -3}}, textString = "UPu"), Text(origin = {-19, -117}, extent = {{-33, 21}, {9, -3}}, textString = "QInjPu"), Text(origin = {41, -117}, extent = {{-33, 21}, {9, -3}}, textString = "PInjPu"), Text(origin = {-135, 79}, extent = {{-23, 13}, {35, -21}}, textString = "PInjRefPu"), Text(origin = {-135, -41}, extent = {{-23, 13}, {35, -21}}, textString = "QInjRefPu"), Text(origin = {-135, 21}, extent = {{-23, 13}, {35, -21}}, textString = "UFilteredPu")}, coordinateSystem(extent = {{-260, -130}, {540, 250}}, grid = {1, 1})));

end ElectricalControlWind;

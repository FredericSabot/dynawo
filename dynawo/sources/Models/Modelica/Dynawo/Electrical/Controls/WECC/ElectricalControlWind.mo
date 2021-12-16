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
  import Dynawo.Electrical.Controls.WECC.Parameters;

  extends Dynawo.Electrical.Controls.WECC.ElectricalControlCommon;

  parameter Real[:, :] VDLIpPoints "Pair of points for voltage dependent active current limitation piecewise linear curve [u1,y1; u2,y2;...]";
  parameter Real[:, :] VDLIqPoints "Pair of points for voltage dependent reactive current limitation piecewise linear curve [u1,y1; u2,y2;...]";
//  parameter Real[1, 1] VDLIpPoints;
//  parameter Real[1, 1] VDLIqPoints;
  parameter Types.PerUnit VRef1Pu "User-defined reference/bias on the inner-loop voltage control (typical: 0 p.u.)";
  parameter Types.Time HoldIpMax "Time delay for which the active current limit (ipMaxPu) is held after voltage dip vDip returns to zero for HoldIpMax seconds at its value during the voltage dip";
  parameter Real HoldIq "Absolute value of HoldIq defines seconds to hold current injection after voltage dip ended. HoldIq < 0 for constant, 0 for no injection after voltage dip, HoldIq > 0 for voltage dependent injection (typical: -1 .. 1 s)";
  parameter Types.PerUnit IqFrzPu "Constant reactive current injection value (typical: -0.1 .. 0.1 p.u.)";
  parameter Boolean PFlag "Power reference flag: const. Pref (0) or consider generator speed (1)";

  Modelica.Blocks.Interfaces.RealInput omegaGPu "Generator frequency from drive train control in p.u (base omegaNom)" annotation(
    Placement(visible = true, transformation(origin = {-269, 134}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {-51, -110}, extent = {{10, 10}, {-10, -10}}, rotation = -90)));

  Modelica.Blocks.Sources.RealExpression UFilteredPu5(y = UFilteredPu) annotation(
    Placement(visible = true, transformation(origin = {340, 32}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Sources.RealExpression IqMax(y = currentLimitsCalculation1.iqMaxPu)  annotation(
    Placement(visible = true, transformation(origin = {130, -6}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Sources.RealExpression IqMin(y = currentLimitsCalculation1.iqMinPu)  annotation(
    Placement(visible = true, transformation(origin = {130, -33}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Sources.BooleanExpression FRTOn2(y = frtOn)  annotation(
    Placement(visible = true, transformation(origin = {4, 145}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
  Modelica.Blocks.Sources.BooleanExpression FRTOn4(y = frtOn) annotation(
    Placement(visible = true, transformation(origin = {395, 27}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Sources.BooleanExpression FRTOn5(y = frtOn) annotation(
    Placement(visible = true, transformation(origin = {292, 60}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Sources.Constant VRefConst1(k = VRef1Pu) annotation(
    Placement(visible = true, transformation(origin = {-80, -37}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Sources.BooleanConstant PFlagConst(k = PFlag)  annotation(
    Placement(visible = true, transformation(origin = {-193, 174}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));

  Modelica.Blocks.Sources.Constant DPMax0(k = 999) annotation(
    Placement(visible = true, transformation(origin = {-40, 202}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Sources.Constant DPMin0(k = -999) annotation(
    Placement(visible = true, transformation(origin = {-40, 158}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));

  Modelica.Blocks.Nonlinear.Limiter limiter3(limitsAtInit = true, uMax = PMaxPu, uMin = PMinPu) annotation(
    Placement(visible = true, transformation(origin = {50, 175}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Nonlinear.SlewRateLimiter slewRateLimiter(Falling = DPMin, Rising = DPMax, initType = Modelica.Blocks.Types.Init.SteadyState, y_start = PInj0Pu)  annotation(
    Placement(visible = true, transformation(origin = {-229, 190}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Logical.Switch pflagswitch annotation(
    Placement(visible = true, transformation(origin = {-136, 175}, extent = {{-10, 10}, {10, -10}}, rotation = 0)));
  Modelica.Blocks.Math.Add add2(k1 = +1, k2 = -1) annotation(
    Placement(visible = true, transformation(origin = {-39, -51}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Math.Product product2 annotation(
    Placement(visible = true, transformation(origin = {-193, 140}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Tables.CombiTable1Ds IpmaxFromUPu(extrapolation = Modelica.Blocks.Types.Extrapolation.HoldLastPoint, smoothness = Modelica.Blocks.Types.Smoothness.LinearSegments, table = VDLIpPoints, tableOnFile = false, verboseExtrapolation = false, verboseRead = false)  annotation(
    Placement(visible = true, transformation(origin = {370, 44}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Tables.CombiTable1Ds IqmaxFromUPu(extrapolation = Modelica.Blocks.Types.Extrapolation.HoldLastPoint, smoothness = Modelica.Blocks.Types.Smoothness.LinearSegments, table = VDLIqPoints, tableOnFile = false, verboseExtrapolation = false, verboseRead = false)  annotation(
    Placement(visible = true, transformation(origin = {370, 16}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));

  Dynawo.Electrical.Controls.WECC.Utilities.IqInjectionLogic iqInjectionLogic(IqFrzPu = IqFrzPu, HoldIq = HoldIq)  annotation(
    Placement(visible = true, transformation(origin = {311, 12}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
  Dynawo.Electrical.Controls.WECC.BaseControls.CurrentLimitsCalculationWind currentLimitsCalculation1(IMaxPu = IMaxPu, PPriority = PPriority, HoldIpMax = HoldIpMax) annotation(
    Placement(visible = true, transformation(origin = {432, 27}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
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
  connect(FRTOn2.y, rateLimFirstOrderFreeze.freeze) annotation(
    Line(points = {{4, 156}, {4, 163}}, color = {255, 0, 255}));
  connect(IqMax.y, varLimPIDFreeze.yMax) annotation(
    Line(points = {{141, -6}, {150, -6}, {150, -13}, {168, -13}, {168, -13}}, color = {0, 0, 127}));
  connect(IqMin.y, varLimPIDFreeze.yMin) annotation(
    Line(points = {{141, -33}, {150, -33}, {150, -25}, {168, -25}}, color = {0, 0, 127}));
  connect(limiter3.y, division1.u1) annotation(
    Line(points = {{61, 175}, {70, 175}, {70, 85}, {78, 85}}, color = {0, 0, 127}));
  connect(limiter1.y, iqInjectionLogic.iqV) annotation(
    Line(points = {{291, 30}, {311, 30}, {311, 22}}, color = {0, 0, 127}));
  connect(iqInjectionLogic.iqInjPu, add1.u1) annotation(
    Line(points = {{311, 2},  {311, -15}, {318, -15}}, color = {0, 0, 127}));
  connect(VRefConst1.y, add2.u1) annotation(
    Line(points = {{-69, -37}, {-60, -37}, {-60, -45}, {-51, -45}}, color = {0, 0, 127}));
  connect(switch1.y, add2.u2) annotation(
    Line(points = {{-109, 20}, {-100, 20}, {-100, -57}, {-51, -57}}, color = {0, 0, 127}));
  connect(add2.y, switch.u3) annotation(
    Line(points = {{-28, -51}, {-2, -51}, {-2, -27}}, color = {0, 0, 127}));
  connect(FRTOn4.y, currentLimitsCalculation1.vDip) annotation(
    Line(points = {{406, 27}, {421, 27}}, color = {255, 0, 255}));
  connect(FRTOn5.y, iqInjectionLogic.vDip) annotation(
    Line(points = {{303, 60}, {319, 60}, {319, 22}}, color = {255, 0, 255}));
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
  connect(PFlagConst.y, pflagswitch.u2) annotation(
    Line(points = {{-182, 174}, {-148, 174}, {-148, 175}}, color = {255, 0, 255}));
  connect(currentLimitsCalculation1.ipMaxPu, variableLimiter1.limit1) annotation(
    Line(points = {{443, 34}, {451, 34}, {451, 50}, {420, 50}, {420, 88}, {445, 88}, {445, 87}}, color = {0, 0, 127}));
  connect(currentLimitsCalculation1.ipMinPu, variableLimiter1.limit2) annotation(
    Line(points = {{443, 30}, {459, 30}, {459, 60}, {430, 60}, {430, 71}, {445, 71}}, color = {0, 0, 127}));
  connect(currentLimitsCalculation1.iqMaxPu, variableLimiter.limit1) annotation(
    Line(points = {{443, 24}, {460, 24}, {460, 0}, {430, 0}, {430, -13}, {445, -13}}, color = {0, 0, 127}));
  connect(currentLimitsCalculation1.iqMinPu, variableLimiter.limit2) annotation(
    Line(points = {{443, 20}, {451, 20}, {451, 10}, {420, 10}, {420, -29}, {445, -29}}, color = {0, 0, 127}));
  connect(UFilteredPu5.y, IpmaxFromUPu.u) annotation(
    Line(points = {{351, 32}, {354, 32}, {354, 44}, {358, 44}}, color = {0, 0, 127}));
  connect(UFilteredPu5.y, IqmaxFromUPu.u) annotation(
    Line(points = {{351, 32}, {354, 32}, {354, 16}, {358, 16}}, color = {0, 0, 127}));
  connect(IqmaxFromUPu.y[1], currentLimitsCalculation1.iqVdlPu) annotation(
    Line(points = {{381, 16}, {410, 16}, {410, 24}, {421, 24}}, color = {0, 0, 127}));
  connect(IpmaxFromUPu.y[1], currentLimitsCalculation1.ipVdlPu) annotation(
    Line(points = {{381, 44}, {409, 44}, {409, 31}, {421, 31}}, color = {0, 0, 127}));
  connect(add1.y, currentLimitsCalculation1.iqCmdPu) annotation(
    Line(points = {{341, -21}, {414, -21}, {414, 20}, {421, 20}}, color = {0, 0, 127}));
  connect(division1.y, currentLimitsCalculation1.ipCmdPu) annotation(
    Line(points = {{101, 79}, {414, 79}, {414, 35}, {421, 35}}, color = {0, 0, 127}));
  connect(DPMax0.y, rateLimFirstOrderFreeze.dyMax) annotation(
    Line(points = {{-29, 202}, {-2, 202}, {-2, 182}}, color = {0, 0, 127}));
  connect(DPMin0.y, rateLimFirstOrderFreeze.dyMin) annotation(
    Line(points = {{-29, 158}, {-2, 158}, {-2, 169}}, color = {0, 0, 127}));
  annotation(preferredView = "diagram",
    Documentation(info = "<html><head></head><body><p style=\"font-size: 12px;\">This block contains the electrical inverter control of the generic WECC Wind model according to (in case page cannot be found, copy link in browser):<br><a href=\"https://www.wecc.org/Reliability/WECC%20Wind%20Plant%20Dynamic%20Modeling%20Guidelines.pdf\">https://www.wecc.org/Reliability/WECC%20Wind%20Plant%20Dynamic%20Modeling%20Guidelines.pdf</a></p>
  <p style=\"font-size: 12px;\">
  Following control modes can be activated:</p>
  <li style=\"font-size: 12px;\">Local coordinated V/Q control: QFlag = true, VFlag = true</li>
  <li style=\"font-size: 12px;\">Only plant level control active: QFlag = false, VFlag = false</li>
  <li style=\"font-size: 12px;\">If plant level control not connected: local powerfactor control: PfFlag = true, otherwise PfFlag = false.</li>
  <li style=\"font-size: 12px;\"> Active power can be dependent or independent on drive train speed by setting PFlag to false (independent from drive train speed) or true. If PFlag is set to false, the model behaves as a Wind turbine generator type 4B, where the drive train is neglected by setting the speed to constant 1 </li>
    <p style=\"font-size: 12px;\">The block calculates the Id and Iq setpoint values for the generator control based on the selected control algorithm.</p></body></html>"),
    Diagram(coordinateSystem(extent = {{-260, -130}, {540, 250}}, grid = {1, 1})),
  Icon(graphics = {Rectangle(extent = {{-100, 100}, {100, -100}}), Text(origin = {-30.3275, -106.118}, extent = {{-14.6724, 6.11766}, {22.3276, -9.88234}}, textString = "omegaG"), Text(origin = {139, -41}, extent = {{-23, 13}, {35, -21}}, textString = "iqCmdPu"), Text(origin = {141, 13}, extent = {{-23, 13}, {17, -11}}, textString = "frtOn"), Text(origin = {89, -113}, extent = {{-23, 13}, {9, -3}}, textString = "UPu"), Text(origin = {41, -117}, extent = {{-33, 21}, {9, -3}}, textString = "PInjPu"), Text(origin = {-135, 79}, extent = {{-23, 13}, {35, -21}}, textString = "PInjRefPu"), Text(origin = {-135, -41}, extent = {{-23, 13}, {35, -21}}, textString = "QInjRefPu"), Text(origin = {-135, 21}, extent = {{-23, 13}, {35, -21}}, textString = "UFilteredPu")}, coordinateSystem(extent = {{-100, -100}, {100, 100}}, grid = {1, 1})));

end ElectricalControlWind;

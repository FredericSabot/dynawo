within Dynawo.Electrical.Sources;

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

model WT4AIECelecAux "Converter Model and grid interface according to IEC 61400-27-1 standard for a wind turbine of type 4A"

  /*
  Equivalent circuit and conventions:
  */

  import Modelica;
  import Modelica.Math;
  import Modelica.ComplexMath;
  import Dynawo;
  import Dynawo.Types;
  import Dynawo.Connectors;
  import Dynawo.Electrical.SystemBase;
  import Dynawo.Electrical.Controls.Basics.SwitchOff;

  extends SwitchOff.SwitchOffGenerator;

  /*Constructive parameters*/
  parameter Types.ApparentPowerModule SNom "Nominal converter apparent power in MVA";
  parameter Types.PerUnit Res "Electrical system serial resistance in p.u (base UNom, SNom)" annotation(
  Dialog(group = "group", tab = "Electrical"));
  parameter Types.PerUnit Xes "Electrical system serial reactance in p.u (base UNom, SNom)" annotation(
  Dialog(group = "group", tab = "Electrical"));
  parameter Types.PerUnit Ges "Electrical system shunt conductance in p.u (base UNom, SNom)" annotation(
  Dialog(group = "group", tab = "Electrical"));
  parameter Types.PerUnit Bes "Electrical system shunt susceptance in p.u (base UNom, SNom)"annotation(
  Dialog(group = "group", tab = "Electrical"));

  /*Control parameters*/
  parameter Types.Time Tg "Current generation time constant in seconds" annotation(
  Dialog(group = "group", tab = "Generator"));
  parameter Types.PerUnit DipMax "Maximun active current ramp rate in p.u (base UNom, SNom)" annotation(
    Dialog(group = "group", tab = "Generator"));
  parameter Types.PerUnit DiqMax "Maximun reactive current ramp rate in p.u (base UNom, SNom)" annotation(
    Dialog(group = "group", tab = "Generator"));
  parameter Types.PerUnit DiqMin "Minimum reactive current ramp rate in p.u (base UNom, SNom)" annotation(
    Dialog(group = "group", tab = "Generator"));

  /*Parameters for initialization from load flow*/
  parameter Types.VoltageModulePu U0Pu "Start value of voltage amplitude at plant terminal (PCC) in p.u (base UNom)" annotation(
  Dialog(group = "group", tab = "Operating point"));
  parameter Types.Angle UPhase0 "Start value of voltage angle at plan terminal (PCC) in radians" annotation(
  Dialog(group = "group", tab = "Operating point"));
  parameter Types.ActivePowerPu P0Pu "Start value of active power at PCC in p.u (base SnRef) (receptor convention)" annotation(
  Dialog(group = "group", tab = "Operating point"));
  parameter Types.ReactivePowerPu Q0Pu "Start value of reactive power at PCC in p.u (base SnRef) (receptor convention)" annotation(
  Dialog(group = "group", tab = "Operating point"));

  /*Parameters for internal initialization*/
  parameter Types.PerUnit IpMax0Pu "Start value of the maximum active current in p.u (base UNom, SNom) (generator convention)" annotation(
  Dialog(group = "group", tab = "Operating point"));
  parameter Types.PerUnit IqMax0Pu "Start value of the maximum reactive current in p.u (base UNom, SNom) (generator convention)" annotation(
  Dialog(group = "group", tab = "Operating point"));
  parameter Types.PerUnit IqMin0Pu "Start value of the minimum reactive current in p.u (base UNom, SNom) (generator convention)" annotation(
  Dialog(group = "group", tab = "Operating point"));
//  final parameter Types.ComplexPerUnit u0Pu = ComplexMath.fromPolar(U0Pu, UPhase0) "Start value of the complex voltage at plant terminal (PCC) in p.u (base UNom)";
//  final parameter Types.ComplexPerUnit i0Pu = ComplexMath.conj(Complex(P0Pu, Q0Pu) / u0Pu) "Start value of the complex current at plant terminal (PCC) in p.u (base UNom, SnRef) (receptor convention)";
  parameter Types.ComplexPerUnit u0Pu "Start value of the complex voltage at plant terminal (PCC) in p.u (base UNom)";
  parameter Types.ComplexPerUnit i0Pu "Start value of the complex current at plant terminal (PCC) in p.u (base UNom, SnRef) (receptor convention)";
  parameter Types.PerUnit IGsRe0Pu "Start value of the real component of the current at the converter's terminals (generator system) in p.u (Ubase, SNom) (generator convention)";
  parameter Types.PerUnit IGsIm0Pu "Start value of the imaginary component of the current at the converter's terminals (generator system) in p.u (Ubase, SNom) (generator convention)";
  parameter Types.PerUnit UGsRe0Pu "Start value of the real component of the voltage at the converter's terminals (generator system) in p.u (base UNom)";
  parameter Types.PerUnit UGsIm0Pu "Start value of the imaginary component of the voltage at the converter's terminals (generator system) in p.u (base UNom)";

  /*Inputs*/
  Modelica.Blocks.Interfaces.RealInput ipCmdPu(start = 0) "d-axis reference current at the generator system module (converter) terminal in p.u (Ubase,SNom) (generator convention)" annotation(
    Placement(visible = true, transformation(origin = {110, 15}, extent = {{10, -10}, {-10, 10}}, rotation = 0), iconTransformation(origin = {110, -40}, extent = {{-10, -10}, {10, 10}}, rotation = 180)));
  Modelica.Blocks.Interfaces.RealInput iqCmdPu(start = Q0Pu*SystemBase.SnRef / (SNom*U0Pu)) "q-axis reference current at the generator system module (converter) terminal in p.u (Ubase,SNom) (generator convention)" annotation(
    Placement(visible = true, transformation(origin = {110, -5}, extent = {{10, -10}, {-10, 10}}, rotation = 0), iconTransformation(origin = {110, -80}, extent = {{-10, -10}, {10, 10}}, rotation = 180)));
  Modelica.Blocks.Interfaces.RealInput ipMaxPu(start = 0) "Maximal d-axis reference current at the generator system module (converter) terminal in p.u (Ubase,SNom) (generator convention)" annotation(
    Placement(visible = true, transformation(origin = {110, 75}, extent = {{10, -10}, {-10, 10}}, rotation = 0), iconTransformation(origin = {110, 80}, extent = {{-10, -10}, {10, 10}}, rotation = 180)));
  Modelica.Blocks.Interfaces.RealInput iqMaxPu(start = IqMax0Pu) "Maximal q-axis reference current at the generator system module (converter) terminal in p.u (Ubase,SNom) (generator convention)" annotation(
    Placement(visible = true, transformation(origin = {110, 55}, extent = {{10, -10}, {-10, 10}}, rotation = 0), iconTransformation(origin = {110, 40}, extent = {{-10, -10}, {10, 10}}, rotation = 180)));
  Modelica.Blocks.Interfaces.RealInput iqMinPu(start = IqMin0Pu) "Minimal q-axis reference current at the generator system module (converter) terminal in p.u (Ubase,SNom) (generator convention)" annotation(
    Placement(visible = true, transformation(origin = {110, 35}, extent = {{10, -10}, {-10, 10}}, rotation = 0), iconTransformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 180)));
  Modelica.Blocks.Interfaces.RealInput theta(start = UPhase0) "Phase shift of the converter's rotating frame with respect to the grid rotating frame in radians" annotation(
    Placement(visible = true, transformation(origin = {110, 95}, extent = {{10, -10}, {-10, 10}}, rotation = 0), iconTransformation(origin = {0, 110}, extent = {{10, -10}, {-10, 10}}, rotation = 90)));

  /*Outputs*/
  Modelica.Blocks.Interfaces.RealOutput uWtRePu(start = u0Pu.re) "Real component of the voltage at the wind turbine (electrical system) terminals in p.u (base UNom)" annotation(
    Placement(visible = true, transformation(origin = {-110, -71}, extent = {{-10, -10}, {10, 10}}, rotation = 180), iconTransformation(origin = {40, -110}, extent = {{10, -10}, {-10, 10}}, rotation = 90)));
  Modelica.Blocks.Interfaces.RealOutput uWtImPu(start = u0Pu.im) "Imaginary component of the voltage at the wind turbine (electrical system) terminals in p.u (base UNom) " annotation(
    Placement(visible = true, transformation(origin = {-110, -90}, extent = {{-10, -10}, {10, 10}}, rotation = 180), iconTransformation(origin = {80, -110}, extent = {{10, -10}, {-10, 10}}, rotation = 90)));
  Modelica.Blocks.Interfaces.RealOutput iWtRePu(start = 0) "Real component of the current at the wind turbine terminals in p.u (Ubase,SNom) (generator convention)" annotation(
    Placement(visible = true, transformation(origin = {-110, -30}, extent = {{-10, -10}, {10, 10}}, rotation = 180), iconTransformation(origin = {-80, -110}, extent = {{10, -10}, {-10, 10}}, rotation = 90)));
  Modelica.Blocks.Interfaces.RealOutput iWtImPu(start = -i0Pu.im * SystemBase.SnRef / SNom) "Imaginary component of the current at the wind turbine terminals n p.u (Ubase,SNom) (generator convention)" annotation(
    Placement(visible = true, transformation(origin = {-110, -51}, extent = {{-10, -10}, {10, 10}}, rotation = 180), iconTransformation(origin = {-40, -110}, extent = {{10, 10}, {-10, -10}}, rotation = 90)));

  /*Blocks*/
  Dynawo.Connectors.ACPower terminal(V(re(start = u0Pu.re), im(start = u0Pu.im)), i(re(start = 0), im(start = i0Pu.im)))  annotation(
    Placement(visible = true, transformation(origin = {-90, 0}, extent = {{-5, -5}, {5, 5}}, rotation = 0), iconTransformation(origin = {-110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));

  Dynawo.Electrical.Sources.BaseConverters.IECElecSystemAux iECElecSystemAux(Bes = Bes, Ges = Ges, IGsIm0Pu = IGsIm0Pu, IGsRe0Pu = IGsRe0Pu, P0Pu = P0Pu, Q0Pu = Q0Pu, Res = Res, SNom = SNom, U0Pu = U0Pu, UGsIm0Pu = UGsIm0Pu, UGsRe0Pu = UGsRe0Pu, UPhase0 = UPhase0, Xes = Xes, i0Pu = i0Pu, u0Pu = u0Pu) annotation(
    Placement(visible = true, transformation(origin = {-40, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));

  Dynawo.Electrical.Sources.BaseConverters.IECGenSystemAux iECGenSystemAux(Bes = Bes,DipMax = DipMax, DiqMax = DiqMax, DiqMin = DiqMin, Ges = Ges, IGsIm0Pu = IGsIm0Pu, IGsRe0Pu = IGsRe0Pu, IpMax0Pu = IpMax0Pu, IqMax0Pu = IqMax0Pu, IqMin0Pu = IqMin0Pu, P0Pu = P0Pu, Q0Pu = Q0Pu, SNom = SNom, Tg = Tg, U0Pu = U0Pu, UPhase0 = UPhase0, i0Pu = i0Pu, u0Pu = u0Pu) annotation(
    Placement(visible = true, transformation(origin = {40, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));

equation

  running.value = iECElecSystemAux.running;

  /*Connectors*/
  connect(terminal, iECElecSystemAux.terminal) annotation(
    Line(points = {{-90, 0}, {-61, 0}}));
  connect(iECElecSystemAux.iGsRePu, iECGenSystemAux.iGsRePu) annotation(
    Line(points = {{-18, 10}, {18, 10}}, color = {0, 0, 127}));
  connect(iECElecSystemAux.iGsImPu, iECGenSystemAux.iGsImPu) annotation(
    Line(points = {{-18, -10}, {18, -10}}, color = {0, 0, 127}));
  connect(iECGenSystemAux.iqCmdPu, iqCmdPu) annotation(
    Line(points = {{62, -6}, {102, -6}, {102, -4}, {110, -4}}, color = {0, 0, 127}));
  connect(uWtImPu, iECElecSystemAux.uWtImPu) annotation(
    Line(points = {{-110, -90}, {-36, -90}, {-36, -22}}, color = {0, 0, 127}));
  connect(uWtRePu, iECElecSystemAux.uWtRePu) annotation(
    Line(points = {{-110, -71}, {-44, -71}, {-44, -22}}, color = {0, 0, 127}));
  connect(iWtRePu, iECElecSystemAux.iWtRePu) annotation(
    Line(points = {{-110, -30}, {-58, -30}, {-58, -22}}, color = {0, 0, 127}));
  connect(iECGenSystemAux.ipCmdPu, ipCmdPu) annotation(
    Line(points = {{62, 0}, {92, 0}, {92, 14}, {110, 14}, {110, 16}}, color = {0, 0, 127}));
  connect(iECGenSystemAux.iqMinPu, iqMinPu) annotation(
    Line(points = {{62, 6}, {86, 6}, {86, 34}, {110, 34}, {110, 36}}, color = {0, 0, 127}));
  connect(iECGenSystemAux.iqMaxPu, iqMaxPu) annotation(
    Line(points = {{62, 12}, {78, 12}, {78, 56}, {110, 56}, {110, 56}}, color = {0, 0, 127}));
  connect(iECGenSystemAux.ipMaxPu, ipMaxPu) annotation(
    Line(points = {{62, 18}, {70, 18}, {70, 74}, {110, 74}, {110, 76}}, color = {0, 0, 127}));
  connect(iWtImPu, iECElecSystemAux.iWtImPu) annotation(
    Line(points = {{-110, -51}, {-51, -51}, {-51, -22}}, color = {0, 0, 127}));
  connect(theta, iECGenSystemAux.theta) annotation(
    Line(points = {{110, 96}, {40, 96}, {40, 24}, {40, 24}, {40, 22}}, color = {0, 0, 127}));

annotation(
    Icon(graphics = {Rectangle(fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, extent = {{-100, 100}, {100, -100}}), Text(origin = {0, 30}, extent = {{-90, -30}, {90, 30}}, textString = "IEC AUX"), Text(origin = {0, -30}, extent = {{-90, -30}, {90, 30}}, textString = "Converter")}, coordinateSystem(initialScale = 0.1)));

end WT4AIECelecAux;

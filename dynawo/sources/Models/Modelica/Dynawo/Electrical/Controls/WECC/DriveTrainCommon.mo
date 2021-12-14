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

partial model DriveTrainCommon
  import Modelica;
  import Dynawo.Types;

  extends Parameters.Params_DriveTrainCommon;

  Modelica.Blocks.Interfaces.RealInput Pe(start = PInj0Pu) annotation(
    Placement(visible = true, transformation(origin = {-214, -54}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-206, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealInput omegaRefPu(start = SystemBase.omegaRef0Pu) annotation(
    Placement(visible = true, transformation(origin = {-214, -110}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-26, -144}, extent = {{-20, -20}, {20, 20}}, rotation = 90)));
  Modelica.Blocks.Interfaces.RealOutput omegaGPu(start = SystemBase.omegaRef0Pu) annotation(
    Placement(visible = true, transformation(origin = {166, -120}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {37, 131}, extent = {{19, -19}, {-19, 19}}, rotation = -90)));
  Modelica.Blocks.Interfaces.RealOutput omegaTPu annotation(
    Placement(visible = true, transformation(origin = {164, 110}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {-99, 133}, extent = {{-19, -19}, {19, 19}}, rotation = 90)));
  Modelica.Blocks.Interfaces.RealOutput dTorqueY annotation(
    Placement(visible = true, transformation(origin = {164, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {-99, 123}, extent = {{-0, -0}, {0,0}}, rotation = 90)));
    Modelica.Blocks.Interfaces.RealOutput dampingY annotation(
    Placement(visible = true, transformation(origin = {92, -36}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {-99, 123}, extent = {{0, 0}, {0,0}}, rotation = 90)));

  Modelica.Blocks.Math.Add OmegaGenerator annotation(
    Placement(visible = true, transformation(origin = {74, -66}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Math.Add OmegaTurbine annotation(
    Placement(visible = true, transformation(origin = {70, 50}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Math.Division TorqueE annotation(
    Placement(visible = true, transformation(origin = {-140, -60}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Math.Division TorqueM annotation(
    Placement(visible = true, transformation(origin = {-140, 56}, extent = {{-10, 10}, {10, -10}}, rotation = 0)));
  Modelica.Blocks.Sources.RealExpression omegaRefPu1(y = omegaRefPu)  annotation(
    Placement(visible = true, transformation(origin = {39, -72}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Sources.RealExpression omegaRefPu2(y = omegaRefPu)  annotation(
    Placement(visible = true, transformation(origin = {31, 43}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Sources.RealExpression dTorqueY1(y = dTorqueY)  annotation(
    Placement(visible = true, transformation(origin = {-118, 64}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Sources.RealExpression dTorqueY2(y = dTorqueY)  annotation(
    Placement(visible = true, transformation(origin = {-118, -68}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Sources.RealExpression dampingY1(y = dampingY)  annotation(
    Placement(visible = true, transformation(origin = {-118, -52}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Sources.RealExpression dampingY2(y = dampingY)  annotation(
    Placement(visible = true, transformation(origin = {-118, 48}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Continuous.Integrator dPhi(initType = Modelica.Blocks.Types.Init.InitialState, k = 1, y(start = PInj0Pu / Kshaft), y_start = PInj0Pu / Kshaft) annotation(
    Placement(visible = true, transformation(origin = {70, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Continuous.Integrator dOmegaTurbine(initType = Modelica.Blocks.Types.Init.InitialState, k = 1 / (2 * Ht), y_start = 0) annotation(
    Placement(visible = true, transformation(origin = {-30, 56}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Continuous.Integrator dOmegaGenerator(initType = Modelica.Blocks.Types.Init.InitialState, k = 1 / (2 * Hg), y_start = 0) annotation(
    Placement(visible = true, transformation(origin = {-30, -60}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Math.Add OmegaDiff(k2 = -1) annotation(
    Placement(visible = true, transformation(origin = {30, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Math.Add3 dTorqueE(k2 = -1) annotation(
    Placement(visible = true, transformation(origin = {-82, -60}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Math.Add3 dTorqueM(k1 = -1, k3 = -1) annotation(
    Placement(visible = true, transformation(origin = {-82, 56}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Math.Gain Damping(k = Dshaft) annotation(
    Placement(visible = true, transformation(origin = {60, -36}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Math.Gain dTorque(k = Kshaft) annotation(
    Placement(visible = true, transformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));


protected

  parameter Types.PerUnit PInj0Pu "Initial value mechanical power in pu";
equation
  connect(Pe, TorqueE.u1) annotation(
    Line(points = {{-214, -54}, {-152, -54}}, color = {0, 0, 127}));
  connect(OmegaDiff.y, Damping.u) annotation(
    Line(points = {{42, 0}, {42, -18}, {42, -36}, {48, -36}}, color = {0, 0, 127}));
  connect(OmegaGenerator.y, TorqueE.u2) annotation(
    Line(points = {{85, -66}, {108, -66}, {108, -120}, {-168, -120}, {-168, -66}, {-152, -66}}, color = {0, 0, 127}));
  connect(OmegaTurbine.y, TorqueM.u2) annotation(
    Line(points = {{81, 50}, {110, 50}, {110, 110}, {-172, 110}, {-172, 63}, {-152, 63}, {-152, 62}}, color = {0, 0, 127}));
  connect(dOmegaGenerator.y, OmegaGenerator.u1) annotation(
    Line(points = {{-19, -60}, {62, -60}}, color = {0, 0, 127}));
  connect(dOmegaTurbine.y, OmegaTurbine.u1) annotation(
    Line(points = {{-19, 56}, {58, 56}}, color = {0, 0, 127}));
  connect(dOmegaGenerator.y, OmegaDiff.u2) annotation(
    Line(points = {{-19, -60}, {8, -60}, {8, -6}, {18, -6}}, color = {0, 0, 127}));
  connect(dOmegaTurbine.y, OmegaDiff.u1) annotation(
    Line(points = {{-19, 56}, {8, 56}, {8, 6}, {18, 6}}, color = {0, 0, 127}));
  connect(OmegaDiff.y, dPhi.u) annotation(
    Line(points = {{42, 0}, {58, 0}}, color = {0, 0, 127}));
  connect(TorqueE.y, dTorqueE.u2) annotation(
    Line(points = {{-129, -60}, {-94, -60}}, color = {0, 0, 127}));
  connect(TorqueM.y, dTorqueM.u2) annotation(
    Line(points = {{-129, 56}, {-94, 56}}, color = {0, 0, 127}));
  connect(OmegaGenerator.y, omegaGPu) annotation(
    Line(points = {{86, -66}, {108, -66}, {108, -120}, {166, -120}}, color = {0, 0, 127}));
  connect(dPhi.y, dTorque.u) annotation(
    Line(points = {{81, 0}, {98, 0}}, color = {0, 0, 127}));
  connect(dTorqueM.y, dOmegaTurbine.u) annotation(
    Line(points = {{-70, 56}, {-42, 56}}, color = {0, 0, 127}));
  connect(dTorqueE.y, dOmegaGenerator.u) annotation(
    Line(points = {{-70, -60}, {-42, -60}}, color = {0, 0, 127}));
  connect(OmegaTurbine.y, omegaTPu) annotation(
    Line(points = {{82, 50}, {110, 50}, {110, 110}, {154, 110}, {154, 110}, {164, 110}}, color = {0, 0, 127}));
  connect(omegaRefPu1.y, OmegaGenerator.u2) annotation(
    Line(points = {{50, -73}, {57, -73}, {57, -72}, {62, -72}}, color = {0, 0, 127}));
  connect(omegaRefPu2.y, OmegaTurbine.u2) annotation(
    Line(points = {{42, 44}, {58, 44}}, color = {0, 0, 127}));
  connect(dTorque.y, dTorqueY) annotation(
    Line(points = {{122, 0}, {164, 0}}, color = {0, 0, 127}));
  connect(dTorqueY1.y, dTorqueM.u1) annotation(
    Line(points = {{-107, 64}, {-94, 64}}, color = {0, 0, 127}));
  connect(dTorqueY2.y, dTorqueE.u3) annotation(
    Line(points = {{-106, -68}, {-94, -68}}, color = {0, 0, 127}));
  connect(dampingY2.y, dTorqueM.u3) annotation(
    Line(points = {{-107, 48}, {-94, 48}}, color = {0, 0, 127}));
  connect(dampingY1.y, dTorqueE.u1) annotation(
    Line(points = {{-107, -52}, {-94, -52}}, color = {0, 0, 127}));
  connect(Damping.y, dampingY) annotation(
    Line(points = {{71, -36}, {92, -36}}, color = {0, 0, 127}));
  annotation(
    uses(Modelica(version = "3.2.3")),
    Diagram(coordinateSystem(extent = {{-200, -140}, {160, 120}})),
    Icon(coordinateSystem(extent = {{-200, -140}, {160, 120}}, initialScale = 0.1), graphics = {Rectangle(origin = {-20, -10}, extent = {{-180, 130}, {180, -130}}), Text(origin = {-13, -3}, extent = {{-147, 103}, {133, -99}}, textString = "WTGT")}),
    version = "");

end DriveTrainCommon;

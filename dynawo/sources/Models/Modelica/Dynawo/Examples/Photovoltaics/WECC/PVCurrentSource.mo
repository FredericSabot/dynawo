within Dynawo.Examples.Photovoltaics.WECC;

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

model PVCurrentSource "WECC PV Model on infinite bus"
  import Modelica;
  import Dynawo;

  extends Icons.Example;

  Dynawo.Electrical.Buses.InfiniteBusWithVariations infiniteBus(U0Pu = 1, UEvtPu = 0.5, UPhase = 0, omega0Pu = 1, omegaEvtPu = 1.01, tOmegaEvtEnd = 6.5, tOmegaEvtStart = 6, tUEvtEnd = 2, tUEvtStart = 1) annotation(
    Placement(visible = true, transformation(origin = {-82, 0}, extent = {{-20, -20}, {20, 20}}, rotation = -90)));
  Dynawo.Electrical.Lines.Line line(RPu = 0, XPu = 0.0000020661, BPu = 0, GPu = 0) annotation(
    Placement(visible = true, transformation(origin = {-40, -1.77636e-15}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
  Dynawo.Electrical.Photovoltaics.WECC.PVCurrentSource PV(DDn = 20, DUp = 0.001, FreqFlag = true, Id0Pu = 0.67611, IMaxPu = 1.05, Iq0Pu = 0.26996, Iqh1Pu = 2, Iql1Pu = -2, IqrMaxPu = 20, IqrMinPu = -20, Kc = 0, Ki = 1.5, KiPLL = 20, Kig = 2.36, Kp = 0.1, KpPLL = 3, Kpg = 0.05, Kqi = 0.5, Kqp = 1, Kqv = 2, Kvi = 1, Kvp = 1, OmegaMaxPu = 1.5, OmegaMinPu = 0.5, P0Pu = -0.7, PF0 = 0.92871, PInj0Pu = 0.7, PMaxPu = 1, PMinPu = 0, PPriority = false, PfFlag = false, Q0Pu = -0.2, QFlag = true, QInj0Pu = 0.2795, QMaxPu = 0.4, QMinPu = -0.4, RPu = 0, RateFlag = false, RefFlag = true, SNom = 100, tFilterPC = 0.04, tFilterGC = 0.02, tFt = 1e-10, tFv = 0.1, tG = 0.02, Tiq = 0.02, tLag = 0.1, tP = 0.04, tPord = 0.02, tRv = 0.02, U0Pu = 1.0, UInj0Pu = 1.03534, UMaxPu = 1.1, UMinPu = 0.9, UPhase0 = 0.00000144621, UPhaseInj0 = 0.10159, VFlag = true, VCompFlag = false, VFrz = 0, VMaxPu = 1.1, VMinPu = 0.9, VRef0Pu = 1, XPu = 0.15, DPMax = 999, DPMin = -999, Dbd = 0.01, Dbd1 = -0.1, Dbd2 = 0.1, EMax = 999, EMin = -999, FDbd1 = 0.004, FDbd2 = 1, FEMax = 999, FEMin = -999, i0Pu = Complex(-0.7, 0.2), iInj0Pu = Complex(0.7, -0.2), Rrpwr = 10, s0Pu = Complex(-0.7, -0.2), u0Pu = Complex(1, 0), uInj0Pu = Complex(1.03, 0.105)) annotation(
    Placement(visible = true, transformation(origin = {20, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 180)));
  Modelica.Blocks.Sources.Constant PRefPu(k = 0.7) annotation(
    Placement(visible = true, transformation(origin = {80, -40}, extent = {{-10, -10}, {10, 10}}, rotation = 180)));
  Modelica.Blocks.Sources.Constant QRefPu(k = 0.2) annotation(
    Placement(visible = true, transformation(origin = {80, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 180)));
  Modelica.Blocks.Sources.Constant omegaRefPu(k = 1) annotation(
    Placement(visible = true, transformation(origin = {80, 40}, extent = {{-10, -10}, {10, 10}}, rotation = 180)));
  Modelica.Blocks.Sources.Constant URefPu(k = 1) annotation(
    Placement(visible = true, transformation(origin = {80, 80}, extent = {{-10, -10}, {10, 10}}, rotation = 180)));

equation
  line.switchOffSignal1.value = false;
  line.switchOffSignal2.value = false;
  PV.injector.switchOffSignal1.value = false;
  PV.injector.switchOffSignal2.value = false;
  PV.injector.switchOffSignal3.value = false;
  connect(line.terminal2, PV.terminal) annotation(
    Line(points = {{-20, 0}, {0, 0}, {0, 0}, {0, 0}}, color = {0, 0, 255}));
  connect(infiniteBus.terminal, line.terminal1) annotation(
    Line(points = {{-82, 0}, {-60, 0}, {-60, 0}, {-60, 0}}, color = {0, 0, 255}));
  connect(omegaRefPu.y, PV.omegaRefPu) annotation(
    Line(points = {{69, 40}, {54, 40}, {54, 12}, {42, 12}}, color = {0, 0, 127}));
  connect(QRefPu.y, PV.QRefPu) annotation(
    Line(points = {{70, 0}, {44, 0}, {44, 0}, {42, 0}}, color = {0, 0, 127}));
  connect(PRefPu.y, PV.PRefPu) annotation(
    Line(points = {{69, -40}, {54, -40}, {54, -12}, {42, -12}}, color = {0, 0, 127}));
  connect(URefPu.y, PV.URefPu) annotation(
    Line(points = {{70, 80}, {20, 80}, {20, 22}}, color = {0, 0, 127}));

  annotation(
    preferredView = "diagram",
    experiment(StartTime = 0, StopTime = 20, Tolerance = 1e-05, Interval = 0.001),
    Documentation(info = "<html><head></head><body><span style=\"font-size: 12px;\">
     This test case consists in one PV park connected to an infinite bus which voltage is reduced to 0.5 pu from t = 1 s to t = 2 s, and which frequency is increased to 1.01 pu from t = 6 s to t = 6.5 s. This is a way to observe the PV park's response to a voltage and frequency variation at its terminal.    </div>
    <div><br></div><div><br></div><div><br></div><div><br></div><div><br></div><div><span style=\"font-size: 12px;\"><br></span></div></div></body></html>
 "),
  __OpenModelica_commandLineOptions = "--matchingAlgorithm=PFPlusExt --indexReductionMethod=dynamicStateSelection -d=initialization,NLSanalyticJacobian,newInst",
  __OpenModelica_simulationFlags(lv = "LOG_STATS", s = "ida", maxIntegrationOrder = "2", nls = "kinsol", noHomotopyOnFirstTry = "()", noRestart = "()", noRootFinding = "()", initialStepSize = "0.00001", maxStepSize = "10"));
end PVCurrentSource;

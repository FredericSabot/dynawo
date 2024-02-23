within Dynawo.Electrical.Events;

/*
* Copyright (c) 2024, RTE (http://www.rte-france.com)
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

model NodeFaultContinuous "Node fault which lasts from tBegin to tEnd"
  /*
    Between tBegin and tEnd, the impedance between the node and the ground is equal to ZPu.
  */
  import Dynawo.NonElectrical.Logs.Timeline;
  import Dynawo.NonElectrical.Logs.TimelineKeys;

  Dynawo.Connectors.ACPower terminal annotation(
    Placement(visible = true, transformation(origin = {2, 2}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {0, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Dynawo.Connectors.BPin nodeFault(value(start = false)) "True when the fault is ongoing, false otherwise";

  parameter Types.PerUnit RPu "Fault resistance in pu (base SnRef)";
  parameter Types.PerUnit XPu "Fault reactance in pu (base SnRef)";
  parameter Types.Time tBegin "Time when the fault begins";
  parameter Types.Time tEnd "Time when the fault ends";
  parameter Types.Time tFilter = 1e-3 "Filter to smooth out the behaviour of the fault";

protected
  parameter Types.ComplexImpedancePu ZPu(re = RPu, im = XPu) "Impedance of the fault in pu (base SnRef)";

equation
  when time >= tEnd then
    Timeline.logEvent1(TimelineKeys.NodeFaultEnd);
    nodeFault.value = false;
  elsewhen time >= tBegin then
    Timeline.logEvent1(TimelineKeys.NodeFaultBegin);
    nodeFault.value = true;
  end when;

  if time > tEnd + tFilter or time < tBegin then
    terminal.i = Complex(0);
  elseif time >= tBegin and time < tBegin + tFilter then
    terminal.i = 1/ZPu * terminal.V * (time - tBegin) / tFilter;
  elseif time >= tBegin + tFilter and time < tEnd then
    terminal.i = 1/ZPu * terminal.V;
  else
    terminal.i = 1/ZPu * terminal.V * (1 - (time - tEnd) / tFilter);
  end if;

  annotation(preferredView = "text",
    Documentation(info = "<html><head></head><body>During the fault, the impedance between the node and the ground is defined by R and X values.</body></html>"),
  Icon(graphics = {Polygon(origin = {0, 55}, fillPattern = FillPattern.Solid, points = {{0, 45}, {-6, 5}, {20, 5}, {0, -45}, {6, -5}, {-20, -5}, {0, 45}, {0, 45}})}));
end NodeFaultContinuous;

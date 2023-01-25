within Dynawo.Electrical.Controls.LVRT;

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

model LVRT "Low voltage ride through"

  import Modelica.Constants;
  import Modelica;

  import Dynawo.Connectors;
  import Dynawo.NonElectrical.Logs.Timeline;
  import Dynawo.NonElectrical.Logs.TimelineKeys;


    parameter Types.VoltageModulePu URPu "Voltage threshold under which the automaton is activated after T2";
    parameter Types.VoltageModulePu UIntPu "Voltage threshold under which the automaton is activated after T1";
    parameter Types.VoltageModulePu UMinPu "Voltage threshold under which the automaton is activated instantaneously";
    parameter Types.Time T1 "Time delay of trip for severe voltage dips";
    parameter Types.Time T2 "Time delay of trip for small voltage dips";

    Connectors.BPin switchOffSignal (value (start = false)) "Switch off message for the generator";
    Modelica.Blocks.Interfaces.RealInput UMonitoredPu "Monitored voltage in pu (base UNom)" annotation(
    Placement(visible = true, transformation(origin = {-120, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-120, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));

  protected
    Types.Time tThresholdReached (start = Constants.inf) "Time when the threshold was reached";

  equation
    // Voltage comparison with the minimum accepted value
    when UMonitoredPu <= URPu and not(pre(switchOffSignal.value)) then
      tThresholdReached = time;
      Timeline.logEvent1(TimelineKeys.LVRTArming);
    elsewhen UMonitoredPu > URPu and pre(tThresholdReached) <> Constants.inf and not(pre(switchOffSignal.value)) then
      tThresholdReached = Constants.inf;
      Timeline.logEvent1(TimelineKeys.LVRTDisarming);
    end when;

    // Delay before tripping the generator
    when UMonitoredPu < UMinPu then // No delay
      switchOffSignal.value = true;
      Timeline.logEvent1(TimelineKeys.LVRTTripped);
    elsewhen time - tThresholdReached >= T1 and UMonitoredPu < UIntPu then
      switchOffSignal.value = true;
      Timeline.logEvent1(TimelineKeys.LVRTTripped);
    elsewhen UMonitoredPu >= UIntPu and  time - tThresholdReached >= T1 + (T2-T1) * (UMonitoredPu - UMinPu)/(URPu-UMinPu) then
      switchOffSignal.value = true;
      Timeline.logEvent1(TimelineKeys.LVRTTripped);
    end when;

annotation(preferredView = "text",
    Documentation(info = "<html><head></head><body>This model will send a tripping order to an (inverter-connected) generator if the voltage stays below a LVRT curve (see attached figure). See fig 7b in <a href=\"https://orbi.uliege.be/bitstream/2268/212146/1/Full_paper.pdf\">https://orbi.uliege.be/bitstream/2268/212146/1/Full_paper.pdf</a></body></html>"));
end LVRT;

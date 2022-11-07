within Dynawo.Electrical.Controls.Protections;

model DistanceProtectionLineSide2 "Simple triangular distance relay, disconnects side 2 of the protected line when the apparent impedance in within a protected zone for longer than a set duration"
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
  import Modelica.Constants;
  import Dynawo.Connectors;
  import Dynawo.NonElectrical.Logs.Timeline;
  import Dynawo.NonElectrical.Logs.TimelineKeys;

  import Dynawo.Electrical.Controls.Basics.SwitchOff;
  extends SwitchOff.SwitchOffProtection;

public
  parameter Types.Time T1 "Time delay of trip in zone 1";
  parameter Types.Time T2 "Time delay of trip in zone 2";
  parameter Types.Time T3 "Time delay of trip in zone 3";
  parameter Types.Time T4 "Time delay of trip in zone 4";

  parameter Real R1;
  parameter Real R2;
  parameter Real R3;
  parameter Real R4;
  parameter Real X1;
  parameter Real X2;
  parameter Real X3;
  parameter Real X4;

  Types.VoltageModulePu UMonitoredPu "Monitored voltage in pu (base UNom)";
  Types.ActivePowerPu PMonitoredPu "Monitored active power in pu (base SNom)";
  Types.ActivePowerPu PMonitoredPuFilter;
  Types.ReactivePowerPu QMonitoredPu "Monitored reactive power in pu (base SNom)";
  Types.ReactivePowerPu QMonitoredPuFilter;
  Complex Z "Apparent impedance in pu (base UNom, SNom)";

  Connectors.ZPin lineState(value(start = 2)) "Switch off message for the protected line";

  Types.Time tThresholdReached1(start = Constants.inf) "Time when enters zone 1";
  Types.Time tThresholdReached2(start = Constants.inf) "Time when enters zone 2";
  Types.Time tThresholdReached3(start = Constants.inf) "Time when enters zone 3";
  Types.Time tThresholdReached4(start = Constants.inf) "Time when enters zone 4";

equation
  if lineState.value == 2 or lineState.value == 4 and running.value then  // Can only read PMonitoredPu and QMonitoredPu if the line is connected on (at least) side2
    if time < 0.1 then  //TODO: clean initialisation
      PMonitoredPuFilter = PMonitoredPu;
      QMonitoredPuFilter = QMonitoredPu;
    else
      der(PMonitoredPuFilter) = 1e3 * (PMonitoredPu - PMonitoredPuFilter);
      der(QMonitoredPuFilter) = 1e3 * (QMonitoredPu - QMonitoredPuFilter);
    end if;
    if PMonitoredPuFilter^2 + QMonitoredPuFilter^2 > 1e-6 then
      Z = UMonitoredPu^2 / Complex(PMonitoredPuFilter, -QMonitoredPuFilter);
    else
      Z = Complex(999,999);
    end if;
  else
    PMonitoredPuFilter = 0;
    QMonitoredPuFilter = 0;
    Z = Complex(999, 999);
  end if;
  /*
  The when equations are not included in the "if lineState.value" condition because this is not
  supported by the Dynawo backend
  */
  // Impedance comparison with the zone 1
  when (Z.re <= R1 and Z.im <= X1 and Z.re > -Z.im)  then
    tThresholdReached1 = time;
    Timeline.logEvent1(TimelineKeys.Zone1Arming);
  elsewhen not (Z.re <= R1 and Z.im <= X1 and Z.re > -Z.im) and pre(tThresholdReached1) <> Constants.inf  then
    tThresholdReached1 = Constants.inf;
    Timeline.logEvent1(TimelineKeys.Zone1Disarming);
  end when;

  // Impedance comparison with the zone 2
  when (Z.re <= R2 and Z.im <= X2 and Z.re > -Z.im)  then
    tThresholdReached2 = time;
    Timeline.logEvent1(TimelineKeys.Zone2Arming);
  elsewhen not (Z.re <= R2 and Z.im <= X2 and Z.re > -Z.im) and pre(tThresholdReached2) <> Constants.inf  then
    tThresholdReached2 = Constants.inf;
    Timeline.logEvent1(TimelineKeys.Zone2Disarming);
  end when;

  // Impedance comparison with the zone 3
  when (Z.re <= R3 and Z.im <= X3 and Z.re > -Z.im)  then
    tThresholdReached3 = time;
    Timeline.logEvent1(TimelineKeys.Zone3Arming);
  elsewhen not (Z.re <= R3 and Z.im <= X3 and Z.re > -Z.im) and pre(tThresholdReached3) <> Constants.inf  then
    tThresholdReached3 = Constants.inf;
    Timeline.logEvent1(TimelineKeys.Zone3Disarming);
  end when;

  // Impedance comparison with the zone 4
  when (Z.re <= R4 and Z.im <= X4 and Z.re > -Z.im)  then
    tThresholdReached4 = time;
    Timeline.logEvent1(TimelineKeys.Zone4Arming);
  elsewhen not (Z.re <= R4 and Z.im <= X4 and Z.re > -Z.im) and pre(tThresholdReached4) <> Constants.inf  then
    tThresholdReached4 = Constants.inf;
    Timeline.logEvent1(TimelineKeys.Zone4Disarming);
  end when;

  // Trips
  /*
  Trips are not included in the if lineState.value condition to avoid the following Modelica error
  Following variable is discrete, but does not appear on the LHS of a when-statement: ‘lineState.value‘.
  */
  when time - tThresholdReached1 >= T1  then
    if lineState.value == 2 or lineState.value == 3 then
      lineState.value = 3;
    else
      lineState.value = 1;
    end if;
    Timeline.logEvent1(TimelineKeys.DistanceTrippedZone1);
  elsewhen time - tThresholdReached2 >= T2  then
    if lineState.value == 2 or lineState.value == 3 then
      lineState.value = 3;
    else
      lineState.value = 1;
    end if;
    Timeline.logEvent1(TimelineKeys.DistanceTrippedZone2);
  elsewhen time - tThresholdReached3 >= T3  then
    if lineState.value == 2 or lineState.value == 3 then
      lineState.value = 3;
    else
      lineState.value = 1;
    end if;
    Timeline.logEvent1(TimelineKeys.DistanceTrippedZone3);
  elsewhen time - tThresholdReached4 >= T4  then
    if lineState.value == 2 or lineState.value == 3 then
      lineState.value = 3;
    else
      lineState.value = 1;
    end if;
    Timeline.logEvent1(TimelineKeys.DistanceTrippedZone4);
  end when;

  annotation(
    preferredView = "text",
    Documentation(info = "<html><head></head><body> </body></html>"));
end DistanceProtectionLineSide2;

within Dynawo.Electrical.Controls.Protections;

model DistanceProtection "Simplified version of a quadrilateral distance relay"
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

  Types.Voltage UMonitored "Monitored voltage";
  Types.Current IMonitored "Monitored current";
  Complex Z "Apparent impedance";
  Boolean forward "True if Z is between -45° and 135° in the complex plane";

  Connectors.BPin switchOffSignal(value(start = false)) "Switch off message for the protected element";

  Types.Time tThresholdReached1(start = Constants.inf) "Time when enters zone 1";
  Types.Time tThresholdReached2(start = Constants.inf) "Time when enters zone 2";
  Types.Time tThresholdReached3(start = Constants.inf) "Time when enters zone 3";
  Types.Time tThresholdReached4(start = Constants.inf) "Time when enters zone 4";

equation
  Z = UMonitored / IMonitored;
  // forward = if ComplexMath.arg(Z) < (3/4 * Constants.pi) or ComplexMath.arg(Z) > (-1/4 * Constants.pi) then true else false;
  forward = if Z.re > -Z.im then true else false;

  // Impedance comparison with the zone 1
  when (Z.re <= R1 and Z.im <= X1 and forward) and not pre(switchOffSignal.value) then
    tThresholdReached1 = time;
    Timeline.logEvent1(TimelineKeys.Zone1Arming);
  elsewhen not (Z.re <= R1 and Z.im <= X1 and forward) and pre(tThresholdReached1) <> Constants.inf and not pre(switchOffSignal.value) then
    tThresholdReached1 = Constants.inf;
    Timeline.logEvent1(TimelineKeys.Zone1Disarming);
  end when;

  // Impedance comparison with the zone 2
  when (Z.re <= R2 and Z.im <= X2 and forward) and not pre(switchOffSignal.value) then
    tThresholdReached2 = time;
    Timeline.logEvent1(TimelineKeys.Zone2Arming);
  elsewhen not (Z.re <= R2 and Z.im <= X2 and forward) and pre(tThresholdReached2) <> Constants.inf and not pre(switchOffSignal.value) then
    tThresholdReached2 = Constants.inf;
    Timeline.logEvent1(TimelineKeys.Zone2Disarming);
  end when;

  // Impedance comparison with the zone 3
  when (Z.re <= R3 and Z.im <= X3 and forward) and not pre(switchOffSignal.value) then
    tThresholdReached3 = time;
    Timeline.logEvent1(TimelineKeys.Zone3Arming);
  elsewhen not (Z.re <= R3 and Z.im <= X3 and forward) and pre(tThresholdReached3) <> Constants.inf and not pre(switchOffSignal.value) then
    tThresholdReached3 = Constants.inf;
    Timeline.logEvent1(TimelineKeys.Zone3Disarming);
  end when;

  // Impedance comparison with the zone 4
  when (Z.re <= R4 and Z.im <= X4 and forward) and not pre(switchOffSignal.value) then
    tThresholdReached4 = time;
    Timeline.logEvent1(TimelineKeys.Zone4Arming);
  elsewhen not (Z.re <= R4 and Z.im <= X4 and forward) and pre(tThresholdReached4) <> Constants.inf and not pre(switchOffSignal.value) then
    tThresholdReached4 = Constants.inf;
    Timeline.logEvent1(TimelineKeys.Zone4Disarming);
  end when;


  // Trips
  when time - tThresholdReached1 >= T1 then
    switchOffSignal.value = true;
    Timeline.logEvent1(TimelineKeys.DistanceTrippedZone1);
  elsewhen time - tThresholdReached2 >= T2 then
    switchOffSignal.value = true;
    Timeline.logEvent1(TimelineKeys.DistanceTrippedZone2);
  elsewhen time - tThresholdReached3 >= T3 then
    switchOffSignal.value = true;
    Timeline.logEvent1(TimelineKeys.DistanceTrippedZone3);
  elsewhen time - tThresholdReached4 >= T4 then
    switchOffSignal.value = true;
    Timeline.logEvent1(TimelineKeys.DistanceTrippedZone4);
  end when;

  annotation(
    preferredView = "text",
    Documentation(info = "<html><head></head><body> </body></html>"));
end DistanceProtection;

within Dynawo.Electrical.Controls.Machines.Protections;

/*
* Copyright (c) 2022, RTE (http://www.rte-france.com)
* See AUTHORS.txt
* All rights reserved.
* This Source Code Form is subject to the terms of the Mozilla Public
* License, v. 2.0. If a copy of the MPL was not distributed with this
* file, you can obtain one at http://mozilla.org/MPL/2.0/.
* SPDX-License-Identifier: MPL-2.0
*
* This file is part of Dynawo, an hybrid C++/Modelica open source suite of simulation tools for power systems.
*/

model LossOfSynchronismProtection "Loss of synchronism protection for generators"
  /* Trips when the internal angle (variable thetaInternal) of a generator is negative and lower than -abs(ThetaMin). The protection never disarms because there is a risk to "loop around" (thetaInternal is always between -pi and pi).*/
  import Modelica.Constants;
  import Dynawo.NonElectrical.Logs.Timeline;
  import Dynawo.NonElectrical.Logs.TimelineKeys;

  parameter Types.Angle ThetaMin "Angle threshold below which the automaton is activated in rad. It is recommended to use a value not too close to pi or -pi as the simulator might skip it.";
  parameter Types.Time tLagAction "Time-lag due to the actual trip action in s";

  input Types.Angle thetaMonitored "Monitored angle in rad";

  Connectors.BPin switchOffSignal(value(start = false)) "Switch off message for the generator";

protected
  Types.Time tThresholdReached(start = Constants.inf) "Time when the threshold was reached in s";

equation
  // Angle comparison with the minimum accepted value
  when thetaMonitored < -abs(ThetaMin) and not(pre(switchOffSignal.value)) then
    tThresholdReached = time;
    Timeline.logEvent1(TimelineKeys.LossOfSynchronismArming);
  end when;

  // Never disarm because there is a risk to "loop around" (theta is always between -pi and pi)

  // Delay before tripping the generator
  when time - tThresholdReached >= tLagAction then
    switchOffSignal.value = true;
    Timeline.logEvent1(TimelineKeys.LossOfSynchronismTripped);
  end when;

  annotation(preferredView = "text",
    Documentation(info = "<html><head></head><body>This model will send a tripping order to a generator if its internal angle stays below a threshold during a certain amount of time.</body></html>"));
end LossOfSynchronismProtection;

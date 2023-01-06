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

model OutOfStep "Out of step protection"
  /* Trips when the angle of a generator is negative (i.e. higher than pi or actually negative)
  and theta is lower than -abs(thetaMinPu)
  The protection never disarms because there is a risk to "loop around" (theta is always between -pi and pi)*/
  import Modelica.Constants;
  import Dynawo.Connectors;
  import Dynawo.NonElectrical.Logs.Timeline;
  import Dynawo.NonElectrical.Logs.TimelineKeys;

  parameter Types.Angle thetaMinPu "Angle threshold above which the automaton is activated in pu (base omegaNom). It is recommended to use a value that is not to close to (-)pi as the simulator migth skip it.";
  parameter Types.Time tLagAction "Time-lag due to the actual trip action in s";

  Types.Angle thetaMonitoredPu "Monitored angle in pu (base omegaNom)";
  Connectors.BPin switchOffSignal(value(start = false)) "Switch off message for the generator";

protected
  Types.Time tThresholdReached(start = Constants.inf) "Time when the threshold was reached";

equation
  // Angle comparison with the minimum accepted value
  when thetaMonitoredPu < -abs(thetaMinPu) and not(pre(switchOffSignal.value)) then
    tThresholdReached = time;
    Timeline.logEvent1(TimelineKeys.OOSArming);
  end when;

  // Never disarm because there is a risk to "loop around" (theta is always between -pi and pi)

  // Delay before tripping the generator
  when time - tThresholdReached >= tLagAction then
    switchOffSignal.value = true;
    Timeline.logEvent1(TimelineKeys.OOSTripped);
  end when;

  annotation(preferredView = "text",
    Documentation(info = "<html><head></head><body>This model will send a tripping order to a generator if the voltage stays below a threshold during a certain amount of time.</body></html>"));
end OutOfStep;

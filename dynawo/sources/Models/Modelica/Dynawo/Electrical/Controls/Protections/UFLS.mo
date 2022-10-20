within Dynawo.Electrical.Controls.Protections;

model UFLS "Simplified version of a quadrilateral distance relay"
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
  parameter Types.Time tLagAction "Time-lag due to the actual trip action in s";

  parameter Real omega1Pu "Frequency of the 1st UFLS speed (in pu)";
  parameter Real omega2Pu "Frequency of the 2nd UFLS speed (in pu)";
  parameter Real omega3Pu "Frequency of the 3rd UFLS speed (in pu)";

  parameter Real UFLSStep1 "Share of load disconnected by step 1 of UFLS (sum of all steps should be at most 1)";
  parameter Real UFLSStep2 "Share of load disconnected by step 2 of UFLS";
  parameter Real UFLSStep3 "Share of load disconnected by step 3 of UFLS";

  Types.Frequency omegaMonitoredPu "Monitored frequency";

  Types.Time tThresholdReached1(start = Constants.inf) "Time when reaches 1st UFLS threshold";
  Types.Time tThresholdReached2(start = Constants.inf) "Time when reaches 2nd UFLS threshold";
  Types.Time tThresholdReached3(start = Constants.inf) "Time when reaches 3rd UFLS threshold";

  Boolean step1Activated (start = false) "True if step 1 of UFLS has been activated";
  Boolean step2Activated (start = false) "True if step 2 of UFLS has been activated";
  Boolean step3Activated (start = false) "True if step 3 of UFLS has been activated";

  Real deltaPQ (start = 0) "Reduction of P and Q due to UFLS disconnections, to connect to the variables deltaP and deltaQ in the load model";

equation
  // Frequency comparison with first UFLS step
  when omegaMonitoredPu <= omega1Pu and not pre(step1Activated) then
    tThresholdReached1 = time;
    Timeline.logEvent1(TimelineKeys.UFLS1Arming);
  end when;

  // Frequency comparison with second UFLS step
  when omegaMonitoredPu <= omega2Pu and not pre(step2Activated) then
    tThresholdReached2 = time;
    Timeline.logEvent1(TimelineKeys.UFLS2Arming);
  end when;

  // Frequency comparison with third UFLS step
  when omegaMonitoredPu <= omega3Pu and not pre(step3Activated) then
    tThresholdReached3 = time;
    Timeline.logEvent1(TimelineKeys.UFLS3Arming);
  end when;


  // Trips
  when time - tThresholdReached1 >= tLagAction then
    step1Activated = true;
    Timeline.logEvent1(TimelineKeys.UFLS1Activated);
  end when;
  when time - tThresholdReached2 >= tLagAction then
    step2Activated = true;
    Timeline.logEvent1(TimelineKeys.UFLS2Activated);
  end when;
  when time - tThresholdReached3 >= tLagAction then
    step3Activated = true;
    Timeline.logEvent1(TimelineKeys.UFLS3Activated);
  end when;

  if step3Activated then
    deltaPQ = - UFLSStep1 - UFLSStep2 - UFLSStep3;
  elseif step2Activated then
    deltaPQ = - UFLSStep1 - UFLSStep2;
  elseif step1Activated then
    deltaPQ = - UFLSStep1;
  else
    deltaPQ = 0;
  end if;

  annotation(
    preferredView = "text",
    Documentation(info = "<html><head></head><body> </body></html>"));
end UFLS;

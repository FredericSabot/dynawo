within Dynawo.Electrical.Controls.Protections;

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

package BaseClasses "Base models for protections"
  extends Icons.BasesPackage;

  partial model BaseDistanceProtectionLine "Base model for line distance protection"
    import Modelica;
    import Modelica.Constants;
    import Modelica.Blocks.Interfaces.RealInput;
    import Dynawo.NonElectrical.Logs.Timeline;
    import Dynawo.NonElectrical.Logs.TimelineKeys;

    parameter Integer NbZones(min = 1, max = 4) "Number of zones in the distance protection scheme";

    parameter Integer LineSide(min = 1, max = 2) "Side of the line where the distance protection is located";

    parameter Types.Time[NbZones] tZone "Time delay of trip in zone i (for each protected zone)";

    parameter Types.PerUnit[NbZones] RPu "Resistive reach of zone i in pu (base UNom, SnRef) (for each protected zone)";
    parameter Types.PerUnit[NbZones] XPu "Reactive reach of zone i in pu (base UNom, SnRef) (for each protected zone)";
    parameter Integer[NbZones] TrippingZone "Positive if zone i can send a tripping signal";

    parameter Types.Time CircuitBreakerTime "Time required to open the circuit breaker in s";

    parameter Boolean WithBlinder "True if a load blinder is used";
    parameter Types.Angle BlinderAnglePu "Load angle of the load blinder in rad";
    parameter Real BlinderReachPu "Reach in the Z plane of the load blinder in pu (base UNom, SnRef)";

    RealInput UMonitoredPu "Monitored voltage in pu (base UNom)";
    RealInput PMonitoredPu "Monitored active power in pu (base SnRef)";
    RealInput QMonitoredPu "Monitored reactive power in pu (base SnRef)";
    Boolean Blinded "True if the apparent impedance is in the load blinder area";

    Connectors.ZPin lineState(value(start = 2)) "Switch off message for the protected line";
    Integer[NbZones] Tripped(each start = 0) "1 if the protection tripped, else 0 (for each protected zone";

    Types.Time[NbZones] tThresholdReached(each start = Constants.inf) "Time when enters zone i in s (for each protected zone)";

  equation
    Blinded = if UMonitoredPu^2 / sqrt(PMonitoredPu^2 + QMonitoredPu^2) > BlinderReachPu and abs(Modelica.Math.atan2(QMonitoredPu, PMonitoredPu)) < BlinderAnglePu then true else false;

    // Check if apparent impedance is in a protected zone
    for i in 1:NbZones loop
      when (UMonitoredPu^2/(PMonitoredPu^2 + QMonitoredPu^2)*PMonitoredPu <= RPu[i] and
          UMonitoredPu^2/(PMonitoredPu^2 + QMonitoredPu^2)*QMonitoredPu <= XPu[i] and
          UMonitoredPu^2/(PMonitoredPu^2 + QMonitoredPu^2)*PMonitoredPu > -UMonitoredPu^2/(PMonitoredPu^2 + QMonitoredPu^2)*QMonitoredPu) and
          not Blinded and pre(tThresholdReached[i]) == Constants.inf then
        tThresholdReached[i] = time;

        if i == 1 then
          Timeline.logEvent1(TimelineKeys.Zone1Arming);
        elseif i == 2 then
          Timeline.logEvent1(TimelineKeys.Zone2Arming);
        elseif i == 3 then
          Timeline.logEvent1(TimelineKeys.Zone3Arming);
        else
          Timeline.logEvent1(TimelineKeys.Zone4Arming);
        end if;
      elsewhen not (UMonitoredPu^2/(PMonitoredPu^2 + QMonitoredPu^2)*PMonitoredPu <= RPu[i] and
          UMonitoredPu^2/(PMonitoredPu^2 + QMonitoredPu^2)*QMonitoredPu <= XPu[i] and
          UMonitoredPu^2/(PMonitoredPu^2 + QMonitoredPu^2)*PMonitoredPu > -UMonitoredPu^2/(PMonitoredPu^2 + QMonitoredPu^2)*QMonitoredPu and
          not Blinded) and
          pre(tThresholdReached[i]) <> Constants.inf and time - pre(tThresholdReached[i]) < tZone[i] then
        tThresholdReached[i] = Constants.inf;

        if i == 1 then
          Timeline.logEvent1(TimelineKeys.Zone1Disarming);
        elseif i == 2 then
          Timeline.logEvent1(TimelineKeys.Zone2Disarming);
        elseif i == 3 then
          Timeline.logEvent1(TimelineKeys.Zone3Disarming);
        else
          Timeline.logEvent1(TimelineKeys.Zone4Disarming);
        end if;
      end when;
    end for;

    // Trips
    /*
    Trips are not included in the if lineState.value condition to avoid the following Modelica error
    Following variable is discrete, but does not appear on the LHS of a when-statement: ‘lineState.value‘.
    */
    for i in 1:NbZones loop
      when time - tThresholdReached[i] >= tZone[i] + CircuitBreakerTime then
        if TrippingZone[i] > 0 then
          Tripped[i] = 1;
        else
          Tripped[i] = pre(Tripped[i]);
        end if;

        if i == 1 then
          Timeline.logEvent1(TimelineKeys.DistanceTrippedZone1);
        elseif i == 2 then
          Timeline.logEvent1(TimelineKeys.DistanceTrippedZone2);
        elseif i == 3 then
          Timeline.logEvent1(TimelineKeys.DistanceTrippedZone3);
        else
          Timeline.logEvent1(TimelineKeys.DistanceTrippedZone4);
        end if;
      end when;
    end for;

    when sum(Tripped) >= 1 then
      if lineState.value == 2 or lineState.value == 5 - LineSide then  // Other end is closed
        lineState.value = 5 - LineSide;
      else  // Other end is open
        lineState.value = 1;
      end if;
    end when;

    annotation(
      preferredView = "text",
      Documentation(info = "<html><head></head><body> </body></html>"));
  end BaseDistanceProtectionLine;

  partial model BaseUFLS "Base model for under-frequency load shedding relays"
    import Modelica.Constants;
    import Modelica.Blocks.Interfaces.RealInput;
    import Dynawo.NonElectrical.Logs.Timeline;
    import Dynawo.NonElectrical.Logs.TimelineKeys;

    parameter Integer NbSteps(min = 1, max = 10) "Number of steps in the UFLS scheme";

    parameter Types.Time tLagAction "Time-lag due to the actual trip action in s";
    parameter Types.Time tFilter = 0.01 "Time constant of the load change filter in s";

    parameter Types.AngularVelocityPu[NbSteps] omegaThresholdPu "Frequency threshold of the UFLS scheme in pu (base omegaNom) (for each step of the UFLS scheme)";
    parameter Real[NbSteps] UFLSStep "Share of load disconnected by a step of UFLS (sum of all steps should be at most 1) (for each step of the UFLS scheme)";

    RealInput omegaMonitoredPu "Monitored frequency in pu (base omegaNom)";

    Types.Time[NbSteps] tThresholdReached(each start = Constants.inf) "Time when reaches the ith UFLS threshold (for each step of the UFLS scheme)";
    Integer[NbSteps] stepActivated(each start = 0) "1 if the ith step of UFLS has been activated, else 0 (for each step of the UFLS scheme)";

    Real deltaPQ(start = 0) "Delta to apply on PRef and QRef in % due to UFLS disconnections, to connect to the variables deltaP and deltaQ in the load model";
    Real deltaPQfiltered(start = 0) "Smoothed out version of deltaPQ for better numerical stability";

  equation
    der(deltaPQfiltered) * tFilter = deltaPQ - deltaPQfiltered;

    for i in 1:NbSteps loop
      when omegaMonitoredPu <= omegaThresholdPu[i] and not (pre(stepActivated[i]) == 1) then
        tThresholdReached[i] = time;

        if i == 1 then
          Timeline.logEvent1(TimelineKeys.UFLS1Arming);
        elseif i == 2 then
          Timeline.logEvent1(TimelineKeys.UFLS2Arming);
        elseif i == 3 then
          Timeline.logEvent1(TimelineKeys.UFLS3Arming);
        elseif i == 4 then
          Timeline.logEvent1(TimelineKeys.UFLS4Arming);
        elseif i == 5 then
          Timeline.logEvent1(TimelineKeys.UFLS5Arming);
        elseif i == 6 then
          Timeline.logEvent1(TimelineKeys.UFLS6Arming);
        elseif i == 7 then
          Timeline.logEvent1(TimelineKeys.UFLS7Arming);
        elseif i == 8 then
          Timeline.logEvent1(TimelineKeys.UFLS8Arming);
        elseif i == 9 then
          Timeline.logEvent1(TimelineKeys.UFLS9Arming);
        else
          Timeline.logEvent1(TimelineKeys.UFLS10Arming);
        end if;
      end when;
    end for;

    // Trips
    for i in 1:NbSteps loop
      when time - tThresholdReached[i] >= tLagAction then
        stepActivated[i] = 1;

        if i == 1 then
          Timeline.logEvent1(TimelineKeys.UFLS1Activated);
        elseif i == 2 then
          Timeline.logEvent1(TimelineKeys.UFLS2Activated);
        elseif i == 3 then
          Timeline.logEvent1(TimelineKeys.UFLS3Activated);
        elseif i == 4 then
          Timeline.logEvent1(TimelineKeys.UFLS4Activated);
        elseif i == 5 then
          Timeline.logEvent1(TimelineKeys.UFLS5Activated);
        elseif i == 6 then
          Timeline.logEvent1(TimelineKeys.UFLS6Activated);
        elseif i == 7 then
          Timeline.logEvent1(TimelineKeys.UFLS7Activated);
        elseif i == 8 then
          Timeline.logEvent1(TimelineKeys.UFLS8Activated);
        elseif i == 9 then
          Timeline.logEvent1(TimelineKeys.UFLS9Activated);
        else
          Timeline.logEvent1(TimelineKeys.UFLS10Activated);
        end if;
      end when;
    end for;

    deltaPQ = -sum(stepActivated .* UFLSStep);

    annotation(
      preferredView = "text",
      Documentation(info = "<html><head></head><body> </body></html>"));
  end BaseUFLS;

end BaseClasses;

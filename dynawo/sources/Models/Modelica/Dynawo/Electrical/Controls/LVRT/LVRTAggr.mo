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

model LVRTAggr "Low voltage ride through with \"partial disconnection\" feature"

  import Modelica;
  extends Modelica.Blocks.Interfaces.SISO(y(start = 1));

  import Modelica.Constants;

  import Dynawo.Connectors;
  import Dynawo.NonElectrical.Logs.Timeline;
  import Dynawo.NonElectrical.Logs.TimelineKeys;

  public
    parameter Types.VoltageModulePu URPu "Voltage threshold under which the automaton is activated after T2";
    parameter Types.VoltageModulePu UIntPu "Voltage threshold under which the automaton is activated after T1";
    parameter Types.VoltageModulePu UMinPu "Voltage threshold under which the automaton is activated instantaneously";
    parameter Types.Time T1 "Time delay of trip for severe voltage dips";
    parameter Types.Time TInt "Time delay of trip for medium voltage dips";
    parameter Types.Time T2 "Time delay of trip for small voltage dips";

    // Parameters of the partial tripping curves
    parameter Types.PerUnit c;
    parameter Types.PerUnit d;
    parameter Types.PerUnit e;
    parameter Types.PerUnit f;
    parameter Types.PerUnit u_;

    Connectors.BPin switchOffSignal (value (start = false)) "Switch off message for the generator";

  protected
    Types.Time tThresholdReached (start = Constants.inf) "Time when the threshold was reached";

    Types.PerUnit f1 (start = 1);
    Types.PerUnit f2 (start = 1);
    Types.PerUnit f3 (start = 1);

    Types.VoltageModulePu UMin1 (start = 1) "Minimum voltage in period [0, T1], 0 being the occurance of the fault";
    Types.VoltageModulePu UMinInt (start = 1) "Minimum voltage in period [T1, TInt], 0 being the occurance of the fault";

  equation
    assert(TInt/T2 < u_ and u_ <= 1, "TInt/T2 < u_ <= 1");

    // Voltage comparison with the minimum accepted value
    when u <= URPu and not(pre(switchOffSignal.value)) then
      tThresholdReached = time;
      Timeline.logEvent1(TimelineKeys.LVRTArming);
    elsewhen u > URPu and pre(tThresholdReached) < 999999999 and not(pre(switchOffSignal.value)) then
      tThresholdReached = Constants.inf;
      Timeline.logEvent1(TimelineKeys.LVRTDisarming);
    end when;

    y = f1*f2*f3;

    if switchOffSignal.value == true then // Full disconnection
      /*f1 = 0;
      f2 = 0;
      f3 = 0;*/
      der(f1) = -1e4  * f1; // Basically, f1 = 0
      der(f2) = -1e4  * f2;
      der(f3) = -1e4  * f3;
      der(UMin1) = 0;
      der(UMinInt) = 0;

    elseif tThresholdReached > 999999999 then // Assume no reconnections
      der(f1) = 0;
      der(f2) = 0;
      der(f3) = 0;
      der(UMin1) = 0;
      der(UMinInt) = 0;

    elseif time - tThresholdReached <= T1 then
      // min(u, pre(UMin1)); // Use u_min + T*der(u_min) = if u <= u_min then u else u_min; if does not work
      // der(UMin1) = -1e4 * (UMin1-u);
      der(UMin1) = -1e4 * (UMin1-min(u, UMin1));
      if UMin1 > UMinPu then
        der(f1) = -1e4  * (f1 - 1);
      elseif UMin1 > d*UMinPu then
        der(f1) = -1e4 * (1 - c * (1 - (UMinPu - UMin1)/(UMinPu - d*UMinPu)));
      else
        der(f1) = -1e4  * f1;
      end if;
      der(f2) = 0;
      der(f3) = 0;
      der(UMinInt) = 0;

    elseif time - tThresholdReached < TInt - 1e-4 then // To be sure this condition is never actived if TInt = T1
      // UMinInt = min(u, UMinInt);
      // der(UMinInt) = -1e-6 * (UMinInt - u);
      der(UMinInt) = -1e-6 * (UMinInt - min(u, UMinInt));
      if UMinInt > UMinPu then
        // f2 = 1;
        der(f2) = -1e4  * (f2 - 1);
      elseif UMinInt > f*UMinPu then
        der(f2) = -1e4  * (f2 - e * (1 - (UMinPu - UMinInt)/(UMinPu - f*UMinPu)));
      else
        // f2 = 0;
        der(f2) = -1e4  * f2;
      end if;
      der(f1) = 0;
      der(f3) = 0;
      der(UMin1) = 0;

    else
      der(f3) = -1/(u*T2 - TInt);
      der(f1) = 0;
      der(f2) = 0;
      der(UMin1) = 0;
      der(UMinInt) = 0;
    end if;

    when y < 0.01 then
      switchOffSignal.value = true;
      Timeline.logEvent1(TimelineKeys.LVRTTripped);
    end when;

annotation(preferredView = "text",
    Documentation(info = "<html><head></head><body>This model progressively reduces the output of an (inverter-connected) generator if the voltage stays below a LVRT curve in order to emulate disconnection of distributed generation. See fig 17 in <a href=\"https://orbi.uliege.be/bitstream/2268/212146/1/Full_paper.pdf\">https://orbi.uliege.be/bitstream/2268/212146/1/Full_paper.pdf</a></body></html>"));
end LVRTAggr;

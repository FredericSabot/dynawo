within Dynawo.Electrical.HVDC.HvdcPV;

/*
* Copyright (c) 2015-2020, RTE (http://www.rte-france.com)
* See AUTHORS.txt
* All rights reserved.
* This Source Code Form is subject to the terms of the Mozilla Public
* License, v. 2.0. If a copy of the MPL was not distributed with this
* file, you can obtain one at http://mozilla.org/MPL/2.0/.
* SPDX-License-Identifier: MPL-2.0
*
* This file is part of Dynawo, an hybrid C++/Modelica open source suite of simulation tools for power systems.
*/

model HvdcPV "Model of PV HVDC link. Each terminal can regulate the voltage or the reactive power, depending on the user's choice."
  import Modelica;
  import Dynawo.Electrical.HVDC;

  extends HVDC.BaseClasses.BaseHvdcP;
  extends AdditionalIcons.Hvdc;

/*
  Equivalent circuit and conventions:

               I1                  I2
   (terminal1) -->-------HVDC-------<-- (terminal2)

*/

  type QStatus = enumeration (Standard "Reactive power is fixed to its initial value",
                              AbsorptionMax "Reactive power is fixed to its absorption limit",
                              GenerationMax "Reactive power is fixed to its generation limit");

  parameter Types.ReactivePowerPu Q1MinPu "Minimum reactive power in pu (base SnRef) at terminal 1 (receptor convention)";
  parameter Types.ReactivePowerPu Q1MaxPu "Maximum reactive power in pu (base SnRef) at terminal 1 (receptor convention)";
  parameter Types.ReactivePowerPu Q2MinPu "Minimum reactive power in pu (base SnRef) at terminal 2 (receptor convention)";
  parameter Types.ReactivePowerPu Q2MaxPu "Maximum reactive power in pu (base SnRef) at terminal 2 (receptor convention)";

  input Types.VoltageModulePu U1RefPu(start = U1Ref0Pu) "Voltage regulation set point in pu (base UNom) at terminal 1";
  input Types.VoltageModulePu U2RefPu(start = U2Ref0Pu) "Voltage regulation set point in pu (base UNom) at terminal 2";
  input Types.ReactivePowerPu Q1RefPu(start = Q1Ref0Pu) "Reactive power regulation set point in pu (base SnRef) (receptor convention) at terminal 1";
  input Types.ReactivePowerPu Q2RefPu(start = Q2Ref0Pu) "Reactive power regulation set point in pu (base SnRef) (receptor convention) at terminal 2";
  input Boolean modeU1(start = modeU10) "Boolean assessing the mode of the control: true if U mode, false if Q mode";
  input Boolean modeU2(start = modeU20) "Boolean assessing the mode of the control: true if U mode, false if Q mode";

  Types.Angle Theta1(start = UPhase10) "Angle of the voltage at terminal 1 in rad";
  Types.Angle Theta2(start = UPhase20) "Angle of the voltage at terminal 2 in rad";

  parameter Types.VoltageModulePu U1Ref0Pu "Start value of the voltage regulation set point in pu (base UNom) at terminal 1";
  parameter Types.VoltageModulePu U2Ref0Pu "Start value of the voltage regulation set point in pu (base UNom) at terminal 2";
  parameter Types.ReactivePowerPu Q1Ref0Pu "Start value of reactive power regulation set point in pu (base SnRef) (receptor convention) at terminal 1";
  parameter Types.ReactivePowerPu Q2Ref0Pu "Start value of reactive power regulation set point in pu (base SnRef) (receptor convention) at terminal 2";
  parameter Boolean modeU10 "Start value of the boolean assessing the mode of the control at terminal 1: true if U mode, false if Q mode";
  parameter Boolean modeU20 "Start value of the boolean assessing the mode of the control at terminal 2: true if U mode, false if Q mode";
  parameter Types.Angle UPhase10 "Start value of voltage angle and filtered voltage angle at terminal 1 in rad";
  parameter Types.Angle UPhase20 "Start value of voltage angle and filtered voltage angle at terminal 2 in rad";

protected
  QStatus q1Status(start = QStatus.Standard) "Voltage regulation status of terminal 1: standard, absorptionMax or generationMax";
  QStatus q2Status(start = QStatus.Standard) "Voltage regulation status of terminal 2: standard, absorptionMax or generationMax";

equation
  Theta1 = Modelica.Math.atan2(terminal1.V.im, terminal1.V.re);
  Theta2 = Modelica.Math.atan2(terminal2.V.im, terminal2.V.re);

  when Q1Pu >= Q1MaxPu and U1Pu >= U1RefPu then
    q1Status = QStatus.AbsorptionMax;
  elsewhen Q1Pu <= Q1MinPu and U1Pu <= U1RefPu then
    q1Status = QStatus.GenerationMax;
  elsewhen (Q1Pu < Q1MaxPu or U1Pu < U1RefPu) and (Q1Pu > Q1MinPu or U1Pu > U1RefPu) then
    q1Status = QStatus.Standard;
  end when;

  when Q2Pu >= Q2MaxPu and U2Pu >= U2RefPu then
    q2Status = QStatus.AbsorptionMax;
  elsewhen Q2Pu <= Q2MinPu and U2Pu <= U2RefPu then
    q2Status = QStatus.GenerationMax;
  elsewhen (Q2Pu < Q2MaxPu or U2Pu < U2RefPu) and (Q2Pu > Q2MinPu or U2Pu > U2RefPu) then
    q2Status = QStatus.Standard;
  end when;

// Voltage/Reactive power regulation at terminal 1
if runningSide1.value then
  if modeU1 then
    if q1Status == QStatus.GenerationMax then
      Q1Pu = Q1MinPu;
    elseif q1Status == QStatus.AbsorptionMax then
      Q1Pu = Q1MaxPu;
    else
      U1Pu = U1RefPu;
    end if;
  else
    if Q1RefPu <= Q1MinPu then
      Q1Pu = Q1MinPu;
    elseif Q1RefPu >= Q1MaxPu then
      Q1Pu = Q1MaxPu;
    else
      Q1Pu = Q1RefPu;
    end if;
  end if;
else
  Q1Pu = 0;
end if;

// Voltage/Reactive power regulation at terminal 2
if runningSide2.value then
  if modeU2 then
    if q2Status == QStatus.GenerationMax then
      Q2Pu = Q2MinPu;
    elseif q2Status == QStatus.AbsorptionMax then
      Q2Pu = Q2MaxPu;
    else
      U2Pu = U2RefPu;
    end if;
  else
    if Q2RefPu <= Q2MinPu then
      Q2Pu = Q2MinPu;
    elseif Q2RefPu >= Q2MaxPu then
      Q2Pu = Q2MaxPu;
    else
      Q2Pu = Q2RefPu;
    end if;
  end if;
else
  Q2Pu = 0;
end if;

  annotation(preferredView = "text",
    Documentation(info = "<html><head></head><body> This HVDC link regulates the active power flowing through itself. It also regulates the voltage or the reactive power at each of its terminals. The active power setpoint is given as an input and can be modified during the simulation, as well as the voltage references and the reactive power references.</div></body></html>"));
end HvdcPV;

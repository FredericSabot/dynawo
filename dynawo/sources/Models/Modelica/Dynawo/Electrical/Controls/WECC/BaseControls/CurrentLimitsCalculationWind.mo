within Dynawo.Electrical.Controls.WECC.BaseControls;

/*
* Copyright (c) 2021, RTE (http://www.rte-france.com)
* See AUTHORS.txt
* All rights reserved.
* This Source Code Form is subject to the terms of the Mozilla Public
* License, v. 2.0. If a copy of the MPL was not distributed with this
* file, you can obtain one at http://mozilla.org/MPL/2.0/.
* SPDX-License-Identifier: MPL-2.0
*
* This file is part of Dynawo, an hybrid C++/Modelica open source suite of simulation tools for power systems.
*/

model CurrentLimitsCalculationWind "This block calculates the current limits of the WECC regulation for the Wind Turbine"
  import Modelica;
  import Dynawo.Types;


  parameter Types.PerUnit IMaxPu "Maximum inverter current amplitude in p.u (base UNom, SNom)";
  parameter Boolean PPriority "Priority: reactive power (false) or active power (true)";
  parameter SIunits.Time tIpMax "Time delay for which the active current limit (Ipmax) is held after voltage dip vDip returns to zero for tIpMax seconds at its value during the voltage dip";

 // Inputs
  Modelica.Blocks.Interfaces.RealInput ipCmdPu "p-axis command current in p.u (base UNom, SNom)" annotation(
    Placement(visible = true, transformation(origin = {-110, -40}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {-110, 40}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealInput iqCmdPu "q-axis command current in p.u (base UNom, SNom)" annotation(
    Placement(visible = true, transformation(origin = {-110, 40}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {-110, -40}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Interfaces.BooleanInput vDip "Ongoing voltage dip" annotation(
    Placement(visible = true, transformation(origin = {-112, 2.22045e-16}, extent = {{-12, -12}, {12, 12}}, rotation = 0), iconTransformation(origin = {-105, -1}, extent = {{-11, -11}, {11, 11}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealInput ipVdlPu "Voltage dependent limit for active current command" annotation(
    Placement(visible = true, transformation(origin = {-111, 33}, extent = {{-11, -11}, {11, 11}}, rotation = 0), iconTransformation(origin = {-105, 41}, extent = {{-11, -11}, {11, 11}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealInput iqVdlPu "Voltage dependent limit for reactive current command" annotation(
    Placement(visible = true, transformation(origin = {-111, -37}, extent = {{-11, -11}, {11, 11}}, rotation = 0), iconTransformation(origin = {-105, -41}, extent = {{-11, -11}, {11, 11}}, rotation = 0)));

 //Outputs:
  Modelica.Blocks.Interfaces.RealOutput ipMaxPu "p-axis maximum current in p.u (base UNom, SNom)" annotation(
    Placement(visible = true, transformation(origin = {110, -60}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110, 20}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealOutput iqMaxPu "q-axis maximum current in p.u (base UNom, SNom)" annotation(
    Placement(visible = true, transformation(origin = {110, 20}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110, -60}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealOutput ipMinPu "p-axis minimum current in p.u (base UNom, SNom)" annotation(
    Placement(visible = true, transformation(origin = {110, -20}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110, 60}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealOutput iqMinPu "q-axis minimum current in p.u (base UNom, SNom)" annotation(
    Placement(visible = true, transformation(origin = {110, 60}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110, -20}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));

protected
  Boolean vDipStart;
  SIunits.PerUnit ipmaxFrzPu;
  SIunits.Time vDipFrzEndTime;
  Types.PerUnit ipLimPu = max(min(abs(ipCmdPu), IMaxPu), 0);
  Types.PerUnit iqLimPu = max(min(abs(iqCmdPu), IMaxPu), - IMaxPu);


    /*  CONTROL VARIABLE EXPLANATION:
   *
   *  The algorithm in this block is similar to the one in the IqInj_logic block,
   *  where it handles time sensitive output control. For a detailed description,
   *  see the IqInj_logic block.
   *
   *  1. At the time the voltage dip ends, the momentary value of Ipmax is memorized
   *     thanks to the when clause, assigned to Ipmax_frz and the end timing is
   *     calculated. During this time, Ipmax is set to Ipmax_frz.
   *  2. When the end timing is reached, Ipmax_frz is reset and the Ipmax uses then
   *     usual calculation formula.
   *
   */

initial algorithm
  vDipStart := false;
  vDipFrzEndTime := -1;
  ipmaxFrzPu := 0;

algorithm
  when vDip == true then
    vDipStart := true;
    vDipFrzEndTime := -1;
  end when;
  when vDip == false and vDipStart == true then
    vDipStart := false;
    vDipFrzEndTime := time + abs(tIpMax);
    ipmaxFrzPu := ipMaxPu;
  end when;
  when time >= vDipFrzEndTime and vDipFrzEndTime >= 0 then
    vDipFrzEndTime := -1;
    ipmaxFrzPu := 0;
  end when;

equation

  if PPriority then
    if time <= vDipFrzEndTime and vDipFrzEndTime >= 0 then
      ipMaxPu = ipmaxFrzPu;
    else
      ipMaxPu = min(ipVdlPu, IMaxPu);
    end if;
    ipMinPu = 0;
    iqMaxPu = min(Iq_vdl, sqrt(Imax ^ 2 - min(Ip_lim,Ip_vdl) ^ 2)); // WECC Implementation, difference in validation against PowerFactory
    // Iqmax = min(Iq_vdl, sqrt(max(0, min(Imax,Iq_vdl) ^ 2 - Ipcmd ^ 2))); // PowerFactory Implementation, difference in validation against REMVT
    Iqmin = -Iqmax;
  else
  // Q priority
    if time <= Vdip_frz_endTime and Vdip_frz_endTime >= 0 then
      Ipmax = Ipmax_frz;
    else
      Ipmax = min(Ip_vdl, sqrt(Imax ^ 2 - min(Iq_lim,Iq_vdl) ^ 2));// WECC Implementation, difference in validation against PowerFactory
      // Ipmax = min(Ip_vdl, sqrt(max(0, min(Imax,Ip_vdl)^ 2 - Iqcmd ^ 2))); // PowerFactory Implementation, difference in validation against REMVT
    end if;
    Ipmin = 0;
    Iqmax = min(Iq_vdl, Imax);
    Iqmin = -Iqmax;
  end if;



  annotation(preferredView = "text",
    Icon(graphics = {Rectangle(extent = {{-100, 100}, {100, -100}}), Text(origin = {44, -1}, extent = {{-124, 81}, {36, 21}}, textString = "Current"), Text(origin = {-115, -25}, extent = {{-27, 9}, {13, -3}}, textString = "iqCmdPu"), Text(origin = {-115, 53}, extent = {{-27, 9}, {13, -3}}, textString = "ipCmdPu"), Text(origin = {127, -9}, extent = {{-27, 9}, {13, -3}}, textString = "iqMinPu"), Text(origin = {127, -49}, extent = {{-27, 9}, {13, -3}}, textString = "iqMaxPu"), Text(origin = {127, 71}, extent = {{-27, 9}, {13, -3}}, textString = "ipMinPu"), Text(origin = {127, 31}, extent = {{-27, 9}, {13, -3}}, textString = "ipMaxPu"), Text(origin = {44, -61}, extent = {{-124, 41}, {36, -19}}, textString = "limits")}, coordinateSystem(initialScale = 0.1)));


end CurrentLimitsCalculationWind;

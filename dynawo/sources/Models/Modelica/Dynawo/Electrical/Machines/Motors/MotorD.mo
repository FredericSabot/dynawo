within Dynawo.Electrical.Machines.Motors;

/*
* Copyright (c) 2023, RTE (http://www.rte-france.com)
* See AUTHORS.txt
* All rights reserved.
* This Source Code Form is subject to the terms of the Mozilla Public
* License, v. 2.0. If a copy of the MPL was not distributed with this
* file, you can obtain one at http://mozilla.org/MPL/2.0/.
* SPDX-License-Identifier: MPL-2.0
*
* This file is part of Dynawo, an hybrid C++/Modelica open source suite of simulation tools for power systems.
*/

model MotorD "Single-phase air conditioner performance-based model from WECC"
  extends BaseClasses.BaseMotor;
  extends AdditionalIcons.Machine;
  import Modelica.Constants;

  // Global parameters
  parameter Real ShareRestarting "Share of motors that can restart after stalling";
  // Motor parameters
  parameter Real PF "Power factor of the motor";
  parameter Types.VoltageModulePu UStallPu "Voltage at which the motor stalls in pu (base UNom)";
  parameter Types.VoltageModulePu UBreakPu = 0.86 "Voltage below which the active power consumed by the motor increases in pu (base UNom)";
  parameter Types.PerUnit RStallPu "Equivalent resistance of the motor when stalled in pu (base SNom, UNom)";
  parameter Types.PerUnit XStallPu "Equivalent impedance of the motor when stalled in pu (base SNom, UNom)";
  parameter Types.Time tRestart "Time needed to restart the motor in s";
  parameter Types.Time tStall "Time after which the motor stalls if the voltage stays below UStallPu in s";
  parameter Real KP1 = 0 "Proportional factor of active power voltage dependance when UPu < UBreakPu";
  parameter Real KP2 = 12 "Proportional factor of active power voltage dependance when UPu > UBreakPu";
  parameter Real NP1 = 0 "Exponent of the active power voltage dependance when UPu < UBreakPu";
  parameter Real NP2 = 3.2 "Exponent of the active power voltage dependance when UPu > UBreakPu";
  parameter Real KQ1 = 6 "Proportional factor of reactive power voltage dependance when UPu > UBreakPu";
  parameter Real KQ2 = 11 "Proportional factor of reactive power voltage dependance when UPu < UBreakPu";
  parameter Real NQ1 = 2 "Exponent of the reactive power voltage dependance when UPu > UBreakPu";
  parameter Real NQ2 = 2.5 "Exponent of the reactive power voltage dependance when UPu < UBreakPu";
  // UVA parameters
  parameter Types.VoltageModulePu UMinPu[2] "Voltage threshold under which the automaton is activated in pu (base UNom) (for each UV relay)";
  parameter Types.Time tLagAction[2] "Time-lag due to the actual trip action in s (for each UV relay)";
  parameter Real ShareToDisconnect "Share of motors that are disconnected by UV relays (not cumulative)";

  // Init
  parameter Types.VoltageModulePu UStallBreakPu "Voltage where the stalling and non-stalling characteristics cross in pu (base UNom)";

  Motor motorA(SNom=SNom * (1-ShareRestarting), P0Pu=SNom * (1-ShareRestarting), Q0Pu=SNom * (1-ShareRestarting)*tan(acos(PF)), RestartPossible=false, PF=PF, UStallPu=UStallPu, UBreakPu=UBreakPu, RStallPu=RStallPu, XStallPu=XStallPu, tRestart=tRestart, tStall=tStall, KP1=KP1, KP2=KP2, NP1=NP1, NP2=NP2, KQ1=KQ1, KQ2=KQ2, NQ1=NQ1, NQ2=NQ2, UStallBreakPu=UStallBreakPu, u0Pu=u0Pu);
  Motor motorB(SNom=SNom * ShareRestarting, P0Pu=SNom * ShareRestarting, Q0Pu=SNom * (1-ShareRestarting)*tan(acos(PF)), RestartPossible=true, PF=PF, UStallPu=UStallPu, UBreakPu=UBreakPu, RStallPu=RStallPu, XStallPu=XStallPu, tRestart=tRestart, tStall=tStall, KP1=KP1, KP2=KP2, NP1=NP1, NP2=NP2, KQ1=KQ1, KQ2=KQ2, NQ1=NQ1, NQ2=NQ2, UStallBreakPu=UStallBreakPu, u0Pu=u0Pu);
  UVRelay uvRelay(UMinPu=UMinPu, tLagAction=tLagAction, ShareToDisconnect=ShareToDisconnect);
  ThermalRelay thermalRelayA;
  ThermalRelay thermalRelayB;

  // Firs test without contactors as they will probably not work

  model Motor
    extends AdditionalIcons.Machine;

    parameter Types.ApparentPowerModule SNom "Nominal apparent power of the motor in MVA";
    parameter Real PF  "Power factor of the motor";
    parameter Types.VoltageModulePu UStallPu "Voltage at which the motor stalls in pu (base UNom)";
    parameter Types.VoltageModulePu UBreakPu = 0.86 "Voltage below which the active power consumed by the motor increases in pu (base UNom)";
    parameter Types.PerUnit RStallPu "Equivalent resistance of the motor when stalled in pu (base SNom, UNom)";
    parameter Types.PerUnit XStallPu "Equivalent impedance of the motor when stalled in pu (base SNom, UNom)";
    parameter Boolean RestartPossible "True if the motor can restart after stalling";
    parameter Types.Time tRestart "Time needed to restart the motor in s";
    parameter Types.Time tStall "Time after which the motor stalls if the voltage stays below UStallPu in s";
    parameter Real KP1 = 0 "Proportional factor of active power voltage dependance when UPu < UBreakPu";
    parameter Real KP2 = 12 "Proportional factor of active power voltage dependance when UPu > UBreakPu";
    parameter Real NP1 = 0 "Exponent of the active power voltage dependance when UPu < UBreakPu";
    parameter Real NP2 = 3.2 "Exponent of the active power voltage dependance when UPu > UBreakPu";
    parameter Real KQ1 = 6 "Proportional factor of reactive power voltage dependance when UPu > UBreakPu";
    parameter Real KQ2 = 11 "Proportional factor of reactive power voltage dependance when UPu < UBreakPu";
    parameter Real NQ1 = 2 "Exponent of the reactive power voltage dependance when UPu > UBreakPu";
    parameter Real NQ2 = 2.5 "Exponent of the reactive power voltage dependance when UPu < UBreakPu";

    parameter Types.ComplexVoltagePu u0Pu "Start value of complex voltage at load terminal in pu (base UNom)";
    parameter Types.ActivePowerPu P0Pu "Start value of active power at terminal in pu (base SNom) (receptor convention)";
    parameter Types.ReactivePowerPu Q0Pu "Start value of reactive power at terminal in pu (base SNom) (receptor convention)";
    parameter Types.VoltageModulePu UStallBreakPu "Voltage where the stalling and non-stalling characteristics cross in pu (base UNom)";

    Connectors.ImPin UPu(value(start = ComplexMath.'abs'(u0Pu))) "Voltage amplitude at motor terminal in pu (base UNom)";
    Connectors.ImPin omegaRefPu(value(start = SystemBase.omegaRef0Pu)) "Network angular reference frequency in pu (base omegaNom)";
    Connectors.BPin running(value(start = true)) "True if the motor is running";

    Types.ActivePowerPu PPu(start = P0Pu) "Active power at load terminal in pu (base SnRef) (receptor convention)";
    Types.ReactivePowerPu QPu(start = Q0Pu) "Reactive power at load terminal in pu (base SnRef) (receptor convention)";
    Types.ActivePowerPu heatingPowerPu(start = 0) "Power losses, considered in pu (base SNom)";

    Boolean stalled(start = false) "True if the motor is stalled";
    Types.Time tStallingStarted(start = Constants.inf) "Time when the motor started stalling, i.e. when UPu dropped below UStallPu";
    Types.Time tRestartStarted(start = Constants.inf) "Time when the motor started to restart, i.e. when UPu increasex above UStallPu";

  equation
    if running.value then
      if UPu.value > UBreakPu and not stalled then
        PPu = (P0Pu + KP1 * (UPu.value - UBreakPu)^NP1) * (1 + omegaRefPu.value);
        QPu = (Q0Pu + KQ1 * (UPu.value - UBreakPu)^NQ1) * (1 - 3.3 * omegaRefPu.value);
        heatingPowerPu = 0;
      elseif UPu.value > UStallPu and not stalled then
        PPu = (P0Pu + KP2 * (UBreakPu - UPu.value)^NP2) * (1 + omegaRefPu.value);
        QPu = (Q0Pu + KQ2 * (UBreakPu - UPu.value)^NQ2) * (1 - 3.3 * omegaRefPu.value);
        heatingPowerPu = 0;
      else
        PPu = UPu.value^2 / RStallPu * (SNom / SystemBase.SnRef);
        QPu = - UPu.value^2 / XStallPu * (SNom / SystemBase.SnRef);
        heatingPowerPu = UPu.value^2 / RStallPu;
      end if;
    else
      PPu = 0;
      QPu = 0;
      heatingPowerPu = 0;
    end if;

    when UPu.value <= UStallPu and not pre(stalled) and running.value then
      tStallingStarted = time;
    elsewhen UPu.value > UStallPu and pre(tStallingStarted) <> Constants.inf and not pre(stalled) and running.value then
      tStallingStarted = Constants.inf;
    end when;

    when UPu.value > UStallPu and RestartPossible and pre(stalled) then
      tRestartStarted = time;
    elsewhen UPu.value < UStallPu and RestartPossible and pre(stalled) then
      tRestartStarted = Constants.inf;
    end when;

    when time - tStallingStarted > tStall then
      stalled = true;
    elsewhen (time - tRestartStarted) > tRestart and (time - tRestartStarted) < tStall then
      stalled = false;
    end when;

  end Motor;

  model Contactor

    parameter Types.VoltageModulePu UOn1Pu "Voltage above which all contactors are closed in pu (base UNom)";
    parameter Types.VoltageModulePu UOn2Pu "Voltage above which contactors start to reconnect in pu (base UNom)";
    parameter Types.VoltageModulePu UOff1Pu "Voltage under which contactors start to disconnect in pu (base UNom)";
    parameter Types.VoltageModulePu UOff2Pu "Voltage under which contactors are all disconnected in pu (base UNom)";
    parameter Real Tolerance = 1e-5 "Tolerance on derivative of input";

    input Types.VoltageModulePu UPu(start = ComplexMath.'abs'(u0Pu)) "Voltage amplitude at motor terminal in pu (base UNom)";

    Real connectedShare(start = 1) "Share of the motors with closed contactors";

  equation
    if UPu < UOff2Pu then
      connectedShare = 0;
    elseif UPu > UOn1Pu then
      connectedShare = 1;
    elseif UPu < (UOff2Pu + (UOff1Pu - UOff2Pu) * connectedShare) then
      connectedShare = (UPu - UOff2Pu) / (UOff1Pu - UOff2Pu);
    elseif UPu > (UOn2Pu + (UOn1Pu - UOn2Pu) * connectedShare) then
      connectedShare = (UPu - UOn2Pu) / (UOn1Pu - UOff2Pu);
    else
      der(connectedShare) = 0;
    end if;

  end Contactor;

  model ThermalRelay

    parameter Types.Time tTh "Thermal time constant in s";
    parameter Types.PerUnit T1 "Temperature at which motors start to trip in pu (base SNom)";
    parameter Types.PerUnit T2 "Temperature at which all motors are tripped in pu (base SNom)";
    parameter Types.Time tFilter = 1e-2 "Time constant for calculation of maxTemp in s";

    input Types.ActivePowerPu heatingPowerPu (start = 0) "Heating power in pu (base SNom, UNom)";

    Real shareConnected(start = 1) "Share of motors that are still connected";

    Types.PerUnit maxTemp(start = 0) "Maximum temperature reaches during the simulation in pu (base SNom)";
    Types.PerUnit temp(start = 0) "Temperature in pu (base SNom)";

  equation
    maxTemp + tFilter * der(maxTemp) = if temp < maxTemp then maxTemp else temp;  // Track maximum temperature
    der(temp) = (heatingPowerPu - temp) / tTh;

    if maxTemp < T1 then
      shareConnected = 1;
    elseif maxTemp > T2 then
      shareConnected = 0;
    else
      shareConnected = 1 - (maxTemp - T1) / (T2 - T1);
    end if;

  end ThermalRelay;

  model UVRelay "Undervoltage relay model of the WECC motor D performance model"

    parameter Types.VoltageModulePu UMinPu[2] "Voltage threshold under which the automaton is activated in pu (base UNom) (for each UV relay)";
    parameter Types.Time tLagAction[2] "Time-lag due to the actual trip action in s (for each UV relay)";
    parameter Real ShareToDisconnect "Share of motors that are disconnected by UV relays (not cumulative)";

    Types.VoltageModulePu UMonitoredPu "Monitored voltage in pu (base UNom)";

    Real shareConnected(start = 1) "Share of motors that are still connected";

    Boolean tripped[2] (each start = false) "True if UV relay i tripped (for each UV relay)";

  protected
    Types.Time tThresholdReached[2](each start = Constants.inf) "Time when the threshold was reached (for each UV relay)";

  equation
    for i in 1:2 loop
      // Voltage comparison with the minimum accepted value
      when UMonitoredPu <= UMinPu[i] and not(pre(tripped[i])) then
        tThresholdReached[i] = time;
      elsewhen UMonitoredPu > UMinPu[i] and pre(tThresholdReached[i]) <> Constants.inf and not(pre(tripped[i])) then
        tThresholdReached[i] = Constants.inf;
      end when;

      // Delay before tripping the generator
      when time - tThresholdReached[i] >= tLagAction[i] then
        tripped[i] = true;
      end when;
    end for;

    if tripped[1] or tripped[2] then
      shareConnected = 1 - ShareToDisconnect;
    else
      shareConnected = 1;
    end if;

  end UVRelay;

equation
  motorA.UPu.value = ComplexMath.'abs'(V);
  motorB.UPu.value = ComplexMath.'abs'(V);
  connect(omegaRefPu, motorA.omegaRefPu);
  connect(omegaRefPu, motorB.omegaRefPu);
  connect(running, motorA.running);
  connect(running, motorB.running);

  uvRelay.UMonitoredPu = ComplexMath.'abs'(V);

  thermalRelayA.heatingPowerPu = motorA.heatingPowerPu;
  thermalRelayB.heatingPowerPu = motorB.heatingPowerPu;

  if running.value then
    PPu = (motorA.PPu * thermalRelayA.shareConnected + motorB.PPu * thermalRelayB.shareConnected) * uvRelay.shareConnected * (SNom/SystemBase.SnRef);
    QPu = (motorA.QPu * thermalRelayA.shareConnected + motorB.QPu * thermalRelayB.shareConnected) * uvRelay.shareConnected * (SNom/SystemBase.SnRef);
  else
    PPu = 0;
    QPu = 0;
  end if;

annotation(preferredView = "text");
end MotorD;

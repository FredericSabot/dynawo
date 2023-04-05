within Dynawo.Electrical.InverterBasedGeneration;

model AggregatedIBG "Aggregated model of inverter-based generation (IBG)"
  import Dynawo;
  import Modelica;
  import Dynawo.NonElectrical.Logs.Timeline;
  import Dynawo.NonElectrical.Logs.TimelineKeys;

  // Rating
  parameter Types.ApparentPowerModule SNom "Nominal apparent power of the injector (in MVA)";
  parameter Types.CurrentModulePu IMaxPu "Maximum current of the injector in pu (base UNom, SNom)";

  // Filters
  parameter Types.Time tFilterU "Voltage measurement first order time constant in s";
  parameter Types.Time tFilterOmega "First order time constant for the frequency estimation in s";
  parameter Types.Time tRateLim = 1e-3 "Current slew limiter delay in s";
  parameter Types.Time tG "Current commands filter in s";

  // Frequency support
  parameter Types.AngularVelocityPu OmegaMaxPu "Maximum frequency before disconnection in pu (base omegaNom)";
  parameter Types.AngularVelocityPu OmegaMinPu "Minimum frequency before start of disconnections in pu (base omegaNom)";
  parameter Types.AngularVelocityPu pOmegaPu "Additional frequency drop compared that leads to full trip of units in pu (base omegaNom)";
  parameter Types.AngularVelocityPu OmegaDeadBandPu "Deadband of the overfrequency contribution in pu (base omegaNom)";

  // Voltage support
  parameter Types.VoltageModulePu UMaxPu "Maximum voltage over which the unit is disconnected in pu (base UNom)";
  parameter Types.VoltageModulePu US1 "Lower voltage limit of deadband in pu (base UNom)";
  parameter Types.VoltageModulePu US2 "Higher voltage limit of deadband in pu (base UNom)";
  parameter Real kRCI "Slope of reactive current increase for low voltages";
  parameter Real kRCA "Slope of reactive current decrease for high voltages";
  parameter Real m "Current injection just outside of lower deadband in pu (base IMaxPu)";
  parameter Real n "Current injection just outside of lower deadband in pu (base IMaxPu)";

  // Low voltage ride through
  parameter Types.VoltageModulePu ULVRTArmingPu "Voltage threshold under which the automaton is activated after tLVRTMax in pu (base UNom)";
  parameter Types.VoltageModulePu ULVRTIntPu "Voltage threshold under which the automaton is activated after tLVRTMin in pu (base UNom)";
  parameter Types.VoltageModulePu ULVRTMinPu "Voltage threshold under which the automaton is activated instantaneously in pu (base UNom)";
  parameter Types.Time tLVRTMin "Time delay of trip for severe voltage dips";
  parameter Types.Time tLVRTInt "Time delay of trip for intermediate voltage dips";
  parameter Types.Time tLVRTMax "Time delay of trip for small voltage dips";
  parameter Types.VoltageModulePu UPLLFreezePu "PLL freeze voltage threshold (in pu)";
  parameter Real IqMinPu = 0 "Minimum reactive current command in pu (base UNom, SNom)";
  parameter Types.VoltageModulePu UQPrioPu "Voltage under which priority is given to reactive current injection in pu (base UNom)";
  parameter Real IpSlewMaxPu "Active current slew limit (both up and down) in pu (base UNom, SNom)";

  // Parameters of the partial tripping curves
  parameter Types.PerUnit LVRTc;
  parameter Types.PerUnit LVRTd;
  parameter Types.PerUnit LVRTe;
  parameter Types.PerUnit LVRTf;
  parameter Types.Time tLVRTMaxSlowRecovery;

  // Initial values
  parameter Types.PerUnit P0Pu "Start value of active power at terminal in pu (receptor convention) (base SnRef)";
  parameter Types.PerUnit Q0Pu "Start value of reactive power at terminal in pu (receptor convention) (base SnRef)";
  parameter Types.PerUnit U0Pu "Start value of voltage magnitude at terminal in pu (base UNom)";
  parameter Types.Angle UPhase0 "Start value of voltage phase angle at terminal in rad";

  Types.PerUnit partialTrippingRatio(start = 1) "Coefficient for partial tripping of generators, equals 1 if no trips, 0 if all units are tripped";
  Dynawo.Electrical.Sources.InjectorIDQPLL injector(Id0Pu = Id0Pu, Iq0Pu = Iq0Pu, P0Pu = P0Pu, Q0Pu = Q0Pu, SNom = SNom, U0Pu = U0Pu, UPhase0 = UPhase0, i0Pu = i0Pu, s0Pu = s0Pu, u0Pu = u0Pu) annotation(
    Placement(visible = true, transformation(origin = {240, -6}, extent = {{10, -10}, {-10, 10}}, rotation = 180)));
  Dynawo.Connectors.ACPower terminal(V(re(start = u0Pu.re), im(start = u0Pu.im)), i(re(start = i0Pu.re), im(start = i0Pu.im))) annotation(
    Placement(visible = true, transformation(origin = {306, 2}, extent = {{10, -10}, {-10, 10}}, rotation = 0), iconTransformation(origin = {100, 8.88178e-16}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Dynawo.Electrical.Controls.PLL.PLLFreeze PLLFreeze(OmegaMaxPu = OmegaMaxPu, OmegaMinPu = OmegaMinPu, u0Pu = u0Pu) annotation(
    Placement(visible = true, transformation(origin = {50, -8}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Continuous.FirstOrder UFilter(T = tFilterU, y_start = U0Pu) annotation(
    Placement(visible = true, transformation(origin = {-130, -50}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Logical.GreaterThreshold Vfreeze(threshold = UPLLFreezePu) annotation(
    Placement(visible = true, transformation(origin = {-50, -10}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Dynawo.Electrical.InverterBasedGeneration.AggregatedIBG.LVRT lvrt(ULVRTArmingPu = ULVRTArmingPu, ULVRTIntPu = ULVRTIntPu, ULVRTMinPu = ULVRTMinPu, tLVRTMin = tLVRTMin, tLVRTInt=tLVRTInt, tLVRTMax = tLVRTMax, c = LVRTc, d = LVRTd, e = LVRTe, f = LVRTf, tMaxSlowRecovery = tLVRTMaxSlowRecovery) annotation(
    Placement(visible = true, transformation(origin = {-50, -90}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealInput PextPu(start = -P0Pu*SystemBase.SnRef/SNom) "Available power from the DC source in pu (base SNom)" annotation(
    Placement(visible = true, transformation(origin = {-94, -140}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {-110, -102}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
  Modelica.Blocks.Math.Division division annotation(
    Placement(visible = true, transformation(origin = {6, -160}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Nonlinear.VariableLimiter iPLimiter annotation(
    Placement(visible = true, transformation(origin = {60, -160}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Math.Max max annotation(
    Placement(visible = true, transformation(origin = {-70, -166}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Sources.Constant constant1(k = 0.01) annotation(
    Placement(visible = true, transformation(origin = {-96, -172}, extent = {{-4, -4}, {4, 4}}, rotation = 0)));
  Modelica.Blocks.Logical.GreaterThreshold currentPriority(threshold = UQPrioPu) annotation(
    Placement(visible = true, transformation(origin = {-90, -220}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Dynawo.Electrical.InverterBasedGeneration.GenericIBG.LimitUpdating limitUpdating(IMaxPu = IMaxPu) annotation(
    Placement(visible = true, transformation(origin = {60, -220}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
  Dynawo.Electrical.InverterBasedGeneration.GenericIBG.VoltageSupport voltageSupport(IMaxPu = IMaxPu, US1 = US1, US2 = US2, kRCA = kRCA, kRCI = kRCI, m = m, n = n) annotation(
    Placement(visible = true, transformation(origin = {-90, -274}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealInput iQrefPu(start = IqRef0Pu) annotation(
    Placement(visible = true, transformation(origin = {-94, -304}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {-110, -102}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
  Modelica.Blocks.Math.Add add annotation(
    Placement(visible = true, transformation(origin = {-50, -280}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Nonlinear.VariableLimiter iQLimiter annotation(
    Placement(visible = true, transformation(origin = {60, -280}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Continuous.FirstOrder iQcmdFirstOrder(T = tG, y_start = Iq0Pu) annotation(
    Placement(visible = true, transformation(origin = {110, -280}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Continuous.FirstOrder omegaFilter(T = tFilterOmega, y_start = SystemBase.omegaRef0Pu) annotation(
    Placement(visible = true, transformation(origin = {-90, -344}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealInput omegaRefPu(start = SystemBase.omegaRef0Pu) annotation(
    Placement(visible = true, transformation(origin = {8, -2}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {-110, -102}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
  Dynawo.Electrical.InverterBasedGeneration.AggregatedIBG.FrequencyProtection frequencyProtection(OmegaMaxPu = OmegaMaxPu, OmegaMinPu = OmegaMinPu, p = pOmegaPu) annotation(
    Placement(visible = true, transformation(origin = {-48, -344}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Dynawo.Electrical.InverterBasedGeneration.GenericIBG.OverfrequencySupport overfrequencySupport(OmegaDeadBandPu = OmegaDeadBandPu, OmegaMaxPu = OmegaMaxPu) annotation(
    Placement(visible = true, transformation(origin = {-48, -378}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Math.Add add2(k2 = -1) annotation(
    Placement(visible = true, transformation(origin = {-26, -146}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Dynawo.Electrical.InverterBasedGeneration.GenericIBG.OverVoltageProtection overVoltageProtection(UMaxPu = UMaxPu) annotation(
    Placement(visible = true, transformation(origin = {-50, -50}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Sources.RealExpression Pext(y = PextPu) annotation(
    Placement(visible = true, transformation(origin = {-90, -386}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Dynawo.NonElectrical.Blocks.NonLinear.StandAloneRampRateLimiter iPSlewLimit(DuMax = IpSlewMaxPu, DuMin = -IpSlewMaxPu, Y0 = Id0Pu, tS = tRateLim) annotation(
    Placement(visible = true, transformation(origin = {150, -160}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Continuous.FirstOrder iPcmdFirstOrder(T = tG, y_start = Id0Pu) annotation(
    Placement(visible = true, transformation(origin = {110, -160}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Math.Gain gain(k = -1) annotation(
    Placement(visible = true, transformation(origin = {150, -280}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));

  model LVRT "Low voltage ride through"
    import Dynawo.NonElectrical.Logs.Timeline;
    import Dynawo.NonElectrical.Logs.TimelineKeys;
    import Modelica.Constants;
    parameter Types.VoltageModulePu ULVRTArmingPu "Voltage threshold under which the automaton is activated after tLVRTMax in pu (base UNom)";
    parameter Types.VoltageModulePu ULVRTIntPu "Voltage threshold under which the automaton is activated after t1 in pu (base UNom)";
    parameter Types.VoltageModulePu ULVRTMinPu "Voltage threshold under which the automaton is activated instantaneously in pu (base UNom)";
    parameter Types.Time tLVRTMin "Time delay of trip for severe voltage dips in s";
    parameter Types.Time tLVRTInt "Time delay of trip for intermediate voltage dips in s";
    parameter Types.Time tLVRTMax "Time delay of trip for small voltage dips in s";
    parameter Types.Time tFilter = 1e-2 "Filter time constant for computation of UMin1Pu, UMinIntPu, and tMaxRecovery in s";
    // Parameters of the partial tripping curves
    parameter Types.PerUnit c;
    parameter Types.PerUnit d;
    parameter Types.PerUnit e;
    parameter Types.PerUnit f;
    parameter Types.Time tMaxSlowRecovery "Maximum time during which the voltage can stay below the LVRT curve during [tLVRTInt, tLVRTMax] before all IBGs are disconnected in s. (Replace parameter u in original model)";
    Connectors.BPin switchOffSignal (value (start = false)) "Switch off message for the generator";
    Modelica.Blocks.Interfaces.RealInput UMonitoredPu "Monitored voltage in pu (base UNom)" annotation(
    Placement(visible = true, transformation(origin = {-120, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-120, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
    Types.PerUnit fLVRT (start = 1) "Global partial tripping coefficient, equals to 1 if no trip, 0 if fully tripped";
    Types.PerUnit f1 (start = 1) "Partial tripping coefficient for trips in period [0, tLVRTInt], equals to 1 if no trip, 0 if fully tripped";
    Types.PerUnit f2 (start = 1) "Partial tripping coefficient for trips in period [tLVRTMin, tLVRTInt], equals to 1 if no trip, 0 if fully tripped";
    Types.PerUnit f3 (start = 1) "Partial tripping coefficient for trips in period [tLVRTInt, tLVRTMax], equals to 1 if no trip, 0 if fully tripped";
    Types.VoltageModulePu UMin1Pu (start = 1) "Minimum voltage in period [0, tLVRTMin] in pu (base UNom)";
    Types.VoltageModulePu UMinIntPu (start = 1) "Minimum voltage in period [tLVRTMin, tLVRTInt] in pu (base UNom)";
    Types.Time tMaxRecovery (start = 0) "Maximum 'single-block' duration for which the protection has been armed in s";
  protected
    Types.Time tThresholdReached (start = Constants.inf) "Time when the threshold was reached";
  equation
    // Arming
    when UMonitoredPu <= ULVRTArmingPu and not(pre(switchOffSignal.value)) then
      tThresholdReached = time;
      Timeline.logEvent1(TimelineKeys.LVRTArming);
    elsewhen UMonitoredPu > ULVRTArmingPu and pre(tThresholdReached) <> Constants.inf and not(pre(switchOffSignal.value)) then
      tThresholdReached = Constants.inf;
      Timeline.logEvent1(TimelineKeys.LVRTDisarming);
    end when;

    // Computation of minimum voltages
    if switchOffSignal.value == true or tThresholdReached == Constants.inf then  // Not armed or tripped
      der(UMin1Pu) = 0;
      der(UMinIntPu) = 0;
    elseif time - tThresholdReached < tLVRTMin then
      UMin1Pu + tFilter * der(UMin1Pu) = if UMonitoredPu < UMin1Pu then UMonitoredPu else UMin1Pu;  // Relaxed version of UMin1Pu = min(UMin1Pu, UMonitoredPu)
      der(UMinIntPu) = 0;
    elseif time - tThresholdReached < tLVRTInt then
      UMinIntPu + tFilter * der(UMinIntPu) = if UMonitoredPu < UMinIntPu then UMonitoredPu else UMinIntPu;  // Relaxed version of UMinIntPu = min(UMinIntPu, UMonitoredPu)
      der(UMin1Pu) = 0;
    else
      der(UMin1Pu) = 0;
      der(UMinIntPu) = 0;
    end if;

    // Computation of tMaxRecovery
    if switchOffSignal.value == true or tThresholdReached == Constants.inf then  // Not armed or tripped
      der(tMaxRecovery) = 0;
    else
      tMaxRecovery + tFilter * der(tMaxRecovery) = if (time - tThresholdReached) > tMaxRecovery then (time - tThresholdReached) else tMaxRecovery;
    end if;

    // Partial trips
    if UMin1Pu > ULVRTMinPu then
      f1 = 1;
    elseif UMin1Pu > d*ULVRTMinPu then
      f1 = c * (UMin1Pu - d*ULVRTMinPu) / (ULVRTMinPu - d*ULVRTMinPu);
    else
      f1 = 0;
    end if;

    if UMinIntPu > ULVRTIntPu then
      f2 = 1;
    elseif UMinIntPu > f*ULVRTIntPu then
      f2 = e * (UMinIntPu - f*ULVRTIntPu) / (ULVRTIntPu - f*ULVRTIntPu);
    else
      f2 = 0;
    end if;

    if tMaxRecovery < tLVRTInt then
      f3 = 1;
    elseif tMaxRecovery < tLVRTInt + tMaxSlowRecovery and time < tLVRTMax then
      f3 = (tLVRTInt + tMaxSlowRecovery - tMaxRecovery) / tMaxSlowRecovery;
    else
      f3 = 0;
    end if;

    fLVRT = f1 * f2 * f3;

    when fLVRT < 0.001 then
      switchOffSignal.value = true;
      Timeline.logEvent1(TimelineKeys.LVRTTripped);
    end when;
  end LVRT;

  model FrequencyProtection
    parameter Types.AngularVelocityPu OmegaMaxPu "Maximum frequency before disconnection in pu (base omegaNom)";
    parameter Types.AngularVelocityPu OmegaMinPu "Minimum frequency before start of disconnections in pu (base omegaNom)";
    parameter Types.AngularVelocityPu p "Additional frequency drop compared that leads to full trip of units in pu (base omegaNom)";
    parameter Types.Time tFilter = 1e-2 "Filter time constant for computation of MinOmegaPu in s";
    Connectors.BPin switchOffSignal(value(start = false)) "Switch off message for the generator";
    Modelica.Blocks.Interfaces.RealInput omegaPu annotation(
      Placement(visible = true, transformation(origin = {-120, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-120, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
    Types.AngularVelocityPu MinOmegaPu (start = 1) "Minimum measured frequency in pu (base omegaNom)";
    Types.PerUnit fFrequency (start = 1) "Partial tripping coefficient, equals to 1 if no trip, 0 if fully tripped";
  equation
    when omegaPu > OmegaMaxPu then
      switchOffSignal.value = true;
    elsewhen omegaPu < OmegaMinPu - p then
      switchOffSignal.value = true;
    end when;
    MinOmegaPu + tFilter * der(MinOmegaPu) = if omegaPu < MinOmegaPu then omegaPu else MinOmegaPu;
    if MinOmegaPu > OmegaMinPu then
      fFrequency = 1;
    elseif MinOmegaPu > OmegaMinPu - p then
      fFrequency = (OmegaMinPu - MinOmegaPu) / p;
    else
      fFrequency = 0;
    end if;
  end FrequencyProtection;

  Modelica.Blocks.Math.Product IpPartialTripping annotation(
    Placement(visible = true, transformation(origin = {190, -166}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Math.Product IqPartialTripping annotation(
    Placement(visible = true, transformation(origin = {190, -274}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Sources.RealExpression PartialTripping1(y = partialTrippingRatio)  annotation(
    Placement(visible = true, transformation(origin = {150, -190}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Sources.RealExpression PartialTripping2(y = partialTrippingRatio)  annotation(
    Placement(visible = true, transformation(origin = {150, -250}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Sources.Constant const(k = 0) annotation(
    Placement(visible = true, transformation(origin = {40, -168}, extent = {{-4, -4}, {4, 4}}, rotation = 0)));
  Modelica.Blocks.Sources.Constant constant2(k = IqMinPu) annotation(
    Placement(visible = true, transformation(origin = {36, -288}, extent = {{-4, -4}, {4, 4}}, rotation = 0)));
  Modelica.Blocks.Continuous.FirstOrder iQLimitFilter(T = 0.01, y_start = Iq0Pu) annotation(
    Placement(visible = true, transformation(origin = {79, -257}, extent = {{-3, -3}, {3, 3}}, rotation = 90)));
  Modelica.Blocks.Continuous.FirstOrder iDLimitFilter(T = 0.01, y_start = Id0Pu)  annotation(
    Placement(visible = true, transformation(origin = {79, -183}, extent = {{-3, -3}, {3, 3}}, rotation = -90)));
protected
  parameter Types.ComplexPerUnit u0Pu "Start value of complex voltage at terminal in pu (base UNom)";
  parameter Types.ComplexPerUnit s0Pu "Start value of complex apparent power at terminal in pu (base SnRef) (generator convention)";
  parameter Types.ComplexPerUnit i0Pu "Start value of complex current at terminal in pu (base UNom, SnRef) (generator convention)";
  parameter Types.PerUnit Id0Pu "Start value of d-axs current at injector in pu (base UNom, SNom) (generator convention)";
  parameter Types.PerUnit Iq0Pu "Start value of q-axis current at injector in pu (base UNom, SNom) (generator convention)";
  parameter Types.PerUnit IqRef0Pu "Start value of the reference q-axis current at injector in pu (base UNom, SNom) (generator convention)";
equation
  der(partialTrippingRatio)*1e-3 = lvrt.fLVRT*frequencyProtection.fFrequency - partialTrippingRatio;
// Continuous tripping to help KINSOL
  when partialTrippingRatio < 0.99 then
    Timeline.logEvent1(TimelineKeys.PartialTripping);
  end when;
  injector.switchOffSignal1.value = lvrt.switchOffSignal.value;
  injector.switchOffSignal2.value = frequencyProtection.switchOffSignal.value;
  injector.switchOffSignal3.value = overVoltageProtection.switchOffSignal.value;
  connect(injector.terminal, terminal) annotation(
    Line(points = {{251.5, 1.9}, {305.5, 1.9}}, color = {0, 0, 255}));
  connect(injector.UPu, UFilter.u) annotation(
    Line(points = {{251.5, -14}, {260, -14}, {260, 38}, {-152, 38}, {-152, -49}, {-142, -49}, {-142, -50}}, color = {0, 0, 127}));
  connect(UFilter.y, Vfreeze.u) annotation(
    Line(points = {{-118, -50}, {-110, -50}, {-110, -10}, {-62, -10}}, color = {0, 0, 127}));
  connect(UFilter.y, lvrt.UMonitoredPu) annotation(
    Line(points = {{-118, -50}, {-110, -50}, {-110, -90}, {-62, -90}}, color = {0, 0, 127}));
  connect(injector.uPu, PLLFreeze.uPu) annotation(
    Line(points = {{251.5, -3}, {254, -3}, {254, 32}, {34, 32}, {34, -2}, {39, -2}}, color = {85, 170, 255}));
  connect(division.y, iPLimiter.u) annotation(
    Line(points = {{17, -160}, {48, -160}}, color = {0, 0, 127}));
  connect(Vfreeze.y, PLLFreeze.freeze) annotation(
    Line(points = {{-39, -10}, {-1, -10}, {-1, -16}, {39, -16}}, color = {255, 0, 255}));
  connect(UFilter.y, max.u1) annotation(
    Line(points = {{-118, -50}, {-110, -50}, {-110, -160}, {-82, -160}}, color = {0, 0, 127}));
  connect(constant1.y, max.u2) annotation(
    Line(points = {{-91.6, -172}, {-81.6, -172}}, color = {0, 0, 127}));
  connect(max.y, division.u2) annotation(
    Line(points = {{-59, -166}, {-6, -166}}, color = {0, 0, 127}));
  connect(UFilter.y, currentPriority.u) annotation(
    Line(points = {{-118, -50}, {-110, -50}, {-110, -220}, {-102, -220}}, color = {0, 0, 127}));
  connect(UFilter.y, voltageSupport.Um) annotation(
    Line(points = {{-118, -50}, {-110, -50}, {-110, -274}, {-101, -274}}, color = {0, 0, 127}));
  connect(voltageSupport.IqSupPu, add.u1) annotation(
    Line(points = {{-79.2, -274.2}, {-62.2, -274.2}}, color = {0, 0, 127}));
  connect(iQrefPu, add.u2) annotation(
    Line(points = {{-94, -304}, {-70, -304}, {-70, -286}, {-62, -286}}, color = {0, 0, 127}));
  connect(add.y, iQLimiter.u) annotation(
    Line(points = {{-39, -280}, {48, -280}}, color = {0, 0, 127}));
  connect(iQLimiter.y, iQcmdFirstOrder.u) annotation(
    Line(points = {{71, -280}, {98, -280}}, color = {0, 0, 127}));
  connect(PLLFreeze.omegaPLLPu, omegaFilter.u) annotation(
    Line(points = {{61, -3}, {70, -3}, {70, 10}, {-160, 10}, {-160, -344}, {-102, -344}}, color = {0, 0, 127}));
  connect(PextPu, add2.u1) annotation(
    Line(points = {{-94, -140}, {-38, -140}}, color = {0, 0, 127}));
  connect(add2.y, division.u1) annotation(
    Line(points = {{-15, -146}, {-11, -146}, {-11, -154}, {-6, -154}}, color = {0, 0, 127}));
  connect(overfrequencySupport.deltaP, add2.u2) annotation(
    Line(points = {{-37, -378}, {-23, -378}, {-23, -188}, {-45, -188}, {-45, -152}, {-38, -152}}, color = {0, 0, 127}));
  connect(UFilter.y, overVoltageProtection.Um) annotation(
    Line(points = {{-118, -50}, {-62, -50}}, color = {0, 0, 127}));
  connect(Pext.y, overfrequencySupport.PextPu) annotation(
    Line(points = {{-79, -386}, {-60, -386}}, color = {0, 0, 127}));
  connect(omegaFilter.y, overfrequencySupport.omegaPu) annotation(
    Line(points = {{-78, -344}, {-70, -344}, {-70, -378}, {-60, -378}}, color = {0, 0, 127}));
  connect(omegaRefPu, PLLFreeze.omegaRefPu) annotation(
    Line(points = {{8, -2}, {22, -2}, {22, -8}, {40, -8}}, color = {0, 0, 127}));
  connect(omegaFilter.y, frequencyProtection.omegaPu) annotation(
    Line(points = {{-78, -344}, {-60, -344}}, color = {0, 0, 127}));
  connect(iPcmdFirstOrder.y, iPSlewLimit.u) annotation(
    Line(points = {{121, -160}, {138, -160}}, color = {0, 0, 127}));
  connect(iPLimiter.y, iPcmdFirstOrder.u) annotation(
    Line(points = {{72, -160}, {98, -160}}, color = {0, 0, 127}));
  connect(iQcmdFirstOrder.y, gain.u) annotation(
    Line(points = {{121, -280}, {137, -280}}, color = {0, 0, 127}));
  connect(PLLFreeze.cosPhi, injector.cosPhi) annotation(
    Line(points = {{62, -10}, {140, -10}, {140, -2}, {228, -2}}, color = {0, 0, 127}));
  connect(PLLFreeze.sinPhi, injector.sinPhi) annotation(
    Line(points = {{62, -14}, {142, -14}, {142, 2}, {228, 2}}, color = {0, 0, 127}));
  connect(iPSlewLimit.y, IpPartialTripping.u1) annotation(
    Line(points = {{162, -160}, {178, -160}}, color = {0, 0, 127}));
  connect(IpPartialTripping.y, injector.idPu) annotation(
    Line(points = {{201, -166}, {220, -166}, {220, -14}, {228, -14}}, color = {0, 0, 127}));
  connect(gain.y, IqPartialTripping.u2) annotation(
    Line(points = {{161, -280}, {177, -280}}, color = {0, 0, 127}));
  connect(IqPartialTripping.y, injector.iqPu) annotation(
    Line(points = {{201, -274}, {224, -274}, {224, -10}, {228, -10}}, color = {0, 0, 127}));
  connect(PartialTripping1.y, IpPartialTripping.u2) annotation(
    Line(points = {{162, -190}, {170, -190}, {170, -172}, {178, -172}}, color = {0, 0, 127}));
  connect(PartialTripping2.y, IqPartialTripping.u1) annotation(
    Line(points = {{161, -250}, {169, -250}, {169, -268}, {177, -268}}, color = {0, 0, 127}));
  connect(currentPriority.y, limitUpdating.PPriority) annotation(
    Line(points = {{-78, -220}, {0, -220}, {0, -200}, {86, -200}, {86, -220}, {72, -220}}, color = {255, 0, 255}));
  connect(const.y, iPLimiter.limit2) annotation(
    Line(points = {{44, -168}, {48, -168}}, color = {0, 0, 127}));
  connect(constant2.y, iQLimiter.limit2) annotation(
    Line(points = {{40, -288}, {48, -288}}, color = {0, 0, 127}));
  connect(iPLimiter.y, iDLimitFilter.u) annotation(
    Line(points = {{72, -160}, {79, -160}, {79, -179}}, color = {0, 0, 127}));
  connect(iQLimiter.y, iQLimitFilter.u) annotation(
    Line(points = {{72, -280}, {79, -280}, {79, -261}}, color = {0, 0, 127}));
  connect(iQLimitFilter.y, limitUpdating.IqCmdPu) annotation(
    Line(points = {{79, -254}, {79, -228}, {72, -228}}, color = {0, 0, 127}));
  connect(iDLimitFilter.y, limitUpdating.IpCmdPu) annotation(
    Line(points = {{79, -186}, {79, -212}, {72, -212}}, color = {0, 0, 127}));
  connect(limitUpdating.IpMaxPu, iPLimiter.limit1) annotation(
    Line(points = {{50, -214}, {34, -214}, {34, -152}, {48, -152}}, color = {0, 0, 127}));
  connect(limitUpdating.IqMaxPu, iQLimiter.limit1) annotation(
    Line(points = {{50, -226}, {34, -226}, {34, -272}, {48, -272}}, color = {0, 0, 127}));
  annotation(
    Documentation(preferredView = "diagram", info = "<html>
    <p> Aggregated model of inverter-based generation as defined in p28,74-76 of Gilles Chaspierre's PhD thesis 'Reduced-order modelling of active distribution networks for large-disturbance simulations'. Available: https://orbi.uliege.be/handle/2268/251602 </p></html>"),
    Diagram(coordinateSystem(extent = {{-180, 40}, {320, -400}})));
end AggregatedIBG;

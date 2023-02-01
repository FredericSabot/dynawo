within Dynawo.Electrical.InverterBasedGeneration;

model GenericIBG "Generic model of inverter-based generation (IBG)"
  import Dynawo;
  import Modelica;

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
  parameter Types.AngularVelocityPu OmegaMinPu "Minimum frequency before disconnection in pu (base omegaNom)";
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
  parameter Types.VoltageModulePu ULVRTArmingPu "Voltage threshold under which the automaton is activated after tLVRT2 in pu (base UNom)";
  parameter Types.VoltageModulePu ULVRTIntermediatePu "Voltage threshold under which the automaton is activated after tLVRT1 in pu (base UNom)";
  parameter Types.VoltageModulePu ULVRTMinPu "Voltage threshold under which the automaton is activated instantaneously in pu (base UNom)";
  parameter Types.Time tLVRT1 "Time delay of trip for severe voltage dips";
  parameter Types.Time tLVRT2 "Time delay of trip for small voltage dips";

  parameter Types.VoltageModulePu UPLLFreezePu "PLL freeze voltage threshold (in pu)";
  parameter Real IqMinPu = 0 "Minimum reactive current command in pu (base UNom, SNom)";
  parameter Types.VoltageModulePu UQPrioPu "Voltage under which priority is given to reactive current injection in pu (base UNom)";
  parameter Real IpSlewMaxPu "Active current slew limit (both up and down) in pu (base UNom, SNom)";

  // Initial values
  parameter Types.PerUnit P0Pu "Start value of active power at terminal in pu (receptor convention) (base SnRef)";
  parameter Types.PerUnit Q0Pu "Start value of reactive power at terminal in pu (receptor convention) (base SnRef)";
  parameter Types.PerUnit U0Pu "Start value of voltage magnitude at terminal in pu (base UNom)";
  parameter Types.Angle UPhase0 "Start value of voltage phase angle at terminal in rad";
protected
  parameter Types.ComplexPerUnit u0Pu "Start value of complex voltage at terminal in pu (base UNom)";
  parameter Types.ComplexPerUnit s0Pu "Start value of complex apparent power at terminal in pu (base SnRef) (generator convention)";
  parameter Types.ComplexPerUnit i0Pu "Start value of complex current at terminal in pu (base UNom, SnRef) (generator convention)";
  parameter Types.PerUnit Id0Pu "Start value of d-axs current at injector in pu (base UNom, SNom) (generator convention)";
  parameter Types.PerUnit Iq0Pu "Start value of q-axis current at injector in pu (base UNom, SNom) (generator convention)";
  parameter Types.PerUnit IqRef0Pu "Start value of the reference q-axis current at injector in pu (base UNom, SNom) (generator convention)";

public
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
  Dynawo.Electrical.InverterBasedGeneration.GenericIBG.LVRT lvrt(ULVRTArmingPu = ULVRTArmingPu, ULVRTIntermediatePu = ULVRTIntermediatePu, ULVRTMinPu = ULVRTMinPu, tLVRT1 = tLVRT1, tLVRT2 = tLVRT2) annotation(
    Placement(visible = true, transformation(origin = {-50, -90}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealInput PextPu(start = -P0Pu*SystemBase.SnRef/SNom) "Available power from the DC source in pu (base SNom)" annotation(
    Placement(visible = true, transformation(origin = {-94, -140}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {-110, -102}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
  Modelica.Blocks.Math.Division division annotation(
    Placement(visible = true, transformation(origin = {6, -160}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Nonlinear.VariableLimiter iPLimiter annotation(
    Placement(visible = true, transformation(origin = {60, -160}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Sources.Constant const(k = 0) annotation(
    Placement(visible = true, transformation(origin = {40, -168}, extent = {{-4, -4}, {4, 4}}, rotation = 0)));
  Modelica.Blocks.Math.Max max annotation(
    Placement(visible = true, transformation(origin = {-70, -166}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Sources.Constant constant1(k = 0.01) annotation(
    Placement(visible = true, transformation(origin = {-96, -172}, extent = {{-4, -4}, {4, 4}}, rotation = 0)));
  Modelica.Blocks.Logical.GreaterThreshold currentPriority(threshold = UQPrioPu) annotation(
    Placement(visible = true, transformation(origin = {-90, -224}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Dynawo.Electrical.InverterBasedGeneration.GenericIBG.LimitUpdating limitUpdating(IMaxPu = IMaxPu) annotation(
    Placement(visible = true, transformation(origin = {12, -224}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Dynawo.Electrical.InverterBasedGeneration.GenericIBG.VoltageSupport voltageSupport(IMaxPu = IMaxPu, US1 = US1, US2 = US2, kRCA = kRCA, kRCI = kRCI, m = m, n = n) annotation(
    Placement(visible = true, transformation(origin = {-90, -264}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealInput iQrefPu(start = IqRef0Pu) annotation(
    Placement(visible = true, transformation(origin = {-94, -294}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {-110, -102}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
  Modelica.Blocks.Math.Add add annotation(
    Placement(visible = true, transformation(origin = {-50, -270}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Nonlinear.VariableLimiter iQLimiter annotation(
    Placement(visible = true, transformation(origin = {60, -270}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Sources.Constant constant2(k = IqMinPu) annotation(
    Placement(visible = true, transformation(origin = {36, -278}, extent = {{-4, -4}, {4, 4}}, rotation = 0)));
  Modelica.Blocks.Continuous.FirstOrder iQcmdFirstOrder(T = tG, y_start = Iq0Pu) annotation(
    Placement(visible = true, transformation(origin = {110, -270}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Continuous.FirstOrder omegaFilter(T = tFilterOmega, y_start = SystemBase.omegaRef0Pu) annotation(
    Placement(visible = true, transformation(origin = {-90, -344}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealInput omegaRefPu(start = SystemBase.omegaRef0Pu) annotation(
    Placement(visible = true, transformation(origin = {8, -2}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {-110, -102}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
  Dynawo.Electrical.InverterBasedGeneration.GenericIBG.FrequencyProtection frequencyProtection(OmegaMaxPu = OmegaMaxPu, OmegaMinPu = OmegaMinPu) annotation(
    Placement(visible = true, transformation(origin = {-48, -344}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Dynawo.Electrical.InverterBasedGeneration.GenericIBG.OverfrequencySupport overfrequencySupport(OmegaDeadBandPu = OmegaDeadBandPu, OmegaMaxPu = OmegaMaxPu) annotation(
    Placement(visible = true, transformation(origin = {-48, -378}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Math.Add add2(k2 = -1) annotation(
    Placement(visible = true, transformation(origin = {-26, -146}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Dynawo.Electrical.InverterBasedGeneration.GenericIBG.OverVoltageProtection overVoltageProtection(UMaxPu = UMaxPu) annotation(
    Placement(visible = true, transformation(origin = {-50, -50}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Sources.RealExpression Pext(y = PextPu) annotation(
    Placement(visible = true, transformation(origin = {-90, -386}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Continuous.FirstOrder iPMaxFilter(T = 0.01, y_start = IMaxPu) annotation(
    Placement(visible = true, transformation(origin = {33, -193}, extent = {{-3, -3}, {3, 3}}, rotation = 90)));
  Modelica.Blocks.Continuous.FirstOrder iQMaxFilter(T = 0.01, y_start = IMaxPu)  annotation(
    Placement(visible = true, transformation(origin = {33, -249}, extent = {{-3, -3}, {3, 3}}, rotation = -90)));
  Dynawo.NonElectrical.Blocks.NonLinear.StandAloneRampRateLimiter iPSlewLimit(DuMax = IpSlewMaxPu, DuMin = -IpSlewMaxPu, Y0 = Id0Pu, tS = tRateLim) annotation(
    Placement(visible = true, transformation(origin = {150, -160}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Continuous.FirstOrder iPcmdFirstOrder(T = tG, y_start = Id0Pu) annotation(
    Placement(visible = true, transformation(origin = {110, -160}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Math.Gain gain(k = -1)  annotation(
    Placement(visible = true, transformation(origin = {150, -270}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));

  model LimitUpdating
    Types.CurrentModulePu IMaxPu "Maximum current of the injector in pu (base UNom, SNom)";
    Modelica.Blocks.Interfaces.RealInput IpCmd "Active current command in pu (base UNom, SNom)" annotation(
      Placement(visible = true, transformation(origin = {-120, 80}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-108, 80}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
    Modelica.Blocks.Interfaces.RealInput IqCmd "Reactive current command in pu (base UNom, SNom)" annotation(
      Placement(visible = true, transformation(origin = {-120, -80}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-100, -74}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
    Modelica.Blocks.Interfaces.BooleanInput Pflag "Priority to active power (true) or reactive power (false)" annotation(
      Placement(visible = true, transformation(origin = {-120, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-102, 2}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
    Modelica.Blocks.Interfaces.RealOutput IpLim annotation(
      Placement(visible = true, transformation(origin = {110, 60}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {108, 60}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Blocks.Interfaces.RealOutput IqLim annotation(
      Placement(visible = true, transformation(origin = {110, -60}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110, -60}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  protected
    Types.PerUnit IpLimPu = min(IpCmd, IMaxPu);
    Types.PerUnit IqLimPu = min(IpCmd, IMaxPu);
  equation
    if Pflag then
      IpLim = IMaxPu;
      IqLim = sqrt(IMaxPu^2 - IpLimPu^2);
    else
      IpLim = sqrt(IMaxPu^2 - IqLimPu^2);
      IqLim = IMaxPu;
    end if;
  end LimitUpdating;

  model LVRT "Low voltage ride through"
    import Dynawo.NonElectrical.Logs.Timeline;
    import Dynawo.NonElectrical.Logs.TimelineKeys;
    import Modelica.Constants;
    parameter Types.VoltageModulePu ULVRTArmingPu "Voltage threshold under which the automaton is activated after tLVRT2 in pu (base UNom)";
    parameter Types.VoltageModulePu ULVRTIntermediatePu "Voltage threshold under which the automaton is activated after t1 in pu (base UNom)";
    parameter Types.VoltageModulePu ULVRTMinPu "Voltage threshold under which the automaton is activated instantaneously in pu (base UNom)";
    parameter Types.Time tLVRT1 "Time delay of trip for severe voltage dips in s";
    parameter Types.Time tLVRT2 "Time delay of trip for small voltage dips in s";
    Connectors.BPin switchOffSignal (value (start = false)) "Switch off message for the generator";
    Modelica.Blocks.Interfaces.RealInput UMonitoredPu "Monitored voltage in pu (base UNom)" annotation(
    Placement(visible = true, transformation(origin = {-120, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-120, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
  protected
    Types.Time tThresholdReached (start = Constants.inf) "Time when the threshold was reached";

  equation
    // Voltage comparison with the minimum accepted value
    when UMonitoredPu <= ULVRTArmingPu and not(pre(switchOffSignal.value)) then
      tThresholdReached = time;
      Timeline.logEvent1(TimelineKeys.LVRTArming);
    elsewhen UMonitoredPu > ULVRTArmingPu and pre(tThresholdReached) <> Constants.inf and not(pre(switchOffSignal.value)) then
      tThresholdReached = Constants.inf;
      Timeline.logEvent1(TimelineKeys.LVRTDisarming);
    end when;

    // Delay before tripping the generator
    when UMonitoredPu < ULVRTMinPu then // No delay
      switchOffSignal.value = true;
      Timeline.logEvent1(TimelineKeys.LVRTTripped);
    elsewhen time - tThresholdReached >= tLVRT1 and UMonitoredPu < ULVRTIntermediatePu then
      switchOffSignal.value = true;
      Timeline.logEvent1(TimelineKeys.LVRTTripped);
    elsewhen UMonitoredPu >= ULVRTIntermediatePu and  time - tThresholdReached >= tLVRT1 + (tLVRT2-tLVRT1) * (UMonitoredPu - ULVRTMinPu)/(ULVRTArmingPu-ULVRTMinPu) then
      switchOffSignal.value = true;
      Timeline.logEvent1(TimelineKeys.LVRTTripped);
    end when;
  end LVRT;

  model VoltageSupport
    parameter Types.VoltageModulePu US1 "Lower voltage limit of deadband in pu (base UNom)";
    parameter Types.VoltageModulePu US2 "Higher voltage limit of deadband in pu (base UNom)";
    parameter Real kRCI "Slope of reactive current increase for low voltages";
    parameter Real kRCA "Slope of reactive current decrease for high voltages";
    parameter Real m "Current injection just outside of lower deadband in pu (base IMaxPu)";
    parameter Real n "Current injection just outside of lower deadband in pu (base IMaxPu)";
    parameter Types.CurrentModulePu IMaxPu "Maximum current of the injector in pu (base UNom, SNom)";
    Modelica.Blocks.Interfaces.RealInput Um annotation(
      Placement(visible = true, transformation(origin = {-120, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-110, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
    Modelica.Blocks.Interfaces.RealOutput IqSupPu annotation(
      Placement(visible = true, transformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {108, -2}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  equation
    if Um < US1 then
      IqSupPu = m*IMaxPu + kRCI*(US1 - Um);
    elseif Um < US2 then
      IqSupPu = 0;
    else
      IqSupPu = -n*IMaxPu + kRCA*(Um - US2);
    end if;
  end VoltageSupport;

  model FrequencyProtection
    parameter Types.AngularVelocityPu OmegaMaxPu "Maximum frequency before disconnection in pu (base omegaNom)";
    parameter Types.AngularVelocityPu OmegaMinPu "Minimum frequency before disconnection in pu (base omegaNom)";
    Connectors.BPin switchOffSignal(value(start = false)) "Switch off message for the generator";
    Modelica.Blocks.Interfaces.RealInput omegaPu annotation(
      Placement(visible = true, transformation(origin = {-120, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-120, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
  equation
    when omegaPu > OmegaMaxPu then
      switchOffSignal.value = true;
    elsewhen omegaPu < OmegaMinPu then
      switchOffSignal.value = true;
    end when;
  end FrequencyProtection;

  model OverfrequencySupport
    parameter Types.AngularVelocityPu OmegaMaxPu "Maximum frequency before disconnection in pu (base omegaNom)";
    parameter Types.AngularVelocityPu OmegaDeadBandPu "Deadband of the overfrequency contribution in pu (base omegaNom)";
    Modelica.Blocks.Interfaces.RealInput omegaPu annotation(
      Placement(visible = true, transformation(origin = {-120, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-120, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
    Modelica.Blocks.Interfaces.RealOutput deltaP annotation(
      Placement(visible = true, transformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Blocks.Interfaces.RealInput PextPu annotation(
      Placement(visible = true, transformation(origin = {-120, -80}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-120, -80}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
  equation
    if omegaPu < OmegaDeadBandPu then
      deltaP = 0;
    elseif omegaPu < OmegaMaxPu then
      deltaP = PextPu*(OmegaMaxPu - omegaPu);
    else
      deltaP = PextPu;
    end if;
  end OverfrequencySupport;

  model OverVoltageProtection
    Connectors.BPin switchOffSignal(value(start = false)) "Switch off message for the generator";
    Modelica.Blocks.Interfaces.RealInput Um annotation(
      Placement(visible = true, transformation(origin = {-120, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-120, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
    parameter Types.VoltageModulePu UMaxPu "Maximum voltage over which the unit is disconnected in pu (base UNom)";
  equation
    when Um > UMaxPu then
      switchOffSignal.value = true;
    end when;
  end OverVoltageProtection;

equation
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
  connect(const.y, iPLimiter.limit2) annotation(
    Line(points = {{44.4, -168}, {48.4, -168}}, color = {0, 0, 127}));
  connect(Vfreeze.y, PLLFreeze.freeze) annotation(
    Line(points = {{-39, -10}, {-1, -10}, {-1, -16}, {39, -16}}, color = {255, 0, 255}));
  connect(UFilter.y, max.u1) annotation(
    Line(points = {{-118, -50}, {-110, -50}, {-110, -160}, {-82, -160}}, color = {0, 0, 127}));
  connect(constant1.y, max.u2) annotation(
    Line(points = {{-91.6, -172}, {-81.6, -172}}, color = {0, 0, 127}));
  connect(max.y, division.u2) annotation(
    Line(points = {{-59, -166}, {-6, -166}}, color = {0, 0, 127}));
  connect(UFilter.y, currentPriority.u) annotation(
    Line(points = {{-118, -50}, {-110, -50}, {-110, -224}, {-102, -224}}, color = {0, 0, 127}));
  connect(UFilter.y, voltageSupport.Um) annotation(
    Line(points = {{-118, -50}, {-110, -50}, {-110, -264}, {-101, -264}}, color = {0, 0, 127}));
  connect(voltageSupport.IqSupPu, add.u1) annotation(
    Line(points = {{-79.2, -264.2}, {-62.2, -264.2}}, color = {0, 0, 127}));
  connect(iQrefPu, add.u2) annotation(
    Line(points = {{-94, -294}, {-70, -294}, {-70, -276}, {-62, -276}}, color = {0, 0, 127}));
  connect(add.y, iQLimiter.u) annotation(
    Line(points = {{-39, -270}, {48, -270}}, color = {0, 0, 127}));
  connect(constant2.y, iQLimiter.limit2) annotation(
    Line(points = {{40.4, -278}, {48.4, -278}}, color = {0, 0, 127}));
  connect(iQLimiter.y, iQcmdFirstOrder.u) annotation(
    Line(points = {{71, -270}, {98, -270}}, color = {0, 0, 127}));
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
  connect(iPMaxFilter.y, iPLimiter.limit1) annotation(
    Line(points = {{33, -190}, {33, -152}, {48, -152}}, color = {0, 0, 127}));
  connect(iQMaxFilter.y, iQLimiter.limit1) annotation(
    Line(points = {{34, -252}, {34, -262}, {48, -262}}, color = {0, 0, 127}));
  connect(iPcmdFirstOrder.y, iPSlewLimit.u) annotation(
    Line(points = {{121, -160}, {138, -160}}, color = {0, 0, 127}));
  connect(iPLimiter.y, iPcmdFirstOrder.u) annotation(
    Line(points = {{72, -160}, {98, -160}}, color = {0, 0, 127}));
  connect(iQcmdFirstOrder.y, gain.u) annotation(
    Line(points = {{122, -270}, {138, -270}}, color = {0, 0, 127}));
  connect(currentPriority.y, limitUpdating.Pflag) annotation(
    Line(points = {{-78, -224}, {2, -224}}, color = {255, 0, 255}));
  connect(iPLimiter.y, limitUpdating.IpCmd) annotation(
    Line(points = {{72, -160}, {80, -160}, {80, -208}, {-10, -208}, {-10, -216}, {2, -216}}, color = {0, 0, 127}));
  connect(iQLimiter.y, limitUpdating.IqCmd) annotation(
    Line(points = {{72, -270}, {80, -270}, {80, -240}, {-10, -240}, {-10, -232}, {2, -232}}, color = {0, 0, 127}));
  connect(limitUpdating.IpLim, iPMaxFilter.u) annotation(
    Line(points = {{22, -218}, {33, -218}, {33, -197}}, color = {0, 0, 127}));
  connect(limitUpdating.IqLim, iQMaxFilter.u) annotation(
    Line(points = {{24, -230}, {34, -230}, {34, -246}}, color = {0, 0, 127}));
  connect(PLLFreeze.cosPhi, injector.cosPhi) annotation(
    Line(points = {{62, -10}, {140, -10}, {140, -2}, {228, -2}}, color = {0, 0, 127}));
  connect(PLLFreeze.sinPhi, injector.sinPhi) annotation(
    Line(points = {{62, -14}, {142, -14}, {142, 2}, {228, 2}}, color = {0, 0, 127}));
  connect(iPSlewLimit.y, injector.idPu) annotation(
    Line(points = {{162, -160}, {200, -160}, {200, -14}, {228, -14}}, color = {0, 0, 127}));
  connect(gain.y, injector.iqPu) annotation(
    Line(points = {{162, -270}, {202, -270}, {202, -10}, {228, -10}}, color = {0, 0, 127}));
  annotation(
    Documentation(preferredView = "diagram", info = "<html>
    <p> Generic model of inverter-based generation as defined in p28 of Gilles Chaspierre's PhD thesis 'Reduced-order modelling of active distribution networks for large-disturbance simulations'. Available: https://orbi.uliege.be/handle/2268/251602 </p></html>"),
    Diagram(coordinateSystem(extent = {{-180, 40}, {320, -400}})));
end GenericIBG;

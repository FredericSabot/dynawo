within Dynawo.Electrical.InverterBasedGeneration;

model GenericIBG "Generic model of inverter-based generation (IBG)"
  import Dynawo;
  import Modelica;

  parameter Real SNom "Nominal apparent power of the injector (in MVA)";
  parameter Real Inom "Nominal current of the injector";
  parameter Real Tm "Voltage measurement first order time constant";
  parameter Real VPLLPu "PLL freeze voltage threshold (in pu)";
  parameter Real iQminPu "Minimum reactive current command (base UNom, SnRef)";
  parameter Real iQmaxPu "Maximum reactive current command (base UNom, SnRef)";
  parameter Real VRCIPu "Voltage under which priority is given to reactive current injection";
  parameter Real Tg "Current slew limiter first order time constant";
  parameter Real iPSlewMaxPu "Current slew limit (base UNom, SnRef)";

  parameter Real omegaMaxPu "Maximum frequency before disconnection (in pu)";
  parameter Real omegaMinPu "Minimum frequency before disconnection (in pu)";
  parameter Real Tfltr "First order time constant for the frequency estimation";

  // Voltage support
  parameter Real VS1 "Lower voltage limit of deadband";
  parameter Real VS2 "Higher voltage limit of deadband";

  parameter Real kRCI "Slope of reactive current increase for low voltages";
  parameter Real kRCA "Slope of reactive current decrease for high voltages";

  parameter Real m "Current injection just outside of lower deadband (in pu of Inom)";
  parameter Real n "Current injection just outside of lower deadband (in pu of Inom)";

  parameter Types.PerUnit P0Pu "Start value of active power at terminal in pu (receptor convention) (base SnRef)";
  parameter Types.PerUnit Q0Pu "Start value of reactive power at terminal in pu (receptor convention) (base SnRef)";
  parameter Types.PerUnit U0Pu "Start value of voltage magnitude at terminal in pu (bae UNom)";
  parameter Types.Angle UPhase0 "Start value of voltage phase angle at terminal in rad";
  Dynawo.Electrical.Sources.InjectorIDQ injector(Id0Pu = Id0Pu, Iq0Pu = Iq0Pu, P0Pu = P0Pu, Q0Pu = Q0Pu, SNom = SNom, U0Pu = U0Pu, UPhase0 = UPhase0, i0Pu = i0Pu, s0Pu = s0Pu, u0Pu = u0Pu) annotation(
    Placement(visible = true, transformation(origin = {240, -6}, extent = {{10, -10}, {-10, 10}}, rotation = 180)));
  Dynawo.Connectors.ACPower terminal(V(re(start = u0Pu.re), im(start = u0Pu.im)), i(re(start = i0Pu.re), im(start = i0Pu.im))) annotation(
    Placement(visible = true, transformation(origin = {306, 2}, extent = {{10, -10}, {-10, 10}}, rotation = 0), iconTransformation(origin = {100, 8.88178e-16}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Dynawo.Electrical.Controls.PLL.PLLFreeze PLLFreeze(OmegaMaxPu = omegaMaxPu, OmegaMinPu = omegaMinPu, u0Pu = u0Pu)  annotation(
    Placement(visible = true, transformation(origin = {50, -8}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Continuous.FirstOrder firstOrder(T = Tm, y_start = U0Pu)  annotation(
    Placement(visible = true, transformation(origin = {-130, -50}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Logical.GreaterThreshold Vfreeze(threshold = VPLLPu)  annotation(
    Placement(visible = true, transformation(origin = {-50, -10}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Dynawo.Electrical.Controls.LVRT.LVRT lvrt annotation(
    Placement(visible = true, transformation(origin = {-50, -90}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealInput PextPu(start = - P0Pu * SystemBase.SnRef / SNom) "Available power from the DC source" annotation(
    Placement(visible = true, transformation(origin = {-94, -140}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {-110, -102}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
  Modelica.Blocks.Math.Division division annotation(
    Placement(visible = true, transformation(origin = {6, -160}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Nonlinear.VariableLimiter iPLimiter annotation(
    Placement(visible = true, transformation(origin = {60, -160}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Sources.Constant const(k = 0)  annotation(
    Placement(visible = true, transformation(origin = {40, -168}, extent = {{-4, -4}, {4, 4}}, rotation = 0)));
  Modelica.Blocks.Nonlinear.Limiter iPSlewLimiter(limitsAtInit = true, uMax = Tg * iPSlewMaxPu, uMin = 0)  annotation(
    Placement(visible = true, transformation(origin = {154, -160}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Math.Gain gain(k = 1 / Tg)  annotation(
    Placement(visible = true, transformation(origin = {190, -160}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Continuous.Integrator integrator(y_start = i0Pu.re * (SystemBase.SnRef / SNom))  annotation(
    Placement(visible = true, transformation(origin = {222, -160}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Math.Feedback feedback1 annotation(
    Placement(visible = true, transformation(origin = {118, -160}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.ComplexBlocks.ComplexMath.RealToComplex realToComplex annotation(
    Placement(visible = true, transformation(origin = {156, 4}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Dynawo.Electrical.Controls.WECC.Utilities.TransformRItoDQ transformRItoDQ annotation(
    Placement(visible = true, transformation(origin = {190, -8}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Math.Max max annotation(
    Placement(visible = true, transformation(origin = {-70, -166}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Sources.Constant constant1(k = 0.01) annotation(
    Placement(visible = true, transformation(origin = {-96, -172}, extent = {{-4, -4}, {4, 4}}, rotation = 0)));
  Modelica.Blocks.Logical.GreaterThreshold currentPriority(threshold = VRCIPu)  annotation(
    Placement(visible = true, transformation(origin = {-90, -224}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));

  model LimitUpdating

    parameter Real Inom "Nominal current of the injector";

    Modelica.Blocks.Interfaces.RealInput iPcmd annotation(
        Placement(visible = true, transformation(origin = {-120, 80}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-108, 80}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
    Modelica.Blocks.Interfaces.RealInput iQcmd annotation(
        Placement(visible = true, transformation(origin = {-120, -80}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-100, -74}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
    Modelica.Blocks.Interfaces.BooleanInput Pflag annotation(
        Placement(visible = true, transformation(origin = {-120, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-102, 2}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
    Modelica.Blocks.Interfaces.RealOutput iPlim annotation(
        Placement(visible = true, transformation(origin = {110, 60}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {108, 60}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Blocks.Interfaces.RealOutput iQlim annotation(
        Placement(visible = true, transformation(origin = {110, -60}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110, -60}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  equation

    if Pflag then
      iPlim = Inom;
      iQlim = sqrt(Inom^2 - iPcmd^2);
    else
      iPlim = sqrt(Inom^2 - iQcmd^2);
      iQlim = Inom;
    end if;

  end LimitUpdating;

  Dynawo.Electrical.InverterBasedGeneration.GenericIBG.LimitUpdating limitUpdating(Inom = Inom)  annotation(
    Placement(visible = true, transformation(origin = {12, -224}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));

  model VoltageSupport

    parameter Real VS1 "Lower voltage limit of deadband";
    parameter Real VS2 "Higher voltage limit of deadband";

    parameter Real kRCI "Slope of reactive current increase for low voltages";
    parameter Real kRCA "Slope of reactive current decrease for high voltages";

    parameter Real m "Current injection just outside of lower deadband (in pu of Inom)";
    parameter Real n "Current injection just outside of lower deadband (in pu of Inom)";

    parameter Real Inom "Nominal current of injector";


    Modelica.Blocks.Interfaces.RealInput Vm annotation(
        Placement(visible = true, transformation(origin = {-120, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-110, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
    Modelica.Blocks.Interfaces.RealOutput iQsupPu annotation(
        Placement(visible = true, transformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {108, -2}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));

  equation

    if Vm < VS1 then
      iQsupPu = m * Inom + kRCI * (VS1 - Vm);
    elseif Vm < VS2 then
      iQsupPu = 0;
    else
      iQsupPu = -n * Inom + kRCA * (Vm - VS2);
    end if;

  end VoltageSupport;

  model FrequencyProtection

    Connectors.BPin switchOffSignal (value (start = false)) "Switch off message for the generator";
    Modelica.Blocks.Interfaces.RealInput omegaPu annotation(
      Placement(visible = true, transformation(origin = {-120, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-120, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));

    parameter Types.AngularVelocityPu OmegaMaxPu "Maximum frequency over which the unit is disconnected in pu (base omegaNom)";
    parameter Types.AngularVelocityPu OmegaMinPu "Minimum frequency over which the unit is disconnected in pu (base omegaNom)";

  equation

    when omegaPu > OmegaMaxPu then
      switchOffSignal.value = true;
    elsewhen omegaPu < OmegaMinPu then
      switchOffSignal.value = true;
    end when;

  end FrequencyProtection;

  Dynawo.Electrical.InverterBasedGeneration.GenericIBG.VoltageSupport voltageSupport(Inom = Inom, VS1 = VS1, VS2 = VS2, kRCA = kRCA, kRCI = kRCI, m = m, n = n)  annotation(
    Placement(visible = true, transformation(origin = {-90, -264}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealInput iQrefPu(start = iQref0Pu) annotation(
    Placement(visible = true, transformation(origin = {-94, -294}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {-110, -102}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
  Modelica.Blocks.Math.Add add annotation(
    Placement(visible = true, transformation(origin = {-50, -270}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Nonlinear.VariableLimiter iQLimiter annotation(
    Placement(visible = true, transformation(origin = {60, -270}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Sources.Constant constant2(k = iQminPu) annotation(
    Placement(visible = true, transformation(origin = {36, -278}, extent = {{-4, -4}, {4, 4}}, rotation = 0)));
  Modelica.Blocks.Continuous.FirstOrder firstOrder1(T = Tg, y_start = i0Pu.im * (SystemBase.SnRef / SNom))  annotation(
    Placement(visible = true, transformation(origin = {178, -270}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Continuous.FirstOrder omegaFilter(T = Tfltr, y_start = SystemBase.omegaRef0Pu)  annotation(
    Placement(visible = true, transformation(origin = {-90, -344}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealInput omegaRefPu(start = SystemBase.omegaRef0Pu) annotation(
    Placement(visible = true, transformation(origin = {8, -2}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {-110, -102}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
  Dynawo.Electrical.InverterBasedGeneration.GenericIBG.FrequencyProtection frequencyProtection(OmegaMaxPu = omegaMaxPu, OmegaMinPu = omegaMinPu)  annotation(
    Placement(visible = true, transformation(origin = {-48, -344}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));

  model OverfrequencySupport

    parameter Real omega_sPu "Frequency at which generation curtailment starts";
    parameter Real omegaMaxPu "Frequency at which generation is completely disconnected";

    Modelica.Blocks.Interfaces.RealInput omegaPu annotation(
        Placement(visible = true, transformation(origin = {-120, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-120, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
    Modelica.Blocks.Interfaces.RealOutput deltaP annotation(
        Placement(visible = true, transformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Blocks.Interfaces.RealInput PextPu annotation(
      Placement(visible = true, transformation(origin = {-120, -80}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-120, -80}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));

  equation

    if omegaPu < omega_sPu then
      deltaP = 0;
    elseif omegaPu < omegaMaxPu then
      deltaP = PextPu * (omegaMaxPu - omegaPu);
    else
      deltaP = PextPu;
    end if;

  end OverfrequencySupport;

  Dynawo.Electrical.InverterBasedGeneration.GenericIBG.OverfrequencySupport overfrequencySupport(omegaMaxPu = omegaMaxPu)  annotation(
    Placement(visible = true, transformation(origin = {-48, -378}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Math.Add add2(k2 = -1)  annotation(
    Placement(visible = true, transformation(origin = {-26, -146}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));

  model OverVoltageProtection

    Connectors.BPin switchOffSignal (value (start = false)) "Switch off message for the generator";
    Modelica.Blocks.Interfaces.RealInput Vm annotation(
      Placement(visible = true, transformation(origin = {-120, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-120, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));

    parameter Real VMax "Maximum voltage over which the unit is disconnected";

  equation

    when Vm > VMax then
      switchOffSignal.value = true;
    end when;

  end OverVoltageProtection;

  Dynawo.Electrical.InverterBasedGeneration.GenericIBG.OverVoltageProtection overVoltageProtection annotation(
    Placement(visible = true, transformation(origin = {-50, -50}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Sources.RealExpression Pext(y = PextPu)  annotation(
    Placement(visible = true, transformation(origin = {-90, -386}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
protected
  parameter Types.ComplexPerUnit u0Pu "Start value of complex voltage at terminal in pu (base UNom)";
  parameter Types.ComplexPerUnit s0Pu "Start value of complex apparent power at terminal in pu (base SnRef) (receptor convention)";
  parameter Types.ComplexPerUnit i0Pu "Start value of complex current at terminal in pu (base UNom, SnRef) (receptor convention)";

  parameter Types.PerUnit Id0Pu "Start value of d-axs current at injector in pu (base UNom, SNom) (generator convention)";
  parameter Types.PerUnit Iq0Pu "Start value of q-axis current at injector in pu (base UNom, SNom) (generator convention)";
  parameter Types.PerUnit iQref0Pu "Start value of the reference q-axis current at injector in pu (base UNom, SNom) (generator convention)";
equation
  injector.switchOffSignal1.value = lvrt.switchOffSignal.value;
  injector.switchOffSignal2.value = frequencyProtection.switchOffSignal.value;
  injector.switchOffSignal3.value = overVoltageProtection.switchOffSignal.value;
  connect(injector.terminal, terminal) annotation(
    Line(points = {{251.5, 1.9}, {305.5, 1.9}}, color = {0, 0, 255}));
  connect(injector.UPu, firstOrder.u) annotation(
    Line(points = {{251.5, -14}, {260, -14}, {260, 38}, {-152, 38}, {-152, -49}, {-142, -49}, {-142, -50}}, color = {0, 0, 127}));
  connect(firstOrder.y, Vfreeze.u) annotation(
    Line(points = {{-118, -50}, {-110, -50}, {-110, -10}, {-62, -10}}, color = {0, 0, 127}));
  connect(firstOrder.y, lvrt.UMonitoredPu) annotation(
    Line(points = {{-118, -50}, {-110, -50}, {-110, -90}, {-62, -90}}, color = {0, 0, 127}));
  connect(injector.uPu, PLLFreeze.uPu) annotation(
    Line(points = {{251.5, -3}, {254, -3}, {254, 32}, {34, 32}, {34, -2}, {39, -2}}, color = {85, 170, 255}));
  connect(division.y, iPLimiter.u) annotation(
    Line(points = {{17, -160}, {48, -160}}, color = {0, 0, 127}));
  connect(const.y, iPLimiter.limit2) annotation(
    Line(points = {{44.4, -168}, {48.4, -168}}, color = {0, 0, 127}));
  connect(iPSlewLimiter.y, gain.u) annotation(
    Line(points = {{165, -160}, {178, -160}}, color = {0, 0, 127}));
  connect(gain.y, integrator.u) annotation(
    Line(points = {{201, -160}, {210, -160}}, color = {0, 0, 127}));
  connect(iPLimiter.y, feedback1.u1) annotation(
    Line(points = {{71, -160}, {110, -160}}, color = {0, 0, 127}));
  connect(feedback1.y, iPSlewLimiter.u) annotation(
    Line(points = {{127, -160}, {142, -160}}, color = {0, 0, 127}));
  connect(Vfreeze.y, PLLFreeze.freeze) annotation(
    Line(points = {{-39, -10}, {-1, -10}, {-1, -16}, {39, -16}}, color = {255, 0, 255}));
  connect(realToComplex.y, transformRItoDQ.uPu) annotation(
    Line(points = {{167, 4}, {169, 4}, {169, -0.5}, {179, -0.5}, {179, -1}}, color = {85, 170, 255}));
  connect(PLLFreeze.cosPhi, transformRItoDQ.cosPhi) annotation(
    Line(points = {{61, -11}, {82, -11}, {82, -10}, {180, -10}}, color = {0, 0, 127}));
  connect(transformRItoDQ.sinPhi, PLLFreeze.sinPhi) annotation(
    Line(points = {{180, -14}, {82, -14}, {82, -15}, {61, -15}}, color = {0, 0, 127}));
  connect(integrator.y, realToComplex.re) annotation(
    Line(points = {{233, -160}, {240, -160}, {240, -60}, {120, -60}, {120, 10}, {144, 10}}, color = {0, 0, 127}));
  connect(firstOrder.y, max.u1) annotation(
    Line(points = {{-118, -50}, {-110, -50}, {-110, -160}, {-82, -160}}, color = {0, 0, 127}));
  connect(constant1.y, max.u2) annotation(
    Line(points = {{-91.6, -172}, {-81.6, -172}}, color = {0, 0, 127}));
  connect(max.y, division.u2) annotation(
    Line(points = {{-59, -166}, {-6, -166}}, color = {0, 0, 127}));
  connect(firstOrder.y, currentPriority.u) annotation(
    Line(points = {{-118, -50}, {-110, -50}, {-110, -224}, {-102, -224}}, color = {0, 0, 127}));
  connect(currentPriority.y, limitUpdating.Pflag) annotation(
    Line(points = {{-79, -224}, {2, -224}}, color = {255, 0, 255}));
  connect(firstOrder.y, voltageSupport.Vm) annotation(
    Line(points = {{-118, -50}, {-110, -50}, {-110, -264}, {-101, -264}}, color = {0, 0, 127}));
  connect(voltageSupport.iQsupPu, add.u1) annotation(
    Line(points = {{-79.2, -264.2}, {-62.2, -264.2}}, color = {0, 0, 127}));
  connect(iQrefPu, add.u2) annotation(
    Line(points = {{-94, -294}, {-70, -294}, {-70, -276}, {-62, -276}}, color = {0, 0, 127}));
  connect(add.y, iQLimiter.u) annotation(
    Line(points = {{-39, -270}, {48, -270}}, color = {0, 0, 127}));
  connect(constant2.y, iQLimiter.limit2) annotation(
    Line(points = {{40.4, -278}, {48.4, -278}}, color = {0, 0, 127}));
  connect(iQLimiter.y, firstOrder1.u) annotation(
    Line(points = {{71, -270}, {166, -270}}, color = {0, 0, 127}));
  connect(firstOrder1.y, realToComplex.im) annotation(
    Line(points = {{189, -270}, {246, -270}, {246, -56}, {124, -56}, {124, -2}, {144, -2}}, color = {0, 0, 127}));
  connect(PLLFreeze.omegaPLLPu, omegaFilter.u) annotation(
    Line(points = {{61, -3}, {70, -3}, {70, 10}, {-160, 10}, {-160, -344}, {-102, -344}}, color = {0, 0, 127}));
  connect(PextPu, add2.u1) annotation(
    Line(points = {{-94, -140}, {-38, -140}}, color = {0, 0, 127}));
  connect(add2.y, division.u1) annotation(
    Line(points = {{-15, -146}, {-11, -146}, {-11, -154}, {-6, -154}}, color = {0, 0, 127}));
  connect(overfrequencySupport.deltaP, add2.u2) annotation(
    Line(points = {{-37, -378}, {-23, -378}, {-23, -188}, {-45, -188}, {-45, -152}, {-38, -152}}, color = {0, 0, 127}));
  connect(firstOrder.y, overVoltageProtection.Vm) annotation(
    Line(points = {{-118, -50}, {-62, -50}}, color = {0, 0, 127}));
  connect(iPLimiter.y, limitUpdating.iPcmd) annotation(
    Line(points = {{71, -160}, {80, -160}, {80, -208}, {-10, -208}, {-10, -216}, {1, -216}}, color = {0, 0, 127}));
  connect(limitUpdating.iQlim, iQLimiter.limit1) annotation(
    Line(points = {{23, -230}, {32, -230}, {32, -262}, {48, -262}}, color = {0, 0, 127}));
  connect(limitUpdating.iPlim, iPLimiter.limit1) annotation(
    Line(points = {{22.8, -218}, {31.8, -218}, {31.8, -152}, {47.8, -152}}, color = {0, 0, 127}));
  connect(iQLimiter.y, limitUpdating.iQcmd) annotation(
    Line(points = {{71, -270}, {80, -270}, {80, -240}, {-10, -240}, {-10, -231}, {2, -231}}, color = {0, 0, 127}));
  connect(integrator.y, feedback1.u2) annotation(
    Line(points = {{234, -160}, {240, -160}, {240, -190}, {118, -190}, {118, -168}}, color = {0, 0, 127}));
  connect(Pext.y, overfrequencySupport.PextPu) annotation(
    Line(points = {{-79, -386}, {-60, -386}}, color = {0, 0, 127}));
  connect(omegaFilter.y, overfrequencySupport.omegaPu) annotation(
    Line(points = {{-78, -344}, {-70, -344}, {-70, -378}, {-60, -378}}, color = {0, 0, 127}));
  connect(omegaRefPu, PLLFreeze.omegaRefPu) annotation(
    Line(points = {{8, -2}, {22, -2}, {22, -8}, {40, -8}}, color = {0, 0, 127}));
  connect(transformRItoDQ.uqPu, injector.iqPu) annotation(
    Line(points = {{202, -14}, {214, -14}, {214, -2}, {228, -2}}, color = {0, 0, 127}));
  connect(transformRItoDQ.udPu, injector.idPu) annotation(
    Line(points = {{202, -2}, {208, -2}, {208, -8}, {220, -8}, {220, -12}, {228, -12}}, color = {0, 0, 127}));
  connect(omegaFilter.y, frequencyProtection.omegaPu) annotation(
    Line(points = {{-78, -344}, {-60, -344}}, color = {0, 0, 127}));
  annotation(
    Documentation(preferredView = "diagram",
    info = "<html>
    <p> Generic model of inverter-based generation as defined in p28 of Gilles Chaspierre's PhD thesis 'Reduced-order modelling of active distribution networks for large-disturbance simulations'. Available: https://orbi.uliege.be/handle/2268/251602 </p></html>"),
    Diagram(coordinateSystem(extent = {{-180, 40}, {320, -400}})));
end GenericIBG;

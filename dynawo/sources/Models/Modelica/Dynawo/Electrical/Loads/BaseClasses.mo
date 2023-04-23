within Dynawo.Electrical.Loads;

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

package BaseClasses

  extends Icons.BasesPackage;

  partial model BaseLoad "Base model for loads"
    import Modelica;
    import Dynawo.Connectors;
    import Dynawo.Electrical.Controls.Basics.SwitchOff;
    import Dynawo.Electrical.SystemBase;

    extends SwitchOff.SwitchOffLoad;

    Connectors.ACPower terminal(V(re(start = u0Pu.re), im(start = u0Pu.im)), i(re(start = i0Pu.re), im(start = i0Pu.im))) "Connector used to connect the load to the grid" annotation(
      Placement(visible = true, transformation(origin = {-1.42109e-14, 98}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {-1.42109e-14, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));

    // in order to change the load set-point, connect an event to PRefPu or QRefPu
    Modelica.Blocks.Interfaces.RealInput PRefPu(start = s0Pu.re) "Active power request" annotation(
      Placement(visible = true, transformation(origin = {-50, -100}, extent = {{-20, -20}, {20, 20}}, rotation = 90), iconTransformation(origin = {-61, -85}, extent = {{-15, -15}, {15, 15}}, rotation = 90)));
    Modelica.Blocks.Interfaces.RealInput QRefPu(start = s0Pu.im) "Reactive power request" annotation(
      Placement(visible = true, transformation(origin = {50, -100}, extent = {{-20, -20}, {20, 20}}, rotation = 90), iconTransformation(origin = {60, -84}, extent = {{-16, -16}, {16, 16}}, rotation = 90)));
    Modelica.Blocks.Interfaces.RealInput deltaP(start = 0) "Delta to apply on PRef in %" annotation(
      Placement(visible = true, transformation(origin = {-75, -100}, extent = {{-20, -20}, {20, 20}}, rotation = 90), iconTransformation(origin = {-61, -85}, extent = {{-15, -15}, {15, 15}}, rotation = 90)));
    Modelica.Blocks.Interfaces.RealInput deltaQ(start = 0) "Delta to apply on QRef in %" annotation(
      Placement(visible = true, transformation(origin = {75, -100}, extent = {{-20, -20}, {20, 20}}, rotation = 90), iconTransformation(origin = {60, -84}, extent = {{-16, -16}, {16, 16}}, rotation = 90)));

    Connectors.ImPin UPu(value(start = ComplexMath.'abs'(u0Pu))) "Voltage amplitude at load terminal in pu (base UNom)";
    Types.ActivePowerPu PPu(start = s0Pu.re) "Active power at load terminal in pu (base SnRef) (receptor convention)";
    Types.ReactivePowerPu QPu(start = s0Pu.im) "Reactive power at load terminal in pu (base SnRef) (receptor convention)";
    Types.ComplexApparentPowerPu SPu(re(start = s0Pu.re), im(start = s0Pu.im)) "Apparent power at load terminal in pu (base SnRef) (receptor convention)";

    parameter Types.ComplexVoltagePu u0Pu "Start value of complex voltage at load terminal in pu (base UNom)";
    parameter Types.ComplexApparentPowerPu s0Pu "Start value of apparent power at load terminal in pu (base SnRef) (receptor convention)";
    parameter Types.ComplexCurrentPu i0Pu "Start value of complex current at load terminal in pu (base UNom, SnRef) (receptor convention)";

  equation
    SPu = Complex(PPu, QPu);
    SPu = terminal.V * ComplexMath.conj(terminal.i);
    if running.value then
      UPu.value = ComplexMath.'abs'(terminal.V);
    else
      UPu.value = 0;
    end if;

    annotation(
      preferredView = "text");
  end BaseLoad;

  partial model BaseLoadMotorSimplified
    import Dynawo;
    extends BaseLoad;

    parameter Integer NbMotors "Number of motors modelled in the load";
    parameter Real ActiveMotorShare[NbMotors] (each min=0, each max=1) "Share of active power consumed by motors (between 0 and 1)";
    parameter Types.ApparentPowerModule SNom[NbMotors] = ActiveMotorShare * s0Pu.re * SystemBase.SnRef "Nominal apparent power of a single motor in MVA";
    parameter Types.PerUnit RsPu[NbMotors] "Stator resistance in pu (base UNom, SNom)";
    parameter Types.PerUnit RrPu[NbMotors] "Rotor resistance in pu (base UNom, SNom)";
    parameter Types.PerUnit XsPu[NbMotors] "Stator leakage reactance in pu (base UNom, SNom)";
    parameter Types.PerUnit XrPu[NbMotors] "Rotor leakage reactance in pu (base UNom, SNom)";
    parameter Types.PerUnit XmPu[NbMotors] "Magnetizing reactance in pu (base UNom, SNom)";
    parameter Real H[NbMotors] "Inertia constant (s, base UNom, SNom)";
    parameter Real torqueExponent[NbMotors] "Exponent of the torsque speed dependancy";

    parameter Types.ActivePowerPu PLoad0Pu "Start value of the active power consumed by the load in pu (SnRef base)";
    parameter Types.ReactivePowerPu QLoad0Pu "Start value of the reactive power consumed by the load in pu (SnRef base)";
    parameter Types.ComplexCurrentPu is0Pu[NbMotors] "Start value of the stator current in pu (base SNom, UNom)";
    parameter Types.ComplexCurrentPu im0Pu[NbMotors] "Start value of the magnetising current in pu (base SNom, UNom)";
    parameter Types.ComplexCurrentPu ir0Pu[NbMotors] "Start value of the rotor current in pu (base SNom, UNom)";
    parameter Types.PerUnit ce0Pu[NbMotors] "Start value of the electrical torque in pu (SNom base)";
    parameter Real s0[NbMotors] "Start value of the slip of the motor";
    parameter Types.AngularVelocity omegaR0Pu[NbMotors] "Start value of the angular velocity of the motor";
    parameter Types.ComplexCurrentPu motori0Pu[NbMotors] "Start value of complex current at load terminal in pu (base UNom, SnRef) (receptor convention)";
    parameter Types.ComplexApparentPowerPu motors0Pu[NbMotors] "Start value of complex apparent power in pu (base SnRef) (receptor convention)";

    Connectors.ImPin omegaRefPu(value(start = SystemBase.omegaRef0Pu)) "Network angular reference frequency in pu (base omegaNom)";

    Types.ActivePowerPu PLoadPu (start = PLoad0Pu) "Active power consumed by the load in pu (base SnRef) (receptor convention)";
    Types.ReactivePowerPu QLoadPu (start = QLoad0Pu) "Reactive power consumed by the load in pu (base SnRef) (receptor convention)";

    Dynawo.Electrical.Machines.Motors.SimplifiedMotor motors[NbMotors](SNom=SNom, RsPu=RsPu, RrPu=RrPu, XsPu=XsPu, XrPu=XrPu, XmPu=XmPu, H=H, torqueExponent=torqueExponent, each NbSwitchOffSignals=2, is0Pu=is0Pu, im0Pu=im0Pu, ir0Pu=ir0Pu, ce0Pu=ce0Pu, s0=s0, omegaR0Pu=omegaR0Pu, i0Pu=motori0Pu, s0Pu=motors0Pu, each u0Pu=u0Pu);
    Types.ActivePowerPu PLoadCmdPu (start = PLoad0Pu) "Active power reference for the load in pu (base SnRef) (receptor convention)";
    Types.ReactivePowerPu QLoadCmdPu (start = QLoad0Pu) "Reactive power reference for in pu (base SnRef) (receptor convention)";

  equation
    for i in 1:NbMotors loop
      motors[i].V = terminal.V;
      connect(motors[i].omegaRefPu, omegaRefPu);

      switchOffSignal1.value = motors[i].switchOffSignal1.value;
      switchOffSignal2.value = motors[i].switchOffSignal2.value;
    end for;

    if running.value then
      PLoadCmdPu = (1-sum(ActiveMotorShare)) * PRefPu * (1 + deltaP);
      QLoadCmdPu = QRefPu * (1 + deltaQ) - sum(motors.s0Pu.im) * (PRefPu/s0Pu.re) * (1 + deltaP); // s0Pu.re = PRef0Pu (if PRefPu increases but QRefPu stays constant, the reactive power consumed by the motor increases, so the reactive power of the load is reduced to keep the total constant).
      PPu = PLoadPu + sum(motors.PPu);
      QPu = QLoadPu + sum(motors.QPu);
    else
      PLoadCmdPu = 0;
      QLoadCmdPu = 0;
      terminal.i = Complex(0);
    end if;

  end BaseLoadMotorSimplified;

  partial model BaseLoadMotorFifthOrder
    import Dynawo;
    extends BaseLoad;

    parameter Integer NbMotors "Number of motors modelled in the load";
    parameter Real ActiveMotorShare[NbMotors] (each min=0, each max=1) "Share of active power consumed by motors (between 0 and 1)";
    parameter Types.ApparentPowerModule SNom[NbMotors] = ActiveMotorShare * s0Pu.re * SystemBase.SnRef "Nominal apparent power of a single motor in MVA";
    parameter Types.PerUnit RsPu[NbMotors] "Stator resistance in pu (base SNom, UNom)";
    parameter Types.PerUnit LsPu[NbMotors] "Synchronous reactance in pu (base SNom, UNom)";
    // Notation: L (reactance) + P ("'" or "Prim") + Pu (Per unit)
    parameter Types.PerUnit LPPu[NbMotors] "Transient reactance in pu (base SNom, UNom)";
    parameter Types.PerUnit LPPPu[NbMotors] "Subtransient reactance in pu (base SNom, UNom)";
    parameter Types.Time tP0[NbMotors] "Transient open circuit time constant in s";
    parameter Types.Time tPP0[NbMotors] "Subtransient open circuit time constant in s";
    parameter Real H[NbMotors] "Inertia constant (s, base UNom, SNom)";
    parameter Real torqueExponent[NbMotors] "Exponent of the torque speed dependancy";

    parameter Types.ActivePowerPu PLoad0Pu "Start value of the active power consumed by the load in pu (SnRef base)";
    parameter Types.ReactivePowerPu QLoad0Pu "Start value of the reactive power consumed by the load in pu (SnRef base)";
    parameter Types.PerUnit EdP0Pu[NbMotors];
    parameter Types.PerUnit EqP0Pu[NbMotors];
    parameter Types.PerUnit EdPP0Pu[NbMotors];
    parameter Types.PerUnit EqPP0Pu[NbMotors];
    parameter Types.PerUnit Ud0Pu[NbMotors] "Start value of voltage of direct axis in pu";
    parameter Types.PerUnit Uq0Pu[NbMotors] "Start value of voltage of quadrature axis in pu";
    parameter Types.PerUnit id0Pu[NbMotors] "Start value of current of direct axis in pu";
    parameter Types.PerUnit iq0Pu[NbMotors] "Start value of current of quadrature axis in pu";
    parameter Types.PerUnit ce0Pu[NbMotors] "Start value of the electrical torque in pu (SNom base)";
    parameter Real s0[NbMotors] "Start value of the slip of the motor";
    parameter Types.AngularVelocity omegaR0Pu[NbMotors] "Start value of the angular velocity of the motor";
    parameter Types.ComplexCurrentPu motori0Pu[NbMotors] "Start value of complex current at load terminal in pu (base UNom, SnRef) (receptor convention)";
    parameter Types.ComplexApparentPowerPu motors0Pu[NbMotors] "Start value of complex apparent power in pu (base SnRef) (receptor convention)";

    Connectors.ImPin omegaRefPu(value(start = SystemBase.omegaRef0Pu)) "Network angular reference frequency in pu (base omegaNom)";

    Types.ActivePowerPu PLoadPu (start = PLoad0Pu) "Active power consumed by the load in pu (base SnRef) (receptor convention)";
    Types.ReactivePowerPu QLoadPu (start = QLoad0Pu) "Reactive power consumed by the load in pu (base SnRef) (receptor convention)";

    Dynawo.Electrical.Machines.Motors.MotorFifthOrder motors[NbMotors](SNom=SNom, RsPu=RsPu, LsPu=LsPu, LPPu=LPPu, LPPPu=LPPPu, tP0=tP0, tPP0=tPP0, H=H, torqueExponent=torqueExponent, each NbSwitchOffSignals = 2, EdP0Pu=EdP0Pu, EqP0Pu=EqP0Pu, EdPP0Pu=EdPP0Pu, EqPP0Pu=EqPP0Pu, Ud0Pu=Ud0Pu, Uq0Pu=Uq0Pu, id0Pu=id0Pu, iq0Pu=iq0Pu, ce0Pu=ce0Pu, s0=s0, omegaR0Pu=omegaR0Pu, i0Pu=motori0Pu, s0Pu=motors0Pu, each u0Pu=u0Pu);
    Types.ActivePowerPu PLoadCmdPu (start = PLoad0Pu) "Active power reference for the load in pu (base SnRef) (receptor convention)";
    Types.ReactivePowerPu QLoadCmdPu (start = QLoad0Pu) "Reactive power reference for in pu (base SnRef) (receptor convention)";

  equation
    for i in 1:NbMotors loop
      motors[i].V = terminal.V;
      connect(motors[i].omegaRefPu, omegaRefPu);

      switchOffSignal1.value = motors[i].switchOffSignal1.value;
      switchOffSignal2.value = motors[i].switchOffSignal2.value;
    end for;

    if running.value then
      PLoadCmdPu = (1-sum(ActiveMotorShare)) * PRefPu * (1 + deltaP);
      QLoadCmdPu = QRefPu * (1 + deltaQ) - sum(motors.s0Pu.im) * (PRefPu/s0Pu.re) * (1 + deltaP); // s0Pu.re = PRef0Pu (if PRefPu increases but QRefPu stays constant, the reactive power consumed by the motor increases, so the reactive power of the load is reduced to keep the total constant).
      PPu = PLoadPu + sum(motors.PPu);
      QPu = QLoadPu + sum(motors.QPu);
    else
      PLoadCmdPu = 0;
      QLoadCmdPu = 0;
      PPu = 0;
      QPu = 0;
    end if;

  end BaseLoadMotorFifthOrder;
end BaseClasses;

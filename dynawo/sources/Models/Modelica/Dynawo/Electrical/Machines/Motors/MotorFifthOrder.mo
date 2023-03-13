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

model MotorFifthOrder
  extends BaseClasses.BaseMotor;
  extends AdditionalIcons.Machine;

  parameter Types.PerUnit RsPu "Stator resistance in pu (base SNom, UNom)";
  parameter Types.PerUnit LsPu "Synchronous reactance in pu (base SNom, UNom)";
  // Notation: L (reactance) + P ("'" or "Prim") + Pu (Per unit)
  parameter Types.PerUnit LPPu "Transient reactance in pu (base SNom, UNom)";
  parameter Types.PerUnit LPPPu "Subtransient reactance in pu (base SNom, UNom)";
  parameter Types.Time tP0 "Transient open circuit time constant in s";
  parameter Types.Time tPP0 "Subtransient open circuit time constant in s";
  parameter Real torqueExponent "Exponent of the torsque speed dependancy";
  parameter Types.Time H "Kinetic constant = kinetic energy / rated power";

  parameter Types.PerUnit EdP0Pu;
  parameter Types.PerUnit EqP0Pu;
  parameter Types.PerUnit EdPP0Pu;
  parameter Types.PerUnit EqPP0Pu;
  parameter Types.PerUnit Ud0Pu "Start value of voltage of direct axis in pu";
  parameter Types.PerUnit Uq0Pu "Start value of voltage of quadrature axis in pu";
  parameter Types.PerUnit id0Pu "Start value of current of direct axis in pu";
  parameter Types.PerUnit iq0Pu "Start value of current of quadrature axis in pu";
  parameter Types.AngularVelocity omegaR0Pu "Start value of the angular velocity of the motor";
  parameter Types.PerUnit ce0Pu "Start value of the electrical torque in pu (SNom/omegaNom base)";
  parameter Real s0 "Start value of the slip of the motor";

  Types.AngularVelocity omegaRPu(start = omegaR0Pu) "Angular velocity of the motor in pu (base omegaNom)";
  Types.PerUnit EdPPu(start = EdP0Pu);
  Types.PerUnit EqPPu(start = EqP0Pu);
  Types.PerUnit EdPPPu(start = EdPP0Pu);
  Types.PerUnit EqPPPu(start = EqPP0Pu);
  Types.PerUnit UdPu(start = Ud0Pu) "Voltage of direct axis in pu";
  Types.PerUnit UqPu(start = Uq0Pu) "Voltage of quadrature axis in pu";
  Types.PerUnit idPu(start = id0Pu) "Current of direct axis in pu";
  Types.PerUnit iqPu(start = iq0Pu) "Current of quadrature axis in pu";
  Real s(start = s0) "Slip of the motor";
  Types.PerUnit cePu(start = ce0Pu) "Electrical torque in pu (base SNom, omegaNom)";
  Types.PerUnit clPu(start = ce0Pu) "Load torque in pu (base SNom, omegaNom)";

equation
  der(EqPPu) * tP0 = -EqPPu - idPu * (LsPu - LPPu) - EdPPu * omegaRefPu.value * s * tP0;
  der(EdPPu) * tP0 = -EdPPu + iqPu * (LsPu - LPPu) + EqPPu * omegaRefPu.value * s * tP0;
  der(EqPPPu) = (tP0 - tPP0) / (tP0 * tPP0) * EqPPu - (tPP0 * (LsPu - LPPu) + tP0 * (LPPu - LPPPu)) / (tP0 * tPP0) * idPu - EqPPPu / tPP0 - omegaRefPu.value * s * EdPPPu;
  der(EdPPPu) = (tP0 - tPP0) / (tP0 * tPP0) * EdPPu + (tPP0 * (LsPu - LPPu) + tP0 * (LPPu - LPPPu)) / (tP0 * tPP0) * iqPu - EdPPPu / tPP0 + omegaRefPu.value * s * EqPPPu;

  idPu = RsPu / (RsPu^2 + LPPPu^2) * (UdPu + EdPPPu) + LPPPu / (RsPu^2 + LPPPu^2) * (UqPu + EqPPPu);
  iqPu = RsPu / (RsPu^2 + LPPPu^2) * (UqPu + EqPPPu) - LPPPu / (RsPu^2 + LPPPu^2) * (UdPu + EdPPPu);

  PPu = (UdPu * idPu + UqPu * iqPu) * (SNom / SystemBase.SnRef);
  QPu = (UqPu * idPu - UdPu * iqPu) * (SNom / SystemBase.SnRef);

  // dq reference frame rotating at synchronous speed
  UdPu = terminal.V.re;
  UqPu = terminal.V.im;

  s = (omegaRefPu.value - omegaRPu) / omegaRefPu.value;
  cePu = EdPPPu * idPu + EqPPPu * iqPu;
  clPu = ce0Pu * (omegaRPu / omegaR0Pu)^torqueExponent;
  2*H*der(omegaRPu) = cePu - clPu;

  annotation(preferredView = "text");
end MotorFifthOrder;

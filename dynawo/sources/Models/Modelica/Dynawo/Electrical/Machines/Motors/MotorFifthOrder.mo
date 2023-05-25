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

model MotorFifthOrder "Two-cage (or one-cage if Lpp = Lp) induction motor model, based on https://www.powerworld.com/WebHelp/Content/TransientModels_HTML/Load%20Characteristic%20MOTORW.htm, must be incorporated in a load model."
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

  parameter Types.PerUnit EdP0Pu "Name?";
  parameter Types.PerUnit EqP0Pu;
  parameter Types.PerUnit EdPP0Pu;
  parameter Types.PerUnit EqPP0Pu;
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
  Types.PerUnit idPu(start = id0Pu) "Current of direct axis in pu";
  Types.PerUnit iqPu(start = iq0Pu) "Current of quadrature axis in pu";
  Real s(start = s0) "Slip of the motor";
  Types.PerUnit cePu(start = ce0Pu) "Electrical torque in pu (base SNom, omegaNom)";
  Types.PerUnit clPu(start = ce0Pu) "Load torque in pu (base SNom, omegaNom)";

equation
  if (running.value) then
    der(EqPPu) * tP0 = -EqPPu + idPu * (LsPu - LPPu) - EdPPu * SystemBase.omegaNom * omegaRefPu.value * s * tP0;
    der(EdPPu) * tP0 = -EdPPu - iqPu * (LsPu - LPPu) + EqPPu * SystemBase.omegaNom * omegaRefPu.value * s * tP0;
    der(EqPPPu) = der(EqPPu) + 1/tPP0 * (EqPPu - EqPPPu + (LPPu - LPPPu) * idPu) + SystemBase.omegaNom * omegaRefPu.value * s * (EdPPu - EdPPPu);
    der(EdPPPu) = der(EdPPu) + 1/tPP0 * (EdPPu - EdPPPu - (LPPu - LPPPu) * iqPu) - SystemBase.omegaNom * omegaRefPu.value * s * (EqPPu - EqPPPu);

    V = Complex(EdPPPu, EqPPPu) + Complex(RsPu, LPPPu) * Complex(idPu, iqPu);
    Complex(PPu, QPu) = V * Complex(idPu, -iqPu) * (SNom / SystemBase.SnRef);

    s = (omegaRefPu.value - omegaRPu) / omegaRefPu.value;
    cePu = EdPPPu * idPu + EqPPPu * iqPu;
    clPu = ce0Pu * (omegaRPu / omegaR0Pu)^torqueExponent;
    2*H*der(omegaRPu) = cePu - clPu;
  else
    der(EqPPu) = 0;
    der(EdPPu) = 0;
    der(EqPPPu) = 0;
    der(EdPPPu) = 0;
    idPu = 0;
    iqPu = 0;
    SPu = Complex(0);
    s = 0;
    cePu = 0;
    clPu = 0;
    der(omegaRPu) = 0;
  end if;

  annotation(preferredView = "text");
end MotorFifthOrder;

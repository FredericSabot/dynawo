within Dynawo.Electrical.Loads;

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

model LoadAlphaBetaMotorSimplified
  extends BaseClasses.BaseLoadMotorSimplified;
  redeclare parameter Integer NbMotors = 1;
  parameter Real Alpha "Active load sensitivity to voltage";
  parameter Real Beta "Reactive load sensitivity to voltage";

equation
  PLoadPu = PLoadCmdPu * ((ComplexMath.'abs' (terminal.V) / ComplexMath.'abs' (u0Pu)) ^ Alpha);
  QLoadPu = QLoadCmdPu * ((ComplexMath.'abs' (terminal.V) / ComplexMath.'abs' (u0Pu)) ^ Beta);

  annotation(preferredView = "text");
end LoadAlphaBetaMotorSimplified;

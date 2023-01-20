within Dynawo.Electrical.Controls.WECC.Utilities;

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

model CurrentReduction "This block measures the voltage, current, active power and reactive power in pu (base UNom, SNom or SnRef)"

/*
  Equivalent circuit and conventions:

               iPu, uPu
   (terminal1) -->---------MEASUREMENTS------------ (terminal2)

*/
  import Modelica;
  import Dynawo.Connectors;
  import Modelica.ComplexMath;

  Connectors.ACPower terminal1 annotation(
    Placement(visible = true, transformation(origin = {-100, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {-100, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Connectors.ACPower terminal2 annotation(
    Placement(visible = true, transformation(origin = {100, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {100, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealInput f (start = 1) "Fraction of distributed generation that is still connected" annotation(
    Placement(visible = true, transformation(origin = {-120, 60}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-98, 58}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));

equation
  terminal1.i * f = - terminal2.i;
  terminal1.V = terminal2.V;


annotation(preferredView = "text");
end CurrentReduction;

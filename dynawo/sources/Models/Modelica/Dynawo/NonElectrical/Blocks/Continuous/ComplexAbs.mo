within Dynawo.NonElectrical.Blocks.Continuous;

model ComplexAbs

import Modelica;
extends Modelica.Blocks.Interfaces.SI2SO;

equation
  y = sqrt(u1^2 + u2^2);

end ComplexAbs;

within Dynawo.Electrical.Lines;

function GetOppositeLineVoltage "Function that, given the complex voltage and current at one side of a line, gives the complex voltage at the other side of the line"
  extends Icons.Function;

  input Types.PerUnit RPu "Resistance in pu (base SnRef)";
  input Types.PerUnit XPu "Reactance in pu (base SnRef)";
  input Types.PerUnit GPu "Half-conductance in pu (base SnRef)";
  input Types.PerUnit BPu "Half-susceptance in pu (base SnRef)";
  input Types.ComplexCurrentPu i1Pu "Complex current on side 1 in pu (base SnRef)";
  input Types.ComplexVoltagePu u1Pu "Complex voltage on side 1 in pu (baeUNom, SnRef);";

  output Types.ComplexVoltagePu u2Pu "Complex voltage on side 1 in pu (baeUNom, SnRef);";

protected
  Types.ComplexImpedancePu ZPu(re = RPu, im = XPu) "Line impedance";
  Types.ComplexAdmittancePu YPu(re = GPu, im = BPu) "Line half-admittance";

algorithm
  u2Pu := u1Pu - ZPu * (i1Pu - YPu * u1Pu);

  annotation(preferredView = "text");
end GetOppositeLineVoltage;

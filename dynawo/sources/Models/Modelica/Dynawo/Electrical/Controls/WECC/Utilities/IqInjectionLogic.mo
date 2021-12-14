within Dynawo.Electrical.Controls.WECC.Utilities;

block IqInjectionLogic "Reactive Current Injection Logic"

  import Modelica;
  import Modelica.SIunits;
  import Dynawo.Types;

  parameter SIunits.PerUnit IqFrzPu "Constant reactive current injection value in Pu";
  parameter SIunits.Time tIq "Absolute value of tIq defines seconds to hold current injection after voltage dip ended. tIq < 0 for constant, 0 for no injection after voltage dip, tIq > 0 for voltage dependent injection";


  Modelica.Blocks.Interfaces.RealInput iqV "Input for voltage dependent reactive current injection" annotation(
    Placement(visible = true, transformation(origin = {-108, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-100, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
  Modelica.Blocks.Interfaces.BooleanInput vDip "Ongoing voltage dip" annotation(
    Placement(visible = true, transformation(origin = {-108, 38}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-100, 80}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealOutput iqInjPu "Reactive current injection" annotation(
    Placement(visible = true, transformation(origin = {104, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {104, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));

  // Variables to detect if a voltage dip started/ended and to handle the time dependency
  Boolean vDipStart "boolean value defining the voltage dip start : true if voltage dip started ";
  Modelica.SIunits.Time vDipInjEndTime "ending time of the voltage dip start : true if voltage dip started ";

initial algorithm
  vDipStart := false;
  vDipInjEndTime := -1;

algorithm

  /*  CONTROL VARIABLE EXPLANATION:
   *
   *  This algorithm handles the time sensitive control variables that manage
   *  the equations and ultimately the output of the block.
   *  If vDipInjEndTime is set to -1, there is no end time, and thus no freeze
   *  period. It uses "when" clauses, which activate only once, when the condition
   *  is met, in other words: when an event happens.
   *
   *  1. If a Voltage dip occurs, vDipStart is set to true (and end timing is reset).
   *     Now the injection is set to the voltage dependent input.
   *  2. When the Voltage dip ends, vDipStart is set to false and the end timing is
   *     calculated by adding the absolute value of tIq to the current time. During
   *     this time, Equations are activated, managing the injection depending on the
   *     value of tIq.
   *  3. Finally, if the current time passes the end timing vDipInjEndTime, it is
   *     reset to -1, and thus, the injection returns to 0.
   *
   *  If tIq = 0, the end time will instantly be reached, which is in accordance to the
   *  'if' system in the equation section, since only the first or the else branch will
   *  be reached if tIq = 0. Therefore, there is no need to check for tIq != 0.*/

  when vDip == true then
    vDipStart := true;
    vDipInjEndTime := -1; // Reset end timing if new voltage dip starts during freeze phase.
  end when;
  when vDip == false and vDipStart == true then // Vdip has ended, set timing and reset Vdip detection variable
    vDipStart := false;
    vDipInjEndTime := time + abs(tIq);
  end when;
  when time >= vDipInjEndTime and vDipInjEndTime >= 0 then // End time reached, reset.
    vDipInjEndTime := -1;
  end when;

equation

  if (vDip == true) or (vDip == false and vDipInjEndTime >= 0 and tIq > 0 and time <= vDipInjEndTime) then // check for vDipInjEndTime >= 0 to see if there is a freeze state and time <= vDipInjEndTime for additional safety.
    iqInjPu = iqV;
  elseif vDip == false and vDipInjEndTime >= 0 and tIq < 0 and time <= vDipInjEndTime then
    iqInjPu = IqFrzPu;
  else
    iqInjPu = 0;
  end if;

annotation(
    Documentation(info = "<html><head></head><body><p>This block implements the behavior of the switch mechanic for reactive current injection, as specified in:<br><a href=\"https://www.wecc.org/Reliability/WECC-Second-Generation-Wind-Turbine-Models-012314.pdf\">https://www.wecc.org/Reliability/WECC-Second-Generation-Wind-Turbine-Models-012314.pdf</a></p><ul><li>Setting tIq to 0 results in abrupt ending of injection after the voltage dip has ended.</li><li>Setting tIq to a positive value continues the injection with the voltage dependent injection for the absolute value of tIq seconds after the voltage dip has ended.</li><li>Setting tIq to a negative value continues the injection with a set constant (IqFrzPu) for the absolute value of tIq seconds after the voltage dip has ended.</li></ul></body></html>"),
    Icon(graphics = {Rectangle(extent = {{-100, 100}, {100, -100}}), Text(origin = {-2, 74}, extent = {{-74, -38}, {82, -78}}, textString = "Reactive Current"), Text(origin = {1, -7}, extent = {{-63, 17}, {69, -21}}, textString = "Injection Logic")}));


end IqInjectionLogic;

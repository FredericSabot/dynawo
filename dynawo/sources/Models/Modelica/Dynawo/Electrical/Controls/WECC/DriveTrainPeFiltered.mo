within Dynawo.Electrical.Controls.WECC;

model DriveTrainPeFiltered
  extends DriveTrainCommon;

  parameter Types.Time Tp "Filter time constant for mechanical power calculation (typical: - )";

  Modelica.Blocks.Continuous.FirstOrder Pmech(T = Tp, initType =    Modelica.Blocks.Types.Init.InitialOutput, k = 1, y(start = PInj0Pu), y_start = PInj0Pu) annotation(
    Placement(visible = true, transformation(origin = {-166, 4}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));

equation
  connect(Pe, Pmech.u) annotation(
    Line(points = {{-214, -54}, {-166, -54}, {-166, -8}}, color = {0, 0, 127}));
  connect(Pmech.y, TorqueM.u1) annotation(
    Line(points = {{-166, 15},{-166, 51}, {-152, 51}}, color = {0, 0, 127}));

annotation(Documentation(info = "<html>
<p> This block contains the updated drive train model for generic WECC WTG type 4 model according to (in case page cannot be found, copy link in browser): <br><a href=\"https://www.wecc.org/Administrative/Memo%20RES%20Modeling%20Updates-%20Pourbeik.pdf\" >https://www.wecc.org/Administrative/Memo%20RES%20Modeling%20Updates-%20Pourbeik.pdf</a> </p>
<p> This model is a simplified model for the purpose of emulating the behavior of torsional mode oscillations. The shaft damping coefficient (Dshaft) in the drive-train model is fitted to capture the net damping of the torsional mode seen in the post fault electrical power response. In the actual equipment, the drive train oscillations are damped through filtered signals and active damping controllers, which obviously are significantly different from the simple generic two mass drive train model used here. Therefore, the
parameters (and variables) of this simple drive-train model cannot necessarily be compared with
actual physical quantities directly. </p>
<p>In this updated version B of the drive train, the mechanical power is derived from the electrical power by filtering with first order delay. </p></html>"));


end DriveTrainPeFiltered;

within Dynawo.Electrical.Controls.Generic;

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

model GNS3Connection "Generic control automaton, communicates throught ip packets"

  import Dynawo.Electrical.Controls.Generic.Functions;
  import Dynawo.Electrical.Controls.Generic.GNS3ConnectionConstants;
  import Modelica;


  parameter Types.Time SamplingTime "Automaton sampling time";
  parameter Integer NbInputs "Number of required inputs data for the automaton";
  parameter Integer NbOutputs "Number of required outputs data from the automaton";
  parameter String InputsName[GenericAutomatonConstants.inputsMaxSize] = {"EMPTY" for i in 1:GNS3ConnectionConstants.inputsMaxSize} "Names of required inputs data for the automaton";
  parameter String OutputsName[GenericAutomatonConstants.outputsMaxSize] = {"EMPTY" for i in 1:GNS3ConnectionConstants.outputsMaxSize} "Names of required outputs data from the automaton";

  Types.Time t0 (start = 0) "First time when the automaton will act";
  Boolean initialize(start = true) "Indicates if the automaton should be called at initialization. Has to be true so that the program can define the wall-clock time at t=0";
  Real inputs[GNS3ConnectionConstants.inputsMaxSize] "Inputs provided to the automaton";
  Real outputs[GNS3ConnectionConstants.outputsMaxSize] "Outputs got from the automaton";
  Real initWallTime[2] "Wall-clock time when the automaton is called for the first time ([1] is part in seconds, [2] is part in nanoseconds)";
  parameter Real test = 0;

  Modelica.Blocks.Math.Sin sin annotation(
    Placement(visible = true, transformation(origin = {0, 2}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));

equation
sin.u = time * 2 * Modelica.Constants.pi / SamplingTime;
when pre(initialize) then
  initWallTime = Functions.GetWallTime(test);
  t0 = time;
  initialize = false;
  outputs = Functions.GNS3Automaton(initWallTime, t0, inputs, InputsName, NbInputs, GenericAutomatonConstants.inputsMaxSize, OutputsName, NbOutputs,GenericAutomatonConstants.outputsMaxSize);
// elsewhen time >= pre(t0) + SamplingTime then
elsewhen sin.y < 0 then
  // initWallTime[1] = pre(initWallTime[1]);  // Omedit want those uncommented, but dynawo does not
  // initWallTime[2] = pre(initWallTime[2]);
  t0 = time;
  initialize = false;
  outputs = Functions.GNS3Automaton(initWallTime, t0, inputs, InputsName, NbInputs, GenericAutomatonConstants.inputsMaxSize, OutputsName, NbOutputs,GenericAutomatonConstants.outputsMaxSize);
end when;

annotation(preferredView = "text",
    Documentation(info = "<html><head></head><body>This model enables to call an external C method representing the behavior of any control system. It communicates through IP packets and is designed to be used with GNS3.</body></html>"));
end GNS3Connection;

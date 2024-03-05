within Dynawo.Electrical.Controls.Generic.Cosimulation;

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

model CosimulationAutomaton "Generic control automaton, call an external model"
  import Dynawo.Electrical.Controls.Generic.Cosimulation.Functions;
  import Dynawo.Electrical.Controls.Generic.GenericAutomatonConstants;
  parameter Types.Time SamplingTime "Automaton sampling time";
  parameter Integer NbInputs "Number of required inputs data for the automaton";
  parameter Integer NbOutputs "Number of required outputs data from the automaton";
  parameter String InputsName[GenericAutomatonConstants.inputsMaxSize] = {"EMPTY" for i in 1:GenericAutomatonConstants.inputsMaxSize} "Names of required inputs data for the automaton";
  parameter String OutputsName[GenericAutomatonConstants.outputsMaxSize] = {"EMPTY" for i in 1:GenericAutomatonConstants.outputsMaxSize} "Names of required outputs data from the automaton";
  
  Types.Time t0(start = 0) "First time when the automaton will act";
  Real inputs[GenericAutomatonConstants.inputsMaxSize] "Inputs provided to the automaton";
  Real outputs[GenericAutomatonConstants.outputsMaxSize] "Outputs got from the automaton";
  Boolean outputs_boolean[GenericAutomatonConstants.outputsMaxSize] "Outputs got from the automaton";

equation
  when time >= pre(t0) + SamplingTime then
    (outputs) = Functions.CosimulationInterface(t0, inputs, InputsName, NbInputs, GenericAutomatonConstants.inputsMaxSize, OutputsName, NbOutputs,GenericAutomatonConstants.outputsMaxSize);

    t0 = pre(t0) + SamplingTime;
  end when;

algorithm
  for i in 1:GenericAutomatonConstants.outputsMaxSize loop
    outputs_boolean[i] := (outputs[i] > 0);
  end for;

  annotation(preferredView = "text",
    Documentation(info = "<html><head></head><body>This model enables to call an external C method representing the behavior of any control system. For example, it could be used to call every few seconds an OPF that will change the system state, according to some objective function.</body></html>"));
end CosimulationAutomaton;
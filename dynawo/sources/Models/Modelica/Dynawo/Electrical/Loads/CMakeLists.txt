# Copyright (c) 2015-2019, RTE (http://www.rte-france.com)
# See AUTHORS.txt
# All rights reserved.
# This Source Code Form is subject to the terms of the Mozilla Public
# License, v. 2.0. If a copy of the MPL was not distributed with this
# file, you can obtain one at http://mozilla.org/MPL/2.0/.
# SPDX-License-Identifier: MPL-2.0
#
# This file is part of Dynawo, an hybrid C++/Modelica open source time domain simulation tool for power systems.

set(MODEL_FILES
  # Electrical Loads models
  package.mo
  package.order
  BaseClasses.mo
  BaseClasses_INIT.mo
  CompositeLoadModel.mo
  CompositeLoadModel.extvar
  CompositeLoadModel_INIT.mo
  ElectronicLoad.mo
  ElectronicLoad.extvar
  LoadAlphaBeta.mo
  LoadAlphaBeta.extvar
  LoadAlphaBetaRestorative.mo
  LoadAlphaBetaRestorative.extvar
  LoadAlphaBetaRestorativeLimitsRecalc_INIT.mo
  LoadAlphaBetaMotor.mo
  LoadAlphaBetaMotor.extvar
  LoadAlphaBetaMotor_INIT.mo
  LoadAlphaBetaMotorFifthOrder.mo
  LoadAlphaBetaMotorFifthOrder.extvar
  LoadAlphaBetaMotorFifthOrder_INIT.mo
  LoadAlphaBetaTwoMotorFifthOrder.mo
  LoadAlphaBetaTwoMotorFifthOrder.extvar
  LoadAlphaBetaTwoMotorFifthOrder_INIT.mo
  LoadAlphaBetaMotorSimplified.mo
  LoadAlphaBetaMotorSimplified.extvar
  LoadAlphaBetaMotorSimplified_INIT.mo
  LoadAlphaBetaTwoMotorSimplified.mo
  LoadAlphaBetaTwoMotorSimplified.extvar
  LoadAlphaBetaTwoMotorSimplified_INIT.mo
  LoadPQ.mo
  LoadPQ.extvar
  LoadZIP.mo
  LoadZIP.extvar
  Load_INIT.mo
  LoadPF_INIT.mo
  LoadConnect_INIT.mo
  LoadAuxiliaries_INIT.mo
  SwitchOffLoad.extvar
  )

#
# Modelica models install
#
foreach(MODEL_FILE ${MODEL_FILES})
  install_model_file(${MODEL_FILE})
endforeach(MODEL_FILE)

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
  package.mo
  package.order
  BaseClasses.mo
  DistanceProtectionLineSide1.mo
  DistanceProtectionLineSide1.extvar
  DistanceProtectionLineSide1LoadBlinder.mo
  DistanceProtectionLineSide1LoadBlinder.extvar
  DistanceProtectionLineSide2.mo
  DistanceProtectionLineSide2.extvar
  DistanceProtectionLineSide2LoadBlinder.mo
  DistanceProtectionLineSide2LoadBlinder.extvar
  UFLS10Steps.mo
  UFLS10Steps.extvar
  UFLS3Steps.mo
  UFLS3Steps.extvar
  )

#
# Modelica models install
#
foreach(MODEL_FILE ${MODEL_FILES})
  install_model_file(${MODEL_FILE})
endforeach(MODEL_FILE)

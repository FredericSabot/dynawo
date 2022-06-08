<!--
    Copyright (c) 2015-2019, RTE (http://www.rte-france.com)
    See AUTHORS.txt
    All rights reserved.
    This Source Code Form is subject to the terms of the Mozilla Public
    License, v. 2.0. If a copy of the MPL was not distributed with this
    file, you can obtain one at http://mozilla.org/MPL/2.0/.
    SPDX-License-Identifier: MPL-2.0

    This file is part of Dynawo, an hybrid C++/Modelica open source time domain
    simulation tool for power systems.
-->
This branch contains the full model of the Roy Billinton Test System (RBTS) used in the paper titled "Impact of the reliability of ICT systems on power systems with system integrity protection schemes" presented at the 2022 edition of the LambdaMu conference [https://www.imdr-lambdamu.eu/](https://www.imdr-lambdamu.eu/). The data set is in examples/DynaSwing/RBTS/

For instructructions on how to install Dyna&omega;o, please refer to: [http://dynawo.org](http://dynawo.org). The data set contains model that are not included in the standard Dynawo library. You thus have to compile this branch from source.

## Static data

The original static data is in "RBTS_ACOPF_peak.iidm". The modified versions named "RBTS_ACOPF_XXX_L2_shunt_typical.iidm" where "XXX" stands for the total load of the system (peak = 185MW). The modifications consist in

- Doubling the line lengths
- Using more realistic line capacitances
- Adding shunt compensation to maintain acceptable voltages (0.95 < V < 1.05)

The "RBTS.py" is a [pandapower](http://www.pandapower.org/) file used to generate the different versions. Due to the limited capacities of pandapower, some constraints have been added "manually" (i.e. have to be changed for each load level):

- Switch on/off generators
- Minimum spinning reserve

Pandapower currently cannot directly export file to .iidm format. Thus, you have to export to .mat format, then convert it using [PowSyBl](https://www.powsybl.org/) using the command

``` bash
itools loadflow --case-file RBTS_ACOPF_peak.mat --output-case-file RBTS_ACOPF_peak.xiidm --output-case-format XIIDM
```

But first, you should add

``` bash
import-export-parameters-default-value:
  matpower.import.ignore-base-voltage: false
```

to the config.yml file, otherwise PowSyBl sets all base voltages to 1.

## Dynamic data

The dynamic models used are listed in "RBTS.dyd". The hydro governor model is based on [1]. The exciter and coal governor models are the same as in [2]. Typical values are used for the parameters of the synchronous machines, those are listed in [3] and given in "RBTS.par". The reduced inertia version is given in "RBTS_lowinertia.par".

It should be noted that at low load levels, the unused generators (and their connections) should be removed from the .dyd files.

## Quoting this data set

If you use this version of the RBTS in your work, please quote the following paper

``` bash
@inproceedings{DynamicRBTS,
author = {Sabot, Frédéric and Henneaux, Pierre and Labeau, Pierre-Etienne and Dricot, Jean-Michel},
booktitle={\(\lambda\mu23\) congress},
year = {2022},
title = {Impact of the reliability of {ICT} systems on power systems with system integrity protection schemes},
doi = {To be added after publication}
}
```

## Quoting Dynawo

Please refer to [https://dynawo.github.io/publications/](https://dynawo.github.io/publications/)

## References

[1] W. -x. Liu, N. Liu, Y. -f. Fan, L. -x. Zhang and x. Zhang, "Reliability analysis of Wide Area Measurement System based on the Centralized Distributed model," 2009 IEEE/PES Power Systems Conference and Exposition, 2009, pp. 1-6, doi: 10.1109/PSCE.2009.4840137.
[2] P. Demetriou, M. Asprou, J. Quirós-Tortós, and E. Kyriakides, “Dynamic IEEE Test Systems for Transient Analysis,” IEEE Systems Journal, vol. 11, no. 4, pp. 2108-2117, Dec. 2017.
[3] Vijay Vittal, James D. McCalley. Power System Control and Stability, third edition. Annex D "Typical System Data"

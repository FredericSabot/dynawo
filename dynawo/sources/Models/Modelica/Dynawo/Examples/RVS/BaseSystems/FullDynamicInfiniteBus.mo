within Dynawo.Examples.RVS.BaseSystems;

/*
* Copyright (c) 2023, RTE (http://www.rte-france.com)
* See AUTHORS.txt
* All rights reserved.
* This Source Code Form is subject to the terms of the Mozilla Public
* License, v. 2.0. If a copy of the MPL was not distributed with this
* file, you can obtain one at http://mozilla.org/MPL/2.0/.
* SPDX-License-Identifier: MPL-2.0
*
* This file is part of Dynawo, an hybrid C++/Modelica open source suite
* of simulation tools for power systems.
*/

model FullDynamicInfiniteBus
  import Modelica.SIunits.Conversions.from_deg;
  import Dynawo;
  import Dynawo.Electrical.SystemBase.SnRef;

  extends NetworkWithTrfRestorative;

  parameter Types.ActivePowerPu P0Pu_sVarC_10106_ALBER_SVC;
  parameter Types.ReactivePowerPu Q0Pu_sVarC_10106_ALBER_SVC;
  parameter Types.VoltageModulePu U0Pu_sVarC_10106_ALBER_SVC;
  parameter Types.Angle UPhase0_sVarC_10106_ALBER_SVC;
  parameter Types.ActivePowerPu P0Pu_sVarC_10114_ARNOLD_SVC;
  parameter Types.ReactivePowerPu Q0Pu_sVarC_10114_ARNOLD_SVC;
  parameter Types.VoltageModulePu U0Pu_sVarC_10114_ARNOLD_SVC;
  parameter Types.Angle UPhase0_sVarC_10114_ARNOLD_SVC;
  // Generators
  parameter Types.ActivePowerPu P0Pu_gen_10101_ABEL_G1;
  parameter Types.ReactivePowerPu Q0Pu_gen_10101_ABEL_G1;
  parameter Types.VoltageModulePu U0Pu_gen_10101_ABEL_G1;
  parameter Types.Angle UPhase0_gen_10101_ABEL_G1;
  parameter Types.ActivePowerPu P0Pu_gen_10102_ADAMS_G1;
  parameter Types.ReactivePowerPu Q0Pu_gen_10102_ADAMS_G1;
  parameter Types.VoltageModulePu U0Pu_gen_10102_ADAMS_G1;
  parameter Types.Angle UPhase0_gen_10102_ADAMS_G1;
  parameter Types.ActivePowerPu P0Pu_gen_10107_ALDER_G1;
  parameter Types.ReactivePowerPu Q0Pu_gen_10107_ALDER_G1;
  parameter Types.VoltageModulePu U0Pu_gen_10107_ALDER_G1;
  parameter Types.Angle UPhase0_gen_10107_ALDER_G1;
  parameter Types.ActivePowerPu P0Pu_gen_10113_ARNE_G1;
  parameter Types.ReactivePowerPu Q0Pu_gen_10113_ARNE_G1;
  parameter Types.VoltageModulePu U0Pu_gen_10113_ARNE_G1;
  parameter Types.Angle UPhase0_gen_10113_ARNE_G1;
  parameter Types.ActivePowerPu P0Pu_gen_10115_ARTHUR_G1;
  parameter Types.ReactivePowerPu Q0Pu_gen_10115_ARTHUR_G1;
  parameter Types.VoltageModulePu U0Pu_gen_10115_ARTHUR_G1;
  parameter Types.Angle UPhase0_gen_10115_ARTHUR_G1;
  parameter Types.ActivePowerPu P0Pu_gen_10116_ASSER_G1;
  parameter Types.ReactivePowerPu Q0Pu_gen_10116_ASSER_G1;
  parameter Types.VoltageModulePu U0Pu_gen_10116_ASSER_G1;
  parameter Types.Angle UPhase0_gen_10116_ASSER_G1;
  parameter Types.ActivePowerPu P0Pu_gen_10118_ASTOR_G1;
  parameter Types.ReactivePowerPu Q0Pu_gen_10118_ASTOR_G1;
  parameter Types.VoltageModulePu U0Pu_gen_10118_ASTOR_G1;
  parameter Types.Angle UPhase0_gen_10118_ASTOR_G1;
  parameter Types.ActivePowerPu P0Pu_gen_10122_AUBREY_G1;
  parameter Types.ReactivePowerPu Q0Pu_gen_10122_AUBREY_G1;
  parameter Types.VoltageModulePu U0Pu_gen_10122_AUBREY_G1;
  parameter Types.Angle UPhase0_gen_10122_AUBREY_G1;
  parameter Types.ActivePowerPu P0Pu_gen_10123_AUSTEN_G1;
  parameter Types.ReactivePowerPu Q0Pu_gen_10123_AUSTEN_G1;
  parameter Types.VoltageModulePu U0Pu_gen_10123_AUSTEN_G1;
  parameter Types.Angle UPhase0_gen_10123_AUSTEN_G1;
  parameter Types.ActivePowerPu P0Pu_gen_20101_ABEL_G2;
  parameter Types.ReactivePowerPu Q0Pu_gen_20101_ABEL_G2;
  parameter Types.VoltageModulePu U0Pu_gen_20101_ABEL_G2;
  parameter Types.Angle UPhase0_gen_20101_ABEL_G2;
  parameter Types.ActivePowerPu P0Pu_gen_20102_ADAMS_G2;
  parameter Types.ReactivePowerPu Q0Pu_gen_20102_ADAMS_G2;
  parameter Types.VoltageModulePu U0Pu_gen_20102_ADAMS_G2;
  parameter Types.Angle UPhase0_gen_20102_ADAMS_G2;
  parameter Types.ActivePowerPu P0Pu_gen_20107_ALDER_G2;
  parameter Types.ReactivePowerPu Q0Pu_gen_20107_ALDER_G2;
  parameter Types.VoltageModulePu U0Pu_gen_20107_ALDER_G2;
  parameter Types.Angle UPhase0_gen_20107_ALDER_G2;
  parameter Types.ActivePowerPu P0Pu_gen_20113_ARNE_G2;
  parameter Types.ReactivePowerPu Q0Pu_gen_20113_ARNE_G2;
  parameter Types.VoltageModulePu U0Pu_gen_20113_ARNE_G2;
  parameter Types.Angle UPhase0_gen_20113_ARNE_G2;
  parameter Types.ActivePowerPu P0Pu_gen_20115_ARTHUR_G2;
  parameter Types.ReactivePowerPu Q0Pu_gen_20115_ARTHUR_G2;
  parameter Types.VoltageModulePu U0Pu_gen_20115_ARTHUR_G2;
  parameter Types.Angle UPhase0_gen_20115_ARTHUR_G2;
  parameter Types.ActivePowerPu P0Pu_gen_20122_AUBREY_G2;
  parameter Types.ReactivePowerPu Q0Pu_gen_20122_AUBREY_G2;
  parameter Types.VoltageModulePu U0Pu_gen_20122_AUBREY_G2;
  parameter Types.Angle UPhase0_gen_20122_AUBREY_G2;
  parameter Types.ActivePowerPu P0Pu_gen_20123_AUSTEN_G2;
  parameter Types.ReactivePowerPu Q0Pu_gen_20123_AUSTEN_G2;
  parameter Types.VoltageModulePu U0Pu_gen_20123_AUSTEN_G2;
  parameter Types.Angle UPhase0_gen_20123_AUSTEN_G2;
  parameter Types.ActivePowerPu P0Pu_gen_30101_ABEL_G3;
  parameter Types.ReactivePowerPu Q0Pu_gen_30101_ABEL_G3;
  parameter Types.VoltageModulePu U0Pu_gen_30101_ABEL_G3;
  parameter Types.Angle UPhase0_gen_30101_ABEL_G3;
  parameter Types.ActivePowerPu P0Pu_gen_30102_ADAMS_G3;
  parameter Types.ReactivePowerPu Q0Pu_gen_30102_ADAMS_G3;
  parameter Types.VoltageModulePu U0Pu_gen_30102_ADAMS_G3;
  parameter Types.Angle UPhase0_gen_30102_ADAMS_G3;
  parameter Types.ActivePowerPu P0Pu_gen_30107_ALDER_G3;
  parameter Types.ReactivePowerPu Q0Pu_gen_30107_ALDER_G3;
  parameter Types.VoltageModulePu U0Pu_gen_30107_ALDER_G3;
  parameter Types.Angle UPhase0_gen_30107_ALDER_G3;
  parameter Types.ActivePowerPu P0Pu_gen_30113_ARNE_G3;
  parameter Types.ReactivePowerPu Q0Pu_gen_30113_ARNE_G3;
  parameter Types.VoltageModulePu U0Pu_gen_30113_ARNE_G3;
  parameter Types.Angle UPhase0_gen_30113_ARNE_G3;
  parameter Types.ActivePowerPu P0Pu_gen_30115_ARTHUR_G3;
  parameter Types.ReactivePowerPu Q0Pu_gen_30115_ARTHUR_G3;
  parameter Types.VoltageModulePu U0Pu_gen_30115_ARTHUR_G3;
  parameter Types.Angle UPhase0_gen_30115_ARTHUR_G3;
  parameter Types.ActivePowerPu P0Pu_gen_30122_AUBREY_G3;
  parameter Types.ReactivePowerPu Q0Pu_gen_30122_AUBREY_G3;
  parameter Types.VoltageModulePu U0Pu_gen_30122_AUBREY_G3;
  parameter Types.Angle UPhase0_gen_30122_AUBREY_G3;
  parameter Types.ActivePowerPu P0Pu_gen_30123_AUSTEN_G3;
  parameter Types.ReactivePowerPu Q0Pu_gen_30123_AUSTEN_G3;
  parameter Types.VoltageModulePu U0Pu_gen_30123_AUSTEN_G3;
  parameter Types.Angle UPhase0_gen_30123_AUSTEN_G3;
  parameter Types.ActivePowerPu P0Pu_gen_40101_ABEL_G4;
  parameter Types.ReactivePowerPu Q0Pu_gen_40101_ABEL_G4;
  parameter Types.VoltageModulePu U0Pu_gen_40101_ABEL_G4;
  parameter Types.Angle UPhase0_gen_40101_ABEL_G4;
  parameter Types.ActivePowerPu P0Pu_gen_40102_ADAMS_G4;
  parameter Types.ReactivePowerPu Q0Pu_gen_40102_ADAMS_G4;
  parameter Types.VoltageModulePu U0Pu_gen_40102_ADAMS_G4;
  parameter Types.Angle UPhase0_gen_40102_ADAMS_G4;
  parameter Types.ActivePowerPu P0Pu_gen_40115_ARTHUR_G4;
  parameter Types.ReactivePowerPu Q0Pu_gen_40115_ARTHUR_G4;
  parameter Types.VoltageModulePu U0Pu_gen_40115_ARTHUR_G4;
  parameter Types.Angle UPhase0_gen_40115_ARTHUR_G4;
  parameter Types.ActivePowerPu P0Pu_gen_40122_AUBREY_G4;
  parameter Types.ReactivePowerPu Q0Pu_gen_40122_AUBREY_G4;
  parameter Types.VoltageModulePu U0Pu_gen_40122_AUBREY_G4;
  parameter Types.Angle UPhase0_gen_40122_AUBREY_G4;
  parameter Types.ActivePowerPu P0Pu_gen_50115_ARTHUR_G5;
  parameter Types.ReactivePowerPu Q0Pu_gen_50115_ARTHUR_G5;
  parameter Types.VoltageModulePu U0Pu_gen_50115_ARTHUR_G5;
  parameter Types.Angle UPhase0_gen_50115_ARTHUR_G5;
  parameter Types.ActivePowerPu P0Pu_gen_50122_AUBREY_G5;
  parameter Types.ReactivePowerPu Q0Pu_gen_50122_AUBREY_G5;
  parameter Types.VoltageModulePu U0Pu_gen_50122_AUBREY_G5;
  parameter Types.Angle UPhase0_gen_50122_AUBREY_G5;
  parameter Types.ActivePowerPu P0Pu_gen_60115_ARTHUR_G6;
  parameter Types.ReactivePowerPu Q0Pu_gen_60115_ARTHUR_G6;
  parameter Types.VoltageModulePu U0Pu_gen_60115_ARTHUR_G6;
  parameter Types.Angle UPhase0_gen_60115_ARTHUR_G6;
  parameter Types.ActivePowerPu P0Pu_gen_60122_AUBREY_G6;
  parameter Types.ReactivePowerPu Q0Pu_gen_60122_AUBREY_G6;
  parameter Types.VoltageModulePu U0Pu_gen_60122_AUBREY_G6;
  parameter Types.Angle UPhase0_gen_60122_AUBREY_G6;

  /*
  final constant Types.ActivePowerPu P0Pu_sVarC_10106_ALBER_SVC = -4.0495384823202585e-14;
  final constant Types.ReactivePowerPu Q0Pu_sVarC_10106_ALBER_SVC = -0.6164618807059862;
  final constant Types.VoltageModulePu U0Pu_sVarC_10106_ALBER_SVC = 1.049999952316467;
  final constant Types.Angle UPhase0_sVarC_10106_ALBER_SVC = -0.4170880483313303;
  final constant Types.ActivePowerPu P0Pu_sVarC_10114_ARNOLD_SVC = -8.97615315409439e-14;
  final constant Types.ReactivePowerPu Q0Pu_sVarC_10114_ARNOLD_SVC = -1.254762105260428;
  final constant Types.VoltageModulePu U0Pu_sVarC_10114_ARNOLD_SVC = 1.049999952316385;
  final constant Types.Angle UPhase0_sVarC_10114_ARNOLD_SVC = -0.17175578394927937;
  // Generators
  final constant Types.ActivePowerPu P0Pu_gen_10101_ABEL_G1 = -0.09999999999492322;
  final constant Types.ReactivePowerPu Q0Pu_gen_10101_ABEL_G1 = -0.0747006931432781;
  final constant Types.VoltageModulePu U0Pu_gen_10101_ABEL_G1 = 1.0296020605827343;
  final constant Types.Angle UPhase0_gen_10101_ABEL_G1 = -0.26528375249349523;
  final constant Types.ActivePowerPu P0Pu_gen_10102_ADAMS_G1 = -0.09999999999494089;
  final constant Types.ReactivePowerPu Q0Pu_gen_10102_ADAMS_G1 = -0.10294511872209362;
  final constant Types.VoltageModulePu U0Pu_gen_10102_ADAMS_G1 = 1.047286860269937;
  final constant Types.Angle UPhase0_gen_10102_ADAMS_G1 = -0.26873055958340697;
  final constant Types.ActivePowerPu P0Pu_gen_10107_ALDER_G1 = -0.7999999999726052;
  final constant Types.ReactivePowerPu Q0Pu_gen_10107_ALDER_G1 = -0.5106708523802469;
  final constant Types.VoltageModulePu U0Pu_gen_10107_ALDER_G1 = 1.0392093432573648;
  final constant Types.Angle UPhase0_gen_10107_ALDER_G1 = -0.2909981796235494;
  final constant Types.ActivePowerPu P0Pu_gen_10113_ARNE_G1 = -1.6250000001927132;
  final constant Types.ReactivePowerPu Q0Pu_gen_10113_ARNE_G1 = -0.0378376197136411;
  final constant Types.VoltageModulePu U0Pu_gen_10113_ARNE_G1 = 0.9699722451585717;
  final constant Types.Angle UPhase0_gen_10113_ARNE_G1 = 0.00410841273654951;
  final constant Types.ActivePowerPu P0Pu_gen_10115_ARTHUR_G1 = -0.11999999999728818;
  final constant Types.ReactivePowerPu Q0Pu_gen_10115_ARTHUR_G1 = -0.06383030028772678;
  final constant Types.VoltageModulePu U0Pu_gen_10115_ARTHUR_G1 = 1.0237450113009863;
  final constant Types.Angle UPhase0_gen_10115_ARTHUR_G1 = 0.14591107650805835;
  final constant Types.ActivePowerPu P0Pu_gen_10116_ASSER_G1 = -1.550000148856329;
  final constant Types.ReactivePowerPu Q0Pu_gen_10116_ASSER_G1 = -0.5361276776730247;
  final constant Types.VoltageModulePu U0Pu_gen_10116_ASSER_G1 = 1.005707605152536;
  final constant Types.Angle UPhase0_gen_10116_ASSER_G1 = 0.13728705789917342;
  final constant Types.ActivePowerPu P0Pu_gen_10118_ASTOR_G1 = -4.000000355338675;
  final constant Types.ReactivePowerPu Q0Pu_gen_10118_ASTOR_G1 = -2.0301805325991826;
  final constant Types.VoltageModulePu U0Pu_gen_10118_ASTOR_G1 = 1.049501524085332;
  final constant Types.Angle UPhase0_gen_10118_ASTOR_G1 = 0.219179365403202;
  final constant Types.ActivePowerPu P0Pu_gen_10122_AUBREY_G1 = -0.5000004099960184;
  final constant Types.ReactivePowerPu Q0Pu_gen_10122_AUBREY_G1 = -0.036463694362460664;
  final constant Types.VoltageModulePu U0Pu_gen_10122_AUBREY_G1 = 1.0025914159367653;
  final constant Types.Angle UPhase0_gen_10122_AUBREY_G1 = 0.35407238234960037;
  final constant Types.ActivePowerPu P0Pu_gen_10123_AUSTEN_G1 = -1.550000063473528;
  final constant Types.ReactivePowerPu Q0Pu_gen_10123_AUSTEN_G1 = -0.7087170883544405;
  final constant Types.VoltageModulePu U0Pu_gen_10123_AUSTEN_G1 = 1.0505080247133123;
  final constant Types.Angle UPhase0_gen_10123_AUSTEN_G1 = 0.15217939470869046;
  final constant Types.ActivePowerPu P0Pu_gen_20101_ABEL_G2 = -0.09999999999492801;
  final constant Types.ReactivePowerPu Q0Pu_gen_20101_ABEL_G2 = -0.07470069314325702;
  final constant Types.VoltageModulePu U0Pu_gen_20101_ABEL_G2 = 1.029602060582723;
  final constant Types.Angle UPhase0_gen_20101_ABEL_G2 = -0.265283752493495;
  final constant Types.ActivePowerPu P0Pu_gen_20102_ADAMS_G2 = -0.09999999999493148;
  final constant Types.ReactivePowerPu Q0Pu_gen_20102_ADAMS_G2 = -0.10294511872206842;
  final constant Types.VoltageModulePu U0Pu_gen_20102_ADAMS_G2 = 1.047286860269925;
  final constant Types.Angle UPhase0_gen_20102_ADAMS_G2 = -0.2687305595834085;
  final constant Types.ActivePowerPu P0Pu_gen_20107_ALDER_G2 = -0.7999999999725957;
  final constant Types.ReactivePowerPu Q0Pu_gen_20107_ALDER_G2 = -0.5107086749439578;
  final constant Types.VoltageModulePu U0Pu_gen_20107_ALDER_G2 = 1.0392137465540379;
  final constant Types.Angle UPhase0_gen_20107_ALDER_G2 = -0.29099869448046284;
  final constant Types.ActivePowerPu P0Pu_gen_20113_ARNE_G2 = -1.6249999999306455;
  final constant Types.ReactivePowerPu Q0Pu_gen_20113_ARNE_G2 = -1.2008084007899797;
  final constant Types.VoltageModulePu U0Pu_gen_20113_ARNE_G2 = 1.0427229897892534;
  final constant Types.Angle UPhase0_gen_20113_ARNE_G2 = -0.005206386553900649;
  final constant Types.ActivePowerPu P0Pu_gen_20115_ARTHUR_G2 = -0.11999999999728826;
  final constant Types.ReactivePowerPu Q0Pu_gen_20115_ARTHUR_G2 = -0.06383030028773819;
  final constant Types.VoltageModulePu U0Pu_gen_20115_ARTHUR_G2 = 1.023745011300991;
  final constant Types.Angle UPhase0_gen_20115_ARTHUR_G2 = 0.14591107650805607;
  final constant Types.ActivePowerPu P0Pu_gen_20122_AUBREY_G2 = -0.5000004099960331;
  final constant Types.ReactivePowerPu Q0Pu_gen_20122_AUBREY_G2 = -0.03646369436252084;
  final constant Types.VoltageModulePu U0Pu_gen_20122_AUBREY_G2 = 1.002591415936771;
  final constant Types.Angle UPhase0_gen_20122_AUBREY_G2 = 0.3540723823496088;
  final constant Types.ActivePowerPu P0Pu_gen_20123_AUSTEN_G2 = -1.5500000634748163;
  final constant Types.ReactivePowerPu Q0Pu_gen_20123_AUSTEN_G2 = -0.7087170883555296;
  final constant Types.VoltageModulePu U0Pu_gen_20123_AUSTEN_G2 = 1.0505080247133731;
  final constant Types.Angle UPhase0_gen_20123_AUSTEN_G2 = 0.15217939470878503;
  final constant Types.ActivePowerPu P0Pu_gen_30101_ABEL_G3 = -0.7599999999788776;
  final constant Types.ReactivePowerPu Q0Pu_gen_30101_ABEL_G3 = -0.22178314963648094;
  final constant Types.VoltageModulePu U0Pu_gen_30101_ABEL_G3 = 1.0161596964727235;
  final constant Types.Angle UPhase0_gen_30101_ABEL_G3 = -0.1984468778479227;
  final constant Types.ActivePowerPu P0Pu_gen_30102_ADAMS_G3 = -0.7599999999782525;
  final constant Types.ReactivePowerPu Q0Pu_gen_30102_ADAMS_G3 = -0.18651193763838703;
  final constant Types.VoltageModulePu U0Pu_gen_30102_ADAMS_G3 = 1.0119115875875173;
  final constant Types.Angle UPhase0_gen_30102_ADAMS_G3 = -0.1999727128428949;
  final constant Types.ActivePowerPu P0Pu_gen_30107_ALDER_G3 = -0.7999999999726064;
  final constant Types.ReactivePowerPu Q0Pu_gen_30107_ALDER_G3 = -0.5106708523801484;
  final constant Types.VoltageModulePu U0Pu_gen_30107_ALDER_G3 = 1.0392093432573528;
  final constant Types.Angle UPhase0_gen_30107_ALDER_G3 = -0.29099817962354707;
  final constant Types.ActivePowerPu P0Pu_gen_30113_ARNE_G3 = -1.6249999999307339;
  final constant Types.ReactivePowerPu Q0Pu_gen_30113_ARNE_G3 = -1.2008829225627582;
  final constant Types.VoltageModulePu U0Pu_gen_30113_ARNE_G3 = 1.042727343363208;
  final constant Types.Angle UPhase0_gen_30113_ARNE_G3 = -0.005206911147618021;
  final constant Types.ActivePowerPu P0Pu_gen_30115_ARTHUR_G3 = -0.11999999999728914;
  final constant Types.ReactivePowerPu Q0Pu_gen_30115_ARTHUR_G3 = -0.0638303002877329;
  final constant Types.VoltageModulePu U0Pu_gen_30115_ARTHUR_G3 = 1.023745011300996;
  final constant Types.Angle UPhase0_gen_30115_ARTHUR_G3 = 0.14591107650805565;
  final constant Types.ActivePowerPu P0Pu_gen_30122_AUBREY_G3 = -0.5000004099960347;
  final constant Types.ReactivePowerPu Q0Pu_gen_30122_AUBREY_G3 = -0.03646369436251523;
  final constant Types.VoltageModulePu U0Pu_gen_30122_AUBREY_G3 = 1.0025914159367775;
  final constant Types.Angle UPhase0_gen_30122_AUBREY_G3 = 0.35407238234960403;
  final constant Types.ActivePowerPu P0Pu_gen_30123_AUSTEN_G3 = -3.5000001656289816;
  final constant Types.ReactivePowerPu Q0Pu_gen_30123_AUSTEN_G3 = -1.3305380937734208;
  final constant Types.VoltageModulePu U0Pu_gen_30123_AUSTEN_G3 = 1.041325051732186;
  final constant Types.Angle UPhase0_gen_30123_AUSTEN_G3 = 0.1531333025151461;
  final constant Types.ActivePowerPu P0Pu_gen_40101_ABEL_G4 = -0.7599999999788802;
  final constant Types.ReactivePowerPu Q0Pu_gen_40101_ABEL_G4 = -0.22175966597865165;
  final constant Types.VoltageModulePu U0Pu_gen_40101_ABEL_G4 = 1.0161558787590688;
  final constant Types.Angle UPhase0_gen_40101_ABEL_G4 = -0.19844631612764815;
  final constant Types.ActivePowerPu P0Pu_gen_40102_ADAMS_G4 = -0.7599999999782637;
  final constant Types.ReactivePowerPu Q0Pu_gen_40102_ADAMS_G4 = -0.18651193763830845;
  final constant Types.VoltageModulePu U0Pu_gen_40102_ADAMS_G4 = 1.0119115875874976;
  final constant Types.Angle UPhase0_gen_40102_ADAMS_G4 = -0.19997271284288592;
  final constant Types.ActivePowerPu P0Pu_gen_40115_ARTHUR_G4 = -0.1199999999972882;
  final constant Types.ReactivePowerPu Q0Pu_gen_40115_ARTHUR_G4 = -0.06383987947986561;
  final constant Types.VoltageModulePu U0Pu_gen_40115_ARTHUR_G4 = 1.023754566693637;
  final constant Types.Angle UPhase0_gen_40115_ARTHUR_G4 = 0.1459096513171526;
  final constant Types.ActivePowerPu P0Pu_gen_40122_AUBREY_G4 = -0.5000004099960342;
  final constant Types.ReactivePowerPu Q0Pu_gen_40122_AUBREY_G4 = -0.03646369436253674;
  final constant Types.VoltageModulePu U0Pu_gen_40122_AUBREY_G4 = 1.002591415936775;
  final constant Types.Angle UPhase0_gen_40122_AUBREY_G4 = 0.3540723823496089;
  final constant Types.ActivePowerPu P0Pu_gen_50115_ARTHUR_G5 = -0.1199999999972885;
  final constant Types.ReactivePowerPu Q0Pu_gen_50115_ARTHUR_G5 = -0.06383030028772911;
  final constant Types.VoltageModulePu U0Pu_gen_50115_ARTHUR_G5 = 1.0237450113009892;
  final constant Types.Angle UPhase0_gen_50115_ARTHUR_G5 = 0.14591107650805663;
  final constant Types.ActivePowerPu P0Pu_gen_50122_AUBREY_G5 = -0.5000004099960343;
  final constant Types.ReactivePowerPu Q0Pu_gen_50122_AUBREY_G5 = -0.0364636943624852;
  final constant Types.VoltageModulePu U0Pu_gen_50122_AUBREY_G5 = 1.0025914159367704;
  final constant Types.Angle UPhase0_gen_50122_AUBREY_G5 = 0.35407238234960603;
  final constant Types.ActivePowerPu P0Pu_gen_60115_ARTHUR_G6 = -1.550000141046319;
  final constant Types.ReactivePowerPu Q0Pu_gen_60115_ARTHUR_G6 = -0.8602969818617706;
  final constant Types.VoltageModulePu U0Pu_gen_60115_ARTHUR_G6 = 1.0261638226400809;
  final constant Types.Angle UPhase0_gen_60115_ARTHUR_G6 = 0.14471146857702907;
  final constant Types.ActivePowerPu P0Pu_gen_60122_AUBREY_G6 = -0.5000004099960347;
  final constant Types.ReactivePowerPu Q0Pu_gen_60122_AUBREY_G6 = -0.03646369436248653;
  final constant Types.VoltageModulePu U0Pu_gen_60122_AUBREY_G6 = 1.00259141593677;
  final constant Types.Angle UPhase0_gen_60122_AUBREY_G6 = 0.3540723823496069;*/

  Components.GeneratorWithControl.SteamEXACFrame gen_40101_ABEL_G4(P0Pu = P0Pu_gen_40101_ABEL_G4, Q0Pu = Q0Pu_gen_40101_ABEL_G4, U0Pu = U0Pu_gen_40101_ABEL_G4, UPhase0 = UPhase0_gen_40101_ABEL_G4, exac1Preset = Dynawo.Examples.RVS.Components.GeneratorWithControl.Util.ParametersEXAC.genFramePreset.g40101, gen = Dynawo.Examples.RVS.Components.GeneratorWithControl.Util.ParametersGenerators.genFramePreset.g40101, ieeeg1Preset = Dynawo.Examples.RVS.Components.GeneratorWithControl.Util.ParametersIEEEG1.genFramePreset.g40101, oelPreset = Dynawo.Examples.RVS.Components.GeneratorWithControl.Util.ParametersOEL.oelFramePreset.all, pss2bPreset = Dynawo.Examples.RVS.Components.GeneratorWithControl.Util.ParametersPSS2B.genFramePreset.g40101) annotation(
    Placement(visible = true, transformation(origin = {-128, -266}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Components.GeneratorWithControl.SteamEXACFrame gen_30101_ABEL_G3(P0Pu = P0Pu_gen_30101_ABEL_G3, Q0Pu = Q0Pu_gen_30101_ABEL_G3, U0Pu = U0Pu_gen_30101_ABEL_G3, UPhase0 = UPhase0_gen_30101_ABEL_G3, exac1Preset = Dynawo.Examples.RVS.Components.GeneratorWithControl.Util.ParametersEXAC.genFramePreset.g30101, gen = Dynawo.Examples.RVS.Components.GeneratorWithControl.Util.ParametersGenerators.genFramePreset.g30101, ieeeg1Preset = Dynawo.Examples.RVS.Components.GeneratorWithControl.Util.ParametersIEEEG1.genFramePreset.g30101, oelPreset = Dynawo.Examples.RVS.Components.GeneratorWithControl.Util.ParametersOEL.oelFramePreset.all, pss2bPreset = Dynawo.Examples.RVS.Components.GeneratorWithControl.Util.ParametersPSS2B.genFramePreset.g30101) annotation(
    Placement(visible = true, transformation(origin = {-168, -266}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Components.GeneratorWithControl.SteamEXACFrame gen_10101_ABEL_G1(P0Pu = P0Pu_gen_10101_ABEL_G1, Q0Pu = Q0Pu_gen_10101_ABEL_G1, U0Pu = U0Pu_gen_10101_ABEL_G1, UPhase0 = UPhase0_gen_10101_ABEL_G1, exac1Preset = Dynawo.Examples.RVS.Components.GeneratorWithControl.Util.ParametersEXAC.genFramePreset.g10101, gen = Dynawo.Examples.RVS.Components.GeneratorWithControl.Util.ParametersGenerators.genFramePreset.g10101, ieeeg1Preset = Dynawo.Examples.RVS.Components.GeneratorWithControl.Util.ParametersIEEEG1.genFramePreset.g10101, oelPreset = Dynawo.Examples.RVS.Components.GeneratorWithControl.Util.ParametersOEL.oelFramePreset.all, pss2bPreset = Dynawo.Examples.RVS.Components.GeneratorWithControl.Util.ParametersPSS2B.genFramePreset.g10101) annotation(
    Placement(visible = true, transformation(origin = {-248, -266}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Components.GeneratorWithControl.SteamEXACFrame gen_20101_ABEL_G2(P0Pu = P0Pu_gen_20101_ABEL_G2, Q0Pu = Q0Pu_gen_20101_ABEL_G2, U0Pu = U0Pu_gen_20101_ABEL_G2, UPhase0 = UPhase0_gen_20101_ABEL_G2, exac1Preset = Dynawo.Examples.RVS.Components.GeneratorWithControl.Util.ParametersEXAC.genFramePreset.g20101, gen = Dynawo.Examples.RVS.Components.GeneratorWithControl.Util.ParametersGenerators.genFramePreset.g20101, ieeeg1Preset = Dynawo.Examples.RVS.Components.GeneratorWithControl.Util.ParametersIEEEG1.genFramePreset.g20101, oelPreset = Dynawo.Examples.RVS.Components.GeneratorWithControl.Util.ParametersOEL.oelFramePreset.all, pss2bPreset = Dynawo.Examples.RVS.Components.GeneratorWithControl.Util.ParametersPSS2B.genFramePreset.g20101) annotation(
    Placement(visible = true, transformation(origin = {-208, -266}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Dynawo.Examples.RVS.Components.GeneratorWithControl.SteamEXACFrame gen_20102_ADAMS_G2(P0Pu = P0Pu_gen_20102_ADAMS_G2, Q0Pu = Q0Pu_gen_20102_ADAMS_G2, U0Pu = U0Pu_gen_20102_ADAMS_G2, UPhase0 = UPhase0_gen_20102_ADAMS_G2, exac1Preset = Dynawo.Examples.RVS.Components.GeneratorWithControl.Util.ParametersEXAC.genFramePreset.g20102, gen = Dynawo.Examples.RVS.Components.GeneratorWithControl.Util.ParametersGenerators.genFramePreset.g20102, ieeeg1Preset = Dynawo.Examples.RVS.Components.GeneratorWithControl.Util.ParametersIEEEG1.genFramePreset.g20102, oelPreset = Dynawo.Examples.RVS.Components.GeneratorWithControl.Util.ParametersOEL.oelFramePreset.all, pss2bPreset = Dynawo.Examples.RVS.Components.GeneratorWithControl.Util.ParametersPSS2B.genFramePreset.g20102) annotation(
    Placement(visible = true, transformation(origin = {-4, -306}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Dynawo.Examples.RVS.Components.GeneratorWithControl.SteamEXACFrame gen_40102_ADAMS_G4(P0Pu = P0Pu_gen_40102_ADAMS_G4, Q0Pu = Q0Pu_gen_40102_ADAMS_G4, U0Pu = U0Pu_gen_40102_ADAMS_G4, UPhase0 = UPhase0_gen_40102_ADAMS_G4, exac1Preset = Dynawo.Examples.RVS.Components.GeneratorWithControl.Util.ParametersEXAC.genFramePreset.g40102, gen = Dynawo.Examples.RVS.Components.GeneratorWithControl.Util.ParametersGenerators.genFramePreset.g40102, ieeeg1Preset = Dynawo.Examples.RVS.Components.GeneratorWithControl.Util.ParametersIEEEG1.genFramePreset.g40102, oelPreset = Dynawo.Examples.RVS.Components.GeneratorWithControl.Util.ParametersOEL.oelFramePreset.all, pss2bPreset = Dynawo.Examples.RVS.Components.GeneratorWithControl.Util.ParametersPSS2B.genFramePreset.g40102) annotation(
    Placement(visible = true, transformation(origin = {76, -306}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Components.GeneratorWithControl.SteamEXACFrame gen_10102_ADAMS_G1(P0Pu = P0Pu_gen_10102_ADAMS_G1, Q0Pu = Q0Pu_gen_10102_ADAMS_G1, U0Pu = U0Pu_gen_10102_ADAMS_G1, UPhase0 = UPhase0_gen_10102_ADAMS_G1, exac1Preset = Dynawo.Examples.RVS.Components.GeneratorWithControl.Util.ParametersEXAC.genFramePreset.g10102, gen = Dynawo.Examples.RVS.Components.GeneratorWithControl.Util.ParametersGenerators.genFramePreset.g10102, ieeeg1Preset = Dynawo.Examples.RVS.Components.GeneratorWithControl.Util.ParametersIEEEG1.genFramePreset.g10102, oelPreset = Dynawo.Examples.RVS.Components.GeneratorWithControl.Util.ParametersOEL.oelFramePreset.all, pss2bPreset = Dynawo.Examples.RVS.Components.GeneratorWithControl.Util.ParametersPSS2B.genFramePreset.g10102) annotation(
    Placement(visible = true, transformation(origin = {-44, -306}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Dynawo.Examples.RVS.Components.GeneratorWithControl.SteamEXACFrame gen_30102_ADAMS_G3(P0Pu = P0Pu_gen_30102_ADAMS_G3, Q0Pu = Q0Pu_gen_30102_ADAMS_G3, U0Pu = U0Pu_gen_30102_ADAMS_G3, UPhase0 = UPhase0_gen_30102_ADAMS_G3, exac1Preset = Dynawo.Examples.RVS.Components.GeneratorWithControl.Util.ParametersEXAC.genFramePreset.g30102, gen = Dynawo.Examples.RVS.Components.GeneratorWithControl.Util.ParametersGenerators.genFramePreset.g30102, ieeeg1Preset = Dynawo.Examples.RVS.Components.GeneratorWithControl.Util.ParametersIEEEG1.genFramePreset.g30102, oelPreset = Dynawo.Examples.RVS.Components.GeneratorWithControl.Util.ParametersOEL.oelFramePreset.all, pss2bPreset = Dynawo.Examples.RVS.Components.GeneratorWithControl.Util.ParametersPSS2B.genFramePreset.g30102) annotation(
    Placement(visible = true, transformation(origin = {36, -306}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Components.GeneratorWithControl.SteamSCRXFrame gen_30107_ALDER_G3(P0Pu = P0Pu_gen_30107_ALDER_G3, Q0Pu = Q0Pu_gen_30107_ALDER_G3, U0Pu = U0Pu_gen_30107_ALDER_G3, UPhase0 = UPhase0_gen_30107_ALDER_G3, gen = Dynawo.Examples.RVS.Components.GeneratorWithControl.Util.ParametersGenerators.genFramePreset.g30107, ieeeg1Preset = Dynawo.Examples.RVS.Components.GeneratorWithControl.Util.ParametersIEEEG1.genFramePreset.g30107, oelPreset = Dynawo.Examples.RVS.Components.GeneratorWithControl.Util.ParametersOEL.oelFramePreset.all, pss2bPreset = Dynawo.Examples.RVS.Components.GeneratorWithControl.Util.ParametersPSS2B.genFramePreset.g30107, scrxPreset = Dynawo.Examples.RVS.Components.GeneratorWithControl.Util.ParametersSCRX.genFramePreset.g30107) annotation(
    Placement(visible = true, transformation(origin = {296, -214}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
  Components.GeneratorWithControl.SteamSCRXFrame gen_10107_ALDER_G1(P0Pu = P0Pu_gen_10107_ALDER_G1, Q0Pu = Q0Pu_gen_10107_ALDER_G1, U0Pu = U0Pu_gen_10107_ALDER_G1, UPhase0 = UPhase0_gen_10107_ALDER_G1, gen = Dynawo.Examples.RVS.Components.GeneratorWithControl.Util.ParametersGenerators.genFramePreset.g10107, ieeeg1Preset = Dynawo.Examples.RVS.Components.GeneratorWithControl.Util.ParametersIEEEG1.genFramePreset.g10107, oelPreset = Dynawo.Examples.RVS.Components.GeneratorWithControl.Util.ParametersOEL.oelFramePreset.all, pss2bPreset = Dynawo.Examples.RVS.Components.GeneratorWithControl.Util.ParametersPSS2B.genFramePreset.g10107, scrxPreset = Dynawo.Examples.RVS.Components.GeneratorWithControl.Util.ParametersSCRX.genFramePreset.g10107) annotation(
    Placement(visible = true, transformation(origin = {296, -134}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
  Components.GeneratorWithControl.SteamSCRXFrame gen_20107_ALDER_G2(P0Pu = P0Pu_gen_20107_ALDER_G2, Q0Pu = Q0Pu_gen_20107_ALDER_G2, U0Pu = U0Pu_gen_20107_ALDER_G2, UPhase0 = UPhase0_gen_20107_ALDER_G2, gen = Dynawo.Examples.RVS.Components.GeneratorWithControl.Util.ParametersGenerators.genFramePreset.g20107, ieeeg1Preset = Dynawo.Examples.RVS.Components.GeneratorWithControl.Util.ParametersIEEEG1.genFramePreset.g20107, oelPreset = Dynawo.Examples.RVS.Components.GeneratorWithControl.Util.ParametersOEL.oelFramePreset.all, pss2bPreset = Dynawo.Examples.RVS.Components.GeneratorWithControl.Util.ParametersPSS2B.genFramePreset.g20107, scrxPreset = Dynawo.Examples.RVS.Components.GeneratorWithControl.Util.ParametersSCRX.genFramePreset.g20107) annotation(
    Placement(visible = true, transformation(origin = {296, -174}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
  Electrical.Buses.InfiniteBus infiniteBus(UPhase = from_deg(13.4), UPu = 1.04685) annotation(
    Placement(visible = true, transformation(origin = {-134, 322}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
  Components.GeneratorWithControl.SteamSCRXFrame gen_20113_ARNE_G2(P0Pu = P0Pu_gen_20113_ARNE_G2, Q0Pu = Q0Pu_gen_20113_ARNE_G2, U0Pu = U0Pu_gen_20113_ARNE_G2, UPhase0 = UPhase0_gen_20113_ARNE_G2, gen = Dynawo.Examples.RVS.Components.GeneratorWithControl.Util.ParametersGenerators.genFramePreset.g20113, ieeeg1Preset = Dynawo.Examples.RVS.Components.GeneratorWithControl.Util.ParametersIEEEG1.genFramePreset.g20113, oelPreset = Dynawo.Examples.RVS.Components.GeneratorWithControl.Util.ParametersOEL.oelFramePreset.all, pss2bPreset = Dynawo.Examples.RVS.Components.GeneratorWithControl.Util.ParametersPSS2B.genFramePreset.g20113, scrxPreset = Dynawo.Examples.RVS.Components.GeneratorWithControl.Util.ParametersSCRX.genFramePreset.g20113) annotation(
    Placement(visible = true, transformation(origin = {186, 6}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
  Dynawo.Examples.RVS.Components.GeneratorWithControl.SteamSCRXFrame gen_30113_ARNE_G3(P0Pu = P0Pu_gen_30113_ARNE_G3, Q0Pu = Q0Pu_gen_30113_ARNE_G3, U0Pu = U0Pu_gen_30113_ARNE_G3, UPhase0 = UPhase0_gen_30113_ARNE_G3, gen = Dynawo.Examples.RVS.Components.GeneratorWithControl.Util.ParametersGenerators.genFramePreset.g30113, ieeeg1Preset = Dynawo.Examples.RVS.Components.GeneratorWithControl.Util.ParametersIEEEG1.genFramePreset.g30113, oelPreset = Dynawo.Examples.RVS.Components.GeneratorWithControl.Util.ParametersOEL.oelFramePreset.all, pss2bPreset = Dynawo.Examples.RVS.Components.GeneratorWithControl.Util.ParametersPSS2B.genFramePreset.g30113, scrxPreset = Dynawo.Examples.RVS.Components.GeneratorWithControl.Util.ParametersSCRX.genFramePreset.g30113) annotation(
    Placement(visible = true, transformation(origin = {186, -34}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
  Components.GeneratorWithControl.SteamSCRXFrame gen_10113_ARNE_G1(P0Pu = P0Pu_gen_10113_ARNE_G1, Q0Pu = Q0Pu_gen_10113_ARNE_G1, U0Pu = U0Pu_gen_10113_ARNE_G1, UPhase0 = UPhase0_gen_10113_ARNE_G1, gen = Dynawo.Examples.RVS.Components.GeneratorWithControl.Util.ParametersGenerators.genFramePreset.g10113, ieeeg1Preset = Dynawo.Examples.RVS.Components.GeneratorWithControl.Util.ParametersIEEEG1.genFramePreset.g10113, oelPreset = Dynawo.Examples.RVS.Components.GeneratorWithControl.Util.ParametersOEL.oelFramePreset.all, pss2bPreset = Dynawo.Examples.RVS.Components.GeneratorWithControl.Util.ParametersPSS2B.genFramePreset.g10113, scrxPreset = Dynawo.Examples.RVS.Components.GeneratorWithControl.Util.ParametersSCRX.genFramePreset.g10113) annotation(
    Placement(visible = true, transformation(origin = {186, 46}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
  Components.GeneratorWithControl.SteamSCRXFrame gen_20123_AUSTEN_G2(P0Pu = P0Pu_gen_20123_AUSTEN_G2, Q0Pu = Q0Pu_gen_20123_AUSTEN_G2, U0Pu = U0Pu_gen_20123_AUSTEN_G2, UPhase0 = UPhase0_gen_20123_AUSTEN_G2, gen = Dynawo.Examples.RVS.Components.GeneratorWithControl.Util.ParametersGenerators.genFramePreset.g20123, ieeeg1Preset = Dynawo.Examples.RVS.Components.GeneratorWithControl.Util.ParametersIEEEG1.genFramePreset.g20123, oelPreset = Dynawo.Examples.RVS.Components.GeneratorWithControl.Util.ParametersOEL.oelFramePreset.all, pss2bPreset = Dynawo.Examples.RVS.Components.GeneratorWithControl.Util.ParametersPSS2B.genFramePreset.g20123, scrxPreset = Dynawo.Examples.RVS.Components.GeneratorWithControl.Util.ParametersSCRX.genFramePreset.g20123) annotation(
    Placement(visible = true, transformation(origin = {296, 106}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
  Components.GeneratorWithControl.SteamSCRXFrame gen_10123_AUSTEN_G1(P0Pu = P0Pu_gen_10123_AUSTEN_G1, Q0Pu = Q0Pu_gen_10123_AUSTEN_G1, U0Pu = U0Pu_gen_10123_AUSTEN_G1, UPhase0 = UPhase0_gen_10123_AUSTEN_G1, gen = Dynawo.Examples.RVS.Components.GeneratorWithControl.Util.ParametersGenerators.genFramePreset.g10123, ieeeg1Preset = Dynawo.Examples.RVS.Components.GeneratorWithControl.Util.ParametersIEEEG1.genFramePreset.g10123, oelPreset = Dynawo.Examples.RVS.Components.GeneratorWithControl.Util.ParametersOEL.oelFramePreset.all, pss2bPreset = Dynawo.Examples.RVS.Components.GeneratorWithControl.Util.ParametersPSS2B.genFramePreset.g10123, scrxPreset = Dynawo.Examples.RVS.Components.GeneratorWithControl.Util.ParametersSCRX.genFramePreset.g10123) annotation(
    Placement(visible = true, transformation(origin = {296, 146}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
  Components.GeneratorWithControl.SteamSCRXFrame gen_30123_AUSTEN_G3(P0Pu = P0Pu_gen_30123_AUSTEN_G3, Q0Pu = Q0Pu_gen_30123_AUSTEN_G3, U0Pu = U0Pu_gen_30123_AUSTEN_G3, UPhase0 = UPhase0_gen_30123_AUSTEN_G3, gen = Dynawo.Examples.RVS.Components.GeneratorWithControl.Util.ParametersGenerators.genFramePreset.g30123, ieeeg1Preset = Dynawo.Examples.RVS.Components.GeneratorWithControl.Util.ParametersIEEEG1.genFramePreset.g30123, oelPreset = Dynawo.Examples.RVS.Components.GeneratorWithControl.Util.ParametersOEL.oelFramePreset.all, pss2bPreset = Dynawo.Examples.RVS.Components.GeneratorWithControl.Util.ParametersPSS2B.genFramePreset.g30123, scrxPreset = Dynawo.Examples.RVS.Components.GeneratorWithControl.Util.ParametersSCRX.genFramePreset.g30123) annotation(
    Placement(visible = true, transformation(origin = {296, 66}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
  Components.GeneratorWithControl.SteamSCRXFrame gen_50115_ARTHUR_G5(P0Pu = P0Pu_gen_50115_ARTHUR_G5, Q0Pu = Q0Pu_gen_50115_ARTHUR_G5, U0Pu = U0Pu_gen_50115_ARTHUR_G5, UPhase0 = UPhase0_gen_50115_ARTHUR_G5, gen = Dynawo.Examples.RVS.Components.GeneratorWithControl.Util.ParametersGenerators.genFramePreset.g50115, ieeeg1Preset = Dynawo.Examples.RVS.Components.GeneratorWithControl.Util.ParametersIEEEG1.genFramePreset.g50115, oelPreset = Dynawo.Examples.RVS.Components.GeneratorWithControl.Util.ParametersOEL.oelFramePreset.all, pss2bPreset = Dynawo.Examples.RVS.Components.GeneratorWithControl.Util.ParametersPSS2B.genFramePreset.g50115, scrxPreset = Dynawo.Examples.RVS.Components.GeneratorWithControl.Util.ParametersSCRX.genFramePreset.g50115) annotation(
    Placement(visible = true, transformation(origin = {-264, 132}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
  Dynawo.Examples.RVS.Components.GeneratorWithControl.SteamSCRXFrame gen_60115_ARTHUR_G6(P0Pu = P0Pu_gen_60115_ARTHUR_G6, Q0Pu = Q0Pu_gen_60115_ARTHUR_G6, U0Pu = U0Pu_gen_60115_ARTHUR_G6, UPhase0 = UPhase0_gen_60115_ARTHUR_G6, gen = Dynawo.Examples.RVS.Components.GeneratorWithControl.Util.ParametersGenerators.genFramePreset.g60115, ieeeg1Preset = Dynawo.Examples.RVS.Components.GeneratorWithControl.Util.ParametersIEEEG1.genFramePreset.g60115, oelPreset = Dynawo.Examples.RVS.Components.GeneratorWithControl.Util.ParametersOEL.oelFramePreset.all, pss2bPreset = Dynawo.Examples.RVS.Components.GeneratorWithControl.Util.ParametersPSS2B.genFramePreset.g60115, scrxPreset = Dynawo.Examples.RVS.Components.GeneratorWithControl.Util.ParametersSCRX.genFramePreset.g60115) annotation(
    Placement(visible = true, transformation(origin = {-264, 92}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
  Components.GeneratorWithControl.SteamSCRXFrame gen_10115_ARTHUR_G1(P0Pu = P0Pu_gen_10115_ARTHUR_G1, Q0Pu = Q0Pu_gen_10115_ARTHUR_G1, U0Pu = U0Pu_gen_10115_ARTHUR_G1, UPhase0 = UPhase0_gen_10115_ARTHUR_G1, gen = Dynawo.Examples.RVS.Components.GeneratorWithControl.Util.ParametersGenerators.genFramePreset.g10115, ieeeg1Preset = Dynawo.Examples.RVS.Components.GeneratorWithControl.Util.ParametersIEEEG1.genFramePreset.g10115, oelPreset = Dynawo.Examples.RVS.Components.GeneratorWithControl.Util.ParametersOEL.oelFramePreset.all, pss2bPreset = Dynawo.Examples.RVS.Components.GeneratorWithControl.Util.ParametersPSS2B.genFramePreset.g10115, scrxPreset = Dynawo.Examples.RVS.Components.GeneratorWithControl.Util.ParametersSCRX.genFramePreset.g10115) annotation(
    Placement(visible = true, transformation(origin = {-264, 292}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
  Components.GeneratorWithControl.SteamSCRXFrame gen_40115_ARTHUR_G4(P0Pu = P0Pu_gen_40115_ARTHUR_G4, Q0Pu = Q0Pu_gen_40115_ARTHUR_G4, U0Pu = U0Pu_gen_40115_ARTHUR_G4, UPhase0 = UPhase0_gen_40115_ARTHUR_G4, gen = Dynawo.Examples.RVS.Components.GeneratorWithControl.Util.ParametersGenerators.genFramePreset.g40115, ieeeg1Preset = Dynawo.Examples.RVS.Components.GeneratorWithControl.Util.ParametersIEEEG1.genFramePreset.g40115, oelPreset = Dynawo.Examples.RVS.Components.GeneratorWithControl.Util.ParametersOEL.oelFramePreset.all, pss2bPreset = Dynawo.Examples.RVS.Components.GeneratorWithControl.Util.ParametersPSS2B.genFramePreset.g40115, scrxPreset = Dynawo.Examples.RVS.Components.GeneratorWithControl.Util.ParametersSCRX.genFramePreset.g40115) annotation(
    Placement(visible = true, transformation(origin = {-264, 172}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
  Dynawo.Examples.RVS.Components.GeneratorWithControl.SteamSCRXFrame gen_30115_ARTHUR_G3(P0Pu = P0Pu_gen_30115_ARTHUR_G3, Q0Pu = Q0Pu_gen_30115_ARTHUR_G3, U0Pu = U0Pu_gen_30115_ARTHUR_G3, UPhase0 = UPhase0_gen_30115_ARTHUR_G3, gen = Dynawo.Examples.RVS.Components.GeneratorWithControl.Util.ParametersGenerators.genFramePreset.g30115, ieeeg1Preset = Dynawo.Examples.RVS.Components.GeneratorWithControl.Util.ParametersIEEEG1.genFramePreset.g30115, oelPreset = Dynawo.Examples.RVS.Components.GeneratorWithControl.Util.ParametersOEL.oelFramePreset.all, pss2bPreset = Dynawo.Examples.RVS.Components.GeneratorWithControl.Util.ParametersPSS2B.genFramePreset.g30115, scrxPreset = Dynawo.Examples.RVS.Components.GeneratorWithControl.Util.ParametersSCRX.genFramePreset.g30115) annotation(
    Placement(visible = true, transformation(origin = {-264, 212}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
  Dynawo.Examples.RVS.Components.GeneratorWithControl.SteamSCRXFrame gen_20115_ARTHUR_G2(P0Pu = P0Pu_gen_20115_ARTHUR_G2, Q0Pu = Q0Pu_gen_20115_ARTHUR_G2, U0Pu = U0Pu_gen_20115_ARTHUR_G2, UPhase0 = UPhase0_gen_20115_ARTHUR_G2, gen = Dynawo.Examples.RVS.Components.GeneratorWithControl.Util.ParametersGenerators.genFramePreset.g20115, ieeeg1Preset = Dynawo.Examples.RVS.Components.GeneratorWithControl.Util.ParametersIEEEG1.genFramePreset.g20115, oelPreset = Dynawo.Examples.RVS.Components.GeneratorWithControl.Util.ParametersOEL.oelFramePreset.all, pss2bPreset = Dynawo.Examples.RVS.Components.GeneratorWithControl.Util.ParametersPSS2B.genFramePreset.g20115, scrxPreset = Dynawo.Examples.RVS.Components.GeneratorWithControl.Util.ParametersSCRX.genFramePreset.g20115) annotation(
    Placement(visible = true, transformation(origin = {-264, 252}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
  Components.GeneratorWithControl.SteamSCRXFrame gen_10116_ASSER_G1(P0Pu = P0Pu_gen_10116_ASSER_G1, Q0Pu = Q0Pu_gen_10116_ASSER_G1, U0Pu = U0Pu_gen_10116_ASSER_G1, UPhase0 = UPhase0_gen_10116_ASSER_G1, gen = Dynawo.Examples.RVS.Components.GeneratorWithControl.Util.ParametersGenerators.genFramePreset.g10116, ieeeg1Preset = Dynawo.Examples.RVS.Components.GeneratorWithControl.Util.ParametersIEEEG1.genFramePreset.g10116, oelPreset = Dynawo.Examples.RVS.Components.GeneratorWithControl.Util.ParametersOEL.oelFramePreset.all, pss2bPreset = Dynawo.Examples.RVS.Components.GeneratorWithControl.Util.ParametersPSS2B.genFramePreset.g10116, scrxPreset = Dynawo.Examples.RVS.Components.GeneratorWithControl.Util.ParametersSCRX.genFramePreset.g10116) annotation(
    Placement(visible = true, transformation(origin = {-146, 144}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
  Components.GeneratorWithControl.SteamIEEET1Frame gen_10118_ASTOR_G1(P0Pu = P0Pu_gen_10118_ASTOR_G1, Q0Pu = Q0Pu_gen_10118_ASTOR_G1, U0Pu = U0Pu_gen_10118_ASTOR_G1, UPhase0 = UPhase0_gen_10118_ASTOR_G1, gen = Dynawo.Examples.RVS.Components.GeneratorWithControl.Util.ParametersGenerators.genFramePreset.g10118, ieeeg1Preset = Dynawo.Examples.RVS.Components.GeneratorWithControl.Util.ParametersIEEEG1.genFramePreset.g10118, ieeet1Preset = Dynawo.Examples.RVS.Components.GeneratorWithControl.Util.ParametersIEEET1.genFramePreset.g10118, oelPreset = Dynawo.Examples.RVS.Components.GeneratorWithControl.Util.ParametersOEL.oelFramePreset.all, pss2bPreset = Dynawo.Examples.RVS.Components.GeneratorWithControl.Util.ParametersPSS2B.genFramePreset.g10118) annotation(
    Placement(visible = true, transformation(origin = {-54, 200}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
  Components.GeneratorWithControl.HydroFrame gen_30122_AUBREY_G3(GovInService = true, P0Pu = P0Pu_gen_30122_AUBREY_G3, Q0Pu = Q0Pu_gen_30122_AUBREY_G3, U0Pu = U0Pu_gen_30122_AUBREY_G3, UPhase0 = UPhase0_gen_30122_AUBREY_G3, gen = Dynawo.Examples.RVS.Components.GeneratorWithControl.Util.ParametersGenerators.genFramePreset.g30122, hygovPreset = Dynawo.Examples.RVS.Components.GeneratorWithControl.Util.ParametersHYGOV.genFramePreset.g30122, oelPreset = Dynawo.Examples.RVS.Components.GeneratorWithControl.Util.ParametersOEL.oelFramePreset.all, pss2bPreset = Dynawo.Examples.RVS.Components.GeneratorWithControl.Util.ParametersPSS2B.genFramePreset.g30122, scrxPreset = Dynawo.Examples.RVS.Components.GeneratorWithControl.Util.ParametersSCRX.genFramePreset.g30122) annotation(
    Placement(visible = true, transformation(origin = {294, 308}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
  Components.GeneratorWithControl.HydroFrame gen_10122_AUBREY_G1(GovInService = true, P0Pu = P0Pu_gen_10122_AUBREY_G1, Q0Pu = Q0Pu_gen_10122_AUBREY_G1, U0Pu = U0Pu_gen_10122_AUBREY_G1, UPhase0 = UPhase0_gen_10122_AUBREY_G1, gen = Dynawo.Examples.RVS.Components.GeneratorWithControl.Util.ParametersGenerators.genFramePreset.g10122, hygovPreset = Dynawo.Examples.RVS.Components.GeneratorWithControl.Util.ParametersHYGOV.genFramePreset.g10122, oelPreset = Dynawo.Examples.RVS.Components.GeneratorWithControl.Util.ParametersOEL.oelFramePreset.all, pss2bPreset = Dynawo.Examples.RVS.Components.GeneratorWithControl.Util.ParametersPSS2B.genFramePreset.g10122, scrxPreset = Dynawo.Examples.RVS.Components.GeneratorWithControl.Util.ParametersSCRX.genFramePreset.g10122) annotation(
    Placement(visible = true, transformation(origin = {162, 278}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
  Components.GeneratorWithControl.HydroFrame gen_50122_AUBREY_G5(GovInService = true, P0Pu = P0Pu_gen_50122_AUBREY_G5, Q0Pu = Q0Pu_gen_50122_AUBREY_G5, U0Pu = U0Pu_gen_50122_AUBREY_G5, UPhase0 = UPhase0_gen_50122_AUBREY_G5, gen = Dynawo.Examples.RVS.Components.GeneratorWithControl.Util.ParametersGenerators.genFramePreset.g50122, hygovPreset = Dynawo.Examples.RVS.Components.GeneratorWithControl.Util.ParametersHYGOV.genFramePreset.g50122, oelPreset = Dynawo.Examples.RVS.Components.GeneratorWithControl.Util.ParametersOEL.oelFramePreset.all, pss2bPreset = Dynawo.Examples.RVS.Components.GeneratorWithControl.Util.ParametersPSS2B.genFramePreset.g50122, scrxPreset = Dynawo.Examples.RVS.Components.GeneratorWithControl.Util.ParametersSCRX.genFramePreset.g50122) annotation(
    Placement(visible = true, transformation(origin = {294, 228}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
  Components.GeneratorWithControl.HydroFrame gen_60122_AUBREY_G6(GovInService = true, P0Pu = P0Pu_gen_60122_AUBREY_G6, Q0Pu = Q0Pu_gen_60122_AUBREY_G6, U0Pu = U0Pu_gen_60122_AUBREY_G6, UPhase0 = UPhase0_gen_60122_AUBREY_G6, gen = Dynawo.Examples.RVS.Components.GeneratorWithControl.Util.ParametersGenerators.genFramePreset.g60122, hygovPreset = Dynawo.Examples.RVS.Components.GeneratorWithControl.Util.ParametersHYGOV.genFramePreset.g60122, oelPreset = Dynawo.Examples.RVS.Components.GeneratorWithControl.Util.ParametersOEL.oelFramePreset.all, pss2bPreset = Dynawo.Examples.RVS.Components.GeneratorWithControl.Util.ParametersPSS2B.genFramePreset.g60122, scrxPreset = Dynawo.Examples.RVS.Components.GeneratorWithControl.Util.ParametersSCRX.genFramePreset.g60122) annotation(
    Placement(visible = true, transformation(origin = {294, 188}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
  Components.GeneratorWithControl.HydroFrame gen_20122_AUBREY_G2(GovInService = true, P0Pu = P0Pu_gen_20122_AUBREY_G2, Q0Pu = Q0Pu_gen_20122_AUBREY_G2, U0Pu = U0Pu_gen_20122_AUBREY_G2, UPhase0 = UPhase0_gen_20122_AUBREY_G2, gen = Dynawo.Examples.RVS.Components.GeneratorWithControl.Util.ParametersGenerators.genFramePreset.g20122, hygovPreset = Dynawo.Examples.RVS.Components.GeneratorWithControl.Util.ParametersHYGOV.genFramePreset.g20122, oelPreset = Dynawo.Examples.RVS.Components.GeneratorWithControl.Util.ParametersOEL.oelFramePreset.all, pss2bPreset = Dynawo.Examples.RVS.Components.GeneratorWithControl.Util.ParametersPSS2B.genFramePreset.g20122, scrxPreset = Dynawo.Examples.RVS.Components.GeneratorWithControl.Util.ParametersSCRX.genFramePreset.g20122) annotation(
    Placement(visible = true, transformation(origin = {162, 238}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
  Components.GeneratorWithControl.HydroFrame gen_40122_AUBREY_G4(GovInService = true, P0Pu = P0Pu_gen_40122_AUBREY_G4, Q0Pu = Q0Pu_gen_40122_AUBREY_G4, U0Pu = U0Pu_gen_40122_AUBREY_G4, UPhase0 = UPhase0_gen_40122_AUBREY_G4, gen = Dynawo.Examples.RVS.Components.GeneratorWithControl.Util.ParametersGenerators.genFramePreset.g40122, hygovPreset = Dynawo.Examples.RVS.Components.GeneratorWithControl.Util.ParametersHYGOV.genFramePreset.g40122, oelPreset = Dynawo.Examples.RVS.Components.GeneratorWithControl.Util.ParametersOEL.oelFramePreset.all, pss2bPreset = Dynawo.Examples.RVS.Components.GeneratorWithControl.Util.ParametersPSS2B.genFramePreset.g40122, scrxPreset = Dynawo.Examples.RVS.Components.GeneratorWithControl.Util.ParametersSCRX.genFramePreset.g40122) annotation(
    Placement(visible = true, transformation(origin = {294, 268}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
  Dynawo.Examples.RVS.Components.StaticVarCompensators.SVarCRVS sVarC_10106_ALBER_SVC(K = 150 * 150, Q0Pu = Q0Pu_sVarC_10106_ALBER_SVC, SBase = 100, U0Pu = U0Pu_sVarC_10106_ALBER_SVC, UPhase0 = UPhase0_sVarC_10106_ALBER_SVC, URef0Pu = U0Pu_sVarC_10106_ALBER_SVC, svcPreset = Dynawo.Examples.RVS.Components.StaticVarCompensators.Util.ParametersSVC.svcFramePreset.sVarC_10106) annotation(
    Placement(visible = true, transformation(origin = {176, -226}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Dynawo.Examples.RVS.Components.StaticVarCompensators.SVarCRVS sVarC_10114_ARNOLD_SVC(K = 150 * 250, Q0Pu = Q0Pu_sVarC_10114_ARNOLD_SVC, SBase = 100, U0Pu = U0Pu_sVarC_10114_ARNOLD_SVC, UPhase0 = UPhase0_sVarC_10114_ARNOLD_SVC, URef0Pu = U0Pu_sVarC_10114_ARNOLD_SVC, svcPreset = Dynawo.Examples.RVS.Components.StaticVarCompensators.Util.ParametersSVC.svcFramePreset.sVarC_10114) annotation(
    Placement(visible = true, transformation(origin = {16, 124}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));

protected
  final parameter Types.ComplexApparentPowerPu s0Pu_sVarC_10106_ALBER_SVC = Complex(P0Pu_sVarC_10106_ALBER_SVC, Q0Pu_sVarC_10106_ALBER_SVC);
  final parameter Types.ComplexVoltagePu u0Pu_sVarC_10106_ALBER_SVC = ComplexMath.fromPolar(U0Pu_sVarC_10106_ALBER_SVC, UPhase0_sVarC_10106_ALBER_SVC);
  final parameter Types.ComplexCurrentPu i0Pu_sVarC_10106_ALBER_SVC = ComplexMath.conj(s0Pu_sVarC_10106_ALBER_SVC / u0Pu_sVarC_10106_ALBER_SVC);
  final parameter Types.PerUnit B0Pu_sVarC_10106_ALBER_SVC = Q0Pu_sVarC_10106_ALBER_SVC / U0Pu_sVarC_10106_ALBER_SVC ^ 2;
  final parameter Types.ComplexApparentPowerPu s0Pu_sVarC_10114_ARNOLD_SVC = Complex(P0Pu_sVarC_10114_ARNOLD_SVC, Q0Pu_sVarC_10114_ARNOLD_SVC);
  final parameter Types.ComplexVoltagePu u0Pu_sVarC_10114_ARNOLD_SVC = ComplexMath.fromPolar(U0Pu_sVarC_10114_ARNOLD_SVC, UPhase0_sVarC_10114_ARNOLD_SVC);
  final parameter Types.ComplexCurrentPu i0Pu_sVarC_10114_ARNOLD_SVC = ComplexMath.conj(s0Pu_sVarC_10114_ARNOLD_SVC / u0Pu_sVarC_10114_ARNOLD_SVC);
  final parameter Types.PerUnit B0Pu_sVarC_10114_ARNOLD_SVC = Q0Pu_sVarC_10114_ARNOLD_SVC / U0Pu_sVarC_10114_ARNOLD_SVC ^ 2;

equation
  gen_10101_ABEL_G1.generatorSynchronous.switchOffSignal1.value = false;
  gen_10101_ABEL_G1.generatorSynchronous.switchOffSignal2.value = false;
  gen_10101_ABEL_G1.generatorSynchronous.switchOffSignal3.value = false;
  gen_10101_ABEL_G1.generatorSynchronous.omegaRefPu.value = 1;
  gen_20101_ABEL_G2.generatorSynchronous.switchOffSignal1.value = false;
  gen_20101_ABEL_G2.generatorSynchronous.switchOffSignal2.value = false;
  gen_20101_ABEL_G2.generatorSynchronous.switchOffSignal3.value = false;
  gen_20101_ABEL_G2.generatorSynchronous.omegaRefPu.value = 1;
  gen_30101_ABEL_G3.generatorSynchronous.switchOffSignal1.value = false;
  gen_30101_ABEL_G3.generatorSynchronous.switchOffSignal2.value = false;
  gen_30101_ABEL_G3.generatorSynchronous.switchOffSignal3.value = false;
  gen_30101_ABEL_G3.generatorSynchronous.omegaRefPu.value = 1;
  gen_40101_ABEL_G4.generatorSynchronous.switchOffSignal1.value = false;
  gen_40101_ABEL_G4.generatorSynchronous.switchOffSignal2.value = false;
  gen_40101_ABEL_G4.generatorSynchronous.switchOffSignal3.value = false;
  gen_40101_ABEL_G4.generatorSynchronous.omegaRefPu.value = 1;
  gen_10102_ADAMS_G1.generatorSynchronous.switchOffSignal1.value = false;
  gen_10102_ADAMS_G1.generatorSynchronous.switchOffSignal2.value = false;
  gen_10102_ADAMS_G1.generatorSynchronous.switchOffSignal3.value = false;
  gen_10102_ADAMS_G1.generatorSynchronous.omegaRefPu.value = 1;
  gen_20102_ADAMS_G2.generatorSynchronous.switchOffSignal1.value = false;
  gen_20102_ADAMS_G2.generatorSynchronous.switchOffSignal2.value = false;
  gen_20102_ADAMS_G2.generatorSynchronous.switchOffSignal3.value = false;
  gen_20102_ADAMS_G2.generatorSynchronous.omegaRefPu.value = 1;
  gen_30102_ADAMS_G3.generatorSynchronous.switchOffSignal1.value = false;
  gen_30102_ADAMS_G3.generatorSynchronous.switchOffSignal2.value = false;
  gen_30102_ADAMS_G3.generatorSynchronous.switchOffSignal3.value = false;
  gen_30102_ADAMS_G3.generatorSynchronous.omegaRefPu.value = 1;
  gen_40102_ADAMS_G4.generatorSynchronous.switchOffSignal1.value = false;
  gen_40102_ADAMS_G4.generatorSynchronous.switchOffSignal2.value = false;
  gen_40102_ADAMS_G4.generatorSynchronous.switchOffSignal3.value = false;
  gen_40102_ADAMS_G4.generatorSynchronous.omegaRefPu.value = 1;
  gen_10107_ALDER_G1.generatorSynchronous.switchOffSignal1.value = false;
  gen_10107_ALDER_G1.generatorSynchronous.switchOffSignal2.value = false;
  gen_10107_ALDER_G1.generatorSynchronous.switchOffSignal3.value = false;
  gen_10107_ALDER_G1.generatorSynchronous.omegaRefPu.value = 1;
  gen_20107_ALDER_G2.generatorSynchronous.switchOffSignal1.value = false;
  gen_20107_ALDER_G2.generatorSynchronous.switchOffSignal2.value = false;
  gen_20107_ALDER_G2.generatorSynchronous.switchOffSignal3.value = false;
  gen_20107_ALDER_G2.generatorSynchronous.omegaRefPu.value = 1;
  gen_30107_ALDER_G3.generatorSynchronous.switchOffSignal1.value = false;
  gen_30107_ALDER_G3.generatorSynchronous.switchOffSignal2.value = false;
  gen_30107_ALDER_G3.generatorSynchronous.switchOffSignal3.value = false;
  gen_30107_ALDER_G3.generatorSynchronous.omegaRefPu.value = 1;
  gen_10113_ARNE_G1.generatorSynchronous.switchOffSignal1.value = false;
  gen_10113_ARNE_G1.generatorSynchronous.switchOffSignal2.value = false;
  gen_10113_ARNE_G1.generatorSynchronous.switchOffSignal3.value = false;
  gen_10113_ARNE_G1.generatorSynchronous.omegaRefPu.value = 1;
  gen_20113_ARNE_G2.generatorSynchronous.switchOffSignal1.value = false;
  gen_20113_ARNE_G2.generatorSynchronous.switchOffSignal2.value = false;
  gen_20113_ARNE_G2.generatorSynchronous.switchOffSignal3.value = false;
  gen_20113_ARNE_G2.generatorSynchronous.omegaRefPu.value = 1;
  gen_30113_ARNE_G3.generatorSynchronous.switchOffSignal1.value = false;
  gen_30113_ARNE_G3.generatorSynchronous.switchOffSignal2.value = false;
  gen_30113_ARNE_G3.generatorSynchronous.switchOffSignal3.value = false;
  gen_30113_ARNE_G3.generatorSynchronous.omegaRefPu.value = 1;
  gen_10118_ASTOR_G1.generatorSynchronous.switchOffSignal1.value = false;
  gen_10118_ASTOR_G1.generatorSynchronous.switchOffSignal2.value = false;
  gen_10118_ASTOR_G1.generatorSynchronous.switchOffSignal3.value = false;
  gen_10118_ASTOR_G1.generatorSynchronous.omegaRefPu.value = 1;
  gen_10115_ARTHUR_G1.generatorSynchronous.switchOffSignal1.value = false;
  gen_10115_ARTHUR_G1.generatorSynchronous.switchOffSignal2.value = false;
  gen_10115_ARTHUR_G1.generatorSynchronous.switchOffSignal3.value = false;
  gen_10115_ARTHUR_G1.generatorSynchronous.omegaRefPu.value = 1;
  gen_20115_ARTHUR_G2.generatorSynchronous.switchOffSignal1.value = false;
  gen_20115_ARTHUR_G2.generatorSynchronous.switchOffSignal2.value = false;
  gen_20115_ARTHUR_G2.generatorSynchronous.switchOffSignal3.value = false;
  gen_20115_ARTHUR_G2.generatorSynchronous.omegaRefPu.value = 1;
  gen_30115_ARTHUR_G3.generatorSynchronous.switchOffSignal1.value = false;
  gen_30115_ARTHUR_G3.generatorSynchronous.switchOffSignal2.value = false;
  gen_30115_ARTHUR_G3.generatorSynchronous.switchOffSignal3.value = false;
  gen_30115_ARTHUR_G3.generatorSynchronous.omegaRefPu.value = 1;
  gen_40115_ARTHUR_G4.generatorSynchronous.switchOffSignal1.value = false;
  gen_40115_ARTHUR_G4.generatorSynchronous.switchOffSignal2.value = false;
  gen_40115_ARTHUR_G4.generatorSynchronous.switchOffSignal3.value = false;
  gen_40115_ARTHUR_G4.generatorSynchronous.omegaRefPu.value = 1;
  gen_50115_ARTHUR_G5.generatorSynchronous.switchOffSignal1.value = false;
  gen_50115_ARTHUR_G5.generatorSynchronous.switchOffSignal2.value = false;
  gen_50115_ARTHUR_G5.generatorSynchronous.switchOffSignal3.value = false;
  gen_50115_ARTHUR_G5.generatorSynchronous.omegaRefPu.value = 1;
  gen_60115_ARTHUR_G6.generatorSynchronous.switchOffSignal1.value = false;
  gen_60115_ARTHUR_G6.generatorSynchronous.switchOffSignal2.value = false;
  gen_60115_ARTHUR_G6.generatorSynchronous.switchOffSignal3.value = false;
  gen_60115_ARTHUR_G6.generatorSynchronous.omegaRefPu.value = 1;
  gen_10122_AUBREY_G1.generatorSynchronous.switchOffSignal1.value = false;
  gen_10122_AUBREY_G1.generatorSynchronous.switchOffSignal2.value = false;
  gen_10122_AUBREY_G1.generatorSynchronous.switchOffSignal3.value = false;
  gen_10122_AUBREY_G1.generatorSynchronous.omegaRefPu.value = 1;
  gen_20122_AUBREY_G2.generatorSynchronous.switchOffSignal1.value = false;
  gen_20122_AUBREY_G2.generatorSynchronous.switchOffSignal2.value = false;
  gen_20122_AUBREY_G2.generatorSynchronous.switchOffSignal3.value = false;
  gen_20122_AUBREY_G2.generatorSynchronous.omegaRefPu.value = 1;
  gen_30122_AUBREY_G3.generatorSynchronous.switchOffSignal1.value = false;
  gen_30122_AUBREY_G3.generatorSynchronous.switchOffSignal2.value = false;
  gen_30122_AUBREY_G3.generatorSynchronous.switchOffSignal3.value = false;
  gen_30122_AUBREY_G3.generatorSynchronous.omegaRefPu.value = 1;
  gen_40122_AUBREY_G4.generatorSynchronous.switchOffSignal1.value = false;
  gen_40122_AUBREY_G4.generatorSynchronous.switchOffSignal2.value = false;
  gen_40122_AUBREY_G4.generatorSynchronous.switchOffSignal3.value = false;
  gen_40122_AUBREY_G4.generatorSynchronous.omegaRefPu.value = 1;
  gen_50122_AUBREY_G5.generatorSynchronous.switchOffSignal1.value = false;
  gen_50122_AUBREY_G5.generatorSynchronous.switchOffSignal2.value = false;
  gen_50122_AUBREY_G5.generatorSynchronous.switchOffSignal3.value = false;
  gen_50122_AUBREY_G5.generatorSynchronous.omegaRefPu.value = 1;
  gen_60122_AUBREY_G6.generatorSynchronous.switchOffSignal1.value = false;
  gen_60122_AUBREY_G6.generatorSynchronous.switchOffSignal2.value = false;
  gen_60122_AUBREY_G6.generatorSynchronous.switchOffSignal3.value = false;
  gen_60122_AUBREY_G6.generatorSynchronous.omegaRefPu.value = 1;
  gen_10116_ASSER_G1.generatorSynchronous.switchOffSignal1.value = false;
  gen_10116_ASSER_G1.generatorSynchronous.switchOffSignal2.value = false;
  gen_10116_ASSER_G1.generatorSynchronous.switchOffSignal3.value = false;
  gen_10116_ASSER_G1.generatorSynchronous.omegaRefPu.value = 1;
  gen_10123_AUSTEN_G1.generatorSynchronous.switchOffSignal1.value = false;
  gen_10123_AUSTEN_G1.generatorSynchronous.switchOffSignal2.value = false;
  gen_10123_AUSTEN_G1.generatorSynchronous.switchOffSignal3.value = false;
  gen_10123_AUSTEN_G1.generatorSynchronous.omegaRefPu.value = 1;
  gen_20123_AUSTEN_G2.generatorSynchronous.switchOffSignal1.value = false;
  gen_20123_AUSTEN_G2.generatorSynchronous.switchOffSignal2.value = false;
  gen_20123_AUSTEN_G2.generatorSynchronous.switchOffSignal3.value = false;
  gen_20123_AUSTEN_G2.generatorSynchronous.omegaRefPu.value = 1;
  gen_30123_AUSTEN_G3.generatorSynchronous.switchOffSignal1.value = false;
  gen_30123_AUSTEN_G3.generatorSynchronous.switchOffSignal2.value = false;
  gen_30123_AUSTEN_G3.generatorSynchronous.switchOffSignal3.value = false;
  gen_30123_AUSTEN_G3.generatorSynchronous.omegaRefPu.value = 1;
  sVarC_10114_ARNOLD_SVC.sVarCVPropInterface.switchOffSignal1.value = false;
  sVarC_10114_ARNOLD_SVC.sVarCVPropInterface.switchOffSignal2.value = false;
  sVarC_10114_ARNOLD_SVC.URefPu = U0Pu_sVarC_10114_ARNOLD_SVC;
  sVarC_10106_ALBER_SVC.sVarCVPropInterface.switchOffSignal1.value = false;
  sVarC_10106_ALBER_SVC.sVarCVPropInterface.switchOffSignal2.value = false;
  sVarC_10106_ALBER_SVC.URefPu = U0Pu_sVarC_10106_ALBER_SVC;
  connect(gen_10101_ABEL_G1.terminal, bus_10101_ABEL_G1.terminal) annotation(
    Line(points = {{-248, -266}, {-248, -246}}, color = {0, 0, 255}));
  connect(gen_20101_ABEL_G2.terminal, bus_20101_ABEL_G2.terminal) annotation(
    Line(points = {{-208, -266}, {-208, -246}}, color = {0, 0, 255}));
  connect(gen_30101_ABEL_G3.terminal, bus_30101_ABEL_G3.terminal) annotation(
    Line(points = {{-168, -266}, {-168, -246}}, color = {0, 0, 255}));
  connect(gen_40101_ABEL_G4.terminal, bus_40101_ABEL_G4.terminal) annotation(
    Line(points = {{-128, -266}, {-128, -246}}, color = {0, 0, 255}));
  connect(gen_10102_ADAMS_G1.terminal, bus_10102_ADAMS_G1.terminal) annotation(
    Line(points = {{-44, -306}, {-44, -286}}, color = {0, 0, 255}));
  connect(gen_20102_ADAMS_G2.terminal, bus_20102_ADAMS_G2.terminal) annotation(
    Line(points = {{-4, -306}, {-4, -286}}, color = {0, 0, 255}));
  connect(gen_30102_ADAMS_G3.terminal, bus_30102_ADAMS_G3.terminal) annotation(
    Line(points = {{36, -306}, {36, -286}}, color = {0, 0, 255}));
  connect(gen_40102_ADAMS_G4.terminal, bus_40102_ADAMS_G4.terminal) annotation(
    Line(points = {{76, -306}, {76, -286}}, color = {0, 0, 255}));
  connect(gen_10107_ALDER_G1.terminal, bus_10107_ALDER_G1.terminal) annotation(
    Line(points = {{296, -134}, {276, -134}}, color = {0, 0, 255}));
  connect(gen_20107_ALDER_G2.terminal, bus_20107_ALDER_G2.terminal) annotation(
    Line(points = {{296, -174}, {276, -174}}, color = {0, 0, 255}));
  connect(gen_30107_ALDER_G3.terminal, bus_30107_ALDER_G3.terminal) annotation(
    Line(points = {{296, -214}, {276, -214}}, color = {0, 0, 255}));
  connect(infiniteBus.terminal, bus_10121_ATTLEE_G1.terminal) annotation(
    Line(points = {{-134, 322}, {-134, 282}}, color = {0, 0, 255}));
  connect(gen_10113_ARNE_G1.terminal, bus_10113_ARNE_G1.terminal) annotation(
    Line(points = {{186, 46}, {166, 46}}, color = {0, 0, 255}));
  connect(gen_20113_ARNE_G2.terminal, bus_20113_ARNE_G2.terminal) annotation(
    Line(points = {{186, 6}, {166, 6}}, color = {0, 0, 255}));
  connect(gen_30113_ARNE_G3.terminal, bus_30113_ARNE_G3.terminal) annotation(
    Line(points = {{186, -34}, {166, -34}}, color = {0, 0, 255}));
  connect(gen_10123_AUSTEN_G1.terminal, bus_10123_AUSTEN_G1.terminal) annotation(
    Line(points = {{296, 146}, {266, 146}}, color = {0, 0, 255}));
  connect(gen_20123_AUSTEN_G2.terminal, bus_20123_AUSTEN_G2.terminal) annotation(
    Line(points = {{296, 106}, {266, 106}}, color = {0, 0, 255}));
  connect(gen_30123_AUSTEN_G3.terminal, bus_30123_AUSTEN_G3.terminal) annotation(
    Line(points = {{296, 66}, {266, 66}}, color = {0, 0, 255}));
  connect(gen_10115_ARTHUR_G1.terminal, bus_10115_ARTHUR_G1.terminal) annotation(
    Line(points = {{-264, 292}, {-244, 292}}, color = {0, 0, 255}));
  connect(gen_20115_ARTHUR_G2.terminal, bus_20115_ARTHUR_G2.terminal) annotation(
    Line(points = {{-264, 252}, {-244, 252}}, color = {0, 0, 255}));
  connect(gen_30115_ARTHUR_G3.terminal, bus_30115_ARTHUR_G3.terminal) annotation(
    Line(points = {{-264, 212}, {-244, 212}}, color = {0, 0, 255}));
  connect(gen_40115_ARTHUR_G4.terminal, bus_40115_ARTHUR_G4.terminal) annotation(
    Line(points = {{-264, 172}, {-244, 172}}, color = {0, 0, 255}));
  connect(gen_50115_ARTHUR_G5.terminal, bus_50115_ARTHUR_G5.terminal) annotation(
    Line(points = {{-264, 132}, {-244, 132}}, color = {0, 0, 255}));
  connect(gen_60115_ARTHUR_G6.terminal, bus_60115_ARTHUR_G6.terminal) annotation(
    Line(points = {{-264, 92}, {-244, 92}}, color = {0, 0, 255}));
  connect(gen_10116_ASSER_G1.terminal, bus_10116_ASSER_G1.terminal) annotation(
    Line(points = {{-146, 144}, {-124, 144}}, color = {0, 0, 255}));
  connect(gen_10118_ASTOR_G1.terminal, bus_10118_ASTOR_G1.terminal) annotation(
    Line(points = {{-54, 200}, {-34, 200}}, color = {0, 0, 255}));
  connect(gen_30122_AUBREY_G3.terminal, bus_30122_AUBREY_G3.terminal) annotation(
    Line(points = {{294, 308}, {266, 308}}, color = {0, 0, 255}));
  connect(gen_40122_AUBREY_G4.terminal, bus_40122_AUBREY_G4.terminal) annotation(
    Line(points = {{294, 268}, {266, 268}}, color = {0, 0, 255}));
  connect(gen_50122_AUBREY_G5.terminal, bus_50122_AUBREY_G5.terminal) annotation(
    Line(points = {{294, 228}, {266, 228}}, color = {0, 0, 255}));
  connect(gen_60122_AUBREY_G6.terminal, bus_60122_AUBREY_G6.terminal) annotation(
    Line(points = {{294, 188}, {266, 188}}, color = {0, 0, 255}));
  connect(gen_10122_AUBREY_G1.terminal, bus_10122_AUBREY_G1.terminal) annotation(
    Line(points = {{162, 278}, {186, 278}}, color = {0, 0, 255}));
  connect(gen_20122_AUBREY_G2.terminal, bus_20122_AUBREY_G2.terminal) annotation(
    Line(points = {{162, 238}, {186, 238}}, color = {0, 0, 255}));
  connect(sVarC_10106_ALBER_SVC.terminal, bus_10106_ALBER_SVC.terminal) annotation(
    Line(points = {{176, -226}, {152, -226}}, color = {0, 0, 255}));
  connect(sVarC_10114_ARNOLD_SVC.terminal, bus_10114_ARNOLD_SVC.terminal) annotation(
    Line(points = {{16, 124}, {-6, 124}}, color = {0, 0, 255}));

  annotation(
    experiment(StartTime = 0, StopTime = 10, Tolerance = 1e-03, Interval = 0.1),
    __OpenModelica_commandLineOptions = "--matchingAlgorithm=PFPlusExt --indexReductionMethod=dynamicStateSelection -d=initialization,NLSanalyticJacobian --daeMode",
    __OpenModelica_simulationFlags(lv = "LOG_STATS", s = "ida"),
    Diagram(coordinateSystem(extent = {{-200, -300}, {200, 300}})),
    Icon(coordinateSystem(extent = {{-200, -300}, {200, 300}})));
end FullDynamicInfiniteBus;

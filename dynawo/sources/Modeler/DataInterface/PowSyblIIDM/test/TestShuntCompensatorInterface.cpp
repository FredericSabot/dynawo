//
// Copyright (c) 2015-2020, RTE (http://www.rte-france.com)
// See AUTHORS.txt
// All rights reserved.
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, you can obtain one at http://mozilla.org/MPL/2.0/.
// SPDX-License-Identifier: MPL-2.0
//
// This file is part of Dynawo, an hybrid C++/Modelica open source time domain
// simulation tool for power systems.
//

#include "DYNShuntCompensatorInterfaceIIDM.h"

#include "DYNBusInterfaceIIDM.h"
#include "DYNInjectorInterfaceIIDM.h"
#include "DYNVoltageLevelInterfaceIIDM.h"

#include <powsybl/iidm/Bus.hpp>
#include <powsybl/iidm/Network.hpp>
#include <powsybl/iidm/ShuntCompensator.hpp>
#include <powsybl/iidm/ShuntCompensatorAdder.hpp>
#include <powsybl/iidm/Substation.hpp>

#include "gtest_dynawo.h"

namespace DYN {

using powsybl::iidm::Bus;
using powsybl::iidm::Network;
using powsybl::iidm::ShuntCompensator;
using powsybl::iidm::Substation;
using powsybl::iidm::TopologyKind;
using powsybl::iidm::VoltageLevel;

TEST(DataInterfaceTest, ShuntCompensator) {
  Network network("test", "test");

  Substation& substation = network.newSubstation()
                               .setId("S1")
                               .setName("S1_NAME")
                               .setCountry(powsybl::iidm::Country::FR)
                               .setTso("TSO")
                               .add();

  VoltageLevel& vl1 = substation.newVoltageLevel()
                          .setId("VL1")
                          .setName("VL1_NAME")
                          .setTopologyKind(TopologyKind::BUS_BREAKER)
                          .setNominalV(380.0)
                          .setLowVoltageLimit(340.0)
                          .setHighVoltageLimit(420.0)
                          .add();

  Bus& bus1 = vl1.getBusBreakerView().newBus().setId("VL1_BUS1").add();

  vl1.newShuntCompensator()
      .setId("SHUNT1")
      .setName("SHUNT1_NAME")
      .setBus(bus1.getId())
      .setConnectableBus(bus1.getId())
      .newLinearModel()
      .setBPerSection(12.0)
      .setMaximumSectionCount(3UL)
      .add()
      .setSectionCount(2UL)
      .add();

  ShuntCompensator& shuntCompensator = network.getShuntCompensator("SHUNT1");
  ShuntCompensatorInterfaceIIDM shuntCompensatorIfce(shuntCompensator);
  const boost::shared_ptr<VoltageLevelInterface> voltageLevelIfce(new VoltageLevelInterfaceIIDM(vl1));
  shuntCompensatorIfce.setVoltageLevelInterface(voltageLevelIfce);

  ASSERT_EQ(shuntCompensatorIfce.getComponentVarIndex(std::string("currentSection")), ShuntCompensatorInterfaceIIDM::VAR_CURRENTSECTION);
  ASSERT_EQ(shuntCompensatorIfce.getComponentVarIndex(std::string("wrongIndex")), -1);
  ASSERT_EQ(shuntCompensatorIfce.getComponentVarIndex(std::string("q")), ShuntCompensatorInterfaceIIDM::VAR_Q);
  ASSERT_EQ(shuntCompensatorIfce.getComponentVarIndex(std::string("state")), ShuntCompensatorInterfaceIIDM::VAR_STATE);

  ASSERT_EQ(shuntCompensatorIfce.getID(), shuntCompensator.getId());

  ASSERT_TRUE(shuntCompensatorIfce.getInitialConnected());
  ASSERT_TRUE(shuntCompensatorIfce.isConnected());
  ASSERT_TRUE(shuntCompensatorIfce.isPartiallyConnected());
  shuntCompensator.getTerminal().disconnect();
  ASSERT_FALSE(shuntCompensatorIfce.isPartiallyConnected());
  ASSERT_FALSE(shuntCompensatorIfce.isConnected());
  ASSERT_TRUE(shuntCompensatorIfce.getInitialConnected());

  ASSERT_EQ(shuntCompensatorIfce.getBusInterface().get(), nullptr);
  const boost::shared_ptr<BusInterface> busIfce(new BusInterfaceIIDM(bus1));
  shuntCompensatorIfce.setBusInterface(busIfce);
  ASSERT_EQ(shuntCompensatorIfce.getBusInterface().get()->getID(), "VL1_BUS1");

  ASSERT_DOUBLE_EQ(shuntCompensatorIfce.getCurrentSection(), 2UL);
  ASSERT_DOUBLE_EQ(shuntCompensatorIfce.getMaximumSection(), 3UL);
  shuntCompensator.getTerminal().setQ(4.0);
  ASSERT_DOUBLE_EQ(shuntCompensatorIfce.getQ(), 4.0);
  ASSERT_DOUBLE_EQ(shuntCompensatorIfce.getVNom(), 380);
  ASSERT_TRUE(shuntCompensatorIfce.isLinear());
  ASSERT_DOUBLE_EQ(shuntCompensatorIfce.getB(0), 0.);
  ASSERT_DOUBLE_EQ(shuntCompensatorIfce.getB(1), 12.);
  ASSERT_DOUBLE_EQ(shuntCompensatorIfce.getB(2), 24.);
  ASSERT_DOUBLE_EQ(shuntCompensatorIfce.getB(3), 36.);

  vl1.newShuntCompensator()
      .setId("SHUNT2")
      .setName("SHUNT2_NAME")
      .setBus(bus1.getId())
      .setConnectableBus(bus1.getId())
      .newNonLinearModel()
      .beginSection()
      .setB(11.)
      .endSection()
      .beginSection()
      .setB(24.0)
      .endSection()
      .add()
      .setSectionCount(2)
      .add();
  ShuntCompensator& shuntCompensator_2 = network.getShuntCompensator("SHUNT2");
  ShuntCompensatorInterfaceIIDM shuntCompensatorIfce_2(shuntCompensator_2);
  const boost::shared_ptr<VoltageLevelInterface> voltageLevelIfce_2(new VoltageLevelInterfaceIIDM(vl1));
  shuntCompensatorIfce_2.setVoltageLevelInterface(voltageLevelIfce_2);
  ASSERT_FALSE(shuntCompensatorIfce_2.isLinear());
  ASSERT_DOUBLE_EQ(shuntCompensatorIfce_2.getB(0), 0.);
  ASSERT_DOUBLE_EQ(shuntCompensatorIfce_2.getB(1), 11.);
  ASSERT_DOUBLE_EQ(shuntCompensatorIfce_2.getB(2), 24.);
}  // TEST(DataInterfaceTest, ShuntCompensator)

}  // namespace DYN

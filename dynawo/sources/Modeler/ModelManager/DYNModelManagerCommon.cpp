//
// Copyright (c) 2015-2019, RTE (http://www.rte-france.com)
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

/**
 * @file  DYNModelManagerCommon.cpp
 *
 * @brief implementation of useful/utilitaries methods
 *
 */
#include <sstream>
#include <fstream>
#include <iomanip>
#include <stdio.h>
#include <string>
#include <map>
#include <cstdlib>
#include <boost/algorithm/string.hpp>
#include <boost/lexical_cast.hpp>

#include "DYNMacrosMessage.h"
#include "DYNSubModel.h"
#include "DYNModelManager.h"
#include "DYNModelManagerCommon.h"
#include "DYNTerminate.h"
#include "DYNTrace.h"
#include "DYNExecUtils.h"
#include "DYNFileSystemUtils.h"

#ifdef HELICS_ENABLED
#include "helics/cpp98/helics.hpp"
#include "helics/cpp98/ValueFederate.hpp"
#include "helics/cpp98/Publication.hpp"
#include "helics/cpp98/Input.hpp"
#endif

namespace DYN {

void printLogToStdOut_(ModelManager *model, const std::string & message) {
  SubModel * sub = dynamic_cast<SubModel*> (model);
  if (sub == NULL)
    throw DYNError(Error::GENERAL, WrongDynamicCast);
  std::cout << sub->name() << ":" << message << std::endl;
  Trace::debug() << sub->name() << ":" << message << Trace::endline;
}

void printLogExecution_(ModelManager * model, const std::string & message) {
  SubModel * sub = dynamic_cast<SubModel*> (model);
  if (sub == NULL)
    throw DYNError(Error::GENERAL, WrongDynamicCast);
  sub->addMessage(message);
}

void addLogEvent_(ModelManager* model, const MessageTimeline& messageTimeline) {
  if (!model->hasTimeline()) {
    return;
  }

  model->addEvent(model->name(), messageTimeline);
}

void addLogEventRaw2_(ModelManager* model, const char* message1, const char* message2) {
  if (!model->hasTimeline()) {
    return;
  }

  std::stringstream ss("");
  ss << message1 << message2;
  const std::string fullMessage = ss.str();

  addLogEvent_(model, (MessageTimeline("", fullMessage)));
}

void
addLogEventRaw3_(ModelManager* model, const char* message1, const char* message2,
        const char* message3) {
  if (!model->hasTimeline()) {
    return;
  }

  std::stringstream ss("");
  ss << message1 << message2 << message3;
  const std::string fullMessage = ss.str();

  addLogEvent_(model, (MessageTimeline("", fullMessage)));
}

void addLogEventRaw4_(ModelManager* model, const char* message1, const char* message2,
        const char* message3, const char* message4) {
  if (!model->hasTimeline()) {
    return;
  }

  std::stringstream ss("");
  ss << message1 << message2 << message3 << message4;
  const std::string fullMessage = ss.str();

  addLogEvent_(model, (MessageTimeline("", fullMessage)));
}

void addLogEventRaw5_(ModelManager* model, const char* message1, const char* message2,
        const char* message3, const char* message4, const char* message5) {
  if (!model->hasTimeline()) {
    return;
  }

  std::stringstream ss("");
  ss << message1 << message2 << message3 << message4 << message5;
  const std::string fullMessage = ss.str();

  addLogEvent_(model, (MessageTimeline("", fullMessage)));
}

void addLogConstraintBegin_(ModelManager* model, const Message& message) {
  model->addConstraint(model->name(), true, message);
}

void addLogConstraintEnd_(ModelManager* model, const Message& message) {
  model->addConstraint(model->name(), false, message);
}

#ifdef __clang__
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wmissing-noreturn"
#endif  // __clang__

void assert_(ModelManager* model, const Message& message) {
  throw MessageError(model->name() + " : " + message.str());
}

void throw_(ModelManager* model, const Message& message) {
  throw MessageError(model->name() + " : " + message.str());
}

void terminate_(ModelManager* model, const MessageTimeline& messageTimeline) {
  model->addEvent(model->name(), messageTimeline);
  throw DYNTerminate(TerminateInModel, model->name(), messageTimeline.str());
}

#ifdef __clang__
#pragma clang diagnostic pop
#endif  // __clang__

const char* modelica_integer_to_modelica_string(modelica_integer i, modelica_integer /*minLen*/, modelica_boolean /*leftJustified*/) {
  // @todo warning: no thread safe
  std::stringstream ss("");
  ss << i;
  return memoryManagerChars::keep(ss.str());
}

const char* cat_modelica_string(modelica_string_const s1, modelica_string_const s2) {
  std::stringstream ss("");
  ss << s1 << s2;
  return memoryManagerChars::keep(ss.str());
}

const char* cat_modelica_string(std::string s1, modelica_string_const s2) {
  std::stringstream ss("");
  ss << s1 << s2;
  return memoryManagerChars::keep(ss.str());
}

const char* cat_modelica_string(modelica_string_const s1, std::string s2) {
  std::stringstream ss("");
  ss << s1 << s2;
  return memoryManagerChars::keep(ss.str());
}

const char* stringAppend(const modelica_string s1, const modelica_string s2) {
  std::stringstream ss("");
  ss << s1 << s2;
  return memoryManagerChars::keep(ss.str());
}

const char* stringAppend(const modelica_string s1, const std::string s2) {
  std::stringstream ss("");
  ss << s1 << s2;
  return memoryManagerChars::keep(ss.str());
}

const char* stringAppend(const std::string s1, const modelica_string s2) {
  std::stringstream ss("");
  ss << s1 << s2;
  return memoryManagerChars::keep(ss.str());
}

std::string mmc_strings_len1(unsigned int size) {
  const std::string tmp(size, '\0');
  return tmp;
}

const char* modelica_real_to_modelica_string_format(modelica_real r, std::string /*format*/) {
  std::stringstream ss("");
  ss << std::setprecision(3) << std::fixed << r;
  return memoryManagerChars::keep(ss.str());
}

const char* modelica_integer_to_modelica_string_format(modelica_integer i, std::string /*format*/) {
  std::stringstream ss("");
  ss << i;
  return memoryManagerChars::keep(ss.str());
}

const char * modelica_real_to_modelica_string(modelica_real r, modelica_integer /*minLen*/, modelica_boolean /*leftJustified*/,
                                              modelica_integer signDigits) {
  // @todo warning: no thread safe
  std::stringstream ss("");
  ss << std::setprecision(signDigits) << std::fixed << r;
  return memoryManagerChars::keep(ss.str());
}

const char * modelica_boolean_to_modelica_string(modelica_boolean b, modelica_integer /*minLen*/, modelica_boolean /*leftJustified*/) {
  if (b) {
    return "true";
  } else {
    return "false";
  }
}

bool compareString_(const std::string& x, const std::string& y) {
  return (x.compare(y) == 0);
}

const
modelica_integer* integerArrayElementAddress1_(const modelica_integer * source, int dim1) {
  return ( source + dim1 - 1);
}

modelica_string enumToModelicaString_(modelica_integer nr, const char *e[]) {
  return e[nr - 1];
}

modelica_integer sizeOffArray_(const modelica_integer array[], modelica_integer dim) {
  modelica_integer size;
  if (array == NULL) {
    size = 0;
  } else {
    size = array[dim];
  }

  return size;
}

void
callExternalAutomatonModel(const std::string& modelName, const char* command, const double time,
    const double* inputs, const char** inputsName, const int nbInputs, const int nbMaxInputs,
    double* outputs, const char** outputsName, const int nbOutputs, const int nbMaxOutputs,
    int* intOutputs, const char** intOutputsName, const int nbIntOutputs, const int nbMaxIntOutputs, const std::string& workingDirectory) {
  if (nbInputs >= nbMaxInputs)
    throw DYNError(Error::GENERAL, AutomatonMaximumInputSizeReached, modelName,
        boost::lexical_cast<std::string>(nbInputs), boost::lexical_cast<std::string>(nbMaxInputs));
  if (nbOutputs >= nbMaxOutputs || nbIntOutputs >= nbMaxIntOutputs)
    throw DYNError(Error::GENERAL, AutomatonMaximumOutputSizeReached, modelName,
        boost::lexical_cast<std::string>(nbOutputs), boost::lexical_cast<std::string>(nbMaxOutputs));
  static std::string separator = ";";
  std::string workingDir = workingDirectory + "/execution/" + modelName + "/";
  if (!exists(workingDir))
    create_directory(workingDir);

  std::string outputFile = workingDir + "file_in.csv";

  // write input file
  std::ofstream stream(outputFile.c_str(), std::ofstream::out);
  stream << "time" << separator << time << std::endl;
  for (int i=0; i< nbInputs; ++i)
    stream << inputsName[i] << separator << inputs[i] << std::endl;
  stream.close();

  // launch command
  std::stringstream ss;
  std::string commandStr(command);
  boost::replace_all(commandStr, "${AUTOMATON_WORKING_DIR}", workingDir);
  if (executeCommand(commandStr, ss) != 0)
    throw DYNError(Error::GENERAL, SystemCallFailed, commandStr, ss.str());

  Trace::debug() << ss.str() << Trace::endline;

  std::string inputFile = workingDir + "file_out.csv";
  // read output file
  std::ifstream ifstream(inputFile.c_str(), std::ifstream::in);
  std::string line;
  std::map<std::string, double> values;
  while (std::getline(ifstream, line)) {
    std::size_t found = line.find_last_of(separator);
    std::string name = line.substr(0, found);
    double value = atof(line.substr(found+1).c_str());
    values[name] = value;
    line.clear();
  }

  // assign outputs thanks to name
  for (int i=0; i< nbOutputs; ++i) {
    std::map<std::string, double>::const_iterator iter = values.find(outputsName[i]);
    if (iter == values.end())
      throw DYNError(Error::GENERAL, UnknownAutomatonOutput, modelName, outputsName[i]);
    outputs[i] = iter->second;
  }
  for (int i=0; i< nbIntOutputs; ++i) {
    std::map<std::string, double>::const_iterator iter = values.find(intOutputsName[i]);
    if (iter == values.end())
      throw DYNError(Error::GENERAL, UnknownAutomatonOutput, modelName, intOutputsName[i]);
    intOutputs[i] = static_cast<int>(iter->second);
  }
}

#ifdef HELICS_ENABLED
void initHelicsCosimulationInterface(ModelManager* manager, const std::string& workingDirectory) {
  std::shared_ptr<helicscpp::ValueFederate> fed = std::make_shared<helicscpp::ValueFederate>(workingDirectory + "/Dynawo.json");

  assert(fed != nullptr);

  if (!fed->getIntegerProperty(HELICS_FLAG_UNINTERRUPTIBLE)) {
    std::cout << "Dynawo should be flagged uninterruptible" << std::endl;
    throw;
  }
  manager->fed_ = fed;

  fed->enterExecutingMode();
}

std::string
getLocalHelicsPubSubName(std::string pubName) {
  std::string localName;
  auto npos = pubName.find("/");

  if (npos != std::string::npos)
    return pubName.substr(npos + 1);
  return pubName;
}

void callHelicsCosimulationInterfaceModel(ModelManager* manager, const std::string& modelName,
    const double time, const double* inputs, const char** inputs_name, const int nbInputs,
    const int nbMaxInputs, double* outputs, const char** outputs_name, const int nbOutputs,
    const int nbMaxOutputs, const std::string& workingDirectory) {
  if (nbInputs >= nbMaxInputs)
    throw DYNError(Error::GENERAL, AutomatonMaximumInputSizeReached, modelName,
        boost::lexical_cast<std::string>(nbInputs), boost::lexical_cast<std::string>(nbMaxInputs));
  if (nbOutputs >= nbMaxOutputs)
    throw DYNError(Error::GENERAL, AutomatonMaximumOutputSizeReached, modelName,
        boost::lexical_cast<std::string>(nbOutputs), boost::lexical_cast<std::string>(nbMaxOutputs));

  if (manager->helicsTime_ < 0) {
    manager->helicsTime_ = 0;
    initHelicsCosimulationInterface(manager, workingDirectory);
  }

  // Only communicate with helics the first time the automaton is activated for at a given time (not after solver reinits)
  if (time > manager->helicsTime_) {
    while (manager->helicsTime_ < time)
      manager->helicsTime_ = manager->fed_->requestTime(time);

    int pubCount = manager->fed_->getPublicationCount();
    int subCount = manager->fed_->getInputCount();

    for (int i = 0; i < pubCount; i++) {
      helicscpp::Publication pub = manager->fed_->getPublication(i);
      std::string pubName = getLocalHelicsPubSubName(pub.getName());

      // assign publication thanks to name
      int found = 1;
      while (found < nbInputs + 1) {
        if (inputs_name[found] == pubName)
          break;
        found++;
      }
      if (found == nbInputs + 1) {
        throw DYNError(Error::GENERAL, UnknownAutomatonInput, modelName, pubName);
      }
      pub.publish(inputs[found]);
    }

    for (int i = 0; i < subCount; i++) {
      helicscpp::Input sub = manager->fed_->getInput(i);
      std::string subName = getLocalHelicsPubSubName(sub.getName());

      // assign subscription thanks to name
      int found = 1;
      while (found < nbOutputs + 1) {
        if (outputs_name[found] == subName)
          break;
        found++;
      }

      if (found == nbOutputs + 1) {
        throw DYNError(Error::GENERAL, UnknownAutomatonOutput, modelName, subName);
      }

      outputs[found] = sub.getDouble();
    }
  }
}

#endif

modelica_real
computeDelay(ModelManager* manager, int exprNumber, double exprValue, double time, double delayTime, double delayMax) {
  return manager->computeDelay(exprNumber, exprValue, time, delayTime, delayMax);
}

void
addDelay(ModelManager* manager, int exprNumber, const double* time, const double* exprValue, double delayMax) {
  manager->addDelay(exprNumber, time, exprValue, delayMax);
}

}  // namespace DYN

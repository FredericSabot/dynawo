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
#include <time.h>
#include <iostream>
#include <iomanip>
#include <tins/tins.h>

#include "DYNMacrosMessage.h"
#include "DYNSubModel.h"
#include "DYNModelManager.h"
#include "DYNModelManagerCommon.h"
#include "DYNTerminate.h"
#include "DYNTrace.h"
#include "DYNExecUtils.h"
#include "DYNFileSystemUtils.h"

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
    double* outputs, const char** outputsName, const int nbOutputs, const int nbMaxOutputs, const std::string& workingDirectory) {
  if (nbInputs >= nbMaxInputs)
    throw DYNError(Error::GENERAL, AutomatonMaximumInputSizeReached, modelName,
        boost::lexical_cast<std::string>(nbInputs), boost::lexical_cast<std::string>(nbMaxInputs));
  if (nbOutputs >= nbMaxOutputs)
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
  executeCommand(commandStr, ss);

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
}

timespec timeDiff(timespec end, timespec start) {
  timespec temp;
  if ((end.tv_nsec-start.tv_nsec) < 0) {
    temp.tv_sec = end.tv_sec - start.tv_sec-1;
    temp.tv_nsec = 1000000000 + end.tv_nsec - start.tv_nsec;
  } else {
    temp.tv_sec = end.tv_sec - start.tv_sec;
    temp.tv_nsec = end.tv_nsec - start.tv_nsec;
  }
  return temp;
}

void
callGNS3AutomatonModel(const std::string& modelName, const double* initWallTime, const double time,
    const double* inputs, const char** inputsName, const int nbInputs, const int nbMaxInputs,
    double* outputs, const char** outputsName, const int nbOutputs, const int nbMaxOutputs, const std::string& workingDirectory) {
  if (nbInputs >= nbMaxInputs)
    throw DYNError(Error::GENERAL, AutomatonMaximumInputSizeReached, modelName,
        boost::lexical_cast<std::string>(nbInputs), boost::lexical_cast<std::string>(nbMaxInputs));
  if (nbOutputs >= nbMaxOutputs)
    throw DYNError(Error::GENERAL, AutomatonMaximumOutputSizeReached, modelName,
        boost::lexical_cast<std::string>(nbOutputs), boost::lexical_cast<std::string>(nbMaxOutputs));
  std::string workingDir = workingDirectory + "/execution/" + modelName + "/";

  std::string separator = ";";
  // write input file
  std::stringstream unused;
  unused << "time" << separator << time << std::endl;
  for (int i=0; i< nbInputs; ++i)
    unused << inputsName[i] << separator << inputs[i] << std::endl;

  // assign outputs thanks to name
  for (int i=0; i< nbOutputs; ++i) {
    outputs[i] = 0;
    outputsName[i] += ' ';  // Unused parameter for now
  }

  timespec wall_time;
  clock_gettime(CLOCK_MONOTONIC, &wall_time);

  timespec init_time;
  init_time.tv_sec = static_cast<int>(initWallTime[0]);
  init_time.tv_nsec = static_cast<int>(initWallTime[1]);

  timespec elapsed_time = timeDiff(wall_time, init_time);

  const double time_with_margin = time + 0.1;  // At first iteration, time = 0 which would lead to a negative wait_time. Thus add a small margin
  timespec dynawo_time;
  dynawo_time.tv_sec = static_cast<int>(time_with_margin);
  dynawo_time.tv_nsec = 1000000000 * (time_with_margin - static_cast<int>(time_with_margin));

  timespec wait_time = timeDiff(dynawo_time, elapsed_time);
  // std::cout << "elapsed " << elapsed_time.tv_sec << "s, " << std::setw(9) << std::setfill('0') << elapsed_time.tv_nsec << "ns" << std::endl;
  // std::cout << "dynawo  " << dynawo_time.tv_sec << "s, " << std::setw(9) << std::setfill('0') << dynawo_time.tv_nsec << "ns" << std::endl;
  if (wait_time.tv_sec < 0 && time > 0.1) {  // At t = 0, allow delay because of bug that make it occur twice
    timespec time_zero, time_delay;
    time_zero.tv_nsec = 0;
    time_zero.tv_sec = 0;
    time_delay = timeDiff(time_zero, wait_time);
    std::cout << "At time " << time << ", too late by " << time_delay.tv_sec
        << "s, " << std::setw(9) << std::setfill('0') << time_delay.tv_nsec << "ns" << std::endl;
    // throw DYNError(Error::SIMULATION, CriteriaNotChecked);  // TODO(fsabot): more appropriate error message (dynawo cannot catch up to real time)
  } else {
    int it = 0;
    while (wait_time.tv_sec >= 0) {  // Busy wait loop
      clock_gettime(CLOCK_MONOTONIC, &wall_time);
      elapsed_time = timeDiff(wall_time, init_time);
      wait_time = timeDiff(dynawo_time, elapsed_time);
      it++;
    }
    // std::cout << it << std::endl;
  }
  if (wait_time.tv_nsec < 999000000) {
    std::cout << "More than 1ms delay at time: " << time << std::endl;
    std::cout << wait_time.tv_sec << "s, " << std::setw(9) << std::setfill('0') << wait_time.tv_nsec << "ns" << std::endl;
  }

  Tins::PacketSender sender;
  Tins::IP pkt = Tins::IP("127.0.0.1") / Tins::UDP(22) / Tins::RawPDU("Hello World!");
  sender.send(pkt);

  Tins::SnifferConfiguration config;
  config.set_immediate_mode(true);
  config.set_timeout(1);
  // config.set_filter("ip src 192.168.0.100");
  /*Tins::Sniffer sniffer("lo", config);
  Tins::PDU *some_pdu = sniffer.next_packet();  // TODO(fsabot): this is blocking, so I will need a queue
  std::cout << some_pdu->rfind_pdu<Tins::IP>().RAW << std::endl;
  delete some_pdu;*/
}

void
callGetWallTime(const double test, double* wallTime) {
  timespec time;
  clock_gettime(CLOCK_MONOTONIC, &time);
  wallTime[0] = test;  // Avoid unused
  wallTime[0] = static_cast<int>(time.tv_sec);
  wallTime[1] = static_cast<int>(time.tv_nsec);
}

modelica_real
computeDelay(ModelManager* manager, DYNDATA* data, int exprNumber, double exprValue, double time, double delayTime, double delayMax) {
  return manager->computeDelay(data, exprNumber, exprValue, time, delayTime, delayMax);
}

void
addDelay(ModelManager* manager, int exprNumber, const double* time, const double* exprValue, double delayMax) {
  manager->addDelay(exprNumber, time, exprValue, delayMax);
}

}  // namespace DYN

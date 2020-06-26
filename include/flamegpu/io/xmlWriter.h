#ifndef INCLUDE_FLAMEGPU_IO_XMLWRITER_H_
#define INCLUDE_FLAMEGPU_IO_XMLWRITER_H_

/**
 * @file xmlwriter.h
 * @author
 * @date
 * @brief
 *
 * \todo longer description
 */

#include <memory>
#include <string>
#include <unordered_map>

#include "flamegpu/io/statewriter.h"
#include "flamegpu/model/ModelDescription.h"

// Derived classes
class xmlWriter : public StateWriter {
 public:
    xmlWriter(const std::string &model_name, const unsigned int &instance_id, const std::unordered_map<std::string, std::shared_ptr<AgentPopulation>> &model_state, const unsigned int &iterations, const std::string &output_file);
    int writeStates();
};

#endif  // INCLUDE_FLAMEGPU_IO_XMLWRITER_H_

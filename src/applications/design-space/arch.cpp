/* Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *  * Neither the name of NVIDIA CORPORATION nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS ``AS IS'' AND ANY
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY
 * OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <iostream>
#include <regex>
#include <set>

#include "applications/design-space/arch.hpp"

std::vector<std::string> split(const std::string &s, char delim)
{
  std::stringstream ss(s);
  std::string item;
  std::vector<std::string> elems;
  while (std::getline(ss, item, delim))
  {
    elems.push_back(item);
    // elems.push_back(std::move(item)); // if C++11 (based on comment from @mchiasson)
  }
  return elems;
}

YAML::Node YAMLRecursiveSearch(YAML::Node node, std::string key, std::string indent)
{
  //std::cout << indent << "Looking at node: (type " << node.Type() << ")"<< std::endl;
  //std::cout << node << std::endl;

  switch (node.Type())
  {
    case YAML::NodeType::Map: // ...
      //std::cout << indent << "Found Map: " << std::endl;

      //if scalar, key it leaf node or not
      //if leaf, check if name matches the key
      if (auto next = node["name"]) 
      {
        //std::cout << indent << "Found Name: " << next.as<std::string>() << std::endl;
        //if our search key matches, we found it!
        if (key.compare(next.as<std::string>()) == 0)
        {
          //std::cout << indent << "Success! Returning Node " << std::endl;
          return node;
        }
      }
      
      if (auto next = node["local"])
      {
        //std::cout << indent << "Searching Local Branch" << std::endl;

        auto result = YAMLRecursiveSearch(next, key, indent+"    ");
        //std::cout << indent << "Searching Local Branch (Complete)" << std::endl;        
        if (result.IsNull() == false)
        {
          // std::cout << indent << "Returning Node" << std::endl;        
          return result;
        }
      }

      if (auto next = node["subtree"])
      {
        //std::cout << indent << "Searching Subtree Branch" << std::endl;
        auto result = YAMLRecursiveSearch(next, key, indent+"    ");
        //std::cout << indent << "Searching Subtree Branch Complete" << std::endl;
        if (result.IsNull() == false)
        {
          //std::cout << indent << "Returning Node" << std::endl;        
          return result;
        }
      }
      //std::cout << indent << "Found Map (complete): " << std::endl;

      //we didn't find it in this subtree! return a null YAML::Node
      return YAML::Node(YAML::NodeType::Null);

    case YAML::NodeType::Sequence: // ...

      //std::cout << indent << "Found Sequence" << std::endl;
      for (YAML::iterator it = node.begin(); it != node.end(); ++it)
      {
        //std::cout << indent << "Looking at sequence item" << std::endl;
        const YAML::Node& next = *it;
        auto result = YAMLRecursiveSearch(next, key, indent+"    ");
        //std::cout << indent << "Looking at sequence item complete" << std::endl;
        if (result.IsNull() == false)
        {
          //std::cout << indent << "Returning Node" << std::endl;        
          return result;
        }

      }
      return YAML::Node(YAML::NodeType::Null);
    case YAML::NodeType::Scalar: // ...
      //std::cout << indent << "Found Scalar: " << std::endl;      
      return YAML::Node(YAML::NodeType::Null);
    case YAML::NodeType::Null: // ...
      //std::cout << indent << "Found Null: " << std::endl;      
      return YAML::Node(YAML::NodeType::Null);
    case YAML::NodeType::Undefined: // ...
      //std::cout << indent << "Found Undefined: " << std::endl;            
      return YAML::Node(YAML::NodeType::Null);
  }

  return YAML::Node(YAML::NodeType::Null);

}


ArchSweepNode::ArchSweepNode()
{
}  

ArchSweepNode::ArchSweepNode(std::string n, std::uint64_t min, std::uint64_t max, std::uint64_t step) :
    name_(n), val_curr_(min), val_min_(min), val_max_(max), val_step_size_(step)
{
}

SweepConstraint::SweepConstraint(std::string var1, std::string var2, std::string op) :
  var1(var1), var2(var2), op(op)
{
}

bool SweepConstraint::IsValid(std::vector<ArchSweepNode> space)
{
  bool var1_found = false;
  bool var2_found = false;
  std::uint64_t val1;
  std::uint64_t val2;
  for (std::size_t i = 0; i < space.size(); i++) {
    if (space[i].name_ == var1) {
      val1 = space[i].val_curr_;
      var1_found = true;
    }
    if (space[i].name_ == var2) {
      val2 = space[i].val_curr_;
      var2_found = true;
    }
  }
  if (var1_found && var2_found) {
    if (op == "=") return val1 == val2;
    if (op == ">=") return val1 >= val2;
    if (op == ">") return val1 > val2;
    if (op == "<=") return val1 <= val2;
    if (op == "<") return val1 < val2;
    std::cout << "Op not supported" << std::endl;
    assert(0);
  } else {
    std::cout << "Constraints must only pertain to sweeping variables" << std::endl;
    assert(0);
  }
}

ArchSpaceNode::ArchSpaceNode()
{
}  

ArchSpaceNode::ArchSpaceNode(std::string n, YAML::Node a, std::string h) :
    name_(n), yaml_(a), header_(h)
{
}


ArchSpace::ArchSpace()
{
}

ArchSpace::ArchSpace(std::string n) :
    name_(n)
{
}

  
ArchSpace* ArchSpace::InitializeFromFile(std::string filename)
{  
  return new FileArchSpace(filename);
}


ArchSpace* ArchSpace::InitializeFromFileList(YAML::Node list_yaml)
{  
  std::cout << "Initializing Architectures from list "  << std::endl;

  std::vector<std::string> filenames;

  for (std::size_t i = 0; i < list_yaml.size(); i++)
  {
    std::string filename = list_yaml[i].as<std::string>();
    filenames.push_back(filename);

    std::cout << "  Using arch file: " << filename  << std::endl;
  }
  return new FileListArchSpace(filenames);
}

ArchSpace* ArchSpace::InitializeFromFileSweep(YAML::Node sweep_yaml)
{
  std::cout << "  Reading arch sweep parameters"  << std::endl;
  std::string base_yaml_filename = sweep_yaml["arch-spec"].as<std::string>();

  //get list of arch variables that change
  // - initialize the space vector
  std::vector<ArchSweepNode> space;

  auto list = sweep_yaml["elements"];
  //traverse file, build up the sweep nodes
  for (std::size_t i = 0; i < list.size(); i++)
  {

    std::string name = list[i]["name"].as<std::string>();
    std::uint64_t min = list[i]["min"].as<std::uint64_t>();
    std::uint64_t max = list[i]["max"].as<std::uint64_t>();
    std::uint64_t step = list[i]["step-size"].as<std::uint64_t>();

    std::cout << "    Adding variable " << name << "  min: "  << min << "  max: " << max << "  stepsize: " << step << std::endl;

    space.push_back(ArchSweepNode(name, min, max, step));
  }

  std::vector<SweepConstraint> sweep_constraints;

  auto constr_list = sweep_yaml["constraints"];
  for (std::size_t i = 0; i < constr_list.size(); i++) {

    std::string constraint = constr_list[i].as<std::string>();

    std::regex re("^([^><=]*[^><=\\s]+)\\s*(>|>=|<|<=|=)\\s*([^><=]+)$");
    std::smatch match;

    if (std::regex_match(constraint, match, re)) {
      std::string var1 = match[1];
      std::string var2 = match[3];
      std::string op = match[2];
      std::cout << "Found constraint: var1 = " << var1 << ", var2 = " << var2 << ", op = " << op << std::endl;
      sweep_constraints.push_back(SweepConstraint(var1, var2, op));
    } else {
      std::cout << "Constraint not recognized: " << constraint << std::endl;
      assert(0);
    }
  }

  return new SweepArchSpace(base_yaml_filename, space, sweep_constraints);
}


FileArchSpace::FileArchSpace(std::string filename) :
  filename_(filename), done_(false)
{

}

std::string FileArchSpace::GetExtraHeaders() {
  return "";
}

bool FileArchSpace::HasNext() {
  return !done_;
}

ArchSpaceNode FileArchSpace::GetNext() 
{
  if (done_) {
    return ArchSpaceNode("", YAML::Node(), "");
  } else {
    std::ifstream fin;
    fin.open(filename_);
    YAML::Node filecontents = YAML::Load(fin);

    ArchSpaceNode new_arch = ArchSpaceNode(filename_, filecontents, "");
    return new_arch;
  }
}

std::uint64_t FileArchSpace::GetMaxSize() {
  return 1;
}

std::uint64_t FileArchSpace::GetIndex() {
  return 0;
}

FileListArchSpace::FileListArchSpace(std::vector<std::string> filenames) :
  filenames_(filenames), i(0)
{

}

std::string FileListArchSpace::GetExtraHeaders()
{
  return "Filename, ";
}

bool FileListArchSpace::HasNext() 
{
  return i < filenames_.size();  
}

ArchSpaceNode FileListArchSpace::GetNext()
{
  if (!HasNext()) {
    return ArchSpaceNode("", YAML::Node(), "");
  } else {
    std::ifstream fin;
    fin.open(filenames_[i]);
    YAML::Node filecontents = YAML::Load(fin);

    ArchSpaceNode new_arch = ArchSpaceNode(filenames_[i], filecontents, "");
    i++;
    return new_arch;
  }
}

std::uint64_t FileListArchSpace::GetMaxSize() 
{
  return filenames_.size();
}

std::uint64_t FileListArchSpace::GetIndex()
{
  return i;
}

SweepArchSpace::SweepArchSpace(std::string base_yaml_filename, std::vector<ArchSweepNode> space, std::vector<SweepConstraint> constraints) :
  base_yaml_filename_(base_yaml_filename), space_(space), constraints_(constraints), done_(false), index_(0)
{
  headers_ = "";
  for (std::size_t i = 0; i < space.size(); i++) {
    headers_ += space[i].name_ + ", ";
  }

  max_size_ = 1;
  for (std::size_t i = 0; i < space.size(); i++) {
    std::uint64_t this_var_size = 0;
    for (std::uint64_t v = space[i].val_min_; v <= space[i].val_max_; v *= space[i].val_step_size_) {
      this_var_size++;
    }
    max_size_ *= this_var_size;
  }

  while (!PassesConstraints() && !done_) {
    AdvanceSweepNodes();
  }
}

std::string SweepArchSpace::GetExtraHeaders() {
  return headers_;
}

bool SweepArchSpace::HasNext() {
  return !done_;
}

ArchSpaceNode SweepArchSpace::GetNext() {
  if (!HasNext()) {
    return ArchSpaceNode("", YAML::Node(), "");
  } else {
    std::cout << "Generating Architecture" << std::endl;

    //load base yaml, then modify using the sweep nodes
    std::ifstream fin;
    fin.open(base_yaml_filename_);
    YAML::Node yaml = YAML::Load(fin);
      
    std::string config_append; //the specific arch details of the arch instance
    std::string header = "";
    std::set<std::string> components;
    for (std::size_t i = 0; i < space_.size(); i++){
      std::uint64_t val = space_[i].val_curr_;
      config_append += "." + space_[i].name_ + "." + std::to_string(val);
      header += std::to_string(val) + ", "; 

      std::vector<std::string> yaml_path = split(space_[i].name_, '.');
      components.insert(yaml_path[0]);

      // std::cout << "Searching for module: " << yaml_path[0] << std::endl;        
      auto active = YAMLRecursiveSearch(yaml["architecture"], yaml_path[0], "");
      if (active.IsNull() == false)
      {
        // std::cout << "Updating node: \n " << active << std::endl;
        active["attributes"][yaml_path[1]] = val;
      }
      else {
        std::cout << "Can't find " << yaml_path[0] << std::endl;
        assert(0);
      }
    }

    for (std::set<std::string>::iterator it = components.begin(); it != components.end(); it++) {
      std::string component_name = *it;
      auto component_yaml = YAMLRecursiveSearch(yaml["architecture"], component_name, "");
      if (model::isBufferClass(component_yaml["class"].as<std::string>())) {
        // Must satisfy invariant: block_size x datawidth x cluster_size == width
        bool is_specified[4] = {false, false, false, false};
        std::uint64_t values[4] = {1, 1, 1, 1};
        std::vector<std::vector<std::string>> names = { {"block-size", "block_size", "n-words"}, 
                                            {"datawidth", "word-bits", "word_width"},
                                            {"cluster-size", "cluster_size"},
                                            {"width", "memory_width", "data_storage_width"} };
        std::uint64_t defaults[4] = {1, 64, 1, 64};
        
        auto attributes = component_yaml["attributes"];
        std::size_t num_specified = 0;

        for (std::size_t i = 0; i < 4; i++) {
          for (auto name : names[i]) {
            if (attributes[name]) {
              is_specified[i] = true;
              values[i] = attributes[name].as<std::uint64_t>();
              break;
            }
          }
          num_specified += is_specified[i];
        }

        std::uint64_t left_hand_side = 1;
        std::uint64_t right_hand_side = values[3];
        
        for (std::size_t i = 0; i < 3; i++) {
          left_hand_side *= values[i];
        }

        if (num_specified == 4) {
          assert (left_hand_side == right_hand_side); //block_size x datawidth x cluster_size == width
          continue;
        }

        if (!is_specified[3]) {
          // RHS not specified, fill in with defaults and calculate RHS
          for (std::size_t i = 0; i < 3; i++) {
            if (!is_specified[i]) {
              attributes[names[i][0]] = defaults[i];
              left_hand_side *= defaults[i];
            }
          }
          attributes["width"] = left_hand_side;
        } else {
          // RHS specified

          if (!is_specified[1]) {
            // Set word bits to default to proceed
            attributes["datawidth"] = defaults[1];
            left_hand_side *= defaults[1];
            values[1] = defaults[1];
          }

          std::uint64_t width = values[3];
          std::uint64_t word_bits = values[1];
          std::uint64_t block_size = values[0];
          std::uint64_t cluster_size = values[2];

          assert (width % (word_bits * block_size) == 0);

          if (is_specified[0] && is_specified[2]) {
            // Block size and cluster size specified
            assert (block_size * word_bits * cluster_size == width);
          } else if (is_specified[2]) {
            // Cluster size specified
            attributes["block-size"] = width / cluster_size / word_bits;
          } else if (is_specified[0]) {
            // Block size specified
            attributes["cluster-size"] = width / (word_bits * block_size);
          } else {
            // Neither specified
            attributes["block-size"] = width / word_bits;
            attributes["cluster-size"] = 1;
          }
        }
      }
    }

    ArchSpaceNode new_arch = ArchSpaceNode(base_yaml_filename_ + config_append, yaml, header);
    PrepareNext();
    return new_arch;
  }
}

std::uint64_t SweepArchSpace::GetMaxSize() 
{
  return max_size_;
}

std::uint64_t SweepArchSpace::GetIndex()
{
  return index_;
}

void SweepArchSpace::PrepareNext() 
{
  AdvanceSweepNodes();
  while (!PassesConstraints() && !done_) {
    AdvanceSweepNodes();
  }
}



void SweepArchSpace::AdvanceSweepNodes() {
  for (std::size_t i = 0; i < space_.size(); i++) {
    std::cout << space_[i].name_ << ": " << space_[i].val_curr_ << ", ";
  }
  std::cout << std::endl;
  //increment (step through) the sweep space
  std::size_t i = 0;
  while (i < space_.size())
  {
    space_[i].val_curr_ *= space_[i].val_step_size_;
    
    //if we ov
    if (space_[i].val_curr_ > space_[i].val_max_)
    {
      //check if we reached the end, we are finished generating
      if ((i + 1) >= space_.size())
      {
        done_ = true;
      }
      //reset and carry to increment the next "digit" of the sweep
      else {
        space_[i].val_curr_ = space_[i].val_min_; //reset
        i++; //move to next
      }
    }
    else { //we dont need to carry, we are done.
      break; 
    }
  }
  index_++;
}

bool SweepArchSpace::PassesConstraints() {
  for (std::size_t j = 0; j < constraints_.size(); j++) {
    if (!constraints_[j].IsValid(space_)) {
      return false;
    }
  }
  return true;
}

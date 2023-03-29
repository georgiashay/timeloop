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
          std::cout << indent << "Returning Node" << std::endl;        
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

  
void ArchSpace::InitializeFromFile(std::string filename)
{    
  std::ifstream fin;
  fin.open(filename);
  YAML::Node filecontents = YAML::Load(fin);

  headers_ = "";

  ArchSpaceNode new_arch = ArchSpaceNode(filename, filecontents, "");
  architectures_.push_back(new_arch);
}


void ArchSpace::InitializeFromFileList(YAML::Node list_yaml)
{  
  std::cout << "Initializing Architectures from list "  << std::endl;

  headers_ = "Filename, ";
  //traverse list, create new nodes and push_back
  for (std::size_t i = 0; i < list_yaml.size(); i++)
  {
    std::string filename = list_yaml[i].as<std::string>();

    std::ifstream fin;
    fin.open(filename);
    YAML::Node filecontents = YAML::Load(fin);
    std::cout << "  Using arch file: " << filename  << std::endl;

    ArchSpaceNode new_arch = ArchSpaceNode(filename, filecontents, filename);
    architectures_.push_back(new_arch);
  }
}

void ArchSpace::InitializeFromFileSweep(YAML::Node sweep_yaml)
{
  std::cout << "  Reading arch sweep parameters"  << std::endl;
  std::string base_yaml_filename = sweep_yaml["arch-spec"].as<std::string>();

  //get list of arch variables that change
  // - initialize the space vector
  std::vector<ArchSweepNode> space;

  headers_ = "";
  auto list = sweep_yaml["elements"];
  //traverse file, build up the sweep nodes
  for (std::size_t i = 0; i < list.size(); i++)
  {

    std::string name = list[i]["name"].as<std::string>();
    int min = list[i]["min"].as<int>();
    int max = list[i]["max"].as<int>();
    int step = list[i]["step-size"].as<int>();

    headers_ += name + ", ";

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

  //iterate through the space
  bool done = false;
  while(!done)
  {
    std::cout << "Generating Architecture" << std::endl;

    //load base yaml, then modify using the sweep nodes
    std::ifstream fin;
    fin.open(base_yaml_filename);
    YAML::Node yaml = YAML::Load(fin);
    //std::cout << "YAML (before) " << yaml << std::endl;
      
    std::string config_append; //the specific arch details of the arch instance
    std::string header = "";
    for (std::size_t i = 0; i < space.size(); i++){
      std::uint64_t val = space[i].val_curr_;
      config_append += "." + space[i].name_ + "." + std::to_string(val);
      header += std::to_string(val) + ", "; 

      std::vector<std::string> yaml_path = split(space[i].name_, '.');

      std::cout << "Searching for module: " << yaml_path[0] << std::endl;        
      auto active = YAMLRecursiveSearch(yaml["architecture"], yaml_path[0], "");
      if (active.IsNull() == false)
      {
        std::cout << "Updating node: \n " << active << std::endl;
        active["attributes"][yaml_path[1]] = val;
      }
      else {
        assert(0);
      }
        
      /*
        std::cout << "    Using variable " << space[i].name_ << "  currval: "  << val << std::endl;

        //traverse the yaml to find the value, then update it
        //the yaml string path can contain numbers (index into list/set), which need to be converted
        std::vector<YAML::Node> nodes; //all the nodes we have to traverse to modify child value
        nodes.push_back(yaml);
        for (std::size_t i = 0; i < yaml_path.size(); i++){

        //atempt to convert to number
        char* end;
        long index = strtol(yaml_path[i].c_str(), &end, 10); 
        // conversion failed because the input wasn't a number
        // using string version
        if (*end) {
        //std::cout << "        Tracing (" << i << ") as key: " << yaml_path[i].c_str() << std::endl;
        nodes.push_back((nodes[i])[yaml_path[i]]);
        }
        else { // use number index
        //std::cout << "        Tracing (" << i << ") as index: " << index << std::endl;
        nodes.push_back((nodes[i])[index]);
        }

          
        }
        //finished finding the node, update it
        nodes[yaml_path.size()] = val;
      */

    }
    //std::cout << "YAML (after) " << yaml << std::endl;

    ArchSpaceNode new_arch = ArchSpaceNode(base_yaml_filename + config_append, yaml, header);
    architectures_.push_back(new_arch);

    bool made_one_valid = false;
    while (!made_one_valid) {
      std::cout << "Increment to next architecture spec." << std::endl;
      for (std::size_t i = 0; i < space.size(); i++) {
        std::cout << space[i].name_ << ": " << space[i].val_curr_ << ", ";
      }
      std::cout << std::endl;
      //increment (step through) the sweep space
      unsigned int i = 0;
      while (i < space.size())
      {
        space[i].val_curr_ *= space[i].val_step_size_;
        
        //if we ov
        if (space[i].val_curr_ > space[i].val_max_)
        {
          //check if we reached the end, we are finished generating
          if ((i + 1) >= space.size())
          {
            done = true;
          }
          //reset and carry to increment the next "digit" of the sweep
          else {
            space[i].val_curr_ = space[i].val_min_; //reset
            i++; //move to next
          }
        }
        else { //we dont need to carry, we are done.
          break; 
        }
      }

      if (done) {
        made_one_valid = true;
      } else {
        bool constraints_passed = true;
        for (std::size_t j = 0; j < sweep_constraints.size(); j++) {
          if (!sweep_constraints[j].IsValid(space)) {
            constraints_passed = false;
            break;
          }
        }
        made_one_valid = constraints_passed;
      }
     
    }

  }

  std::cout << "Generated " << architectures_.size() << " architecture configs" << std::endl;
}

std::string ArchSpace::GetExtraHeaders()
{
  return headers_;
}
  

int ArchSpace::GetSize()
{
  return architectures_.size();
} 

ArchSpaceNode& ArchSpace::GetNode(int index)
{
  return architectures_[index];
}

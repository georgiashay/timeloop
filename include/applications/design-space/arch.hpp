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

#pragma once

#include <fstream>
#include <iomanip>
#include <string>
#include <sstream>
#include <vector>

#include "compound-config/compound-config.hpp"
#include "model/topology.hpp"

class ArchSweepNode
{
 public:
  std::string name_;
  std::uint64_t val_curr_;   
  std::uint64_t val_min_;
  std::uint64_t val_max_;
  std::uint64_t val_step_size_;

  ArchSweepNode();
  ArchSweepNode(std::string n, std::uint64_t min, std::uint64_t max, std::uint64_t step);
};

class SweepConstraint
{
  public:
    SweepConstraint(std::string var1, std::string var2, std::string op);
    bool IsValid(std::vector<std::vector<ArchSweepNode>> spaces);
  private:
    std::string var1;
    std::string var2;
    std::string op;
};

class Derivation
{
  public:
    static Derivation* FromYaml(YAML::Node yaml);
    virtual uint64_t value(std::vector<ArchSweepNode> vars, std::vector<ArchSweepNode> space, YAML::Node arch) = 0;
};

class DeriveOperation : public Derivation
{
  private:
    std::string op_;
    Derivation* operand1_;
    Derivation* operand2_;

  public:
    DeriveOperation(std::string op, Derivation* operand1, Derivation* operand2);
    uint64_t value(std::vector<ArchSweepNode> vars, std::vector<ArchSweepNode> space, YAML::Node arch);
};

class DeriveValue : public Derivation
{
  private:
    std::uint64_t value_;

  public:
    DeriveValue(std::uint64_t value);
    uint64_t value(std::vector<ArchSweepNode> vars, std::vector<ArchSweepNode> space, YAML::Node arch);
};

class DeriveVar : public Derivation
{
  private:
    std::string name_;
  public:
    DeriveVar(std::string name);
    uint64_t value(std::vector<ArchSweepNode> vars, std::vector<ArchSweepNode> space, YAML::Node arch);
};

class DeriveNode
{
  public:
    std::string name_;
    Derivation* derivation_;

    DeriveNode(std::string name, Derivation* derivation);
};


class ArchSpaceNode
{
 public:
  std::string name_; //descriptive name
  YAML::Node yaml_; //text version of YAML/ should be YAML
  std::string header_;

  ArchSpaceNode();
  ArchSpaceNode(std::string n, YAML::Node a, std::string h);
};


class ArchSpace
{
 protected:
  std::string chip_name_;

 public:
  ArchSpace(std::string chip="");

  static ArchSpace* InitializeFromFile(std::string filename);
  static ArchSpace* InitializeFromFileList(YAML::Node list_yaml, std::string chip_name);
  static ArchSpace* InitializeFromFileSweep(YAML::Node sweep_yaml, std::string chip_name);

  virtual std::string GetExtraHeaders() = 0;
  virtual bool HasNext() = 0;
  virtual ArchSpaceNode GetNext() = 0;
  virtual std::uint64_t GetMaxSize() = 0;
  virtual std::uint64_t GetIndex() = 0;

  std::string ChipName() { return chip_name_; };
};

class SweepArchSpace : public ArchSpace
{
  private:
    std::string base_yaml_filename_;
    std::vector<ArchSweepNode> vars_;
    std::vector<ArchSweepNode> space_;
    std::vector<SweepConstraint> constraints_;
    std::vector<DeriveNode> derived_;
    std::string headers_;
    bool done_;
    std::uint64_t max_size_;
    std::uint64_t index_;

  public:
    SweepArchSpace(std::string base_yaml_filename, std::string chip_name, std::vector<ArchSweepNode> vars, std::vector<ArchSweepNode> space, std::vector<SweepConstraint> constraints, std::vector<DeriveNode> derived);
    std::string GetExtraHeaders();
    bool HasNext();
    ArchSpaceNode GetNext();
    std::uint64_t GetMaxSize();
    std::uint64_t GetIndex();

  private:
    std::uint64_t NumVars();
    ArchSweepNode& GetVar(std::uint64_t i);
    void PrepareNext();
    bool PassesConstraints();
    void AdvanceSweepNodes();
};

class FileArchSpace : public ArchSpace
{
  private:
    std::string filename_;
    bool done_;

  public:
    FileArchSpace(std::string filename);
    std::string GetExtraHeaders();
    bool HasNext();
    ArchSpaceNode GetNext();
    std::uint64_t GetMaxSize();
    std::uint64_t GetIndex();
};

class FileListArchSpace : public ArchSpace
{
  private:
    std::vector<std::string> filenames_;
    std::size_t i;
  
  public:
    FileListArchSpace(std::vector<std::string> filenames, std::string chip_name);
    std::string GetExtraHeaders();
    bool HasNext();
    ArchSpaceNode GetNext();
    std::uint64_t GetMaxSize();
    std::uint64_t GetIndex();
};

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
    bool IsValid(std::vector<ArchSweepNode> space);
  private:
    std::string var1;
    std::string var2;
    std::string op;
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
  std::string name_;

 public:
  ArchSpace();
  ArchSpace(std::string n);

  static ArchSpace* InitializeFromFile(std::string filename);
  static ArchSpace* InitializeFromFileList(YAML::Node list_yaml);
  static ArchSpace* InitializeFromFileSweep(YAML::Node sweep_yaml);

  virtual std::string GetExtraHeaders() = 0;
  virtual bool HasNext() = 0;
  virtual ArchSpaceNode GetNext() = 0;
  virtual std::uint64_t GetMaxSize() = 0;
  virtual std::uint64_t GetIndex() = 0;
};

class SweepArchSpace : public ArchSpace
{
  private:
    std::string base_yaml_filename_;
    std::vector<ArchSweepNode> space_;
    std::vector<SweepConstraint> constraints_;
    std::string headers_;
    bool done_;
    std::uint64_t max_size_;
    std::uint64_t index_;

  public:
    SweepArchSpace(std::string base_yaml_filename, std::vector<ArchSweepNode> space, std::vector<SweepConstraint> constraints);
    std::string GetExtraHeaders();
    bool HasNext();
    ArchSpaceNode GetNext();
    std::uint64_t GetMaxSize();
    std::uint64_t GetIndex();

  private:
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
    FileListArchSpace(std::vector<std::string> filenames);
    std::string GetExtraHeaders();
    bool HasNext();
    ArchSpaceNode GetNext();
    std::uint64_t GetMaxSize();
    std::uint64_t GetIndex();
};

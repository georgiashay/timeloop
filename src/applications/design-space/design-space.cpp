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

#include <iomanip>
#include <algorithm>
#include <filesystem>

#include "applications/design-space/design-space.hpp"

using namespace config;

PointResult::PointResult(std::string name, EvaluationResult result, model::Topology::Specs specs, ArchSpaceNode arch) :
    config_name_(name), result_(result), specs_(specs), arch_(arch)
{
}
  
void PointResult::PrintEvaluationResultsHeader(ArchSpace* aspace, std::ostream& out)
{
  out << aspace->GetExtraHeaders();
  out << "Total Computes, Cycles, Area, utilization, pJ/Compute, Mapping" << std::endl;
}

void PointResult::PrintEvaluationResult(std::ostream& out)
{
  out << arch_.header_;
  out << result_.stats.algorithmic_computes;
  out << ", " << result_.stats.cycles;
  out << ", " << specs_.GetArea();
  out << ", " << std::setw(4) << OUT_FLOAT_FORMAT << std::setprecision(4) << result_.stats.utilization;
  out << ", " << std::setw(8) << OUT_FLOAT_FORMAT << PRINTFLOAT_PRECISION << result_.stats.energy / result_.stats.algorithmic_computes;
  out << ", " << result_.mapping.PrintCompact() << std::endl;
}

//--------------------------------------------//
//                Application                 //
//--------------------------------------------//

DesignSpaceExplorer::DesignSpaceExplorer(std::string problemfile, std::string archfile, bool keep_files)
{
  problemspec_filename_ = problemfile;
  archspec_filename_ = archfile;
  keep_files_ = keep_files;
}

// ---------------
// Run the design space exploration.
// ---------------
void DesignSpaceExplorer::Run()
{
  //read in the problem spec space file as YAML, decide type of space we have:
  // 1: list of problems
  // 2: *later* single problem with swept parameters
  std::ifstream pspec_stream;
  pspec_stream.open(problemspec_filename_);
  YAML::Node pspec_yaml = YAML::Load(pspec_stream);

  std::cout << "****** INITIALIZING PROBLEM SPACE ******" << std::endl;    
  ProblemSpace pspec_space;    
  if (auto list = pspec_yaml["problem-space-files"])
  {
    pspec_space.InitializeFromFileList(list);
  }
  else
  {
    pspec_space.InitializeFromFile(problemspec_filename_);      
  }

  //read in the arch spec space file as YAML, decide type of space we have:
  // 1: list of arch to evaluate
  // 2: *later* single arch with swept parameters
  std::ifstream aspec_stream;
  aspec_stream.open(archspec_filename_);
  YAML::Node aspec_yaml = YAML::Load(aspec_stream);

  std::cout << "****** INITIALIZING ARCH SPACE ******" << std::endl;
  ArchSpace* aspec_space;
  if (auto list = aspec_yaml["arch-space-files"])
  {
    aspec_space = ArchSpace::InitializeFromFileList(list);
  }
  else if (auto sweep = aspec_yaml["arch-space-sweep"])
  {
    aspec_space = ArchSpace::InitializeFromFileSweep(sweep);
  }
  else
  {
    aspec_space = ArchSpace::InitializeFromFile(archspec_filename_);      
  }
    
  std::cout << "*** total arch: " << aspec_space->GetMaxSize() << "   total prob: " << pspec_space.GetSize() << std::endl;        

  std::cout << "****** SOLVING ******" << std::endl;  

  std::filesystem::create_directory("results");
  assert (std::filesystem::exists("results"));

  std::string result_filename =  "overview_" + archspec_filename_ + problemspec_filename_ + ".txt";
  replace(result_filename.begin(),result_filename.end(),'/', '.'); 
  std::ofstream result_txt_file("results/" + result_filename);

  PointResult::PrintEvaluationResultsHeader(aspec_space, result_txt_file);

  //main loop, do the full product of problems x arches
  while(aspec_space->HasNext())
  {
    //retrieved via reference
    ArchSpaceNode curr_arch = aspec_space->GetNext();

    std::cout << "*** working on arch : " << curr_arch.name_ << "  " << aspec_space->GetIndex() << " / " << aspec_space->GetMaxSize() << std::endl;        

    for (int problem_id = 0; problem_id < pspec_space.GetSize(); problem_id ++)
    {
      //retrieved via reference
      ProblemSpaceNode curr_problem = pspec_space.GetNode(problem_id);
        
      // use problem and arch to run a mapper
      std::string config_name = curr_arch.name_ + "--" + curr_problem.name_;
      std::cout << "*** working on config : " << config_name << std::endl;        
      replace(config_name.begin(),config_name.end(),'/', '.'); 

      std::string file_name = "results/" + config_name;

      //output the two yaml to a single file
      //CompoundConfigNode arch = CompoundConfigNode(nullptr, YAML::Clone(curr_arch.yaml_));
      //CompoundConfigNode problem = CompoundConfigNode(nullptr, YAML::Clone(curr_problem.yaml_));
      std::ofstream combined_yaml_file("temp_dse.yaml");
      combined_yaml_file << curr_arch.yaml_ << std::endl; // <------------ HERE.
      combined_yaml_file << curr_problem.yaml_ << std::endl; // <------------ HERE.
      combined_yaml_file.close();
      //pull tempfile into a compound config
      config::CompoundConfig config("temp_dse.yaml");

      //std::cout << "arch yaml: \n" << curr_arch.yaml_ << std::endl;
      Application mapper(&config, file_name);
      //SimpleMapper mapper = SimpleMapper(config_name, arch, problem);
      mapper.Run();

      model::Engine::Specs arch_specs = mapper.GetArchSpecs();
      model::Engine engine;
      engine.Spec(arch_specs);
      PointResult result(config_name, mapper.GetGlobalBest(), engine.GetTopology().GetSpecs(), curr_arch);
      result.PrintEvaluationResult(result_txt_file);
      if (!keep_files_) {
        std::filesystem::remove_all(file_name);
      }
      std::cout << "*** total arch: " << aspec_space->GetMaxSize() << "   total prob: " << pspec_space.GetSize() << std::endl;        
        
    }
  }
  free(aspec_space);
  result_txt_file.close();
}

#pragma once
#include "ma/Mesh.h"
#include "ma/Types.h"

#include <atomic>
#include <functional>
#include <string>

namespace ma {

// Progress callback: stage, fraction in [0,1], short message. May be empty.
using ProgressFn = std::function<void(Stage, float, const std::string&)>;

struct PipelineParams {
  bool detectPlanes = true;
  bool detectCylinders = true;
  bool detectSymmetry = true;
};

// Run the full auto-orientation pipeline. Safe to call off the UI thread.
// If `cancel` becomes true the run returns early with whatever is complete.
AnalysisResult analyze(const Mesh& mesh, const ProgressFn& progress = {},
                       const std::atomic<bool>* cancel = nullptr,
                       const PipelineParams& params = {});

}  // namespace ma

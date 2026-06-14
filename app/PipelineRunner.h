#pragma once
#include "ma/AnalysisPipeline.h"
#include "ma/Mesh.h"
#include "ma/Types.h"

#include <atomic>
#include <mutex>
#include <string>
#include <thread>

namespace ma::ui {

// Runs the analysis pipeline on a worker thread and hands the result back to the
// UI thread via poll(). Progress is a lock-protected snapshot polled each frame.
class PipelineRunner {
 public:
  struct Progress {
    ma::Stage stage = ma::Stage::Idle;
    float pct = 0.0f;
    std::string msg;
  };

  ~PipelineRunner() {
    cancel_ = true;
    if (worker_.joinable()) worker_.join();
  }

  bool running() const { return running_.load(); }

  void start(const ma::Mesh& mesh) {
    if (running_.load()) return;
    if (worker_.joinable()) worker_.join();
    cancel_ = false;
    hasResult_ = false;
    running_ = true;
    {
      std::lock_guard<std::mutex> lk(mtx_);
      progress_ = {ma::Stage::Validation, 0.0f, "Starting"};
    }
    ma::Mesh copy = mesh;  // detach from UI-owned mesh for thread safety
    worker_ = std::thread([this, copy = std::move(copy)]() mutable {
      auto cb = [this](ma::Stage s, float p, const std::string& m) {
        std::lock_guard<std::mutex> lk(mtx_);
        progress_ = {s, p, m};
      };
      ma::AnalysisResult r = ma::analyze(copy, cb, &cancel_);
      {
        std::lock_guard<std::mutex> lk(mtx_);
        result_ = std::move(r);
      }
      hasResult_ = true;
      running_ = false;
    });
  }

  void cancelRun() { cancel_ = true; }

  Progress progress() {
    std::lock_guard<std::mutex> lk(mtx_);
    return progress_;
  }

  // On the UI thread: if a finished result is pending, move it out and join.
  bool poll(ma::AnalysisResult& out) {
    if (!hasResult_.exchange(false)) return false;
    if (worker_.joinable()) worker_.join();
    std::lock_guard<std::mutex> lk(mtx_);
    out = std::move(result_);
    return true;
  }

 private:
  std::thread worker_;
  std::atomic<bool> running_{false};
  std::atomic<bool> cancel_{false};
  std::atomic<bool> hasResult_{false};
  std::mutex mtx_;
  Progress progress_;
  ma::AnalysisResult result_;
};

}  // namespace ma::ui

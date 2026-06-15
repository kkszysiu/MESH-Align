#pragma once
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <string>
#include <vector>

// UI-agnostic data-transfer objects shared between the geometry core and the app.
// All values are copyable and contain no GL / ImGui state.
namespace ma {

enum class DatumRole { None, Top, Front, Right };

struct DetectedPlane {
  Eigen::Vector3d point{0, 0, 0};
  Eigen::Vector3d normal{0, 0, 1};
  double confidence = 0.0;      // [0,1]
  double area = 0.0;            // mm^2 of inlier faces
  std::vector<int> inlierFaces;
  DatumRole role = DatumRole::None;
  std::string label;               // e.g. "Face 0"
  std::string tier = "Candidate";  // Primary / Secondary / Tertiary / Candidate
  std::string source = "feature";  // how derived: face / symmetry / bore-seat / pca ...
};

struct DetectedCylinder {
  Eigen::Vector3d axisPoint{0, 0, 0};
  Eigen::Vector3d axisDir{0, 0, 1};
  double radius = 0.0;
  double length = 0.0;  // axial extent of the inlier region
  double confidence = 0.0;
  std::vector<int> inlierFaces;
  std::string label;
};

struct SymmetryPlane {
  Eigen::Vector3d point{0, 0, 0};
  Eigen::Vector3d normal{1, 0, 0};
  double score = 0.0;          // [0,1], higher = more symmetric
};

struct OrientationResult {
  Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();  // part -> datum frame
  Eigen::Vector3d eulerXYZ_deg{0, 0, 0};                    // display only
  Eigen::Vector3d pcaRatio{1, 1, 1};                        // shape elongation (largest:mid:1)
  double confidence = 0.0;
  std::string method;                                       // winning cue(s)
  int topPlaneId = -1, frontPlaneId = -1, rightPlaneId = -1;
};

struct ToleranceBands {
  double nominal = 0.0;
  double inTolMm = 0.1;   // |d| <= inTol  -> in tolerance (green)
  double warnMm = 0.5;    // inTol < |d| <= warn -> warning (yellow)
};

struct DeviationField {
  Eigen::VectorXd signedDistMm;  // per-vertex, + = material excess
  double minMm = 0.0, maxMm = 0.0;
  ToleranceBands bands;
};

struct ValidationReport {
  int components = 0;
  int nonManifoldEdges = 0;
  int degenerateFaces = 0;
  int duplicateVertices = 0;
  int boundaryEdges = 0;
  bool isWatertight = false;
  std::vector<std::string> notes;
};

struct AnalysisResult {
  ValidationReport validation;
  std::vector<DetectedPlane> planes;
  std::vector<DetectedCylinder> cylinders;
  std::vector<SymmetryPlane> symmetry;
  OrientationResult orientation;
};

// Pipeline stages (used for progress reporting in M2+).
enum class Stage {
  Idle, Validation, Normals, Planes, Cylinders, Symmetry, Orientation, Deviation, Done
};

const char* stageName(Stage s);

}  // namespace ma

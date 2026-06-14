# meshalign_core — pure geometry + data. Links NO GL, NO ImGui.
add_library(meshalign_core STATIC
  src/Mesh.cpp
  src/PrimitiveMeshes.cpp
  src/MeshValidation.cpp
  src/Geometry.cpp
  src/PlaneExtractor.cpp
  src/CylinderExtractor.cpp
  src/SymmetryDetector.cpp
  src/Orienter.cpp
  src/Picking.cpp
  src/AnalysisPipeline.cpp
  src/io/IO.cpp
  src/io/LoadSTL.cpp
  src/io/LoadOBJ.cpp
  src/io/LoadPLY.cpp
)
add_library(meshalign::core ALIAS meshalign_core)

target_include_directories(meshalign_core PUBLIC
  "${CMAKE_CURRENT_SOURCE_DIR}/include")

target_link_libraries(meshalign_core PUBLIC eigen)
target_link_libraries(meshalign_core PRIVATE tinyobjloader tinyply nanoflann)
target_compile_features(meshalign_core PUBLIC cxx_std_17)

if(MSVC)
  target_compile_options(meshalign_core PRIVATE /W4 /bigobj)
else()
  target_compile_options(meshalign_core PRIVATE -Wall -Wextra)
endif()

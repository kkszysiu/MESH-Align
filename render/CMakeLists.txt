# meshalign_render — OpenGL 3.3 core rendering. No ImGui.
add_library(meshalign_render STATIC
  src/Shader.cpp
  src/Camera.cpp
  src/Framebuffer.cpp
  src/MeshRenderer.cpp
  src/TriadRenderer.cpp
  src/PlaneRenderer.cpp
  src/Heatmap.cpp
)
add_library(meshalign::render ALIAS meshalign_render)

target_include_directories(meshalign_render PUBLIC
  "${CMAKE_CURRENT_SOURCE_DIR}/include")

target_link_libraries(meshalign_render PUBLIC meshalign_core OpenGL::GL)

if(APPLE)
  target_compile_definitions(meshalign_render PUBLIC GL_SILENCE_DEPRECATION)
else()
  target_link_libraries(meshalign_render PUBLIC glad_gl)
  target_compile_definitions(meshalign_render PUBLIC MA_USE_GLAD)
endif()

# meshalign — the GUI application (Dear ImGui shell).
add_executable(meshalign
  main.cpp
  Panels.cpp
)

target_link_libraries(meshalign PRIVATE
  meshalign_render   # transitively brings meshalign_core
  imgui
  glfw
  OpenGL::GL
  portable_file_dialogs
)
if(NOT APPLE)
  target_link_libraries(meshalign PRIVATE glad_gl)
endif()

target_compile_features(meshalign PRIVATE cxx_std_17)

if(APPLE)
  set_target_properties(meshalign PROPERTIES
    MACOSX_BUNDLE FALSE)   # plain executable for M0; .app bundle comes in M5
endif()

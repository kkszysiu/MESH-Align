# meshalign — the GUI application (Dear ImGui shell).
set(MA_ASSETS "${CMAKE_SOURCE_DIR}/assets")

set(MA_APP_SOURCES main.cpp Panels.cpp)

# ---- platform app-icon resources -------------------------------------------
if(WIN32)
  # Generate an .rc that embeds assets/icon.ico into the executable.
  set(MA_ICON_ICO "${MA_ASSETS}/icon.ico")
  configure_file("${CMAKE_CURRENT_SOURCE_DIR}/meshalign.rc.in"
                 "${CMAKE_CURRENT_BINARY_DIR}/meshalign.rc" @ONLY)
  list(APPEND MA_APP_SOURCES "${CMAKE_CURRENT_BINARY_DIR}/meshalign.rc")
elseif(APPLE)
  set(MA_ICNS "${MA_ASSETS}/icon.icns")
  list(APPEND MA_APP_SOURCES "${MA_ICNS}")
  set_source_files_properties("${MA_ICNS}" PROPERTIES
    MACOSX_PACKAGE_LOCATION "Resources")
endif()

add_executable(meshalign ${MA_APP_SOURCES})

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

# ---- macOS: build a proper .app bundle with the icon ------------------------
if(APPLE)
  set_target_properties(meshalign PROPERTIES
    MACOSX_BUNDLE TRUE
    MACOSX_BUNDLE_BUNDLE_NAME "MESH-Align"
    MACOSX_BUNDLE_GUI_IDENTIFIER "com.meshalign.app"
    MACOSX_BUNDLE_BUNDLE_VERSION "${PROJECT_VERSION}"
    MACOSX_BUNDLE_SHORT_VERSION_STRING "${PROJECT_VERSION}"
    MACOSX_BUNDLE_ICON_FILE "icon.icns")
endif()

# ---- Linux: install icon + .desktop for desktop integration ----------------
if(UNIX AND NOT APPLE)
  include(GNUInstallDirs)
  install(TARGETS meshalign RUNTIME DESTINATION "${CMAKE_INSTALL_BINDIR}")
  install(FILES "${MA_ASSETS}/icon.png"
          DESTINATION "${CMAKE_INSTALL_DATADIR}/icons/hicolor/512x512/apps"
          RENAME "meshalign.png")
  install(FILES "${MA_ASSETS}/meshalign.desktop"
          DESTINATION "${CMAKE_INSTALL_DATADIR}/applications")
endif()

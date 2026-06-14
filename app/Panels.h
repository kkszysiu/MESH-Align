#pragma once

namespace ma::ui {
struct AppState;

void drawMenuBar(AppState& s);
void drawLeftPanel(AppState& s);
void drawRightPanel(AppState& s);
void drawViewport(AppState& s);   // renders the 3D scene to the FBO and shows it

}  // namespace ma::ui

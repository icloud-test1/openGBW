#pragma once

//Methods
void setupScale();
bool tareScale();
// Apply shot offset adjustment when the user exits the finished screen.
// This was previously triggered by a timer; the adjustment now runs when
// the user presses the button to leave the "Grinding finished" state.
void applyShotOffsetAdjustmentOnExit();
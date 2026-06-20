#ifndef WHEEL_LEG_SIMULATE_TOOLS_PLOTTER_H_
#define WHEEL_LEG_SIMULATE_TOOLS_PLOTTER_H_

#include <initializer_list>

namespace mujoco {
class Simulate;
}

namespace wheel_leg {

struct PlotLineValue {
  const char* label;
  double value;
};

void SetPlotSimulateHandle(mujoco::Simulate* sim);
void ResetPlots();

// Updates or creates a figure identified by `figure_key`.
// Repeated calls append new samples to each line's history and refresh the UI.
void PlotLines(const char* figure_key, const char* title, const char* y_format,
               std::initializer_list<PlotLineValue> lines);

}  // namespace wheel_leg

#endif  // WHEEL_LEG_SIMULATE_TOOLS_PLOTTER_H_

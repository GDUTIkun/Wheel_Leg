#include "plotter.h"

#include "simulate.h"

#include <algorithm>
#include <array>
#include <cstdio>
#include <initializer_list>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace wheel_leg {
namespace {

constexpr int kPlotHistory = 1000;
constexpr int kPlotWidth = 720;
constexpr int kPlotHeight = 260;
constexpr int kPlotMargin = 20;
constexpr int kPlotUpdateStride = 10;

struct PlotState {
  std::string title;
  std::string y_format;
  std::vector<std::string> labels;
  std::vector<std::array<double, kPlotHistory>> history;
  int sample_count = 0;
  int update_counter = 0;
};

mujoco::Simulate* g_sim = nullptr;
std::unordered_map<std::string, PlotState> g_plots;

void PushHistory(std::array<double, kPlotHistory>& history, int& sample_count,
                 double value) {
  if (sample_count < kPlotHistory) {
    history[sample_count] = value;
    return;
  }

  for (int i = 1; i < kPlotHistory; ++i) {
    history[i - 1] = history[i];
  }
  history[kPlotHistory - 1] = value;
}

void RebuildFigures() {
  if (!g_sim) {
    return;
  }

  g_sim->user_figures_new_.clear();

  int plot_index = 0;
  for (auto& [key, plot] : g_plots) {
    (void)key;
    if (plot.labels.empty() || plot.sample_count <= 0) {
      continue;
    }

    mjvFigure fig;
    mjv_defaultFigure(&fig);

    std::snprintf(fig.title, sizeof(fig.title), "%s", plot.title.c_str());
    std::snprintf(fig.xlabel, sizeof(fig.xlabel), "%s", "step");
    std::snprintf(fig.yformat, sizeof(fig.yformat), "%s", plot.y_format.c_str());

    fig.figurergba[0] = 0.1f;
    fig.figurergba[3] = 0.6f;
    fig.gridsize[0] = 4;
    fig.gridsize[1] = 4;
    fig.flg_extend = 1;

    double ymin = plot.history[0][0];
    double ymax = plot.history[0][0];

    const int line_count =
        std::min<int>(static_cast<int>(plot.labels.size()), mjMAXLINE);
    for (int line = 0; line < line_count; ++line) {
      std::snprintf(fig.linename[line], sizeof(fig.linename[line]), "%s",
                    plot.labels[line].c_str());
      fig.linepnt[line] = plot.sample_count;
      for (int i = 0; i < plot.sample_count; ++i) {
        const double value = plot.history[line][i];
        ymin = std::min(ymin, value);
        ymax = std::max(ymax, value);
        fig.linedata[line][2 * i] = i - plot.sample_count + 1;
        fig.linedata[line][2 * i + 1] = value;
      }
    }

    // Auto-scale to the currently visible data window instead of keeping a
    // large accumulated range across the whole run. This makes small changes
    // like leg-length tracking much easier to read.
    const double visible_range = ymax - ymin;
    const double center_magnitude =
        std::max(std::abs(ymin), std::abs(ymax));
    const double margin = std::max(1e-4,
                                   std::max(visible_range * 0.2,
                                            center_magnitude * 0.02));
    fig.range[0][0] = -kPlotHistory + 1;
    fig.range[0][1] = 0;
    fig.range[1][0] = ymin - margin;
    fig.range[1][1] = ymax + margin;

    g_sim->user_figures_new_.push_back({
        mjrRect{20, kPlotMargin + plot_index * (kPlotHeight + kPlotMargin),
                kPlotWidth, kPlotHeight},
        fig,
    });
    ++plot_index;
  }

  g_sim->newfigurerequest.store(1);
}

}  // namespace

void SetPlotSimulateHandle(mujoco::Simulate* sim) {
  g_sim = sim;
}

void ResetPlots() {
  g_plots.clear();
  if (!g_sim) {
    return;
  }
  g_sim->user_figures_new_.clear();
  g_sim->newfigurerequest.store(1);
}

void PlotLines(const char* figure_key, const char* title, const char* y_format,
               std::initializer_list<PlotLineValue> lines) {
  if (!figure_key || !title || !y_format || lines.size() == 0) {
    return;
  }

  PlotState& plot = g_plots[figure_key];
  plot.title = title;
  plot.y_format = y_format;

  if (plot.labels.size() != lines.size()) {
    plot.labels.clear();
    plot.history.clear();
    plot.sample_count = 0;
    plot.update_counter = 0;
    plot.labels.reserve(lines.size());
    plot.history.resize(lines.size());
    for (const auto& line : lines) {
      plot.labels.push_back(line.label ? line.label : "");
    }
  }

  int line_index = 0;
  for (const auto& line : lines) {
    if (line_index >= static_cast<int>(plot.history.size())) {
      break;
    }
    PushHistory(plot.history[line_index], plot.sample_count, line.value);
    ++line_index;
  }

  if (plot.sample_count < kPlotHistory) {
    ++plot.sample_count;
  }

  ++plot.update_counter;
  if (plot.update_counter % kPlotUpdateStride == 0) {
    RebuildFigures();
  }
}

}  // namespace wheel_leg

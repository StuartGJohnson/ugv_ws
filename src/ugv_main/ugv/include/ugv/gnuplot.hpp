#include <cstdio>
#include <stdexcept>
#include <string>
#include <vector>
#include <cmath>
#include <thread>
#include <chrono>

#include <algorithm>
#include <cmath>
#include <memory>
#include <string>
#include <unordered_map>
#include <variant>
#include <vector>

#include <iostream>
#include <sstream>

struct XY { std::vector<double> x; std::vector<double> y; };

class GnuplotPipe {
public:
  GnuplotPipe() {
    gp_ = popen("gnuplot -persist", "w");
    if (!gp_) throw std::runtime_error("Failed to open gnuplot");
    cmd("set grid");
  }
  ~GnuplotPipe() {
    if (gp_) pclose(gp_);
  }

  void cmd(const std::string& s) {
    std::fprintf(gp_, "%s\n", s.c_str());
    std::fflush(gp_);
  }

  void plot_xy(const std::vector<double>& ptsx,
               const std::vector<double>& ptsy,
               const std::string& title,
               int window_id,
               const std::string& style = "lines") {
    size_t n = std::min(ptsx.size(), ptsy.size());
    cmd("set term qt " + std::to_string(window_id));
    cmd("set title '" + title + "'");
    cmd("plot '-' with " + style + " title '" + title + "'");
    for (size_t i=0; i<n; i++) {
      std::fprintf(gp_, "%.12g %.12g\n", ptsx[i], ptsy[i]);
    }
    std::fprintf(gp_, "e\n");
    std::fflush(gp_);
  }

  static inline bool finite(double v) { return std::isfinite(v); }

  void plot_two_xy(const std::vector<double>& x1,
                 const std::vector<double>& y1,
                 const std::vector<double>& x2,
                 const std::vector<double>& y2,
                 const std::string& title,
                 int window_id)
  {
    if (!gp_) throw std::runtime_error("gnuplot pipe not open");

    const size_t n1 = std::min(x1.size(), y1.size());
    const size_t n2 = std::min(x2.size(), y2.size());

    std::ostringstream ss;
    ss.precision(12);

    ss << "set term qt " << window_id << "\n"
      << "set output\n"
      << "set grid\n"
      << "set size ratio -1\n"
      << "set title '" << title << "'\n"
      << "plot '-' w l lw 2 lc rgb 'red' title 'GT', "
          "'-' w l lw 2 lc rgb 'blue' title 'EKF'\n";

    // Dataset 1 (skip non-finite points so gnuplot doesn't abort parsing)
    for (size_t i = 0; i < n1; ++i) {
      if (finite(x1[i]) && finite(y1[i])) {
        ss << x1[i] << " " << y1[i] << "\n";
      }
    }
    ss << "e\n";

    // Dataset 2
    for (size_t i = 0; i < n2; ++i) {
      if (finite(x2[i]) && finite(y2[i])) {
        ss << x2[i] << " " << y2[i] << "\n";
      }
    }
    ss << "e\n";

    const std::string s = ss.str();
    std::fwrite(s.data(), 1, s.size(), gp_);
    std::fflush(gp_);

    // Optional extra delay if your test exits immediately after plotting.
    // std::this_thread::sleep_for(std::chrono::milliseconds(500));
  }

  void plot_two_ty(const std::vector<double>& x1,
                 const std::vector<double>& y1,
                 const std::vector<double>& x2,
                 const std::vector<double>& y2,
                 const std::string& title,
                 int window_id,
                 const std::string& xy1_name="GT",
                 const std::string& xy2_name="EKF"
                )
  {
    if (!gp_) throw std::runtime_error("gnuplot pipe not open");

    const size_t n1 = std::min(x1.size(), y1.size());
    const size_t n2 = std::min(x2.size(), y2.size());

    std::ostringstream ss;
    ss.precision(12);

    ss << "set term qt " << window_id << "\n"
      << "set output\n"
      << "set grid\n"
      << "set title '" << title << "'\n"
      << "plot '-' w l lw 2 lc rgb 'red' title '" << xy1_name << "', "
          "'-' w l lw 2 lc rgb 'blue' title '" << xy2_name << "'\n";

    // Dataset 1 (skip non-finite points so gnuplot doesn't abort parsing)
    for (size_t i = 0; i < n1; ++i) {
      if (finite(x1[i]) && finite(y1[i])) {
        ss << x1[i] << " " << y1[i] << "\n";
      }
    }
    ss << "e\n";

    // Dataset 2
    for (size_t i = 0; i < n2; ++i) {
      if (finite(x2[i]) && finite(y2[i])) {
        ss << x2[i] << " " << y2[i] << "\n";
      }
    }
    ss << "e\n";

    const std::string s = ss.str();
    std::fwrite(s.data(), 1, s.size(), gp_);
    std::fflush(gp_);
  }

private:
  FILE* gp_{nullptr};
};

XY make_sinc_data()
{
  const int N = 50;
  const double xmin = -10.0;
  const double xmax = 10.0;

  std::vector<double> ptsx;
  std::vector<double> ptsy;
  ptsx.reserve(N);
  ptsy.reserve(N);

  for (int i = 0; i < N; ++i) {
    const double x = xmin + (xmax - xmin) * i / (N - 1);

    double y;
    if (std::abs(x) < 1e-12) {
      y = 1.0;   // lim sin(x)/x → 1
    } else {
      y = std::sin(x) / x;
    }

    ptsx.push_back(x);
    ptsy.push_back(y);
  }

  XY xy;
  xy.x = ptsx;
  xy.y = ptsy;

  return xy;
}
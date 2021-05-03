#ifndef TOOLS_TIC_TOC_H_
#define TOOLS_TIC_TOC_H_

#include <ctime>
#include <cstdlib>
#include <chrono>

class TicToc {
  public:
    TicToc() {
        tic();
    }

    TicToc(bool is_display) {
        display_ = is_display;
        tic();
    }

    void tic() {
        start = std::chrono::system_clock::now();
    }

    void toc(std::string about_task) {
        end = std::chrono::system_clock::now();
        std::chrono::duration<double> elapsed_seconds = end - start;
        double elapsed_ms = elapsed_seconds.count() * 1000;

        if (display_) {
            std::cout.precision(3);
            std::cout << about_task << ": " << elapsed_ms << " msec." << std::endl;
        }
    }

  private:
    std::chrono::time_point<std::chrono::system_clock> start, end;
    bool display_ = false;
};
#endif

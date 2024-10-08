/*
* Copyright (c) 2016 Carnegie Mellon University, Author <dimatura@cmu.edu>
*
* For License information please see the LICENSE file in the root directory.
*
*/

#include <chrono>
#include <iostream>
#include <thread>
#include "tictoc_profiler/profiler.hpp"

long stupid_fibonacci(int i) {
  if (i == 1 || i == 2) {
    return 1;
  }
  return stupid_fibonacci(i - 1) + stupid_fibonacci(i - 2);
}

long less_stupid_fibonacci(int i) {
  if (i == 1 || i == 2) {
    return 1;
  }
  long f_im1 = 1, f_im2 = 1, f_i = 2;
  for (int ctr = 3; ctr < i; ++ctr) {
    f_im2 = f_im1;
    f_im1 = f_i;
    f_i = f_im1 + f_im2;
  }
  return f_i;
}

void sleeper(int ms) {
  ca::AutoTicToc tt("raii sleeper");
  // *sleeps intensely*
  std::this_thread::sleep_for(std::chrono::milliseconds(ms));
}

void sleeper2(int ms) {
  std::this_thread::sleep_for(std::chrono::milliseconds(ms));
}

int main(int argc, char *argv[]) {
  ca::Profiler::enable();

  constexpr int fib = 32;

  for (int i = 0; i < 6; ++i) {
    std::cout << "f = " << (fib + i) << "\n";
    ca::Profiler::tictoc("stupid_fibonacci");
    long f1 = stupid_fibonacci(fib + i);
    ca::Profiler::tictoc("stupid_fibonacci");
    std::cerr << "f1 = " << f1 << std::endl;

    CA_TICTOC("less_stupid_fibonacci")
    long f2 = less_stupid_fibonacci(fib + i);
    CA_TICTOC("less_stupid_fibonacci")
    std::cerr << "f2 = " << f2 << std::endl;
  }

  for (int i = 0; i < 8; ++i) {
    sleeper(28);
  }

  for (int i = 0; i < 8; ++i) {
    {
      ca::AutoTicToc tt("raii sleeper2");
      sleeper2(28);
    }

    CA_TICTOC("sleeper2");
    sleeper2(28);
    CA_TICTOC("sleeper2");
  }

  // print *all* entries, lots of info but useful for later analyses
  // see also the message option
  // ca::Profiler::print_all(std::cerr);

  // print aggregated in CSV format, useful to parse in a script
  // ca::profiler::print_aggregated_csv(std::cerr);

  // the most useful one imo
  ca::Profiler::print_aggregated(std::cerr);
  return 0;
}

/**
 * Copyright (c) 2015-2018 Carnegie Mellon University, Daniel Maturana
 * <dimatura@cmu.edu>
 *
 * For License information please see the LICENSE file in the root directory.
 *
 */

#ifndef PROFILER_HPP_JUEKLQ5B
#define PROFILER_HPP_JUEKLQ5B

#include <stdint.h>
#include <time.h>

#include <chrono>
#include <iostream>
#include <string>
#include <vector>

#include <ros/ros.h>

#include "tictoc_profiler/flat_map.hpp"

/**
 * Useful for profiling at the function level, e.g.
 * void myfunction() {
 *   CA_FUN_TICTOC
 *   ... do stuff ...
 *   CA_FUN_TICTOC
 * }
 */
#define CA_FUN_TICTOC \
  { ca::Profiler::tictoc(__PRETTY_FUNCTION__); }

/**
 * Convenience macro. Use like CA_TICTOC("foo") ...code... CA_TICTOC("foo")
 */
#define CA_TICTOC(x) \
  { ca::Profiler::tictoc(x); }

#define CA_AUTO_FUN_TICTOC \
  ca::AutoTicToc _autotictoc_fun_tt(__PRETTY_FUNCTION__);

#define CA_AUTO_TICTOC(x) ca::AutoTicToc _autotictoc_tt##x(#x);

namespace ca {

using tictoc_timestamp_t = std::chrono::time_point<std::chrono::system_clock>;

/**
 * Profiler keeps a vector of entries -- it logs *all* tic-tocs, for
 * fine-grained analyses
 */
struct ProfilerEntry {
  tictoc_timestamp_t start_time;
  tictoc_timestamp_t end_time;
  ProfilerEntry(tictoc_timestamp_t _start_time)
      : start_time(_start_time), end_time() {}
};

/**
 * Thin wrapper around vector, adding 'running' info.
 */
class ProfilerEntries {
 public:
  using EntriesVec = std::vector<ProfilerEntry>;
  using iterator = EntriesVec::iterator;
  using const_iterator = EntriesVec::const_iterator;

  bool running() { return running_; }

  void start() { running_ = true; }
  void stop() { running_ = false; }

  iterator begin() { return entries_vec_.begin(); }
  iterator end() { return entries_vec_.end(); }

  const_iterator cbegin() const { return entries_vec_.cbegin(); }
  const_iterator cend() const { return entries_vec_.cend(); }

  size_t size() const { return entries_vec_.size(); }

  ProfilerEntry& back() { return entries_vec_.back(); }

  void create_and_push(tictoc_timestamp_t ts) { entries_vec_.emplace_back(ts); }

  void push_back(const ProfilerEntry& entry) { entries_vec_.emplace_back(entry); }

  void pop_back() { entries_vec_.pop_back(); }

 private:
  bool running_ = false;
  EntriesVec entries_vec_;
};

/**
 * Represents aggregated output, summarizing the entries
 */
struct AggregatedProfilerEntry {
  double total_ms = 0.;
  double min_ms = 0.;
  double max_ms = 0.;
  double avg_ms = 0.;
  double median_ms = 0.;
  double std_ms = 0.;
  int num_calls = 0.;
  std::string name;
};

/**
 * Main class that keeps track of tictocs. Note that this used statically,
 * it is not meant to be instantiated.
 */
class Profiler {
 public:
  // this is the only line that needs to change to use std::map instead of chobo
  // using ProfilerEntryMap = std::map<std::string, ProfilerEntries>;
  using ProfilerEntryMap = chobo::flat_map<std::string, ProfilerEntries>;

 private:
  using us_t = std::chrono::microseconds;
  // floating point milliseconds
  using ms_fp_t =
      std::chrono::duration<double, std::chrono::milliseconds::period>;

 public:
  /**
   * Main function for profiling.
   * Use like this:
   * ca::Profiler::tictoc
   *
   *
   * See src/example.cpp for example.
   */
  static int64_t tictoc(const std::string& name);

  static void enable();
  static void disable();

  static void print_aggregated(std::ostream& os);
  static void print_aggregated_csv(std::ostream& os);
  static void print_all(std::ostream& os);
  static void publish_messages(ros::NodeHandle& nh);

 private:
  Profiler(const Profiler& other) = delete;
  Profiler& operator=(const Profiler& other) = delete;

  static void aggregate_entries(
      std::vector<AggregatedProfilerEntry>* aggregated);

 private:
  static bool enabled_;

  static ProfilerEntryMap entries_map_;
};

/**
 * Automatically perform tictoc with the RAII idiom.
 * use like so:
 *
 * void foo() {
 *   AutoTicToc tt("foo"); // tictoc gets called once
 *   ... do work ...
 *   // tictoc gets called again automatically when it goes of scope.
 * }
 * Note that this approach doesn't really work well with recursive functions.
 *
 * can also be used with bare {} blocks, e.g.
 * ... code
 * { AutoTicToc tt("bar");
 * ... work
 * } // goes out of scope here.
 * ... code
 *
 */
class AutoTicToc {
 public:
  AutoTicToc(const std::string& name) {
    // TODO keep the iterator from the first time,
    // to avoid the lookup cost
    name_ = name;
    ca::Profiler::tictoc(name_);
  }
  AutoTicToc(const AutoTicToc& other) = delete;
  AutoTicToc& operator=(const AutoTicToc& other) = delete;

  ~AutoTicToc() { ca::Profiler::tictoc(name_); }

 private:
  std::string name_;
};
}
#endif /* end of include guard: PROFILER_HPP_JUEKLQ5B */

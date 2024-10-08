/**
 * Copyright (c) 2015 Carnegie Mellon University, Daniel Maturana
 * <dimatura@cmu.edu>
 *
 * For License information please see the LICENSE file in the root directory.
 *
 */

#include "tictoc_profiler/profiler.hpp"

#include <algorithm>
#include <iomanip>
#include <limits>

#include <sys/time.h>

// ROS message profiler entry
#include "tictoc_profiler/ProfilerEntry.h"

namespace ca {

// initialize static data
bool Profiler::enabled_ = false;
Profiler::ProfilerEntryMap Profiler::entries_map_;

static inline bool aggregated_profiler_entry_total_cmp(
    const AggregatedProfilerEntry& a,
    const AggregatedProfilerEntry& b) {
  return (a.avg_ms > b.avg_ms);
};

void Profiler::enable() { Profiler::enabled_ = true; }
void Profiler::disable() { enabled_ = false; }

int64_t Profiler::tictoc(const std::string& name) {
  if (!enabled_) {
    return 0;
  }
  tictoc_timestamp_t timestamp(std::chrono::system_clock::now());

  ProfilerEntryMap::iterator entries_itr(entries_map_.find(name));

  if (entries_itr == entries_map_.end()) {
    // entries not in map. create new entries and mark it as running
    ProfilerEntries entries;
    entries.start();
    entries.create_and_push(timestamp);
    entries_map_[name] = entries;
    return 0;
  }

  ProfilerEntries& entries(entries_itr->second);
  if (!entries.running()) {
    // entries exist but not running. toggle state and start running
    entries.start();
    entries.create_and_push(timestamp);
    return 0;
  }

  // entry exists and is running, stop it and update info.
  entries.stop();
  ProfilerEntry& entry(entries.back());
  entry.end_time = timestamp;

  // TODO should we return nanoseconds instead?
  us_t delta_us =
      std::chrono::duration_cast<us_t>(entry.end_time - entry.start_time);
  return delta_us.count();
}

void Profiler::aggregate_entries(
    std::vector<AggregatedProfilerEntry>* aggregated) {
  if (!enabled_) {
    return;
  }

  for (ProfilerEntryMap::iterator itr(entries_map_.begin()),
       end_itr(entries_map_.end());
       itr != end_itr;
       ++itr) {
    if (itr->second.running()) {
      ROS_WARN_STREAM("ca::tictoc entry " << itr->first
                                          << " was running when program ended");
      // ignore last tictoc
      itr->second.pop_back();
    }
    AggregatedProfilerEntry ag;
    ag.name = itr->first;
    ag.min_ms = std::numeric_limits<double>::max();
    ag.max_ms = -1;
    ag.total_ms = 0.;
    ProfilerEntries& name_entries(itr->second);
    ag.num_calls = static_cast<int>(name_entries.size());

    std::vector<double> delta_ms_vec;
    delta_ms_vec.reserve(name_entries.size());

    for (const ProfilerEntry& entry : name_entries) {
      double delta_ms = (ms_fp_t(entry.end_time - entry.start_time)).count();
      ag.total_ms += delta_ms;
      ag.min_ms = std::min(ag.min_ms, delta_ms);
      ag.max_ms = std::max(ag.max_ms, delta_ms);
      delta_ms_vec.emplace_back(delta_ms);
    }
    ag.avg_ms = ag.total_ms / ag.num_calls;

    // median
    size_t med_ix(delta_ms_vec.size() / 2);
    std::nth_element(delta_ms_vec.begin(),
                     delta_ms_vec.begin() + med_ix,
                     delta_ms_vec.end());
    ag.median_ms = delta_ms_vec.at(med_ix);

    // std
    double sum_squares = 0.;
    for (double ms : delta_ms_vec) {
      sum_squares += (ms - ag.avg_ms) * (ms - ag.avg_ms);
    }
    ag.std_ms = std::sqrt(sum_squares / delta_ms_vec.size());

    aggregated->emplace_back(ag);
  }
}

void Profiler::print_aggregated(std::ostream& os) {
  if (!enabled_) {
    return;
  }
  std::vector<AggregatedProfilerEntry> aggregated;
  aggregate_entries(&aggregated);
  std::sort(aggregated.begin(),
            aggregated.end(),
            aggregated_profiler_entry_total_cmp);
  os << "\n\n";
  os << std::setw(60) << std::setfill(' ') << "Description";
  os << std::setw(20) << std::setfill(' ') << "Calls";
  os << std::setw(20) << std::setfill(' ') << "Total ms";
  os << std::setw(20) << std::setfill(' ') << "Avg ms";
  os << std::setw(20) << std::setfill(' ') << "Med ms";
  os << std::setw(20) << std::setfill(' ') << "Std ms";
  os << std::setw(20) << std::setfill(' ') << "Min ms";
  os << std::setw(20) << std::setfill(' ') << "Max ms";
  os << "\n";
  for (size_t i = 0; i < aggregated.size(); ++i) {
    AggregatedProfilerEntry& ag(aggregated[i]);
    os << std::setw(60) << std::setfill(' ') << ag.name;
    os << std::setw(20) << std::setprecision(5) << std::setfill(' ')
       << ag.num_calls;
    os << std::setw(20) << std::setprecision(5) << std::setfill(' ')
       << ag.total_ms;
    os << std::setw(20) << std::setprecision(5) << std::setfill(' ')
       << ag.avg_ms;
    os << std::setw(20) << std::setprecision(5) << std::setfill(' ')
       << ag.median_ms;
    os << std::setw(20) << std::setprecision(5) << std::setfill(' ')
       << ag.std_ms;
    os << std::setw(20) << std::setprecision(5) << std::setfill(' ')
       << ag.min_ms;
    os << std::setw(20) << std::setprecision(5) << std::setfill(' ')
       << ag.max_ms;
    os << "\n";
  }
  os << "\n";
}

void Profiler::print_aggregated_csv(std::ostream& os) {
  if (!enabled_) {
    return;
  }
  std::vector<AggregatedProfilerEntry> aggregated;
  aggregate_entries(&aggregated);
  std::sort(aggregated.begin(),
            aggregated.end(),
            aggregated_profiler_entry_total_cmp);
  os << "description";
  os << ",calls";
  os << ",total_ms";
  os << ",avg_ms";
  os << ",median_ms";
  os << ",std_ms";
  os << ",min_ms";
  os << ",max_ms";
  os << "\n";
  for (AggregatedProfilerEntry& ag : aggregated) {
    os << ag.name;
    os << "," << ag.num_calls;
    os << "," << ag.total_ms;
    os << "," << ag.avg_ms;
    os << "," << ag.median_ms;
    os << "," << ag.std_ms;
    os << "," << ag.min_ms;
    os << "," << ag.max_ms;
    os << "\n";
  }
  os << "\n";
}

void Profiler::print_all(std::ostream& os) {
  if (!enabled_) {
    return;
  }
  os << "start_time; ";
  os << "description; ";
  os << "duration";
  os << "\n";
  for (ProfilerEntryMap::iterator itr = entries_map_.begin();
       itr != entries_map_.end();
       ++itr) {
    std::string name(itr->first);
    ProfilerEntries& name_entries(itr->second);
    for (const ProfilerEntry& entry : name_entries) {
      int64_t start_time_ns(entry.start_time.time_since_epoch().count());
      os << start_time_ns << "; ";
      os << name << "; ";
      os << (entry.end_time - entry.start_time).count();
      os << "\n";
    }
  }
}

void Profiler::publish_messages(ros::NodeHandle& nh) {
  ros::Publisher pub =
      nh.advertise<tictoc_profiler::ProfilerEntry>("profiler", 20);
  int seq = 0;
  for (ProfilerEntryMap::iterator itr = entries_map_.begin();
       itr != entries_map_.end();
       ++itr) {
    std::string name(itr->first);
    ProfilerEntries& name_entries(itr->second);
    for (const ProfilerEntry& entry : name_entries) {
      tictoc_profiler::ProfilerEntry entry_msg;
      entry_msg.seq = seq++;
      entry_msg.name = name;
      // NOTE this is in nanoseconds afaict, not microseconds as before.
      entry_msg.start_time = entry.start_time.time_since_epoch().count();
      entry_msg.end_time = entry.end_time.time_since_epoch().count();

      double delta_ms = (ms_fp_t(entry.end_time - entry.start_time)).count();
      entry_msg.delta_time_ms = delta_ms;
      pub.publish(entry_msg);
    }
  }
}
}

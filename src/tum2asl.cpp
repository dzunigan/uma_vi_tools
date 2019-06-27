// Copyright (C) 2019  David Zuñiga-Noël <dzuniga at uma.es>

#define PROGRAM_NAME \
    "tum2asl"

#define FLAGS_CASES                                                                                \
    FLAG_CASE(double, time_offset, 0.0, "Timestamp offset [s]")

#define ARGS_CASES                                                                                 \
    ARG_CASE(input_file)                                                                           \
    ARG_CASE(output_file)

// STL
#include <cmath>
#include <iostream>
#include <utility>
#include <vector>

// Boost
#include <boost/filesystem.hpp>

#include "args.hpp"
#include "io.hpp"
#include "macros.h"

using TrajectoryTUM = std::vector<io::trajectory_t<double>>;

static const std::string TRAJ_HEADER = "#timestamp [ns],tx,ty,tz,qw,qx,qy,qz";

void ValidateArgs() {
    RUNTIME_ASSERT(boost::filesystem::is_regular_file(ARGS_input_file));
}

void ValidateFlags() {

}

int main(int argc, char* argv[]) {

    // Handle help flag
    if (args::HelpRequired(argc, argv)) {
        args::ShowHelp();
        return 0;
    }

    // Parse input flags
    args::ParseCommandLineNonHelpFlags(&argc, &argv, true);

    // Check number of args
    if (argc-1 != args::NumArgs()) {
        args::ShowHelp();
        return -1;
    }

    // Parse input args
    args::ParseCommandLineArgs(argc, argv);

    // Validate input arguments
    ValidateFlags();
    ValidateArgs();

    // Read input trajectory
    TrajectoryTUM tum = io::read_file<TrajectoryTUM::value_type>(ARGS_input_file);

    // Apply time offset (if any)
    if (std::abs(FLAGS_time_offset) > 0) {
        for (TrajectoryTUM::value_type& elem : tum)
            elem.timestamp += FLAGS_time_offset;
    }

    io::Trajectory asl;
    asl.reserve(tum.size());
    for (const TrajectoryTUM::value_type& elem : tum) {
        io::timestamp_t timestamp = static_cast<io::timestamp_t>(elem.timestamp * 1e9);
        io::pose_t pose = elem.pose;

        std::swap(pose.qw, pose.qx);
        std::swap(pose.qw, pose.qy);
        std::swap(pose.qw, pose.qz);
        asl.push_back(io::trajectory_t<io::timestamp_t>(timestamp, pose));
    }

    bool saved = io::write_file<>(asl, ARGS_output_file, TRAJ_HEADER);
    RUNTIME_ASSERT(saved);

    return 0;
}

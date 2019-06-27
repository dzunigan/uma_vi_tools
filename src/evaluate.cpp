// Copyright (C) 2019  David Zuñiga-Noël <dzuniga at uma.es>

#define PROGRAM_NAME \
    "evaluate"

#define FLAGS_CASES                                                                                \
    FLAG_CASE(int64, time_offset, 0ll, "Time offset added to trajectory [ns]")                     \
    FLAG_CASE(uint64, max_difference, 2000000ull, "Max time difference for associaiton [ns]")      \
    FLAG_CASE(bool, compute_scale, false, "Compute Sim(3) intead of SE(3) transformations")        \
    FLAG_CASE(uint64, o, 0ull, "Sequence offset for input trajectory")                             \
    FLAG_CASE(uint64, s, 0ull, "Skip sequence elements for input trajectory")                      \
    FLAG_CASE(uint64, n, 0ull, "Max sequence length for input trajectory")                         \

#define ARGS_CASES                                                                                 \
    ARG_CASE(reference)                                                                            \
    ARG_CASE(trajectory)

// STL
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <iostream>
#include <utility>
#include <vector>

// Boost
#include <boost/filesystem.hpp>

// Eigen
#include <Eigen/Core>
#include <Eigen/Geometry>

#include "colmap/math.hpp"

#include "args.hpp"
#include "associate.hpp"
#include "decimate.hpp"
#include "io.hpp"
#include "macros.h"
#include "statistics.hpp"

inline double translation_distance(const Eigen::Isometry3d& T) {
    return T.translation().norm();
}

inline double rotation_angle(const Eigen::Isometry3d& T) {
    return std::acos(colmap::Clip(0.5*(T.linear().trace() - 1.0), -1.0, 1.0));
}

void ValidateArgs() {
    RUNTIME_ASSERT(boost::filesystem::is_regular_file(ARGS_reference));
    RUNTIME_ASSERT(boost::filesystem::is_regular_file(ARGS_trajectory));
}

void ValidateFlags() {

}

Eigen::Isometry3d align(const io::Trajectory& reference, const io::Trajectory& trajectory, bool compute_scale = false, int* N_ptr = nullptr) {
    // Compute pose associations
    std::vector<std::pair<std::size_t, std::size_t>> pairs = associate(reference, trajectory, FLAGS_max_difference);

    const int N = pairs.size();
    RUNTIME_ASSERT(N >= 3);
    if (N_ptr != nullptr) *N_ptr = N;

    // Compute least-squares rigid body transformation
    Eigen::MatrixXd src(3, N);
    Eigen::MatrixXd dst(3, N);

    int index = 0;
    for (const std::pair<std::size_t, std::size_t>& match : pairs) {
        const io::pose_t ref = reference.at(match.first).pose;
        const io::pose_t traj = trajectory.at(match.second).pose;

        dst.col(index) = Eigen::Vector3d(ref.tx, ref.ty, ref.tz);
        src.col(index) = Eigen::Vector3d(traj.tx, traj.ty, traj.tz);
        index++;
    }

    Eigen::Matrix4d Tmatrix = Eigen::umeyama(src, dst, compute_scale);

    Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
    T.linear() = Tmatrix.block<3, 3>(0, 0);
    T.translation() = Tmatrix.block<3, 1>(0, 3);
    return T;
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

    // Read input trajectories
    io::Trajectory reference = io::read_file<io::Trajectory::value_type>(ARGS_reference);
    io::Trajectory trajectory = io::read_file<io::Trajectory::value_type>(ARGS_trajectory);

    // Apply time offset (if any)
    if (std::abs(FLAGS_time_offset) > 0) {
        for (io::Trajectory::value_type& elem : trajectory)
            elem.timestamp += FLAGS_time_offset;
    }

    // Decimate reference trajectory
    trajectory = decimate_sequence(trajectory, FLAGS_o, FLAGS_s, FLAGS_n);

    // Compute breakpoint
    RUNTIME_ASSERT(reference.size() >= 2);
    io::timestamp_t breakpoint = reference[1].timestamp;
    io::timestamp_t max_diff = reference[1].timestamp - reference[0].timestamp;
    for (std::size_t i = 2; i < reference.size(); ++i) {
        io::timestamp_t diff = reference[i].timestamp - reference[i-1].timestamp;
        if (max_diff < diff) {
            max_diff = diff;
            breakpoint = reference[i].timestamp;
        }
    }

    // Split reference trajectory
    io::Trajectory start, end;
    for (const io::Trajectory::value_type& entry : reference) {
        if (entry.timestamp < breakpoint) {
            start.push_back(entry);
        } else {
            end.push_back(entry);
        }
    }

    // Independent align
    int N_start, N_end;
    Eigen::Isometry3d T_s = align(start, trajectory, FLAGS_compute_scale, &N_start);
    Eigen::Isometry3d T_e = align(end, trajectory, FLAGS_compute_scale, &N_end);

    const double start_scale = std::cbrt(T_s.linear().determinant());
    const double end_scale = std::cbrt(T_e.linear().determinant());

    std::cout << "scale: " << std::endl;
    std::cout << "  start_segment " << start_scale << std::endl;
    std::cout << "  end_segment " << end_scale << std::endl;

    T_s.linear() /= start_scale;
    T_e.linear() /= end_scale;

    // Compute drift
    Eigen::Isometry3d T_drift;
    T_drift = T_e*T_s.inverse(); // TODO incorrect

    std::cout << "computed_pairs: " << std::endl;
    std::cout << "  start_segment " << N_start << std::endl;
    std::cout << "  end_segment " << N_end << std::endl;

    std::cout << "translational_drift " << translation_distance(T_drift) << " m" << std::endl;
    std::cout << "rotational_drift " << colmap::RadToDeg(rotation_angle(T_drift)) << " deg" << std::endl;

    if (FLAGS_compute_scale) {
        // TODO output scale drift
    }
    std::cout << std::endl;

    std::vector<double> abs_errors;
    abs_errors.reserve(trajectory.size());
    for (const io::Trajectory::value_type& entry : trajectory) {
        const io::pose_t pose = entry.pose;
        const Eigen::Vector3d pi = Eigen::Vector3d(pose.tx, pose.ty, pose.tz);
        
        Eigen::Vector3d error = T_s * pi - T_e * pi;
        abs_errors.push_back(error.norm());
    }

    std::cout << "computed_pairs " << abs_errors.size() << std::endl;
    std::cout << "translational_alignment_error.rmse " << vector_rms<double>(abs_errors) << " m" << std::endl;
    std::cout << "translational_alignment_error.mean " << vector_mean<double>(abs_errors) << " m" << std::endl;
    std::cout << "translational_alignment_error.median " << vector_median<double>(abs_errors) << " m" << std::endl;
    std::cout << "translational_alignment_error.std " << vector_stdv<double>(abs_errors) << " m" << std::endl;

    std::pair<std::vector<double>::const_iterator, std::vector<double>::const_iterator> min_max 
         = std::minmax_element(abs_errors.cbegin(), abs_errors.cend());
    std::cout << "translational_alignment_error.min " << *(min_max.first) << " m" << std::endl;
    std::cout << "translational_alignment_error.max " << *(min_max.second) << " m" << std::endl;

    return 0;
}


#ifndef IO_HPP_
#define IO_HPP_

// STL
#include <algorithm>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

// Eigen
#include <Eigen/Core>
#include <Eigen/Geometry>

// ROS
//#include <ros/ros.h>

#include "string.hpp"

namespace io {

static std::string OUTPUT_SEPARATOR = ",";
static unsigned int OUTPUT_PRECISION = 18;

// Types
using timestamp_t = std::uint64_t;
using record_t = std::pair<timestamp_t, std::string>;

struct pose_t {
    double tx, ty, tz, qw, qx, qy, qz;

    pose_t()
        : tx(0.0), ty(0.0), tz(0.0), qw(1.0), qx(0.0), qy(0.0), qz(0.0)
    { }

    pose_t(double tx, double ty, double tz, double qw, double qx, double qy, double qz)
        : tx(tx), ty(ty), tz(tz), qw(qw), qx(qx), qy(qy), qz(qz)
    {
        q_normalize();
    }

    // From Eigen Transform (constructor)
    template<typename Scalar, int Type>
    pose_t(const Eigen::Transform<Scalar, 3, Type> &T) {
        Eigen::Quaternion<Scalar> q(T.rotation());
        q.normalize();

        tx = T.translation()(0);
        ty = T.translation()(1);
        tz = T.translation()(2);
        qw = q.w();
        qx = q.x();
        qy = q.y();
        qz = q.z();
    }

    // To Eigen Transform (implicit conversion)
    template<typename Scalar, int Type>
    operator Eigen::Transform<Scalar, 3, Type>() const {
        Eigen::Quaternion<Scalar> q(qw, qx, qy, qz);
        q.normalize();

        Eigen::Transform<Scalar, 3, Type> T(q);

        T.translation()(0) = tx;
        T.translation()(1) = ty;
        T.translation()(2) = tz;

        return T;
    }

    // Inverse
    inline pose_t inverse() {
        return Eigen::Isometry3d(*this).inverse();
    }

    inline void q_normalize() {
        double norm = std::sqrt(qw*qw + qx*qx + qy*qy + qz*qz);
        qw /= norm;
        qx /= norm;
        qy /= norm;
        qz /= norm;
    }
};

template<typename T>
struct trajectory_t {
    union {
        T id;
        T timestamp;
    };
    pose_t pose;

    trajectory_t()
        : id(0), pose()
    { }

    trajectory_t(T id, const pose_t& pose)
        : id(id), pose(pose)
    { }
};

struct imu_data_t {
    timestamp_t timestamp;
    double w_x, w_y, w_z, a_x, a_y, a_z;

    imu_data_t()
        : timestamp(0), w_x(0.0), w_y(0.0), w_z(0.0), a_x(0.0), a_y(0.0), a_z(0.0)
    { }

    imu_data_t(timestamp_t timestamp, double w_x, double w_y, double w_z, double a_x, double a_y, double a_z)
        : timestamp(timestamp), w_x(w_x), w_y(w_y), w_z(w_z), a_x(a_x), a_y(a_y), a_z(a_z)
    { }
};

struct exposure_t {
    timestamp_t timestamp;
    std::string filename;
    timestamp_t exposure;

    exposure_t()
        : timestamp(0), filename(""), exposure(0)
    { }

    exposure_t(timestamp_t timestamp, const std::string& filename, timestamp_t exposure)
        :  timestamp(timestamp), filename(filename), exposure(exposure)
    { }
};

using Records = std::vector<record_t>;
using Trajectory = std::vector<trajectory_t<timestamp_t>>;
using ImuData = std::vector<imu_data_t>;

// ------------------------------------------------------------

// Comparators

template<typename T>
inline bool operator<(const trajectory_t<T>& lhs, const trajectory_t<T>& rhs) {
    return (lhs.id < rhs.id);
}

inline bool operator<(const imu_data_t& lhs, const imu_data_t& rhs) {
    return (lhs.timestamp < rhs.timestamp);
}

inline bool operator<(const exposure_t& lhs, const exposure_t& rhs) {
    return (lhs.timestamp < rhs.timestamp);
}

// ------------------------------------------------------------

// IO helpers

/*
inline timestamp_t timestamp(const ros::Time& t) {
    return static_cast<timestamp_t>(t.nsec) + static_cast<timestamp_t>(t.sec) * static_cast<timestamp_t>(1000000000ull);
}
*/

inline std::string to_string(int n, int w) {

    std::stringstream ss;
    ss << std::setw(w) << std::setfill('0') << n;
    return ss.str();
}

inline std::string to_string(double n, int precision) {

    std::stringstream ss;
    ss << std::fixed << std::setprecision(precision) << n;
    return ss.str();
}

inline std::ostream& operator<<(std::ostream& lhs, const record_t& rhs) {

    lhs << std::to_string(rhs.first) << OUTPUT_SEPARATOR << rhs.second;
    return lhs;
}

inline std::istream& operator>>(std::istream &lhs, pose_t &rhs) {

    lhs >> rhs.tx >> rhs.ty >> rhs.tz >> rhs.qw >> rhs.qx >> rhs.qy >> rhs.qz;
    rhs.q_normalize(); // to handle finite precision
    return lhs;
}

inline std::ostream& operator<<(std::ostream& lhs, const pose_t& rhs) {

    lhs << to_string(rhs.tx, OUTPUT_PRECISION) << OUTPUT_SEPARATOR << to_string(rhs.ty, OUTPUT_PRECISION) << OUTPUT_SEPARATOR << to_string(rhs.tz, OUTPUT_PRECISION) << OUTPUT_SEPARATOR
        << to_string(rhs.qw, OUTPUT_PRECISION) << OUTPUT_SEPARATOR << to_string(rhs.qx, OUTPUT_PRECISION) << OUTPUT_SEPARATOR << to_string(rhs.qy, OUTPUT_PRECISION) << OUTPUT_SEPARATOR << to_string(rhs.qz, OUTPUT_PRECISION);
    return lhs;
}

template<typename T>
inline std::istream& operator>>(std::istream& lhs, trajectory_t<T>& rhs) {

    lhs >> rhs.id >> rhs.pose;
    return lhs;
}

template<typename T>
inline std::ostream& operator<<(std::ostream& lhs, const trajectory_t<T>& rhs) {

    lhs << std::to_string(rhs.id) << OUTPUT_SEPARATOR << rhs.pose;
    return lhs;
}

inline std::istream& operator>>(std::istream& lhs, imu_data_t& rhs) {

    lhs >> rhs.timestamp >> rhs.w_x >> rhs.w_y >> rhs.w_z >> rhs.a_x >> rhs.a_y >> rhs.a_z;
    return lhs;
}

inline std::ostream& operator<<(std::ostream& lhs, const imu_data_t& rhs) {

    lhs << rhs.timestamp << OUTPUT_SEPARATOR << to_string(rhs.w_x, OUTPUT_PRECISION) << OUTPUT_SEPARATOR << to_string(rhs.w_y, OUTPUT_PRECISION) << OUTPUT_SEPARATOR << to_string(rhs.w_z, OUTPUT_PRECISION) << OUTPUT_SEPARATOR << to_string(rhs.a_x, OUTPUT_PRECISION) << OUTPUT_SEPARATOR << to_string(rhs.a_y, OUTPUT_PRECISION) << OUTPUT_SEPARATOR << to_string(rhs.a_z, OUTPUT_PRECISION);
    return lhs;
}

inline std::istream& operator>>(std::istream &lhs, exposure_t& rhs) {

    lhs >> rhs.timestamp >> rhs.filename >> rhs.exposure;
    return lhs;
}

inline std::ostream& operator<<(std::ostream& lhs, const exposure_t& rhs) {

    lhs << rhs.timestamp << OUTPUT_SEPARATOR << rhs.filename << OUTPUT_SEPARATOR << rhs.exposure;
    return lhs;
}

template<typename T>
inline std::vector<T> read_file(const std::string &path) {

    std::vector<T> records;

    std::ifstream input(path);
    if (!input.is_open()) return records;

    for (std::string line; std::getline(input, line);) {
        if (line.empty() || line.front() == '#') continue;

        if (OUTPUT_SEPARATOR.compare(" ") != 0) line = StringReplace(line, OUTPUT_SEPARATOR, " ");
        std::istringstream iss(line);
        T record;
        if (iss >> record) records.push_back(std::move(record));
    }

    std::sort(records.begin(), records.end());
    return records;
}

template<typename T>
inline void write_stream(const std::vector<T>& records, std::ostream& stream) {
    for (const T& record : records)
        stream << record << std::endl;
}

template<typename T>
inline bool write_file(const std::vector<T>& records, const std::string& path, const std::string& header = std::string()) {

    std::ofstream output(path);
    if (!output.is_open()) return false;

    if (!header.empty()) output << header << std::endl;
    write_stream(records, output);

    output.close();
    return (!output.fail() && !output.bad());
}

// ------------------------------------------------------------

} // namespace io

#endif // IO_HPP_

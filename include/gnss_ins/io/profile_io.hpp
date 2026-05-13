#pragma once
#include "gnss_ins/common.hpp"
#include <fstream>
#include <iomanip>
#include <sstream>
#include <string>
#include <vector>

namespace gnss_ins::io {

struct ProfileRow {
    double time;
    double lat;
    double lon;
    double h;
    Vec3   v_eb_n;
    Vec3   euler_nb;
};

inline std::vector<ProfileRow> readProfile(const std::string& path)
{
    std::ifstream f(path);
    if (!f.is_open())
        throw std::runtime_error("readProfile: cannot open " + path);

    std::vector<ProfileRow> out;
    std::string line;
    while (std::getline(f, line)) {
        if (line.empty() || line[0] == '#') continue;
        std::istringstream ss(line);
        ProfileRow r;
        char c;
        ss >> r.time            >> c;
        ss >> r.lat             >> c;  r.lat          *= kD2R;
        ss >> r.lon             >> c;  r.lon          *= kD2R;
        ss >> r.h               >> c;
        ss >> r.v_eb_n[0]       >> c;
        ss >> r.v_eb_n[1]       >> c;
        ss >> r.v_eb_n[2]       >> c;
        ss >> r.euler_nb[0]     >> c;  r.euler_nb[0]  *= kD2R;
        ss >> r.euler_nb[1]     >> c;  r.euler_nb[1]  *= kD2R;
        ss >> r.euler_nb[2];           r.euler_nb[2]  *= kD2R;
        out.push_back(r);
    }
    return out;
}

inline void writeProfile(const std::string& path,
                         const std::vector<ProfileRow>& rows)
{
    std::ofstream f(path);
    if (!f.is_open())
        throw std::runtime_error("writeProfile: cannot open " + path);
    f << std::fixed << std::setprecision(9);
    for (const auto& r : rows) {
        f << r.time                    << ','
          << r.lat          * kR2D     << ','
          << r.lon          * kR2D     << ','
          << r.h                       << ','
          << r.v_eb_n[0]               << ','
          << r.v_eb_n[1]               << ','
          << r.v_eb_n[2]               << ','
          << r.euler_nb[0]  * kR2D     << ','
          << r.euler_nb[1]  * kR2D     << ','
          << r.euler_nb[2]  * kR2D     << '\n';
    }
}

}

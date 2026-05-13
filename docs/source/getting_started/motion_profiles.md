# Motion Profiles

Five CSV truth-trajectory files drive every simulation in the suite.
Each row is one navigation epoch at **10 ms (100 Hz)** spacing.

## Available profiles

| File | Scenario | Epochs | Duration | Speed / Alt |
|---|---|---|---|---|
| `Profile_0.csv` | Stationary and level | 6 000 | 60 s | 0 m/s · 10 m |
| `Profile_1.csv` | Car — two 90° turns | 6 000 | 60 s | ~10 m/s · 50 m |
| `Profile_2.csv` | Car — turns, curve and halt | 17 501 | 175 s | ~10 m/s · 50 m |
| `Profile_3.csv` | Aircraft — two 45° turns and 500 m climb | 41 801 | 418 s | ~200 m/s · 10 km |
| `Profile_4.csv` | Boat — sea state 3, two 45° turns | 30 001 | 300 s | ~10 m/s · heave visible |

All profiles begin near **51 °N, 0 °E** (southern England) to match the
MATLAB demo script configurations.

## Column format

Each row has exactly **10 comma-separated values**:

| Column | Field | Unit in file | After `readProfile()` | Notes |
|---|---|---|---|---|
| 1 | Time | seconds | seconds | Epoch timestamp |
| 2 | Latitude | **degrees** | **radians** | WGS-84 geodetic |
| 3 | Longitude | **degrees** | **radians** | WGS-84 geodetic |
| 4 | Height | metres | metres | Above ellipsoid |
| 5 | v\_North | m/s | m/s | NED frame |
| 6 | v\_East | m/s | m/s | NED frame |
| 7 | v\_Down | m/s | m/s | NED frame |
| 8 | Roll φ | **degrees** | **radians** | Body Euler angle |
| 9 | Pitch θ | **degrees** | **radians** | Body Euler angle |
| 10 | Yaw ψ | **degrees** | **radians** | Body heading |

```{warning}
Columns 2, 3, 8, 9, 10 are stored in **degrees** in the CSV but must be
converted to **radians** on load. The original MATLAB ``Read_profile.m``
does this conversion; your C++ ``io::readProfile()`` must replicate it
exactly or every downstream calculation will produce wrong results.
```

## C++ reader skeleton

```cpp
// include/gnss_ins/io/profile_io.hpp
#pragma once
#include "gnss_ins/common.hpp"
#include <fstream>
#include <sstream>
#include <vector>

namespace gnss_ins::io {

struct ProfileRow {
    double            time;      ///< epoch time (s)
    double            lat;       ///< geodetic latitude (rad)
    double            lon;       ///< longitude (rad)
    double            h;         ///< height above ellipsoid (m)
    Vec3              v_eb_n;    ///< NED velocity (m/s)
    Vec3              euler_nb;  ///< roll/pitch/yaw (rad)
};

/// @brief Load a Profile_N.csv file.
/// Converts columns 2,3,8,9,10 from degrees to radians on load.
/// @see Read_profile.m (original MATLAB)
inline std::vector<ProfileRow> readProfile(const std::string& path)
{
    std::ifstream f(path);
    std::string   line;
    std::vector<ProfileRow> out;
    while (std::getline(f, line)) {
        if (line.empty() || line[0] == '#') continue;
        std::istringstream ss(line);
        ProfileRow r;
        char comma;
        ss >> r.time   >> comma;
        ss >> r.lat    >> comma;  r.lat          *= kD2R;   // col 2
        ss >> r.lon    >> comma;  r.lon          *= kD2R;   // col 3
        ss >> r.h      >> comma;                            // col 4
        ss >> r.v_eb_n[0] >> comma;                         // col 5
        ss >> r.v_eb_n[1] >> comma;                         // col 6
        ss >> r.v_eb_n[2] >> comma;                         // col 7
        ss >> r.euler_nb[0] >> comma; r.euler_nb[0] *= kD2R; // col 8
        ss >> r.euler_nb[1] >> comma; r.euler_nb[1] *= kD2R; // col 9
        ss >> r.euler_nb[2];          r.euler_nb[2] *= kD2R; // col 10
        out.push_back(r);
    }
    return out;
}

} // namespace gnss_ins::io
```

## Validation check

After implementing the reader, verify with Profile_0.csv:

```text
row[0].time  == 0.0
row[0].lat   ≈  0.890118 rad   (= 51.0 × π/180)
row[0].lon   == 0.0
row[0].h     == 10.0 m
row[0].v_eb_n.norm() == 0.0    (stationary)
total rows   == 6 000
```

```{tip}
The stationary Profile_0 is the easiest to validate because all velocities
and attitude rates are zero, making the expected output trivial to compute
by hand.
```

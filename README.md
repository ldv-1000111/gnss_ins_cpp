# Groves GNSS / INS Navigation — C++ Migration

[![version](https://img.shields.io/badge/version-0.1.0--step1-blue)](https://github.com/ldv-1000111/gnss_ins_cpp)
[![license](https://img.shields.io/badge/license-BSD-green)](./LICENSE)
[![standard](https://img.shields.io/badge/standard-C%2B%2B17-orange)](https://en.cppreference.com/w/cpp/17)

C++17 port of Paul D. Groves' navigation simulation suite from *Principles of GNSS, Inertial, and Multisensor Integrated Navigation Systems* (2nd ed., 2013).

## Project Attribution & Authority

This repository is a **C++17 migration** of the navigation simulation suite originally developed in MATLAB by **Paul D. Groves**.

* **Lead Developer/Maintainer:** ldv-1000111 (2026)
* **Original Algorithms & Data:** Based on *Principles of GNSS, Inertial, and Multisensor Integrated Navigation Systems* (2nd Ed., 2013)
* **Technical Stewardship:** All C++ implementation, GoogleTest suites, and Docker-based CI/CD scaffolding are original works developed by ldv-1000111

### Legal Status
The math and logic belong to the original author, while this specific C++ "expression" and implementation are the creation of the current maintainer. This project is a technical transformation and does not imply official endorsement by Paul Groves

**Status:** Step 1 Complete — Project scaffolding (CMake, constants, CSV I/O).

## Quick Start

### Prerequisites

- Docker (with Docker Compose v2)
- Git

### Build and test

```bash
git clone git@github.com:ldv-1000111/gnss_ins_cpp.git
cd gnss_ins_cpp
docker compose --profile build build build-gcc
docker compose --profile dev run --rm shell-gcc
```

Inside the shell:

```bash
cmake -S /workspace -B /build/gcc -G Ninja
cmake --build /build/gcc --parallel 16
ctest --test-dir /build/gcc -V
```

### Run with Clang 19

```bash
docker compose --profile build build build-clang
docker compose --profile dev run --rm shell-clang
cmake -S /workspace -B /build/clang -G Ninja
cmake --build /build/clang --parallel 16
ctest --test-dir /build/clang -V
```

### Full CI pipeline (both compilers)

```bash
docker compose --profile ci up --abort-on-container-exit
```

```text
Project structure

include/gnss_ins/
├── common.hpp
└── io/profile_io.hpp

tests/
├── test_io.cpp
└── CMakeLists.txt

docker/
├── Dockerfile.gcc
├── Dockerfile.clang
├── Dockerfile.docs
└── toolchain-clang.cmake

data/
├── Profile_0.csv
├── Profile_1.csv
├── Profile_2.csv
├── Profile_3.csv
└── Profile_4.csv

CMakeLists.txt
docker-compose.yml
```

## Documentation

```bash
docker compose --profile docs up docs
# browse http://localhost:8080
```

## Development workflow

```bash
docker compose --profile dev run --rm shell-gcc
cmake -S /workspace -B /build/gcc -G Ninja
cmake --build /build/gcc --parallel 16
ctest --test-dir /build/gcc -V
```

## Compiler versions

- GCC 14.2 (Debian Trixie)
- Clang 19 (Debian Trixie)
- CMake 3.28
- Eigen3 3.4
- GoogleTest 1.14.0

## License

BSD 3-Clause License

## Next steps

Step 2: Earth Model & Math Utilities (7 functions)

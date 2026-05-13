# Docker Development Workflow

Day-to-day commands for building, testing, and iterating on the GNSS/INS
C++ code inside the Docker containers on the HPE server.

---

## Prerequisites

- Project copied to `/opt/gnss_ins_cpp/`
- Docker images built: `docker compose build` completed
- CSV motion profiles in `data/Profile_0.csv` … `data/Profile_4.csv`
- Running as user `lvs` (already in docker group, no sudo needed)

---

## Interactive development shell

Start a shell with full read-write access to the source tree and
persistent build volumes:

```bash
cd /opt/gnss_ins_cpp

# GCC 14 + 64 cores
docker compose --profile dev run --rm shell-gcc

# Or Clang 19 + clang-tidy + lldb
docker compose --profile dev run --rm shell-clang
```

Inside the shell, the environment is set up:
- `/workspace` = read-write mount of the project
- `/build/gcc` or `/build/clang` = persistent build directory
- `/ccache` = persistent compiler cache (2 GB cap per compiler)
- CMake, Ninja, Eigen3, GoogleTest all installed

---

## Building from the shell

### First build (cold)

```bash
# Inside shell-gcc or shell-clang
cmake -S /workspace -B /build/gcc -DCMAKE_BUILD_TYPE=RelWithDebInfo -G Ninja
cmake --build /build/gcc --parallel 16
```

Expected output: CMake configure, then Ninja compilation with `[100%]` at
the end. No errors.

### Incremental build (after editing headers)

```bash
# Inside shell
cmake --build /build/gcc --parallel 16
```

Ninja only recompiles changed files. Typical time after editing one
header: **15–45 seconds**.

### Single target

```bash
# Build only the test executable
cmake --build /build/gcc --target test_io

# Or specific test with GoogleTest
ctest --test-dir /build/gcc -R "ProfileIO" -V
```

---

## Running tests

### All tests for the current build

```bash
# Inside shell
ctest --test-dir /build/gcc --output-on-failure --parallel 4
```

### One test by name (verbose)

```bash
ctest --test-dir /build/gcc -R "RowCount" -V
```

### With memory checking (Valgrind)

```bash
valgrind --leak-check=full /build/gcc/test_io
```

---

## One-shot CI pipeline (no shell)

Build and test with both compilers in one command. No interactive shell,
containers exit after completion:

```bash
cd /opt/gnss_ins_cpp
docker compose --profile ci up --abort-on-container-exit
echo "Exit code: $?"
```

Exit code `0` = all tests passed. This runs:
1. GCC configure + build
2. Clang configure + build
3. GCC ctest
4. Clang ctest

All in parallel where possible. Total time on first run: **4–6 minutes**.
Incremental: **30–60 seconds**.

---

## Static analysis (Clang only)

Inside `shell-clang`:

```bash
# Check one header file
clang-tidy include/gnss_ins/common.hpp \
           -p /build/clang/compile_commands.json

# Check all headers
clang-tidy include/gnss_ins/*.hpp \
           -p /build/clang/compile_commands.json 2>&1 | head -50
```

### Code formatting (Clang only)

```bash
# Dry run — show what would change
clang-format --dry-run include/gnss_ins/common.hpp

# Apply formatting in-place
clang-format -i include/gnss_ins/common.hpp
```

---

## Debugging (GCC shell only)

Inside `shell-gcc`:

```bash
# Compile with debug symbols
cmake -S /workspace -B /build/gcc \
      -DCMAKE_BUILD_TYPE=Debug -G Ninja
cmake --build /build/gcc

# Run test under GDB
gdb --args /build/gcc/test_io
```

Then inside GDB:
```
(gdb) run
(gdb) bt          # backtrace on crash
(gdb) quit
```

---

## Compiler caches and volumes

### Check ccache status

Inside any shell:

```bash
ccache --show-stats
# Shows hit rate, size, max size (2 GB)
```

To clear it:

```bash
ccache --clear
```

### Persistent build volumes on host

The four named volumes hold build artefacts:

```bash
# From the host (not inside shell)
docker volume ls | grep gnss_ins
docker volume inspect gnss_ins_build-gcc-vol
```

To wipe and rebuild from scratch:

```bash
docker compose down -v
docker compose build
docker compose --profile dev run --rm shell-gcc
```

---

## Common workflows

### Add a new header file to the project

```bash
# 1. Inside shell, edit the file
nano /workspace/include/gnss_ins/earth/radii.hpp

# 2. Rebuild
cmake --build /build/gcc --parallel 16

# 3. If it's used by tests, run them
ctest --test-dir /build/gcc -R "Radii" -V
```

### Sync between compilers

After editing, test with both:

```bash
# GCC
exit  # leave shell-gcc
docker compose --profile dev run --rm shell-gcc \
    bash -c "cmake --build /build/gcc && ctest --test-dir /build/gcc"

# Clang
docker compose --profile dev run --rm shell-clang \
    bash -c "cmake --build /build/clang && ctest --test-dir /build/clang"
```

### Follow the tutorial steps

Each tutorial step (1–7) builds on the previous. After completing a step:

```bash
# Commit the changes (if using git)
git add -A && git commit -m "Step X: [description]"

# Test on the HPE server
docker compose --profile ci up --abort-on-container-exit

# If passing, move to Step X+1
```

---

## Exiting the shell

```bash
# Inside shell
exit
# or Ctrl-D
```

The build volumes persist. Next time you enter `shell-gcc`, the CMake
cache and ccache are still there — incremental rebuild is instant.

---

## Troubleshooting inside the shell

### CMake cannot find Eigen3

```bash
cmake --version
dpkg -s libeigen3-dev | grep Version
```

If libeigen3-dev is missing, the `Dockerfile.gcc` or `Dockerfile.clang`
apt-get step failed. Rebuild the image:

```bash
# From host, not inside shell
exit
docker compose build --no-cache gcc  # or clang
docker compose --profile dev run --rm shell-gcc
```

### ``clang-tidy: error: unable to handle compilation flag``

Clang-tidy is strict about flags. If you added a `-Wno-something` flag to
CMakeLists.txt that Clang doesn't recognize, clang-tidy will complain.
Work around it with:

```bash
clang-tidy file.hpp -- -xc++ -I/usr/include/eigen3
```

### Out of disk space on ``/data/docker``

Inside the shell, build artefacts accumulate. From the host:

```bash
df -h /data/docker
docker system df -v
docker image prune -f
```

This cleans up dangling images but not the named volumes (which are
project data). If the volumes themselves are huge, something went wrong
in the build — contact the team.

---

## Exiting and re-entering between steps

The workflow for each tutorial step is:

```bash
# Step N: Start shell
docker compose --profile dev run --rm shell-gcc

# Inside: edit, build, test
cmake --build /build/gcc --parallel 16
ctest --test-dir /build/gcc -V

# Inside: gate passes? Commit or note the step
exit

# Between steps: optionally clean
docker compose down -v  # Only if starting fresh

# Step N+1: Re-enter same shell, volumes persist
docker compose --profile dev run --rm shell-gcc
cmake --build /build/gcc
```

No rebuilding the Docker images, no recreating the volumes — just edit,
build, test, commit, next step.

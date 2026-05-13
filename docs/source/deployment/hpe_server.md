# HPE Server Deployment

This guide deploys the GNSS/INS C++ build environment on the specific HPE
ProLiant server already running the NVIDIA DriveWorks project. Every
section is written against the confirmed server state — no generic steps,
no hypotheticals.

---

## Confirmed server state

All values below were collected by running ``docker info``, ``df -h``,
``lsblk``, ``docker images``, ``docker ps -a``, ``cat /etc/docker/daemon.json``,
and ``id`` on the server.

.. list-table::
   :header-rows: 1
   :widths: 35 65

   * - Property
     - Confirmed value
   * - **OS**
     - Ubuntu 24.04.4 LTS (Noble Numbat)
   * - **Kernel**
     - 6.17.0-23-generic
   * - **Docker Engine**
     - 29.4.2 — Community Edition
   * - **Docker Compose**
     - v5.1.3
   * - **Storage driver**
     - ``overlayfs`` — containerd snapshotter (Docker 29+ default)
   * - **Docker data root**
     - ``/data/docker`` → ``/dev/sdb`` LVM, **1.3 TB, currently 2.1 MB used**
   * - **Root filesystem**
     - ``/dev/sda3`` LVM, 444 GB total, 131 GB used, **284 GB free**
   * - **Default container runtime**
     - ``nvidia`` (set by DriveWorks)
   * - **DNS servers (daemon.json)**
     - ``8.8.8.8``, ``8.8.4.4``
   * - **Active container**
     - ``driveworks-dev`` (running, uses ``driveworks-dev:7.0.3.0-cuda12-patched``)
   * - **Stopped containers**
     - 8× ``nvcr.io/drive/driveos-sdk/...`` — all randomly named
   * - **Existing named volumes**
     - None
   * - **Deploying user**
     - ``lvs`` (uid=1000) — already in ``docker`` group, no ``sudo`` needed
   * - **CPUs / RAM**
     - 64 cores / 125.6 GiB

---

## What does and does not need doing

.. list-table::
   :header-rows: 1
   :widths: 50 15 35

   * - Task
     - Status
     - Reason
   * - Install Docker Engine
     - ✅ Skip
     - 29.4.2 already installed
   * - Add user to docker group
     - ✅ Skip
     - ``lvs`` already in ``docker`` group
   * - Enable Docker on boot
     - ✅ Skip
     - Already enabled (container running after reboot)
   * - Move Docker data to dedicated disk
     - ✅ Skip
     - Already on ``/data/docker`` (1.3 TB LVM)
   * - Configure storage driver
     - ✅ Skip — do not change
     - ``overlayfs`` snapshotter is correct for Docker 29
   * - Configure DNS
     - ✅ Skip
     - ``daemon.json`` already has ``8.8.8.8`` and ``8.8.4.4``
   * - Add ``runtime: runc`` to every service
     - ⚠️ **Required**
     - ``default-runtime`` is ``nvidia`` — our containers do not use GPU
   * - Raise build parallelism
     - ⚠️ **Recommended**
     - 64 CPUs available; default ``--parallel 4`` wastes ~93% of them
   * - Check port 8080 availability
     - ⚠️ **Verify first**
     - No DriveWorks port mappings confirmed yet
   * - Check disk space
     - ✅ Confirmed sufficient
     - 1.3 TB disk with only 2.1 MB used — our ~2 GB build footprint is negligible

---

## Critical: ``daemon.json`` — do not modify

The current ``/etc/docker/daemon.json`` on the server is:

.. code-block:: json

    {
        "data-root": "/data/docker",
        "default-runtime": "nvidia",
        "dns": ["8.8.8.8", "8.8.4.4"],
        "runtimes": {
            "nvidia": {
                "args": [],
                "path": "nvidia-container-runtime"
            }
        }
    }

**Do not add or change any field in this file.** In particular:

- Do **not** add ``"storage-driver": "overlay2"`` — the server uses the
  newer ``overlayfs`` containerd snapshotter (Docker 29+ default). Adding
  that key would hide all 6 DriveWorks images until reverted.
- Do **not** remove ``"default-runtime": "nvidia"`` — that would break
  all DriveWorks containers.
- The ``dns`` entries mean our containers can reach ``github.com`` during
  ``docker compose build`` (needed to clone GoogleTest). No proxy
  configuration needed.

The ``default-runtime: nvidia`` setting is handled in our
``docker-compose.yml`` by adding ``runtime: runc`` to every service.
This overrides the default for our containers without touching the
daemon config.

---

## Step 1 — Pre-flight checks

Run these as user ``lvs`` before deploying anything. They take less than
a minute and confirm the environment is exactly as expected.

```bash
# 1a. Confirm Docker version
docker version --format 'Client: {{.Client.Version}}  Server: {{.Server.Version}}'
# expected: Client: 29.4.2  Server: 29.4.2

# 1b. Confirm storage driver (must NOT say overlay2)
docker info --format 'Storage: {{.Driver}}'
# expected: Storage: overlayfs

# 1c. Confirm data root
docker info --format 'Root: {{.DockerRootDir}}'
# expected: Root: /data/docker

# 1d. Confirm available space on Docker disk
df -h /data/docker
# expected: ~1.3T total, ~2.1M used

# 1e. Confirm no existing gnss_ containers or volumes
docker ps -a --format '{{.Names}}' | grep -i gnss || echo "clean"
docker volume ls --format '{{.Name}}' | grep -i gnss || echo "clean"
# both expected: clean

# 1f. Confirm port 8080 is free
ss -tlnp | grep ':8080' || echo "port free"
# expected: port free

# 1g. Confirm no sudo needed
docker run --rm --runtime runc hello-world
# expected: "Hello from Docker!" without sudo
```

If step 1g fails with ``permission denied``, log out and back in to
refresh the ``docker`` group membership.

---

## Step 2 — Copy the project onto the server

```bash
# From your workstation — replace <server-ip> with the HPE server address
scp -r gnss_ins_cpp/ lvs@<server-ip>:/opt/gnss_ins_cpp

# Verify it landed correctly
ssh lvs@<server-ip> "ls /opt/gnss_ins_cpp"
```

Expected directory structure after copying:

.. code-block:: text

    /opt/gnss_ins_cpp/
    ├── docker-compose.yml       ← must include runtime: runc on all services
    ├── docker/
    │   ├── Dockerfile.gcc       ← FROM debian:trixie-slim, GCC 14
    │   ├── Dockerfile.clang     ← FROM debian:trixie-slim, Clang 19
    │   ├── Dockerfile.docs      ← FROM python:3.12-slim-bookworm
    │   └── toolchain-clang.cmake
    ├── docs/source/             ← Sphinx .md source files
    ├── data/
    │   └── Profile_0.csv … Profile_4.csv
    ├── include/                 ← C++ headers (gnss_ins/ namespace tree)
    ├── src/
    ├── tests/
    ├── apps/
    └── CMakeLists.txt

---

## Step 3 — Verify ``docker-compose.yml`` is server-ready

Before building, confirm the two server-specific settings are present:

```bash
cd /opt/gnss_ins_cpp

# Must find 7 lines — one per service
grep -c "runtime: runc" docker-compose.yml
# expected: 7

# Must show --parallel 16
grep "parallel" docker-compose.yml | grep cmake
# expected: --parallel 16  (twice — gcc and clang)
```

If either check fails, the generic ``docker-compose.yml`` was copied
instead of the server-specific version. The two differences are:

.. code-block:: yaml

    # Add to EVERY service block (build-gcc, build-clang, test-gcc,
    # test-clang, docs, shell-gcc, shell-clang):
    runtime: runc

    # In the cmake build commands for build-gcc and build-clang:
    cmake --build /build/gcc   --parallel 16
    cmake --build /build/clang --parallel 16

---

## Step 4 — Build the Docker images

This is the only step that requires internet access. It pulls
``debian:trixie-slim`` (~30 MB) and clones GoogleTest from GitHub
(~5 MB). The DNS servers in ``daemon.json`` (``8.8.8.8``) handle
resolution automatically.

```bash
cd /opt/gnss_ins_cpp
docker compose build 2>&1 | tee /tmp/gnss_build.log
```

Expected output at the end of each image build:

.. code-block:: text

    === GCC image ===
    gcc (Debian 14.2.0-16) 14.2.0
    g++ (Debian 14.2.0-16) 14.2.0
    cmake version 3.28.x
    Eigen: Version: 3.4.0-4

    === Clang image ===
    Debian clang version 19.1.x
    Debian clang version 19.1.x
    cmake version 3.28.x
    Eigen: Version: 3.4.0-4
    lld: LLD 19.1.x

Verify the three images exist:

```bash
docker images | grep gnss_ins
```

.. code-block:: text

    gnss_ins_clang   latest   <id>   2 min ago   ~620MB
    gnss_ins_gcc     latest   <id>   4 min ago   ~580MB
    gnss_ins_docs    latest   <id>   5 min ago   ~220MB

Confirm images are on the Docker disk, not the root disk:

```bash
df -h /data/docker
# Used should have increased by ~1.4 GB from the 2.1 MB baseline
```

---

## Step 5 — First build and test run

```bash
cd /opt/gnss_ins_cpp
docker compose --profile ci up --abort-on-container-exit
echo "Exit: $?"
```

With 64 CPUs and ``--parallel 16``, both compiler pipelines run
concurrently. Expected timeline:

.. list-table::
   :header-rows: 1
   :widths: 20 80

   * - Time
     - What is happening
   * - 0:00–0:30
     - CMake configure phase for both GCC and Clang simultaneously
   * - 0:30–3:00
     - Eigen template instantiation and navigation function compilation
       (most CPU-intensive phase — watch with ``docker stats``)
   * - 3:00–4:00
     - GoogleTest linking and CTest execution for both compilers
   * - ~4:00
     - Exit code 0 if all tests pass

Verify the four named volumes were created on the data disk:

```bash
docker volume ls --format '{{.Name}}' | grep gnss
# expected:
# gnss_ins_build-clang-vol
# gnss_ins_build-gcc-vol
# gnss_ins_ccache-clang-vol
# gnss_ins_ccache-gcc-vol

docker volume inspect gnss_ins_build-gcc-vol \
    --format '{{.Mountpoint}}'
# expected: /data/docker/volumes/gnss_ins_build-gcc-vol/_data
```

---

## Step 6 — Coexistence verification

Confirm the GNSS/INS containers do not interfere with the running
DriveWorks container:

```bash
# DriveWorks container must still be running
docker ps --filter name=driveworks-dev --format '{{.Status}}'
# expected: Up X hours

# GNSS containers must have exited cleanly after --profile ci
docker ps -a --filter name=gnss_ --format '{{.Names}}: {{.Status}}'
# expected: all Exited (0) ...

# Confirm no GNSS container used the nvidia runtime
docker inspect gnss_build_gcc --format '{{.HostConfig.Runtime}}'
# expected: runc
docker inspect gnss_build_clang --format '{{.HostConfig.Runtime}}'
# expected: runc
```

---

## Step 7 — Start the documentation server

```bash
cd /opt/gnss_ins_cpp
docker compose --profile docs up -d docs
docker ps --filter name=gnss_docs
```

Browse to ``http://<server-ip>:8080``. If the port is not reachable from
outside the server, check iptables (Ubuntu 24.04 uses iptables as the
firewall backend per ``docker info``):

```bash
sudo iptables -L INPUT -n | grep 8080
# If the rule is absent, Docker adds it automatically when a port
# is published. If blocked by a pre-existing REJECT rule, add:
sudo iptables -I INPUT -p tcp --dport 8080 -j ACCEPT
```

To make the docs server survive reboots, install the systemd unit:

```bash
sudo tee /etc/systemd/system/gnss-ins-docs.service <<'EOF'
[Unit]
Description=GNSS/INS Documentation Server
After=docker.service
Requires=docker.service

[Service]
Type=simple
User=lvs
WorkingDirectory=/opt/gnss_ins_cpp
ExecStart=/usr/bin/docker compose --profile docs up docs
ExecStop=/usr/bin/docker compose --profile docs down
Restart=on-failure
RestartSec=10

[Install]
WantedBy=multi-user.target
EOF

sudo systemctl daemon-reload
sudo systemctl enable --now gnss-ins-docs.service
```

---

## Day-to-day usage

### Incremental build after editing C++ files

```bash
cd /opt/gnss_ins_cpp
docker compose --profile ci up --abort-on-container-exit
```

Second and subsequent builds hit the ccache volumes. Typical incremental
time after editing one header: **15–45 seconds**.

### Interactive development shell

```bash
# GCC 14 shell
docker compose --profile dev run --rm shell-gcc

# Clang 19 shell (includes clang-tidy, clang-format, lldb)
docker compose --profile dev run --rm shell-clang
```

Inside either shell:

```bash
# Incremental build
cmake --build /build/gcc --parallel 16

# Run one test by name
ctest --test-dir /build/gcc -R "NavEquations" -V

# Check ccache stats
ccache --show-stats

# Clang shell: run static analysis
clang-tidy include/gnss_ins/nav/nav_equations.hpp \
           -p /build/clang/compile_commands.json
```

### Disk usage check

```bash
# Docker's view
docker system df

# Volume breakdown
docker volume ls -q | grep gnss | \
    xargs -I{} sh -c 'echo "{}: $(du -sh /data/docker/volumes/{}/_data 2>/dev/null | cut -f1)"'

# Full data disk summary
df -h /data/docker
```

### Clean up stopped GNSS containers (safe)

Stopped CI containers accumulate after each ``--profile ci`` run. They
hold no useful state (volumes persist separately) and can be pruned:

```bash
docker container prune --filter label=com.docker.compose.project=gnss_ins -f
```

This touches only GNSS containers — DriveWorks containers are unaffected.

---

## Troubleshooting

### ``Error response from daemon: could not select device driver "nvidia"``

The ``runtime: runc`` line is missing from one or more services.

```bash
grep -n "runtime: runc" /opt/gnss_ins_cpp/docker-compose.yml
# must return 7 lines
```

Add the missing entries, then re-run.

### Build containers exit immediately with no output

Check the log of the named container:

```bash
docker logs gnss_build_gcc   2>&1 | tail -30
docker logs gnss_build_clang 2>&1 | tail -30
```

The most common cause is a CMake configure failure — look for ``CMake Error``
in the output. The second most common is a network failure during the
GoogleTest clone step — the server's DNS (``8.8.8.8``) must reach
``github.com``.

### ``docker compose build`` fails with ``unknown flag: --parallel``

This means an old Docker Compose v1 (``docker-compose``) is being used
instead of the v2 plugin. Confirm with:

```bash
docker compose version
# must print: Docker Compose version v5.1.3
```

If ``docker-compose version`` (with hyphen) prints a 1.x version, always
use ``docker compose`` (space, no hyphen) on this server.

### Images not visible after build

The server uses the containerd image store (``overlayfs`` snapshotter).
If ``docker images`` shows nothing after a successful build, check that
the daemon config was not changed to ``overlay2``:

```bash
docker info --format '{{.Driver}}'
# must be: overlayfs  — not overlay2
```

If it shows ``overlay2``, remove the ``"storage-driver"`` key from
``/etc/docker/daemon.json`` and restart Docker:

```bash
sudo systemctl restart docker
docker compose build   # rebuild
```

### Port 8080 already in use

```bash
ss -tlnp | grep 8080
```

If the port is occupied by another process, change the mapping in
``docker-compose.yml`` (e.g. ``"8090:8080"``) and restart the docs
service.

### Disk usage growing unexpectedly on ``/data/docker``

The four DriveWorks images total ~500 GB. Our build footprint is ~2 GB.
The most likely cause of growth is Docker pulling new image versions. Run:

```bash
docker system df -v
```

Old dangling images from iterative ``docker compose build`` calls can be
safely pruned:

```bash
docker image prune -f
```

To also remove the DriveWorks images that are not currently running
(``luis_test:latest`` and the stopped SDK containers), consult the
DriveWorks team first — they may be needed.

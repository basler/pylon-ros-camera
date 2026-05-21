# Docker – pylon_ros2_camera

Ready-to-use Docker images for the **pylon_ros2_camera** driver.  
Supported ROS 2 distributions: **Humble** (Ubuntu 22.04), **Jazzy** (Ubuntu 24.04), **Kilted** (Ubuntu 24.04).

---

## Prerequisites

### 1. Docker Engine (Linux only)

> **Important:** Use **Docker Engine**, not Docker Desktop.  
> Docker Desktop on Linux runs an internal VM, so `--network=host` does not expose
> the physical NIC.  GigE cameras cannot be discovered without direct NIC access.

Install Docker Engine and the Compose plugin:
```bash
sudo apt update && sudo apt install docker.io docker-compose-v2
sudo usermod -aG docker $USER   # allow running docker without sudo
newgrp docker                   # apply group change in the current shell
```

> **Note:** The Compose plugin package name depends on the Docker installation method.
> With `docker.io` (Ubuntu default repos) the package is `docker-compose-v2`.
> With the official Docker repo it is `docker-compose-plugin`.
> Both install the same `docker compose` subcommand.

### 2. pylon Camera Software Suite

The pylon SDK is not bundled in this repository.  Download the **Linux Debian
Installer** for your architecture (amd64 or arm64) from the Basler website:

> https://www2.baslerweb.com/en/downloads/software-downloads/  
> → *pylon Camera Software Suite – Linux – Debian Installer*

Copy the downloaded `.deb` file into this `docker/` directory and rename it
`pylon.deb`, **or** update `PYLON_DEB` in `docker-compose.yaml` to match your
filename.

---

## Build

All three distributions share the same `Dockerfile`.  The ROS 2 distro and the
git branch to clone are controlled by build arguments.

### Using docker compose (recommended)

The distro is selected via a `.env` file in the `docker/` directory.  
Create it once and all subsequent `docker compose` commands use it automatically.

```bash
cd docker/

# Kilted (default — .env file not required):
docker compose build

# Jazzy — create a .env file first:
echo -e "PYLON_ROS_DISTRO=jazzy\nPYLON_REPO_BRANCH=jazzy" > .env
docker compose build

# Humble — create a .env file first:
echo -e "PYLON_ROS_DISTRO=humble\nPYLON_REPO_BRANCH=humble" > .env
docker compose build
```

To go back to the default (kilted), delete the `.env` file:
```bash
rm docker/.env
```

> **Note:** `PYLON_ROS_DISTRO` is used instead of `ROS_DISTRO` in the `.env`
> file and when overriding on the command line.  This avoids a conflict with
> the host's `ROS_DISTRO` environment variable, which is set automatically
> when you source a ROS 2 installation (e.g. `. /opt/ros/jazzy/setup.bash`).
> If you used `ROS_DISTRO` in a previous `.env` file, rename the key to
> `PYLON_ROS_DISTRO`.

### Using docker build directly

```bash
cd docker/

docker build \
    --build-arg PYLON_DEB=pylon.deb \
    -t pylon_ros2_camera:kilted .

docker build \
    --build-arg ROS_DISTRO=jazzy \
    --build-arg REPO_BRANCH=jazzy \
    --build-arg PYLON_DEB=pylon.deb \
    -t pylon_ros2_camera:jazzy .

docker build \
    --build-arg ROS_DISTRO=humble \
    --build-arg REPO_BRANCH=humble \
    --build-arg PYLON_DEB=pylon.deb \
    -t pylon_ros2_camera:humble .
```

The build clones the driver from GitHub and compiles it with `colcon`.  
Expect **15–20 minutes** on the first build; subsequent builds use the cache.

---

## Run

### Using docker compose (recommended)

```bash
cd docker/
docker compose up          # foreground, shows driver logs
docker compose up -d       # detached (background)
docker compose down        # stop and remove the container
```

### Using docker run directly

```bash
docker run --rm \               # remove the container automatically when it stops
    --network=host \            # share the host network stack (required for GigE camera discovery)
    --cap-add=NET_RAW \         # raw socket access for the pylon GigE transport layer
    --cap-add=NET_ADMIN \       # network admin operations (e.g. adjusting MTU from inside the container)
    -e ROS_DOMAIN_ID=0 \        # ROS 2 domain ID — must match any external ROS 2 nodes you want to talk to
    pylon_ros2_camera:kilted    # image tag — replace with :jazzy or :humble as needed
```

### Why `--network=host` is mandatory for GigE cameras

GigE Vision uses **UDP broadcast** to discover cameras on the local network.
Docker's default bridge network isolates the container from the physical NIC, so
broadcast packets never reach the camera.  With `--network=host` the container
shares the host network stack directly and camera discovery works as on the host.

> Without `--network=host` GigE cameras will **never** be found by the driver.

### Useful launch arguments

Pass extra arguments after the image name:

```bash
# Connect to a specific camera by its DeviceUserID
docker run --rm --network=host pylon_ros2_camera:kilted \
    ros2 launch pylon_ros2_camera_wrapper pylon_ros2_camera.launch.py \
        device_user_id:=my_camera

# Enable jumbo frames (requires host NIC and switch to support 9000-byte MTU)
docker run --rm --network=host pylon_ros2_camera:kilted \
    ros2 launch pylon_ros2_camera_wrapper pylon_ros2_camera.launch.py \
        mtu_size:=9000

# Load factory defaults on startup
docker run --rm --network=host pylon_ros2_camera:kilted \
    ros2 launch pylon_ros2_camera_wrapper pylon_ros2_camera.launch.py \
        startup_user_set:=Default
```

> Replace `:kilted` with `:jazzy` or `:humble` to match the image you built.

### Custom configuration file

Mount your own `default.yaml` to override camera parameters without rebuilding:

```bash
docker run --rm --network=host \
    -v /path/to/my_camera.yaml:/ros2_ws/install/pylon_ros2_camera_wrapper/share/pylon_ros2_camera_wrapper/config/default.yaml:ro \
    pylon_ros2_camera:kilted
```

Or uncomment the `volumes` section in `docker-compose.yaml`.

> Replace `:kilted` with `:jazzy` or `:humble` to match the image you built.

### Opening a shell in the running container

With docker compose (from the `docker/` directory):
```bash
docker compose exec pylon_ros2_camera bash
```

With plain docker (from any directory):
```bash
sudo docker exec -it $(sudo docker ps -qf "ancestor=pylon_ros2_camera:kilted") bash
```

Once inside, source the ROS environment before running any `ros2` commands:
```bash
source /opt/ros/${ROS_DISTRO}/setup.bash
source /ros2_ws/install/setup.bash
```

> **Note:** `sudo` is required unless your user is in the `docker` group.  
> To add yourself permanently: `sudo usermod -aG docker $USER` then `newgrp docker`.

---

## Test

> All `ros2` commands below can be run either on the **host** (with a local ROS 2
> installation sourced: `source /opt/ros/<distro>/setup.bash`) or **inside the
> container** (open a shell with `docker compose exec pylon_ros2_camera bash`,
> then source the environment as described in the [Opening a shell](#opening-a-shell-in-the-running-container)
> section above, or prefix each command with `/entrypoint.sh`).

### 1. Verify the driver started and detected the camera

When the driver starts successfully you should see lines like:
```
[pylon_ros2_camera_wrapper] Connected to camera: <serial_number>
[pylon_ros2_camera_wrapper] Starting image acquisition ...
```

### 2. List active ROS 2 topics

Run this on the **host** (with your local ROS 2 sourced) or inside the container:

```bash
# On the host
source /opt/ros/<distro>/setup.bash
ros2 topic list

# From the host — run a single command inside the container via the entrypoint
# (the entrypoint sources the ROS environment automatically):
docker compose exec pylon_ros2_camera /entrypoint.sh ros2 topic list

# Or open a shell inside the container and source the environment manually:
docker compose exec pylon_ros2_camera bash
# (now inside the container)
source /opt/ros/${ROS_DISTRO}/setup.bash
source /ros2_ws/install/setup.bash
ros2 topic list
```

Expected output includes:
```
/my_camera/pylon_ros2_camera_node/image_raw
/my_camera/pylon_ros2_camera_node/camera_info
/my_camera/pylon_ros2_camera_node/status
```

### 3. Check the image stream frame rate

```bash
ros2 topic hz /my_camera/pylon_ros2_camera_node/image_raw
```

### 4. Inspect one image (no GUI required)

```bash
ros2 topic echo --once /my_camera/pylon_ros2_camera_node/image_raw \
    | grep -E "height|width|encoding"
```

### 5. Visualise images (GUI)

```bash
ros2 run rqt_image_view rqt_image_view
```

Select `/my_camera/pylon_ros2_camera_node/image_raw` in the topic dropdown.

---

## USB3 Vision cameras

For USB cameras, `--network=host` is **not** needed.  Map the USB bus instead.
Uncomment the `pylon_ros2_camera_usb` service in `docker-compose.yaml` (and
comment out the GigE service).

```bash
docker run --rm \
    --device=/dev/bus/usb:/dev/bus/usb \
    --group-add plugdev \
    -e ROS_DOMAIN_ID=0 \
    pylon_ros2_camera:kilted
```

> Replace `:kilted` with `:jazzy` or `:humble` to match the image you built.

---

## Troubleshooting

| Symptom | Likely cause | Fix |
|---|---|---|
| Driver starts but no camera found | Missing `--network=host` | Add `--network=host` to the run command / ensure `network_mode: host` in compose |
| Camera found on host but not in container | Docker Desktop instead of Docker Engine | [Install Docker Engine](https://docs.docker.com/engine/install/ubuntu/) and uninstall Docker Desktop |
| `ros2 topic list` shows nothing | `ROS_DOMAIN_ID` mismatch | Set the same `ROS_DOMAIN_ID` on host and container |
| Low frame rate | Standard MTU (1500 bytes) | Enable jumbo frames: `mtu_size:=9000` (requires NIC/switch support) |
| `pylon.deb: not found` build error | `.deb` not placed in `docker/` or wrong filename | Place the file in `docker/` and set `PYLON_DEB` accordingly |
| Wrong ROS distro used (e.g. jazzy when kilted expected) | Host `ROS_DISTRO` env var (set by sourcing ROS) leaks into Compose | Use `PYLON_ROS_DISTRO` instead of `ROS_DISTRO` in `.env` and on the command line |
| `permission denied` connecting to Docker socket | Current user is not in the `docker` group | Run with `sudo`, or add your user permanently: `sudo usermod -aG docker $USER` then `newgrp docker` |
| `ros2: command not found` inside `exec` shell | ROS environment not sourced in `exec` sessions | Run `source /opt/ros/${ROS_DISTRO}/setup.bash && source /ros2_ws/install/setup.bash`, or prefix commands with `/entrypoint.sh` |

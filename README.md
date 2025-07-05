# 🌀 DAVIS Driver (ROS 2)

ROS 2 driver for DAVIS/DVS cameras.
Provides access to event and frame streams from DAVIS cameras using `libcaer`.

> ⚠️ **Note**: This is a work-in-progress (WIP). Some configurations may require manual setup. Improvements and automation are planned in upcoming updates.

---

## 🏗️ Build Instructions

Follow these steps to build the driver using `colcon`.

```bash
# Create a ROS 2 workspace if you haven't already
mkdir -p ~/ros2_dvs_ws/src
cd ~/ros2_dvs_ws/src

# Clone the DAVIS driver
git clone https://github.com/YOUR_USERNAME/davisdriver.git

# Go to the workspace root
cd ~/ros2_dvs_ws

# Build the workspace
colcon build
```

---

## 🧪 Run Instructions

Make sure to source ROS 2 and the workspace before launching the driver:

```bash
# Source ROS 2 and workspace
source /opt/ros/humble/setup.bash
source install/setup.sh

# Run the DAVIS driver node
ros2 run davisdriver davis_driver
```

---

## 📡 Published Topics

The following topics are currently published:

* `/davis1/events`
  Event stream from the DAVIS sensor.

* `/davis1/image_with_ts`
  Grayscale image stream with accurate timestamps.

---

## 🔄 Bag to HDF5 Conversion (WIP)

Tools for converting ROS 2 `.bag` files to `.h5` format are **in progress**.
These tools will be useful for training deep learning models and event data processing.

---

## 🧹 Preprocessing Tools (Coming Soon)

Additional preprocessing utilities for events and images will be added to:

* Denoise raw events
* Align events with frames
* Normalize images

These will be integrated as part of the driver or auxiliary scripts.

---

## 📎 Dependency

This driver is a ROS 2 port of the official `libcaer`-based DAVIS driver:
🔗 [https://gitlab.com/inivation/dv/libcaer](https://gitlab.com/inivation/dv/libcaer)

Make sure you have the underlying DAVIS SDK installed if required by your system.

---

## 🚧 Status

* ✅ Basic streaming of events and frames supported
* ⚙️ Manual configuration needed for some parameters (e.g., resolution, biases)
* 🛠 Active development — stay tuned for updates
* 🙏 Thank you for your patience and support

---

## 🤝 Contribution

Feel free to submit issues or pull requests to improve this driver.
More automation, device detection, and error handling are on the roadmap.

---

## 👤 Author

Maintained by **"Soikat Hasan Ahmed"**
License: MIT

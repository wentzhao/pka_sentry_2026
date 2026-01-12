# Small GICP Relocalization with ScanContext

这是一个基于 **small_gicp** (用于精细追踪) 和 **ScanContext** (用于全局冷启动) 的 ROS 2 激光雷达重定位功能包。

它可以解决机器人“不知道自己在哪”的问题：
1.  **原点启动模式**：假设机器人从建图原点出发。
2.  **任意点启动模式**：通过 ScanContext 匹配历史场景，自动推断机器人的初始位置。

## 1. 依赖项 (Dependencies)

在编译之前，请确保系统已安装以下库：

*   **ROS 2** (Humble/Iron/Jazzy)
*   **PCL** (`libpcl-all-dev`)
*   **OpenCV** (`libopencv-dev`)
*   **Nanoflann** (`libnanoflann-dev`)
*   **OpenMP** (`libomp-dev`)
*   **small_gicp** (需作为源码包放在工作空间中)

## 2. 编译 (Build)

```bash
cd ~/your_colcon_ws
colcon build --packages-select small_gicp_relocalization --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash

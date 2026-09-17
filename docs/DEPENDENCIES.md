# 环境与外部资源

所有示例命令从项目根目录运行。Python 脚本支持文件路径和 `python -m` 两种入口。

## Python

建议 Python 3.10 或更新版本，在独立虚拟环境安装：

```powershell
python -m venv .venv
.\.venv\Scripts\python.exe -m pip install -r requirements.txt
.\.venv\Scripts\python.exe scripts/simulation/run_mujoco.py --help
```

Linux 对应解释器为 `.venv/bin/python`。`requirements.txt` 覆盖仿真、分析、绘图；它不是历史实验的版本锁定文件。视频编码另外需要 PATH 中有 `ffmpeg`。

## C++

- CMake 3.16+，支持 C++20 的编译器和线程库：基础控制器、mock、stepper 和测试。
- Unitree SDK2：仅 Ubuntu 实机目标需要，以 `H1IF_BUILD_UNITREE=ON` 启用。
- LibTorch：仅在线 TorchScript 策略需要，以 `H1IF_BUILD_TORCH_POLICY=ON` 启用；通过 `H1IF_LIBTORCH_DIR` 或 `CMAKE_PREFIX_PATH` 指定安装位置。工程不再默认指向某台机器的下载目录。

## 模型与参考实现

- `models/mujoco/h1/`：随项目分发的 Unitree H1 模型、网格、来源说明及许可证。
- `models/policies/`：在线策略权重，部署参数见 `config/policy/h1.yaml`。
- `reference/matlab/eid_control.m`：MATLAB 参考实现，不参与 CMake 构建。

本次整理前的 `unitree_rl_gym/` 是空目录，已移除；当前 CMake 与源码没有依赖该目录。原 `.gitmodules` 仅声明不存在的 `third_party/ruckig`，当前构建和源码没有引用，已移除该失效声明。未来引入这些外部工程时，应同时补充版本与安装步骤。

> 2026-09-17 远端同步：已补回原仓库跟踪的历史产物；当前 Git 状态、历史路径和子模块引用说明见 [远端同步记录](REMOTE_SYNC.md)。

# 路径规划与组件试验

- 组件固件：`firmware/demo/DEMO_component`，Keil 工程 `mdk/rt1064.uvprojx`。
- 上位机：`E35_machine`，试验日志在 `E35_machine/logs`。
- 规划器与统一调参 JSON：`path_planner`。

仓库中的固件使用 `firmware/demo/libraries`。固件预构建脚本调用仓库根目录下的 `path_planner/prepare_component.py`。本分支已选模块 4；按需要修改 `path_planner/component_tests.json` 中的 `module` 后重新编译并烧录。上位机启动脚本中的 Python 路径以及 Keil 工具链路径需要按本机安装情况配置。

仓库未包含 `.venv`、Python 缓存、Keil Objects/、大型 Listings/ 和旧上位机虚拟环境压缩包；试验 CSV 和配套日志保留。其他文件中若出现 `E:\CARcar` 的绝对路径，应按实际检出位置调整。

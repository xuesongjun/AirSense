Please answer all questions in Chinese.
# Repository Guidelines

## 项目结构与模块组织
AirSense 基于 ESP-IDF，核心代码位于 `main/`：`main.c` 负责传感器轮询与网络逻辑，`sensors/` 内含各传感器驱动，`algorithms/` 存放气体指数算法文件，并通过同目录的 `CMakeLists.txt` 注册组件。第三方依赖由 `managed_components/esp-idf-lib__*` 管理，项目文档在 `doc/`，硬件与 PCB 资料在 `pcb/`，生成物集中在 `build/`。新增模块请紧邻同类目录，并更新组件清单与头文件曝光点。

## 构建、测试与开发命令
- `idf.py set-target esp32c5`：首次进仓库时指定芯片，后续构建沿用。
- `idf.py build`：增量编译主应用，也是提交前的基础校验。
- `idf.py -p COM3 flash monitor`：替换为实际串口后可一键烧录并监听日志（Ctrl+] 退出）。
- `idf.py menuconfig`：调整 `CONFIG_AIRSENSE_WIFI_*`、日志级别或堆栈参数，并写回 `sdkconfig.defaults`。
- `idf.py fullclean && idf.py build`：在修改 `sdkconfig`、依赖或工具链后执行以获得干净产物。

## 代码风格与命名约定
保持 C99 与 4 空格缩进，函数与变量统一 `snake_case`（如 `wifi_prepare_config`），常量/宏使用全大写（`S88LP_TXD_PIN`）并集中在文件顶部备注连线。优先使用 `static` 辅助函数，日志通过 `ESP_LOGx(TAG, ...)` 输出，TAG 与模块名保持一致（例：`AirSense`、`SCD41`）。新增源文件后同步更新 `idf_component_register`，并在对应头文件声明对外 API。

## 测试指南
当前无自动化测试，需在目标硬件上验证：运行 `idf.py build`、`idf.py -p <PORT> flash`，随后用 `idf.py monitor --print_filter "AirSense:*"` 观察传感器输出。驱动自带 `*_self_test()`（如 `sensors/scd41.c`、`sensors/sgp41.c`），修改相关模块时可临时调用并在 PR 中说明期望 ppm/Index 区间。未来若添加 Unity 测试，请放入 `main/tests/test_<module>.c` 并使用 `TEST_CASE("module_behavior", "[group]")` 命名。

## 提交与 Pull Request 规范
历史提交偏好简洁、动词开头的中文主题（如“为DPS310添加进程锁”），主题建议控制在 50 字符内，必要时在正文描述详细硬件步骤。PR 描述需包含测试设备/固件版本、实际执行的 `idf.py` 命令、关键 `monitor` 片段或网页截屏，并关联对应 Issue/需求，方便硬件复现实验。

## 安全与配置提示
禁止提交真实 SSID、口令或私钥。请使用 menuconfig 默认值配合 `main.c` 中的 `wifi_credentials_*` NVS 助手，并仅共享去除私密信息的 `sdkconfig.defaults`。若需调整引脚或供电策略，把宏写入 `hardware_config.h` 或追加至 `doc/`，同时在 PR 说明，避免在代码中散落魔术数字。

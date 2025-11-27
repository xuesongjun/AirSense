# AirSense 编译和烧录指南

## 一、环境准备

### 1.1 安装ESP-IDF

#### Windows系统

```bash
# 1. 下载ESP-IDF离线安装器
# 从Espressif官网下载: https://dl.espressif.com/dl/esp-idf/

# 2. 安装到默认路径 (推荐)
C:\Espressif\

# 3. 安装完成后，在开始菜单找到"ESP-IDF Command Prompt"
```

#### Linux/Mac系统

```bash
# 1. 安装依赖
sudo apt-get install git wget flex bison gperf python3 python3-pip python3-venv cmake ninja-build ccache libffi-dev libssl-dev dfu-util libusb-1.0-0

# 2. 克隆ESP-IDF仓库
mkdir -p ~/esp
cd ~/esp
git clone --recursive https://github.com/espressif/esp-idf.git

# 3. 安装工具链
cd ~/esp/esp-idf
./install.sh esp32c5

# 4. 设置环境变量 (每次打开新终端都需要执行)
. $HOME/esp/esp-idf/export.sh

# 可以添加到~/.bashrc或~/.zshrc自动执行
echo 'alias get_idf=". $HOME/esp/esp-idf/export.sh"' >> ~/.bashrc
```

### 1.2 验证安装

```bash
# 检查ESP-IDF版本
idf.py --version

# 应该输出类似: ESP-IDF v5.x.x
```

## 二、项目配置

### 2.1 设置目标芯片

```bash
cd /path/to/AirSense

# 设置ESP32-C5为目标
idf.py set-target esp32c5
```

### 2.2 配置项目 (可选)

```bash
# 打开配置菜单
idf.py menuconfig

# 可配置项目:
# - Component config -> ESP System settings -> CPU frequency
# - Component config -> FreeRTOS -> Task stack size
# - Serial flasher config -> Flash size
# - Serial flasher config -> Baud rate for flash/monitor
```

推荐配置：
- CPU频率: 160 MHz
- Flash大小: 4MB
- 烧录波特率: 921600

### 2.3 自定义引脚配置

如果你的硬件连接与默认配置不同，编辑 `src/main.c`:

```c
// 修改这些引脚定义
#define S8_TX_PIN               GPIO_NUM_17    // 修改为你的引脚
#define S8_RX_PIN               GPIO_NUM_18
#define DART_TX_PIN             GPIO_NUM_21
#define DART_RX_PIN             GPIO_NUM_22
#define I2C_SDA_PIN             GPIO_NUM_6
#define I2C_SCL_PIN             GPIO_NUM_7
```

## 三、编译项目

### 3.1 完整编译

```bash
cd /path/to/AirSense

# 完整编译
idf.py build
```

编译成功输出示例：
```
[100/100] Generating binary image from built executable
esptool.py v4.x
Creating esp32c5 image...
Successfully created esp32c5 image.
Generated: /path/to/AirSense/build/airsense.bin
```

### 3.2 清理编译

```bash
# 清理编译文件
idf.py fullclean

# 重新编译
idf.py build
```

### 3.3 仅编译特定组件

```bash
# 仅编译主程序
idf.py build --component main
```

## 四、烧录程序

### 4.1 查找串口

#### Windows
```bash
# 在设备管理器中查看端口 (COM3, COM4等)
# 或使用命令
mode
```

#### Linux
```bash
# 查看串口设备
ls /dev/ttyUSB*
ls /dev/ttyACM*

# 设置串口权限
sudo usermod -a -G dialout $USER
# 需要重新登录才生效
```

#### Mac
```bash
ls /dev/cu.*
```

### 4.2 烧录固件

```bash
# Windows
idf.py -p COM3 flash

# Linux/Mac
idf.py -p /dev/ttyUSB0 flash
```

烧录选项：
```bash
# 指定波特率
idf.py -p COM3 -b 921600 flash

# 擦除Flash后烧录
idf.py -p COM3 erase-flash
idf.py -p COM3 flash
```

### 4.3 烧录并监视

```bash
# 烧录后自动打开串口监视器
idf.py -p COM3 flash monitor

# 退出监视器: Ctrl + ]
```

## 五、监视和调试

### 5.1 串口监视器

```bash
# 仅打开监视器
idf.py -p COM3 monitor

# 指定波特率
idf.py -p COM3 -b 115200 monitor

# 使用颜色高亮
idf.py -p COM3 monitor --print_filter="*:I"
```

### 5.2 查看日志级别

在代码中设置日志级别：
```c
esp_log_level_set("*", ESP_LOG_INFO);       // 全局INFO级别
esp_log_level_set("AirSense", ESP_LOG_DEBUG); // 主程序DEBUG级别
esp_log_level_set("SGP41", ESP_LOG_VERBOSE); // SGP41详细日志
```

### 5.3 启用Coredump

```bash
idf.py menuconfig
# Component config -> ESP System Settings -> Core dump destination
# 选择: Flash
```

### 5.4 GDB调试 (需要JTAG)

```bash
# 启动OpenOCD
openocd -f board/esp32c5-builtin.cfg

# 在另一个终端启动GDB
xtensa-esp32c5-elf-gdb build/airsense.elf
(gdb) target remote :3333
(gdb) monitor reset halt
(gdb) continue
```

## 六、常见问题

### 6.1 编译错误

#### 问题: "CMake Error: No CMAKE_C_COMPILER"
```bash
# 重新安装工具链
cd ~/esp/esp-idf
./install.sh esp32c5
. ./export.sh
```

#### 问题: "ninja: build stopped: subcommand failed"
```bash
# 清理后重新编译
idf.py fullclean
idf.py build
```

### 6.2 烧录错误

#### 问题: "serial.serialutil.SerialException: could not open port"
```bash
# Linux: 检查权限
sudo chmod 666 /dev/ttyUSB0

# 或添加用户到dialout组
sudo usermod -a -G dialout $USER
```

#### 问题: "A fatal error occurred: Failed to connect"
```bash
# 1. 按住ESP32-C5的BOOT按钮
# 2. 按一下RST按钮
# 3. 松开BOOT按钮
# 4. 重新烧录

idf.py -p COM3 flash
```

#### 问题: "Timed out waiting for packet header"
```bash
# 降低波特率
idf.py -p COM3 -b 115200 flash
```

### 6.3 运行时错误

#### 问题: 传感器初始化失败
```
E (340) SenseAir_S8: Failed to send Modbus command
```
**解决方案**: 检查UART引脚连接和波特率

#### 问题: I2C通信失败
```
E (350) SGP41: I2C write failed: ESP_FAIL
```
**解决方案**:
1. 检查I2C上拉电阻
2. 确认I2C地址正确
3. 降低I2C频率

### 6.4 监视器问题

#### 问题: 乱码输出
```bash
# 确认波特率匹配
idf.py -p COM3 -b 115200 monitor
```

#### 问题: 无输出
```bash
# 检查USB线是否支持数据传输
# 尝试其他USB口
# 确认ESP32-C5已正常启动 (电源LED亮)
```

## 七、生产烧录

### 7.1 生成单个固件文件

```bash
# 合并所有二进制文件
esptool.py --chip esp32c5 merge_bin -o airsense_full.bin \
  --flash_mode dio --flash_size 4MB \
  0x0 build/bootloader/bootloader.bin \
  0x8000 build/partition_table/partition-table.bin \
  0x10000 build/airsense.bin
```

### 7.2 批量烧录

```bash
# 使用esptool直接烧录
esptool.py --chip esp32c5 --port COM3 --baud 921600 \
  write_flash 0x0 airsense_full.bin
```

### 7.3 OTA升级 (未来版本)

```bash
# 生成OTA固件
idf.py build
# OTA二进制文件: build/airsense.bin
```

## 八、性能优化

### 8.1 编译优化

```bash
idf.py menuconfig
# Compiler options -> Optimization Level
# 选择: Optimize for performance (-O2)
```

### 8.2 减小固件大小

```bash
# 1. 关闭调试信息
idf.py menuconfig
# Component config -> Log output -> Default log verbosity -> No output

# 2. 优化大小
# Compiler options -> Optimization Level -> Optimize for size (-Os)
```

### 8.3 提高启动速度

```bash
# 1. 减少日志输出
# 2. 关闭不需要的组件
# 3. 使用QIO flash模式
```

## 九、CI/CD集成

### 9.1 GitHub Actions示例

```yaml
name: Build AirSense

on: [push, pull_request]

jobs:
  build:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v2
      - name: ESP-IDF Build
        uses: espressif/esp-idf-ci-action@v1
        with:
          esp_idf_version: v5.0
          target: esp32c5
          path: '.'
```

### 9.2 Docker编译

```bash
# 使用官方ESP-IDF Docker镜像
docker run --rm -v $PWD:/project -w /project \
  espressif/idf:release-v5.0 \
  idf.py build
```

## 十、参考资源

- [ESP-IDF编程指南](https://docs.espressif.com/projects/esp-idf/zh_CN/latest/esp32c5/)
- [ESP32-C5技术参考手册](https://www.espressif.com/sites/default/files/documentation/esp32-c5_technical_reference_manual_cn.pdf)
- [esptool文档](https://github.com/espressif/esptool)
- [项目README](../README.md)
- [硬件连接指南](./hardware_setup.md)

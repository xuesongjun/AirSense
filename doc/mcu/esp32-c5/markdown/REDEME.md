## SCD41使用说明



esp32-c5 i2ctools配置

注意事项

1. 传感器上电后进入空闲（idle）状态。
2. 主控通过 I2C 发送 `start_periodic_measurement` 命令，启动周期性测量，测量数据每 5 秒更新一次。
3. 主控周期性地发送 `read_measurement` 命令读取传感器数据。
4. 当不需要继续测量时，主控发送 `stop_periodic_measurement` 命令让传感器回到空闲状态。

同时，处于周期性测量模式时，主控不能发送除以下命令以外的其他命令，以免干扰测量：

- `read_measurement`
- `get_data_ready_status`
- `stop_periodic_measurement`
- `set_ambient_pressure`
- `get_ambient_pressure`

```shell

# 设置使用esp32-c5的port0 i2c,sda是gpio2，scl是gpio3，频率是100KHz
i2c-tools> i2cconfig --port=0 --sda=2 --scl=3 --freq=100000

# 检查是否有设备挂在i2c总线上 scd的地址是0x62
i2c-tools> i2cdetect
     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f
00: 00 -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
10: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
20: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
30: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
40: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
50: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
60: -- -- 62 -- -- -- -- -- -- -- -- -- -- -- -- --
70: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 

# start_periodic_measurement 启动scd41 周期性测量co2(周期5s)
i2c-tools> i2cset -c 0x62 -r 0x21 0xB1
I (29477) cmd_i2ctools: Write OK

# read_measurement 发送读取测量命令
i2c-tools> i2cget -c 0x62 -r 0xec05 -l 9
0x01 0xf4 0x33 0x66 0x67 0xa2 0x5e 0xb9 0x3c
# 返回值说明:
# CO2 = 500 ppm CRC of 0x01f4 is 0x33
# Temp. = 25 °C CRC of 0x6667 is 0xa2
# RH = 37% CRC of 0x5eb9 is 0x3c
# 计算方法:
# 𝐶𝑂2 [ppm]=𝑤𝑜𝑟𝑑[0]
# 𝑇=−45+175∗𝑤𝑜𝑟𝑑[1]/(2^16−1)
# 𝑅𝐻=100∗𝑤𝑜𝑟𝑑[2]/(2^16−1) 

# get_data_ready_status 获取数据测量状态
i2c-tools> i2cget -c 0x62 -r 0xe4b8 -l 3
0x01 0xff 0xd9 
# 返回值说明:如果状态字的低 11 位不全为0，表示 数据已准备好

# stop_periodic_measurement 停止周期性测量
i2c-tools>  i2cset -c 0x62 -r 0x3F 0x86
I (1712817) cmd_i2ctools: Write OK

# set_ambient_pressure 设置大气压
i2c-tools> i2cset -c 0x62 -r 0xe0 0x00 0x03 0xdb 0x42
I (1712817) cmd_i2ctools: Write OK

# get_ambient_pressure 读取大气压设置值
i2c-tools> i2cget -c 0x62 -r 0xe000 -l 3
0x03 0xdb 0x42 
# 返回值说明:
# Ambient P = 98’700 Pa CRC of 0x03db is 0x42
# 计算方法: ambient P [Pa] = word[0] * 100
```

## SenseAir S88使用说明

s88的接口是串口，波特率9600，数据为8，校验位None，停止位1。

协议是Modbus，使用串口发送16进制数据。

```shell
# CO2 read sequence
# master发送
fe 04 00 03 00 01 d5 c5
# 数据格式说明:
# fe:任何地址
# 04:IR4（function code 04，读取寄存器）
# 00 03:寄存器偏移地址
# 00 01:读取的寄存器个数
# d5 c5:crc16校验值

# slave回复
fe 04 02 01 90 ac d8
# 数据格式说明:
# 01 90:co2浓度值的16进制数 400 ppm
```

## SHT86

```shell
i2c-tools> i2cset -c 0x44 -r 0x24 0x00
I (480797) cmd_i2ctools: Write OK


i2c-tools> i2cset -c 0x44 -r 0x20 0x32
i2c-tools> i2cset -c 0x44 -r 0x21 0x30
i2c-tools> i2cset -c 0x44 -r 0x22 0x36
i2c-tools> i2cset -c 0x44 -r 0x23 0x34
i2c-tools> i2cset -c 0x44 -r 0x27 0x37

i2cget -c 0x44 -r 0xe000 -l 6
```




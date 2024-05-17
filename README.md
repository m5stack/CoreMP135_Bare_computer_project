# stm32mp135_Bare_computer_project
这是一个 stm32mp135 的裸机工程，烧录到 sd 卡中后，启动设备 ，PC13 引脚会闪烁。

请在 linux 下食用。

## USE
```bash
# 下载工具链
wget https://github.com/dianjixz/stm32mp135_Bare_computer_project/releases/download/v0.0.1/toolchain.tar.gz

tar zxvf toolchain.tar.gz -C toolchain

# 编译
make 

# 清理编译文件
make clean
```
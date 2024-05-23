# stm32mp135_Bare_computer_project
This is a simple bare-metal project for the STM32MP135. After being flashed to an SD card and booting the device, the PC13 pin will blink.

Please compile under Linux.

## USE
```bash
# Download toolchain.
wget https://github.com/dianjixz/stm32mp135_Bare_computer_project/releases/download/v0.0.1/toolchain.tar.gz

tar zxvf toolchain.tar.gz -C toolchain

# Install the toolkit.
sudo apt install sgdisk make 

# Compile.
make 

# Clean up compiled files.
make clean
```
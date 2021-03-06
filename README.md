# Embedded Linux Kernel version 4.4.179 for RK3399

* A partial branch cloned kernel source from [here](https://github.com/rockchip-linux/kernel/tree/stable-4.4-rk3399-linux-v2.1x)
* Tuned for embedded linux ( not Android ), aarch64.

## Tested RK3399 boards

* Excavator Sapphire EVM
* Excavator Sapphire EVM clone ( variation B and C )
* 96Board Rock960A/B/C

## Included

* Realtek semiconductor Corp. RTL815x GbE USB adapter driver
* Microchip LAN78xx GbE USB adapter driver
* Cypress GX3 GbE USB adapter driver
* RaphKay's USB-C 3.0 driver with rockchip USB VID:PID as 0x2207:0x0009.

## Modified build config 

* arch/arm64/configs/rockchip_linux_defconfig

## Known issues

* Some Realtek manufacturer of RTL8153 GbE USB device makes network hub or router unstable when system power not turned off in sequencely.
* Some Realtek RTL815x 2.5 or faster devices are not fully supported over 1 Gbps.
* Some LAN78xx GbE device cannot be recognized well ( unstable )
* USB-C peripheral driver may not bound until addtional action required by adbd.sh.

## See more details about kernel

* [README](https://github.com/rageworx/rk3399_linux_kernel_4.4.179/blob/master/README)

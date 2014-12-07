This Openwrt fork is intended to add support for the MIPS Creator CI20 board.

The status, for now, is as follows:

Working:
- IRQ controller.
- DMA controller.
- Pinctrl.
- NEMC (nand and external memory controller).
- Davicom DM9000 ethernet.
- TCU and CGU.
- EHCI and OHCI usb port. OHCI HCD interferes with EHCI, so for now is better to not include it.
- RTC controller.
- ACT8600 power regulator.
- I2c bus.

Not workinkg:
- OTG USB driver (it crash when is loaded).
- MMC driver (sometimes work but others not, causing a hang).
- Wifi (brcmfmac): do not work as expected.

To be added:
- Audio subsystem.
- Bluetooth.
- ADC.
- PWM.
- Code to show the CPU, L2CACHE, DDR and AHB clocks at bootup like kernel 3.0.8 does.

How to boot the MIPS Creator CI20 with OpenWRT (for now):
- 1. Make a bootable memory card with an ext4 partition.
- 2. Inside the memory card first partition create a directory called "boot" and copy vmlinux.img on it.
- 3. Copy OpenWRT rootfs into an USB massive storage device. By default the kernel is configured to boot sda1.
- 4. Power on the board and test.

Notes:
- Despite the ethernet controller is attached to an "slow" internal memory bus, the performance is good (~ 75 Mbps) with kernel 3.16 and above.
- More information at elinux: http://elinux.org/MIPS_Creator_CI20

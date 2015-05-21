This Openwrt fork is intended to add support for the MIPS Creator CI20 board.

The status, for now, is as follows:

Working (at least seems to be):
- IRQ controller.
- DMA controller.
- Pinctrl.
- NEMC (nand and external memory controller).
- Davicom DM9000 ethernet.
- TCU and CGU.
- OHCI usb.
- DWC2 OTG USB.
- RTC controller.
- ACT8600 power regulator.
- I2c bus.
- MMC driver.
- Video.

Not workinkg:
- Wifi (brcmfmac): the driver does not properly detect the wireless module.
- EHCI usb.
- Audio: driver problem (wip).

Still not tested:
- NAND flash.

To be added:
- FUSE driver.
- Code to show the CPU, L2CACHE, DDR and AHB clocks at bootup like kernel 3.0.8 does.

How to boot the MIPS Creator CI20 with OpenWRT (for now):
- 1. Make a bootable memory card with an ext4 partition.
- 2. Inside the memory card first partition create a directory called "boot",if it does not exist, and copy vmlinux.img on it.
- 3. Copy OpenWRT rootfs into the MMC card.
- 4. Power on the board and test.

Notes:
- Some drivers need to be improved (WIP).
- Despite the ethernet controller is attached to an "slow" internal memory bus, the performance is good (~ 75 Mbps) with kernel 3.18 and above.
- More information at elinux: http://elinux.org/MIPS_Creator_CI20

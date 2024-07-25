### Broadcom - User mode FC target driver for SPDK FC NVMe.

This user-mode driver

1. provides FC LLD module to be included with Storage Performance Development Kit (SPDK).
2. based on the configuration options may support G6, Prism and Prism+ HBAs.

### Instructions to include the driver module with SPDK NVMF Target:

~~~{.sh}
git clone https://github.com/spdk/spdk spdk
git clone https://github.com/ecdufcdrvr/bcmufctdrvr fc
cd spdk
git submodule update --init
./configure --without-fc
make -j$(nproc)
cd ../fc
make -j$(nproc) DPDK_DIR=../spdk/dpdk/build SPDK_DIR=../spdk
cd ../spdk
./configure --with-fc=../fc/build
make -j$(nproc)
~~~

### Instructions to run the nvmf_tgt app:

1. Boot the kernel with Intel io_mmu. Set grub boot option "intel_iommu=on" and reboot.
# grubby --args=intel_iommu=on --update-kernel /boot/<vmlinux-bootable-image>

2. From the fc folder, run start_spdk_tgt.sh to start the SPDK target.
# sh start_spdk_tgt.sh

3. From a different terminal, run create_spdk_cfg.sh after editing the file appropriately to load the nvmf_tgt app.
# sh create_spdk_cfg.sh


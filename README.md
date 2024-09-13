### Broadcom - User mode FC target driver for SPDK FC NVMe.

This user-mode driver

1. provides FC LLD module to be included with Storage Performance Development Kit (SPDK).
2. based on the configuration options, following Broadcom/Emulex devices are supported,
	- Emulex Corporation LPe37000/LPe38000 Series 32Gb/64Gb Fibre Channel Adapter
	- Emulex Corporation LPe35000/LPe36000 Series 32Gb/64Gb Fibre Channel Adapter
	- Emulex Corporation LPe31000/LPe32000 Series 16Gb/32Gb Fibre Channel Adapter

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

1. Boot the kernel with Intel io_mmu enabled. Set grub boot option "intel_iommu=on" and reboot.
   ~~~{.sh}
      grubby --args=intel_iommu=on --update-kernel /boot/<vmlinux-bootable-image>
   ~~~

2. A sample script, 'start_spdk_tgt.sh' is provided to demonstrate the steps to unbind the
   default kernel drivers such as 'lpfc' and 'ocs_fc_ramd' and assign the adapters to 'vfio-pci' module.
   The 'vfio-pci' kernel module binding is necessary for the user mode driver to claim adapter.

   From the fc folder, run start_spdk_tgt.sh to start the SPDK target.
   ~~~{.sh}
       sh start_spdk_tgt.sh
   ~~~

3. A second script, 'create_spdk_cfg.sh' is included to demonstrate FC transport creation and assigning
   the FC listen address (based on FC port's WWPN/WWNN) to nvme subsystem operations.

   From a different terminal, run create_spdk_cfg.sh after editing the file appropriately to load the nvmf_tgt app.
   ~~~{.sh}
       sh create_spdk_cfg.sh
   ~~~

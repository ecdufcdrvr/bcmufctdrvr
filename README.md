## Broadcom - User mode FC target driver for SPDK FC NVMe.

Driver provides FC LLD module to be included with Storage Performance Development Kit (SPDK).

User mode driver supports Emulex LPe31000-series and LPe32000-series HBAs.

### Instructions to include driver module with SPDK NVMF Target:

~~~{.sh}
git clone https://github.com/spdk/spdk spdk
git clone https://github.com/ecdufcdrvr/bcmufctdrvr fc
cd spdk
git submodule update --init
./configure --without-fc
make
cd ../fc
make DPDK_DIR=../spdk/dpdk/build SPDK_DIR=../spdk
cd ../spdk
./configure --with-fc=../fc/build
make
~~~

### SPDK Releases compatibility

Broadcom FC LLD git master is always compatible with the spdk.io master.
Repository also includes compatible LLD's for various SPDK releases.
FC driver branch code that is compatible with various SPDK releases is
documented below.

    SPDK release            Broadcom LLD branch name
    ============           ==========================
       19.07              remotes/origin/bcm_lld_rel_v19.07.x
       20.01              remotes/origin/bcm_lld_rel_v20.01.x
       20.04              remotes/origin/bcm_lld_rel_v20.04.x
       20.07              remotes/origin/bcm_lld_rel_v20.07.x

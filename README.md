## Broadcom - User mode FC target driver for SPDK FC NVMe.

Driver provides FC LLD module to be included with Storage Performance Development Kit (SPDK).

User mode driver supports Emulex LPe31000-series and LPe32000-series HBAs.

### Instructions to include driver module with SPDK NVMF Target:

~~~{.sh}
git clone https://github.com/spdk/spdk spdk
git clone https://github.com/ecdufcdrvr/bcmufctdrvr fc
cd spdk
git submodule update --init
cd ../fc
make DPDK_DIR=../spdk/dpdk/build SPDK_DIR=../spdk
cd ../spdk
./configure --with-fc=../fc/build
make
~~~

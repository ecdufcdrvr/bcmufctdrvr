## Broadcom - User mode FC target driver for SPDK FC NVMe.

Driver provides FC LLD module to be included with Storage Performance Development Kit (SPDK).

User mode driver supports Emulex LPe31000-series and LPe32000-series HBAs.

### Instructions to include driver module with SPDK NVMF Target:

~~~{.sh}
git clone https://github.com/spdk/spdk
cd spdk
git submodule update --init
git clone https://github.com/ecdufcdrvr/bcmufctdrvr fc
./configure --with-fc
make
~~~

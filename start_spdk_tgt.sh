lsmod | grep lpfc
if [ $? -eq 0 ]; then 
	echo "Unloading lpfc driver..."
	rmmod lpfc
fi

lsmod | grep ocs_fc_ramd
if [ $? -eq 0 ]; then 
	echo "Unloading ocs_fc_ramd driver..."
	rmmod ocs_fc_ramd
fi

lsmod | grep ocs_fc_lio
if [ $? -eq 0 ]; then 
	echo "Unloading ocs_fc_lio driver..."
	rmmod ocs_fc_lio
fi

lsmod | grep ocs_fc_scst
if [ $? -eq 0 ]; then 
	echo "Unloading ocs_fc_scst driver..."
	rmmod ocs_fc_scst
fi

lsmod | grep ocs_uspace
if [ $? -eq 0 ]; then 
	echo "Unloading ocs_uspace driver..."
	rmmod ocs_uspace
fi

lsmod | grep ocs_uio
if [ $? -eq 0 ]; then 
	echo "Unloading ocs_uio driver..."
	rmmod ocs_uio
fi

rmmod vfio_pci vfio_iommu_type1 vfio_virqfd vfio 2>/dev/null
rmmod uio_pci_generic uio 2>/dev/null

# Load vfio driver and configure hugepages
modprobe -v vfio_pci

echo "10df e300" > /sys/bus/pci/drivers/vfio-pci/new_id # LPe31000/LPe32000 Series Adapter
echo "10df f400" > /sys/bus/pci/drivers/vfio-pci/new_id # LPe35000/LPe36000 Series Adapter
echo "10df f500" > /sys/bus/pci/drivers/vfio-pci/new_id # LPe37000/LPe38000 Series Adapter

echo 2048 > /sys/devices/system/node/node0/hugepages/hugepages-2048kB/nr_hugepages
echo 2048 > /sys/devices/system/node/node1/hugepages/hugepages-2048kB/nr_hugepages

cd ../

# Start spdk target
spdk/build/bin/nvmf_tgt -m 0xffff 2>&1 | tee core_log


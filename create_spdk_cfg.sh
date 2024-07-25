cd ../

# Create a nvmf subsystem
spdk/scripts/rpc.py nvmf_create_subsystem -s SPDK00000000000001 -a nqn.2016-06.io.spdk:cnode1

# Create a block device like malloc device
spdk/scripts/rpc.py bdev_malloc_create -b Malloc0 512 512 -u 25eb230b-a9e4-4abb-a3db-a2f507d7b563

# Assign the malloc device to subsystem
spdk/scripts/rpc.py nvmf_subsystem_add_ns -n 1 nqn.2016-06.io.spdk:cnode1 Malloc0 -u 25eb230b-a9e4-4abb-a3db-a2f507d7b563

# Create any other devices or subsystems as required

# Create FC transport
spdk/scripts/rpc.py nvmf_create_transport -t FC

# Finally add the FC listener section for this newly created subsystem; this should be last
spdk/scripts/rpc.py nvmf_subsystem_add_listener -t FC -f FC -a nn-0x200000109bf6d490:pn-0x100000109bf6d490 -s none nqn.2016-06.io.spdk:cnode1

# Save this config for future use; it can be used later like spdk/build/bin/nvmf_tgt -m 0xff -c config.json
spdk/scripts/rpc.py save_config > config.json


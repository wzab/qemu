/tmp/qemu/bin/qemu-system-x86_64 -M q35 -m 16384 \
  -hda hda.img \
  --enable-kvm \
  --device pci-wzdaq1 \
  -virtfs local,path=/tmp/p9,mount_tag=host0,security_model=none,id=host0

#  -cdrom debian-testing-amd64-netinst.iso 

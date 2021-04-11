/tmp/qemu/bin/qemu-system-x86_64 -M q35 -m 16384 \
  -cdrom KNOPPIX_V9.1DVD-2021-01-25-EN.iso \
  --enable-kvm \
  --device pci-wzdaq1 \
  -virtfs local,path=/tmp/p9,mount_tag=host0,security_model=none,id=host0


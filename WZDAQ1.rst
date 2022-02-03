=============
WZDAQ1 branch
=============

This version adds files wzab_daq1.c and wzab_daq1.h in the hw/misc subdirectory.
Those files emulate a PCIe device that is a prototype of the final stage of
a data acquisition system implemented in FPGA.

The aim of that project is to verify the concept of the IP core,
of the Linux driver, and of the data receiving application.

The concentrated data are delivered as records,
delivered via ZMQ protocol.

In the current version, the emulated DMA core works with 256-bit (32 byte) words.
They should be organized as follows:

- "WZDAQ1-D" (followed with 24 0x00 bytes which are ignored).
  Start of a part of a segment of data.   
  After that the data must follow.
- "WZDAQ1-T" (followed with 24 0x00 bytes which are ignored).
  The final part of a segment of data.   
  After that the data must follow.

The compilation has been tested for targets:
./configure --target-list=x86_64-softmmu,arm-softmmu,aarch64-softmmu

To compile, you should add :czmq" to the list of libraries.
I have called in "build" directory the following command:

    LIBS="-lczmq" make

after the configure is done.

For newest gcc (10 and 11) I had to disable certain warnings from being
promoted to errors. The brute-force solution is to do:

CFLAGS=-Wno-error ./configure --target-list=x86_64-softmmu,arm-softmmu,aarch64-softmmu

when configuring.





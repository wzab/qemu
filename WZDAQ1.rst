=============
WZDAQ1 branch
=============

This version adds files wzab_daq1.c and wzab_daq1.h in the hw/misc subdirectory.
Those files emulate a PCIe device that is a prototype of the final stage of
a data acquisition system implemented in FPGA.

The aim of that project is to verify the concept of the IP core,
of the Linux driver, and of the data receiving application.

The concentrated data are delivered as msgpack-packed records,
delivered via ZMQ protocol.

In the first version, the structure is as follows:

- Type of the record (8-byte string)
  - "WZDAQ1-B" Start of the new dataset
  - "WZDAQ1-C" Complete dataset
  - "WZDAQ1-E" End of the dataset
- Sequence of 64-bit words with the collected data



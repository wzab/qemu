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

In the first version, the structure is as follows:

- "WZDAQ1-E" Start of the new dataset

- "WZDAQ1-D" Data chunk for the dataset, must be followed by the number of
   64-bit data words belonging to that chunk.
   
   After that the data must follow.

To compile, you should add :czmq" to the list of libraries.
I have called in "build" directory the following command:

    LIBS="-lczmq" make

after the configure is done.




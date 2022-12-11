#ifndef __WZAB_DAQ_H__
#define __WZAB_DAQ_H__

// Define the number of bytes in the DMA data word
#define DAQ1_BYTES_IN_DATA_WORD 32
// Define the length of the single huge page-based buffer in bytes...
#define DAQ1_BUFLEN_IN_BYTES (2*1024*1024)
// ...and in words
#define DAQ1_BUFLEN_IN_WORDS (DAQ1_BUFLEN_IN_BYTES / DAQ1_BYTES_IN_DATA_WORD)
// Definitions of 64-bit registers
// They are located in BAR2. Below are just offsets
#define AXI_REGS 0x0000
#define AXI_HLS  0x10000
//Length of the area that should be mmapped for debugging
#define AXI_MMAP_LEN 0x20000

//https://www.xilinx.com/support/documentation/ip_documentation/axi_gpio/v2_0/pg144-axi-gpio.pdf

// Access to the GPIO with core ID
#define AXI_ID_IND (AXI_REGS + 0x0)
// Expected ID value
#define AXI_ID_VAL 0x32abd3a2

// Access to the Control GPIO OUTPUT ports
#define AXI_CTRL_OUTD (AXI_REGS + 4 * 0x4)
// Assignment of bits:
// 0 - ap_start
// 1 - run (in DMA core)
// 2 - start (in emulated data source)
// 3 - interrupt enable
#define CTRL_OUTD_BIT_AP_START 0
#define CTRL_OUTD_BIT_AP_nRST 1
#define CTRL_OUTD_BIT_SRC_START 2
#define CTRL_OUTD_BIT_IRQ_ENA 3

// Access to the Control GPIO INPUT ports
#define AXI_CTRL_IND (AXI_REGS + 4 * 0x7)
//Assignment of bits
// 0 - ap_done
// 1 - ap_ready
// 2 - ap_idle
// 3 - overrun
// 4 - pkt_av
#define CTRL_IND_BIT_DONE 0
#define CTRL_IND_BIT_READY 1
#define CTRL_IND_BIT_IDLE 2
#define CTRL_IND_BIT_OVERRUN 3
#define CTRL_IND_BIT_PKTAV 4

// Access to the GPIOs with the packet numbers
#define AXI_REGS_PKT_CUR (AXI_REGS + 4 * 0x1)
#define AXI_REGS_PKT_SRV (AXI_REGS + 4 * 0x3)
#define AXI_REGS_PKT_NR (AXI_REGS + 4 * 0x5)

// Access to the GPIOs with the buffers numbers
#define AXI_REGS_BUF_CUR (AXI_REGS + 4 * 0x2)
#define AXI_REGS_BUF_NR (AXI_REGS + 4 * 0x6)

// Access to the HLS-defined part
#include "xdma1_hw.h"

//Address of the huge page with descriptors of events
#define DAQ1_DESCS	(AXI_HLS + XDMA1_CONTROL_ADDR_DESCS_V_DATA)
//Address of the register with the number of the first not handled packet
#define DAQ1_CUR_PKT   (AXI_REGS_PKT_CUR)
//Address of the register with the number of the first not scheduled for handling packet
#define DAQ1_SRV_PKT   (AXI_REGS_PKT_SRV)
//Address of the register with the number of the currently filled packet
#define DAQ1_NR_PKT   (AXI_REGS_PKT_NR)

//Address of the register with the number of the first not handled buffer
#define DAQ1_CUR_BUF   (AXI_REGS_BUF_CUR)
//Address of the register with the number of the currently filled buffer
#define DAQ1_NR_BUF   (AXI_REGS_BUF_NR)

//Address of the register with the number of the buffers
#define DAQ1_NOF_BUFS   (AXI_HLS + XDMA1_CONTROL_ADDR_NOF_BUFS_V_DATA)

//Base address of the memory
#define DAQ1_BUFS   (AXI_HLS + XDMA1_CONTROL_ADDR_BUFS_V_BASE)
#define DAQ1_BUFS_HIGH   (AXI_HLS + XDMA1_CONTROL_ADDR_BUFS_V_HIGH)

// Maximum number of the HP-based buffers:
#define DAQ1_MAX_NOF_BUFS (DAQ1_BUFS_HIGH - DAQ1_BUFS + 1)
// Size of the event descriptor in bytes (must be power of 2)
#define DAQ1_EVT_DESC_SIZE 32

//Layout of the EVENT descriptor
#define DAQ1_EVT_NUM 0
#define DAQ1_EVT_FIRST 1
#define DAQ1_EVT_AFTER_LAST 2
#define DAQ1_EVT_STATUS 3

//Number of the EVENT descriptors
#define DAQ1_NUM_EVT_DESCS (DAQ1_BUFLEN_IN_BYTES / DAQ1_EVT_DESC_SIZE)

//Subcommands handled by IOC_CTRL command
#define DAQ1_CMD_STOP 0
#define DAQ1_CMD_START 1
#define DAQ1_CMD_ENA_IRQ 2
#define DAQ1_CMD_DIS_IRQ 3
#define DAQ1_CMD_CONFIRM 4
#define DAQ1_CMD_INIT 5
#define DAQ1_CMD_DEINIT 6
#define DAQ1_CMD_DO_RESET 7
//Two commands below must be used with care! 
//Any access to the HLS part registers without releasing reset
//may result with locking the PCIe-AXI bridge.
#define DAQ1_CMD_SET_RESET 8
#define DAQ1_CMD_CLR_RESET 9

#endif


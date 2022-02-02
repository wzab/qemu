// ==============================================================
// Vivado(TM) HLS - High-Level Synthesis from C, C++ and SystemC v2020.1 (64-bit)
// Copyright 1986-2020 Xilinx, Inc. All Rights Reserved.
// ==============================================================
// control
// 0x0000 : reserved
// 0x0004 : reserved
// 0x0008 : reserved
// 0x000c : reserved
// 0x2000 : Data signal of descs_V
//          bit 31~0 - descs_V[31:0] (Read/Write)
// 0x2004 : Data signal of descs_V
//          bit 31~0 - descs_V[63:32] (Read/Write)
// 0x2008 : reserved
// 0x200c : Data signal of nof_bufs_V
//          bit 31~0 - nof_bufs_V[31:0] (Read/Write)
// 0x2010 : reserved
// 0x1000 ~
// 0x1fff : Memory 'bufs_V' (512 * 64b)
//          Word 2n   : bit [31:0] - bufs_V[n][31: 0]
//          Word 2n+1 : bit [31:0] - bufs_V[n][63:32]
// (SC = Self Clear, COR = Clear on Read, TOW = Toggle on Write, COH = Clear on Handshake)

#define XDMA1_CONTROL_ADDR_DESCS_V_DATA    0x2000
#define XDMA1_CONTROL_BITS_DESCS_V_DATA    64
#define XDMA1_CONTROL_ADDR_NOF_BUFS_V_DATA 0x200c
#define XDMA1_CONTROL_BITS_NOF_BUFS_V_DATA 32
#define XDMA1_CONTROL_ADDR_BUFS_V_BASE     0x1000
#define XDMA1_CONTROL_ADDR_BUFS_V_HIGH     0x1fff
#define XDMA1_CONTROL_WIDTH_BUFS_V         64
#define XDMA1_CONTROL_DEPTH_BUFS_V         512


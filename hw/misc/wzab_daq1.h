// Definitions of 64-bit registers
// They are located at the begining of the BAR0
#define DAQ1_DIV  	0
#define DAQ1_READP	1
#define DAQ1_WRITEP	2
#define DAQ1_CTRL       3
#define DAQ1_STAT	4
//Address of the huge page with descriptors of events
#define DAQ1_EVTS	5

//Number of the 64-bit registers
#define DAQ1_REGS_NUM	6

// The memory that stores the huge pages forming the data buffer is located in BAR0 at offset DAQ1_BUFF_OFFS
// We want to align it to its size. Now we assume that 8192 buffers will be sufficient, so the offset is set to:
// 8*8192 = 8 * 0x2000 = 0x10000
#define DAQ1_BUFS	0x10000
// Number of the buffers:
#define DAQ1_NBUFS	0x2000


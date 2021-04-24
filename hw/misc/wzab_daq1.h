// Definitions of 64-bit registers
// They are located at the begining of the BAR0
#define DAQ1_READP	0
#define DAQ1_WRITEP	1
#define DAQ1_CTRL       2
#define DAQ1_STAT	3
//Address of the huge page with descriptors of events
#define DAQ1_EVTS	4
#define DAQ1_PERIOD     5
#define DAQ1_HPSHFT     6
#define DAQ1_EVT_READP  7
#define DAQ1_EVT_WRITEP 8
#define DAQ1_NOF_HP     9
//Number of the 64-bit registers
#define DAQ1_REGS_NUM	10

// The memory that stores the huge pages forming the data buffer is located in BAR0 at offset DAQ1_BUFF_OFFS
// We want to align it to its size. Now we assume that 8192 buffers will be sufficient, so the offset is set to:
// 8*8192 = 8 * 0x2000 = 0x10000
// Number of the buffers:
#define DAQ1_NBUFS	0x2000
// Size of the event descriptor in 64-bit words (must be power of 2)
#define DAQ1_EVT_DESC_SIZE 32

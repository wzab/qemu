//It would be good to prepare it in the form, where
//we can easily switch between th 32-bit and 64-bit implementations.

typedef uint64_t com1_addr_t;

//The structure below describes a single buffer
typedef struct BufDesc {
    com1_addr_t BufAddr;
    com1_addr_t Offset;
    com1_addr_t Length;
} __attribute__((packed)) BufDesc;

//The structure below describes is a single descriptor
typedef struct Com1Desc {
    com1_addr_t Length; //Number of entries
    com1_addr_t Next; /* Pointer to the next descriptor (NULL means
	                   that was the last one.*/
	BufDesc Descs[]; //Array with descriptors of buffers
} __attribute__((packed)) Com1Desc;

typedef struct WzCom1Regs {
    com1_addr_t Id; // To check that the device is here and is accessible
    com1_addr_t Test; // To check that we can write and read the register
    com1_addr_t Ctrl;
    com1_addr_t Status;
    com1_addr_t FirstDesc;
    com1_addr_t CurDesc; //Currently processed descriptor
    com1_addr_t CurBuf; //Number of the currently processed buffer
    com1_addr_t CurPos; //Currently processed position in the buffer
    com1_addr_t CurCount; //Number of bytes processed in the current transfer
} __attribute__((packed)) WzCom1Regs;

//Commands
#define COM1_CMD_START 1
#define COM1_CMD_ENAIRQ 2
#define COM1_CMD_DISIRQ 3
#define COM1_CMD_ACKIRQ 4

//Errors
#define COM1_ERR_NOTINIT 1
#define COM1_ERR_BUSY 2



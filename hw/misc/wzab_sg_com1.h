//It would be good to prepare it in the form, where
//we can easily switch between th 32-bit and 64-bit implementations.

typedef uint32_t com1_addr_t;

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
    uint32_t Ctrl;
    uint32_t Status;
    com1_addr_t FirstDesc;
    com1_addr_t CurDesc; //Currently processed descriptor
    com1_addr_t CurBuf; //Number of the currently processed buffer
    com1_addr_t CurPos; //Currently processed position in the buffer
    com1_addr_t CurCount; //Number of bytes processed in the current transfer 
} __attribute__((packed)) WzCom1Regs;

//Commands
#define COM1_CMD_DECR 1
#define COM1_CMD_ENCR 2
#define COM1_CMD_DATA 3
#define COM1_CMD_STOP 4
#define COM1_CMD_ENAIRQ 5
#define COM1_CMD_DISIRQ 6
#define COM1_CMD_ACKIRQ 7

//Errors
#define COM1_ERR_NOTINIT 1
#define COM1_ERR_BUSY 2



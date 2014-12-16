//Definitions of 32-bit registers
#define WZENC1_NOF_PAGES 4
typedef struct WzEnc1Page {
  uint32_t PhysAddr;
  uint32_t Offset;
  uint32_t Length;
} WzEnc1Page;

typedef struct WzEnc1Regs {
  uint32_t Ctrl;
  uint32_t Status;
  WzEnc1Page Pages[WZENC1_NOF_PAGES]; 
} WzEnc1Regs;


//Commands
#define ENC1_CMD_DECR 1
#define ENC1_CMD_ENCR 2
#define ENC1_CMD_DATA 3
#define ENC1_CMD_STOP 4
#define ENC1_CMD_ENAIRQ 5
#define ENC1_CMD_DISIRQ 6
#define ENC1_CMD_ACKIRQ 7

//Errors
#define ENC1_ERR_NOTINIT 1
#define ENC1_ERR_BUSY 2



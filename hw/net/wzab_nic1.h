//Definitions of 32-bit registers
#define WZNIC1_NOF_TX_PAGES 4
#define WZNIC1_NOF_RX_PAGES 4
typedef struct  {
    uint32_t PhysAddr;
} WzNic1Page;

typedef struct {
    uint32_t MAC_L;
    uint32_t MAC_H;
    uint32_t Ctrl;
    uint32_t Status;
    uint32_t RxHead;
    uint32_t RxTail;
    uint32_t RxNumOfBytes;
    uint32_t TxHead;
    uint32_t TxTail;
    uint32_t TxNumOfBytes;
    WzNic1Page TxPages[WZNIC1_NOF_TX_PAGES];
    WzNic1Page RxPages[WZNIC1_NOF_RX_PAGES];
} WzNic1Regs;

#define WZNIC1_PKT_MAGIC_NUMBER (0xbcda1234)
typedef struct {
    uint32_t MagicNumber;
    uint32_t Length;
    uint32_t Flags;
} WzNic1_PktHeader;

//Commands
#define NIC1_CMD_ENA_TX_IRQ 1
#define NIC1_CMD_DIS_TX_IRQ 2
#define NIC1_CMD_ENA_RX_IRQ 3
#define NIC1_CMD_DIS_RX_IRQ 4
#define NIC1_CMD_ENA_RX 5
#define NIC1_CMD_DIS_RX 6
#define NIC1_CMD_SEND 7
#define NIC1_CMD_CLEAR 8
#define NIC1_CMD_ENA_PROMISCOUS 9
#define NIC1_CMD_DIS_PROMISCOUS 10


//Bit masks in the status registers
#define NIC1_ST_IRQ 0x8000
#define NIC1_ST_TX_IRQ 0x4000
#define NIC1_ST_TX_IRQ_ENA 0x2000
#define NIC1_ST_RX_IRQ 0x1000
#define NIC1_ST_RX_IRQ_ENA 0x0800
#define NIC1_ST_PROMISCOUS 0x0400
//
#define NIC1_ST_ENA_RX 0x0004
#define NIC1_ST_TX_COR_BUF 0x0002
#define NIC1_ST_ERROR 0x0001
//Errors



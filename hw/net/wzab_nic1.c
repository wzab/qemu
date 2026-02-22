/*
 * QEMU -  Emulation of network adapter with Bus Mastering DMA and scattered packet buffers
 *
 * Copyright Wojciech M. Zabolotny ( wzab@ise.pw.edu.pl ) 2011-2014
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

/* based on mipsnet.c taken from original QEMU sources */

//#define DEBUG_wzab1 1
#define MAX_ETH_FRAME_SIZE 1514
//PCI IDs below are not registred! Use only for experiments!
#define PCI_VENDOR_ID_WZAB 0xabba
#define PCI_DEVICE_ID_WZAB_WZNIC1 0x0232

//Simulated processing time
#define NIC1_PROCESSING_TIME 5000000
#include <string.h>
#include "hw/sysbus.h"
#include "hw/hw.h"
#include "hw/pci/pci.h"
#include "net/net.h"
#include "qemu/timer.h"
#include "wzab_nic1.h"
#include <stdint.h>

#define min(x,y) ((x<y) ? x : y)
/*
  Description of functionalities.
  The emulated hardware is a network adapter working with packet buffers
  located in the host's memory.
  There are two packet buffers - transmit buffer and receive buffer.
  Each of them may span across several pages.

  Packet buffers are served as circular buffers.
  Each packet is stored in a special form, allowing to separate individual
  packets:
  A. Header:
   1. Packet magic number
   2. Packet length (including the header and the contents)
   3. Packet flags (not used yet)
  B. Packet contents

  Packet transmission:
  Driver writes the packet to the packet buffer. After the packet is stored,
  the driver updates the head pointer in the Tx packet buffer.
*/
/* Structure used as a pointer to data in Tx/Rx buffers */
typedef struct
{
    int page;
    int offset; //in 32 bit words!
} WzNic1Ptr;

typedef struct WzNic1State
{
    
    /*<private>*/
    PCIDevice parent_obj;
    /*<public>*/
    union
    {
        WzNic1Regs r;
        uint32_t u32[sizeof ( WzNic1Regs ) /sizeof ( uint32_t ) ];
    } regs;
    
    WzNic1Ptr Tx_Tail;
    WzNic1Ptr Tx_Head;
    WzNic1Ptr Rx_Tail;
    WzNic1Ptr Rx_Head;
    MemoryRegion mmio;
    QEMUTimer * timer;
    QemuMutex lock;
    NICState *nic;
    NICConf conf;
unsigned int Error:
    1;
unsigned int TxCorruptedBuffer:
    1 ;
unsigned int Tx_Irq_Enabled :
    1;
unsigned int Rx_Irq_Enabled :
    1;
unsigned int Rx_Enabled :
    1;
unsigned int tx_irq_pending :
    1;
unsigned int rx_irq_pending :
    1;
unsigned int irq_pending :
    1;
unsigned int promiscous :
    1;
} WzNic1State;

#define TYPE_PCI_WZNIC1 "pci-wznic1"
#define PCI_WZNIC1(obj) \
    OBJECT_CHECK(WzNic1State, (obj), TYPE_PCI_WZNIC1)

//Some prototypes...
static uint64_t pci_wz_nic1_read ( void *opaque, hwaddr addr, unsigned size );
static void pci_wz_nic1_write ( void *opaque, hwaddr addr, uint64_t val, unsigned size );
static int pci_wz_nic1_init ( PCIDevice *dev );
void wz_nic1_send_packets ( WzNic1State * s );
void wz_nic1_update_irq(WzNic1State * s);
static NetClientInfo wz_nic1_nc_info;

static const MemoryRegionOps pci_wz_enc1_mmio_ops = {
    .read = pci_wz_nic1_read,
    .write = pci_wz_nic1_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .impl = {
        .min_access_size = 4, //Always 32-bit access!!
        .max_access_size = 4,
    },
};

static void wz_nic1_reset ( WzNic1State *s )
{
    memset ( ( void * ) s->regs.u32,0,sizeof ( s->regs.u32 ) );
#ifdef DEBUG_wzab1
    printf ( "wzab_nic1 reset!\n" );
#endif
}

/* called for read accesses to our register memory area */
uint64_t pci_wz_nic1_read ( void *opaque, hwaddr addr, unsigned size )
{
    int i_addr;
#ifdef DEBUG_wzab1
    printf ( "Memory read: address %x\n ", ( unsigned int ) addr );
#endif
    WzNic1State *s = opaque;
    if ( addr>=sizeof ( WzNic1Regs ) )
    {
        //Read above registers area - return "0xbada4ea" to show it
        return 0xbada4ea;
    }
    else if ( addr == offsetof ( WzNic1Regs, Status ) )
    {
        //Reading of the Status - synthesize the correct value
        uint32_t res = s->Error & 0xffff ; //Lower 16 bits!
        if ( s->irq_pending ) res |= NIC1_ST_IRQ;
        if ( s->tx_irq_pending ) res |= NIC1_ST_TX_IRQ;
        if ( s->Tx_Irq_Enabled ) res |= NIC1_ST_TX_IRQ_ENA;
        if ( s->rx_irq_pending ) res |= NIC1_ST_RX_IRQ;
        if ( s->Rx_Irq_Enabled ) res |= NIC1_ST_RX_IRQ_ENA;
        if ( s->promiscous ) res |= NIC1_ST_PROMISCOUS;
        //
        if (s->Rx_Enabled) res |= NIC1_ST_ENA_RX;
        if (s->TxCorruptedBuffer) res |= NIC1_ST_TX_COR_BUF;
        if (s->Error) res |= NIC1_ST_ERROR;
#ifdef DEBUG_wzab1
        printf ( "Status=%d\n",res );
#endif
        return res;
    }
    else if ( addr == offsetof ( WzNic1Regs, RxTail ) )
    {
        uint32_t res;
        qemu_mutex_lock(&s->lock);
        res = s->Rx_Tail.offset+(s->Rx_Tail.page<<10);
        qemu_mutex_unlock(&s->lock);
        return res;
    }
    else if ( addr == offsetof ( WzNic1Regs, RxHead ) )
    {
        uint32_t res;
        qemu_mutex_lock(&s->lock);
        res = s->Rx_Head.offset+(s->Rx_Head.page<<10);
        qemu_mutex_unlock(&s->lock);
        return res;
    }
    else if ( addr == offsetof ( WzNic1Regs, RxNumOfBytes ) )
    {
        uint32_t res;
        qemu_mutex_lock(&s->lock);
        res = WZNIC1_NOF_RX_PAGES * 4096;
        qemu_mutex_unlock(&s->lock);
        return res;
    }
    else if ( addr == offsetof ( WzNic1Regs, TxTail ) )
    {
        uint32_t res;
        qemu_mutex_lock(&s->lock);
        res = s->Tx_Tail.offset+(s->Tx_Tail.page<<10);
        qemu_mutex_unlock(&s->lock);
        return res;
    }
    else if ( addr == offsetof ( WzNic1Regs, TxHead ) )
    {
        uint32_t res;
        qemu_mutex_lock(&s->lock);
        res = s->Tx_Head.offset+(s->Tx_Head.page<<10);
        qemu_mutex_unlock(&s->lock);
        return res;
    }
    else if ( addr == offsetof ( WzNic1Regs, TxNumOfBytes ) )
    {
        uint32_t res;
        qemu_mutex_lock(&s->lock);
        res = WZNIC1_NOF_TX_PAGES * 4096;
        qemu_mutex_unlock(&s->lock);
        return res;
    }
    else
    {
        i_addr = addr/4;
        //Read the register
        return s->regs.u32[i_addr];
    }
}

/* Function used to dereference bytes/words in the Tx buffer */
static inline uint32_t tx_buf_addr ( WzNic1State *s, WzNic1Ptr ptr )
{
    return ( s->regs.r.TxPages[ptr.page].PhysAddr ) +4* ( ptr.offset );
}

static inline void tx_increase_ptr ( WzNic1State *s, WzNic1Ptr * ptr )
{
    int tmp = ptr->offset + 1;
    if ( tmp>=1024 )
    {
        tmp -= 1024;
        ptr->page ++;
        if ( ptr->page >= WZNIC1_NOF_TX_PAGES )
            ptr->page= ptr->page % WZNIC1_NOF_TX_PAGES;
    }
    ptr->offset = tmp;
}

static inline void tx_add_ptr (WzNic1State *s, WzNic1Ptr * ptr, uint32_t val)
{
    uint32_t tmp = ptr->offset+val;
    uint32_t npages = tmp >> 10; //Number of pages
    ptr->offset = tmp & 0x3ff; //lower 10 bits, as pages are 4096 bytes long
    ptr->page = (ptr->page + npages) % WZNIC1_NOF_TX_PAGES;
}

/* Function used to dereference bytes/words in the Rx buffer */
static inline uint32_t rx_buf_addr ( WzNic1State *s, WzNic1Ptr ptr )
{
    return  ( s->regs.r.RxPages[ptr.page].PhysAddr ) +4* ( ptr.offset );
}

static inline void rx_increase_ptr ( WzNic1State *s, WzNic1Ptr * ptr )
{
    int tmp = ptr->offset + 1;
    if ( tmp>=1024 )
    {
        tmp -= 1024;
        ptr->page ++;
        if ( ptr->page >= WZNIC1_NOF_RX_PAGES )
            ptr->page= ptr->page % WZNIC1_NOF_RX_PAGES;
    }
    ptr->offset = tmp;
}

static inline void rx_add_ptr (WzNic1State *s, WzNic1Ptr * ptr, uint32_t val)
{
    uint32_t tmp = ptr->offset+val;
    uint32_t npages = tmp >> 10; //Number of pages
    ptr->offset = tmp & 0x3ff; //lower 10 bits, as pages are 4096 bytes long
    ptr->page = (ptr->page + npages) % WZNIC1_NOF_RX_PAGES;
}

static inline uint32_t rx_buf_space(WzNic1State *s)
{
    const uint32_t num_of_bytes = WZNIC1_NOF_RX_PAGES*4096;
    int space;
    qemu_mutex_lock(&s->lock);
    int head = s->Rx_Head.page*4096+s->Rx_Head.offset*4;
    int tail = s->Rx_Tail.page*4096+s->Rx_Tail.offset*4;
    qemu_mutex_unlock(&s->lock);
    space = tail-head;
    if (space <= 0) space += num_of_bytes;
    space --;
    return space;
}
/* Procedure sending the packets from the Tx buffer */
void wz_nic1_send_packets ( WzNic1State * s )
{
    //Check if there is anything to send
    //We have a few options to send the packets
    //either we send all packets immediately, or we simulate delay associated
    //with sending of packets
    /* Sending of all packets at the same time is easier */
    while (1)
    {
        //There is something to send
        //Check the header
        WzNic1Ptr ptr;
        uint32_t flags, tmp;
        int bptr;
        uint32_t len_in_bytes; //Length of the packet without the header
        uint32_t len_in_u32; //Length of the packet in 32-bit words
        //int niov; //Number of iovecs to send the packet
        qemu_mutex_lock(&s->lock);
        if ((s->Tx_Tail.offset == s->Tx_Head.offset) &&
                (s->Tx_Tail.page == s->Tx_Head.page)) {
            qemu_mutex_unlock(&s->lock);
            return; //No more packets to send
        }
        memcpy ( &ptr,&s->Tx_Tail,sizeof ( ptr ) );
        qemu_mutex_unlock(&s->lock);
        bptr = tx_buf_addr ( s, ptr );
        pci_dma_read(&s->parent_obj,bptr,&tmp,4);
        if (tmp != WZNIC1_PKT_MAGIC_NUMBER )
        {
            s->TxCorruptedBuffer = 1;
            s->Error = 1;
            //Error - set the error flag and return
            return;
        }
        //Take the packet length in bytes
        tx_increase_ptr ( s,&ptr );
        pci_dma_read(&s->parent_obj, tx_buf_addr ( s,ptr ),&len_in_bytes,4);
        len_in_u32 = ( len_in_bytes+3 ) >>2;
        tx_increase_ptr ( s,&ptr );
        //Take the flags
        pci_dma_read(&s->parent_obj, tx_buf_addr ( s,ptr ),&flags,4);
        //Mark the packet as serviced
        {
            //To be done!!!
        }
        tx_increase_ptr ( s,&ptr );
        //Now the pointer points to the first word containing the packet data
        //Send the packet
        {
            //Allocate temporary buffer on stack (hmmm... is it safe for long buffers?)
            uint32_t pkt_buf[len_in_u32];
            int len = len_in_bytes;
            int i=0;
            while (len>0) {
                //Copy the whole world
                pci_dma_read(&s->parent_obj, tx_buf_addr ( s,ptr ),&pkt_buf[i],4);
                tx_increase_ptr ( s,&ptr );
                len -= 4;
                i++;
            }
            if(0){
	      int i;
	      printf("send: ");
	      for(i=0;i<10;i++) printf("%2.2x ",(int) pkt_buf[i]);
	      printf("\n");
	    }
            qemu_send_packet ( s->nic->ncs, (uint8_t *) pkt_buf, len_in_bytes);
        }
        /*
        * The code below could be nice and efficient, but requires
        * use of cpu_physical_memory_map,
        //Send the packet using the iovec
        //Calculate the number of iovecs needed to send
        niov = ( 4*ptr.offset + len_in_bytes + 0xfff ) >>12;
        {
            struct iovec iov[niov]; //Use the GCC extension - dynamic allocation on stack
            //Fill the iovec structures
            int bytes_left = len_in_bytes;
            int offs=ptr.offset;
            int i;
            for ( i=0;i<niov;i++ )
            {
                iov[i].iov_base = s->regs.r.TxPages[ ( ptr.page+i ) % WZNIC1_NOF_TX_PAGES].PhysAddr+offs;
                iov[i].iov_len = min ( 4096-offs,bytes_left );
                bytes_left -= min ( 4096-offs,bytes_left );
                offs=0;
            }
            //After this loop our packet is ready, so transmit it!
            qemu_sendv_packet ( &s->nic->nc, iov, niov );
            //Finally update interrupts
            wz_nic1_update_irq ( s );
        }
        */
        qemu_mutex_lock(&s->lock);
        memcpy ( &s->Tx_Tail,&ptr,sizeof ( ptr ) );
        qemu_mutex_unlock(&s->lock);
        wz_nic1_update_irq(s);
    }
    return;
}

static int wz_nic1_buffer_full ( WzNic1State *s )
{
    //Calculate the amount of free buffer

    if ( rx_buf_space(s) <= MAX_ETH_FRAME_SIZE+20 )
        return 1;
    return 0;
}

static int wz_nic1_can_receive ( NetClientState *nc )
{
    WzNic1State *s = qemu_get_nic_opaque(nc);
    if (s->Rx_Enabled==0) return 0;
    return !wz_nic1_buffer_full ( s );
}

static ssize_t wz_nic1_receive ( NetClientState *nc, const uint8_t *buf, size_t size )
{
    WzNic1State *s = qemu_get_nic_opaque(nc);

    if ( !wz_nic1_can_receive ( nc ) )
        return -1;
#ifdef DEBUG_wzab1
    printf ( "wz_nic1: receiving len=%zu\n", size );
#endif

    /* Just accept everything. */
    {
        WzNic1Ptr ptr;
        int bptr;
        size_t tmp_size;
        uint32_t tmp;
        int i;
        memcpy ( &ptr,&s->Rx_Head,sizeof ( ptr ) );
        bptr = rx_buf_addr ( s, ptr ); //To jest adres w maszynie wirtualnej!!!
        tmp = WZNIC1_PKT_MAGIC_NUMBER;
        pci_dma_write(&s->parent_obj,bptr,&tmp,4);
        /* Write packet header */
        //We are already sure, that the packet will fit
        rx_increase_ptr ( s,&ptr );
        bptr = rx_buf_addr (s, ptr );
        pci_dma_write(&s->parent_obj,bptr,&size,4); //Length of the packet
        rx_increase_ptr ( s, &ptr );
        bptr = rx_buf_addr (s, ptr );
        tmp = 0 ; //Flags - @@ to be corrected!
        pci_dma_write(&s->parent_obj,bptr,&tmp,4); //Flags of the packet
        rx_increase_ptr ( s, &ptr );
        //Now we can copy the data
        tmp_size = size;
        while ( tmp_size >= 4 )
        {
            bptr = rx_buf_addr (s, ptr );
            pci_dma_write(&s->parent_obj,bptr,buf,4); //Contents of the packet
            rx_increase_ptr ( s, &ptr );
            tmp_size -= 4;
            buf+=4;
        }
        //Last part of the buffer must be copied byte by byte...
        //We are working on a LE platform, so we can handle it as follows:
        if (tmp_size) {
            //First we have to check if such "remainder" exists...
            tmp = 0;
            i=0;
            while (tmp_size) {
                tmp |= ((uint32_t) *buf) << (8*i);
                i++;
                buf++;
                tmp_size--;
            }
            bptr = rx_buf_addr (s, ptr );
            pci_dma_write(&s->parent_obj,bptr,&tmp,4); //Rest of the content of the packet
            //OK. The whole packet is stored, so now we have to update the pointer
            rx_increase_ptr ( s, &ptr );
        }
        qemu_mutex_lock(&s->lock);
        memcpy ( &s->Rx_Head, &ptr, sizeof ( ptr ) );
        qemu_mutex_unlock(&s->lock);
    }
    wz_nic1_update_irq( s );
    return size;
}


/* called for write accesses to our register memory area */
static void pci_wz_nic1_write ( void *opaque, hwaddr addr, uint64_t val, unsigned size )
{
    WzNic1State *s = opaque;
    int i_addr;
#ifdef DEBUG_wzab1
    printf ( "wzab1: zapis pod adres = 0x%08x, 0x%08x\n", ( unsigned int ) addr, val );
#endif
    /* Check which register is accessed */
    if ( addr>=sizeof ( WzNic1Regs ) )
    {
        //Write above registers area - ignore it!
    }
    else
    {
        i_addr = addr/4;
        //Write the value
        s->regs.u32[i_addr]=val;
        //Interprete its special meaning
        if (addr==offsetof (WzNic1Regs,RxTail))
        {
            uint32_t tmp;
            qemu_mutex_lock(&s->lock);
            s->Rx_Tail.offset=val & 0x3ff;
            tmp=val >> 10;
            if (tmp >= WZNIC1_NOF_RX_PAGES) tmp = 0;
            s->Rx_Tail.page = tmp;
            qemu_mutex_unlock(&s->lock);
        }
        if (addr==offsetof (WzNic1Regs,TxHead))
        {
            uint32_t tmp;
            qemu_mutex_lock(&s->lock);
            s->Tx_Head.offset=val & 0x3ff;
            tmp=val >> 10;
            if (tmp >= WZNIC1_NOF_TX_PAGES) tmp = 0;
            s->Tx_Head.page = tmp;
            qemu_mutex_unlock(&s->lock);
        }
        if ( addr==offsetof ( WzNic1Regs,Ctrl ) )
        {
            //Write to control register - we need to check what operation is required
            switch ( val )
            {
            case NIC1_CMD_ENA_TX_IRQ:
                s->Tx_Irq_Enabled = 1;
                break;
            case NIC1_CMD_DIS_TX_IRQ:
                s->Tx_Irq_Enabled = 0;
                break;
            case NIC1_CMD_ENA_RX_IRQ:
                s->Rx_Irq_Enabled = 1;
                break;
            case NIC1_CMD_DIS_RX_IRQ:
                s->Rx_Irq_Enabled = 0;
                break;
            case NIC1_CMD_ENA_RX:
                s->Rx_Enabled = 1;
                break;
            case NIC1_CMD_DIS_RX:
                s->Rx_Enabled = 0;
                break;
            case NIC1_CMD_SEND:
                wz_nic1_send_packets(s);
                break;
            case NIC1_CMD_CLEAR:
                s->Rx_Enabled = 0;
                s->TxCorruptedBuffer = 0;
                s->Error = 0;
                s->Rx_Tail.offset=0;
                s->Rx_Tail.page=0;
                s->Rx_Head.offset=0;
                s->Rx_Head.page=0;
                s->Tx_Tail.offset=0;
                s->Tx_Tail.page=0;
                s->Tx_Head.offset=0;
                s->Tx_Head.page=0;
                break;
            case NIC1_CMD_ENA_PROMISCOUS:
                s->promiscous = 1;
                break;
            case NIC1_CMD_DIS_PROMISCOUS:
                s->promiscous = 0;
                break;

            }
        }
    }
    wz_nic1_update_irq(s);
}

/* The procedure below performs the real encryption, after simulated processing time is expired */
static void wzab1_tick ( void *opaque )
{
    //WzNic1State * s = opaque;
    //Encrypt the data
    {
    }
#ifdef DEBUG_wzab1
    printf ( "Data processed - request IRQ!\n" );
#endif

}

//Sorry, but the state description below is not complete!
//You are free to fix it!
static const VMStateDescription vmstate_wz_nic1 =
{
    .name = "wz_nic1",
    .version_id = 2,
    .minimum_version_id = 2,
    .minimum_version_id_old = 2,
    .fields      = ( VMStateField [] )
    {
        VMSTATE_PCI_DEVICE ( parent_obj, WzNic1State ),
        VMSTATE_TIMER_PTR ( timer,WzNic1State ),
        VMSTATE_END_OF_LIST()
    }
};

static void wz_nic1_on_reset ( void *opaque )
{
    WzNic1State *s = PCI_WZNIC1(opaque);
    wz_nic1_reset ( s );
}

static int pci_wz_nic1_init ( PCIDevice *dev )
{
    WzNic1State *s = PCI_WZNIC1 ( dev );
    uint8_t *c = s->parent_obj.config;
    s->Rx_Irq_Enabled=0;
    s->Tx_Irq_Enabled=0;
    s->Rx_Enabled=0;
    s->Rx_Head.offset=0;
    s->Rx_Head.page=0;
    s->Rx_Tail.offset=0;
    s->Rx_Tail.page=0;
    s->Tx_Head.offset=0;
    s->Tx_Head.page=0;
    s->Tx_Tail.offset=0;
    s->Tx_Tail.page=0;
    s->Error=0;
    s->TxCorruptedBuffer=0;
    qemu_mutex_init(&s->lock);

    /* TODO: RST# value should be 0. */
    c[PCI_INTERRUPT_PIN] = 1;
    //Register memory mapped registers
    memory_region_init_io(&s->mmio,OBJECT(s),&pci_wz_enc1_mmio_ops,s,
                        "pci-wznic1-mmio", 0x100); //@@sizeof(s->regs.u32));
    pci_register_bar (&s->parent_obj, 0,  PCI_BASE_ADDRESS_SPACE_MEMORY,&s->mmio);
    qemu_register_reset ( wz_nic1_on_reset, s );
    qemu_macaddr_default_if_unset(&s->conf.macaddr);
    s->nic = qemu_new_nic(&wz_nic1_nc_info, &s->conf,
                            object_get_typename(OBJECT(&dev->qdev)), dev->qdev.id, s);
    qemu_format_nic_info_str(qemu_get_queue(s->nic), s->conf.macaddr.a);
    //Register timer used to simulate processing time
    s->timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, wzab1_tick, s );
    wz_nic1_reset ( s );
    return 0;
}

void wz_nic1_update_irq(WzNic1State * s)
{
    //Check Tx interrupt
    s->irq_pending = 0;
    qemu_mutex_lock(&s->lock);
    if ((s->Tx_Head.page==s->Tx_Tail.page) && (s->Tx_Head.offset==s->Tx_Tail.offset))
        s->tx_irq_pending = 1;
    else
        s->tx_irq_pending = 0;
    qemu_mutex_unlock(&s->lock);
    //Check Rx interrupt
    qemu_mutex_lock(&s->lock);
    if ((s->Rx_Head.page==s->Rx_Tail.page) && (s->Rx_Head.offset==s->Rx_Tail.offset))
        s->rx_irq_pending = 0;
    else
        s->rx_irq_pending = 1;
    qemu_mutex_unlock(&s->lock);
    if (s->Rx_Irq_Enabled && s->rx_irq_pending)
        s->irq_pending |= 1;
    if (s->Tx_Irq_Enabled && s->tx_irq_pending)
        s->irq_pending |= 1;
    if (s->irq_pending)
        pci_irq_assert(&s->parent_obj);
    else
        pci_irq_deassert(&s->parent_obj);
}

static void wz_nic1_cleanup(NetClientState *nc)
{
    struct WzNic1State *s = qemu_get_nic_opaque(nc);

    s->nic = NULL;
}

static NetClientInfo wz_nic1_nc_info = {
    .type = NET_CLIENT_OPTIONS_KIND_NIC,
    .size = sizeof(NICState),
    .can_receive = wz_nic1_can_receive,
    .receive = wz_nic1_receive,
    .cleanup = wz_nic1_cleanup,
};

static void
pci_wz_nic1_uninit(PCIDevice *dev)
{
    WzNic1State *d = PCI_WZNIC1(dev);
    wz_nic1_reset(d);
    timer_free(d->timer);
}

static void qdev_pci_wznic1_reset(DeviceState *dev)
{
    WzNic1State *d = PCI_WZNIC1(dev);
    wz_nic1_reset(d);
}

static Property wznic1_properties[] = {
    DEFINE_NIC_PROPERTIES(WzNic1State, conf),
    DEFINE_PROP_END_OF_LIST(),
};

static void pci_wznic1_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    PCIDeviceClass *k = PCI_DEVICE_CLASS(klass);

    k->init = pci_wz_nic1_init;
    k->exit = pci_wz_nic1_uninit;
    k->vendor_id = PCI_VENDOR_ID_WZAB;
    k->device_id = PCI_DEVICE_ID_WZAB_WZNIC1;
    k->revision = 0x00;
    k->class_id = PCI_CLASS_NETWORK_ETHERNET;
    dc->desc = "PCI demo Ethernet card";
    set_bit(DEVICE_CATEGORY_NETWORK, dc->categories);
    dc->reset = qdev_pci_wznic1_reset;
    dc->vmsd = &vmstate_wz_nic1;
    dc->props = wznic1_properties;
}

static const TypeInfo pci_wznic1_info = {
    .name          = TYPE_PCI_WZNIC1,
    .parent        = TYPE_PCI_DEVICE,
    .instance_size = sizeof(WzNic1State),
    .class_init    = pci_wznic1_class_init,
};

static void pci_wznic1_register_types(void)
{
    type_register_static(&pci_wznic1_info);
}

type_init(pci_wznic1_register_types)


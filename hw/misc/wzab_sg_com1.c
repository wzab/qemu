/* This modell is not usable yet! It is currently under development. */

/*
 * QEMU - Model of the device providing the trivial communication, using the
 * scatter-gather BM DMA.
 *
 * When the DMA transfer is started, the model creates a new file,
 * and puts all received data into it.
 *
 * How to create a file with the next number?
 * In the first approach we simply use the current directory.
 * We create files with numbers starting from 0 (so we will overwrite
 * files created in the previous session if any).
 *
 * Copyright Wojciech M. Zabolotny ( wzab@ise.pw.edu.pl ) 2011-2019
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

#define DEBUG_wzab1 1

//PCI IDs below are not registred! Use only for experiments!
#define PCI_VENDOR_ID_WZAB 0xabba
#define PCI_DEVICE_ID_WZAB_WZCOM1 0x0125

//Simulated processing time
#define COM1_PROCESSING_TIME 1000
#include "qemu/osdep.h"
#include "qemu/compiler.h"
#include <string.h>
#include <inttypes.h>
#include "hw/sysbus.h"
#include "hw/hw.h"
#include "hw/pci/pci.h"
#include <mcrypt.h>
#include "qemu/timer.h"
#include "wzab_sg_com1.h"
#include <stdio.h>
/*
  Description of functionalities.
  The device simply takes the SG buffer described by the descriptors organized in a list:
  The list of descriptors may contain single buffer descriptors or array of descriptors.

  Offset   - Meaning
  0          Number of buffers
  1          Next descriptor
  2          Table of buffers

  Table of buffers contains entries of form:
  Offset   - Meaning
  0          Base address
  1          Length
  2          Transferred length
  3          Control (R/W?, Interrupt after transfer?)
  4          Status

  The emulated device connects to the socket and transmitts the data to be written
  or receives the data to be read.
*/

//Some prototypes...
static uint64_t pci_wz_com1_read(void *opaque, hwaddr addr, unsigned size);
static void pci_wz_com1_write(void *opaque, hwaddr addr, uint64_t val, unsigned size);
int wz_com1_init (PCIBus *bus);

typedef struct WzCom1State
{
    /*<private>*/
    PCIDevice parent_obj;
    /*<public>*/
    MemoryRegion mmio;
    union
    {
        WzCom1Regs r;
        com1_addr_t ureg[sizeof(WzCom1Regs)/sizeof(com1_addr_t)];
    } regs;
    com1_addr_t wz_com1_mmio_io_addr;
    uint8_t irq_pending;
    uint8_t irq_mask;
    uint8_t Error;
    uint8_t Working;
    QEMUTimer * timer;
} WzCom1State;

#define TYPE_PCI_WZCOM1 "pci-wzcom1"

#define PCI_WZCOM1(obj) \
    OBJECT_CHECK(WzCom1State, (obj), TYPE_PCI_WZCOM1)
static const MemoryRegionOps pci_wz_com1_mmio_ops =
{
    .read = pci_wz_com1_read,
    .write = pci_wz_com1_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .impl = {
        .min_access_size = 8, //Always 32-bit access!!
        .max_access_size = 8,
    },
};


static void wz_com1_reset (void * opaque)
{
    WzCom1State *s = opaque;
    memset((void *)s->regs.ureg,0,sizeof(s->regs.ureg));
#ifdef DEBUG_wzab1
    printf("wzab_com1 reset!\n");
#endif
}

/* called for read accesses to our register memory area */
static uint64_t pci_wz_com1_read(void *opaque, hwaddr addr, unsigned size)
{
    int i_addr;
#ifdef DEBUG_wzab1
    printf("Memory read: address %x\n ", (unsigned int) addr);
#endif
    WzCom1State *s = opaque;
    if(addr>=sizeof(WzCom1Regs))
    {
        //Read above registers area - return "0xbada4ea" to show it
        return 0xbada4ea;
    }
    else if (addr == offsetof(WzCom1Regs, Status))
    {
        //Reading of the Status - synthesize the correct value
        com1_addr_t res = s->Error & 0xffff ; //Lower 16 bits!
        if (s->Working)
            res |= 0x1000;
        if (s->irq_pending)
            res |= 0x8000;
        if (s->irq_mask)
            res |= 0x4000;
#ifdef DEBUG_wzab1
        printf("Status=%016" PRIu64" \n",res);
#endif
        return res;
    }
    else if (addr == offsetof(WzCom1Regs, Id)) {
        return 0x32abd3a1;
    }
    else
    {
        i_addr = addr/sizeof(com1_addr_t);
        //Read the register
        return s->regs.ureg[i_addr];
    }
}


/* The function that performs the DMA transfer */
static void wz_com1_do_dma(void *opaque)
{
    WzCom1State * s = opaque;
    //If the first descriptor is NULL, return
    if(!s->regs.r.FirstDesc)
        return;
    //Open or create the file for writing
    FILE * fout = fopen("dma_out.bin","w");
    //Set the run flag
    s->Working = 1;
    //Read the first descriptor
    s->regs.r.CurDesc = s->regs.r.FirstDesc;
    while(1)
    {
        //Here we start processing of the current descriptor
        com1_addr_t n_entries, next;
        //Read the number of entries
        pci_dma_read(&s->parent_obj,s->regs.r.CurDesc+offsetof(Com1Desc, Length), &next, sizeof(com1_addr_t));
        //Read the address of the next descriptor
        pci_dma_read(&s->parent_obj,s->regs.r.CurDesc+offsetof(Com1Desc, Next), &n_entries, sizeof(com1_addr_t));
        com1_addr_t bufs = s->regs.r.CurDesc+offsetof(Com1Desc, Descs);
        //Start processing of subsequent buffers in the descriptor
        for(int i=0; i<n_entries; i++)
        {
            BufDesc cur_buf;
            pci_dma_read(&s->parent_obj, bufs+i*sizeof(BufDesc), &cur_buf, sizeof(cur_buf));
            //Now we receive the data from the buffer
            for(int j=0; j < cur_buf.Length; j++)
            {
                //It is inefficient, but simple - let's copy it byte by byte.
                uint8_t b;
                pci_dma_read(&s->parent_obj, cur_buf.BufAddr+cur_buf.Offset+j,&b,1);
                fwrite(&b,1,1,fout);
            }
        }
        //All entries serviced, let's go to the next descriptor;
        if(!next)
        {
            //That was the last descriptor
            fclose(fout);
            break;
        }
        else
        {
            s->regs.r.CurDesc = next;
        }
    }
    //Signal that the work is done and raise the IRQ
    s->Working = 0;
    s->irq_pending = 1;
    if (!s->irq_mask)
        pci_irq_assert(&(s->parent_obj));
    return;
}

static void pci_wz_com1_write(void *opaque, hwaddr addr, uint64_t val, unsigned size)
{
    WzCom1State *s = opaque;
    int i_addr;
#ifdef DEBUG_wzab1
    printf("wzab1: zapis pod adres = 0x%016" PRIu64 " , 0x%016" PRIu64 "\n", (uint64_t) addr, val);
#endif
    /* Check which register is accessed */
    if(addr>=sizeof(WzCom1Regs))
    {
        //Write above registers area - ignore it!
    }
    else
    {
        i_addr = addr/sizeof(com1_addr_t);
        //Write the value
        s->regs.ureg[i_addr]=val;
        //Interprete its special meaning
        if (addr==offsetof(WzCom1Regs,Ctrl))
        {
            //Write to control register - we need to check what operation is required
            switch(val)
            {
            case COM1_CMD_START:
                //End of encryption!
                s->irq_pending = 0;
                pci_irq_deassert(&(s->parent_obj));
                //Start processing in a separate thread via timer
                timer_mod_anticipate(s->timer,qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL)+COM1_PROCESSING_TIME);
                break;
            case COM1_CMD_ACKIRQ:
                //Confirm reception of interrupt (and unmask it)
                s->irq_pending = 0;
                s->irq_mask = 0;
                pci_irq_deassert(&(s->parent_obj));
                break;
            case COM1_CMD_DISIRQ:
                //Mask interrupt
                s->irq_mask = 1;
                pci_irq_deassert(&(s->parent_obj));
                break;
            case COM1_CMD_ENAIRQ:
                //Unmask interrupt
                s->irq_mask = 0;
                if(s->irq_pending)
                    pci_irq_assert(&(s->parent_obj));
                break;
            }
        }
    }
}

static void pci_wz_com1_realize (PCIDevice *pdev, Error **errp)
{
    WzCom1State *s = PCI_WZCOM1(pdev);
    uint8_t *c = s->parent_obj.config;

    /* TODO: RST# value should be 0. */
    c[PCI_INTERRUPT_PIN] = 1;
    //Register memory mapped registers
    memory_region_init_io(&s->mmio,OBJECT(s),&pci_wz_com1_mmio_ops,s,
                          "pci-wzcom1-mmio", 0x100); //@@sizeof(s->regs.u32));
    pci_register_bar (&s->parent_obj, 0,  PCI_BASE_ADDRESS_SPACE_MEMORY,&s->mmio);
    qemu_register_reset (wz_com1_reset, s);
    //Register timer used to simulate processing time
    s->timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, wz_com1_do_dma, s);
    wz_com1_reset (s);
    //return 0;
}

static void pci_wz_com1_init(Object *obj)
{
}
static void
pci_wz_sg_com1_uninit(PCIDevice *dev)
{
    WzCom1State *d = PCI_WZCOM1(dev);

    wz_com1_reset(d);
    timer_free(d->timer);
}

static void qdev_pci_wz_com1_reset(DeviceState *dev)
{
    WzCom1State *d = PCI_WZCOM1(dev);
    wz_com1_reset(d);
}

static void pci_wz_com1_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    PCIDeviceClass *k = PCI_DEVICE_CLASS(klass);
    k->realize = pci_wz_com1_realize;
    k->exit = pci_wz_sg_com1_uninit;
    k->vendor_id = PCI_VENDOR_ID_WZAB;
    k->device_id = PCI_DEVICE_ID_WZAB_WZCOM1;
    k->revision = 0x00;
    k->class_id = PCI_CLASS_OTHERS;
    dc->desc = "PCI SG DMA demonstrator";
    set_bit(DEVICE_CATEGORY_MISC, dc->categories);
    dc->reset = qdev_pci_wz_com1_reset;
}

static void pci_wz_com1_register_types(void)
{
    static InterfaceInfo interfaces[] =
    {
        { INTERFACE_CONVENTIONAL_PCI_DEVICE },
        { },
    };
    static const TypeInfo pci_wz_com1_info =
    {
        .name          = TYPE_PCI_WZCOM1,
        .parent        = TYPE_PCI_DEVICE,
        .instance_size = sizeof(WzCom1State),
        .instance_init = pci_wz_com1_init,
        .class_init    = pci_wz_com1_class_init,
        .interfaces = interfaces,
    };
    type_register_static(&pci_wz_com1_info);
}

type_init(pci_wz_com1_register_types)


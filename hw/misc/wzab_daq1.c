/*
 * Emulation of the FPGA-based DAQ engine for QEMU
 *
 * Copyright Wojciech M. Zabolotny ( wzab@ise.pw.edu.pl ) 2021
 *
 * The code below implements a DAQ DMA engine, that writes the data to the
 * memory buffers, using the DMA.
 * It is assumed, that the memory buffers are based on hugepages
 * with size 2MB (maybe it should be parameterized?).
 * The 64-bit base addresses of the hugepages are stored in the block
 * memory in the FPGA. Here it is modeled as an "mem_bufs" array.
 * (should they really be 64-bit? the lowest 21 bits are always zero!
 * so we have only 43 bits. Is there any additional limit on the
 * upper bits?)
 *
 * Except of this, we need a control register, that holds configuration
 * information.
 * o) Number of memory buffers
 * o) Whether the acquisition is started or not
 * Additionally we need a runtime information
 * o) One huge pages stores a circular buffer for event descriptors - as 256 bit structures.
 *      => we need a special register for that!
 * o) Other huge pages are in a block memory - 2^N buffers
 * o) We receive the data until we get "tlast". After that we have to store the event.
 *    (how we can simulate that in QEMU?)
 *
 * So what must be in the "runtime information"?
 * o) Current write position - number of the HP page and offset in it.
 * o) Start position of the current event
 * o) Last unread position - number of the HP page and offset to it.
 *
 * Additionally we need a control ans status registers.
 * o) In control register we only start the engine.
 * o) In the status register we only stop it.
 *
 * ONE IMPORTANT REMARK:
 * The addresses of the huge pages are given as byte addresses.
 * All indexes and masks used to count word positions are as 64-bit word addresses

 * The QEMU model must contain also the emulated data source and its configuration.
 * Therefore there will be a few additional registers.
 *
 *
 * The code was written by Wojciech M. Zabolotny (wzab<at>ise.pw.edu.pl)
 * in March and April 2021, however it is significantly based on different
 * source codes provided in the QEMU sources.
 * Therefore I leave the original license:
 *
 * Copyright (c) 2003 Fabrice Bellard
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
 *
 * The device is connected to the PCI bus - it's memory and interrupt resources are
 * reported to the host using the standard PC mechanisms.
 */

#define DEBUG_wzab1 1
//PCI IDs below are not officially registered! Use only for experiments!
#define PCI_VENDOR_ID_WZAB 0xabba
#define PCI_DEVICE_ID_WZAB_WZDAQ1 0x3342
#include "qemu/osdep.h"
#include <inttypes.h>
#include "qemu/compiler.h"
#include <string.h>
#include "hw/sysbus.h"
#include "hw/hw.h"
#include "hw/pci/pci.h"
#include "hw/pci/msi.h"
#include "qemu/timer.h"
#include <string.h>
#include <endian.h>
#include <czmq.h>
#include "wzab_daq1.h"

static uint64_t pci_wzdaq1_read(void *opaque, hwaddr addr, unsigned size);
static void pci_wzdaq1_write(void *opaque, hwaddr addr, uint64_t val, unsigned size);

typedef struct WzDaq1State {
    PCIDevice pdev;
    MemoryRegion mmio;
    uint64_t regs[DAQ1_REGS_NUM];
    uint64_t write_ptr, read_ptr, ptr_mask;
    uint64_t hpage_size, hpage_mask;
    int hpage_shift;
    int nof_hp;
    uint64_t buf_hps[DAQ1_NBUFS]; //Addresses of HPs creating the circular buffer
    uint64_t evt_write_ptr, evt_read_ptr, evt_ptr_mask;
    uint64_t evt_hp; //Address of HP keeping the event descriptors
    uint64_t evt_num; //Number of the event
    uint32_t error;
    uint32_t running;
    uint32_t irq_pending;
    uint32_t irq_enabled;
    //QEMUTimer * daq_timer;
    QemuThread thread;
} WzDaq1State;

#define TYPE_PCI_WZDAQ1 "pci-wzdaq1"

#define PCI_WZDAQ1(obj) OBJECT_CHECK(WzDaq1State, obj, TYPE_PCI_WZDAQ1)

static const MemoryRegionOps pci_wzdaq1_mmio_ops = {
    .read = pci_wzdaq1_read,
    .write = pci_wzdaq1_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .impl = {
        .min_access_size = 8, //Always 64-bit access!!
        .max_access_size = 8,
    },
    .valid = {
        .min_access_size = 8, //Always 64-bit access!!
        .max_access_size = 8,
    },
};

/*
  static bool wzdaq1_msi_enabled(WzDaq1State * s)
  {
  return msi_enabled(&s->pdev);
  }
*/

static void wzdaq1_reset (void * opaque)
{
    WzDaq1State *s = opaque;
    memset(s->regs,0,sizeof(s->regs));
#ifdef DEBUG_wzab1
    printf("wzdaq1 reset!\n");
#endif
}

/* called for read accesses to our register memory area */
static uint64_t pci_wzdaq1_read(void *opaque, hwaddr addr, unsigned size)
{
#ifdef DEBUG_wzab1
    printf("Memory read: address %" PRIx64 "\n", (uint64_t) addr);
#endif
    WzDaq1State *s = opaque;
    uint64_t ret=0xbada4ea55aa55aa; //Special value returned when accessed non-existing register
    addr = addr/8;
    //Special cases
    if(addr==DAQ1_READP) {
        ret = s->read_ptr;
#ifdef DEBUG_wzab1
        printf(" value %"PRIx64"\n",ret);
#endif
        return ret;
    }
    if(addr==DAQ1_WRITEP) {
        ret = s->write_ptr;
#ifdef DEBUG_wzab1
        printf(" value %"PRIx64"\n",ret);
#endif
        return ret;
    }
    if(addr==DAQ1_NOF_HP) {
        ret = s->nof_hp;
#ifdef DEBUG_wzab1
        printf(" value %"PRIx64"\n",ret);
#endif
        return ret;
    }
    if(addr==DAQ1_EVT_WRITEP) {
        ret = s->evt_write_ptr;
#ifdef DEBUG_wzab1
        printf(" value %"PRIx64"\n",ret);
#endif
        return ret;
    }
    if(addr==DAQ1_EVT_READP) {
        ret = s->evt_read_ptr;
#ifdef DEBUG_wzab1
        printf(" value %"PRIx64"\n",ret);
#endif
        return ret;
    }
    if(addr==DAQ1_STAT) {
        ret = 0;
        if(s->error)
            ret |= 1;
#ifdef DEBUG_wzab1
        printf(" value %"PRIx64"\n",ret);
#endif
        return ret;
    }
    //normal case
    if(addr<DAQ1_REGS_NUM) {
        ret = s->regs[addr];
#ifdef DEBUG_wzab1
        printf(" value %"PRIx64"\n",ret);
#endif
        return ret;
    }
    if (( addr >= DAQ1_NBUFS ) && ( addr < 2 * DAQ1_NBUFS )) {
        // Write the hugepage address to the buffer
        ret = s->buf_hps[addr-DAQ1_NBUFS];
        return ret;
    }

    return ret;
}


/* called for write accesses to our register memory area */
void pci_wzdaq1_write(void *opaque, hwaddr addr, uint64_t val, unsigned size)
{
    WzDaq1State *s = opaque;
#ifdef DEBUG_wzab1
    printf("wzab1: zapis pod adres = 0x%016" PRIx64 ", 0x%016" PRIx64 "\n", addr, val);
#endif
    /* convert to wzdaq1 memory offset */
    addr = (addr/8);
    /* always write value to the shadow register */
    if(addr<DAQ1_REGS_NUM) {
        s->regs[addr]=val;
        /* for certain registers take additional actions */
        switch(addr) {
        case DAQ1_READP:
            s->read_ptr = val;
            break;
        case DAQ1_WRITEP:
            s->write_ptr = val;
            break;
        case DAQ1_EVT_READP:
            s->evt_read_ptr = val;
            break;
        case DAQ1_EVT_WRITEP:
            s->evt_write_ptr = val;
            break;
        case DAQ1_NOF_HP:
            s->nof_hp = val;
            s->ptr_mask = ( s->hpage_size * val ) - 1;
            break;
        case DAQ1_HPSHFT:
            s->hpage_shift = val;
            s->hpage_size = ((uint64_t) 1) << val;
            s->hpage_mask = s->hpage_size - 1;
            s->evt_ptr_mask = s->hpage_size - 1;
            break;
        case DAQ1_CTRL:
            if(val == DAQ1_CMD_STOP) {
                //Stop the engine
                s->running = 0;
            }
            if(val == DAQ1_CMD_START) {
                int i;
                uint64_t zero = 0;
                //Start the engine
                //Zero the 0th event
                for(i=0; i<DAQ1_EVT_DESC_SIZE; i++)
                    pci_dma_write(&s->pdev,s->evt_hp+8*i, &zero, sizeof(zero));
                s->read_ptr = 0;
                s->write_ptr = 0;
                s->evt_read_ptr = 0;
                s->evt_write_ptr = 0;
                s->error = 0;
                s->running = 1;
            }
            if(val == DAQ1_CMD_DIS_IRQ) {
                //Switch the IRQ off
                s->irq_enabled = 0;
                pci_irq_deassert(&s->pdev);
            }
            if(val == DAQ1_CMD_ENA_IRQ) {
                s->irq_enabled = 1;
                //Rise the IRQ if it's pending
                if(s->irq_pending) pci_irq_assert(&s->pdev);
            }
            if(val == DAQ1_CMD_CONFIRM) {
                if(s->evt_read_ptr == s->evt_write_ptr) {
                    //Nothing to confirm!
                    return;
                }
                s->evt_read_ptr = (s->evt_read_ptr + DAQ1_EVT_DESC_SIZE) & s->evt_ptr_mask;
                //Shouldn't it be protected with a mutex?
                if(s->evt_read_ptr == s->evt_write_ptr) {
                    s->irq_pending = 0;
                    pci_irq_deassert(&s->pdev);
                }
            }
            break;
        case DAQ1_EVTS:
            s->evt_hp = val;
            s->evt_write_ptr = 0;
            s->evt_read_ptr = 0;
        default:
            return;
            break;
        }
    } else if (( addr >= DAQ1_NBUFS ) && ( addr < 2 * DAQ1_NBUFS )) {
        // Write the hugepage address to the buffer
        s->buf_hps[addr-DAQ1_NBUFS] = val;
    }
}

/* The procedure closes the current data segment.
 * Currently we write only the address of the last word
 * in the circular buffer.
 *
 * However, it creates one problem. I don't want the kernel
 * to access the descriptors area (I don't have kernel mapping for it).
 * How can I check in the kernel if the event is ready then?
 * If I advance the write pointer when completing the previous event,
 * I'll get an error, if the next event is not processed yet.
 * However, that's OK as long, as I don't try to write a new event.
 *
 * I could encode the state of the event in the least significant bit of the
 * write pointer. 0 - the event is being written, 1 - the event is completed.
 *
 * So now I simply call start_segment immediately after end_segment...
 */

static int end_segment(WzDaq1State * s)
{
    uint64_t status = 1;
    //Address of the currently written event description
    uint64_t daddr = s->evt_hp + s->evt_write_ptr *8 ;
    pci_dma_write(&s->pdev,daddr+8*DAQ1_EVT_AFTER_LAST, &s->write_ptr, sizeof(s->write_ptr));
    pci_dma_write(&s->pdev,daddr+8*DAQ1_EVT_STATUS, &status, sizeof(status));
    return 0;
}

/* The procedure starts the new data segment.
 * Currently we zero the segment descriptor, then
 * we write the segment number, and the position of the
 * first word in the circular buffer.
 *
 * What wrong may happen in that process?
 * We may be not able to start the new segment.
 * In that case we have to ignore the whole segment.
 * We should set the appropriate bit in the status register.
 *
 */

static int start_segment(WzDaq1State * s)
{
    int i;
    uint64_t zero = 0;
    uint64_t new_evt_ptr = (s->evt_write_ptr + DAQ1_EVT_DESC_SIZE) & s->evt_ptr_mask;
    if(new_evt_ptr == s->evt_read_ptr) {
        //All segments are occupied, ignore that one and set the proper flag...
        //That flag should also block reception of data (as they can't be assigned
        //to the correct segment!
        //We stop the acquisition and set the error flag!
        s->running = 0;
        s->error = 1;
        s->irq_pending = 1;
        if(s->irq_enabled) {
            pci_irq_assert(&s->pdev);
        }
    } else {
        uint64_t daddr = s->evt_hp + new_evt_ptr *8 ;
        //Zero the segment
        for(i=0; i<DAQ1_EVT_DESC_SIZE; i++)
            pci_dma_write(&s->pdev,daddr+8*i, &zero, sizeof(zero));
        //Now set the current event number;
        pci_dma_write(&s->pdev,daddr+8*DAQ1_EVT_NUM, &s->evt_num, sizeof(s->evt_num++));
        //Now set the current write position
        pci_dma_write(&s->pdev,daddr+8*DAQ1_EVT_FIRST, &s->write_ptr, sizeof(s->write_ptr));
        s->evt_write_ptr = new_evt_ptr;
    }
    return 0;
}

/* The procedure that adds words to the cyclic buffer */
static int add_words(WzDaq1State * s, uint64_t * wbuf, int nwords)
{
    //Check how much space do we have in the buffer
    uint64_t write_pos;
    uint64_t npage;
    uint64_t page_ofs;
    uint64_t to_write;
    int res __attribute__((unused));
    uint64_t wfree = (s->read_ptr - s->write_ptr - 1) & s->ptr_mask;
    if (nwords > wfree) {
        // We should somehow signal that we had to reject some data?
        nwords = wfree;
    }
    //We assume that this is an atomic operation
    write_pos = s->write_ptr;
    while(nwords > 0) {
        npage = write_pos >> s->hpage_shift;
        page_ofs = write_pos & s->hpage_mask;
        to_write = s->hpage_size - page_ofs;
        if(to_write > nwords)
            to_write = nwords;
        // Now we should calculate how many words we can write at once, and write it
        // as a block transfer!
        res = pci_dma_write(&s->pdev,s->buf_hps[npage]+8*page_ofs, wbuf, 8*to_write);
        write_pos += to_write;
        nwords -= to_write;
        wbuf += to_write;
    }
    // We assume that this is an atomic operation
    s->write_ptr = write_pos;
    return 1;
}

/* That procedure receives the events in a separate thread and writes data to the cyclic buffer */
static void * receive_data_thread(void * arg)
{
    WzDaq1State * s = (WzDaq1State *) arg;
    zsock_t *pull = zsock_new_pair ("");
    zsock_bind(pull,"tcp://0.0.0.0:*[3000-]");
    while(1) {
        zmsg_t * msg = zmsg_recv (pull);
        //The engine must be running
        if(s->running) {
            //This must be a single frame message!
            if(zmsg_size(msg) != 1) {
                //Handle improper size of the message
            } else {
                zframe_t * frame = zmsg_first(msg);
                //Process the frame
                uint64_t * ptr = (uint64_t *)zframe_data(frame);
                if(zframe_size(frame) % 8) {
                    printf("Frame size not N*8\n");
                } else {
                    uint64_t * pend = ptr + zframe_size(frame)/8;
                    do {
                        //Check the type of the record
                        void * rec = (void *) ptr;
                        if (!memcmp(rec,"WZDAQ1-E",8)) {
                            printf("End of set!\n");
                            //Here we should close the current data set
                            end_segment(s);
                            //and start the next one (maybe it should be suspended until the first data?)
                            //No, it can't because it interferes with the method used to detect if
                            //the event was completed.
                            start_segment(s);
                            //Raise the irq
                            s->irq_pending = 1;
                            if (s->irq_enabled)
                                pci_irq_assert(&s->pdev);
                            ptr ++;
                        } else if (!memcmp(rec,"WZDAQ1-D",8)) {
                            if(++ptr >= pend) {
                                printf("Error: WZDAQ1-D at end of frame\n");
                                abort();
                            }
                            int nwords = be64toh(* ptr++);
                            if (ptr + nwords > pend) {
                                printf("Error: WZDAQ1-D data beyond the end of frame\n");
                                abort();
                            }
                            printf("Frame: %d words\n",nwords);
                            //Add the words to the circular buffer
                            add_words(s,(uint64_t *) zframe_data(frame),nwords);
                            ptr += nwords;
                        } else if (!memcmp(rec,"WZDAQ1-Q",8)) {
                            printf("End of run!\n");
                            break;
                        } else {
                            printf("Error: No data type!");
                            abort();
                        }
                    } while(ptr < pend);
                }
            }
        }
        zmsg_destroy(&msg);
    }
    zsock_destroy (&pull);
    return 0;
}

static void pci_wzdaq1_realize (PCIDevice *pdev, Error **errp)
{
    WzDaq1State *s = PCI_WZDAQ1(pdev);
    uint8_t *c = s->pdev.config;

    /* TODO: RST# value should be 0. */
    c[PCI_INTERRUPT_PIN] = 1;
    memory_region_init_io(&s->mmio,OBJECT(s),&pci_wzdaq1_mmio_ops,s,
                          "pci-wzdaq1-mmio", sizeof(uint64_t) * 2 * DAQ1_NBUFS);
    pci_register_bar (&s->pdev, 0,  PCI_BASE_ADDRESS_SPACE_MEMORY | PCI_BASE_ADDRESS_MEM_TYPE_64, &s->mmio);
    //Timer is not used, data are delivered by ZMQ!
    //s->daq_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, wzdaq1_tick, s);
    //Add the thread receiving the data
    qemu_thread_create(&s->thread, "receive_data", receive_data_thread, s,
                       QEMU_THREAD_JOINABLE);
    //AUD_register_card ("wz_adc1", &s->card);
    wzdaq1_reset (s);
    //return 0;
}



static void pci_wzdaq1_init(Object *obj)
{
}

static void
pci_wzdaq1_uninit(PCIDevice *dev)
{
    WzDaq1State *s = PCI_WZDAQ1(dev);
    wzdaq1_reset(s);

    qemu_thread_join(&s->thread);
    //timer_free(s->daq_timer);
}

static void qdev_pci_wzdaq1_reset(DeviceState *dev)
{
    WzDaq1State *s = PCI_WZDAQ1(dev);
    wzdaq1_reset(s);
}

static void pci_wzdaq1_class_init(ObjectClass *class, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(class);
    PCIDeviceClass *k = PCI_DEVICE_CLASS(class);

    k->realize = pci_wzdaq1_realize;
    k->exit = pci_wzdaq1_uninit;
    k->vendor_id = PCI_VENDOR_ID_WZAB;
    k->device_id = PCI_DEVICE_ID_WZAB_WZDAQ1;
    k->revision = 0x00;
    k->class_id = PCI_CLASS_OTHERS;
    dc->desc = "PCI demo BM DMA ADC";
    set_bit(DEVICE_CATEGORY_MISC, dc->categories);
    dc->reset = qdev_pci_wzdaq1_reset;
}


static void pci_wzdaq1_register_types(void)
{
    static InterfaceInfo interfaces[] = {
        { INTERFACE_PCIE_DEVICE },
        { },
    };
    static const TypeInfo pci_wzdaq1_info = {
        .name          = TYPE_PCI_WZDAQ1,
        .parent        = TYPE_PCI_DEVICE,
        .instance_size = sizeof(WzDaq1State),
        .instance_init = pci_wzdaq1_init,
        .class_init    = pci_wzdaq1_class_init,
        .interfaces = interfaces,
    };

    type_register_static(&pci_wzdaq1_info);
}

type_init(pci_wzdaq1_register_types)



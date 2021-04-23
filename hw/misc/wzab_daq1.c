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
#include <msgpack.h>
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
    uint64_t buf_hps[DAQ1_NBUFS]; //Addresses of HPs creating the circular buffer
    uint64_t evt_hp; //Address of HP keeping the event descriptors
    uint32_t overrun;
    uint32_t irq_pending;
    QEMUTimer * daq_timer;
    QemuThread thread;
} WzDaq1State;

#define TYPE_PCI_WZDAQ1 "pci-wzdaq1"

static uint64_t dt_buf[20000]; //Here we generatr the data before sending via DMA
static uint64_t cur_dta = 0; //Current data, will be increased

#define PCI_WZDAQ1(obj) OBJECT_CHECK(WzDaq1State, obj, TYPE_PCI_WZDAQ1)

static const MemoryRegionOps pci_wzdaq1_mmio_ops = {
    .read = pci_wzdaq1_read,
    .write = pci_wzdaq1_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .impl = {
        .min_access_size = 8, //Always 64-bit access!!
        .max_access_size = 8,
    }
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
    printf("Memory read: address %" PRIu64 "\n", (uint64_t) addr);
#endif
    WzDaq1State *s = opaque;
    uint64_t ret=0xbada4ea55aa55aa; //Special value returned when accessed non-existing register
    addr = (addr/8) & 0x0ff;
    //Special cases
    if(addr==DAQ1_READP) {
        ret = s->read_ptr;
#ifdef DEBUG_wzab1
        printf(" value %"PRIu64"\n",ret);
#endif
        return ret;
    }
    if(addr==DAQ1_WRITEP) {
        ret = s->write_ptr;
#ifdef DEBUG_wzab1
        printf(" value %"PRIu64"\n",ret);
#endif
        return ret;
    }
    //normal case
    if(addr<DAQ1_REGS_NUM) {
        ret = s->regs[addr];
#ifdef DEBUG_wzab1
        printf(" value %"PRIu64"\n",ret);
#endif
    }
    return ret;
}


/* called for write accesses to our register memory area */
void pci_wzdaq1_write(void *opaque, hwaddr addr, uint64_t val, unsigned size)
{
    WzDaq1State *s = opaque;
#ifdef DEBUG_wzab1
    printf("wzab1: zapis pod adres = 0x%016" PRIu64 ", 0x%016" PRIu64 "\n", addr, val);
#endif
    /* convert to wzdaq1 memory offset */
    addr = (addr/8);
    /* always write value to the shadow register */
    if(addr<DAQ1_REGS_NUM) {
        s->regs[addr]=val;
        /* for certain registers take additional actions */
        switch(addr) {
        case DAQ1_PERIOD:
            if(val) {
                //Start sampling
                s->read_ptr = 0;
                s->write_ptr = -1;
                s->overrun = 0;
                timer_mod(s->daq_timer,qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL)+s->regs[DAQ1_PERIOD]);
            } else {
                //Stop sampling
#ifdef DEBUG_wzab1
                printf("deleting timer!\n");
#endif
                timer_del(s->daq_timer);
                s->read_ptr = 0;
                s->write_ptr = 0;
                s->overrun = 0;
            }
            break;

        case DAQ1_READP:
            s->read_ptr = val;
            break;
        case DAQ1_HPSHFT:
            s->hpage_shift = val;
            s->hpage_size = ((uint64_t) 1) << val;
            s->hpage_mask = s->hpage_size - 1;
            break;
        case DAQ1_CTRL:
            if((val & 1) == 0) {
                //Switch the IRQ off
                pci_irq_deassert(&s->pdev);
            } else {
                //Rise the IRQ if it's pending
                if(s->irq_pending) pci_irq_assert(&s->pdev);
            }
            break;
        default:
            return;
            break;
        }
    } else if (( addr >= DAQ1_NBUFS ) && ( addr < 2 * DAQ1_NBUFS )) {
        // Write the hugepage address to the buffer
        s->buf_hps[addr-DAQ1_NBUFS] = val;
    }
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
    uint64_t wfree = (s->write_ptr - s->read_ptr) & s->ptr_mask;
    if (nwords > wfree) {
        nwords = wfree;
    }
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
      //Now convert it to msgpack
      msgpack_unpacked umsg;
      msgpack_unpacked_init(&umsg);
      zframe_t * frame = zmsg_first(msg);

      while(frame) {
          msgpack_unpack_return ret = msgpack_unpack_next(&umsg,(const char *) zframe_data(frame), zframe_size(frame), NULL);
          if(ret < 0) {
              perror("unpacking data");
              abort(); // To be corrected!
          }
        frame = zmsg_next(msg);
      };
      /* print the deserialized object. (here will be the handling of the message) */
      msgpack_object_print(stdout, umsg.data);
      puts("");
      //Adds the data
      add_words(s,(uint64_t *) &umsg.data, 12); /* Just placeholder!!! */
      zmsg_destroy(&msg);
      msgpack_unpacked_destroy(&umsg);
    }
    zsock_destroy (&pull);
    return 0;    
}

/* The procedure is called cyclically and either adds the next part of an event to the circular buffer,
 * or closes the current event and writes the header of the new one.
 *
 *
 */

static void wzdaq1_tick(void *opaque)
{
    WzDaq1State * s = opaque;
    //Set the timer to the next value.
    timer_mod(s->daq_timer,qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL)+s->regs[DAQ1_PERIOD]);
    //Write a few random samples to the buffer
    {
        int evl;     //length of the segment
        int i;
        evl = (random() & 0x1fff) + 11000; // Less than 20000 (length of the buffer)
        for(i=0; i<evl; i++) {
            dt_buf[i] = cur_dta++;
        }
        add_words(s, dt_buf, evl);
    }
    //Raise IRQ informing that the new block have arrived
    s->irq_pending = 1;
#ifdef DEBUG_wzab1
    printf("Request IRQ!\n");
#endif
    pci_irq_assert(&s->pdev);
}

static void pci_wzdaq1_realize (PCIDevice *pdev, Error **errp)
{
    WzDaq1State *s = PCI_WZDAQ1(pdev);
    uint8_t *c = s->pdev.config;

    /* TODO: RST# value should be 0. */
    c[PCI_INTERRUPT_PIN] = 1;
    memory_region_init_io(&s->mmio,OBJECT(s),&pci_wzdaq1_mmio_ops,s,
                          "pci-wzadc1-mmio", 0x100); //@@sizeof(s->regs.u32));
    pci_register_bar (&s->pdev, 0,  PCI_BASE_ADDRESS_SPACE_MEMORY,&s->mmio);
    s->daq_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, wzdaq1_tick, s);
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
    timer_free(s->daq_timer);
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
        { INTERFACE_CONVENTIONAL_PCI_DEVICE },
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



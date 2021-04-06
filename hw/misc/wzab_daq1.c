/*
 * QEMU WZADC1 emulation
 *
 * Copyright Wojciech M. Zabolotny ( wzab@ise.pw.edu.pl ) 2011-2014
 *
 * The code below implements a simple Analog to Digital converter
 * for QEMU, which uses Bus Mastering DMA to transfer data to the PC.
 * The real device may be implemented in an FPGA connected to the 
 * system device, however this model is assumed to be used mainly
 * as a didicatical aid for students learning how to write and debug
 * Linux device drivers.
 * The code was written by Wojciech M. Zabolotny (wzab<at>ise.pw.edu.pl)
 * in March and April 2011, however it is significantly based on different
 * source codes provided
 * in the QEMU sources.
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
 *
 *
 * Description of the ADC
 * 
 * Device uses 8 32-bit registers (with symbolic names defined in the file wzab_tst1.h)
 * TST1_DIV - writing to this register sets the internal frequency divider, defining
 *            the sampling rate of the converter. Writing of zero stops sampling at all.
 *            Writing of nonzero value - starts sampling
 * TST1_PAGE1 - TST_PAGE4
 *            The 4 registers above should be written with physical addresses of the
 *            pages allocated as the buffer for acquired data.
 *            Therefore the device offers 16*4096=16384 bytes buffer (i.e. buffer for
 *            4096 32-bit samples).
 * TST1_READP - The read pointer for data buffer (counts in 32-bit dwords!)
 *              the 30-th bit informs if the device requests interrupt
 *              the 31-st bit informs if "overrun" occured (see below).
 * TST1_WRITEP	- The write pointer for data buffer (count in 32-bit dwords!).
 *                If the new acquired value is about to overwrite the previous one, which 
 *                has not been received yet, the "overrun" status is set, and new data are ignored.
 *                To restart correct operation, you have to write TST1_DIV with 0, and then
 *                with the correct value.
 * TST1_CTRL - Currently only one (0th) bit of this register is used. If this bit is set
 *             to 1, the device generates interrupt after new samples arrive. If this
 *             bit is cleared, the device does not generate the interrupt
 * 
 * The device is connected to the PCI bus - it's memory and interrupt resources are
 * reported to the host using the standard PC mechanisms.
 */

#define DEBUG_wzab1 1 
//PCI IDs below are not registred! Use only for experiments!
#define PCI_VENDOR_ID_WZAB 0xabba
#define PCI_DEVICE_ID_WZAB_WZADC1 0x0133
#include "qemu/osdep.h"
#include <inttypes.h>
#include "qemu/compiler.h"
#include <string.h>
#include "hw/sysbus.h"
#include "hw/hw.h"
#include "hw/pci/pci.h"
#include "qemu/timer.h"
#include "wzab_adc1.h"

static uint64_t pci_wz_adc1_read(void *opaque, hwaddr addr, unsigned size);
static void pci_wz_adc1_write(void *opaque, hwaddr addr, uint64_t val, unsigned size);

typedef struct WzAdc1State {
  /*<private>*/
  PCIDevice parent_obj;
  /*<public>*/
  MemoryRegion mmio;
  uint32_t regs[TST1_REGS_NUM];
  uint32_t writep, readp;
  uint32_t wz_adc1_mmio_io_addr;
  uint32_t overrun;
  uint32_t irq_pending;
  uint32_t div;
  uint32_t sample_val;
  QEMUTimer * timer;
} WzAdc1State;

#define TYPE_PCI_WZADC1 "pci-wzadc1"
 
#define PCI_WZADC1(obj) \
OBJECT_CHECK(WzAdc1State, (obj), TYPE_PCI_WZADC1) 
static const MemoryRegionOps pci_wz_adc1_mmio_ops = {
    .read = pci_wz_adc1_read,
    .write = pci_wz_adc1_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .impl = {
        .min_access_size = 4, //Always 32-bit access!!
        .max_access_size = 4,
    },
};

static void wz_adc1_reset (void * opaque)
{
  WzAdc1State *s = opaque;
  memset(s->regs,0,sizeof(s->regs));
#ifdef DEBUG_wzab1
  printf("wzab_tst1 reset!\n");
#endif
}

/* called for read accesses to our register memory area */
static uint64_t pci_wz_adc1_read(void *opaque, hwaddr addr, unsigned size)
{
#ifdef DEBUG_wzab1
  printf("Memory read: address %" PRIu64 "\n", (uint64_t) addr);
#endif
  WzAdc1State *s = opaque;
  uint32_t ret=0xbada4ea; //Special value returned when accessed non-existing register
  addr = (addr/4) & 0x0ff;
  //Special cases
  if(addr==TST1_READP) {
    ret = s->readp | (s->overrun ? (1<<31) : 0) | (s->irq_pending ? (1<<30) : 0);
#ifdef DEBUG_wzab1
    printf(" value %x\n",ret);
#endif
    return ret;
  }
  if(addr==TST1_WRITEP) {
    ret = s->writep;
#ifdef DEBUG_wzab1
    printf(" value %x\n",ret);
#endif
    return s->writep;
  }
  //normal case
  if(addr<TST1_REGS_NUM) {
    ret = s->regs[addr];
#ifdef DEBUG_wzab1
    printf(" value %x\n",ret);
#endif
  }
  return ret;
}


/* called for write accesses to our register memory area */
void pci_wz_adc1_write(void *opaque, hwaddr addr, uint64_t val, unsigned size)
{
  WzAdc1State *s = opaque;
#ifdef DEBUG_wzab1  
  printf("wzab1: zapis pod adres = 0x%016" PRIu64 ", 0x%016" PRIu64 "\n", addr, val);
#endif
  /* convert to wzab1 memory offset */
  addr = (addr/4) & 0xff;
  /* always write value to the register */
  if(addr<TST1_REGS_NUM) {
    s->regs[addr]=val;
  }
  switch(addr) {
  case TST1_DIV:
    if(val) {
      //Start sampling
      s->readp = 0;
      s->writep = 0;
      s->overrun = 0;
      timer_mod(s->timer,qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL)+s->regs[TST1_DIV]);
    } else {
      //Stop sampling
#ifdef DEBUG_wzab1
      printf("deleting timer!\n");      
#endif
      timer_del(s->timer);
      s->readp = 0;
      s->writep = 0;
      s->overrun = 0;
    }
    break;
  case TST1_PAGE1:
  case TST1_PAGE2:
  case TST1_PAGE3:
  case TST1_PAGE4:
    break;
  case TST1_READP:
    s->readp = val & ((1<<12)-1);
    break;
  case TST1_CTRL:
    if((val & 1) == 0) {
      //Switch the IRQ off
      pci_irq_deassert(&s->parent_obj);
    } else {
      //Rise the IRQ if it's pending
      if(s->irq_pending) pci_irq_assert(&s->parent_obj);
    }	
    break;
  default:
    return;
    break;
  }
}

/* The procedure below emulates 100 of sampling cycles */
static void wzab1_tick(void *opaque)
{ 
  WzAdc1State * s = opaque;
  //rearm timer
  timer_mod(s->timer,qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL)+s->regs[TST1_DIV]);
  //Write 100 samples to the buffer
  {
    int i;
    for(i=0;i<100;i++) {
      //We have place for 4096 samples, so we use only 12-bit pointer
      int new_writep = (s->writep+1) & ((1<<12) - 1);
      if (new_writep == s->readp) { //Overrun!
	s->overrun = 1;
	timer_del(s->timer); //Stop sampling!
	break;
      } else if (s->overrun == 0) {
        
	int pagenr = (s->writep >> 10); // bits 11 and 10 - page nr
	int dwordnr = s->writep & ((1<<10) - 1); // bits 9 to 0 - dword nr
#ifdef DEBUG_wzab1
	printf("trying to write in %08x \n",s->regs[TST1_PAGE1+pagenr]+dwordnr*4);
#endif
	pci_dma_write(&s->parent_obj, s->regs[TST1_PAGE1+pagenr]+dwordnr*4,(uint8_t *)&(s->sample_val),4);
	s->writep = new_writep; //Should be done in atomic way, so no locking...
	//Modify the sample value. Currently we generate the sawtooth waveform with values between 0 and 1110
	s->sample_val = s->sample_val+1;
        if(s->sample_val == 1111) s->sample_val = 0;
      }
    }
  }
  //Raise IRQ
  s->irq_pending = 1;
#ifdef DEBUG_wzab1
  printf("Request IRQ!\n");
#endif
  pci_irq_assert(&s->parent_obj);
}

/*
static const VMStateDescription vmstate_wz_adc1 = {
  .name = "wz_adc1",
  .version_id = 2,
  .minimum_version_id = 2,
  .minimum_version_id_old = 2,
  //.post_load = wz_adc1_post_load,
  .fields      = (VMStateField []) {
    VMSTATE_PCI_DEVICE(parent_obj, WzAdc1State),
    VMSTATE_TIMER_PTR(timer,WzAdc1State),
    VMSTATE_END_OF_LIST()
  }
};
*/

/*
static void wz_adc1_on_reset (void *opaque)
{
  WzAdc1State *s = opaque;
  wz_adc1_reset (s);
}
*/

static void pci_wzadc1_realize (PCIDevice *pdev, Error **errp)
{
  WzAdc1State *s = PCI_WZADC1(pdev);
  uint8_t *c = s->parent_obj.config;

  /* TODO: RST# value should be 0. */
  c[PCI_INTERRUPT_PIN] = 1;
  memory_region_init_io(&s->mmio,OBJECT(s),&pci_wz_adc1_mmio_ops,s,
                        "pci-wzadc1-mmio", 0x100); //@@sizeof(s->regs.u32));
  pci_register_bar (&s->parent_obj, 0,  PCI_BASE_ADDRESS_SPACE_MEMORY,&s->mmio);
  s->timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, wzab1_tick, s);
  //AUD_register_card ("wz_adc1", &s->card);
  wz_adc1_reset (s);
  //return 0;
}

static void pci_wz_adc1_init(Object *obj)
{
}
 
static void
pci_wz_adc1_uninit(PCIDevice *dev)
{
    WzAdc1State *d = PCI_WZADC1(dev);
    wz_adc1_reset(d);
    timer_free(d->timer);
}

static void qdev_pci_wz_adc1_reset(DeviceState *dev)
{
    WzAdc1State *d = PCI_WZADC1(dev);
    wz_adc1_reset(d);
}

static void pci_wzadc1_class_init(ObjectClass *class, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(class);
    PCIDeviceClass *k = PCI_DEVICE_CLASS(class);

    k->realize = pci_wzadc1_realize,
    k->exit = pci_wz_adc1_uninit;
    k->vendor_id = PCI_VENDOR_ID_WZAB;
    k->device_id = PCI_DEVICE_ID_WZAB_WZADC1;
    k->revision = 0x00;
    k->class_id = PCI_CLASS_OTHERS;
    dc->desc = "PCI demo BM DMA ADC";
    set_bit(DEVICE_CATEGORY_MISC, dc->categories);
    dc->reset = qdev_pci_wz_adc1_reset;
}


static void pci_wzadc1_register_types(void)
{
    static InterfaceInfo interfaces[] = {
        { INTERFACE_CONVENTIONAL_PCI_DEVICE },
        { },
    };
    static const TypeInfo pci_wz_adc1_info = {
    	.name          = TYPE_PCI_WZADC1,
    	.parent        = TYPE_PCI_DEVICE,
    	.instance_size = sizeof(WzAdc1State),
    	.instance_init = pci_wz_adc1_init,
    	.class_init    = pci_wzadc1_class_init,
        .interfaces = interfaces,
	};
    
    type_register_static(&pci_wz_adc1_info);
}

type_init(pci_wzadc1_register_types)



/*
 * QEMU WZTIM1 emulation
 *
 * Copyright Wojciech M. Zabolotny ( wzab@ise.pw.edu.pl ) 2011-2014
 *
 * The code below implements a simple TIMER for QEMU, which
 * generates the periodic interrupts and allows you to
 * check how much time elapsed between the last interrupt
 * and the time when the register is read.
 * The initial code was written by Wojciech M. Zabolotny (wzab<at>ise.pw.edu.pl)
 * in March and April 2011, now (December 2018) it has been modified for
 * timer.
 * The code is significantly based on different source codes provided
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
 * Description of the timer
 *
 * Device uses 8 32-bit registers (with symbolic names defined in the file wzab_tim1.h)
 * TIM1_ID   - You can read this register to make sure, that the device is present
 * TIM1_DIVH - most significant word of the interrupt period
 * TIM1_DIVL - least significant word of the interruput period
 *             writing to this register sets the internal frequency divider, defining
 *             the period of interrupts generated by the timer.
 *             If both TIM1_DIVL and TIM1_DIVH are zero, generation of interrupts
 *             is stopped at all.
 *             Writing of nonzero value - starts generation of periodic interupts
 * TIM1_CNTH - reading of that register returns the most significant word of the time
 *             elapsed since the last interrupt
 * TIM1_CNTL - reading of that register latches the time elapsed since the last interrupt
 *             and returns the lower word of that time counter.
 *             writing to this register cancels the interrupt.
 * TIM1_STAT - reading of this register allows you to check if there is an interrupt pending
 *             (bit 31). Bit 0 allows you to mask the interrupt
 */

#define DEBUG_wzab1 1
//PCI IDs below are not registred! Use only for experiments!
#define PCI_VENDOR_ID_WZAB 0xabba
#define PCI_DEVICE_ID_WZAB_WZTIM1 0x0123

#include <inttypes.h>
#include "qemu/osdep.h"
#include <string.h>
#include "qemu/compiler.h"
#include "hw/sysbus.h"
#include "hw/hw.h"
#include "hw/pci/pci.h"
#include "qemu/timer.h"
#include "wzab_tim1.h"

static uint64_t pci_wztim1_read(void *opaque, hwaddr addr, unsigned size);
static void pci_wztim1_write(void *opaque, hwaddr addr, uint64_t val, unsigned size);
int wzadc1_init(SysBusDevice *sbd);

typedef struct WzTim1State {
    /*<private>*/
    PCIDevice parent_obj;
    /*<public>*/
    MemoryRegion mmio;
    uint32_t regs[TIM1_REGS_NUM];
    uint32_t irq_pending;
    uint32_t irq_mask;
    uint64_t prev_tick;
    uint64_t next_tick;
    uint64_t time_elapsed;
    uint64_t period;
    QEMUTimer * timer;
} WzTim1State;

#define TYPE_PCI_WZTIM1 "pci-wztim1"

#define PCI_WZTIM1(obj) \
OBJECT_CHECK(WzTim1State, (obj), TYPE_PCI_WZTIM1)
static const MemoryRegionOps pci_wztim1_mmio_ops = {
    .read = pci_wztim1_read,
    .write = pci_wztim1_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .impl = {
        .min_access_size = 4, //Always 32-bit access!!
        .max_access_size = 4,
    },
};

static void wztim1_reset (void * opaque)
{
    WzTim1State *s = opaque;
    memset(s->regs,0,sizeof(s->regs));
#ifdef DEBUG_wzab1
    printf("wztim1 reset!\n");
#endif
}

/* called for read accesses to our register memory area */
static uint64_t pci_wztim1_read(void *opaque, hwaddr addr, unsigned size)
{
#ifdef DEBUG_wzab1
    printf("Memory read: address %" PRIx64 "\n", (uint64_t) addr);
#endif
    WzTim1State *s = opaque;
    uint64_t ret=0xbada4ea; //Special value returned when accessed non-existing register
    addr = (addr/4) & 0x0ff;
    //Special cases
    if(addr==TIM1_STAT) {
        ret = (s->irq_pending ? (1<<31) : 0) | (s->irq_mask ? 1 : 0);
#ifdef DEBUG_wzab1
        printf(" value %lx\n",ret);
#endif
        return ret;
    }
    if(addr==TIM1_CNTL) {
        //We find the time elapsed since the last interrupt
        s->time_elapsed = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) - s->prev_tick;
        ret = s->time_elapsed & 0xffffffff;
#ifdef DEBUG_wzab1
        printf(" time elapsed: %lx\n",s->time_elapsed);
        printf(" value %lx\n",ret);
#endif
        return ret;
    }
    if(addr==TIM1_CNTH) {
        ret = (s->time_elapsed >> 32) & 0xffffffff;
#ifdef DEBUG_wzab1
        printf(" value %lx\n",ret);
#endif
        return ret;
    }
    if(addr==TIM1_ID) {
        ret = 0x7130900d;
#ifdef DEBUG_wzab1
        printf(" value %lx\n",ret);
#endif
        return ret;
    }
    //If there is any register not covered by the special handling,
    //just return its value (now it is only TIM1_DIV)
    if(addr<TIM1_REGS_NUM) {
        ret = s->regs[addr];
#ifdef DEBUG_wzab1
        printf(" value %lx\n",ret);
#endif
    }
    return ret;
}


/* called for write accesses to our register memory area */
void pci_wztim1_write(void *opaque, hwaddr addr, uint64_t val, unsigned size)
{
    WzTim1State *s = opaque;
#ifdef DEBUG_wzab1
    printf("wzab1: zapis pod adres = 0x%016" PRIx64 ", 0x%016" PRIx64 "\n", addr, val);
#endif
    /* convert to wzab1 memory offset */
    addr = (addr/4) & 0xff;
    /* always write value to the register */
    if(addr<TIM1_REGS_NUM) {
        s->regs[addr]=val;
    }
    switch(addr) {
    case TIM1_DIVL:
        s->period = ((uint64_t) s->regs[TIM1_DIVH]) << 32 | val;
        if(s->period) {
            //Start generating interrupts
            s->prev_tick = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
            s->next_tick = s->prev_tick + s->period;
            timer_mod(s->timer,s->next_tick);
        } else {
            //Stop sampling
#ifdef DEBUG_wzab1
            printf("deleting timer!\n");
#endif
            timer_del(s->timer);
        }
        break;
    //TIM1_DIVH is handled by a standard procedure
    case TIM1_STAT:
        if((val & 1) == 0) {
            //Switch the IRQ off
            s->irq_mask = 1;
            pci_irq_deassert(&s->parent_obj);
        } else {
            //Rise the IRQ if it's pending
            s->irq_mask = 0;
            if(s->irq_pending) pci_irq_assert(&s->parent_obj);
        }
        break;
    case TIM1_CNTL:
        s->irq_pending = 0;
        pci_irq_deassert(&s->parent_obj);
        break;
    default:
        return;
        break;
    }
}

/* This is the timer handler routine */
static void wzab1_tick(void *opaque)
{
    WzTim1State * s = opaque;
    //rearm timer (we use the prev_tick value, to avoid
    //increase of average interrupt period)
    uint64_t next_tick = s->next_tick+s->period;
    timer_mod(s->timer,next_tick);
    s->prev_tick = s->next_tick;
    s->next_tick = next_tick;
    //Raise IRQ
    s->irq_pending = 1;
#ifdef DEBUG_wzab1
    printf("Request IRQ!\n");
#endif
    if(s->irq_mask==0) pci_irq_assert(&s->parent_obj);
}

static void wztim1_on_reset (void *opaque)
{
    WzTim1State *s = opaque;
    wztim1_reset (s);
}

static void pci_wztim1_init (Object *obj)
{
}

static void pci_wztim1_realize (PCIDevice *pdev, Error **erp)
{
    WzTim1State *s = PCI_WZTIM1(pdev);
    uint8_t *c = s->parent_obj.config;
    /* TODO: RST# value should be 0. */
    c[PCI_INTERRUPT_PIN] = 1;
    memory_region_init_io(&s->mmio,OBJECT(s),&pci_wztim1_mmio_ops,s,
                          "pci-wztim1-mmio", 0x100); //@@sizeof(s->regs.u32));
    pci_register_bar(&s->parent_obj,0,PCI_BASE_ADDRESS_SPACE_MEMORY,&s->mmio);
    qemu_register_reset (wztim1_on_reset, s);
    s->timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, wzab1_tick, s);
    wztim1_reset (s);
    //return 0;
}

static void
pci_wztim1_uninit(PCIDevice *dev)
{
    WzTim1State *d = PCI_WZTIM1(dev);
    wztim1_reset(d);
    timer_free(d->timer);
}

static void pci_wztim1_reset(DeviceState *dev)
{
    WzTim1State *d = PCI_WZTIM1(dev);
    wztim1_reset(d);
}

static void pci_wztim1_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    PCIDeviceClass *k = PCI_DEVICE_CLASS(klass);
    k->realize = pci_wztim1_realize;
    //k->init = sysbus_wztim1_init;
    k->exit = pci_wztim1_uninit;
    k->vendor_id = PCI_VENDOR_ID_WZAB;
    k->device_id = PCI_DEVICE_ID_WZAB_WZTIM1;
    dc->desc = "PCI demo TIMER";
    set_bit(DEVICE_CATEGORY_MISC, dc->categories);
    dc->reset = pci_wztim1_reset;
}

static void pci_wztim1_register_types(void)
{
    static InterfaceInfo interfaces[] = {
        { INTERFACE_CONVENTIONAL_PCI_DEVICE },
        { },
    };
    static const TypeInfo pci_wztim1_info = {
        .name          = TYPE_PCI_WZTIM1,
        .parent        = TYPE_PCI_DEVICE,
        .instance_size = sizeof(WzTim1State),
        .instance_init = pci_wztim1_init,
        .class_init    = pci_wztim1_class_init,
        .interfaces = interfaces,
    };
    type_register_static(&pci_wztim1_info);
}

type_init(pci_wztim1_register_types)



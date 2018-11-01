/*
 * QEMU - AES256 cryptographic engine WZENC1 emulation
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

#define DEBUG_wzab1 1 

//Simulated processing time
#define ENC1_PROCESSING_TIME 5000000
#include <inttypes.h>
#include "qemu/osdep.h"
#include <string.h>
#include "qemu/compiler.h"
#include "hw/sysbus.h"
#include "hw/hw.h"
#include <mcrypt.h>
#include "qemu/timer.h"
#include "wzab_enc1.h"

/*
  Description of functionalities.
  The emulated hardware is able to perform encryption/decryption using AES256 algorithm.
  It is a sysbus connected (platform) device with memory mapped registers.
  Register definitions are provided in "wzab_enc1.h" file
  The data transfer is fulfilled using bus-mastering DMA
  
  Registers (all 32-bit):
  Ctrl - offset 0
  Status - offset 1
  Pages[4] - starting from offset 2 
       4 sets of registers describing pages used to transfer data
       each set contains:
       PhysAddr register - physical address of the begining of the page
       Offset   register - start address of the encrypted/decrypted content on the page
       Length   register - length (in bytes) of the encrypted/decrypted content on the page
                           length should be equal to N*B where B is length of the block
                           in the cipher (in case of AES256 - 32 bytes).

  It is relatively easy to modify the model to use more pages.
  The model should be used as follows:
  1) Initialization of the core
     a) Key should be stored in the first 32 bytes of page pointed by Pages[0].PhysAddr
     b) Initialization vector should be stored in the first 32 bytes of page
        pointed by Pages[1].PhysAddr (Offset registers are not used)
     c) Write ENC1_CMD_ENCR to Ctrl to initialize core for encryption or
        write ENC1_CMD_DECR to Ctrl to initialize core for decryption
  2) Processing of data
     a) Data should be stored in buffers located on up to 4 pages, described
        by registers PhysAddr, Offset, Length in Pages register sets.
     b) Processing ends either after 4 pages are processed or after first 
        page with Length set to 0 is found.
     c) When data are written to buffers and Pages registers are set, write
        the ENC1_CMD_DATA command to Ctrl
   3) After all data are encrypted or decrypted, the core requests interrupt.
     a) The interrupt may be masked by writing ENC1_CMD_DISIRQ to Ctrl
     b) The interrupt may be unmasked by writing ENC1_CMD_ENAIRQ to Ctrl
     c) After the interrupt is serviced, it should be acknowledged by writing
        ENC1_CMD_ACKIRQ to Ctrl
   4) If data do not fit in 4 pages, step (3) may be repeated until all data are
      processed.
   5) After all data are processed, write ENC1_CMD_STOP to Ctrl to free the core
   6) Following errors are detected:
      a) Attempt to use non-initialized (see step 1) core - ENC1_ERR_NOTINIT
      b) Attempt to start next processing when previous is not finished
         ENC1_ERR_BUSY
   7) Errors are reported via Status register. The bits of this register
      have following meaning:
      a) 31st - Interrupt is pending
      b) 30th - Interrupt is masked
      c) 28th - Core is busy
      d) 15th-0th - Error code
*/

//Some prototypes...
static uint64_t wz_enc1_read(void *opaque, hwaddr addr, unsigned size);
static void wz_enc1_write(void *opaque, hwaddr addr, uint64_t val, unsigned size);
int wz_enc1_init (SysBusDevice *sbd);

typedef struct WzEnc1State {
  /*<private>*/
  SysBusDevice parent_obj;
  /*<public>*/
  MemoryRegion iomem;
  union {
    WzEnc1Regs r;
    uint32_t u32[sizeof(WzEnc1Regs)/sizeof(uint32_t)];
  } regs;
  uint32_t wz_enc1_io_addr;
  uint32_t irq_pending;
  uint32_t irq_mask;
  uint8_t Error;
  uint8_t Working;
  MCRYPT td;
  uint8_t enc_ndec;
  QEMUTimer * timer;
  qemu_irq irq;
} WzEnc1State;

#define TYPE_SYSBUS_WZENC1 "sysbus-wzenc1"
 
#define WZENC1(obj) \
OBJECT_CHECK(WzEnc1State, (obj), TYPE_SYSBUS_WZENC1) 
static const MemoryRegionOps wz_enc1_iomem_ops = {
    .read = wz_enc1_read,
    .write = wz_enc1_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .impl = {
        .min_access_size = 4, //Always 32-bit access!!
        .max_access_size = 4,
    },
};


static void wz_enc1_reset (void * opaque)
{
  WzEnc1State *s = opaque;
  memset((void *)s->regs.u32,0,sizeof(s->regs.u32));
#ifdef DEBUG_wzab1
  printf("wzab_tst1 reset!\n");
#endif
}

/* called for read accesses to our register memory area */
static uint64_t wz_enc1_read(void *opaque, hwaddr addr, unsigned size)
{
  int i_addr;
#ifdef DEBUG_wzab1
  printf("Memory read: address %x\n ", (unsigned int) addr);
#endif
  WzEnc1State *s = opaque;
  if(addr>=sizeof(WzEnc1Regs)){
    //Read above registers area - return "0xbada4ea" to show it
    return 0xbada4ea;
  } else if (addr == offsetof(WzEnc1Regs, Status)) {
    //Reading of the Status - synthesize the correct value
    uint32_t res = s->Error & 0xffff ; //Lower 16 bits!
    if (s->Working) res |= 0x1000;
    if (s->irq_pending) res |= 0x8000;
    if (s->irq_mask) res |= 0x4000;
#ifdef DEBUG_wzab1
    printf("Status=%d\n",res);
#endif
    return res;
  }else {
    i_addr = addr/4;
    //Read the register
    return s->regs.u32[i_addr];
  }
}

static char cipher[] = "rijndael-256";
static char cipher_mode[] = "cbc";

/* called for write accesses to our register memory area */

static void wz_enc1_write(void *opaque, hwaddr addr, uint64_t val, unsigned size)
{
  WzEnc1State *s = opaque;
  int i_addr;
#ifdef DEBUG_wzab1  
  printf("wzab1: zapis pod adres = 0x%016" PRIu64 " , 0x%016" PRIu64 "\n", (uint64_t) addr, val);
#endif
  /* Check which register is accessed */
  if(addr>=sizeof(WzEnc1Regs)){
    //Write above registers area - ignore it!
  } else {
    i_addr = addr/4;
    //Write the value
    s->regs.u32[i_addr]=val;
    //Interprete its special meaning
    if (addr==offsetof(WzEnc1Regs,Ctrl)) {
      //Write to control register - we need to check what operation is required
      switch(val) {
      case ENC1_CMD_DECR:
      case ENC1_CMD_ENCR: {
	//required initialization of the encryption engine
	//key and IV provided
	uint8_t key[32];
	uint8_t iv[32];
	s->td = mcrypt_module_open(cipher, NULL, cipher_mode, NULL);
	if(s->td == NULL) {
	  printf("Initialization A of mcrypt impossible!\n");
	  exit(1);
	}
	if (val==ENC1_CMD_ENCR) {
	  s->enc_ndec = 1; //We will encrypt data
	} else {
	  s->enc_ndec = 0; //We will decrypt data
	}
        cpu_physical_memory_read(s->regs.r.Pages[0].PhysAddr,key,32);
        cpu_physical_memory_read(s->regs.r.Pages[1].PhysAddr,iv,32);
	if(mcrypt_generic_init(s->td,(void*)key,32,(void*)iv)<0){
	  printf("Initialization B of mcrypt impossible!\n");
	  exit(1);
	}
        break;
      }
      case ENC1_CMD_DATA:
	//next set of data provided, no reinitialization of encryption engine
	if(s->td == NULL) {
	  //Error - hardware not initialized
	  s->Error = ENC1_ERR_NOTINIT; //Error hardware not initialized
	  s->irq_pending = 1;
	  if(s->irq_mask==0) qemu_irq_raise(s->irq);
	} else if(s->Working) {
	  //Error - engine is still busy!
	  s->Error = ENC1_ERR_BUSY; //Error!
	  s->irq_pending = 1;
	  if(s->irq_mask==0) qemu_irq_lower(s->irq);
	} else {
	  //Normal operation - submit data to processing
	  s->Working = 0x1;
	  timer_mod(s->timer,qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL)+ENC1_PROCESSING_TIME);
	}
	break;
      case ENC1_CMD_STOP:
	//End of encryption!
	mcrypt_generic_deinit(s->td);
	mcrypt_module_close(s->td);
	s->td = NULL;
	s->irq_pending = 0;
	s->irq_mask = 0;
	qemu_irq_lower(s->irq);
	break;
      case ENC1_CMD_ACKIRQ:
	//Confirm reception of interrupt (and unmask it)
	s->irq_pending = 0;
	s->irq_mask = 0;
	qemu_irq_lower(s->irq);
	break;
      case ENC1_CMD_DISIRQ:
	//Mask interrupt
	s->irq_mask = 1;
	qemu_irq_lower(s->irq);
	break;
      case ENC1_CMD_ENAIRQ:
	//Unmask interrupt
	s->irq_mask = 0;
	if(s->irq_pending) qemu_irq_raise(s->irq);
	break;
      }
    }
  }
}

/* The procedure below performs the real encryption, after simulated processing time is expired */
static void wzab1_tick(void *opaque)
{ 
  WzEnc1State * s = opaque;
  //Encrypt the data
  {
    uint8_t block[32];
    int p;
    for(p=0;p<WZENC1_NOF_PAGES;p++) {
      //Loop through all pages
      int i;
      int j = s->regs.r.Pages[p].PhysAddr + s->regs.r.Pages[p].Offset;
      int len = s->regs.r.Pages[p].Length ;
      if(!len) break; //Page with Length=0 is found
      for(i=0; i<len ; i+=32) {
	//Read data from page to processing buffer
	cpu_physical_memory_read(j+i,block,32);
	if (s->enc_ndec) {
	  //Encrypt data
	  mcrypt_generic (s->td, block, 32);
	} else {
	  //Decrypt data
	  mdecrypt_generic (s->td, block, 32); 
	}
	//Write processed data back
	cpu_physical_memory_write(j+i,block,32);
      }
    }
  }
#ifdef DEBUG_wzab1
  printf("Data processed - request IRQ!\n");
#endif
  s->Working = 0;
  s->irq_pending = 1;
  if(s->irq_mask==0) qemu_irq_raise(s->irq);
}


//Sorry, but the state description below is not complete!
//You are free to fix it!
static const VMStateDescription vmstate_wz_enc1 = {
  .name = "wz_enc1",
  .version_id = 2,
  .minimum_version_id = 2,
  .minimum_version_id_old = 2,
  .fields      = (VMStateField []) {
    VMSTATE_TIMER_PTR(timer,WzEnc1State),
    VMSTATE_END_OF_LIST()
  }
};

/*
static void wz_enc1_on_reset (void *opaque)
{
  WzEnc1State *s = PCI_WZENC1(opaque);
  wz_enc1_reset (s);
}
*/

static int sysbus_wzenc1_init (SysBusDevice *sbd)
{
  DeviceState *dev = DEVICE(sbd);
  WzEnc1State *s = WZENC1(dev);
  //Register memory mapped registers
  memory_region_init_io(&s->iomem,OBJECT(s),&wz_enc1_iomem_ops,s,
                        "sysbus-wzenc1-iomem", 0x100); //@@sizeof(s->regs.u32));
  sysbus_init_mmio(sbd, &s->iomem);
  sysbus_init_irq(sbd, &s->irq);
  qemu_register_reset (wz_enc1_reset, s);
  //Register timer used to simulate processing time
  s->timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, wzab1_tick, s);
  wz_enc1_reset (s);
  return 0;
}

/*
static void
sysbus_wzenc1_uninit(SysBusDevice *sbd)
{
    WzEnc1State *s = WZENC1(sbd);

    wz_enc1_reset(s);
    timer_free(s->timer);
}
*/
static void sysbus_wzenc1_reset(DeviceState *dev)
{
    WzEnc1State *d = WZENC1(dev);
    wz_enc1_reset(d);
}

static void sysbus_wzenc1_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    SysBusDeviceClass *k = SYS_BUS_DEVICE_CLASS(klass);

    k->init = sysbus_wzenc1_init;
    //k->exit = sysbus_wzenc1_uninit;
    dc->desc = "SYSBUS demo AES accelerator";
    dc->reset = sysbus_wzenc1_reset;
    dc->vmsd = &vmstate_wz_enc1;
}

static const TypeInfo sysbus_wzenc1_info = {
    .name          = TYPE_SYSBUS_WZENC1,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(WzEnc1State),
    .class_init    = sysbus_wzenc1_class_init,
};

static void sysbus_wzenc1_register_types(void)
{
    type_register_static(&sysbus_wzenc1_info);
}

type_init(sysbus_wzenc1_register_types)


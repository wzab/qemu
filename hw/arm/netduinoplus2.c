/*
 * Netduino Plus 2 Machine Model
 *
 * Copyright (c) 2014 Alistair Francis <alistair@alistair23.me>
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

#include "qemu/osdep.h"
#include "qapi/error.h"
#include "hw/boards.h"
#include "hw/qdev-properties.h"
#include "sysemu/reset.h"
#include "qemu/error-report.h"
#include "hw/arm/stm32f405_soc.h"
#include "hw/arm/boot.h"

/* Main SYSCLK frequency in Hz (168MHz) */
#define SYSCLK_FRQ 168000000ULL

typedef struct ARMV7MResetArgs {
    ARMCPU *cpu;
    uint32_t reset_sp;
    uint32_t reset_pc;
} ARMV7MResetArgs;

static void armv7m_reset(void *opaque)
{
    ARMV7MResetArgs *args = opaque;

    cpu_reset(CPU(args->cpu));

    args->cpu->env.regs[13] = args->reset_sp & 0xFFFFFFFC;
    args->cpu->env.thumb = args->reset_pc & 1;
    args->cpu->env.regs[15] = args->reset_pc & ~1;
}

static void netduinoplus2_init(MachineState *machine)
{
    DeviceState *dev;
    ARMV7MResetArgs reset_args;
    uint64_t entry;

    /*
     * TODO: ideally we would model the SoC RCC and let it handle
     * system_clock_scale, including its ability to define different
     * possible SYSCLK sources.
     */
    system_clock_scale = NANOSECONDS_PER_SECOND / SYSCLK_FRQ;

    dev = qdev_new(TYPE_STM32F405_SOC);
    qdev_prop_set_string(dev, "cpu-type", ARM_CPU_TYPE_NAME("cortex-m4"));
    sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);

    entry = armv7m_load_kernel(ARM_CPU(first_cpu),
                               machine->kernel_filename,
                               FLASH_SIZE);
    reset_args = (ARMV7MResetArgs) {
        .cpu = ARM_CPU(first_cpu),
        .reset_pc = entry,
        .reset_sp = (SRAM_BASE_ADDRESS + (SRAM_SIZE * 2) / 3),
    };
    qemu_register_reset(armv7m_reset,
                        g_memdup(&reset_args, sizeof(reset_args)));
}

static void netduinoplus2_machine_init(MachineClass *mc)
{
    mc->desc = "Netduino Plus 2 Machine (Cortex-M4)";
    mc->init = netduinoplus2_init;
}

DEFINE_MACHINE("netduinoplus2", netduinoplus2_machine_init)

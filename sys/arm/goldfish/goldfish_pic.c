/*-
 * Copyright (c) 2014 Alexander Tarasikov <alexander.tarasikov@gmail.com>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 */
#include <sys/cdefs.h>
__FBSDID("$FreeBSD$");

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/kernel.h>
#include <sys/module.h>
#include <sys/malloc.h>
#include <sys/rman.h>
#include <sys/timetc.h>
#include <machine/bus.h>
#include <machine/intr.h>

#include <dev/fdt/fdt_common.h>
#include <dev/ofw/openfirm.h>

#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>

enum {
	GOLDFISH_PIC_STATUS = 0x0,
	GOLDFISH_PIC_NUMBER = 0x4,
	GOLDFISH_PIC_DISABLE_ALL = 0x8,
	GOLDFISH_PIC_DISABLE = 0xc,
	GOLDFISH_PIC_ENABLE = 0x10,
};

struct goldfish_pic_softc {
	struct resource *	li_res;
	bus_space_tag_t		li_bst;
	bus_space_handle_t	li_bsh;
};

static int goldfish_pic_probe(device_t);
static int goldfish_pic_attach(device_t);
static void goldfish_pic_eoi(void *);

static struct goldfish_pic_softc *intc_softc = NULL;

#define	intc_read_4(reg)		\
    bus_space_read_4(intc_softc->li_bst, intc_softc->li_bsh, reg)
#define	intc_write_4(reg, val)		\
    bus_space_write_4(intc_softc->li_bst, intc_softc->li_bsh, reg, val)

static int
goldfish_pic_probe(device_t dev)
{

	if (!ofw_bus_is_compatible(dev, "arm,goldfish-pic"))
		return (ENXIO);

	device_set_desc(dev, "Goldfish PIC");
	return (BUS_PROBE_DEFAULT);
}

static int
goldfish_pic_attach(device_t dev)
{
	struct goldfish_pic_softc *sc = device_get_softc(dev);
	int rid = 0;

	if (intc_softc)
		return (ENXIO);

	sc->li_res = bus_alloc_resource_any(dev, SYS_RES_MEMORY, &rid,
	    RF_ACTIVE);
	if (!sc->li_res)
		return (ENXIO);

	sc->li_bst = rman_get_bustag(sc->li_res);
	sc->li_bsh = rman_get_bushandle(sc->li_res);
	intc_softc = sc;
	arm_post_filter = goldfish_pic_eoi;

	intc_write_4(GOLDFISH_PIC_DISABLE_ALL, 1);
	intc_write_4(GOLDFISH_PIC_DISABLE_ALL, 0);
	return (0);
}

static device_method_t goldfish_pic_methods[] = {
	DEVMETHOD(device_probe,		goldfish_pic_probe),
	DEVMETHOD(device_attach,	goldfish_pic_attach),
	{ 0, 0 }
};

static driver_t goldfish_pic_driver = {
	"pic",
	goldfish_pic_methods,
	sizeof(struct goldfish_pic_softc),
};

static devclass_t goldfish_pic_devclass;

DRIVER_MODULE(pic, simplebus, goldfish_pic_driver, goldfish_pic_devclass, 0, 0);

int
arm_get_next_irq(int last)
{
	if (!intc_softc)
		return (-1);

	int rc = intc_read_4(GOLDFISH_PIC_NUMBER);
	if (rc > 0)
		return (rc);

	return (-1);
}

void
arm_mask_irq(uintptr_t nb)
{
	if (intc_softc)
		intc_write_4(GOLDFISH_PIC_DISABLE, nb);
}

void
arm_unmask_irq(uintptr_t nb)
{
	if (intc_softc)
		intc_write_4(GOLDFISH_PIC_ENABLE, nb);
}

static void
goldfish_pic_eoi(void *data)
{
	/* NO-OP */
}

struct fdt_fixup_entry fdt_fixup_table[] = {
	{ NULL, NULL }
};

static int
fdt_pic_decode_ic(phandle_t node, pcell_t *intr, int *interrupt, int *trig,
    int *pol)
{
	if (!fdt_is_compatible(node, "arm,goldfish-pic"))
		return (ENXIO);

	*interrupt = fdt32_to_cpu(intr[0]);
	*trig = INTR_TRIGGER_CONFORM;
	*pol = INTR_POLARITY_CONFORM;
	return (0);
}

fdt_pic_decode_t fdt_pic_table[] = {
	&fdt_pic_decode_ic,
	NULL
};

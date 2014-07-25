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

enum pdev_opcode {
	PDEV_OP_INIT = 0x0,
	PDEV_OP_DONE = 0x0,
	PDEV_OP_REMOVE = 0x4,
	PDEV_OP_ADD = 0x8,
};

enum pdev_register {
	PDEV_OP = 0x0,
	PDEV_GET_NAME = 0x4,
	PDEV_NAME_LEN = 0x8,
	PDEV_ID = 0xc,
	PDEV_IO_BASE = 0x10,
	PDEV_IO_SIZE = 0x14,
	PDEV_IRQ = 0x18,
	PDEV_IRQ_COUNT = 0x1c,
};

struct goldfish_pdev_softc {
	struct resource *	li_res;
	bus_space_tag_t		li_bst;
	bus_space_handle_t	li_bsh;
	struct resource*	irq_res;
	void*			intr_hl;
};

static int goldfish_pdev_probe(device_t);
static int goldfish_pdev_attach(device_t);

static struct goldfish_pdev_softc *pdev_softc = NULL;

#define	pdev_read_4(reg)		\
    bus_space_read_4(pdev_softc->li_bst, pdev_softc->li_bsh, reg)
#define	pdev_write_4(reg, val)		\
    bus_space_write_4(pdev_softc->li_bst, pdev_softc->li_bsh, reg, val)

static int
goldfish_pdev_probe(device_t dev)
{

	if (!ofw_bus_is_compatible(dev, "arm,goldfish-pdev"))
		return (ENXIO);

	device_set_desc(dev, "Goldfish PDEV");
	return (BUS_PROBE_DEFAULT);
}

static void
goldfish_pdev_add(void) {
	uint32_t irq = 0;
	uint32_t irq_count = 0;
	uint32_t io_base = 0;
	uint32_t io_size = 0;
	uint32_t name_len = 0;
	char *name;

	io_base = pdev_read_4(PDEV_IO_BASE);
	irq_count = pdev_read_4(PDEV_IRQ_COUNT);
	name_len = pdev_read_4(PDEV_NAME_LEN);

	name = malloc((name_len + 1), M_DEVBUF, M_ZERO|M_NOWAIT);
	if (NULL == name) {
		name = "NULL";
	}
	else {
		pdev_write_4(PDEV_GET_NAME, (uint32_t)name);
		name[name_len] = '\0';
	}

	io_size = pdev_read_4(PDEV_IO_SIZE);
	if (irq_count)
		irq = pdev_read_4(PDEV_IRQ);

	printf("%s: io_base=%08x io_size=%08x, irq=%d, irq_count=%d, name=%s\n",
		__func__, io_base, io_size, irq, irq_count, name);
}

static int
goldfish_pdev_intr(void *arg)
{
	printf("%s: irq\n", __func__);
	for (;;) {
		switch (pdev_read_4(PDEV_OP)) {
			case PDEV_OP_DONE:
				printf("%s: done\n", __func__);
				return (FILTER_HANDLED);
			case PDEV_OP_REMOVE:
				printf("%s: remove device\n", __func__);
				break;
			case PDEV_OP_ADD:
				printf("%s: add device\n", __func__);
				goldfish_pdev_add();
				break;
			default:
				break;
		}
	}
	return (FILTER_HANDLED);
}

static int
goldfish_pdev_attach(device_t dev)
{
	struct goldfish_pdev_softc *sc = device_get_softc(dev);
	int li_rid = 0, irq_rid = 0;

	if (pdev_softc)
		return (ENXIO);

	sc->li_res = bus_alloc_resource_any(dev, SYS_RES_MEMORY, &li_rid, RF_ACTIVE);
	if (!sc->li_res)
		goto fail;

	sc->irq_res = bus_alloc_resource_any(dev, SYS_RES_IRQ, &irq_rid, RF_ACTIVE);
	if (!sc->irq_res)
		goto fail;

	if (bus_setup_intr(dev, sc->irq_res, INTR_TYPE_MISC,
		goldfish_pdev_intr, NULL, sc, &sc->intr_hl) != 0)
		goto fail;

	sc->li_bst = rman_get_bustag(sc->li_res);
	sc->li_bsh = rman_get_bushandle(sc->li_res);
	pdev_softc = sc;

	pdev_write_4(PDEV_OP, PDEV_OP_INIT);

	return (0);

fail:
	if (sc->irq_res)
		bus_release_resource(dev, SYS_RES_IRQ, irq_rid, sc->irq_res);

	if (sc->li_res)
		bus_release_resource(dev, SYS_RES_MEMORY, li_rid, sc->li_res);

	return (ENXIO);
}

static device_method_t goldfish_pdev_methods[] = {
	DEVMETHOD(device_probe,		goldfish_pdev_probe),
	DEVMETHOD(device_attach,	goldfish_pdev_attach),
	{ 0, 0 }
};

static driver_t goldfish_pdev_driver = {
	"pdev",
	goldfish_pdev_methods,
	sizeof(struct goldfish_pdev_softc),
};

static devclass_t goldfish_pdev_devclass;
DRIVER_MODULE(pdev, simplebus, goldfish_pdev_driver, goldfish_pdev_devclass, 0, 0);

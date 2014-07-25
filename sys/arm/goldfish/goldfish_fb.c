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
#include <sys/fbio.h>
#include <sys/consio.h>

#include <machine/bus.h>
#include <machine/fdt.h>
#include <machine/intr.h>
#include <machine/frame.h>

#include <dev/fdt/fdt_common.h>
#include <dev/ofw/openfirm.h>

#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>

#include <dev/vt/vt.h>
#include <dev/vt/colors/vt_termcolors.h>

#include "fb_if.h"

enum android_hal_pixel_format {
	/* default one, unless set explicitely by android's libcopyb it.
	 * they probably do it via qemu socket, because there is no register
	 * to control pixel format in the virtual framebuffer device */
	HAL_PIXEL_FORMAT_RGB_565 = 4,
};

enum goldfish_fbio_regs {
	GOLDFISH_FB_WIDTH = 0x0,
	GOLDFISH_FB_HEIGHT = 0x4,
	GOLDFISH_FB_FORMAT = 0x24,

	GOLDFISH_FB_SET_INT_ENABLE = 0x0c,
	GOLDFISH_FB_SET_BASE = 0x10,
	GOLDFISH_FB_SET_BLANK = 0x18,
};

enum goldfish_int_bits {
	FB_INT_BASE_UPDATE_DONE = 1U << 1,
};

struct goldfish_fbio_softc {
	struct resource *	li_res;
	bus_space_tag_t	li_bst;
	bus_space_handle_t	li_bsh;

	device_t dev;
	device_t sc_fbd;
	struct fb_info sc_info;
};

static int goldfish_fbio_probe(device_t);
static int goldfish_fbio_attach(device_t);

#define	fb_read_4(reg)		\
    bus_space_read_4(sc->li_bst, sc->li_bsh, reg)
#define	fb_write_4(reg, val)		\
    bus_space_write_4(sc->li_bst, sc->li_bsh, reg, val)

static int goldfish_fbio_probe(device_t dev)
{
	if (!ofw_bus_status_okay(dev))
		return (ENXIO);
	if (!ofw_bus_is_compatible(dev, "arm,goldfish-fbio"))
		return (ENXIO);

	device_set_desc(dev, "Goldfish Framebuffer");
	return (BUS_PROBE_DEFAULT);
}

static int
goldfish_fbio_intr(void *arg)
{
	return (FILTER_HANDLED);
}

static int
goldfish_fbio_attach(device_t dev)
{
	struct goldfish_fbio_softc *sc = device_get_softc(dev);
	int li_rid = 0;
	size_t fbmem_size = 0;
	uint32_t format = 0;

	sc->li_res = bus_alloc_resource_any(dev, SYS_RES_MEMORY, &li_rid, RF_ACTIVE);
	if (!sc->li_res)
		goto fail;

	sc->li_bst = rman_get_bustag(sc->li_res);
	sc->li_bsh = rman_get_bushandle(sc->li_res);

	format = fb_read_4(GOLDFISH_FB_FORMAT);
	if (format != HAL_PIXEL_FORMAT_RGB_565) {
		device_printf(dev, "unsupported pixel format %d\n", format);
		goto fail;
	}

	sc->sc_info.fb_width = fb_read_4(GOLDFISH_FB_WIDTH);
	sc->sc_info.fb_height = fb_read_4(GOLDFISH_FB_HEIGHT);
	sc->sc_info.fb_stride = 2 * sc->sc_info.fb_width;
	sc->sc_info.fb_bpp = sc->sc_info.fb_depth = 16;
	sc->sc_info.fb_size = sc->sc_info.fb_height * sc->sc_info.fb_stride;

	fbmem_size = round_page(sc->sc_info.fb_size);
	sc->sc_info.fb_vbase = (intptr_t)contigmalloc(fbmem_size, M_DEVBUF, M_ZERO,
		0, ~0, PAGE_SIZE, 0);
	if (!sc->sc_info.fb_vbase) {
		device_printf(dev, "failed to allocate memory for framebuffer\n");
		goto fail;
	}
	sc->sc_info.fb_pbase = (intptr_t)vtophys(sc->sc_info.fb_vbase);

	fb_write_4(GOLDFISH_FB_SET_BASE, sc->sc_info.fb_pbase);
	fb_write_4(GOLDFISH_FB_SET_BLANK, 0);
	fb_write_4(GOLDFISH_FB_SET_INT_ENABLE, FB_INT_BASE_UPDATE_DONE);
	memset((void*)sc->sc_info.fb_vbase, 0x0, fbmem_size);

	sc->sc_info.fb_name = device_get_nameunit(dev);

	/* Ask newbus to attach framebuffer device to me. */
	sc->sc_fbd = device_add_child(dev, "fbd", device_get_unit(dev));
	if (sc->sc_fbd == NULL) {
		device_printf(dev, "Can't attach fbd device\n");
		goto fail;
	}

	if (device_probe_and_attach(sc->sc_fbd) != 0) {
		device_printf(dev, "Failed to attach fbd device\n");
		goto fail;
	}

	return (0);

fail:
	printf("%s: failed\n", __func__);
	if (sc->li_res)
		bus_release_resource(dev, SYS_RES_MEMORY, li_rid, sc->li_res);

	return (ENXIO);
}

static struct fb_info *
goldfish_fb_getinfo(device_t dev)
{
	struct goldfish_fbio_softc *sc = device_get_softc(dev);

	return (&sc->sc_info);
}


static device_method_t goldfish_fbio_methods[] = {
	DEVMETHOD(device_probe,		goldfish_fbio_probe),
	DEVMETHOD(device_attach,	goldfish_fbio_attach),
	DEVMETHOD(fb_getinfo,		goldfish_fb_getinfo),
	DEVMETHOD_END
};

static driver_t goldfish_fbio_driver = {
	"fb",
	goldfish_fbio_methods,
	sizeof(struct goldfish_fbio_softc),
};

static devclass_t goldfish_fbio_devclass;
DRIVER_MODULE(fb, simplebus, goldfish_fbio_driver, goldfish_fbio_devclass, 0, 0);

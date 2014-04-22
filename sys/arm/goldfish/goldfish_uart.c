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
 */

#include <sys/cdefs.h>
__FBSDID("$FreeBSD$");

#include <sys/types.h>
#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/cons.h>
#include <sys/consio.h>
#include <sys/kernel.h>
#include <sys/module.h>
#include <sys/rman.h>

#include <machine/bus.h>
#include <machine/intr.h>

#include <dev/fdt/fdt_common.h>
#include <dev/ofw/openfirm.h>

#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>

enum goldfish_guart_regs {
    TTY_PUT_CHAR       = 0x00,
    TTY_BYTES_READY    = 0x04,
    TTY_CMD            = 0x08,

    TTY_DATA_PTR       = 0x10,
    TTY_DATA_LEN       = 0x14,
    TTY_DATA_PTR_HIGH  = 0x18,

    TTY_CMD_INT_DISABLE    = 0,
    TTY_CMD_INT_ENABLE     = 1,
    TTY_CMD_WRITE_BUFFER   = 2,
    TTY_CMD_READ_BUFFER    = 3,
};

struct goldfish_guart_softc {
	struct resource *	li_res;
	bus_space_tag_t		li_bst;
	bus_space_handle_t	li_bsh;
};

static int goldfish_guart_probe(device_t);
static int goldfish_guart_attach(device_t);

static struct goldfish_guart_softc *uart_softc = NULL;

#define	uart_read_4(reg)		\
    bus_space_read_4(uart_softc->li_bst, uart_softc->li_bsh, reg)
#define	uart_write_4(reg, val)		\
    bus_space_write_4(uart_softc->li_bst, uart_softc->li_bsh, reg, val)

static int
goldfish_guart_probe(device_t dev)
{

	if (!ofw_bus_is_compatible(dev, "arm,goldfish-uart"))
		return (ENXIO);

	device_set_desc(dev, "Goldfish (Android Emulator) UART");
	return (BUS_PROBE_DEFAULT);
}

static int
goldfish_guart_attach(device_t dev)
{
	struct goldfish_guart_softc *sc = device_get_softc(dev);
	int rid = 0;

	if (uart_softc)
		return (ENXIO);

	sc->li_res = bus_alloc_resource_any(dev, SYS_RES_MEMORY, &rid, 
	    RF_ACTIVE);
	if (!sc->li_res) {
		device_printf(dev, "could not alloc resources\n");
		return (ENXIO);
	}

	sc->li_bst = rman_get_bustag(sc->li_res);
	sc->li_bsh = rman_get_bushandle(sc->li_res);
	uart_softc = sc;
	return (0);
}

static device_method_t goldfish_guart_methods[] = {
	DEVMETHOD(device_probe,		goldfish_guart_probe),
	DEVMETHOD(device_attach,	goldfish_guart_attach),
	{ 0, 0 }
};

static driver_t goldfish_guart_driver = {
	"guart",
	goldfish_guart_methods,
	sizeof(struct goldfish_guart_softc),
};

static devclass_t goldfish_guart_devclass;

DRIVER_MODULE(guart, simplebus, goldfish_guart_driver, goldfish_guart_devclass, 0, 0);

static void
uart_setreg(uint32_t reg, uint32_t val)
{
	if (uart_softc) {
		uart_write_4(reg, val);
	}
}

static void
ub_putc(unsigned char c)
{
	if (c == '\n')
		ub_putc('\r');

	uart_setreg(TTY_PUT_CHAR, c);
}

static cn_probe_t	uart_cnprobe;
static cn_init_t	uart_cninit;
static cn_term_t	uart_cnterm;
static cn_getc_t	uart_cngetc;
static cn_putc_t	uart_cnputc;
static cn_grab_t	uart_cngrab;
static cn_ungrab_t	uart_cnungrab;

void
uart_cnputc(struct consdev *cp, int c)
{
	ub_putc(c);
}

int
uart_cngetc(struct consdev * cp)
{
	return 0;
}

static void
uart_cngrab(struct consdev *cp)
{
}

static void
uart_cnungrab(struct consdev *cp)
{
}


static void
uart_cnprobe(struct consdev *cp)
{
	sprintf(cp->cn_name, "uart_goldfish");
	cp->cn_pri = CN_NORMAL;
}

static void
uart_cninit(struct consdev *cp)
{
}

static void
uart_cnterm(struct consdev * cp)
{
}

CONSOLE_DRIVER(uart);

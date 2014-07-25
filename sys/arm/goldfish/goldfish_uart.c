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

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/conf.h>
#include <sys/cons.h>
#include <sys/tty.h>
#include <sys/rman.h>
#include <machine/bus.h>
#include <machine/intr.h>

#include <dev/uart/uart.h>
#include <dev/uart/uart_cpu.h>
#include <dev/uart/uart_bus.h>

#include "uart_if.h"

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

/*
 * Low-level UART interface.
 */
static int goldfish_probe(struct uart_bas *bas);
static void goldfish_init(struct uart_bas *bas, int, int, int, int);
static void goldfish_term(struct uart_bas *bas);
static void goldfish_putc(struct uart_bas *bas, int);
static int goldfish_rxready(struct uart_bas *bas);
static int goldfish_getc(struct uart_bas *bas, struct mtx *mtx);

extern SLIST_HEAD(uart_devinfo_list, uart_devinfo) uart_sysdevs;

struct uart_ops uart_goldfish_ops = {
	.probe = goldfish_probe,
	.init = goldfish_init,
	.term = goldfish_term,
	.putc = goldfish_putc,
	.rxready = goldfish_rxready,
	.getc = goldfish_getc,
};

static int
goldfish_probe(struct uart_bas *bas)
{
	return (0);
}

static void
goldfish_init(struct uart_bas *bas, int baudrate, int databits, int stopbits,
    int parity)
{
}

static void
goldfish_term(struct uart_bas *bas)
{
}

static void
goldfish_putc(struct uart_bas *bas, int c)
{
	uart_setreg(bas, TTY_PUT_CHAR, c);
}

#define READ_SUPPORT 1

static int
goldfish_rxready(struct uart_bas *bas)
{
#if READ_SUPPORT
	return (uart_getreg(bas, TTY_BYTES_READY));
#else
	return 0;
#endif
}

static int
goldfish_getc(struct uart_bas *bas, struct mtx *mtx)
{
	int c = 0;

	uart_lock(mtx);

	while (!uart_getreg(bas, TTY_BYTES_READY))
		;

#if READ_SUPPORT
	uart_setreg(bas, TTY_DATA_PTR, (int)&c);
	uart_setreg(bas, TTY_DATA_LEN, 1);
	uart_setreg(bas, TTY_CMD, TTY_CMD_READ_BUFFER);
#endif

	uart_unlock(mtx);
	return (c & 0xff);
}

static int goldfish_bus_probe(struct uart_softc *sc);
static int goldfish_bus_attach(struct uart_softc *sc);
static int goldfish_bus_flush(struct uart_softc *, int);
static int goldfish_bus_getsig(struct uart_softc *);
static int goldfish_bus_ioctl(struct uart_softc *, int, intptr_t);
static int goldfish_bus_ipend(struct uart_softc *);
static int goldfish_bus_param(struct uart_softc *, int, int, int, int);
static int goldfish_bus_receive(struct uart_softc *);
static int goldfish_bus_setsig(struct uart_softc *, int);
static int goldfish_bus_transmit(struct uart_softc *);

static kobj_method_t goldfish_methods[] = {
	KOBJMETHOD(uart_probe,		goldfish_bus_probe),
	KOBJMETHOD(uart_attach, 	goldfish_bus_attach),
	KOBJMETHOD(uart_flush,		goldfish_bus_flush),
	KOBJMETHOD(uart_getsig,		goldfish_bus_getsig),
	KOBJMETHOD(uart_ioctl,		goldfish_bus_ioctl),
	KOBJMETHOD(uart_ipend,		goldfish_bus_ipend),
	KOBJMETHOD(uart_param,		goldfish_bus_param),
	KOBJMETHOD(uart_receive,	goldfish_bus_receive),
	KOBJMETHOD(uart_setsig,		goldfish_bus_setsig),
	KOBJMETHOD(uart_transmit,	goldfish_bus_transmit),

	{0, 0 }
};

int
goldfish_bus_probe(struct uart_softc *sc)
{

	sc->sc_txfifosz = 16;
	sc->sc_rxfifosz = 16;

	return (0);
}

static int
goldfish_bus_attach(struct uart_softc *sc)
{

	sc->sc_hwiflow = 0;
	sc->sc_hwoflow = 0;

	return (0);
}

static int
goldfish_bus_transmit(struct uart_softc *sc)
{
	int i;

	//uart_lock(sc->sc_hwmtx);

	for (i = 0; i < sc->sc_txdatasz; i++) {
		goldfish_putc(&sc->sc_bas, sc->sc_txbuf[i]);
		//uart_barrier(&sc->sc_bas);
	}

	//sc->sc_txbusy = 1;
	//uart_unlock(sc->sc_hwmtx);

	return (0);
}

static int
goldfish_bus_setsig(struct uart_softc *sc, int sig)
{

	return (0);
}

static int
goldfish_bus_receive(struct uart_softc *sc)
{
	return (0);
}

static int
goldfish_bus_param(struct uart_softc *sc, int baudrate, int databits,
    int stopbits, int parity)
{
	return (0);
}

static int
goldfish_bus_ipend(struct uart_softc *sc)
{
	return (0);
}

static int
goldfish_bus_flush(struct uart_softc *sc, int what)
{

	return (0);
}

static int
goldfish_bus_getsig(struct uart_softc *sc)
{

	return (0);
}

static int
goldfish_bus_ioctl(struct uart_softc *sc, int request, intptr_t data)
{

	return (EINVAL);
}

struct uart_class uart_goldfish_class = {
	"goldfish class",
	goldfish_methods,
	1,
	.uc_ops = &uart_goldfish_ops,
	.uc_range = 8,
	.uc_rclk = 0,
};

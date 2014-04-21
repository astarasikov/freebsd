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
#include <sys/cons.h>
#include <sys/consio.h>
#include <sys/kernel.h>

//XXX: write a proper UART driver with input support and UART1 multiplexing

#define GOLDFISH_UART_BASE 0xff000000

enum goldfish_uart_regs {
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

static void
uart_setreg(uint32_t *bas, uint32_t val)
{
	*((volatile uint32_t *)(bas)) = val;
}

static void
ub_putc(unsigned char c)
{
	if (c == '\n')
		ub_putc('\r');

	uart_setreg((uint32_t *)(GOLDFISH_UART_BASE + TTY_PUT_CHAR), c);
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
	sprintf(cp->cn_name, "uart");
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

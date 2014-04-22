/*
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
#include <sys/kernel.h>
#include <sys/module.h>
#include <sys/malloc.h>
#include <sys/rman.h>
#include <sys/timeet.h>
#include <sys/timetc.h>
#include <sys/watchdog.h>
#include <machine/bus.h>
#include <machine/cpu.h>
#include <machine/frame.h>
#include <machine/intr.h>

#include <dev/fdt/fdt_common.h>
#include <dev/ofw/openfirm.h>
#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>

#include <machine/bus.h>
#include <machine/fdt.h>

//XXX: make it 64-bit :)

enum {
	GOLDFISH_TIMER_LOW = 0x0,
	GOLDFISH_TIMER_HIGH = 0x4,
	GOLDFISH_TIMER_ALARM_LOW = 0x08,
	GOLDFISH_TIMER_ALARM_HIGH = 0x0c,
	GOLDFISH_TIMER_CLEAR_INTERRUPT = 0x10,
	GOLDFISH_TIMER_CLEAR_ALARM = 0x14,
};

#define	DEFAULT_FREQUENCY	1000000
/*
 * QEMU seems to have problem with full frequency
 */
#define	DEFAULT_DIVISOR		16
#define	DEFAULT_CONTROL_DIV	TIMER_CONTROL_DIV16

struct goldfish_timer_softc {
	struct resource*	mem_res;
	struct resource*	irq_res;
	void*			intr_hl;
	uint32_t		sysclk_freq;
	bus_space_tag_t		bst;
	bus_space_handle_t	bsh;
	struct timecounter	tc;
	bool			et_enabled;
	struct eventtimer	et;
};

/* Read/Write macros for Timer used as timecounter */
#define goldfish_timer_tc_read_4(reg)		\
	bus_space_read_4(sc->bst, sc->bsh, reg)

#define goldfish_timer_tc_write_4(reg, val)	\
	bus_space_write_4(sc->bst, sc->bsh, reg, val)

static unsigned goldfish_timer_tc_get_timecount(struct timecounter *);

static unsigned
goldfish_timer_tc_get_timecount(struct timecounter *tc)
{
	struct goldfish_timer_softc *sc = tc->tc_priv;
	unsigned timeval = goldfish_timer_tc_read_4(GOLDFISH_TIMER_LOW);
	goldfish_timer_tc_read_4(GOLDFISH_TIMER_HIGH);
	return 0xffffffff - timeval;
}

static int
goldfish_timer_start(struct eventtimer *et, sbintime_t first, sbintime_t period)
{
	struct goldfish_timer_softc *sc = et->et_priv;
	uint32_t count;

	if (first != 0) {
		sc->et_enabled = 1;

		count = ((uint32_t)et->et_frequency * first) >> 32;

		goldfish_timer_tc_write_4(GOLDFISH_TIMER_ALARM_HIGH, 0);
		goldfish_timer_tc_write_4(GOLDFISH_TIMER_ALARM_LOW, count);

		return (0);
	} 

	if (period != 0) {
		panic("period");
	}

	return (EINVAL);
}

static int
goldfish_timer_stop(struct eventtimer *et)
{
	struct goldfish_timer_softc *sc = et->et_priv;

	sc->et_enabled = 0;
	goldfish_timer_tc_write_4(GOLDFISH_TIMER_CLEAR_ALARM, 1);

	return (0);
}

static int
goldfish_timer_intr(void *arg)
{
	struct goldfish_timer_softc *sc = arg;
	goldfish_timer_tc_write_4(GOLDFISH_TIMER_CLEAR_INTERRUPT, 1);

	if (sc->et_enabled) {
		if (sc->et.et_active) {
			sc->et.et_event_cb(&sc->et, sc->et.et_arg);
		}
	}

	return (FILTER_HANDLED);
}

static int
goldfish_timer_probe(device_t dev)
{

	if (ofw_bus_is_compatible(dev, "arm,goldfish-timer")) {
		device_set_desc(dev, "Goldfish System Timer");
		return (BUS_PROBE_DEFAULT);
	}

	return (ENXIO);
}

static int
goldfish_timer_attach(device_t dev)
{
	struct goldfish_timer_softc *sc = device_get_softc(dev);
	int rid = 0;

	sc->mem_res = bus_alloc_resource_any(dev, SYS_RES_MEMORY, &rid, RF_ACTIVE);
	if (sc->mem_res == NULL) {
		device_printf(dev, "could not allocate memory resource\n");
		return (ENXIO);
	}

	sc->bst = rman_get_bustag(sc->mem_res);
	sc->bsh = rman_get_bushandle(sc->mem_res);

	/* Request the IRQ resources */
	sc->irq_res =  bus_alloc_resource_any(dev, SYS_RES_IRQ, &rid, RF_ACTIVE);
	if (sc->irq_res == NULL) {
		device_printf(dev, "Error: could not allocate irq resources\n");
		return (ENXIO);
	}

	/* TODO: get frequency from FDT */
	sc->sysclk_freq = DEFAULT_FREQUENCY;

	/* Setup and enable the timer */
	if (bus_setup_intr(dev, sc->irq_res, INTR_TYPE_CLK,
			goldfish_timer_intr, NULL, sc,
			&sc->intr_hl) != 0) {
		bus_release_resource(dev, SYS_RES_IRQ, rid,
			sc->irq_res);
		device_printf(dev, "Unable to setup the clock irq handler.\n");
		return (ENXIO);
	}

	/*
	 * Timer 1, timecounter
	 */
	sc->tc.tc_frequency = DEFAULT_FREQUENCY;
	sc->tc.tc_name = "Goldfish Timecouter";
	sc->tc.tc_get_timecount = goldfish_timer_tc_get_timecount;
	sc->tc.tc_poll_pps = NULL;
	sc->tc.tc_counter_mask = ~0u;
	sc->tc.tc_quality = 200;
	sc->tc.tc_priv = sc;

	goldfish_timer_tc_write_4(GOLDFISH_TIMER_CLEAR_ALARM, 1);
	goldfish_timer_tc_write_4(GOLDFISH_TIMER_CLEAR_INTERRUPT, 1);

	tc_init(&sc->tc);

	/* 
	 * Timer 2, event timer
	 */
	sc->et_enabled = 0;
	sc->et.et_name = malloc(64, M_DEVBUF, M_NOWAIT | M_ZERO);
	sprintf(sc->et.et_name, "Goldfish Event Timer %d",
		device_get_unit(dev));
	sc->et.et_flags = ET_FLAGS_PERIODIC | ET_FLAGS_ONESHOT;
	sc->et.et_quality = 200;
	sc->et.et_frequency = sc->sysclk_freq / DEFAULT_DIVISOR;
	sc->et.et_min_period = (0x00000001LLU << 32) / sc->et.et_frequency;
	sc->et.et_max_period = (0xffffffffLLU << 32) / sc->et.et_frequency;
	sc->et.et_start = goldfish_timer_start;
	sc->et.et_stop = goldfish_timer_stop;
	sc->et.et_priv = sc;
	et_register(&sc->et);

	return (0);
}

static device_method_t goldfish_timer_methods[] = {
	DEVMETHOD(device_probe,		goldfish_timer_probe),
	DEVMETHOD(device_attach,	goldfish_timer_attach),
	{ 0, 0 }
};

static driver_t goldfish_timer_driver = {
	"timer",
	goldfish_timer_methods,
	sizeof(struct goldfish_timer_softc),
};

static devclass_t goldfish_timer_devclass;

DRIVER_MODULE(goldfish_timer, simplebus, goldfish_timer_driver, goldfish_timer_devclass, 0, 0);

void
DELAY(int usec)
{
	int32_t counts;
	uint32_t first, last;
	device_t timer_dev;
	struct goldfish_timer_softc *sc;

	timer_dev = devclass_get_device(goldfish_timer_devclass, 0);

	if (timer_dev == NULL) {
		/*
		 * Timer is not initialized yet
		 */
		for (; usec > 0; usec--)
			for (counts = 200; counts > 0; counts--)
				/* Prevent gcc from optimizing  out the loop */
				cpufunc_nullop();
		return;
	}

	sc = device_get_softc(timer_dev);

	/* Get the number of times to count */
	counts = usec * ((sc->tc.tc_frequency / 1000000) + 1);

	first = goldfish_timer_tc_get_timecount(&sc->tc);

	while (counts > 0) {
		last = goldfish_timer_tc_get_timecount(&sc->tc);
		if (last == first)
			continue;
		if (last>first) {
			counts -= (int32_t)(last - first);
		} else {
			counts -= (int32_t)((0xFFFFFFFF - first) + last);
		}
		first = last;
	}
}

void
cpu_initclocks(void)
{
}

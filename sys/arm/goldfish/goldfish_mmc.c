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
#include <sys/bio.h>
#include <sys/bus.h>
#include <sys/conf.h>
#include <sys/endian.h>
#include <sys/kernel.h>
#include <sys/kthread.h>
#include <sys/lock.h>
#include <sys/malloc.h>
#include <sys/module.h>
#include <sys/mutex.h>
#include <sys/queue.h>
#include <sys/resource.h>
#include <sys/rman.h>
#include <sys/time.h>
#include <sys/timetc.h>
#include <sys/watchdog.h>

#include <sys/kdb.h>

#include <machine/bus.h>
#include <machine/cpu.h>
#include <machine/cpufunc.h>
#include <machine/resource.h>
#include <machine/intr.h>

#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>

#include <dev/mmc/bridge.h>
#include <dev/mmc/mmcreg.h>
#include <dev/mmc/mmcbrvar.h>

enum {
	/* status register */
	GOLDFISH_MMC_INT_STATUS	        = 0x00,
	/* set this to enable IRQ */
	GOLDFISH_MMC_INT_ENABLE	        = 0x04,
	/* set this to specify buffer address */
	GOLDFISH_MMC_SET_BUFFER          = 0x08,

	/* MMC command number */
	GOLDFISH_MMC_CMD	                = 0x0C,

	/* MMC argument */
	GOLDFISH_MMC_ARG	                = 0x10,

	/* MMC response (or R2 bits 0 - 31) */
	GOLDFISH_MMC_RESP_0		        = 0x14,

	/* MMC R2 response bits 32 - 63 */
	GOLDFISH_MMC_RESP_1		        = 0x18,

	/* MMC R2 response bits 64 - 95 */
	GOLDFISH_MMC_RESP_2		        = 0x1C,

	/* MMC R2 response bits 96 - 127 */
	GOLDFISH_MMC_RESP_3		        = 0x20,

	GOLDFISH_MMC_BLOCK_LENGTH        = 0x24,
	GOLDFISH_MMC_BLOCK_COUNT         = 0x28,

	/* MMC state flags */
	GOLDFISH_MMC_STATE               = 0x2C,

	/* MMC_INT_STATUS bits */

	GOLDFISH_MMC_STAT_END_OF_CMD     = 1U << 0,
	GOLDFISH_MMC_STAT_END_OF_DATA    = 1U << 1,
	GOLDFISH_MMC_STAT_STATE_CHANGE   = 1U << 2,

	/* MMC_STATE bits */
	GOLDFISH_MMC_STATE_INSERTED     = 1U << 0,
	GOLDFISH_MMC_STATE_READ_ONLY    = 1U << 1,
};

/* goldfish mmc controller is modeled after OMAP */
enum goldfish_mmc_cmdtype_flags {
	GOLDFISH_CMDTYPE_BC = 0,
	GOLDFISH_CMDTYPE_BCR = 1,
	GOLDFISH_CMDTYPE_AC = 2,
	GOLDFISH_CMDTYPE_ADTC = 3,
};

#define	GOLDFISH_MMC_BLOCK_SIZE	2048

#ifdef DEBUG
#define debugf(fmt, args...) do { printf("%s(): ", __func__);   \
    printf(fmt,##args); } while (0)
#else
#define debugf(fmt, args...)
#endif

struct goldfish_mmc_dmamap_arg {
	bus_addr_t		gf_dma_busaddr;
};

struct goldfish_mmc_softc {
	device_t		gf_dev;
	struct mtx		gf_mtx;
	struct resource *	gf_mem_res;
	struct resource *	gf_irq_res;
	bus_space_tag_t		gf_bst;
	bus_space_handle_t	gf_bsh;
	void *			gf_intrhand;
	struct mmc_host		gf_host;
	struct mmc_request *	gf_req;
	struct mmc_data *	gf_data;
	uint32_t		gf_flags;
	int			gf_xfer_direction;
#define	DIRECTION_READ		0
#define	DIRECTION_WRITE		1
	int			gf_xfer_done;
	int			gf_bus_busy;
	bus_dma_tag_t		gf_dma_tag;
	bus_dmamap_t		gf_dma_map;
	bus_addr_t		gf_buffer_phys;
	void *			gf_buffer;
};

static int goldfish_mmc_probe(device_t);
static int goldfish_mmc_attach(device_t);
static int goldfish_mmc_detach(device_t);
static void goldfish_mmc_intr(void *);

static void goldfish_mmc_cmd(struct goldfish_mmc_softc *, struct mmc_command *);
static void goldfish_mmc_setup_xfer(struct goldfish_mmc_softc *, struct mmc_data *);
static void goldfish_mmc_xfer_done(struct goldfish_mmc_softc *sc);

static int goldfish_mmc_update_ios(device_t, device_t);
static int goldfish_mmc_request(device_t, device_t, struct mmc_request *);
static int goldfish_mmc_get_ro(device_t, device_t);
static int goldfish_mmc_acquire_host(device_t, device_t);
static int goldfish_mmc_release_host(device_t, device_t);

static void goldfish_mmc_dmamap_cb(void *, bus_dma_segment_t *, int, int);

#define	goldfish_mmc_lock(_sc)						\
    mtx_lock(&_sc->gf_mtx);
#define	goldfish_mmc_unlock(_sc)						\
    mtx_unlock(&_sc->gf_mtx);
#define	goldfish_mmc_read_4(_sc, _reg)					\
    bus_space_read_4(_sc->gf_bst, _sc->gf_bsh, _reg)
#define	goldfish_mmc_write_4(_sc, _reg, _value)				\
    bus_space_write_4(_sc->gf_bst, _sc->gf_bsh, _reg, _value)

static int
goldfish_mmc_probe(device_t dev)
{
	if (!ofw_bus_status_okay(dev))
		return (ENXIO);

	if (!ofw_bus_is_compatible(dev, "arm,goldfish-mmc"))
		return (ENXIO);

	device_set_desc(dev, "Goldfish MMC controller");
	return (BUS_PROBE_DEFAULT);
}

static int
goldfish_mmc_attach(device_t dev)
{
	struct goldfish_mmc_softc *sc = device_get_softc(dev);
	struct goldfish_mmc_dmamap_arg ctx;
	device_t child;
	int mem_rid = 0, irq_rid = 0, err;

	sc->gf_dev = dev;
	sc->gf_req = NULL;

	mtx_init(&sc->gf_mtx, "goldfish_mmc", "gf_mmc", MTX_DEF);

	sc->gf_mem_res = bus_alloc_resource_any(dev, SYS_RES_MEMORY, &mem_rid,
	    RF_ACTIVE);
	if (!sc->gf_mem_res) {
		device_printf(dev, "cannot allocate memory window\n");
		err = ENXIO;
		goto fail;
	}

	sc->gf_bst = rman_get_bustag(sc->gf_mem_res);
	sc->gf_bsh = rman_get_bushandle(sc->gf_mem_res);

	sc->gf_irq_res = bus_alloc_resource_any(dev, SYS_RES_IRQ, &irq_rid,
	    RF_ACTIVE);
	if (!sc->gf_irq_res) {
		device_printf(dev, "cannot allocate interrupt\n");
		err = ENXIO;
		goto fail;
	}

	if (bus_setup_intr(dev, sc->gf_irq_res, INTR_TYPE_MISC | INTR_MPSAFE,
	    NULL, goldfish_mmc_intr, sc, &sc->gf_intrhand))
	{
		device_printf(dev, "cannot setup interrupt handler\n");
		err = ENXIO;
		goto fail;
	}

	sc->gf_host.f_min = 400000;
	sc->gf_host.f_max = 24000000;
	sc->gf_host.host_ocr = MMC_OCR_320_330 | MMC_OCR_330_340;
	sc->gf_host.caps = MMC_CAP_4_BIT_DATA;

	/* Alloc DMA memory */
	err = bus_dma_tag_create(
	    bus_get_dma_tag(sc->gf_dev),
	    4, 0,			/* alignment, boundary */
	    BUS_SPACE_MAXADDR_32BIT,	/* lowaddr */
	    BUS_SPACE_MAXADDR,		/* highaddr */
	    NULL, NULL,			/* filter, filterarg */
	    GOLDFISH_MMC_BLOCK_SIZE, 1,	/* maxsize, nsegments */
	    GOLDFISH_MMC_BLOCK_SIZE, 0,	/* maxsegsize, flags */
	    NULL, NULL,			/* lockfunc, lockarg */
	    &sc->gf_dma_tag);

	err = bus_dmamem_alloc(sc->gf_dma_tag, (void **)&sc->gf_buffer,
	    0, &sc->gf_dma_map);
	if (err) {
		device_printf(dev, "cannot allocate framebuffer\n");
		goto fail;
	}

	err = bus_dmamap_load(sc->gf_dma_tag, sc->gf_dma_map, sc->gf_buffer,
	    GOLDFISH_MMC_BLOCK_SIZE, goldfish_mmc_dmamap_cb, &ctx, BUS_DMA_NOWAIT);
	if (err) {
		device_printf(dev, "cannot load DMA map\n");
		goto fail;
	}

	sc->gf_buffer_phys = ctx.gf_dma_busaddr;

	child = device_add_child(dev, "mmc", -1);
	if (!child) {
		debugf("attaching MMC bus failed!\n");
		err = ENXIO;
		goto fail;
	}

	if (bus_generic_probe(dev)) {
		goto fail;
	}

	if (bus_generic_attach(dev)) {
		goto fail;
	}

	goldfish_mmc_write_4(sc, GOLDFISH_MMC_SET_BUFFER,
		sc->gf_buffer_phys);
	goldfish_mmc_write_4(sc, GOLDFISH_MMC_INT_ENABLE,
		GOLDFISH_MMC_STAT_END_OF_CMD
		| GOLDFISH_MMC_STAT_END_OF_DATA
		| GOLDFISH_MMC_STAT_STATE_CHANGE);

	return (0);

fail:
	mtx_destroy(&sc->gf_mtx);
	if (sc->gf_intrhand)
		bus_teardown_intr(dev, sc->gf_irq_res, sc->gf_intrhand);
	if (sc->gf_irq_res)
		bus_release_resource(dev, SYS_RES_IRQ, irq_rid, sc->gf_irq_res);
	if (sc->gf_mem_res)
		bus_release_resource(dev, SYS_RES_MEMORY, mem_rid, sc->gf_mem_res);
	return (err);
}

static int
goldfish_mmc_detach(device_t dev)
{
	return (EBUSY);
}

static void
goldfish_mmc_cmd_done(struct goldfish_mmc_softc *sc)
{
	struct mmc_command *cmd;

	if (!sc || !sc->gf_req || !sc->gf_req->cmd) {
		return;
	}

	cmd = sc->gf_req->cmd;

	if (cmd->flags & MMC_RSP_136) {
		cmd->resp[3] = goldfish_mmc_read_4(sc, GOLDFISH_MMC_RESP_0);
		cmd->resp[2] = goldfish_mmc_read_4(sc, GOLDFISH_MMC_RESP_1);
		cmd->resp[1] = goldfish_mmc_read_4(sc, GOLDFISH_MMC_RESP_2);
		cmd->resp[0] = goldfish_mmc_read_4(sc, GOLDFISH_MMC_RESP_3);
	}
	else {
		cmd->resp[0] = goldfish_mmc_read_4(sc, GOLDFISH_MMC_RESP_0);
	}
	cmd->error = MMC_ERR_NONE;

	if (cmd->data){
		goldfish_mmc_setup_xfer(sc, cmd->data);
		goldfish_mmc_xfer_done(sc);
	}

	if (sc->gf_req) {
		sc->gf_req->done(sc->gf_req);
		sc->gf_req = NULL;
	}
}

static void
goldfish_mmc_end_of_data(struct goldfish_mmc_softc *sc)
{
	goldfish_mmc_xfer_done(sc);
	if (sc->gf_req) {
		sc->gf_req->cmd->error = MMC_ERR_NONE;
		sc->gf_req->done(sc->gf_req);
		sc->gf_req = NULL;
	}
}

static void
goldfish_mmc_intr(void *arg)
{
	struct goldfish_mmc_softc *sc = (struct goldfish_mmc_softc *)arg;
	uint32_t status;
	int end_command = 0, end_transfer = 0, state_changed = 0;

	goldfish_mmc_lock(sc);

	while ((status = goldfish_mmc_read_4(sc, GOLDFISH_MMC_INT_STATUS)) != 0)
	{
		goldfish_mmc_write_4(sc, GOLDFISH_MMC_INT_STATUS, status);

		if (status & GOLDFISH_MMC_STAT_END_OF_CMD) {
			end_command = 1;
		}
		if (status & GOLDFISH_MMC_STAT_END_OF_DATA) {
			end_transfer = 1;
		}
		if (status & GOLDFISH_MMC_STAT_STATE_CHANGE) {
			state_changed = 1;
		}
	}

	if (end_command) {
		goldfish_mmc_cmd_done(sc);
	}

	if (end_transfer) {
		goldfish_mmc_end_of_data(sc);
	}

	if (state_changed)
	{
		debugf("%s: state changed\n", __func__);
	}
	goldfish_mmc_unlock(sc);
}

static int
goldfish_mmc_request(device_t bus, device_t child, struct mmc_request *req)
{
	struct goldfish_mmc_softc *sc = device_get_softc(bus);

	if (!sc) {
		return (EBUSY);
	}

	if (!req || !req->cmd) {
		debugf("%s: REQ is NULL\n", __func__);
		return (EBUSY);
	}

	goldfish_mmc_lock(sc);
	if (sc->gf_req) {
		debugf("%s: BUSY\n", __func__);
		return (EBUSY);
	}

	sc->gf_req = req;

	if (req->cmd->data) {
		goldfish_mmc_setup_xfer(sc, req->cmd->data);
		goldfish_mmc_xfer_done(sc);
	}

	goldfish_mmc_cmd(sc, req->cmd);
	goldfish_mmc_unlock(sc);

	return (0);
}

static void
goldfish_mmc_cmd(struct goldfish_mmc_softc *sc, struct mmc_command *cmd)
{
	uint32_t cmdreg = 0;
	uint32_t cmdtype = 0;

	if ((cmd->opcode == 6) && (cmd->arg == 0xffffff)) {
		//Android Emulator is using 0xfffff1 constant
		//as a magic value to return the response
		cmd->arg = 0xfffff1;
	}

	uint32_t resptype = 0;
	if (cmd->flags & MMC_RSP_PRESENT) {
		switch (MMC_RSP(cmd->flags)) {
		case MMC_RSP_R1:
		case MMC_RSP_R1B:

		case MMC_RSP_R6:
			resptype = 1;
			break;
		case MMC_RSP_R2:
			resptype = 2;
			break;
		case MMC_RSP_R3:
			resptype = 3;
			break;
		case MMC_RSP_NONE:
			break;
		default:
			debugf("%s: invalid response %lx\n", __func__, MMC_RSP(cmd->flags));
			break;
		}
	}

	if (cmd->flags & MMC_CMD_ADTC) {
		cmdtype = GOLDFISH_CMDTYPE_ADTC;
	}
	else if (cmd->flags & MMC_CMD_BC) {
		cmdtype = GOLDFISH_CMDTYPE_BC;
	}
	else if (cmd->flags & MMC_CMD_BCR) {
		cmdtype = GOLDFISH_CMDTYPE_BCR;
	}
	else {
		cmdtype = GOLDFISH_CMDTYPE_AC;
	}

	cmdreg = cmd->opcode | (resptype << 8) | (cmdtype << 12);

	if (cmd->flags & MMC_RSP_BUSY) {
		cmdreg |= 1 << 11;
	}

	if (cmd->data && !(cmd->data->flags & MMC_DATA_WRITE)) {
		cmdreg |= 1 << 15;
	}

	goldfish_mmc_write_4(sc, GOLDFISH_MMC_ARG, cmd->arg);
	goldfish_mmc_write_4(sc, GOLDFISH_MMC_CMD, cmdreg);
}

static void
goldfish_mmc_xfer_done(struct goldfish_mmc_softc *sc)
{
#define CHECK_PTR(ptr) if (!ptr) { goto done; }
	CHECK_PTR(sc->gf_req);
	CHECK_PTR(sc->gf_req->cmd);
	CHECK_PTR(sc->gf_req->cmd->data);
	CHECK_PTR(sc->gf_req->cmd->data->data);
#undef CHECK_PTR

	void *resp_ptr = sc->gf_req->cmd->data->data;
	if (sc->gf_xfer_direction == DIRECTION_WRITE) {
		memcpy(sc->gf_buffer, resp_ptr, sc->gf_req->cmd->data->len);
	}
	else {
		memcpy(resp_ptr, sc->gf_buffer, sc->gf_req->cmd->data->len);
	}

done:
	return;
}

static void
goldfish_mmc_setup_xfer(struct goldfish_mmc_softc *sc, struct mmc_data *data)
{
	sc->gf_data = data;
	sc->gf_xfer_done = 0;

	if (data->data == NULL) {
		goldfish_mmc_write_4(sc, GOLDFISH_MMC_BLOCK_LENGTH, 0);
		goldfish_mmc_write_4(sc, GOLDFISH_MMC_BLOCK_COUNT, 0);
		return;
	}

	if (data->flags & MMC_DATA_READ) {
		sc->gf_xfer_direction = DIRECTION_READ;
	}

	if (data->flags & MMC_DATA_WRITE) {
		sc->gf_xfer_direction = DIRECTION_WRITE;
	}

	goldfish_mmc_write_4(sc, GOLDFISH_MMC_BLOCK_LENGTH, data->len - 1);
	goldfish_mmc_write_4(sc, GOLDFISH_MMC_BLOCK_COUNT, 0);
}

static int
goldfish_mmc_read_ivar(device_t bus, device_t child, int which,
    uintptr_t *result)
{

	struct goldfish_mmc_softc *sc = device_get_softc(bus);

	switch (which) {
	case MMCBR_IVAR_BUS_MODE:
		*(int *)result = sc->gf_host.ios.bus_mode;
		break;
	case MMCBR_IVAR_BUS_WIDTH:
		*(int *)result = sc->gf_host.ios.bus_width;
		break;
	case MMCBR_IVAR_CHIP_SELECT:
		*(int *)result = sc->gf_host.ios.chip_select;
		break;
	case MMCBR_IVAR_CLOCK:
		*(int *)result = sc->gf_host.ios.clock;
		break;
	case MMCBR_IVAR_F_MIN:
		*(int *)result = sc->gf_host.f_min;
		break;
	case MMCBR_IVAR_F_MAX:
		*(int *)result = sc->gf_host.f_max;
		break;
	case MMCBR_IVAR_HOST_OCR:
		*(int *)result = sc->gf_host.host_ocr;
		break;
	case MMCBR_IVAR_MODE:
		*(int *)result = sc->gf_host.mode;
		break;
	case MMCBR_IVAR_OCR:
		*(int *)result = sc->gf_host.ocr;
		break;
	case MMCBR_IVAR_POWER_MODE:
		*(int *)result = sc->gf_host.ios.power_mode;
		break;
	case MMCBR_IVAR_VDD:
		*(int *)result = sc->gf_host.ios.vdd;
		break;
	case MMCBR_IVAR_CAPS:
		*(int *)result = sc->gf_host.caps;
		break;
	case MMCBR_IVAR_MAX_DATA:
		*(int *)result = 1;
		break;
	default:
		return (EINVAL);
	}

	return (0);
}

static int
goldfish_mmc_write_ivar(device_t bus, device_t child, int which,
    uintptr_t value)
{

	struct goldfish_mmc_softc *sc = device_get_softc(bus);

	switch (which) {
	case MMCBR_IVAR_BUS_MODE:
		sc->gf_host.ios.bus_mode = value;
		break;
	case MMCBR_IVAR_BUS_WIDTH:
		sc->gf_host.ios.bus_width = value;
		break;
	case MMCBR_IVAR_CHIP_SELECT:
		sc->gf_host.ios.chip_select = value;
		break;
	case MMCBR_IVAR_CLOCK:
		sc->gf_host.ios.clock = value;
		break;
	case MMCBR_IVAR_MODE:
		sc->gf_host.mode = value;
		break;
	case MMCBR_IVAR_OCR:
		sc->gf_host.ocr = value;
		break;
	case MMCBR_IVAR_POWER_MODE:
		sc->gf_host.ios.power_mode = value;
		break;
	case MMCBR_IVAR_VDD:
		sc->gf_host.ios.vdd = value;
		break;
	/* These are read-only */
	case MMCBR_IVAR_CAPS:
	case MMCBR_IVAR_HOST_OCR:
	case MMCBR_IVAR_F_MIN:
	case MMCBR_IVAR_F_MAX:
	case MMCBR_IVAR_MAX_DATA:
	default:
		return (EINVAL);
	}
	return (0);
}

static int
goldfish_mmc_update_ios(device_t bus, device_t child)
{
	return (0);
}

static int
goldfish_mmc_get_ro(device_t bus, device_t child)
{
	struct goldfish_mmc_softc *sc = device_get_softc(bus);
	uint32_t state = goldfish_mmc_read_4(sc, GOLDFISH_MMC_STATE);
	return (!!(state & GOLDFISH_MMC_STATE_READ_ONLY));
}

static int
goldfish_mmc_acquire_host(device_t bus, device_t child)
{
	struct goldfish_mmc_softc *sc = device_get_softc(bus);
	int error = 0;

	goldfish_mmc_lock(sc);
	while (sc->gf_bus_busy)
		error = mtx_sleep(sc, &sc->gf_mtx, PZERO, "mmcah", 0);

	sc->gf_bus_busy++;
	goldfish_mmc_unlock(sc);
	return (error);
}

static int
goldfish_mmc_release_host(device_t bus, device_t child)
{
	struct goldfish_mmc_softc *sc = device_get_softc(bus);

	goldfish_mmc_lock(sc);
	sc->gf_bus_busy--;
	wakeup(sc);
	goldfish_mmc_unlock(sc);
	return (0);
}

static void
goldfish_mmc_dmamap_cb(void *arg, bus_dma_segment_t *segs, int nseg, int err)
{
	struct goldfish_mmc_dmamap_arg *ctx;

	if (err)
		return;

	ctx = (struct goldfish_mmc_dmamap_arg *)arg;
	ctx->gf_dma_busaddr = segs[0].ds_addr;
}

static device_method_t goldfish_mmc_methods[] = {
	/* Device interface */
	DEVMETHOD(device_probe,		goldfish_mmc_probe),
	DEVMETHOD(device_attach,	goldfish_mmc_attach),
	DEVMETHOD(device_detach,	goldfish_mmc_detach),

	/* Bus interface */
	DEVMETHOD(bus_read_ivar,	goldfish_mmc_read_ivar),
	DEVMETHOD(bus_write_ivar,	goldfish_mmc_write_ivar),

	/* MMC bridge interface */
	DEVMETHOD(mmcbr_update_ios,	goldfish_mmc_update_ios),
	DEVMETHOD(mmcbr_request,	goldfish_mmc_request),
	DEVMETHOD(mmcbr_get_ro,		goldfish_mmc_get_ro),
	DEVMETHOD(mmcbr_acquire_host,	goldfish_mmc_acquire_host),
	DEVMETHOD(mmcbr_release_host,	goldfish_mmc_release_host),

	{ 0, 0 }
};

static devclass_t goldfish_mmc_devclass;

static driver_t goldfish_mmc_driver = {
	"goldfish_mmc",
	goldfish_mmc_methods,
	sizeof(struct goldfish_mmc_softc),
};

DRIVER_MODULE(goldfish_mmc, simplebus, goldfish_mmc_driver, goldfish_mmc_devclass, 0, 0);

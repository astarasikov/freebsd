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

#include <dev/fb/fbreg.h>
#include <dev/syscons/syscons.h>

#include <dev/fdt/fdt_common.h>
#include <dev/ofw/openfirm.h>

#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>

/******************************************************************************
 * virtual MMIO framebuffer device
 *****************************************************************************/
enum {
	FONT_HEIGHT = 16,
};

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
	struct resource*	irq_res;
	void*	intr_hl;
};

struct video_adapter_softc {
	/* Videoadpater part */
	video_adapter_t	va;

	intptr_t	fb_vaddr;
	intptr_t	fb_paddr;
	unsigned int	fb_size;

	int		bpp;
	int		depth;
	unsigned int	height;
	unsigned int	width;
	unsigned int	stride;

	unsigned int	xmargin;
	unsigned int	ymargin;

	unsigned char	*font;
	int		initialized;
};

static uint16_t colors[16] = {
	0x0000,	/* black */
	0x001f,	/* blue */
	0x07e0,	/* green */
	0x07ff,	/* cyan */
	0xf800,	/* red */
	0xf81f,	/* magenta */
	0x3800,	/* brown */
	0xc618,	/* light grey */
	0xc618,	/* XXX: dark grey */
	0x001f,	/* XXX: light blue */
	0x07e0,	/* XXX: light green */
	0x07ff,	/* XXX: light cyan */
	0xf800,	/* XXX: light red */
	0xf81f,	/* XXX: light magenta */
	0xffe0,	/* yellow */
	0xffff,	/* white */
};

static int goldfish_fbio_probe(device_t);
static int goldfish_fbio_attach(device_t);

static struct goldfish_fbio_softc *fb_softc = NULL;
static struct video_adapter_softc va_softc;

#define	fb_read_4(reg)		\
    bus_space_read_4(fb_softc->li_bst, fb_softc->li_bsh, reg)
#define	fb_write_4(reg, val)		\
    bus_space_write_4(fb_softc->li_bst, fb_softc->li_bsh, reg, val)

static int goldfish_fbio_probe(device_t dev)
{
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

#define TT printf("%s: %d\n", __func__, __LINE__)

static int
goldfish_fbio_attach(device_t dev)
{
	struct goldfish_fbio_softc *sc = device_get_softc(dev);
	int li_rid = 0, irq_rid = 0;
	size_t fbmem_size = 0;
	uint32_t format = 0;

	if (fb_softc)
		return (ENXIO);

	sc->li_res = bus_alloc_resource_any(dev, SYS_RES_MEMORY, &li_rid, RF_ACTIVE);
	if (!sc->li_res)
		goto fail;

	sc->irq_res = bus_alloc_resource_any(dev, SYS_RES_IRQ, &irq_rid, RF_ACTIVE);
	if (!sc->irq_res)
		goto fail;

	if (bus_setup_intr(dev, sc->irq_res, INTR_TYPE_MISC,
			goldfish_fbio_intr, NULL, sc, &sc->intr_hl) != 0)
		goto fail;

	sc->li_bst = rman_get_bustag(sc->li_res);
	sc->li_bsh = rman_get_bushandle(sc->li_res);
	fb_softc = sc;

	format = fb_read_4(GOLDFISH_FB_FORMAT);
	if (format != HAL_PIXEL_FORMAT_RGB_565) {
		device_printf(dev, "unsupported pixel format %d\n", format);
		TT;
		goto fail;
	}

	va_softc.width = fb_read_4(GOLDFISH_FB_WIDTH);
	va_softc.height = fb_read_4(GOLDFISH_FB_HEIGHT);
	va_softc.bpp = 2;
	va_softc.depth = 8 * va_softc.bpp;
	va_softc.stride = va_softc.bpp * va_softc.width;

	fbmem_size = round_page(va_softc.width * va_softc.height * va_softc.bpp);
	va_softc.fb_vaddr = (uint32_t)contigmalloc(fbmem_size, M_DEVBUF, M_ZERO,
		0, ~0, PAGE_SIZE, 0);
	if (!va_softc.fb_vaddr) {
		TT;
		goto fail;
	}
	va_softc.fb_paddr = vtophys(va_softc.fb_vaddr);

#if 1
	int err = ENXIO;
	err = sc_attach_unit(device_get_unit(dev),
		device_get_flags(dev) | SC_AUTODETECT_KBD);
	if (err) {
		device_printf(dev, "failed to attach syscons\n");
		TT;
		goto fail;
	}
#endif

	fb_write_4(GOLDFISH_FB_SET_BASE, va_softc.fb_paddr);
	fb_write_4(GOLDFISH_FB_SET_BLANK, 0);
	fb_write_4(GOLDFISH_FB_SET_INT_ENABLE, FB_INT_BASE_UPDATE_DONE);
	memset((void*)va_softc.fb_vaddr, 0xff, fbmem_size);

	return (0);

fail:
	printf("%s: failed\n", __func__);
	if (sc->irq_res)
		bus_release_resource(dev, SYS_RES_IRQ, irq_rid, sc->irq_res);

	if (sc->li_res)
		bus_release_resource(dev, SYS_RES_MEMORY, li_rid, sc->li_res); 

	return (ENXIO);
}

static device_method_t goldfish_fbio_methods[] = {
	DEVMETHOD(device_probe,		goldfish_fbio_probe),
	DEVMETHOD(device_attach,	goldfish_fbio_attach),
	DEVMETHOD_END
};

static driver_t goldfish_fbio_driver = {
	"goldfish_fbio",
	goldfish_fbio_methods,
	sizeof(struct goldfish_fbio_softc),
};

static devclass_t goldfish_fbio_devclass;
DRIVER_MODULE(goldfish_fbio, fdtbus, goldfish_fbio_driver, goldfish_fbio_devclass, 0, 0);
MODULE_DEPEND(goldfish_fbio, syscons, 1, 1, 1);

/******************************************************************************
 * video_adapter_t implementation
 *****************************************************************************/
/*
 * Video driver routines and glue.
 */
static int			goldfish_fb_configure(int);
static vi_probe_t		goldfish_fb_probe;
static vi_init_t		goldfish_fb_init;
static vi_get_info_t		goldfish_fb_get_info;
static vi_query_mode_t		goldfish_fb_query_mode;
static vi_set_mode_t		goldfish_fb_set_mode;
static vi_save_font_t		goldfish_fb_save_font;
static vi_load_font_t		goldfish_fb_load_font;
static vi_show_font_t		goldfish_fb_show_font;
static vi_save_palette_t	goldfish_fb_save_palette;
static vi_load_palette_t	goldfish_fb_load_palette;
static vi_set_border_t		goldfish_fb_set_border;
static vi_save_state_t		goldfish_fb_save_state;
static vi_load_state_t		goldfish_fb_load_state;
static vi_set_win_org_t		goldfish_fb_set_win_org;
static vi_read_hw_cursor_t	goldfish_fb_read_hw_cursor;
static vi_set_hw_cursor_t	goldfish_fb_set_hw_cursor;
static vi_set_hw_cursor_shape_t	goldfish_fb_set_hw_cursor_shape;
static vi_blank_display_t	goldfish_fb_blank_display;
static vi_mmap_t		goldfish_fb_mmap;
static vi_ioctl_t		goldfish_fb_ioctl;
static vi_clear_t		goldfish_fb_clear;
static vi_fill_rect_t		goldfish_fb_fill_rect;
static vi_bitblt_t		goldfish_fb_bitblt;
static vi_diag_t		goldfish_fb_diag;
static vi_save_cursor_palette_t	goldfish_fb_save_cursor_palette;
static vi_load_cursor_palette_t	goldfish_fb_load_cursor_palette;
static vi_copy_t		goldfish_fb_copy;
static vi_putp_t		goldfish_fb_putp;
static vi_putc_t		goldfish_fb_putc;
static vi_puts_t		goldfish_fb_puts;
static vi_putm_t		goldfish_fb_putm;

static video_switch_t goldfish_fbvidsw = {
	.probe			= goldfish_fb_probe,
	.init			= goldfish_fb_init,
	.get_info		= goldfish_fb_get_info,
	.query_mode		= goldfish_fb_query_mode,
	.set_mode		= goldfish_fb_set_mode,
	.save_font		= goldfish_fb_save_font,
	.load_font		= goldfish_fb_load_font,
	.show_font		= goldfish_fb_show_font,
	.save_palette		= goldfish_fb_save_palette,
	.load_palette		= goldfish_fb_load_palette,
	.set_border		= goldfish_fb_set_border,
	.save_state		= goldfish_fb_save_state,
	.load_state		= goldfish_fb_load_state,
	.set_win_org		= goldfish_fb_set_win_org,
	.read_hw_cursor		= goldfish_fb_read_hw_cursor,
	.set_hw_cursor		= goldfish_fb_set_hw_cursor,
	.set_hw_cursor_shape	= goldfish_fb_set_hw_cursor_shape,
	.blank_display		= goldfish_fb_blank_display,
	.mmap			= goldfish_fb_mmap,
	.ioctl			= goldfish_fb_ioctl,
	.clear			= goldfish_fb_clear,
	.fill_rect		= goldfish_fb_fill_rect,
	.bitblt			= goldfish_fb_bitblt,
	.diag			= goldfish_fb_diag,
	.save_cursor_palette	= goldfish_fb_save_cursor_palette,
	.load_cursor_palette	= goldfish_fb_load_cursor_palette,
	.copy			= goldfish_fb_copy,
	.putp			= goldfish_fb_putp,
	.putc			= goldfish_fb_putc,
	.puts			= goldfish_fb_puts,
	.putm			= goldfish_fb_putm,
};

VIDEO_DRIVER(goldfish_fb, goldfish_fbvidsw, goldfish_fb_configure);

extern sc_rndr_sw_t txtrndrsw;
RENDERER(goldfish_fb, 0, txtrndrsw, gfb_set);
RENDERER_MODULE(goldfish_fb, gfb_set);

static uint16_t goldfish_fb_static_window[ROW*COL];
extern u_char dflt_font_16[];

static int
goldfish_fb_configure(int flags)
{
	struct video_adapter_softc *sc;

	sc = &va_softc;

	if (sc->initialized || !fb_softc)
		return 0;

	goldfish_fb_init(0, &sc->va, 0);

	sc->initialized = 1;

	return (0);
}

static int
goldfish_fb_probe(int unit, video_adapter_t **adp, void *arg, int flags)
{
	return (0);
}

static int
goldfish_fb_init(int unit, video_adapter_t *adp, int flags)
{
	struct video_adapter_softc *sc;
	video_info_t *vi;

	sc = (struct video_adapter_softc *)adp;
	vi = &adp->va_info;

	vid_init_struct(adp, "goldfish_fb", -1, unit);

	sc->font = dflt_font_16;
	vi->vi_cheight = FONT_HEIGHT;
	vi->vi_cwidth = 8;
	vi->vi_width = sc->width/8;
	vi->vi_height = sc->height/vi->vi_cheight;

	/*
	 * Clamp width/height to syscons maximums
	 */
	if (vi->vi_width > COL)
		vi->vi_width = COL;
	if (vi->vi_height > ROW)
		vi->vi_height = ROW;

	sc->xmargin = (sc->width - (vi->vi_width * vi->vi_cwidth)) / 2;
	sc->ymargin = (sc->height - (vi->vi_height * vi->vi_cheight))/2;

	adp->va_window = (vm_offset_t) goldfish_fb_static_window;
	adp->va_flags |= V_ADP_FONT | V_ADP_INITIALIZED /* | V_ADP_COLOR | V_ADP_MODECHANGE */;
	adp->va_line_width = sc->stride;
	adp->va_buffer_size = sc->fb_size;

	if (vid_register(&sc->va) < 0) {
		printf("%s: failed to register va\n", __func__);
	}
	adp->va_flags |= V_ADP_REGISTERED;

	return (0);
}

static int
goldfish_fb_get_info(video_adapter_t *adp, int mode, video_info_t *info)
{

	bcopy(&adp->va_info, info, sizeof(*info));
	return (0);
}

static int
goldfish_fb_query_mode(video_adapter_t *adp, video_info_t *info)
{
	return (0);
}

static int
goldfish_fb_set_mode(video_adapter_t *adp, int mode)
{
	return (0);
}

static int
goldfish_fb_save_font(video_adapter_t *adp, int page, int size, int width,
    u_char *data, int c, int count)
{
	return (0);
}

static int
goldfish_fb_load_font(video_adapter_t *adp, int page, int size, int width,
    u_char *data, int c, int count)
{
	struct video_adapter_softc *sc;

	sc = (struct video_adapter_softc *)adp;
	sc->font = data;

	return (0);
}

static int
goldfish_fb_show_font(video_adapter_t *adp, int page)
{
	return (0);
}

static int
goldfish_fb_save_palette(video_adapter_t *adp, u_char *palette)
{
	return (0);
}

static int
goldfish_fb_load_palette(video_adapter_t *adp, u_char *palette)
{
	return (0);
}

static int
goldfish_fb_set_border(video_adapter_t *adp, int border)
{
	return (goldfish_fb_blank_display(adp, border));
}

static int
goldfish_fb_save_state(video_adapter_t *adp, void *p, size_t size)
{
	return (0);
}

static int
goldfish_fb_load_state(video_adapter_t *adp, void *p)
{
	return (0);
}

static int
goldfish_fb_set_win_org(video_adapter_t *adp, off_t offset)
{
	return (0);
}

static int
goldfish_fb_read_hw_cursor(video_adapter_t *adp, int *col, int *row)
{
	*col = *row = 0;
	return (0);
}

static int
goldfish_fb_set_hw_cursor(video_adapter_t *adp, int col, int row)
{
	return (0);
}

static int
goldfish_fb_set_hw_cursor_shape(video_adapter_t *adp, int base, int height,
    int celsize, int blink)
{
	return (0);
}

static int
goldfish_fb_blank_display(video_adapter_t *adp, int mode)
{
	return (0);
}

static int
goldfish_fb_mmap(video_adapter_t *adp, vm_ooffset_t offset, vm_paddr_t *paddr,
    int prot, vm_memattr_t *memattr)
{
	struct video_adapter_softc *sc;

	sc = (struct video_adapter_softc *)adp;

	/*
	 * This might be a legacy VGA mem request: if so, just point it at the
	 * framebuffer, since it shouldn't be touched
	 */
	if (offset < sc->stride * sc->height) {
		*paddr = sc->fb_paddr + offset;
		return (0);
	}

	return (EINVAL);
}

static int
goldfish_fb_ioctl(video_adapter_t *adp, u_long cmd, caddr_t data)
{
	struct video_adapter_softc *sc;
	struct fbtype *fb;

	sc = (struct video_adapter_softc *)adp;

	switch (cmd) {
	case FBIOGTYPE:
		fb = (struct fbtype *)data;
		fb->fb_type = FBTYPE_PCIMISC;
		fb->fb_height = sc->height;
		fb->fb_width = sc->width;
		fb->fb_depth = sc->depth;
		if (sc->depth <= 1 || sc->depth > 8)
			fb->fb_cmsize = 0;
		else
			fb->fb_cmsize = 1 << sc->depth;
		fb->fb_size = sc->fb_size;
		break;
	case FBIOSCURSOR:
		return (ENODEV);
	default:
		return (fb_commonioctl(adp, cmd, data));
	}

	return (0);
}

static int
goldfish_fb_clear(video_adapter_t *adp)
{
	return (goldfish_fb_blank_display(adp, 0));
}

static int
goldfish_fb_fill_rect(video_adapter_t *adp, int val, int x, int y, int cx, int cy)
{
	return (0);
}

static int
goldfish_fb_bitblt(video_adapter_t *adp, ...)
{
	return (0);
}

static int
goldfish_fb_diag(video_adapter_t *adp, int level)
{
	return (0);
}

static int
goldfish_fb_save_cursor_palette(video_adapter_t *adp, u_char *palette)
{
	return (0);
}

static int
goldfish_fb_load_cursor_palette(video_adapter_t *adp, u_char *palette)
{
	return (0);
}

static int
goldfish_fb_copy(video_adapter_t *adp, vm_offset_t src, vm_offset_t dst, int n)
{
	return (0);
}

static int
goldfish_fb_putp(video_adapter_t *adp, vm_offset_t off, uint32_t p, uint32_t a,
    int size, int bpp, int bit_ltor, int byte_ltor)
{
	return (0);
}

static int
goldfish_fb_putc(video_adapter_t *adp, vm_offset_t off, uint8_t c, uint8_t a)
{
	struct video_adapter_softc *sc;
	int col, row, bpp;
	int b, i, j, k;
	uint8_t *addr;
	u_char *p;
	uint32_t fg, bg, color;

	sc = (struct video_adapter_softc *)adp;
	bpp = sc->bpp;

	if (sc->fb_vaddr == 0)
		return (0);
	row = (off / adp->va_info.vi_width) * adp->va_info.vi_cheight;
	col = (off % adp->va_info.vi_width) * adp->va_info.vi_cwidth;
	p = sc->font + c * FONT_HEIGHT;
	addr = (uint8_t *)sc->fb_vaddr
	    + (row + sc->ymargin) * (sc->stride)
	    + bpp * (col + sc->xmargin);

	bg = colors[(a >> 4) & 0x0f];
	fg = colors[a & 0x0f];

	for (i = 0; i < FONT_HEIGHT; i++) {
		for (j = 0, k = 7; j < 8; j++, k--) {
			if ((p[i] & (1 << k)) == 0)
				color = bg;
			else
				color = fg;
			/* FIXME: BPP maybe different */
			for (b = 0; b < bpp; b ++)
				addr[bpp * j + b] =
				    (color >> (b << 3)) & 0xff;
		}

		addr += (sc->stride);
	}

	return (0);
}

static int
goldfish_fb_puts(video_adapter_t *adp, vm_offset_t off, u_int16_t *s, int len)
{
	int i;

	for (i = 0; i < len; i++) 
		goldfish_fb_putc(adp, off + i, s[i] & 0xff, (s[i] & 0xff00) >> 8);

	return (0);
}

static int
goldfish_fb_putm(video_adapter_t *adp, int x, int y, uint8_t *pixel_image,
    uint32_t pixel_mask, int size, int width)
{
	return (0);
}


/*
 * Define a stub keyboard driver in case one hasn't been
 * compiled into the kernel
 */
#include <sys/kbio.h>
#include <dev/kbd/kbdreg.h>

static int dummy_kbd_configure(int flags);

keyboard_switch_t goldfish_dummysw;

static int
dummy_kbd_configure(int flags)
{
	return (0);
}
KEYBOARD_DRIVER(goldfish_dummy, goldfish_dummysw, dummy_kbd_configure);

// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2019, Amarula Solutions.
 * Author: Jagan Teki <jagan@amarulasolutions.com>
 */

#include <drm/drm_mipi_dsi.h>
#include <drm/drm_modes.h>
#include <drm/drm_panel.h>
#include <drm/drm_print.h>

#include <linux/bitfield.h>
#include <linux/gpio/consumer.h>
#include <linux/delay.h>
#include <linux/media-bus-format.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/regulator/consumer.h>
#include <linux/spi/spi.h>

#include <video/mipi_display.h>

#include <linux/kernel.h>

#define SPI_DATA_FLAG			0x100

/* Command2 BKx selection command */
#define DSI_CMD2BKX_SEL			0xFF
#define DSI_CMD1			    0
#define DSI_CMD2			    BIT(4)
#define DSI_CMD2BK_MASK			GENMASK(3, 0)

/* Command2, BK0 commands */
#define DSI_CMD2_BK0_PVGAMCTRL		0xB0 /* Positive Voltage Gamma Control */
#define DSI_CMD2_BK0_NVGAMCTRL		0xB1 /* Negative Voltage Gamma Control */
#define DSI_CMD2_BK0_LNESET		    0xC0 /* Display Line setting */
#define DSI_CMD2_BK0_PORCTRL		0xC1 /* Porch control */
#define DSI_CMD2_BK0_INVSEL		    0xC2 /* Inversion selection, Frame Rate Control */

/* Command2, BK1 commands */
#define DSI_CMD2_BK1_VRHS		    0xB0 /* Vop amplitude setting */
#define DSI_CMD2_BK1_VCOM		    0xB1 /* VCOM amplitude setting */
#define DSI_CMD2_BK1_VGHSS		    0xB2 /* VGH Voltage setting */
#define DSI_CMD2_BK1_TESTCMD		0xB3 /* TEST Command Setting */
#define DSI_CMD2_BK1_VGLS		    0xB5 /* VGL Voltage setting */
#define DSI_CMD2_BK1_PWCTLR1		0xB7 /* Power Control 1 */
#define DSI_CMD2_BK1_PWCTLR2		0xB8 /* Power Control 2 */
#define DSI_CMD2_BK1_SPD1		    0xC1 /* Source pre_drive timing set1 */
#define DSI_CMD2_BK1_SPD2		    0xC2 /* Source EQ2 Setting */
#define DSI_CMD2_BK1_MIPISET1		0xD0 /* MIPI Setting 1 */

/*
 * Command2 with BK function selection.
 *
 * BIT[4].....CN2
 * BIT[1:0]...BKXSEL
 * 1:00 = CMD2BK0, Command2 BK0
 * 1:01 = CMD2BK1, Command2 BK1
 * 1:11 = CMD2BK3, Command2 BK3
 * 0:00 = Command2 disable
 */
#define DSI_CMD2BK0_SEL			0x10
#define DSI_CMD2BK1_SEL			0x11
#define DSI_CMD2BK3_SEL			0x13
#define DSI_CMD2BKX_SEL_NONE	0x00
#define SPI_CMD2BK3_SEL			(SPI_DATA_FLAG | DSI_CMD2BK3_SEL)
#define SPI_CMD2BK1_SEL			(SPI_DATA_FLAG | DSI_CMD2BK1_SEL)
#define SPI_CMD2BK0_SEL			(SPI_DATA_FLAG | DSI_CMD2BK0_SEL)
#define SPI_CMD2BKX_SEL_NONE	(SPI_DATA_FLAG | DSI_CMD2BKX_SEL_NONE)

/* Command2, BK0 bytes */
#define DSI_CMD2_BK0_GAMCTRL_AJ_MASK	GENMASK(7, 6)
#define DSI_CMD2_BK0_GAMCTRL_VC0_MASK	GENMASK(3, 0)
#define DSI_CMD2_BK0_GAMCTRL_VC4_MASK	GENMASK(5, 0)
#define DSI_CMD2_BK0_GAMCTRL_VC8_MASK	GENMASK(5, 0)
#define DSI_CMD2_BK0_GAMCTRL_VC16_MASK	GENMASK(4, 0)
#define DSI_CMD2_BK0_GAMCTRL_VC24_MASK	GENMASK(4, 0)
#define DSI_CMD2_BK0_GAMCTRL_VC52_MASK	GENMASK(3, 0)
#define DSI_CMD2_BK0_GAMCTRL_VC80_MASK	GENMASK(5, 0)
#define DSI_CMD2_BK0_GAMCTRL_VC108_MASK	GENMASK(3, 0)
#define DSI_CMD2_BK0_GAMCTRL_VC147_MASK	GENMASK(3, 0)
#define DSI_CMD2_BK0_GAMCTRL_VC175_MASK	GENMASK(5, 0)
#define DSI_CMD2_BK0_GAMCTRL_VC203_MASK	GENMASK(3, 0)
#define DSI_CMD2_BK0_GAMCTRL_VC231_MASK	GENMASK(4, 0)
#define DSI_CMD2_BK0_GAMCTRL_VC239_MASK	GENMASK(4, 0)
#define DSI_CMD2_BK0_GAMCTRL_VC247_MASK	GENMASK(5, 0)
#define DSI_CMD2_BK0_GAMCTRL_VC251_MASK	GENMASK(5, 0)
#define DSI_CMD2_BK0_GAMCTRL_VC255_MASK	GENMASK(4, 0)
#define DSI_CMD2_BK0_LNESET_LINE_MASK	GENMASK(6, 0)
#define DSI_CMD2_BK0_LNESET_LDE_EN	    BIT(7)
#define DSI_CMD2_BK0_LNESET_LINEDELTA	GENMASK(1, 0)
#define DSI_CMD2_BK0_PORCTRL_VBP_MASK	GENMASK(7, 0)
#define DSI_CMD2_BK0_PORCTRL_VFP_MASK	GENMASK(7, 0)
#define DSI_CMD2_BK0_INVSEL_ONES_MASK	GENMASK(5, 4)
#define DSI_CMD2_BK0_INVSEL_NLINV_MASK	GENMASK(2, 0)
#define DSI_CMD2_BK0_INVSEL_RTNI_MASK	GENMASK(4, 0)

/* Command2, BK1 bytes */
#define DSI_CMD2_BK1_VRHA_MASK		    GENMASK(7, 0)
#define DSI_CMD2_BK1_VCOM_MASK		    GENMASK(7, 0)
#define DSI_CMD2_BK1_VGHSS_MASK		    GENMASK(3, 0)
#define DSI_CMD2_BK1_TESTCMD_VAL	    BIT(7)
#define DSI_CMD2_BK1_VGLS_ONES		    BIT(6)
#define DSI_CMD2_BK1_VGLS_MASK		    GENMASK(3, 0)
#define DSI_CMD2_BK1_PWRCTRL1_AP_MASK	GENMASK(7, 6)
#define DSI_CMD2_BK1_PWRCTRL1_APIS_MASK	GENMASK(3, 2)
#define DSI_CMD2_BK1_PWRCTRL1_APOS_MASK	GENMASK(1, 0)
#define DSI_CMD2_BK1_PWRCTRL2_AVDD_MASK	GENMASK(5, 4)
#define DSI_CMD2_BK1_PWRCTRL2_AVCL_MASK	GENMASK(1, 0)
#define DSI_CMD2_BK1_SPD1_ONES_MASK	    GENMASK(6, 4)
#define DSI_CMD2_BK1_SPD1_T2D_MASK	    GENMASK(3, 0)
#define DSI_CMD2_BK1_SPD2_ONES_MASK	    GENMASK(6, 4)
#define DSI_CMD2_BK1_SPD2_T3D_MASK	    GENMASK(3, 0)
#define DSI_CMD2_BK1_MIPISET1_ONES	    BIT(7)
#define DSI_CMD2_BK1_MIPISET1_EOT_EN	BIT(3)

#define CFIELD_PREP(_mask, _val)					\
    (((typeof(_mask))(_val) << (__builtin_ffsll(_mask) - 1)) & (_mask))

enum op_bias {
    OP_BIAS_OFF = 0,
    OP_BIAS_MIN,
    OP_BIAS_MIDDLE,
    OP_BIAS_MAX
};

struct st7701;

struct st7701;

enum st7701_ctrl_if {
    ST7701_CTRL_DSI,
    ST7701_CTRL_SPI,
};

struct st7701_panel_desc {
    const struct drm_display_mode *mode;
    unsigned int lanes;
    enum mipi_dsi_pixel_format format;
    u32 mediabus_format;
    unsigned int panel_sleep_delay;
    void (*init_sequence)(struct st7701 *st7701);
    unsigned int conn_type;
    enum st7701_ctrl_if interface;
    u32 bus_flags;

    /* TFT matrix driver configuration, panel specific. */
    const u8	pv_gamma[16];	/* Positive voltage gamma control */
    const u8	nv_gamma[16];	/* Negative voltage gamma control */
    const u8	nlinv;		/* Inversion selection */
    const u32	vop_uv;		/* Vop in uV */
    const u32	vcom_uv;	/* Vcom in uV */
    const u16	vgh_mv;		/* Vgh in mV */
    const s16	vgl_mv;		/* Vgl in mV */
    const u16	avdd_mv;	/* Avdd in mV */
    const s16	avcl_mv;	/* Avcl in mV */
    const enum op_bias	gamma_op_bias;
    const enum op_bias	input_op_bias;
    const enum op_bias	output_op_bias;
    const u16	t2d_ns;		/* T2D in ns */
    const u16	t3d_ns;		/* T3D in ns */
    const bool	eot_en;

    /* GIP sequence, fully custom and undocumented. */
    void		(*gip_sequence)(struct st7701 *st7701);
};

struct st7701 {
    struct drm_panel panel;
    struct mipi_dsi_device *dsi;
    struct spi_device *spi;
    const struct device *dev;

    const struct st7701_panel_desc *desc;

    struct regulator_bulk_data supplies[2];
    struct gpio_desc *reset;
    unsigned int sleep_delay;
    enum drm_panel_orientation orientation;
};

static inline struct st7701 *panel_to_st7701(struct drm_panel *panel)
{
    return container_of(panel, struct st7701, panel);
}

static inline int st7701_dsi_write(struct st7701 *st7701, const void *seq,
                   size_t len)
{
    return mipi_dsi_dcs_write_buffer(st7701->dsi, seq, len);
}

#define ST7701_DSI(st7701, seq...)				\
    {							\
        const u8 d[] = { seq };				\
        st7701_dsi_write(st7701, d, ARRAY_SIZE(d));	\
    }

/*static u8 st7701_vgls_map(struct st7701 *st7701)
{
    const struct st7701_panel_desc *desc = st7701->desc;
    struct {
        s32	vgl;
        u8	val;
    } map[16] = {
        { -7060, 0x0 }, { -7470, 0x1 },
        { -7910, 0x2 }, { -8140, 0x3 },
        { -8650, 0x4 }, { -8920, 0x5 },
        { -9210, 0x6 }, { -9510, 0x7 },
        { -9830, 0x8 }, { -10170, 0x9 },
        { -10530, 0xa }, { -10910, 0xb },
        { -11310, 0xc }, { -11730, 0xd },
        { -12200, 0xe }, { -12690, 0xf }
    };
    int i;

    for (i = 0; i < ARRAY_SIZE(map); i++)
        if (desc->vgl_mv == map[i].vgl)
            return map[i].val;

    return 0;
}*/

static void st7701_switch_cmd_bkx(struct st7701 *st7701, bool cmd2, u8 bkx)
{
    u8 val;

    if (cmd2)
        val = DSI_CMD2 | FIELD_PREP(DSI_CMD2BK_MASK, bkx);
    else
        val = DSI_CMD1;

    ST7701_DSI(st7701, DSI_CMD2BKX_SEL, 0x77, 0x01, 0x00, 0x00, val);
}

#define ST7701_SPI(st7701, seq...)				\
    {							\
        const u16 d[] = { seq };			\
        struct spi_transfer xfer = { };			\
        struct spi_message spi;				\
                                \
        spi_message_init(&spi);				\
                                \
        xfer.tx_buf = d;				\
        xfer.bits_per_word = 9;				\
        xfer.len = sizeof(u16) * ARRAY_SIZE(d);		\
                                \
        spi_message_add_tail(&xfer, &spi);		\
        spi_sync((st7701)->spi, &spi);			\
    }

static void kd029qhfid001_init_sequence(struct st7701 *st7701)
{
    const struct st7701_panel_desc *desc = st7701->desc;

    st7701_switch_cmd_bkx(st7701, true, 3);
    ST7701_DSI(st7701, 0xEF, 0x08);

    st7701_switch_cmd_bkx(st7701, true, 0);
    ST7701_DSI(st7701, DSI_CMD2_BK0_LNESET,  0x77, 0x00);
    ST7701_DSI(st7701, DSI_CMD2_BK0_PORCTRL, 0x0C, 0x0C);
    ST7701_DSI(st7701, DSI_CMD2_BK0_INVSEL,  0x27, 0x0A);
    ST7701_DSI(st7701, 0xCC, 0x10);
    mipi_dsi_dcs_write(st7701->dsi, DSI_CMD2_BK0_PVGAMCTRL, desc->pv_gamma, ARRAY_SIZE(desc->pv_gamma));
    mipi_dsi_dcs_write(st7701->dsi, DSI_CMD2_BK0_NVGAMCTRL, desc->nv_gamma, ARRAY_SIZE(desc->nv_gamma));

    st7701_switch_cmd_bkx(st7701, true, 1);
    ST7701_DSI(st7701, DSI_CMD2_BK1_VRHS,     0x25);
    ST7701_DSI(st7701, DSI_CMD2_BK1_VCOM,     0x76);
    ST7701_DSI(st7701, DSI_CMD2_BK1_VGHSS,    0x81);
    ST7701_DSI(st7701, DSI_CMD2_BK1_TESTCMD,  0x80);
    ST7701_DSI(st7701, DSI_CMD2_BK1_VGLS,     0x49);
    ST7701_DSI(st7701, DSI_CMD2_BK1_PWCTLR1,  0x85);
    ST7701_DSI(st7701, DSI_CMD2_BK1_PWCTLR2,  0x20);
    ST7701_DSI(st7701, DSI_CMD2_BK1_SPD1,     0x78);
    ST7701_DSI(st7701, DSI_CMD2_BK1_SPD2,     0x78);
    ST7701_DSI(st7701, DSI_CMD2_BK1_MIPISET1, 0x88);
}

static void kd029qhfid001_gip_sequence(struct st7701 *st7701)
{
    ST7701_DSI(st7701, 0xE0, 0x00, 0x00, 0x02);
    ST7701_DSI(st7701, 0xE1, 0x02, 0x8C, 0x04, 0x8C, 0x01, 0x8C, 0x03, 0x8C, 0x00, 0x44, 0x44);
    ST7701_DSI(st7701, 0xE2, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00);
    ST7701_DSI(st7701, 0xE3, 0x00, 0x00, 0x33, 0x33);
    ST7701_DSI(st7701, 0xE4, 0x44, 0x44);
    ST7701_DSI(st7701, 0xE5, 0x09, 0xD2, 0x35, 0x8C, 0x0B, 0xD4, 0x35, 0x8C, 0x05, 0xCE, 0x35, 0x8C, 0x07, 0xD0, 0x35, 0x8C);
    ST7701_DSI(st7701, 0xE6, 0x00, 0x00, 0x33, 0x33);
    ST7701_DSI(st7701, 0xE7, 0x44, 0x44);
    ST7701_DSI(st7701, 0xE8, 0x08, 0xD1, 0x35, 0x8C, 0x0A, 0xD3, 0x35, 0x8C, 0x04, 0xCD, 0x35, 0x8C, 0x06, 0xCF, 0x35, 0x8C);
    ST7701_DSI(st7701, 0xEB, 0x00, 0x01, 0xE4, 0xE4, 0x44, 0x00);
    ST7701_DSI(st7701, 0xED, 0x77, 0x66, 0x55, 0x44, 0xCA, 0xF1, 0x03, 0xBF, 0xFB, 0x30, 0x1F, 0xAC, 0x44, 0x55, 0x66, 0x77);
    ST7701_DSI(st7701, 0xEF, 0x10, 0x0D, 0x04, 0x08, 0x3F, 0x1F);

    st7701_switch_cmd_bkx(st7701, false, 0);
    ST7701_DSI(st7701, 0x3A, 0x70);
    ST7701_DSI(st7701, 0x36, 0x00);
    ST7701_DSI(st7701, 0x35, 0x00);
}

static int st7701_prepare(struct drm_panel *panel)
{
    struct st7701 *st7701 = panel_to_st7701(panel);
    int ret;

    gpiod_set_value(st7701->reset, 0);

    ret = regulator_bulk_enable(ARRAY_SIZE(st7701->supplies),
                    st7701->supplies);
    if (ret < 0)
        return ret;
    msleep(20);

    gpiod_set_value(st7701->reset, 1);
    msleep(150);
    printk(KERN_INFO "st7701_prepare -> reset -> done\n");

    st7701->desc->init_sequence(st7701);

    if (st7701->desc->gip_sequence)
        st7701->desc->gip_sequence(st7701);

    ST7701_DSI(st7701, MIPI_DCS_EXIT_SLEEP_MODE);
    msleep(150);
    printk(KERN_INFO "st7701_prepare -> done\n");

    return 0;
}

static int st7701_enable(struct drm_panel *panel)
{
    struct st7701 *st7701 = panel_to_st7701(panel);
    ST7701_DSI(st7701, MIPI_DCS_SET_DISPLAY_ON);
    printk(KERN_INFO "st7701_enable -> done\n");
    return 0;
}

static int st7701_disable(struct drm_panel *panel)
{
    struct st7701 *st7701 = panel_to_st7701(panel);
    ST7701_DSI(st7701, MIPI_DCS_SET_DISPLAY_OFF);
    msleep(20);
    printk(KERN_INFO "st7701_disable -> done\n");
    return 0;
}

static int st7701_unprepare(struct drm_panel *panel)
{
    struct st7701 *st7701 = panel_to_st7701(panel);

    ST7701_DSI(st7701, MIPI_DCS_ENTER_SLEEP_MODE);

    msleep(st7701->sleep_delay);

    gpiod_set_value(st7701->reset, 0);

    /**
     * During the Resetting period, the display will be blanked
     * (The display is entering blanking sequence, which maximum
     * time is 120 ms, when Reset Starts in Sleep Out –mode. The
     * display remains the blank state in Sleep In –mode.) and
     * then return to Default condition for Hardware Reset.
     *
     * So we need wait sleep_delay time to make sure reset completed.
     */
    msleep(st7701->sleep_delay);

    regulator_bulk_disable(ARRAY_SIZE(st7701->supplies), st7701->supplies);

    return 0;
}

static int st7701_get_modes(struct drm_panel *panel,
                struct drm_connector *connector)
{
    struct st7701 *st7701 = panel_to_st7701(panel);
    const struct drm_display_mode *desc_mode = st7701->desc->mode;
    struct drm_display_mode *mode;

    mode = drm_mode_duplicate(connector->dev, desc_mode);
    if (!mode) {
        dev_err(st7701->dev, "failed to add mode %ux%u@%u\n",
            desc_mode->hdisplay, desc_mode->vdisplay,
            drm_mode_vrefresh(desc_mode));
        return -ENOMEM;
    }

    drm_mode_set_name(mode);
    drm_mode_probed_add(connector, mode);

    if (st7701->desc->mediabus_format)
        drm_display_info_set_bus_formats(&connector->display_info,
                         &st7701->desc->mediabus_format,
                         1);
    connector->display_info.bus_flags = 0;

    connector->display_info.width_mm = desc_mode->width_mm;
    connector->display_info.height_mm = desc_mode->height_mm;

    /*
     * TODO: Remove once all drm drivers call
     * drm_connector_set_orientation_from_panel()
     */
    drm_connector_set_panel_orientation(connector, st7701->orientation);

    if (st7701->desc->bus_flags)
        connector->display_info.bus_flags = st7701->desc->bus_flags;

    return 1;
}

static enum drm_panel_orientation st7701_get_orientation(struct drm_panel *panel)
{
    struct st7701 *st7701 = panel_to_st7701(panel);

    return st7701->orientation;
}

static const struct drm_panel_funcs st7701_funcs = {
    .disable	= st7701_disable,
    .unprepare	= st7701_unprepare,
    .prepare	= st7701_prepare,
    .enable		= st7701_enable,
    .get_modes	= st7701_get_modes,
    .get_orientation = st7701_get_orientation,
};

static const struct drm_display_mode kd029qhfid001_mode = {
    .clock       = 30000,

    .hdisplay	 = 376,
    .hsync_start = 376 + 56,
    .hsync_end   = 376 + 56 + 10,
    .htotal      = 376 + 56 + 10 + 30,

    .vdisplay    = 960,
    .vsync_start = 960 + 60,
    .vsync_end   = 960 + 60 + 10,
    .vtotal      = 960 + 60 + 10 + 30,

    .width_mm    = 27,
    .height_mm   = 68,

    .type = DRM_MODE_TYPE_DRIVER | DRM_MODE_TYPE_PREFERRED,
};

static const struct st7701_panel_desc kd029qhfid001_desc = {
    .mode = &kd029qhfid001_mode,
    .lanes = 2,
    .format = MIPI_DSI_FMT_RGB888,
    .panel_sleep_delay = 80,
    .init_sequence = kd029qhfid001_init_sequence,
    .conn_type = DRM_MODE_CONNECTOR_DSI,
    .interface = ST7701_CTRL_DSI,
    .pv_gamma = {
        0x00, 0x0C, 0x19, 0x0B,
        0x0F, 0x06, 0x05, 0x08,
        0x08, 0x1F, 0x04, 0x11,
        0x0F, 0x26, 0x2F, 0x1D
    },
    .nv_gamma = {
        0x00, 0x17, 0x19, 0x0F,
        0x12, 0x05, 0x05, 0x08,
        0x07, 0x1F, 0x03, 0x10,
        0x10, 0x27, 0x2F, 0x1D
    },
    .gip_sequence = kd029qhfid001_gip_sequence,
};

static int st7701_probe(struct device *dev, struct st7701 **ret_st7701)
{
    const struct st7701_panel_desc *desc;
    struct st7701 *st7701;
    int ret;

    st7701 = devm_kzalloc(dev, sizeof(*st7701), GFP_KERNEL);
    if (!st7701)
        return -ENOMEM;

    desc = of_device_get_match_data(dev);
    if (!desc)
        return -EINVAL;

    st7701->supplies[0].supply = "VCC";
    st7701->supplies[1].supply = "IOVCC";

    ret = devm_regulator_bulk_get(dev, ARRAY_SIZE(st7701->supplies),
                      st7701->supplies);

    if (ret < 0)
        return ret;

    st7701->reset = devm_gpiod_get_optional(dev, "reset", GPIOD_OUT_LOW);
    if (IS_ERR(st7701->reset)) {
        dev_err(dev, "Couldn't get our reset GPIO\n");
        return PTR_ERR(st7701->reset);
    }

    drm_panel_init(&st7701->panel, dev, &st7701_funcs,
               desc->conn_type);
    st7701->panel.prepare_prev_first = true;

    /**
     * Once sleep out has been issued, ST7701 IC required to wait 120ms
     * before initiating new commands.
     *
     * On top of that some panels might need an extra delay to wait, so
     * add panel specific delay for those cases. As now this panel specific
     * delay information is referenced from those panel BSP driver, example
     * ts8550b and there is no valid documentation for that.
     */
    st7701->sleep_delay = 120 + desc->panel_sleep_delay;

    ret = drm_panel_of_backlight(&st7701->panel);
    if (ret)
        return ret;

    drm_panel_add(&st7701->panel);

    st7701->desc = desc;
    st7701->dev = dev;

    *ret_st7701 = st7701;

    return 0;
}

static int st7701_dsi_probe(struct mipi_dsi_device *dsi)
{
    struct st7701 *st7701;
    int ret;

    ret = st7701_probe(&dsi->dev, &st7701);

    if (ret)
        return ret;

    dsi->mode_flags = MIPI_DSI_MODE_VIDEO | MIPI_DSI_MODE_LPM;
    dsi->format = st7701->desc->format;
    dsi->lanes = st7701->desc->lanes;

    mipi_dsi_set_drvdata(dsi, st7701);
    st7701->dsi = dsi;

    ret = mipi_dsi_attach(dsi);
    if (ret)
        goto err_attach;

    ret = of_drm_get_panel_orientation(dsi->dev.of_node, &st7701->orientation);
    if (ret < 0)
        return dev_err_probe(&dsi->dev, ret, "Failed to get orientation\n");

    return 0;

err_attach:
    drm_panel_remove(&st7701->panel);
    return ret;
}

static void st7701_dsi_remove(struct mipi_dsi_device *dsi)
{
    struct st7701 *st7701 = mipi_dsi_get_drvdata(dsi);

    mipi_dsi_detach(dsi);
    drm_panel_remove(&st7701->panel);
}

static const struct of_device_id st7701_dsi_of_match[] = {
    { .compatible = "dmb,kd029qhfid001", .data = &kd029qhfid001_desc },
    { }
};
MODULE_DEVICE_TABLE(of, st7701_dsi_of_match);

static struct mipi_dsi_driver st7701_dsi_driver = {
    .probe		= st7701_dsi_probe,
    .remove		= st7701_dsi_remove,
    .driver = {
        .name	= "panel-dmb-kd029qhfid001",
        .of_match_table	= st7701_dsi_of_match,
    },
};

static int __init panel_st7701_init(void)
{
    if (IS_ENABLED(CONFIG_DRM_MIPI_DSI)) {
        return mipi_dsi_driver_register(&st7701_dsi_driver);
    }

    return -1;
}
module_init(panel_st7701_init);

static void __exit panel_st7701_exit(void)
{
    if (IS_ENABLED(CONFIG_DRM_MIPI_DSI))
        mipi_dsi_driver_unregister(&st7701_dsi_driver);
}
module_exit(panel_st7701_exit);

MODULE_AUTHOR("long568 <zandzx02cn@gmail.com>");
MODULE_DESCRIPTION("DMB KD029QHFID001 panel based on Sitronix ST7701");
MODULE_LICENSE("GPL");

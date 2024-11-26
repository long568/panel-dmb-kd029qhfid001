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

// Added by long 20241124
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

static u8 st7701_vgls_map(struct st7701 *st7701)
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
}

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

static void ts8550b_init_sequence(struct st7701 *st7701)
{
    const struct st7701_panel_desc *desc = st7701->desc;
    const struct drm_display_mode *mode = desc->mode;
    const u8 linecount8 = mode->vdisplay / 8;
    const u8 linecountrem2 = (mode->vdisplay % 8) / 2;

    ST7701_DSI(st7701, MIPI_DCS_SOFT_RESET, 0x00);

    /* We need to wait 5ms before sending new commands */
    msleep(5);

    ST7701_DSI(st7701, MIPI_DCS_EXIT_SLEEP_MODE, 0x00);

    msleep(st7701->sleep_delay);

    /* Command2, BK0 */
    st7701_switch_cmd_bkx(st7701, true, 0);

    mipi_dsi_dcs_write(st7701->dsi, DSI_CMD2_BK0_PVGAMCTRL,
               desc->pv_gamma, ARRAY_SIZE(desc->pv_gamma));
    mipi_dsi_dcs_write(st7701->dsi, DSI_CMD2_BK0_NVGAMCTRL,
               desc->nv_gamma, ARRAY_SIZE(desc->nv_gamma));
    /*
     * Vertical line count configuration:
     * Line[6:0]: select number of vertical lines of the TFT matrix in
     *            multiples of 8 lines
     * LDE_EN: enable sub-8-line granularity line count
     * Line_delta[1:0]: add 0/2/4/6 extra lines to line count selected
     *                  using Line[6:0]
     *
     * Total number of vertical lines:
     * LN = ((Line[6:0] + 1) * 8) + (LDE_EN ? Line_delta[1:0] * 2 : 0)
     */
    ST7701_DSI(st7701, DSI_CMD2_BK0_LNESET,
           FIELD_PREP(DSI_CMD2_BK0_LNESET_LINE_MASK, linecount8 - 1) |
           (linecountrem2 ? DSI_CMD2_BK0_LNESET_LDE_EN : 0),
           FIELD_PREP(DSI_CMD2_BK0_LNESET_LINEDELTA, linecountrem2));
    ST7701_DSI(st7701, DSI_CMD2_BK0_PORCTRL,
           FIELD_PREP(DSI_CMD2_BK0_PORCTRL_VBP_MASK,
                  mode->vtotal - mode->vsync_end),
           FIELD_PREP(DSI_CMD2_BK0_PORCTRL_VFP_MASK,
                  mode->vsync_start - mode->vdisplay));
    /*
     * Horizontal pixel count configuration:
     * PCLK = 512 + (RTNI[4:0] * 16)
     * The PCLK is number of pixel clock per line, which matches
     * mode htotal. The minimum is 512 PCLK.
     */
    ST7701_DSI(st7701, DSI_CMD2_BK0_INVSEL,
           DSI_CMD2_BK0_INVSEL_ONES_MASK |
           FIELD_PREP(DSI_CMD2_BK0_INVSEL_NLINV_MASK, desc->nlinv),
           FIELD_PREP(DSI_CMD2_BK0_INVSEL_RTNI_MASK,
                  (clamp((u32)mode->htotal, 512U, 1008U) - 512) / 16));

    /* Command2, BK1 */
    st7701_switch_cmd_bkx(st7701, true, 1);

    /* Vop = 3.5375V + (VRHA[7:0] * 0.0125V) */
    ST7701_DSI(st7701, DSI_CMD2_BK1_VRHS,
           FIELD_PREP(DSI_CMD2_BK1_VRHA_MASK,
                  DIV_ROUND_CLOSEST(desc->vop_uv - 3537500, 12500)));

    /* Vcom = 0.1V + (VCOM[7:0] * 0.0125V) */
    ST7701_DSI(st7701, DSI_CMD2_BK1_VCOM,
           FIELD_PREP(DSI_CMD2_BK1_VCOM_MASK,
                  DIV_ROUND_CLOSEST(desc->vcom_uv - 100000, 12500)));

    /* Vgh = 11.5V + (VGHSS[7:0] * 0.5V) */
    ST7701_DSI(st7701, DSI_CMD2_BK1_VGHSS,
           FIELD_PREP(DSI_CMD2_BK1_VGHSS_MASK,
                  DIV_ROUND_CLOSEST(clamp(desc->vgh_mv,
                              (u16)11500,
                              (u16)17000) - 11500,
                        500)));

    ST7701_DSI(st7701, DSI_CMD2_BK1_TESTCMD, DSI_CMD2_BK1_TESTCMD_VAL);

    /* Vgl is non-linear */
    ST7701_DSI(st7701, DSI_CMD2_BK1_VGLS,
           DSI_CMD2_BK1_VGLS_ONES |
           FIELD_PREP(DSI_CMD2_BK1_VGLS_MASK, st7701_vgls_map(st7701)));

    ST7701_DSI(st7701, DSI_CMD2_BK1_PWCTLR1,
           FIELD_PREP(DSI_CMD2_BK1_PWRCTRL1_AP_MASK,
                  desc->gamma_op_bias) |
           FIELD_PREP(DSI_CMD2_BK1_PWRCTRL1_APIS_MASK,
                  desc->input_op_bias) |
           FIELD_PREP(DSI_CMD2_BK1_PWRCTRL1_APOS_MASK,
                  desc->output_op_bias));

    /* Avdd = 6.2V + (AVDD[1:0] * 0.2V) , Avcl = -4.4V - (AVCL[1:0] * 0.2V) */
    ST7701_DSI(st7701, DSI_CMD2_BK1_PWCTLR2,
           FIELD_PREP(DSI_CMD2_BK1_PWRCTRL2_AVDD_MASK,
                  DIV_ROUND_CLOSEST(desc->avdd_mv - 6200, 200)) |
           FIELD_PREP(DSI_CMD2_BK1_PWRCTRL2_AVCL_MASK,
                  DIV_ROUND_CLOSEST(-4400 - desc->avcl_mv, 200)));

    /* T2D = 0.2us * T2D[3:0] */
    ST7701_DSI(st7701, DSI_CMD2_BK1_SPD1,
           DSI_CMD2_BK1_SPD1_ONES_MASK |
           FIELD_PREP(DSI_CMD2_BK1_SPD1_T2D_MASK,
                  DIV_ROUND_CLOSEST(desc->t2d_ns, 200)));

    /* T3D = 4us + (0.8us * T3D[3:0]) */
    ST7701_DSI(st7701, DSI_CMD2_BK1_SPD2,
           DSI_CMD2_BK1_SPD2_ONES_MASK |
           FIELD_PREP(DSI_CMD2_BK1_SPD2_T3D_MASK,
                  DIV_ROUND_CLOSEST(desc->t3d_ns - 4000, 800)));

    ST7701_DSI(st7701, DSI_CMD2_BK1_MIPISET1,
           DSI_CMD2_BK1_MIPISET1_ONES |
           (desc->eot_en ? DSI_CMD2_BK1_MIPISET1_EOT_EN : 0));
}

static void ts8550b_gip_sequence(struct st7701 *st7701)
{
    /**
     * ST7701_SPEC_V1.2 is unable to provide enough information above this
     * specific command sequence, so grab the same from vendor BSP driver.
     */
    ST7701_DSI(st7701, 0xE0, 0x00, 0x00, 0x02);
    ST7701_DSI(st7701, 0xE1, 0x0B, 0x00, 0x0D, 0x00, 0x0C, 0x00, 0x0E,
           0x00, 0x00, 0x44, 0x44);
    ST7701_DSI(st7701, 0xE2, 0x33, 0x33, 0x44, 0x44, 0x64, 0x00, 0x66,
           0x00, 0x65, 0x00, 0x67, 0x00, 0x00);
    ST7701_DSI(st7701, 0xE3, 0x00, 0x00, 0x33, 0x33);
    ST7701_DSI(st7701, 0xE4, 0x44, 0x44);
    ST7701_DSI(st7701, 0xE5, 0x0C, 0x78, 0x3C, 0xA0, 0x0E, 0x78, 0x3C,
           0xA0, 0x10, 0x78, 0x3C, 0xA0, 0x12, 0x78, 0x3C, 0xA0);
    ST7701_DSI(st7701, 0xE6, 0x00, 0x00, 0x33, 0x33);
    ST7701_DSI(st7701, 0xE7, 0x44, 0x44);
    ST7701_DSI(st7701, 0xE8, 0x0D, 0x78, 0x3C, 0xA0, 0x0F, 0x78, 0x3C,
           0xA0, 0x11, 0x78, 0x3C, 0xA0, 0x13, 0x78, 0x3C, 0xA0);
    ST7701_DSI(st7701, 0xEB, 0x02, 0x02, 0x39, 0x39, 0xEE, 0x44, 0x00);
    ST7701_DSI(st7701, 0xEC, 0x00, 0x00);
    ST7701_DSI(st7701, 0xED, 0xFF, 0xF1, 0x04, 0x56, 0x72, 0x3F, 0xFF,
           0xFF, 0xFF, 0xFF, 0xF3, 0x27, 0x65, 0x40, 0x1F, 0xFF);
}

static void kd029qhfid001_init_sequence(struct st7701 *st7701)
{
    // printk(KERN_INFO "kd029qhfid001_init_sequence -> mipi_dsi_dcs_exit_sleep_mode(st7701->dsi)\n");
    // mipi_dsi_dcs_exit_sleep_mode(st7701->dsi);

    // printk(KERN_INFO "kd029qhfid001_init_sequence -> ST7701_DSI(st7701, MIPI_DCS_SOFT_RESET, 0x00)\n");
    // ST7701_DSI(st7701, MIPI_DCS_SOFT_RESET, 0x00);
	// msleep(5);

    // printk(KERN_INFO "kd029qhfid001_init_sequence -> ST7701_DSI(st7701, MIPI_DCS_EXIT_SLEEP_MODE, 0x00)\n");
	// ST7701_DSI(st7701, MIPI_DCS_EXIT_SLEEP_MODE, 0x00);
	// msleep(st7701->sleep_delay);

    printk(KERN_INFO "kd029qhfid001_init_sequence -> st7701_switch_cmd_bkx(st7701, true, 3)\n");
    st7701_switch_cmd_bkx(st7701, true, 3);

    printk(KERN_INFO "kd029qhfid001_init_sequence -> ST7701_DSI(st7701, 0xEF, 0x08)\n");
    ST7701_DSI(st7701, 0xEF, 0x08);
}

static void kd029qhfid001_gip_sequence(struct st7701 *st7701)
{
}

static int st7701_prepare(struct drm_panel *panel)
{
    struct st7701 *st7701 = panel_to_st7701(panel);
    int ret;

    gpiod_set_value(st7701->reset, 1);
    msleep(10);

    gpiod_set_value(st7701->reset, 0);

    ret = regulator_bulk_enable(ARRAY_SIZE(st7701->supplies),
                    st7701->supplies);
    if (ret < 0)
        return ret;
    msleep(20);

    gpiod_set_value(st7701->reset, 1);
    msleep(150);
    // printk(KERN_INFO "st7701->reset, done\n");

    st7701->desc->init_sequence(st7701);

    // if (st7701->desc->gip_sequence)
    //     st7701->desc->gip_sequence(st7701);

    // /* Disable Command2 */
    // switch (st7701->desc->interface) {
    // case ST7701_CTRL_DSI:
    //     st7701_switch_cmd_bkx(st7701, false, 0);
    //     break;
    // case ST7701_CTRL_SPI:
    //     ST7701_SPI(st7701, DSI_CMD2BKX_SEL,
    //            0x177, 0x101, 0x100, 0x100, SPI_CMD2BKX_SEL_NONE);
    //     break;
    // }

    return 0;
}

static int st7701_enable(struct drm_panel *panel)
{
    struct st7701 *st7701 = panel_to_st7701(panel);

    printk(KERN_INFO "st7701_enable...0\n");

    // st7701->desc->init_sequence(st7701);

    // switch (st7701->desc->interface) {
    // case ST7701_CTRL_DSI:
    //     ST7701_DSI(st7701, MIPI_DCS_SET_DISPLAY_ON, 0x00);
    //     break;
    // case ST7701_CTRL_SPI:
    //     ST7701_SPI(st7701, MIPI_DCS_SET_DISPLAY_ON);
    //     msleep(30);
    //     break;
    // }

    printk(KERN_INFO "st7701_enable...1\n");

    return 0;
}

static int st7701_disable(struct drm_panel *panel)
{
    struct st7701 *st7701 = panel_to_st7701(panel);

    switch (st7701->desc->interface) {
    case ST7701_CTRL_DSI:
        ST7701_DSI(st7701, MIPI_DCS_SET_DISPLAY_OFF, 0x00);
        break;
    case ST7701_CTRL_SPI:
        ST7701_SPI(st7701, MIPI_DCS_SET_DISPLAY_OFF);
        break;
    }

    return 0;
}

static int st7701_unprepare(struct drm_panel *panel)
{
    struct st7701 *st7701 = panel_to_st7701(panel);

    switch (st7701->desc->interface) {
    case ST7701_CTRL_DSI:
        ST7701_DSI(st7701, MIPI_DCS_ENTER_SLEEP_MODE, 0x00);
        break;
    case ST7701_CTRL_SPI:
        ST7701_SPI(st7701, MIPI_DCS_ENTER_SLEEP_MODE);
        break;
    }

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

static const struct drm_display_mode ts8550b_mode = {
    .clock		= 27500,

    .hdisplay	= 480,
    .hsync_start	= 480 + 38,
    .hsync_end	= 480 + 38 + 12,
    .htotal		= 480 + 38 + 12 + 12,

    .vdisplay	= 854,
    .vsync_start	= 854 + 18,
    .vsync_end	= 854 + 18 + 8,
    .vtotal		= 854 + 18 + 8 + 4,

    .width_mm	= 69,
    .height_mm	= 139,

    .type = DRM_MODE_TYPE_DRIVER | DRM_MODE_TYPE_PREFERRED,
};

static const struct st7701_panel_desc ts8550b_desc = {
    .mode = &ts8550b_mode,
    .lanes = 2,
    .format = MIPI_DSI_FMT_RGB888,
    .panel_sleep_delay = 80, /* panel need extra 80ms for sleep out cmd */
    .init_sequence = ts8550b_init_sequence,
    .conn_type = DRM_MODE_CONNECTOR_DSI,
    .interface = ST7701_CTRL_DSI,

    .pv_gamma = {
        CFIELD_PREP(DSI_CMD2_BK0_GAMCTRL_AJ_MASK, 0) |
        CFIELD_PREP(DSI_CMD2_BK0_GAMCTRL_VC0_MASK, 0),
        CFIELD_PREP(DSI_CMD2_BK0_GAMCTRL_AJ_MASK, 0) |
        CFIELD_PREP(DSI_CMD2_BK0_GAMCTRL_VC4_MASK, 0xe),
        CFIELD_PREP(DSI_CMD2_BK0_GAMCTRL_AJ_MASK, 0) |
        CFIELD_PREP(DSI_CMD2_BK0_GAMCTRL_VC8_MASK, 0x15),
        CFIELD_PREP(DSI_CMD2_BK0_GAMCTRL_VC16_MASK, 0xf),

        CFIELD_PREP(DSI_CMD2_BK0_GAMCTRL_AJ_MASK, 0) |
        CFIELD_PREP(DSI_CMD2_BK0_GAMCTRL_VC24_MASK, 0x11),
        CFIELD_PREP(DSI_CMD2_BK0_GAMCTRL_VC52_MASK, 0x8),
        CFIELD_PREP(DSI_CMD2_BK0_GAMCTRL_VC80_MASK, 0x8),
        CFIELD_PREP(DSI_CMD2_BK0_GAMCTRL_VC108_MASK, 0x8),

        CFIELD_PREP(DSI_CMD2_BK0_GAMCTRL_VC147_MASK, 0x8),
        CFIELD_PREP(DSI_CMD2_BK0_GAMCTRL_VC175_MASK, 0x23),
        CFIELD_PREP(DSI_CMD2_BK0_GAMCTRL_VC203_MASK, 0x4),
        CFIELD_PREP(DSI_CMD2_BK0_GAMCTRL_AJ_MASK, 0) |
        CFIELD_PREP(DSI_CMD2_BK0_GAMCTRL_VC231_MASK, 0x13),

        CFIELD_PREP(DSI_CMD2_BK0_GAMCTRL_VC239_MASK, 0x12),
        CFIELD_PREP(DSI_CMD2_BK0_GAMCTRL_AJ_MASK, 0) |
        CFIELD_PREP(DSI_CMD2_BK0_GAMCTRL_VC247_MASK, 0x2b),
        CFIELD_PREP(DSI_CMD2_BK0_GAMCTRL_AJ_MASK, 0) |
        CFIELD_PREP(DSI_CMD2_BK0_GAMCTRL_VC251_MASK, 0x34),
        CFIELD_PREP(DSI_CMD2_BK0_GAMCTRL_AJ_MASK, 0) |
        CFIELD_PREP(DSI_CMD2_BK0_GAMCTRL_VC255_MASK, 0x1f)
    },
    .nv_gamma = {
        CFIELD_PREP(DSI_CMD2_BK0_GAMCTRL_AJ_MASK, 0) |
        CFIELD_PREP(DSI_CMD2_BK0_GAMCTRL_VC0_MASK, 0),
        CFIELD_PREP(DSI_CMD2_BK0_GAMCTRL_AJ_MASK, 0) |
        CFIELD_PREP(DSI_CMD2_BK0_GAMCTRL_VC4_MASK, 0xe),
        CFIELD_PREP(DSI_CMD2_BK0_GAMCTRL_AJ_MASK, 0x2) |
        CFIELD_PREP(DSI_CMD2_BK0_GAMCTRL_VC8_MASK, 0x15),
        CFIELD_PREP(DSI_CMD2_BK0_GAMCTRL_VC16_MASK, 0xf),

        CFIELD_PREP(DSI_CMD2_BK0_GAMCTRL_AJ_MASK, 0) |
        CFIELD_PREP(DSI_CMD2_BK0_GAMCTRL_VC24_MASK, 0x13),
        CFIELD_PREP(DSI_CMD2_BK0_GAMCTRL_VC52_MASK, 0x7),
        CFIELD_PREP(DSI_CMD2_BK0_GAMCTRL_VC80_MASK, 0x9),
        CFIELD_PREP(DSI_CMD2_BK0_GAMCTRL_VC108_MASK, 0x8),

        CFIELD_PREP(DSI_CMD2_BK0_GAMCTRL_VC147_MASK, 0x8),
        CFIELD_PREP(DSI_CMD2_BK0_GAMCTRL_VC175_MASK, 0x22),
        CFIELD_PREP(DSI_CMD2_BK0_GAMCTRL_VC203_MASK, 0x4),
        CFIELD_PREP(DSI_CMD2_BK0_GAMCTRL_AJ_MASK, 0) |
        CFIELD_PREP(DSI_CMD2_BK0_GAMCTRL_VC231_MASK, 0x10),

        CFIELD_PREP(DSI_CMD2_BK0_GAMCTRL_VC239_MASK, 0xe),
        CFIELD_PREP(DSI_CMD2_BK0_GAMCTRL_AJ_MASK, 0) |
        CFIELD_PREP(DSI_CMD2_BK0_GAMCTRL_VC247_MASK, 0x2c),
        CFIELD_PREP(DSI_CMD2_BK0_GAMCTRL_AJ_MASK, 0) |
        CFIELD_PREP(DSI_CMD2_BK0_GAMCTRL_VC251_MASK, 0x34),
        CFIELD_PREP(DSI_CMD2_BK0_GAMCTRL_AJ_MASK, 0) |
        CFIELD_PREP(DSI_CMD2_BK0_GAMCTRL_VC255_MASK, 0x1f)
    },
    .nlinv = 7,
    .vop_uv = 4400000,
    .vcom_uv = 337500,
    .vgh_mv = 15000,
    .vgl_mv = -9510,
    .avdd_mv = 6600,
    .avcl_mv = -4400,
    .gamma_op_bias = OP_BIAS_MAX,
    .input_op_bias = OP_BIAS_MIN,
    .output_op_bias = OP_BIAS_MIN,
    .t2d_ns = 1600,
    .t3d_ns = 10400,
    .eot_en = true,
    .gip_sequence = ts8550b_gip_sequence,
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
    // .nlinv = 7,
    // .vop_uv = 4400000,
    // .vcom_uv = 337500,
    // .vgh_mv = 15000,
    // .vgl_mv = -9510,
    // .avdd_mv = 6600,
    // .avcl_mv = -4400,
    // .gamma_op_bias = OP_BIAS_MAX,
    // .input_op_bias = OP_BIAS_MIN,
    // .output_op_bias = OP_BIAS_MIN,
    // .t2d_ns = 1600,
    // .t3d_ns = 10400,
    // .eot_en = true,
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

    dsi->mode_flags = MIPI_DSI_MODE_VIDEO | MIPI_DSI_MODE_VIDEO_BURST | MIPI_DSI_MODE_LPM;
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

MODULE_AUTHOR("Jagan Teki <jagan@amarulasolutions.com>");
MODULE_DESCRIPTION("Sitronix ST7701 LCD Panel Driver");
MODULE_LICENSE("GPL");

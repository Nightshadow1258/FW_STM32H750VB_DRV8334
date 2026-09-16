#include "drv8334.h"
#include <zephyr/kernel.h>
#include <zephyr/sys/crc.h>

#define SPI1_NODE DT_NODELABEL(drv8334)

#define DRV8334_CRC_POLY 0x2F
#define DRV8334_CRC_INIT 0xFF

LOG_MODULE_REGISTER(drv8334, CONFIG_LOG_DEFAULT_LEVEL);

/* SPI Mode 1 (CPHA=1, CPOL=0) with 8 bit word size */
const struct drv8334_config drv_cfg = {
    .spi = SPI_DT_SPEC_GET(SPI1_NODE, SPI_WORD_SET(8) | SPI_MODE_CPHA | SPI_TRANSFER_MSB, 0),
    .nsleep = GPIO_DT_SPEC_GET(SPI1_NODE, nsleep_gpios),
    .drvoff = GPIO_DT_SPEC_GET(SPI1_NODE, drvoff_gpios),
    .nfault = GPIO_DT_SPEC_GET(SPI1_NODE, nfault_gpios),
};

static uint8_t drv8334_calc_crc8(const uint8_t *data, size_t len)
{
    return crc8(data, len, DRV8334_CRC_POLY, DRV8334_CRC_INIT, false);
}

int drv8334_reg_write(const struct drv8334_config *cfg, uint8_t addr, uint16_t data)
{
    uint8_t tx_buf[4];
    uint8_t rx_buf[4];
    int ret;

    /* Format: [A6:A0, W0=0] [Data MSB] [Data LSB] [CRC] */
    tx_buf[0] = (addr << 1) & 0xFE; /* Bit 0 = 0 for WRITE */
    tx_buf[1] = (uint8_t)(data >> 8);
    tx_buf[2] = (uint8_t)(data & 0xFF);
    tx_buf[3] = drv8334_calc_crc8(tx_buf, 3);

    struct spi_buf tx = {.buf = tx_buf, .len = sizeof(tx_buf)};
    struct spi_buf_set tx_bufs = {.buffers = &tx, .count = 1};
    struct spi_buf rx = {.buf = rx_buf, .len = sizeof(rx_buf)};
    struct spi_buf_set rx_bufs = {.buffers = &rx, .count = 1};

    /* 1. Low-level SPI transfer check */
    ret = spi_transceive_dt(&cfg->spi, &tx_bufs, &rx_bufs);
    if (ret < 0)
    {
        LOG_ERR("SPI transceive failed for addr 0x%02X: %d", addr, ret);
        return ret;
    }

    /* 2. Received frame CRC verification */
    uint8_t expected_crc = drv8334_calc_crc8(rx_buf, 3);
    if (rx_buf[3] != expected_crc)
    {
        LOG_ERR("CRC error for addr 0x%02X (exp: 0x%02X, got: 0x%02X)",
                addr, expected_crc, rx_buf[3]);
        return -EBADMSG;
    }

    /* 3. Device status check (Evaluate Status Byte in rx_buf[0]) */
    /* Example: Bit 7 or specific fault bits returned by the driver */
    if (!(rx_buf[0] & 40))
    {
        LOG_WRN("DRV8334 reported fault status: 0x%02X on addr 0x%02X",
                rx_buf[0], addr);
        return -EIO;
    }

    return 0;
}

int drv8334_reg_read(const struct drv8334_config *cfg, uint8_t addr, uint16_t *data)
{
    uint8_t tx_buf[4];
    uint8_t rx_buf[4];

    /* Format: [A6:A0, W0=1] [0x00] [0x00] [CRC] */
    tx_buf[0] = ((addr << 1) & 0xFE) | 0x01; /* Bit 0 = 1 for READ */
    tx_buf[1] = 0x00;
    tx_buf[2] = 0x00;
    tx_buf[3] = drv8334_calc_crc8(tx_buf, 3);

    struct spi_buf tx = {.buf = tx_buf, .len = sizeof(tx_buf)};
    struct spi_buf_set tx_bufs = {.buffers = &tx, .count = 1};
    struct spi_buf rx = {.buf = rx_buf, .len = sizeof(rx_buf)};
    struct spi_buf_set rx_bufs = {.buffers = &rx, .count = 1};

    int ret = spi_transceive_dt(&cfg->spi, &tx_bufs, &rx_bufs);
    if (ret < 0)
    {
        return ret;
    }

    /* Verify received CRC from SDO */
    uint8_t expected_crc = drv8334_calc_crc8(rx_buf, 3);
    if (rx_buf[3] != expected_crc)
    {
        return -EBADMSG;
    }

    /* Construct 16-bit register value from rx_buf[1] and rx_buf[2] */
    *data = ((uint16_t)rx_buf[1] << 8) | rx_buf[2];
    return 0;
}

int drv8334_init(const struct drv8334_config *cfg)
{
    if (!spi_is_ready_dt(&cfg->spi))
        return -ENODEV;

    /* Configure GPIO control pins */
    gpio_pin_configure_dt(&cfg->nsleep, GPIO_OUTPUT_LOW);
    gpio_pin_configure_dt(&cfg->drvoff, GPIO_OUTPUT_HIGH);
    gpio_pin_configure_dt(&cfg->nfault, GPIO_INPUT);

    /* Wake up device */
    gpio_pin_set_dt(&cfg->nsleep, 1);
    k_msleep(2); /* Wait tWAKE time */
    uint16_t data;
    drv8334_reg_read(cfg, DRV8334_REG_IC_CTRL2, &data); /* Clear any latched faults */
    printk("INFO: DRV8334_REG_IC_CTRL2 is: 0x%04X\n", data);
    drv8334_reg_read(cfg, DRV8334_REG_IC_CTRL2, &data);      /* Clear any latched faults */
    printk("INFO: DRV8334_REG_IC_CTRL2 is: 0x%04X\n", data); /* check DRV8334_Register_Map.ods for details on the ini
     t configuration */
    drv8334_reg_write(cfg, DRV8334_REG_IC_CTRL3, 0xC009);
    drv8334_reg_write(cfg, DRV8334_REG_GD_CTRL1, 0x0328);
    drv8334_reg_write(cfg, DRV8334_REG_GD_CTRL2, 0x0717);
    drv8334_reg_write(cfg, DRV8334_REG_GD_CTRL3, 0x0717);
    drv8334_reg_write(cfg, DRV8334_REG_GD_CTRL3B, 0x2323);
    drv8334_reg_write(cfg, DRV8334_REG_GD_CTRL4, 0x0200);
    drv8334_reg_write(cfg, DRV8334_REG_GD_CTRL5, 0x0007);
    drv8334_reg_write(cfg, DRV8334_REG_GD_CTRL6, 0x2323);
    drv8334_reg_write(cfg, DRV8334_REG_GD_CTRL7, 0x2323);

    drv8334_reg_write(cfg, DRV8334_REG_CSA_CTRL, 0x0888); /* Set CSA gain to 40V/V for all channels */

    drv8334_reg_write(cfg, DRV8334_REG_MON_CTRL1, 0x0200);
    drv8334_reg_write(cfg, DRV8334_REG_MON_CTRL2, 0x1101);
    drv8334_reg_write(cfg, DRV8334_REG_MON_CTRL3, 0x0042);
    drv8334_reg_write(cfg, DRV8334_REG_MON_CTRL4, 0x0032);
    // drv8334_reg_write(cfg, DRV8334_REG_MON_CTRL5, 0x0000); /* unchanged */
    drv8334_reg_write(cfg, DRV8334_REG_MON_CTRL6, 0x0200);

    // drv8334_reg_write(cfg, DRV8334_REG_DIAG_CTRL1, 0x0000); /* unchanged */
    /* Enable Gate Driver */
    gpio_pin_set_dt(&cfg->drvoff, 0);
    
    k_usleep(10); /*  */
    /* Clear fault register */
    drv8334_reg_write(cfg, DRV8334_REG_IC_CTRL2, 0x8807); /* Clear latched faults */
    drv8334_reg_read(cfg, DRV8334_REG_IC_CTRL2, &data);   /* Clear any latched faults */
    printk("INFO: DRV8334_REG_IC_CTRL2 is: 0x%04X\n", data);


    return 0;
}

static void decode_ic_stat1(const struct shell *sh, uint16_t val)
{
    shell_print(sh, "    [15] SPI_OK   : %u (%s)", (val >> 15) & 0x01, ((val >> 15) & 0x01) ? "No SPI Fault" : "SPI Fault Detected");
    shell_print(sh, "    [14] FAULT    : %u (%s)", (val >> 14) & 0x01, ((val >> 14) & 0x01) ? "Fault Event Active" : "No Fault");
    shell_print(sh, "    [13] WARN     : %u (%s)", (val >> 13) & 0x01, ((val >> 13) & 0x01) ? "Warning Active" : "No Warning");
    shell_print(sh, "    [12] VDS      : %u (%s)", (val >> 12) & 0x01, ((val >> 12) & 0x01) ? "VDS Overcurrent Detected" : "No VDS Fault");
    shell_print(sh, "    [11] VGS      : %u (%s)", (val >> 11) & 0x01, ((val >> 11) & 0x01) ? "VGS Event Detected" : "No VGS Fault");
    shell_print(sh, "    [10] SNS_OCP  : %u (%s)", (val >> 10) & 0x01, ((val >> 10) & 0x01) ? "Sense Overcurrent Detected" : "No Sense OCP Fault");
    shell_print(sh, "    [9]  OV       : %u (%s)", (val >> 9) & 0x01, ((val >> 9) & 0x01) ? "Overvoltage Detected" : "No Overvoltage");
    shell_print(sh, "    [8]  UV       : %u (%s)", (val >> 8) & 0x01, ((val >> 8) & 0x01) ? "Undervoltage Detected" : "No Undervoltage");
    shell_print(sh, "    [1]  OTW      : %u (%s)", (val >> 1) & 0x01, ((val >> 1) & 0x01) ? "Overtemperature Warning" : "No Temp Warning");
    shell_print(sh, "    [0]  DRV_STAT : %u (%s)", (val >> 0) & 0x01, ((val >> 0) & 0x01) ? "Driver Enabled" : "Driver Disabled");
}

static void decode_ic_stat2(const struct shell *sh, uint16_t val)
{
    shell_print(sh, "    [15] CBC_ST    : %u (%s)", (val >> 15) & 0x01, ((val >> 15) & 0x01) ? "CBC Counter > 0" : "CBC Counter 0");
    shell_print(sh, "    [10] SNS_OCP_A : %u (%s)", (val >> 10) & 0x01, ((val >> 10) & 0x01) ? "Phase A Sense OCP Fault" : "OK");
    shell_print(sh, "    [9]  SNS_OCP_B : %u (%s)", (val >> 9) & 0x01, ((val >> 9) & 0x01) ? "Phase B Sense OCP Fault" : "OK");
    shell_print(sh, "    [8]  SNS_OCP_C : %u (%s)", (val >> 8) & 0x01, ((val >> 8) & 0x01) ? "Phase C Sense OCP Fault" : "OK");
    shell_print(sh, "    [5]  VDS_HA    : %u (%s)", (val >> 5) & 0x01, ((val >> 5) & 0x01) ? "Phase A High-side VDS Fault" : "OK");
    shell_print(sh, "    [4]  VDS_LA    : %u (%s)", (val >> 4) & 0x01, ((val >> 4) & 0x01) ? "Phase A Low-side VDS Fault" : "OK");
    shell_print(sh, "    [3]  VDS_HB    : %u (%s)", (val >> 3) & 0x01, ((val >> 3) & 0x01) ? "Phase B High-side VDS Fault" : "OK");
    shell_print(sh, "    [2]  VDS_LB    : %u (%s)", (val >> 2) & 0x01, ((val >> 2) & 0x01) ? "Phase B Low-side VDS Fault" : "OK");
    shell_print(sh, "    [1]  VDS_HC    : %u (%s)", (val >> 1) & 0x01, ((val >> 1) & 0x01) ? "Phase C High-side VDS Fault" : "OK");
    shell_print(sh, "    [0]  VDS_LC    : %u (%s)", (val >> 0) & 0x01, ((val >> 0) & 0x01) ? "Phase C Low-side VDS Fault" : "OK");
}

static void decode_ic_stat3(const struct shell *sh, uint16_t val)
{
    shell_print(sh, "    [5] VGS_HA : %u (%s)", (val >> 5) & 0x01, ((val >> 5) & 0x01) ? "Phase A High-side VGS Fault" : "OK");
    shell_print(sh, "    [4] VGS_LA : %u (%s)", (val >> 4) & 0x01, ((val >> 4) & 0x01) ? "Phase A Low-side VGS Fault" : "OK");
    shell_print(sh, "    [3] VGS_HB : %u (%s)", (val >> 3) & 0x01, ((val >> 3) & 0x01) ? "Phase B High-side VGS Fault" : "OK");
    shell_print(sh, "    [2] VGS_LB : %u (%s)", (val >> 2) & 0x01, ((val >> 2) & 0x01) ? "Phase B Low-side VGS Fault" : "OK");
    shell_print(sh, "    [1] VGS_HC : %u (%s)", (val >> 1) & 0x01, ((val >> 1) & 0x01) ? "Phase C High-side VGS Fault" : "OK");
    shell_print(sh, "    [0] VGS_LC : %u (%s)", (val >> 0) & 0x01, ((val >> 0) & 0x01) ? "Phase C Low-side VGS Fault" : "OK");
}

static void decode_ic_stat4(const struct shell *sh, uint16_t val)
{
    shell_print(sh, "    [15] PVDD_OV   : %u (%s)", (val >> 15) & 0x01, ((val >> 15) & 0x01) ? "PVDD Overvoltage" : "OK");
    shell_print(sh, "    [14] PVDD_UV   : %u (%s)", (val >> 14) & 0x01, ((val >> 14) & 0x01) ? "PVDD Undervoltage" : "OK");
    shell_print(sh, "    [13] VDRAIN_OV : %u (%s)", (val >> 13) & 0x01, ((val >> 13) & 0x01) ? "VDRAIN Overvoltage" : "OK");
    shell_print(sh, "    [12] VDRAIN_UV : %u (%s)", (val >> 12) & 0x01, ((val >> 12) & 0x01) ? "VDRAIN Undervoltage" : "OK");
    shell_print(sh, "    [11] VCP_OV    : %u (%s)", (val >> 11) & 0x01, ((val >> 11) & 0x01) ? "VCP Overvoltage" : "OK");
    shell_print(sh, "    [10] VCP_UV    : %u (%s)", (val >> 10) & 0x01, ((val >> 10) & 0x01) ? "VCP Undervoltage" : "OK");
    shell_print(sh, "    [9]  GVDD_OV   : %u (%s)", (val >> 9) & 0x01, ((val >> 9) & 0x01) ? "GVDD Overvoltage" : "OK");
    shell_print(sh, "    [8]  GVDD_UV   : %u (%s)", (val >> 8) & 0x01, ((val >> 8) & 0x01) ? "GVDD Undervoltage" : "OK");
    shell_print(sh, "    [5]  BSTA_OV   : %u (%s)", (val >> 5) & 0x01, ((val >> 5) & 0x01) ? "BST Phase A Overvoltage" : "OK");
    shell_print(sh, "    [4]  BSTA_UV   : %u (%s)", (val >> 4) & 0x01, ((val >> 4) & 0x01) ? "BST Phase A Undervoltage" : "OK");
    shell_print(sh, "    [3]  BSTB_OV   : %u (%s)", (val >> 3) & 0x01, ((val >> 3) & 0x01) ? "BST Phase B Overvoltage" : "OK");
    shell_print(sh, "    [2]  BSTB_UV   : %u (%s)", (val >> 2) & 0x01, ((val >> 2) & 0x01) ? "BST Phase B Undervoltage" : "OK");
    shell_print(sh, "    [1]  BSTC_OV   : %u (%s)", (val >> 1) & 0x01, ((val >> 1) & 0x01) ? "BST Phase C Overvoltage" : "OK");
    shell_print(sh, "    [0]  BSTC_UV   : %u (%s)", (val >> 0) & 0x01, ((val >> 0) & 0x01) ? "BST Phase C Undervoltage" : "OK");
}

static void decode_ic_stat5(const struct shell *sh, uint16_t val)
{
    shell_print(sh, "    [14] PVDD_UVW        : %u (%s)", (val >> 14) & 0x01, ((val >> 14) & 0x01) ? "PVDD Undervoltage Warning" : "OK");
    shell_print(sh, "    [10] GVDD_CP_LDO     : %u (%s)", (val >> 10) & 0x01, ((val >> 10) & 0x01) ? "LDO Mode" : "Charge Pump Mode");
    shell_print(sh, "    [9]  OTSD            : %u (%s)", (val >> 9) & 0x01, ((val >> 9) & 0x01) ? "Overtemperature Shutdown Active" : "OK");
    shell_print(sh, "    [8]  WDT_FLT         : %u (%s)", (val >> 8) & 0x01, ((val >> 8) & 0x01) ? "Watchdog Timer Fault" : "OK");
    shell_print(sh, "    [7]  SPI_CRC_FLT     : %u (%s)", (val >> 7) & 0x01, ((val >> 7) & 0x01) ? "SPI CRC Fault" : "OK");
    shell_print(sh, "    [6]  SPI_ADDR_FLT    : %u (%s)", (val >> 6) & 0x01, ((val >> 6) & 0x01) ? "SPI Address Fault" : "OK");
    shell_print(sh, "    [5]  SPI_CLK_FLT     : %u (%s)", (val >> 5) & 0x01, ((val >> 5) & 0x01) ? "SPI Clock Framing Fault" : "OK");
    shell_print(sh, "    [4]  OTP_CRC_FLT     : %u (%s)", (val >> 4) & 0x01, ((val >> 4) & 0x01) ? "Factory OTP CRC Fault" : "OK");
    shell_print(sh, "    [3]  OTP_USR_CRC_FLT : %u (%s)", (val >> 3) & 0x01, ((val >> 3) & 0x01) ? "User OTP CRC Fault" : "OK");
    shell_print(sh, "    [1]  STP_FLT         : %u (%s)", (val >> 1) & 0x01, ((val >> 1) & 0x01) ? "Shoot Through Protection Violation" : "OK");
    shell_print(sh, "    [0]  DEADT_FLT       : %u (%s)", (val >> 0) & 0x01, ((val >> 0) & 0x01) ? "Dead Time Violation" : "OK");
}

static void decode_ic_stat6(const struct shell *sh, uint16_t val)
{
    shell_print(sh, "    [15] PHCA_FLT     : %u (%s)", (val >> 15) & 0x01, ((val >> 15) & 0x01) ? "Phase A Comparator Fault" : "OK");
    shell_print(sh, "    [14] PHCB_FLT     : %u (%s)", (val >> 14) & 0x01, ((val >> 14) & 0x01) ? "Phase B Comparator Fault" : "OK");
    shell_print(sh, "    [13] PHCC_FLT     : %u (%s)", (val >> 13) & 0x01, ((val >> 13) & 0x01) ? "Phase C Comparator Fault" : "OK");
    shell_print(sh, "    [11] VREF_OV      : %u (%s)", (val >> 11) & 0x01, ((val >> 11) & 0x01) ? "VREF Overvoltage" : "OK");
    shell_print(sh, "    [10] VREF_UV      : %u (%s)", (val >> 10) & 0x01, ((val >> 10) & 0x01) ? "VREF Undervoltage" : "OK");
    shell_print(sh, "    [9]  VDDSDO_UV    : %u (%s)", (val >> 9) & 0x01, ((val >> 9) & 0x01) ? "VDDSDO Undervoltage" : "OK");
    shell_print(sh, "    [7]  DVDD_OV      : %u (%s)", (val >> 7) & 0x01, ((val >> 7) & 0x01) ? "DVDD Overvoltage" : "OK");
    shell_print(sh, "    [4]  ABIST_FLT    : %u (%s)", (val >> 4) & 0x01, ((val >> 4) & 0x01) ? "Analog BIST Fault" : "OK");
    shell_print(sh, "    [3]  DEV_MODE_FLT : %u (%s)", (val >> 3) & 0x01, ((val >> 3) & 0x01) ? "Device Mode Fault" : "OK");
    shell_print(sh, "    [0]  CLK_MON_FLT  : %u (%s)", (val >> 0) & 0x01, ((val >> 0) & 0x01) ? "Clock Monitor Fault" : "OK");
}

static void drv8334_decode_status_reg(const struct shell *sh, uint8_t addr, uint16_t val)
{
    switch (addr)
    {
    case DRV8334_REG_IC_STAT1:
        decode_ic_stat1(sh, val);
        break;
    case DRV8334_REG_IC_STAT2:
        decode_ic_stat2(sh, val);
        break;
    case DRV8334_REG_IC_STAT3:
        decode_ic_stat3(sh, val);
        break;
    case DRV8334_REG_IC_STAT4:
        decode_ic_stat4(sh, val);
        break;
    case DRV8334_REG_IC_STAT5:
        decode_ic_stat5(sh, val);
        break;
    case DRV8334_REG_IC_STAT6:
        decode_ic_stat6(sh, val);
        break;
    default:
        break;
    }
}

int drv8334_dump_status_regs(const struct drv8334_config *cfg, const struct shell *sh)
{
    static const struct
    {
        uint8_t addr;
        const char *acronym;
        const char *name;
    } status_regs[] = {
        {DRV8334_REG_IC_STAT1, "IC_STAT1", "IC Status Register 1"},
        {DRV8334_REG_IC_STAT2, "IC_STAT2", "IC Status Register 2"},
        {DRV8334_REG_IC_STAT3, "IC_STAT3", "IC Status Register 3"},
        {DRV8334_REG_IC_STAT4, "IC_STAT4", "IC Status Register 4"},
        {DRV8334_REG_IC_STAT5, "IC_STAT5", "IC Status Register 5"},
        {DRV8334_REG_IC_STAT6, "IC_STAT6", "IC Status Register 6"},
    };

    shell_print(sh, "================ STATUS REGISTERS ================");
    for (size_t i = 0; i < ARRAY_SIZE(status_regs); i++)
    {
        uint16_t val = 0;
        int ret = drv8334_reg_read(cfg, status_regs[i].addr, &val);
        if (ret < 0)
        {
            shell_error(sh, "  [0x%02X] %-10s (%-22s): ERROR (%d)",
                        status_regs[i].addr, status_regs[i].acronym, status_regs[i].name, ret);
        }
        else
        {
            shell_print(sh, "  [0x%02X] %-10s (%-22s): 0x%04X",
                        status_regs[i].addr, status_regs[i].acronym, status_regs[i].name, val);
            drv8334_decode_status_reg(sh, status_regs[i].addr, val);
        }
    }
    return 0;
}

int drv8334_dump_control_regs(const struct drv8334_config *cfg, const struct shell *sh)
{
    static const struct
    {
        uint8_t addr;
        const char *acronym;
        const char *name;
    } control_regs[] = {
        {DRV8334_REG_IC_CTRL1, "IC_CTRL1", "IC Control Register 1"},
        {DRV8334_REG_IC_CTRL2, "IC_CTRL2", "IC Control Register 2"},
        {DRV8334_REG_IC_CTRL3, "IC_CTRL3", "IC Control Register 3"},
        {DRV8334_REG_GD_CTRL1, "GD_CTRL1", "Gate Drive Control 1"},
        {DRV8334_REG_GD_CTRL2, "GD_CTRL2", "Gate Drive Control 2"},
        {DRV8334_REG_GD_CTRL3, "GD_CTRL3", "Gate Drive Control 3"},
        {DRV8334_REG_GD_CTRL3B, "GD_CTRL3B", "Gate Drive Control 3B"},
        {DRV8334_REG_GD_CTRL4, "GD_CTRL4", "Gate Drive Control 4"},
        {DRV8334_REG_GD_CTRL5, "GD_CTRL5", "Gate Drive Control 5"},
        {DRV8334_REG_GD_CTRL6, "GD_CTRL6", "Gate Drive Control 6"},
        {DRV8334_REG_GD_CTRL7, "GD_CTRL7", "Gate Drive Control 7"},
        {DRV8334_REG_CSA_CTRL, "CSA_CTRL", "CSA Control Register"},
        {DRV8334_REG_MON_CTRL1, "MON_CTRL1", "Monitor Control Reg 1"},
        {DRV8334_REG_MON_CTRL2, "MON_CTRL2", "Monitor Control Reg 2"},
        {DRV8334_REG_MON_CTRL3, "MON_CTRL3", "Monitor Control Reg 3"},
        {DRV8334_REG_MON_CTRL4, "MON_CTRL4", "Monitor Control Reg 4"},
        {DRV8334_REG_MON_CTRL5, "MON_CTRL5", "Monitor Control Reg 5"},
        {DRV8334_REG_MON_CTRL6, "MON_CTRL6", "Monitor Control Reg 6"},
        {DRV8334_REG_DIAG_CTRL1, "DIAG_CTRL1", "Diagnostic Control 1"},
        {DRV8334_REG_SPI_TEST, "SPI_TEST", "SPI Test Register"},
        {DRV8334_REG_OTP_USR, "OTP_USR", "OTP User Control"},
    };

    shell_print(sh, "================ CONTROL REGISTERS ================");
    for (size_t i = 0; i < ARRAY_SIZE(control_regs); i++)
    {
        uint16_t val = 0;
        int ret = drv8334_reg_read(cfg, control_regs[i].addr, &val);
        if (ret < 0)
        {
            shell_error(sh, "  [0x%02X] %-10s (%-22s): ERROR (%d)",
                        control_regs[i].addr, control_regs[i].acronym, control_regs[i].name, ret);
        }
        else
        {
            shell_print(sh, "  [0x%02X] %-10s (%-22s): 0x%04X",
                        control_regs[i].addr, control_regs[i].acronym, control_regs[i].name, val);
        }
    }
    return 0;
}

static int shell_gdu_dump_status(const struct shell *sh, size_t argc, char **argv)
{
    return drv8334_dump_status_regs(&drv_cfg, sh);
}

static int shell_gdu_dump_control(const struct shell *sh, size_t argc, char **argv)
{
    return drv8334_dump_control_regs(&drv_cfg, sh);
}

static int shell_gdu_reg_read(const struct shell *sh, size_t argc, char **argv)
{
    if (argc < 2)
    {
        shell_error(sh, "Usage: gdu_reg_read <reg_addr_hex>");
        return -EINVAL;
    }

    char *endptr;
    uint8_t addr = (uint8_t)strtoul(argv[1], &endptr, 16);

    if (*endptr != '\0')
    {
        shell_error(sh, "Invalid hex address format: %s", argv[1]);
        return -EINVAL;
    }

    uint16_t val = 0;
    int ret = drv8334_reg_read(&drv_cfg, addr, &val);
    if (ret < 0)
    {
        shell_error(sh, "Failed to read register [0x%02X], err: %d", addr, ret);
    }
    else
    {
        shell_print(sh, "Reg [0x%02X] = 0x%04X", addr, val);
    }

    return ret;
}

static int shell_gdu_reg_write(const struct shell *sh, size_t argc, char **argv)
{
    if (argc < 3)
    {
        shell_error(sh, "Usage: gdu_reg_write <reg_addr_hex> <val_hex>");
        return -EINVAL;
    }

    char *endptr1, *endptr2;
    uint8_t addr = (uint8_t)strtoul(argv[1], &endptr1, 16);
    uint16_t val = (uint16_t)strtoul(argv[2], &endptr2, 16);

    if (*endptr1 != '\0' || *endptr2 != '\0')
    {
        shell_error(sh, "Invalid hex format in arguments");
        return -EINVAL;
    }

    int ret = drv8334_reg_write(&drv_cfg, addr, val);
    if (ret < 0)
    {
        shell_error(sh, "Failed to write register [0x%02X], err: %d", addr, ret);
    }
    else
    {
        shell_print(sh, "Wrote 0x%04X to Reg [0x%02X]", val, addr);
    }

    return ret;
}

SHELL_CMD_REGISTER(gdu_dump_status, NULL, "Dump DRV8334 status registers", shell_gdu_dump_status);
SHELL_CMD_REGISTER(gdu_dump_control, NULL, "Dump DRV8334 control registers", shell_gdu_dump_control);
SHELL_CMD_REGISTER(gdu_reg_read, NULL, "Read DRV8334 register: gdu_reg_read <addr_hex>", shell_gdu_reg_read);
SHELL_CMD_REGISTER(gdu_reg_write, NULL, "Write DRV8334 register: gdu_reg_write <addr_hex> <val_hex>", shell_gdu_reg_write);


static int shell_gdu_drv(const struct shell *sh, size_t argc, char **argv)
{
    if (argc < 2) {
        shell_error(sh, "Usage: gdu_drv <on|off>");
        return -EINVAL;
    }

    if (strcmp(argv[1], "on") == 0) {
        /* Active LOW or OFF pin state: DRVOFF = 0 enables gate drivers */
        gpio_pin_set_dt(&drv_cfg.drvoff, 0);
        shell_print(sh, "DRV8334 Gate Driver ENABLED");
    } else if (strcmp(argv[1], "off") == 0) {
        /* DRVOFF = 1 disables gate drivers */
        gpio_pin_set_dt(&drv_cfg.drvoff, 1);
        shell_print(sh, "DRV8334 Gate Driver DISABLED");
    } else {
        shell_error(sh, "Invalid argument: %s. Use 'on' or 'off'.", argv[1]);
        return -EINVAL;
    }

    return 0;
}
SHELL_CMD_REGISTER(gdu_drv, NULL, "Control DRV8334 gate driver state: gdu_drv <on|off>", shell_gdu_drv);
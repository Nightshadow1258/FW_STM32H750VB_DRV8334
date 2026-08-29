#include "drv8334.h"
#include <zephyr/kernel.h>
#include <zephyr/sys/crc.h>

#define SPI1_NODE DT_NODELABEL(drv8334)

#define DRV8334_CRC_POLY 0x2F
#define DRV8334_CRC_INIT 0xFF

/* SPI Mode 1 (CPHA=1, CPOL=0) with 8 bit word size */
const struct drv8334_config drv_cfg = {
    .spi    = SPI_DT_SPEC_GET(SPI1_NODE, SPI_WORD_SET(8) | SPI_MODE_CPHA | SPI_TRANSFER_MSB, 0),
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

    /* Format: [A6:A0, W0=0] [Data MSB] [Data LSB] [CRC] */
    tx_buf[0] = (addr << 1) & 0xFE; /* Bit 0 = 0 for WRITE */
    tx_buf[1] = (uint8_t)(data >> 8);
    tx_buf[2] = (uint8_t)(data & 0xFF);
    tx_buf[3] = drv8334_calc_crc8(tx_buf, 3);

    struct spi_buf tx = { .buf = tx_buf, .len = sizeof(tx_buf) };
    struct spi_buf_set tx_bufs = { .buffers = &tx, .count = 1 };
    struct spi_buf rx = { .buf = rx_buf, .len = sizeof(rx_buf) };
    struct spi_buf_set rx_bufs = { .buffers = &rx, .count = 1 };

    return spi_transceive_dt(&cfg->spi, &tx_bufs, &rx_bufs);
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

    struct spi_buf tx = { .buf = tx_buf, .len = sizeof(tx_buf) };
    struct spi_buf_set tx_bufs = { .buffers = &tx, .count = 1 };
    struct spi_buf rx = { .buf = rx_buf, .len = sizeof(rx_buf) };
    struct spi_buf_set rx_bufs = { .buffers = &rx, .count = 1 };

    int ret = spi_transceive_dt(&cfg->spi, &tx_bufs, &rx_bufs);
    if (ret < 0) {
        return ret;
    }

    /* Verify received CRC from SDO */
    uint8_t expected_crc = drv8334_calc_crc8(rx_buf, 3);
    if (rx_buf[3] != expected_crc) {
        return -EBADMSG;
    }

    /* Construct 16-bit register value from rx_buf[1] and rx_buf[2] */
    *data = ((uint16_t)rx_buf[1] << 8) | rx_buf[2];
    return 0;
}

int drv8334_init(const struct drv8334_config *cfg)
{
    if (!spi_is_ready_dt(&cfg->spi)) return -ENODEV;

    /* Configure GPIO control pins */
    gpio_pin_configure_dt(&cfg->nsleep, GPIO_OUTPUT_LOW);
    gpio_pin_configure_dt(&cfg->drvoff, GPIO_OUTPUT_HIGH);
    gpio_pin_configure_dt(&cfg->nfault, GPIO_INPUT);

    /* Wake up device */
    gpio_pin_set_dt(&cfg->nsleep, 1);
    k_msleep(2); /* Wait tWAKE time */

    /* check DRV8334_Register_Map.ods for details on the init configuration */
    drv8334_reg_write(cfg, DRV8334_REG_IC_CTRL3, 0xC009);
    drv8334_reg_write(cfg, DRV8334_REG_GD_CTRL1, 0x0328);
    drv8334_reg_write(cfg, DRV8334_REG_GD_CTRL2, 0x0717);
    drv8334_reg_write(cfg, DRV8334_REG_GD_CTRL3, 0x0717);
    drv8334_reg_write(cfg, DRV8334_REG_GD_CTRL3B, 0x2323);
    drv8334_reg_write(cfg, DRV8334_REG_GD_CTRL4, 0x0200);
    drv8334_reg_write(cfg, DRV8334_REG_GD_CTRL5, 0x0007);
    drv8334_reg_write(cfg, DRV8334_REG_GD_CTRL6, 0x2323);
    drv8334_reg_write(cfg, DRV8334_REG_GD_CTRL7, 0x2323);

    drv8334_reg_write(cfg, DRV8334_REG_CSA_CTRL, 0x0888);  /* Set CSA gain to 40V/V for all channels */

    drv8334_reg_write(cfg, DRV8334_REG_MON_CTRL1, 0x0200);
    drv8334_reg_write(cfg, DRV8334_REG_MON_CTRL2, 0x1101);
    drv8334_reg_write(cfg, DRV8334_REG_MON_CTRL3, 0x0042);
    drv8334_reg_write(cfg, DRV8334_REG_MON_CTRL4, 0x0032);
    // drv8334_reg_write(cfg, DRV8334_REG_MON_CTRL5, 0x0000); /* unchanged */
    drv8334_reg_write(cfg, DRV8334_REG_MON_CTRL6, 0x0200);
    
    // drv8334_reg_write(cfg, DRV8334_REG_DIAG_CTRL1, 0x0000); /* unchanged */

    /* Clear fault register */
    drv8334_reg_write(cfg, DRV8334_REG_IC_CTRL2, 0x8807); /* Clear latched faults */
    /* Enable Gate Driver */
    //gpio_pin_set_dt(&cfg->drvoff, 0);

    return 0;
}

int drv8334_dump_status_regs(const struct drv8334_config *cfg, const struct shell *sh)
{
    static const struct {
        uint8_t addr;
        const char *acronym;
        const char *name;
    } status_regs[] = {
        { DRV8334_REG_IC_STAT1, "IC_STAT1", "IC Status Register 1" },
        { DRV8334_REG_IC_STAT2, "IC_STAT2", "IC Status Register 2" },
        { DRV8334_REG_IC_STAT3, "IC_STAT3", "IC Status Register 3" },
        { DRV8334_REG_IC_STAT4, "IC_STAT4", "IC Status Register 4" },
        { DRV8334_REG_IC_STAT5, "IC_STAT5", "IC Status Register 5" },
        { DRV8334_REG_IC_STAT6, "IC_STAT6", "IC Status Register 6" },
    };

    shell_print(sh, "================ STATUS REGISTERS ================");
    for (size_t i = 0; i < ARRAY_SIZE(status_regs); i++) {
        uint16_t val = 0;
        int ret = drv8334_reg_read(cfg, status_regs[i].addr, &val);
        if (ret < 0) {
            shell_error(sh, "  [0x%02X] %-10s (%-22s): ERROR (%d)", 
                        status_regs[i].addr, status_regs[i].acronym, status_regs[i].name, ret);
        } else {
            shell_print(sh, "  [0x%02X] %-10s (%-22s): 0x%04X", 
                        status_regs[i].addr, status_regs[i].acronym, status_regs[i].name, val);
        }
    }
    return 0;
}

int drv8334_dump_control_regs(const struct drv8334_config *cfg, const struct shell *sh)
{
    static const struct {
        uint8_t addr;
        const char *acronym;
        const char *name;
    } control_regs[] = {
        { DRV8334_REG_IC_CTRL1,  "IC_CTRL1",  "IC Control Register 1" },
        { DRV8334_REG_IC_CTRL2,  "IC_CTRL2",  "IC Control Register 2" },
        { DRV8334_REG_IC_CTRL3,  "IC_CTRL3",  "IC Control Register 3" },
        { DRV8334_REG_GD_CTRL1,  "GD_CTRL1",  "Gate Drive Control 1" },
        { DRV8334_REG_GD_CTRL2,  "GD_CTRL2",  "Gate Drive Control 2" },
        { DRV8334_REG_GD_CTRL3,  "GD_CTRL3",  "Gate Drive Control 3" },
        { DRV8334_REG_GD_CTRL3B, "GD_CTRL3B", "Gate Drive Control 3B" },
        { DRV8334_REG_GD_CTRL4,  "GD_CTRL4",  "Gate Drive Control 4" },
        { DRV8334_REG_GD_CTRL5,  "GD_CTRL5",  "Gate Drive Control 5" },
        { DRV8334_REG_GD_CTRL6,  "GD_CTRL6",  "Gate Drive Control 6" },
        { DRV8334_REG_GD_CTRL7,  "GD_CTRL7",  "Gate Drive Control 7" },
        { DRV8334_REG_CSA_CTRL,  "CSA_CTRL",  "CSA Control Register" },
        { DRV8334_REG_MON_CTRL1, "MON_CTRL1", "Monitor Control Reg 1" },
        { DRV8334_REG_MON_CTRL2, "MON_CTRL2", "Monitor Control Reg 2" },
        { DRV8334_REG_MON_CTRL3, "MON_CTRL3", "Monitor Control Reg 3" },
        { DRV8334_REG_MON_CTRL4, "MON_CTRL4", "Monitor Control Reg 4" },
        { DRV8334_REG_MON_CTRL5, "MON_CTRL5", "Monitor Control Reg 5" },
        { DRV8334_REG_MON_CTRL6, "MON_CTRL6", "Monitor Control Reg 6" },
        { DRV8334_REG_DIAG_CTRL1,"DIAG_CTRL1","Diagnostic Control 1" },
        { DRV8334_REG_SPI_TEST,  "SPI_TEST",  "SPI Test Register" },
        { DRV8334_REG_OTP_USR,   "OTP_USR",   "OTP User Control" },
    };

    shell_print(sh, "================ CONTROL REGISTERS ================");
    for (size_t i = 0; i < ARRAY_SIZE(control_regs); i++) {
        uint16_t val = 0;
        int ret = drv8334_reg_read(cfg, control_regs[i].addr, &val);
        if (ret < 0) {
            shell_error(sh, "  [0x%02X] %-10s (%-22s): ERROR (%d)", 
                        control_regs[i].addr, control_regs[i].acronym, control_regs[i].name, ret);
        } else {
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
    if (argc < 2) {
        shell_error(sh, "Usage: gdu_reg_read <reg_addr_hex>");
        return -EINVAL;
    }

    char *endptr;
    uint8_t addr = (uint8_t)strtoul(argv[1], &endptr, 16);

    if (*endptr != '\0') {
        shell_error(sh, "Invalid hex address format: %s", argv[1]);
        return -EINVAL;
    }

    uint16_t val = 0;
    int ret = drv8334_reg_read(&drv_cfg, addr, &val);
    if (ret < 0) {
        shell_error(sh, "Failed to read register [0x%02X], err: %d", addr, ret);
    } else {
        shell_print(sh, "Reg [0x%02X] = 0x%04X", addr, val);
    }

    return ret;
}

static int shell_gdu_reg_write(const struct shell *sh, size_t argc, char **argv)
{
    if (argc < 3) {
        shell_error(sh, "Usage: gdu_reg_write <reg_addr_hex> <val_hex>");
        return -EINVAL;
    }

    char *endptr1, *endptr2;
    uint8_t addr = (uint8_t)strtoul(argv[1], &endptr1, 16);
    uint16_t val = (uint16_t)strtoul(argv[2], &endptr2, 16);

    if (*endptr1 != '\0' || *endptr2 != '\0') {
        shell_error(sh, "Invalid hex format in arguments");
        return -EINVAL;
    }

    int ret = drv8334_reg_write(&drv_cfg, addr, val);
    if (ret < 0) {
        shell_error(sh, "Failed to write register [0x%02X], err: %d", addr, ret);
    } else {
        shell_print(sh, "Wrote 0x%04X to Reg [0x%02X]", val, addr);
    }

    return ret;
}



SHELL_CMD_REGISTER(gdu_dump_status, NULL, "Dump DRV8334 status registers", shell_gdu_dump_status);
SHELL_CMD_REGISTER(gdu_dump_control, NULL, "Dump DRV8334 control registers", shell_gdu_dump_control);
SHELL_CMD_REGISTER(gdu_reg_read, NULL, "Read DRV8334 register: gdu_reg_read <addr_hex>", shell_gdu_reg_read);
SHELL_CMD_REGISTER(gdu_reg_write, NULL, "Write DRV8334 register: gdu_reg_write <addr_hex> <val_hex>", shell_gdu_reg_write);
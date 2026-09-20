#include "drv8334.h"
#include <zephyr/kernel.h>
#include <zephyr/sys/crc.h>
#include <zephyr/shell/shell.h>
#include <stdlib.h>
#include <string.h>

#define SPI1_NODE DT_NODELABEL(drv8334)

#define DRV8334_CRC_POLY 0x2F
#define DRV8334_CRC_INIT 0xFF

LOG_MODULE_REGISTER(drv8334, CONFIG_LOG_DEFAULT_LEVEL);

const struct drv8334_config drv_cfg = {
    .spi = SPI_DT_SPEC_GET(SPI1_NODE, SPI_WORD_SET(8) | SPI_MODE_CPHA | SPI_TRANSFER_MSB, 0),
    .nsleep = GPIO_DT_SPEC_GET(SPI1_NODE, nsleep_gpios),
    .drvoff = GPIO_DT_SPEC_GET(SPI1_NODE, drvoff_gpios),
    .nfault = GPIO_DT_SPEC_GET(SPI1_NODE, nfault_gpios),
};

struct bit_desc {
    uint8_t bit;
    const char *name;
    const char *active_str;
};

static inline uint8_t drv8334_calc_crc8(const uint8_t *data, size_t len)
{
    return crc8(data, len, DRV8334_CRC_POLY, DRV8334_CRC_INIT, false);
}

int drv8334_reg_write(const struct drv8334_config *cfg, uint8_t addr, uint16_t data)
{
    uint8_t tx_buf[4] = {
        [0] = (addr << 1) & 0xFE,
        [1] = (uint8_t)(data >> 8),
        [2] = (uint8_t)(data & 0xFF),
    };
    uint8_t rx_buf[4];
    
    tx_buf[3] = drv8334_calc_crc8(tx_buf, 3);

    const struct spi_buf tx = {.buf = tx_buf, .len = sizeof(tx_buf)};
    const struct spi_buf_set tx_bufs = {.buffers = &tx, .count = 1};
    const struct spi_buf rx = {.buf = rx_buf, .len = sizeof(rx_buf)};
    const struct spi_buf_set rx_bufs = {.buffers = &rx, .count = 1};

    int ret = spi_transceive_dt(&cfg->spi, &tx_bufs, &rx_bufs);
    if (ret < 0) {
        LOG_ERR("SPI transceive failed for addr 0x%02X: %d", addr, ret);
        return ret;
    }

    if (rx_buf[3] != drv8334_calc_crc8(rx_buf, 3)) {
        LOG_ERR("CRC error for addr 0x%02X", addr);
        return -EBADMSG;
    }

    if (!(rx_buf[0] & 0x28)) {
        LOG_WRN("DRV8334 reported fault status: 0x%02X on addr 0x%02X", rx_buf[0], addr);
        return -EIO;
    }

    return 0;
}

int drv8334_reg_read(const struct drv8334_config *cfg, uint8_t addr, uint16_t *data)
{
    uint8_t tx_buf[4] = {
        [0] = ((addr << 1) & 0xFE) | 0x01,
        [1] = 0x00,
        [2] = 0x00,
    };
    uint8_t rx_buf[4];

    tx_buf[3] = drv8334_calc_crc8(tx_buf, 3);

    const struct spi_buf tx = {.buf = tx_buf, .len = sizeof(tx_buf)};
    const struct spi_buf_set tx_bufs = {.buffers = &tx, .count = 1};
    const struct spi_buf rx = {.buf = rx_buf, .len = sizeof(rx_buf)};
    const struct spi_buf_set rx_bufs = {.buffers = &rx, .count = 1};

    int ret = spi_transceive_dt(&cfg->spi, &tx_bufs, &rx_bufs);
    if (ret < 0) {
        return ret;
    }

    if (rx_buf[3] != drv8334_calc_crc8(rx_buf, 3)) {
        return -EBADMSG;
    }

    *data = ((uint16_t)rx_buf[1] << 8) | rx_buf[2];
    return 0;
}

int drv8334_init(const struct drv8334_config *cfg)
{
    uint16_t data;

    if (!spi_is_ready_dt(&cfg->spi)) {
        return -ENODEV;
    }

    gpio_pin_configure_dt(&cfg->nsleep, GPIO_OUTPUT_LOW);
    gpio_pin_configure_dt(&cfg->drvoff, GPIO_OUTPUT_HIGH);
    gpio_pin_configure_dt(&cfg->nfault, GPIO_INPUT);

    gpio_pin_set_dt(&cfg->nsleep, 1);
    k_msleep(2);

    drv8334_reg_read(cfg, DRV8334_REG_IC_CTRL2, &data);
    printk("INFO: DRV8334_REG_IC_CTRL2 is: 0x%04X\n", data);
    drv8334_reg_read(cfg, DRV8334_REG_IC_CTRL2, &data);
    printk("INFO: DRV8334_REG_IC_CTRL2 is: 0x%04X\n", data);

    drv8334_reg_write(cfg, DRV8334_REG_IC_CTRL3, 0xC009);
    drv8334_reg_write(cfg, DRV8334_REG_GD_CTRL1, 0x0328);
    drv8334_reg_write(cfg, DRV8334_REG_GD_CTRL2, 0x0717);
    drv8334_reg_write(cfg, DRV8334_REG_GD_CTRL3, 0x0717);
    drv8334_reg_write(cfg, DRV8334_REG_GD_CTRL3B, 0x2323);
    drv8334_reg_write(cfg, DRV8334_REG_GD_CTRL4, 0x0200);
    drv8334_reg_write(cfg, DRV8334_REG_GD_CTRL5, 0x0007);
    drv8334_reg_write(cfg, DRV8334_REG_GD_CTRL6, 0x2323);
    drv8334_reg_write(cfg, DRV8334_REG_GD_CTRL7, 0x2323);

    drv8334_reg_write(cfg, DRV8334_REG_CSA_CTRL, 0x0888);

    drv8334_reg_write(cfg, DRV8334_REG_MON_CTRL1, 0x0200);
    drv8334_reg_write(cfg, DRV8334_REG_MON_CTRL2, 0x1101);
    drv8334_reg_write(cfg, DRV8334_REG_MON_CTRL3, 0x0042);
    drv8334_reg_write(cfg, DRV8334_REG_MON_CTRL4, 0x0032);
    drv8334_reg_write(cfg, DRV8334_REG_MON_CTRL6, 0x0200);

    gpio_pin_set_dt(&cfg->drvoff, 0);
    k_usleep(10);

    drv8334_reg_write(cfg, DRV8334_REG_IC_CTRL2, 0x8807);
    drv8334_reg_read(cfg, DRV8334_REG_IC_CTRL2, &data);
    printk("INFO: DRV8334_REG_IC_CTRL2 is: 0x%04X\n", data);

    return 0;
}

static void print_bit_fields(const struct shell *sh, uint16_t val, const struct bit_desc *descs, size_t count)
{
    for (size_t i = 0; i < count; i++) {
        uint8_t bit_val = (val >> descs[i].bit) & 0x01;
        shell_print(sh, "    [%2u] %-12s : %u (%s)",
                    descs[i].bit,
                    descs[i].name,
                    bit_val,
                    bit_val ? descs[i].active_str : "OK/No Fault");
    }
}

static void drv8334_decode_status_reg(const struct shell *sh, uint8_t addr, uint16_t val)
{
    static const struct bit_desc stat1[] = {
        {15, "SPI_OK", "No SPI Fault"}, {14, "FAULT", "Fault Event Active"},
        {13, "WARN", "Warning Active"}, {12, "VDS", "VDS Overcurrent"},
        {11, "VGS", "VGS Event"}, {10, "SNS_OCP", "Sense Overcurrent"},
        {9, "OV", "Overvoltage"}, {8, "UV", "Undervoltage"},
        {1, "OTW", "Overtemp Warning"}, {0, "DRV_STAT", "Driver Enabled"}
    };
    static const struct bit_desc stat2[] = {
        {15, "CBC_ST", "CBC Counter > 0"}, {10, "SNS_OCP_A", "Phase A Sense OCP"},
        {9, "SNS_OCP_B", "Phase B Sense OCP"}, {8, "SNS_OCP_C", "Phase C Sense OCP"},
        {5, "VDS_HA", "Phase A HS VDS Fault"}, {4, "VDS_LA", "Phase A LS VDS Fault"},
        {3, "VDS_HB", "Phase B HS VDS Fault"}, {2, "VDS_LB", "Phase B LS VDS Fault"},
        {1, "VDS_HC", "Phase C HS VDS Fault"}, {0, "VDS_LC", "Phase C LS VDS Fault"}
    };
    static const struct bit_desc stat3[] = {
        {5, "VGS_HA", "Phase A HS VGS Fault"}, {4, "VGS_LA", "Phase A LS VGS Fault"},
        {3, "VGS_HB", "Phase B HS VGS Fault"}, {2, "VGS_LB", "Phase B LS VGS Fault"},
        {1, "VGS_HC", "Phase C HS VGS Fault"}, {0, "VGS_LC", "Phase C LS VGS Fault"}
    };
    static const struct bit_desc stat4[] = {
        {15, "PVDD_OV", "PVDD OV"}, {14, "PVDD_UV", "PVDD UV"},
        {13, "VDRAIN_OV", "VDRAIN OV"}, {12, "VDRAIN_UV", "VDRAIN UV"},
        {11, "VCP_OV", "VCP OV"}, {10, "VCP_UV", "VCP UV"},
        {9, "GVDD_OV", "GVDD OV"}, {8, "GVDD_UV", "GVDD UV"},
        {5, "BSTA_OV", "BST A OV"}, {4, "BSTA_UV", "BST A UV"},
        {3, "BSTB_OV", "BST B OV"}, {2, "BSTB_UV", "BST B UV"},
        {1, "BSTC_OV", "BST C OV"}, {0, "BSTC_UV", "BST C UV"}
    };
    static const struct bit_desc stat5[] = {
        {14, "PVDD_UVW", "PVDD UV Warning"}, {10, "GVDD_CP_LDO", "LDO Mode"},
        {9, "OTSD", "Overtemp Shutdown"}, {8, "WDT_FLT", "Watchdog Fault"},
        {7, "SPI_CRC_FLT", "SPI CRC Fault"}, {6, "SPI_ADDR_FLT", "SPI Addr Fault"},
        {5, "SPI_CLK_FLT", "SPI Clock Fault"}, {4, "OTP_CRC_FLT", "Factory OTP CRC"},
        {3, "OTP_USR_CRC", "User OTP CRC"}, {1, "STP_FLT", "Shoot Through"},
        {0, "DEADT_FLT", "Dead Time Fault"}
    };
    static const struct bit_desc stat6[] = {
        {15, "PHCA_FLT", "Phase A Comp Fault"}, {14, "PHCB_FLT", "Phase B Comp Fault"},
        {13, "PHCC_FLT", "Phase C Comp Fault"}, {11, "VREF_OV", "VREF OV"},
        {10, "VREF_UV", "VREF UV"}, {9, "VDDSDO_UV", "VDDSDO UV"},
        {7, "DVDD_OV", "DVDD OV"}, {4, "ABIST_FLT", "Analog BIST Fault"},
        {3, "DEV_MODE_FLT", "Device Mode Fault"}, {0, "CLK_MON_FLT", "Clock Mon Fault"}
    };

    switch (addr) {
    case DRV8334_REG_IC_STAT1: print_bit_fields(sh, val, stat1, ARRAY_SIZE(stat1)); break;
    case DRV8334_REG_IC_STAT2: print_bit_fields(sh, val, stat2, ARRAY_SIZE(stat2)); break;
    case DRV8334_REG_IC_STAT3: print_bit_fields(sh, val, stat3, ARRAY_SIZE(stat3)); break;
    case DRV8334_REG_IC_STAT4: print_bit_fields(sh, val, stat4, ARRAY_SIZE(stat4)); break;
    case DRV8334_REG_IC_STAT5: print_bit_fields(sh, val, stat5, ARRAY_SIZE(stat5)); break;
    case DRV8334_REG_IC_STAT6: print_bit_fields(sh, val, stat6, ARRAY_SIZE(stat6)); break;
    default: break;
    }
}

struct reg_entry {
    uint8_t addr;
    const char *acronym;
    const char *name;
};

static void dump_registers_internal(const struct drv8334_config *cfg, const struct shell *sh,
                                     const struct reg_entry *regs, size_t count, const char *title)
{
    shell_print(sh, "================ %s ================", title);
    for (size_t i = 0; i < count; i++) {
        uint16_t val = 0;
        int ret = drv8334_reg_read(cfg, regs[i].addr, &val);
        if (ret < 0) {
            shell_error(sh, "  [0x%02X] %-10s (%-22s): ERROR (%d)",
                        regs[i].addr, regs[i].acronym, regs[i].name, ret);
        } else {
            shell_print(sh, "  [0x%02X] %-10s (%-22s): 0x%04X",
                        regs[i].addr, regs[i].acronym, regs[i].name, val);
            drv8334_decode_status_reg(sh, regs[i].addr, val);
        }
    }
}

int drv8334_dump_status_regs(const struct drv8334_config *cfg, const struct shell *sh)
{
    static const struct reg_entry status_regs[] = {
        {DRV8334_REG_IC_STAT1, "IC_STAT1", "IC Status Register 1"},
        {DRV8334_REG_IC_STAT2, "IC_STAT2", "IC Status Register 2"},
        {DRV8334_REG_IC_STAT3, "IC_STAT3", "IC Status Register 3"},
        {DRV8334_REG_IC_STAT4, "IC_STAT4", "IC Status Register 4"},
        {DRV8334_REG_IC_STAT5, "IC_STAT5", "IC Status Register 5"},
        {DRV8334_REG_IC_STAT6, "IC_STAT6", "IC Status Register 6"},
    };

    dump_registers_internal(cfg, sh, status_regs, ARRAY_SIZE(status_regs), "STATUS REGISTERS");
    return 0;
}

int drv8334_dump_control_regs(const struct drv8334_config *cfg, const struct shell *sh)
{
    static const struct reg_entry control_regs[] = {
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

    dump_registers_internal(cfg, sh, control_regs, ARRAY_SIZE(control_regs), "CONTROL REGISTERS");
    return 0;
}

static int cmd_gdu_dump_status(const struct shell *sh, size_t argc, char **argv)
{
    return drv8334_dump_status_regs(&drv_cfg, sh);
}

static int cmd_gdu_dump_control(const struct shell *sh, size_t argc, char **argv)
{
    return drv8334_dump_control_regs(&drv_cfg, sh);
}

static int cmd_gdu_read(const struct shell *sh, size_t argc, char **argv)
{
    if (argc < 2) {
        shell_error(sh, "Usage: gdu read <addr_hex>");
        return -EINVAL;
    }

    char *endptr;
    uint8_t addr = (uint8_t)strtoul(argv[1], &endptr, 16);
    if (*endptr != '\0') {
        shell_error(sh, "Invalid hex address: %s", argv[1]);
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

static int cmd_gdu_write(const struct shell *sh, size_t argc, char **argv)
{
    if (argc < 3) {
        shell_error(sh, "Usage: gdu write <addr_hex> <val_hex>");
        return -EINVAL;
    }

    char *endptr1, *endptr2;
    uint8_t addr = (uint8_t)strtoul(argv[1], &endptr1, 16);
    uint16_t val = (uint16_t)strtoul(argv[2], &endptr2, 16);

    if (*endptr1 != '\0' || *endptr2 != '\0') {
        shell_error(sh, "Invalid hex format in parameters");
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

static int cmd_gdu_drv(const struct shell *sh, size_t argc, char **argv)
{
    if (argc < 2) {
        shell_error(sh, "Usage: gdu drv <on|off>");
        return -EINVAL;
    }

    if (strcmp(argv[1], "on") == 0) {
        gpio_pin_set_dt(&drv_cfg.drvoff, 0);
        shell_print(sh, "DRV8334 Gate Driver ENABLED");
    } else if (strcmp(argv[1], "off") == 0) {
        gpio_pin_set_dt(&drv_cfg.drvoff, 1);
        shell_print(sh, "DRV8334 Gate Driver DISABLED");
    } else {
        shell_error(sh, "Invalid state: %s. Options: 'on', 'off'", argv[1]);
        return -EINVAL;
    }

    return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(sub_gdu,
    SHELL_CMD_ARG(dump_status, NULL, "Dump DRV8334 status registers", cmd_gdu_dump_status, 1, 0),
    SHELL_CMD_ARG(dump_control, NULL, "Dump DRV8334 control registers", cmd_gdu_dump_control, 1, 0),
    SHELL_CMD_ARG(read, NULL, "Read DRV8334 register: gdu read <addr_hex>", cmd_gdu_read, 2, 0),
    SHELL_CMD_ARG(write, NULL, "Write DRV8334 register: gdu write <addr_hex> <val_hex>", cmd_gdu_write, 3, 0),
    SHELL_CMD_ARG(drv, NULL, "Control DRV8334 gate driver: gdu drv <on|off>", cmd_gdu_drv, 2, 0),
    SHELL_SUBCMD_SET_END
);

SHELL_CMD_REGISTER(gdu, &sub_gdu, "DRV8334 Driver Interface Commands", NULL);
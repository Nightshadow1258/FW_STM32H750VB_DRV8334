/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: Apache-2.0
 */

#include "storage_manager.h"
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/drivers/flash.h>
#include <zephyr/fs/nvs.h>

#define STORAGE_NODE DT_ALIAS(storage_flash)

static struct nvs_fs fs;
static bool storage_ready = false;

int storage_init(void)
{
    int rc;
    struct flash_pages_info info;

    if (!DT_NODE_EXISTS(STORAGE_NODE)) {
        printk("Error: storage_flash alias does not exist in devicetree\n");
        return -ENODEV;
    }

    fs.flash_device = DEVICE_DT_GET(DT_MTD_FROM_FIXED_PARTITION(STORAGE_NODE));
    if (!device_is_ready(fs.flash_device)) {
        printk("Error: Flash device not ready\n");
        return -ENODEV;
    }

    fs.offset = DT_REG_ADDR(STORAGE_NODE);
    
    rc = flash_get_page_info_by_offs(fs.flash_device, fs.offset, &info);
    if (rc < 0) {
        printk("Error getting page info: %d. Using explicit 4KB sector size.\n", rc);
        fs.sector_size = 4096; 
    } else {
        fs.sector_size = info.size;
    }
    
    fs.sector_count = 8; 

    rc = nvs_mount(&fs);
    if (rc < 0) {
        printk("NVS Mount failed: %d\n", rc);
        return rc;
    }
    
    storage_ready = true;
    printk("NVS mounted successfully. Sector size: %d, Sectors: %d\n", 
           fs.sector_size, fs.sector_count);
    return 0;
}

int storage_save_factor(uint16_t id, float factor)
{
    if (!storage_ready) {
        return -EAGAIN;
    }
    return nvs_write(&fs, id, &factor, sizeof(factor));
}

int storage_load_factor(uint16_t id, float *out_factor)
{
    if (!storage_ready) {
        return -EAGAIN;
    }
    if (out_factor == NULL) {
        return -EINVAL;
    }

    int rc = nvs_read(&fs, id, out_factor, sizeof(float));
    if (rc > 0) {
        return 0; // Success
    } else if (rc == 0) {
        return -ENOENT; // Not found
    }
    
    return rc; // Return negative error code
}

void storage_run_test(void)
{
    printk("\n--- Starting Isolated QSPI NVS Flash Test ---\n");
    
    if (!storage_ready) {
        if (storage_init() != 0) {
            printk("Storage test aborted: Initialization failed.\n");
            return;
        }
    }
    
    uint16_t test_id = 101;
    float test_write_val = 3.14159f;
    float test_read_val = 0.0f;
    
    printk("Writing factor %f to NVS ID %d...\n", (double)test_write_val, test_id);
    int rc = storage_save_factor(test_id, test_write_val);
    
    if (rc < 0) {
        printk("Save failed with error: %d\n", rc);
        return;
    } else if (rc == 0) {
        printk("Save returned 0 bytes written. NVS skipped write (data matches existing entry or flash is unwritten).\n");
    } else {
        printk("Save successful. Wrote %d bytes to flash.\n", rc);
    }
    
    printk("Reading back ID %d from flash...\n", test_id);
    rc = storage_load_factor(test_id, &test_read_val);
    if (rc == 0) {
        printk("Success! Retrieved Factor ID %d: %f\n", test_id, (double)test_read_val);
    } else {
        printk("Read failed with error: %d\n", rc);
    }
    
    printk("--- QSPI NVS Flash Test Complete ---\n");
}
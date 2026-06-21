/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef STORAGE_MANAGER_H_
#define STORAGE_MANAGER_H_

#include <zephyr/types.h>

/**
 * @brief Initializes and mounts the NVS filesystem on the QSPI flash.
 * * @return 0 on success, negative errno code on failure.
 */
int storage_init(void);

/**
 * @brief Saves a floating-point factor to NVS.
 * * @param id Unique identifier for the data record.
 * @param factor The floating-point value to store.
 * @return int 0 or positive bytes written on success, negative errno on failure.
 */
int storage_save_factor(uint16_t id, float factor);

/**
 * @brief Loads a floating-point factor from NVS.
 * * @param id Unique identifier for the data record.
 * @param out_factor Pointer to store the retrieved floating-point value.
 * @return int 0 on success, -ENOENT if ID not found, or negative errno on failure.
 */
int storage_load_factor(uint16_t id, float *out_factor);

/**
 * @brief Executes a self-test of the NVS storage system.
 */
void storage_run_test(void);

#endif /* STORAGE_MANAGER_H_ */
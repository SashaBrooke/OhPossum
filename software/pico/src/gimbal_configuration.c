#include <stdio.h>
#include <string.h>

#include "pico/stdlib.h"
#include "hardware/flash.h" // for the flash erasing and writing
#include "hardware/sync.h" // for the interrupts

#include "gimbal_configuration.h"

#define FLASH_TARGET_OFFSET (256 * 1024) // choosing to start at 256K

static gimbal_configuration_t gc;

void loadGimbalConfiguration(gimbal_configuration_t *config) {
    if (config == NULL) {
        return;
    }

    // Calculate the address in flash memory
    const uint8_t* flash_target_contents = (const uint8_t*)(XIP_BASE + FLASH_TARGET_OFFSET);

    // Copy data from flash into the provided structure
    memcpy(config, flash_target_contents, sizeof(gimbal_configuration_t));

    printf("Gimbal configuration loaded successfully.\n");
}

void saveGimbalConfiguration(gimbal_configuration_t *config) {
    memcpy(&gc, config, sizeof(gimbal_configuration_t));
    uint8_t* configAsBytes = (uint8_t*) &gc;
    int configSize = sizeof(*config);
    
    int writeSize = (configSize / FLASH_PAGE_SIZE) + 1; // how many flash pages we're gonna need to write
    int sectorCount = ((writeSize * FLASH_PAGE_SIZE) / FLASH_SECTOR_SIZE) + 1; // how many flash sectors we're gonna need to erase
        
    printf("Programming flash target region...\n");

    uint32_t interrupts = save_and_disable_interrupts();
    flash_range_erase(FLASH_TARGET_OFFSET, FLASH_SECTOR_SIZE * sectorCount);
    flash_range_program(FLASH_TARGET_OFFSET, configAsBytes, FLASH_PAGE_SIZE * writeSize);
    restore_interrupts(interrupts);

    printf("Done.\n");
}

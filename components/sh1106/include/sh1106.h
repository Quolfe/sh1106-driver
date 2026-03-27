#ifndef SH1106_H
#define SH1106_H

#include <stdint.h>
#include <stdbool.h>
#include "driver/i2c_types.h"
#include "esp_err.h"
#include "freertos/idf_additions.h"

typedef struct {
    uint8_t device_address;
    uint32_t scl_speed_hz;
} sh1106_config_t;

typedef struct sh1106_t sh1106_t;

typedef struct {
    uint8_t x_size;
    uint8_t y_size;
    uint8_t x;
    uint8_t y;
    uint8_t *data;
} bitmap_t;

sh1106_config_t sh1106_default_config(void);
esp_err_t sh1106_init(sh1106_config_t conf, i2c_master_bus_handle_t i2c_handle, sh1106_t *display);

void sh1106_clear_frame(sh1106_t *display);
void sh1106_clear_frame_changes(sh1106_t *display);

void sh1106_update_full_display(sh1106_t *display, SemaphoreHandle_t i2c_handle);
void sh1106_update_part_display(sh1106_t *display, SemaphoreHandle_t i2c_handle);
void sh1106_update_display(sh1106_t *display, SemaphoreHandle_t i2c_handle);

void sh1106_draw_pixel(sh1106_t *display, uint8_t x, uint8_t y, bool on);
void sh1106_draw_bitmap(sh1106_t *display, bitmap_t bitmap);

#endif

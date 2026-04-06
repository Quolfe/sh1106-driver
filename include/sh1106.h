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
    uint8_t pump_voltage;
    uint8_t start_line;
    uint8_t contrast;
    bool segment_remap;
    bool all_pixels_on;
    bool flip_pixels;
    uint8_t multiplex_ratio;
    bool dc_converter;
    bool display_on;
    bool vertical_flip;
    uint8_t offset;
    uint8_t clock_ratio;
    uint8_t clock_frequency;
    uint8_t precharge_period;
    uint8_t discharge_period;
    bool pad_config;
    uint8_t vcom_deselect_voltage;
} sh1106_config_t;

typedef struct sh1106_t sh1106_t;

typedef struct {
    uint8_t *data;
    uint8_t width;
    uint8_t height;
} bitmap_t;

sh1106_t *sh1106_new(void);
void sh1106_free(sh1106_t *display);

void sh1106_take_mutex(sh1106_t *display, TickType_t max_delay);
void sh1106_give_mutex(sh1106_t *display);

sh1106_config_t sh1106_default_config(void);
esp_err_t sh1106_init(sh1106_config_t conf, i2c_master_bus_handle_t i2c_handle, sh1106_t *display);

void sh1106_set_pump_voltage(sh1106_t *display, uint8_t val);
void sh1106_set_start_line(sh1106_t *display, uint8_t val);
void sh1106_set_contrast(sh1106_t *display, uint8_t contrast);
void sh1106_set_segment_remap(sh1106_t *display, bool reverse);
void sh1106_set_all_pixels_on(sh1106_t *display, bool on);
void sh1106_set_flip_pixels(sh1106_t *display, bool reverse);
void sh1106_set_multiplex_ratio(sh1106_t *display, uint8_t val);
void sh1106_set_dc_converter(sh1106_t *display, bool on);
void sh1106_set_display_on(sh1106_t *display, bool on);
void sh1106_set_vertical_flip(sh1106_t *display, bool flip);
void sh1106_set_display_offset(sh1106_t *display, uint8_t offset);
void sh1106_set_clock_ratio(sh1106_t *display, uint8_t ratio);
void sh1106_set_clock_frequency(sh1106_t *display, uint8_t frequency);
void sh1106_set_discharge_period(sh1106_t *display, uint8_t discharge);
void sh1106_set_precharge_period(sh1106_t *display, uint8_t precharge);
void sh1106_set_pads(sh1106_t *display, bool alternative);
void sh1106_set_vcom_deselect(sh1106_t *display, uint8_t val);

void sh1106_update_config(sh1106_t *display, SemaphoreHandle_t i2c_mutex);

void sh1106_clear_frame(sh1106_t *display);
void sh1106_clear_frame_changes(sh1106_t *display);

void sh1106_update_full_display(sh1106_t *display, SemaphoreHandle_t i2c_mutex);
void sh1106_update_part_display(sh1106_t *display, SemaphoreHandle_t i2c_mutex);
void sh1106_update_display(sh1106_t *display, SemaphoreHandle_t i2c_mutex);

void sh1106_draw_pixel(sh1106_t *display, uint8_t x, uint8_t y, bool on);

void sh1106_create_bitmap(bool *bool_data, uint8_t width, uint8_t height, bitmap_t *dest);
void sh1106_free_bitmap(bitmap_t bitmap);
void sh1106_draw_bitmap(sh1106_t *display, bitmap_t bitmap);

#endif

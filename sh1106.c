#include "include/sh1106.h"
#include "freertos/idf_additions.h"
#include "portmacro.h"
#include <freertos/FreeRTOS.h>
#include <driver/i2c_master.h>
#include <stdint.h>
#include <string.h>

struct sh1106_t {
    i2c_master_dev_handle_t handle;
    uint8_t frame_buf[128 * 64 / 8];
    uint8_t frame_change[128];
    uint8_t frame_change_amt;
    SemaphoreHandle_t frame_mutex;
    uint8_t page_change;
    bool force_update;
};

static inline uint8_t sh1106_set_page(uint8_t page)                           { return 0xB0 | page; }
static inline uint8_t sh1106_set_lower_column(uint8_t column)                 { return 0x00 | (column & 0x0F); }
static inline uint8_t sh1106_set_upper_column(uint8_t column)                 { return 0x10 | (column >> 4); }
static inline uint8_t sh1106_set_pump_voltage(uint8_t val)                    { return 0x30 | val; }
static inline uint8_t sh1106_set_start_line(uint8_t val)                      { return 0x40 | val; }
static inline uint8_t sh1106_set_contrast(uint8_t contrast)                   { return contrast; }
static inline uint8_t sh1106_set_segment_remap(bool reverse)                  { return reverse ? 0xA1 : 0xA0; }
static inline uint8_t sh1106_set_all_pixels_on(bool on)                       { return on ? 0xA5 : 0xA4; }
static inline uint8_t sh1106_set_flip_pixels(bool reverse)                    { return reverse ? 0xA7 : 0xA6; }
static inline uint8_t sh1106_set_multiplex_ratio(uint8_t val)                 { return 0x3F & (val - 1); }
static inline uint8_t sh1106_set_dc_converter(bool on)                        { return on ? 0x8B : 0x8A; }
static inline uint8_t sh1106_set_display_on(bool on)                          { return on ? 0xAF : 0xAE; }
static inline uint8_t sh1106_set_vertical_flip(bool flip)                     { return flip ? 0xC0 : 0xC8; }
static inline uint8_t sh1106_set_display_offset(uint8_t offset)               { return 0x3F & offset; }
static inline uint8_t sh1106_set_clock(uint8_t ratio, uint8_t frequency)      { return (frequency << 4) | (0x0F & (ratio - 1)); }
static inline uint8_t sh1106_set_charge_periods(uint8_t discharge, uint8_t precharge) { return (discharge << 4) | (0x0F & precharge); }
static inline uint8_t sh1106_set_pads(bool alternative)                       { return alternative ? 0x12 : 0x02; }
static inline uint8_t sh1106_set_vcom_deselect(uint8_t val)                   { return val; }

static inline bool bit_check(uint8_t val, uint8_t pos) { return (val & (1 << pos)) > 0x00; }

sh1106_t *sh1106_new(void) {
    sh1106_t *ptr = malloc(sizeof(sh1106_t));
    return ptr;
}

void sh1106_free(sh1106_t *display) {
    free(display);
}

void sh1106_take_mutex(sh1106_t *display, TickType_t max_delay) {
    xSemaphoreTake(display->frame_mutex, max_delay);
}

void sh1106_give_mutex(sh1106_t *display) {
    xSemaphoreGive(display->frame_mutex);
}

sh1106_config_t sh1106_default_config(void) {
    sh1106_config_t config = {
        .device_address = 0x3C,
        .scl_speed_hz = 400000,
        .pump_voltage = 2,
        .start_line = 0,
        .contrast = 128, // 127?
        .segment_reverse = false, // true?
        .all_pixels_on = false,
        .flip_pixels = false,
        .muliplex_ratio = 64,
        .dc_converter = true,
        .vertical_flip = false,
        .offset = 0,
        .clock_ratio = 0,
        .clock_frequency = 8,
        .precharge_period = 2,
        .discharge_period = 2,
        .alt_pad_config = true,
        .vcom_deselect_voltage = 0x20,
    };
    return config;
}

esp_err_t sh1106_init(sh1106_config_t conf, i2c_master_bus_handle_t i2c_handle, sh1106_t *display) {
    esp_err_t err;
    err = i2c_master_probe(i2c_handle, 0x3C, 500);
    if (err != ESP_OK) {
        return err;
    };

    i2c_device_config_t sh1106_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = conf.device_address,
        .scl_speed_hz = conf.scl_speed_hz,
    };


    i2c_master_dev_handle_t sh1106_handle;

    err = i2c_master_bus_add_device(i2c_handle, &sh1106_config, &sh1106_handle);
    if (err != ESP_OK) {
        return err;
    }

    display->handle = sh1106_handle;
    display->frame_change_amt = 0;
    display->frame_mutex = xSemaphoreCreateMutex();
    display->page_change = 0x00;
    display->force_update = false;

    // sh1106 initialization
    uint8_t init_cmd_buf[] = {
        0x00, // Command byte
        sh1106_set_display_on(false), // 11
        sh1106_set_pump_voltage(conf.pump_voltage), // 3
        sh1106_set_start_line(conf.start_line), // 4
        0x81, // 5 mode
        sh1106_set_contrast(conf.contrast), // 5 val
        sh1106_set_segment_remap(conf.segment_reverse), // 6
        sh1106_set_all_pixels_on(conf.all_pixels_on), // 7
        sh1106_set_flip_pixels(conf.flip_pixels), // 8
        0xA8, // 9 mode
        sh1106_set_multiplex_ratio(conf.muliplex_ratio), // 9 val
        0xAD, // 10 mode
        sh1106_set_dc_converter(conf.dc_converter), // 10 val
        sh1106_set_vertical_flip(conf.vertical_flip), // 13
        0xD3, // 14 mode
        sh1106_set_display_offset(conf.offset), // 14 val
        0xD5, // 15 mode
        sh1106_set_clock(conf.clock_ratio, conf.clock_frequency), // 15 val
        0xD9, // 16 mode
        sh1106_set_charge_periods(conf.discharge_period, conf.precharge_period), // 16 val
        0xDA, // 17 mode
        sh1106_set_pads(conf.alt_pad_config), // 17 val
        0xDB, // 18 mode
        sh1106_set_vcom_deselect(conf.vcom_deselect_voltage), // 18 val
        sh1106_set_display_on(conf.display_on), // 11
    };

    i2c_master_transmit(display->handle, init_cmd_buf, sizeof(init_cmd_buf), 1000);

    // Clear display
    for (uint8_t page = 0; page < 8; page++) {
        uint8_t pos_cmd_buf[] = {
            0x00,
            sh1106_set_page(page),
            sh1106_set_upper_column(0),
            sh1106_set_lower_column(0),
        };

        i2c_master_transmit(display->handle, pos_cmd_buf, sizeof(pos_cmd_buf) / sizeof(*pos_cmd_buf), 500);

        uint8_t clear_buf[133];
        clear_buf[0] = 0x40;
        memset(clear_buf + 1, 0x00, 132);

        i2c_master_transmit(display->handle, clear_buf, 133, 500);
    }

    sh1106_clear_frame(display);

    return ESP_OK;
}

void sh1106_clear_frame(sh1106_t *display) {
    memset(display->frame_buf, 0x00, sizeof(display->frame_buf));
    memset(display->frame_change, 0x00, sizeof(display->frame_change));
    display->frame_change_amt = 0;
    display->page_change = 0x00;
    display->force_update = true;
}

void sh1106_clear_frame_changes(sh1106_t *display) {
    memset(display->frame_change, 0x00, sizeof(display->frame_change));
    display->frame_change_amt = 0;
    display->page_change = 0x00;
}

void sh1106_update_full_display(sh1106_t *display, SemaphoreHandle_t i2c_mutex) {
    xSemaphoreTake(i2c_mutex, portMAX_DELAY);
    for (uint8_t page = 0; page < 8; page++) {
        uint8_t pos_cmd_buf[] = {
            0x00,
            sh1106_set_page(page),
            sh1106_set_upper_column(2),
            sh1106_set_lower_column(2),
        };
        i2c_master_transmit(display->handle, pos_cmd_buf, sizeof(pos_cmd_buf) / sizeof(*pos_cmd_buf), 500);

        uint8_t data_buf[129];
        data_buf[0] = 0x40;
        memcpy(data_buf + sizeof(*data_buf), display->frame_buf + page * 128, 128);
        i2c_master_transmit(display->handle, data_buf, 129, 500);
    }
    xSemaphoreGive(i2c_mutex);
}

void sh1106_update_part_display(sh1106_t *display, SemaphoreHandle_t i2c_mutex) {
    xSemaphoreTake(i2c_mutex, portMAX_DELAY);
    for (uint8_t page = 0; page < 8; page++) {
        if (!((display->page_change & (0x01 << page)) > 0x00))
            continue;
        bool previous = false;
        uint8_t data_buf[129];
        data_buf[0] = 0x40;
        uint8_t data_amt = 0;
        for (uint8_t col = 0; col < 128; col++) {
            bool update = bit_check(display->frame_change[page * 16 + col / 8], col % 8);
            if (!update && previous) {
                uint8_t pos_cmd_buf[] = {
                    0x00,
                    sh1106_set_page(page),
                    sh1106_set_upper_column(col - data_amt + 2),
                    sh1106_set_lower_column(col - data_amt + 2),
                };
                i2c_master_transmit(display->handle, pos_cmd_buf, sizeof(pos_cmd_buf), 500);
                i2c_master_transmit(display->handle, data_buf, data_amt + 1, 500);
                data_amt = 0;
            }
            if (update) {
                data_buf[data_amt++ + 1] = display->frame_buf[page * 128 + col];
            }
            previous = update;
        }
        if (previous) {
            uint8_t pos_cmd_buf[] = {
                0x00,
                sh1106_set_page(page),
                sh1106_set_upper_column(128 - data_amt + 2),
                sh1106_set_lower_column(128 - data_amt + 2),
            };
            i2c_master_transmit(display->handle, pos_cmd_buf, sizeof(pos_cmd_buf), 500);
            i2c_master_transmit(display->handle, data_buf, data_amt + 1, 500);
        }
    }
    xSemaphoreGive(i2c_mutex);
}

void sh1106_update_display(sh1106_t *display, SemaphoreHandle_t i2c_mutex) {
    xSemaphoreTake(display->frame_mutex, portMAX_DELAY);
    if (display->force_update || display->frame_change_amt > 32) {
        sh1106_update_full_display(display, i2c_mutex);
        display->force_update = false;
    } else {
        sh1106_update_part_display(display, i2c_mutex);
    }
    sh1106_clear_frame_changes(display);
    xSemaphoreGive(display->frame_mutex);
}

void sh1106_draw_pixel(sh1106_t *display, uint8_t x, uint8_t y, bool on) {
    if (x >= 128 || y >= 64) {
        return;
    }
    xSemaphoreTake(display->frame_mutex, portMAX_DELAY);
    bool change_needed;
    if (on) {
        if (bit_check(display->frame_buf[y / 8 * 128 + x], y % 8)) {
            change_needed = false;
        } else {
            change_needed = true;
            display->frame_buf[y / 8 * 128 + x] |= 0x01 << (y % 8);
        }
    } else {
        if (!bit_check(display->frame_buf[y / 8 * 128 + x], y % 8)) {
            change_needed = false;
        } else {
            change_needed = true;
            display->frame_buf[y / 8 * 128 + x] &= (0x01 << (y % 8)) ^ 0xFF;
        }
    }
    if (!change_needed) {
        xSemaphoreGive(display->frame_mutex);
        return;
    }
    bool change_set = bit_check(display->frame_change[y / 8 * 16 + x / 8], x % 8);
    if (!change_set) {
        display->frame_change[y / 8 * 16 + x / 8] |= 0x01 << (x % 8);
        display->page_change |= 0x01 << (y / 8);
        display->frame_change_amt++;
    }
    xSemaphoreGive(display->frame_mutex);
} 

void sh1106_draw_bitmap(sh1106_t *display, bitmap_t bitmap) {
    for (uint8_t y = 0; y < bitmap.y_size; y++) {
        for (uint8_t x = 0; x < bitmap.x_size; x++) {
            sh1106_draw_pixel(display, x + bitmap.x, y + bitmap.y, bitmap.data[y * bitmap.x_size + x]);
        }
    }
}

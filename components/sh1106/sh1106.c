#include "include/sh1106.h"
#include "freertos/idf_additions.h"
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
    bool frame_cleared;
};

static inline uint8_t sh1106_set_page(uint8_t page) {
    return 0xB0 | page;
}

static inline uint8_t sh1106_set_lower_column(uint8_t column) {
    return 0x00 | (column & 0x0F);
}

static inline uint8_t sh1106_set_upper_column(uint8_t column) {
    return 0x10 | (column >> 4);
}

static inline bool bit_check(uint8_t val, uint8_t pos) {
    return (val & (1 << pos)) > 0x00;
}

sh1106_config_t sh1106_default_config(void) {
    sh1106_config_t config = {
        .device_address = 0x3C,
        .scl_speed_hz = 400000,
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
    display->frame_cleared = false;

    // sh1106 initialization
    uint8_t init_cmd_buf[] = {
        0x00,
        0xAE,
        0xD5,
        0x80,
        0xA8,
        0x3F,
        0xD3,
        0x00,
        0x40,
        0xAD,
        0x8B,
        0xA1,
        0xC8,
        0xDA,
        0x12,
        0x81,
        0x7F,
        0xD9,
        0x22,
        0xDB,
        0x20,
        0xA4,
        0xA6,
        0xAF,
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
        memset(clear_buf + sizeof(*clear_buf), 0x00, 132);

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
    display->frame_cleared = true;
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
    if (display->frame_cleared || display->frame_change_amt > 32) {
        sh1106_update_full_display(display, i2c_mutex);
        display->frame_cleared = false;
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
        return;
    }
    bool change_set = bit_check(display->frame_change[y / 8 * 16 + x / 8], x % 8);
    if (!change_set) {
        display->frame_change[y / 8 * 16 + x / 8] |= 0x01 << (x % 8);
        display->page_change |= 0x01 << (y / 8);
        display->frame_change_amt++;
    }
} 

void sh1106_draw_bitmap(sh1106_t *display, bitmap_t bitmap) {
    for (uint8_t y = 0; y < bitmap.y_size; y++) {
        for (uint8_t x = 0; x < bitmap.x_size; x++) {
            sh1106_draw_pixel(display, x + bitmap.x, y + bitmap.y, bitmap.data[y * bitmap.x_size + x]);
        }
    }
}

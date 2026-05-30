#include "include/sh1106.h"
#include "freertos/idf_additions.h"
#include "portmacro.h"
#include <freertos/FreeRTOS.h>
#include <driver/i2c_master.h>
#include <stdint.h>
#include <string.h>

#define SH1106_COMMAND_MODE 0x00
#define SH1106_DATA_MODE 0x40

#define SH1106_SET_CONTRAST_MODE 0x81
#define SH1106_SET_MULTIPLEX_RATIO_MODE 0xA8
#define SH1106_SET_DC_CONVERTER_MODE 0xAD
#define SH1106_SET_OFFSET_MODE 0xD3
#define SH1106_SET_CLOCK_MODE 0xD5
#define SH1106_SET_PERIODS_MODE 0xD9
#define SH1106_SET_PAD_CONFIG_MODE 0xDA
#define SH1106_SET_VCOM_VOLTAGE_MODE 0xDB

typedef struct {
    bool pump_voltage;
    bool start_line;
    bool contrast;
    bool segment_remap;
    bool all_pixels_on;
    bool flip_pixels;
    bool multiplex_ratio;
    bool dc_converter;
    bool display_on;
    bool vertical_flip;
    bool offset;
    bool clock;
    bool periods;
    bool pad_config;
    bool vcom_deselect_voltage;
} sh1106_config_updates_t;

struct sh1106_t {
    i2c_master_dev_handle_t handle;
    uint8_t frame_buf[128 * 64 / 8];
    uint8_t frame_change[128];
    uint8_t frame_change_amt;
    SemaphoreHandle_t frame_mutex;
    uint8_t page_change;
    bool force_update;
    sh1106_config_t config;
    sh1106_config_updates_t config_updates;
};

static inline uint8_t sh1106_set_page_byte(uint8_t page)                                   { return 0xB0 | page; }
static inline uint8_t sh1106_set_lower_column_byte(uint8_t column)                         { return 0x00 | (column & 0x0F); }
static inline uint8_t sh1106_set_upper_column_byte(uint8_t column)                         { return 0x10 | (column >> 4); }
static inline uint8_t sh1106_set_pump_voltage_byte(uint8_t val)                            { return 0x30 | val; }
static inline uint8_t sh1106_set_start_line_byte(uint8_t val)                              { return 0x40 | val; }
static inline uint8_t sh1106_set_contrast_byte(uint8_t contrast)                           { return contrast; }
static inline uint8_t sh1106_set_segment_remap_byte(bool reverse)                          { return reverse ? 0xA1 : 0xA0; }
static inline uint8_t sh1106_set_all_pixels_on_byte(bool on)                               { return on ? 0xA5 : 0xA4; }
static inline uint8_t sh1106_set_flip_pixels_byte(bool reverse)                            { return reverse ? 0xA7 : 0xA6; }
static inline uint8_t sh1106_set_multiplex_ratio_byte(uint8_t val)                         { return 0x3F & (val - 1); }
static inline uint8_t sh1106_set_dc_converter_byte(bool on)                                { return on ? 0x8B : 0x8A; }
static inline uint8_t sh1106_set_display_on_byte(bool on)                                  { return on ? 0xAF : 0xAE; }
static inline uint8_t sh1106_set_vertical_flip_byte(bool flip)                             { return flip ? 0xC0 : 0xC8; }
static inline uint8_t sh1106_set_offset_byte(uint8_t offset)                               { return 0x3F & offset; }
static inline uint8_t sh1106_set_clock_byte(uint8_t ratio, uint8_t frequency)              { return (frequency << 4) | (0x0F & (ratio - 1)); }
static inline uint8_t sh1106_set_charge_periods_byte(uint8_t discharge, uint8_t precharge) { return (discharge << 4) | (0x0F & precharge); }
static inline uint8_t sh1106_set_pad_config_byte(bool alternative)                         { return alternative ? 0x12 : 0x02; }
static inline uint8_t sh1106_set_vcom_deselect_voltage_byte(uint8_t val)                   { return val; }

void sh1106_set_pump_voltage(sh1106_t *display, uint8_t val)           { display->config.pump_voltage = val; display->config_updates.pump_voltage = true; }
void sh1106_set_start_line(sh1106_t *display, uint8_t val)             { display->config.start_line = val; display->config_updates.start_line = true; }
void sh1106_set_contrast(sh1106_t *display, uint8_t contrast)          { display->config.contrast = contrast; display->config_updates.contrast = true; }
void sh1106_set_segment_remap(sh1106_t *display, bool reverse)         { display->config.segment_remap = reverse; display->config_updates.segment_remap = true; }
void sh1106_set_all_pixels_on(sh1106_t *display, bool on)              { display->config.all_pixels_on = on; display->config_updates.all_pixels_on = true; }
void sh1106_set_flip_pixels(sh1106_t *display, bool reverse)           { display->config.flip_pixels = reverse; display->config_updates.flip_pixels = true; }
void sh1106_set_multiplex_ratio(sh1106_t *display, uint8_t val)        { display->config.multiplex_ratio = val; display->config_updates.multiplex_ratio = true; }
void sh1106_set_dc_converter(sh1106_t *display, bool on)               { display->config.dc_converter = on; display->config_updates.dc_converter = true; }
void sh1106_set_display_on(sh1106_t *display, bool on)                 { display->config.display_on = on; display->config_updates.display_on = true; }
void sh1106_set_vertical_flip(sh1106_t *display, bool flip)            { display->config.vertical_flip = flip; display->config_updates.vertical_flip = true; }
void sh1106_set_offset(sh1106_t *display, uint8_t offset)              { display->config.offset = offset; display->config_updates.offset = true; }
void sh1106_set_clock_ratio(sh1106_t *display, uint8_t ratio)          { display->config.clock_ratio = ratio; display->config_updates.clock = true; }
void sh1106_set_clock_frequency(sh1106_t *display, uint8_t frequency)  { display->config.clock_frequency = frequency; display->config_updates.clock = true; }
void sh1106_set_discharge_period(sh1106_t *display, uint8_t discharge) { display->config.discharge_period = discharge; display->config_updates.periods = true; }
void sh1106_set_precharge_period(sh1106_t *display, uint8_t precharge) { display->config.precharge_period = precharge; display->config_updates.periods = true; }
void sh1106_set_pad_config(sh1106_t *display, bool alternative)        { display->config.pad_config = alternative; display->config_updates.pad_config = true; }
void sh1106_set_vcom_deselect_voltage(sh1106_t *display, uint8_t val)  { display->config.vcom_deselect_voltage = val; display->config_updates.vcom_deselect_voltage = true; }

void sh1106_update_config(sh1106_t *display, SemaphoreHandle_t i2c_mutex) {
    uint8_t cmd_buf[32];
    cmd_buf[0] = SH1106_COMMAND_MODE;
    int cmd_amt = 1;

    // single byte commands
    if (display->config_updates.pump_voltage) {
        cmd_buf[cmd_amt++] = sh1106_set_pump_voltage_byte(display->config.pump_voltage);
        display->config_updates.pump_voltage = false;
    }
    if (display->config_updates.start_line) {
        cmd_buf[cmd_amt++] = sh1106_set_start_line_byte(display->config.start_line);
        display->config_updates.start_line = false;
    }
    if (display->config_updates.segment_remap) {
        cmd_buf[cmd_amt++] = sh1106_set_segment_remap_byte(display->config.segment_remap);
        display->config_updates.segment_remap = false;
    }
    if (display->config_updates.all_pixels_on) {
        cmd_buf[cmd_amt++] = sh1106_set_all_pixels_on_byte(display->config.all_pixels_on);
        display->config_updates.all_pixels_on = false;
    }
    if (display->config_updates.flip_pixels) {
        cmd_buf[cmd_amt++] = sh1106_set_flip_pixels_byte(display->config.flip_pixels);
        display->config_updates.flip_pixels = false;
    }
    if (display->config_updates.display_on) {
        cmd_buf[cmd_amt++] = sh1106_set_display_on_byte(display->config.display_on);
        display->config_updates.display_on = false;
    }
    if (display->config_updates.vertical_flip) {
        cmd_buf[cmd_amt++] = sh1106_set_vertical_flip_byte(display->config.vertical_flip);
        display->config_updates.vertical_flip = false;
    }

    // double byte commands
    if (display->config_updates.contrast) {
        cmd_buf[cmd_amt++] = SH1106_SET_CONTRAST_MODE;
        cmd_buf[cmd_amt++] = sh1106_set_contrast_byte(display->config.contrast);
        display->config_updates.contrast = false;
    }
    if (display->config_updates.multiplex_ratio) {
        cmd_buf[cmd_amt++] = SH1106_SET_MULTIPLEX_RATIO_MODE;
        cmd_buf[cmd_amt++] = sh1106_set_multiplex_ratio_byte(display->config.multiplex_ratio);
        display->config_updates.multiplex_ratio = false;
    }
    if (display->config_updates.dc_converter) {
        cmd_buf[cmd_amt++] = SH1106_SET_DC_CONVERTER_MODE;
        cmd_buf[cmd_amt++] = sh1106_set_dc_converter_byte(display->config.dc_converter);
        display->config_updates.dc_converter = false;
    }
    if (display->config_updates.offset) {
        cmd_buf[cmd_amt++] = SH1106_SET_OFFSET_MODE;
        cmd_buf[cmd_amt++] = sh1106_set_offset_byte(display->config.offset);
        display->config_updates.offset = false;
    }
    if (display->config_updates.clock) {
        cmd_buf[cmd_amt++] = SH1106_SET_CLOCK_MODE;
        cmd_buf[cmd_amt++] = sh1106_set_clock_byte(display->config.clock_ratio, display->config.clock_frequency);
        display->config_updates.clock = false;
    }
    if (display->config_updates.periods) {
        cmd_buf[cmd_amt++] = SH1106_SET_PERIODS_MODE;
        cmd_buf[cmd_amt++] = sh1106_set_charge_periods_byte(display->config.discharge_period, display->config.precharge_period);
        display->config_updates.periods  = false;
    }
    if (display->config_updates.pad_config) {
        cmd_buf[cmd_amt++] = SH1106_SET_PAD_CONFIG_MODE;
        cmd_buf[cmd_amt++] = sh1106_set_pad_config_byte(display->config.pad_config);
        display->config_updates.pad_config = false;
    }
    if (display->config_updates.vcom_deselect_voltage) {
        cmd_buf[cmd_amt++] = SH1106_SET_VCOM_VOLTAGE_MODE;
        cmd_buf[cmd_amt++] = sh1106_set_vcom_deselect_voltage_byte(display->config.vcom_deselect_voltage);
        display->config_updates.vcom_deselect_voltage = false;
    }

    if (cmd_amt == 1)
        return;

    xSemaphoreTake(i2c_mutex, portMAX_DELAY);
    i2c_master_transmit(display->handle, cmd_buf, cmd_amt, 500);
    xSemaphoreGive(i2c_mutex);
}

static inline bool bit_check(uint8_t val, uint8_t pos) { return (val & (1 << pos)) > 0x00; }
static inline uint8_t bit_set(uint8_t val, uint8_t pos, bool on) { return val ^ (((0x01 << pos) & val) ^ (on << pos)); }

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
        .contrast = 128,
        .segment_remap = true,
        .all_pixels_on = false,
        .flip_pixels = false,
        .multiplex_ratio = 64,
        .dc_converter = true,
        .display_on = true,
        .vertical_flip = false,
        .offset = 0,
        .clock_ratio = 1,
        .clock_frequency = 8,
        .precharge_period = 2,
        .discharge_period = 2,
        .pad_config = true,
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
    display->config = conf;

    // sh1106 initialization
    uint8_t init_cmd_buf[] = {
        SH1106_COMMAND_MODE, // Command byte
        sh1106_set_display_on_byte(false), // 11
        sh1106_set_pump_voltage_byte(conf.pump_voltage), // 3
        sh1106_set_start_line_byte(conf.start_line), // 4
        SH1106_SET_CONTRAST_MODE, // 5 mode
        sh1106_set_contrast_byte(conf.contrast), // 5 val
        sh1106_set_segment_remap_byte(conf.segment_remap), // 6
        sh1106_set_all_pixels_on_byte(conf.all_pixels_on), // 7
        sh1106_set_flip_pixels_byte(conf.flip_pixels), // 8
        SH1106_SET_MULTIPLEX_RATIO_MODE, // 9 mode
        sh1106_set_multiplex_ratio_byte(conf.multiplex_ratio), // 9 val
        SH1106_SET_DC_CONVERTER_MODE, // 10 mode
        sh1106_set_dc_converter_byte(conf.dc_converter), // 10 val
        sh1106_set_vertical_flip_byte(conf.vertical_flip), // 13
        SH1106_SET_OFFSET_MODE, // 14 mode
        sh1106_set_offset_byte(conf.offset), // 14 val
        SH1106_SET_CLOCK_MODE, // 15 mode
        sh1106_set_clock_byte(conf.clock_ratio, conf.clock_frequency), // 15 val
        SH1106_SET_PERIODS_MODE, // 16 mode
        sh1106_set_charge_periods_byte(conf.discharge_period, conf.precharge_period), // 16 val
        SH1106_SET_PAD_CONFIG_MODE, // 17 mode
        sh1106_set_pad_config_byte(conf.pad_config), // 17 val
        SH1106_SET_VCOM_VOLTAGE_MODE, // 18 mode
        sh1106_set_vcom_deselect_voltage_byte(conf.vcom_deselect_voltage), // 18 val
        sh1106_set_display_on_byte(conf.display_on), // 11
    };

    // Basic initialization command for debugging:
    // uint8_t init_cmd_buf[] = {
        // 0x00, // command byte
        // 0xAE, // 11 (Display off)
        // 0xD5, // 15 mode (Clock)
        // 0x80, // 15 val (oscillator frequency = 8 (15%))(divide ratio = 0 (1))
        // 0xA8, // 9 mode (Multiplex ratio)
        // 0x3F, // 9 val (64)
        // 0xD3, // 14 mode (Display offset)
        // 0x00, // 14 val (0)
        // 0x40, // 4 (Start line)
        // 0xAD, // 10 mode (DC-DC)
        // 0x8B, // 10 val (on)
        // 0xA1, // 6 (Segment remap)
        // 0xC8, // 13 (Common scan direction)
        // 0xDA, // 17 mode (Pads config)
        // 0x12, // 17 val (Alternative)
        // 0x81, // 5 mode (Contrast)
        // 0x7F, // 5 val (127)
        // 0xD9, // 16 mode (Discharge/Precharge period)
        // 0x22, // 16 val (Precharge = 2, Discharge = 2)
        // 0xDB, // 18 mode (VCOM deselect level)
        // 0x20, // 18 val (0x20 = 0.770)
        // 0xA4, // 7 (Entire display on)
        // 0xA6, // 8 (Set normal display)
        // 0xAF, // 11 (Display on)
    // };

    i2c_master_transmit(display->handle, init_cmd_buf, sizeof(init_cmd_buf), 1000);

    // Clear display
    for (uint8_t page = 0; page < 8; page++) {
        uint8_t pos_cmd_buf[] = {
            SH1106_COMMAND_MODE,
            sh1106_set_page_byte(page),
            sh1106_set_upper_column_byte(0),
            sh1106_set_lower_column_byte(0),
        };

        i2c_master_transmit(display->handle, pos_cmd_buf, sizeof(pos_cmd_buf) / sizeof(*pos_cmd_buf), 500);

        uint8_t clear_buf[133];
        clear_buf[0] = SH1106_DATA_MODE;
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
            SH1106_COMMAND_MODE,
            sh1106_set_page_byte(page),
            sh1106_set_upper_column_byte(2),
            sh1106_set_lower_column_byte(2),
        };
        i2c_master_transmit(display->handle, pos_cmd_buf, sizeof(pos_cmd_buf) / sizeof(*pos_cmd_buf), 500);

        uint8_t data_buf[129];
        data_buf[0] = SH1106_DATA_MODE;
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
        data_buf[0] = SH1106_DATA_MODE;
        uint8_t data_amt = 0;
        for (uint8_t col = 0; col < 128; col++) {
            bool update = bit_check(display->frame_change[page * 16 + col / 8], col % 8);
            if (!update && previous) {
                uint8_t pos_cmd_buf[] = {
                    SH1106_COMMAND_MODE,
                    sh1106_set_page_byte(page),
                    sh1106_set_upper_column_byte(col - data_amt + 2),
                    sh1106_set_lower_column_byte(col - data_amt + 2),
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
                SH1106_COMMAND_MODE,
                sh1106_set_page_byte(page),
                sh1106_set_upper_column_byte(128 - data_amt + 2),
                sh1106_set_lower_column_byte(128 - data_amt + 2),
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

bitmap_t sh1106_bitmap_new(bool *bool_data, uint8_t width, uint8_t height) {
    int data_size = height / 8 * width + width;
    uint8_t *data = malloc(data_size);
    if (data == NULL)
        return (bitmap_t) { .data = NULL, .height = 0, .width = 0 };
    memset(data, 0x00, data_size);
    for (uint8_t x = 0; x < width; x++) {
        for (uint8_t y = 0; y < height; y++) {
            if (bool_data[y * width + x])
                data[y / 8 * width + x] = bit_set(data[y / 8 * width + x], y % 8, true);
        }
    }
    bitmap_t res = {
        .data = data,
        .data_length = data_size,
        .width = width,
        .height = height,
    };
    return res;
}

void sh1106_bitmap_destroy(bitmap_t bitmap) {
    free(bitmap.data);
    bitmap.data_length = 0;
    bitmap.width = 0;
    bitmap.height = 0;
}

void sh1106_bitmap_init(bool *bool_data, uint8_t width, uint8_t height, bitmap_t *dest) {
    for (uint8_t x = 0; x < width; x++) {
        for (uint8_t y = 0; y < height; y++) {
            dest->data[y / 8 * width + x] = bit_set(dest->data[y / 8 * width + x], y % 8, bool_data[y * width + x]);
        }
    }
    dest->data_length = height / 8 * width;
    dest->width = width;
    dest->height = height;
}

void sh1106_draw_bitmap(sh1106_t *display, bitmap_t bitmap, uint8_t x, uint8_t y) {
    if (bitmap.height > 8) {
        return; // REMOVE
    }

    xSemaphoreTake(display->frame_mutex, portMAX_DELAY);
    for (uint8_t i = 0; i < bitmap.data_length; i++) {
        uint8_t page = y / 8 + i / bitmap.width;
        uint8_t col = x + i % bitmap.width;
        if (page >= 8)
            break;
        if (col >= 128)
            continue;
        uint8_t bmap_byte = bitmap.data[i];
        uint8_t frame_byte = display->frame_buf[page * 128 + x];
        uint8_t new_byte = (bmap_byte & (0xFF >> (8 - bitmap.height))) | (frame_byte & (0xFF << bitmap.height));
        if (new_byte == frame_byte)
            continue;
        display->frame_buf[page * 128 + x] = new_byte;

        bool change_set = bit_check(display->frame_change[page * 16 + col / 8], col % 8);
        if (!change_set) {
            display->frame_change[page * 16 + col / 8] |= 0x01 << (col % 8);
            display->page_change |= 0x01 << page;
            display->frame_change_amt++;
        }
    }
    xSemaphoreGive(display->frame_mutex);
}

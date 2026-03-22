#include <freertos/FreeRTOS.h>
#include <driver/i2c_master.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

void clear_frame();

i2c_master_bus_handle_t i2c_master_bus_handle;
i2c_master_dev_handle_t sh1106_handle;
SemaphoreHandle_t i2c_mutex;

SemaphoreHandle_t display_update_sem;

uint8_t frame_buf[128 * 64 / 8];
uint8_t frame_change[128];
int frame_change_amt = 0;
SemaphoreHandle_t frame_mutex;
uint8_t page_change = 0x00;
bool frame_cleared = false;

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

void init_i2c() {
    i2c_master_bus_config_t i2c_master_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = I2C_NUM_0,
        .scl_io_num = 22,
        .sda_io_num = 23,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };

    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_master_config, &i2c_master_bus_handle));
    printf("Initialized I2C master bus!\n");

    i2c_mutex = xSemaphoreCreateMutex();
}

void init_sh1106() {
    ESP_ERROR_CHECK(i2c_master_probe(i2c_master_bus_handle, 0x3C, 500));
    printf("Found SH1106 display at address 0x3C!\n");

    i2c_device_config_t sh1106_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = 0x3C,
        .scl_speed_hz = 400000,
    };

    ESP_ERROR_CHECK(i2c_master_bus_add_device(i2c_master_bus_handle, &sh1106_config, &sh1106_handle));

    // SH1106 initialization
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

    i2c_master_transmit(sh1106_handle, init_cmd_buf, sizeof(init_cmd_buf) / sizeof(*init_cmd_buf), 1000);

    // Clear display
    for (uint8_t page = 0; page < 8; page++) {
        uint8_t pos_cmd_buf[] = {
            0x00,
            sh1106_set_page(page),
            sh1106_set_upper_column(0),
            sh1106_set_lower_column(0),
        };

        i2c_master_transmit(sh1106_handle, pos_cmd_buf, sizeof(pos_cmd_buf) / sizeof(*pos_cmd_buf), 500);

        uint8_t clear_buf[133];
        clear_buf[0] = 0x40;
        memset(clear_buf + sizeof(*clear_buf), 0x00, 132);

        i2c_master_transmit(sh1106_handle, clear_buf, 133, 500);
    }

    clear_frame();
    frame_mutex = xSemaphoreCreateMutex();

    printf("Initialized SH1106 device!\n");
}

void update_full_display() {
        xSemaphoreTake(i2c_mutex, portMAX_DELAY);
        for (uint8_t page = 0; page < 8; page++) {
            uint8_t pos_cmd_buf[] = {
                0x00,
                sh1106_set_page(page),
                sh1106_set_upper_column(2),
                sh1106_set_lower_column(2),
            };
            i2c_master_transmit(sh1106_handle, pos_cmd_buf, sizeof(pos_cmd_buf) / sizeof(*pos_cmd_buf), 500);

            uint8_t data_buf[129];
            data_buf[0] = 0x40;
            memcpy(data_buf + sizeof(*data_buf), frame_buf + page * 128 * sizeof(*frame_buf), 128);
            i2c_master_transmit(sh1106_handle, data_buf, 129, 500);
        }
        xSemaphoreGive(i2c_mutex);
}

void update_part_display() {
    xSemaphoreTake(i2c_mutex, portMAX_DELAY);
    for (uint8_t page = 0; page < 8; page++) {
        if (!((page_change & (0x01 << page)) > 0x00))
            continue;
        bool previous = false;
        uint8_t data_buf[129];
        data_buf[0] = 0x40;
        uint8_t data_amt = 0;
        for (uint8_t col = 0; col < 128; col++) {
            bool update = bit_check(frame_change[page * 16 + col / 8], col % 8);
            if (!update && previous) {
                uint8_t pos_cmd_buf[] = {
                    0x00,
                    sh1106_set_page(page),
                    sh1106_set_upper_column(col - data_amt + 2),
                    sh1106_set_lower_column(col - data_amt + 2),
                };
                i2c_master_transmit(sh1106_handle, pos_cmd_buf, sizeof(pos_cmd_buf), 500);
                i2c_master_transmit(sh1106_handle, data_buf, data_amt + 1, 500);
                data_amt = 0;
            }
            if (update) {
                data_buf[data_amt++ + 1] = frame_buf[page * 128 + col];
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
            i2c_master_transmit(sh1106_handle, pos_cmd_buf, sizeof(pos_cmd_buf), 500);
            i2c_master_transmit(sh1106_handle, data_buf, data_amt + 1, 500);
        }
    }
    xSemaphoreGive(i2c_mutex);
}

void clear_frame() {
    memset(frame_buf, 0x00, sizeof(frame_buf));
    memset(frame_change, 0x00, sizeof(frame_change));
    frame_change_amt = 0;
    page_change = 0x00;
    frame_cleared = true;
}

void clear_frame_changes() {
    memset(frame_change, 0x00, sizeof(frame_change));
    frame_change_amt = 0;
    page_change = 0x00;
}

void draw_pixel(uint8_t x, uint8_t y, bool on) {
    if (x >= 128 || y >= 64) {
        return;
    }
    bool change_needed;
    if (on) {
        if (bit_check(frame_buf[y / 8 * 128 + x], y % 8)) {
            change_needed = false;
        } else {
            change_needed = true;
            frame_buf[y / 8 * 128 + x] |= 0x01 << (y % 8);
        }
    } else {
        if (!bit_check(frame_buf[y / 8 * 128 + x], y % 8)) {
            change_needed = false;
        } else {
            change_needed = true;
            frame_buf[y / 8 * 128 + x] &= (0x01 << (y % 8)) ^ 0xFF;
        }
    }
    if (!change_needed) {
        return;
    }
    bool change_set = bit_check(frame_change[y / 8 * 16 + x / 8], x % 8);
    if (!change_set) {
        frame_change[y / 8 * 16 + x / 8] |= 0x01 << (x % 8);
        page_change |= 0x01 << (y / 8);
        frame_change_amt++;
    }
} 

typedef struct {
    uint8_t x;
    uint8_t y;
} Coordinate_t;

void draw_img(uint8_t *bitmap, uint8_t x_size, uint8_t y_size, Coordinate_t pos) {
    for (uint8_t y = 0; y < y_size; y++) {
        for (uint8_t x = 0; x < x_size; x++) {
            draw_pixel(x + pos.x, y + pos.y, bitmap[y * x_size + x]);
        }
    }
}

void update_display(void *arg) {
    while (1) {
        xSemaphoreTake(frame_mutex, portMAX_DELAY);
        if (frame_cleared || frame_change_amt > 32) {
            update_full_display();
            frame_cleared = false;
        } else {
            update_part_display();
        }
        clear_frame_changes();
        xSemaphoreGive(frame_mutex);
        xSemaphoreGive(display_update_sem);
        vTaskDelay(33 / portTICK_PERIOD_MS);
    }
}

void draw_sine_wave(void *arg) {
    double x = -64;
    int x_offset = 64;
    int y_offset = 32;
    bool on = true;
    while (1) {
        xSemaphoreTake(display_update_sem, portMAX_DELAY);
        xSemaphoreTake(frame_mutex, portMAX_DELAY);
        for (int i = 0; i < 8; i++) {
            uint8_t y = (uint8_t) round(sin(x / 4) * 10);
            if (y + y_offset < 0) {
                continue;
            }
            draw_pixel((uint8_t)round(x + x_offset), y + y_offset, on);
            x += 0.25;
            if (round(x + x_offset) >= 128) {
                x = -x_offset;
                on ^= true;
            }
        }
        xSemaphoreGive(frame_mutex);
    }
}

void draw_box(void *arg) {
    int x = 0;
    int y = 0;
    while (1) {
        xSemaphoreTake(display_update_sem, portMAX_DELAY);
        uint8_t box_bitmap[] = {
            1, 1, 1, 1, 1, 1, 1, 1,
            1, 1, 0, 0, 0, 0, 1, 1,
            1, 0, 1, 0, 0, 1, 0, 1,
            1, 0, 0, 1, 1, 0, 0, 1,
            1, 0, 0, 1, 1, 0, 0, 1,
            1, 0, 1, 0, 0, 1, 0, 1,
            1, 1, 0, 0, 0, 0, 1, 1,
            1, 1, 1, 1, 1, 1, 1, 1,
        };
        uint8_t x_size = 8;
        uint8_t y_size = 8;
        Coordinate_t pos = { .x = x, .y = y };

        xSemaphoreTake(frame_mutex, portMAX_DELAY);
        clear_frame();
        draw_img(box_bitmap, x_size, y_size, pos);
        xSemaphoreGive(frame_mutex);
        x += 2;
        if (x + x_size  >= 128) {
            x = 0;
            y += y_size;
        }
        if (y + y_size >= 64) {
            y = 0;
        }
    }
}

void fill_display(void *arg) {
    uint8_t i = 2;
    bool on = true;
    while (1) {
        xSemaphoreTake(display_update_sem, portMAX_DELAY);
        xSemaphoreTake(frame_mutex, portMAX_DELAY);
        for (uint8_t x = 0; x < i && x < 128; x++) {
            for (uint8_t y = 0; x + y < i && y < 64; y++) {
                draw_pixel(x, y, on);
                if (x >= 127 && y >= 63) {
                    on ^= true;
                    i = 2;
                }
            }
        }
        xSemaphoreGive(frame_mutex);
        i++;
    }
}

void app_main(void) {
    init_i2c();
    init_sh1106();

    display_update_sem = xSemaphoreCreateBinary();

    xTaskCreate(update_display, "sh1106 update", 2048, NULL, 5, NULL);
    // xTaskCreate(draw_sine_wave, "sine wave", 2048, NULL, 5, NULL);
    // xTaskCreate(draw_box, "draw box", 2048, NULL, 4, NULL);
    xTaskCreate(fill_display, "fill display", 2048, NULL, 4, NULL);
}

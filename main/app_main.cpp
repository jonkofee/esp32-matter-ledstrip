// main/app_main.cpp — ESP-Matter + WS2812B (ESP32), led_strip v3 (RMT)
#include <algorithm>
#include <cmath>
#include <cstring>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include "esp_log.h"
#include "led_strip.h"
#include "led_strip_rmt.h"

// esp-matter
#include "esp_matter.h"
#include "esp_matter_endpoint.h"
#include "esp_matter_cluster.h"
#include "esp_matter_attribute.h"
#include <app/server/OnboardingCodesUtil.h>

using namespace esp_matter;
using namespace esp_matter::endpoint;
using namespace chip::app::Clusters; // OnOff, LevelControl, ColorControl

static const char *TAG = "matter-ws2812";

// ===================== Настройки ленты =====================
static constexpr gpio_num_t LED_PIN   = GPIO_NUM_18;  // Замените под вашу плату
static constexpr uint16_t   LED_COUNT = 30;           // Длина ленты

static led_strip_handle_t s_strip = nullptr;

// ===================== Состояние света =====================
struct Rgb { uint8_t r, g, b; };
struct LightState {
    bool     on = true;          // OnOff
    uint8_t  level = 254;        // 0..254 (Matter CurrentLevel)
    Rgb      rgb = {255, 180, 80};
} g_light;

static inline uint8_t clamp255(int v) { return (uint8_t)std::max(0, std::min(255, v)); }

// HSV(0..254, 0..254, 0..254) -> RGB(0..255)
static Rgb hsv_to_rgb(uint8_t h_in, uint8_t s_in, uint8_t v_in) {
    float h = (float)h_in * 360.0f / 254.0f;
    float s = (float)s_in / 254.0f;
    float v = (float)v_in / 254.0f;
    float c = v * s;
    float x = c * (1 - fabsf(fmodf(h / 60.0f, 2) - 1));
    float m = v - c;
    float r=0,g=0,b=0;
    if (h < 60)      { r=c; g=x; b=0; }
    else if (h < 120){ r=x; g=c; b=0; }
    else if (h < 180){ r=0; g=c; b=x; }
    else if (h < 240){ r=0; g=x; b=c; }
    else if (h < 300){ r=x; g=0; b=c; }
    else             { r=c; g=0; b=x; }
    return { clamp255((int)((r + m) * 255.0f)),
             clamp255((int)((g + m) * 255.0f)),
             clamp255((int)((b + m) * 255.0f)) };
}

// CIE xy + яркость(level) -> RGB (sRGB, D65)
static Rgb xy_to_rgb(uint16_t x_u16, uint16_t y_u16, uint8_t level_in) {
    float x = (float)x_u16 / 65535.0f;
    float y = (float)y_u16 / 65535.0f;
    float Y = std::max(0.0f, std::min(1.0f, (float)level_in / 254.0f));
    if (y < 1e-6f) return {0,0,0};
    float X = (Y / y) * x;
    float Z = (Y / y) * (1.0f - x - y);
    // XYZ -> linear RGB (sRGB)
    float r_lin =  3.2406f*X - 1.5372f*Y - 0.4986f*Z;
    float g_lin = -0.9689f*X + 1.8758f*Y + 0.0415f*Z;
    float b_lin =  0.0557f*X - 0.2040f*Y + 1.0570f*Z;
    auto gamma = [](float u){
        u = std::max(0.0f, u);
        return (u <= 0.0031308f) ? 12.92f*u : 1.055f*powf(u, 1.0f/2.4f) - 0.055f;
    };
    return { clamp255((int)(gamma(r_lin)*255.0f)),
             clamp255((int)(gamma(g_lin)*255.0f)),
             clamp255((int)(gamma(b_lin)*255.0f)) };
}

// Mireds (1e6/K) -> xy (аппроксимация Питерса, годится для 2000–6500K)
static void mired_to_xy(uint16_t mired, uint16_t &out_x_u16, uint16_t &out_y_u16) {
    float K = 1000000.0f / std::max<uint16_t>(mired, 1); // защита от 0
    K = std::max(1000.0f, std::min(25000.0f, K));
    float x;
    if (K >= 1667.0f && K <= 4000.0f) {
        x = -0.2661239e9f/(K*K*K) - 0.2343580e6f/(K*K) + 0.8776956e3f/K + 0.179910f;
    } else {
        x = -3.0258469e9f/(K*K*K) + 2.1070379e6f/(K*K) + 0.2226347e3f/K + 0.240390f;
    }
    float y;
    if (K >= 1667.0f && K <= 2222.0f) {
        y = -1.1063814f*(x*x*x) - 1.34811020f*(x*x) + 2.18555832f*x - 0.20219683f;
    } else if (K <= 4000.0f) {
        y = -0.9549476f*(x*x*x) - 1.37418593f*(x*x) + 2.09137015f*x - 0.16748867f;
    } else {
        y = 3.0817580f*(x*x*x) - 5.87338670f*(x*x) + 3.75112997f*x - 0.37001483f;
    }
    // вернём как 0..65535
    out_x_u16 = (uint16_t)std::max(0, std::min(65535, (int)std::round(x * 65535.0f)));
    out_y_u16 = (uint16_t)std::max(0, std::min(65535, (int)std::round(y * 65535.0f)));
}

static Rgb ct_mired_to_rgb(uint16_t mired, uint8_t level_in) {
    uint16_t x, y;
    mired_to_xy(mired, x, y);
    return xy_to_rgb(x, y, level_in);
}

static void apply_to_strip() {
    if (!s_strip) return;
    uint8_t scale = g_light.on ? g_light.level : 0; // 0..254
    for (uint32_t i=0; i<LED_COUNT; ++i) {
        uint8_t r = (uint8_t)((g_light.rgb.r * scale) / 254);
        uint8_t g = (uint8_t)((g_light.rgb.g * scale) / 254);
        uint8_t b = (uint8_t)((g_light.rgb.b * scale) / 254);
        led_strip_set_pixel(s_strip, i, r, g, b);
    }
    led_strip_refresh(s_strip);
}

static void led_init() {
    // v3-only init (RMT backend)
    led_strip_config_t strip_config = {};
    strip_config.strip_gpio_num = LED_PIN;
    strip_config.max_leds = LED_COUNT;
    strip_config.led_model = LED_MODEL_WS2812;
    strip_config.color_component_format = LED_STRIP_COLOR_COMPONENT_FMT_GRB; // WS2812B = GRB
    strip_config.flags.invert_out = 0;

    led_strip_rmt_config_t rmt_config = {};
    rmt_config.clk_src = RMT_CLK_SRC_DEFAULT;
    rmt_config.resolution_hz = 10 * 1000 * 1000;  // 10 MHz
    rmt_config.mem_block_symbols = 64;            // default depth
    rmt_config.flags.with_dma = 0;                // set 1 for long strips on S3

    ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_config, &rmt_config, &s_strip));

    // Clear on boot
    led_strip_clear(s_strip);
}

// ===================== Matter callbacks =====================
static uint16_t s_light_endpoint_id = 0;

static esp_err_t app_attribute_update_cb(attribute::callback_type_t type, uint16_t endpoint_id,
                                         uint32_t cluster_id, uint32_t attribute_id,
                                         esp_matter_attr_val_t *val, void *priv) {
    if (type != attribute::PRE_UPDATE) return ESP_OK;
    if (endpoint_id != s_light_endpoint_id) return ESP_OK;

    if (cluster_id == OnOff::Id && attribute_id == OnOff::Attributes::OnOff::Id) {
        g_light.on = val->val.b;
        ESP_LOGI(TAG, "OnOff -> %d", g_light.on);
        apply_to_strip();
        return ESP_OK;
    }
    if (cluster_id == LevelControl::Id && attribute_id == LevelControl::Attributes::CurrentLevel::Id) {
        g_light.level = val->val.u8; // 0..254
        ESP_LOGI(TAG, "Level -> %u", g_light.level);
        apply_to_strip();
        return ESP_OK;
    }
    if (cluster_id == ColorControl::Id) {
        if (attribute_id == ColorControl::Attributes::CurrentHue::Id) {
            uint8_t hue = val->val.u8;
            uint8_t sat = 254; // по умолчанию насыщенность 100%
            g_light.rgb = hsv_to_rgb(hue, sat, 254);
            ESP_LOGI(TAG, "Hue -> %u", hue);
            apply_to_strip();
            return ESP_OK;
        } else if (attribute_id == ColorControl::Attributes::CurrentSaturation::Id) {
            uint8_t sat = val->val.u8;
            uint8_t approx_h = 0; // упрощение
            g_light.rgb = hsv_to_rgb(approx_h, sat, 254);
            ESP_LOGI(TAG, "Saturation -> %u", sat);
            apply_to_strip();
            return ESP_OK;
        } else if (attribute_id == ColorControl::Attributes::ColorTemperatureMireds::Id) {
            uint16_t mired = val->val.u16; // 153..500 типично
            g_light.rgb = ct_mired_to_rgb(mired, g_light.level);
            ESP_LOGI(TAG, "CT (mireds) -> %u", mired);
            apply_to_strip();
            return ESP_OK;
        } else if (attribute_id == ColorControl::Attributes::CurrentX::Id ||
                   attribute_id == ColorControl::Attributes::CurrentY::Id) {
            // Получим обе XY (если пришла одна — вторая всё равно уже есть в БД)
            endpoint_t *ep_h = endpoint::get(endpoint_id);
            cluster_t *cc = cluster::get(ep_h, ColorControl::Id);
            attribute_t *attr_x = attribute::get(cc, ColorControl::Attributes::CurrentX::Id);
            attribute_t *attr_y = attribute::get(cc, ColorControl::Attributes::CurrentY::Id);
            esp_matter_attr_val_t x_val = esp_matter_invalid(nullptr);
            esp_matter_attr_val_t y_val = esp_matter_invalid(nullptr);
            attribute::get_val(attr_x, &x_val);
            attribute::get_val(attr_y, &y_val);
            uint16_t x = x_val.val.u16;
            uint16_t y = y_val.val.u16;
            g_light.rgb = xy_to_rgb(x, y, g_light.level);
            ESP_LOGI(TAG, "XY -> %u,%u", x, y);
            apply_to_strip();
            return ESP_OK;
        }
    }
    return ESP_OK;
}

extern "C" void app_main(void) {
    ESP_ERROR_CHECK(nvs_flash_init());
    led_init();

    // ====== Создаём узел и endpoint света ======
    node::config_t node_cfg;
    node_t *node = node::create(&node_cfg, app_attribute_update_cb, nullptr);
    if (!node) {
        ESP_LOGE(TAG, "Failed to create Matter node");
        return;
    }

    extended_color_light::config_t light_cfg; // цветной светильник
    light_cfg.on_off.on_off = g_light.on;
    light_cfg.level_control.current_level = g_light.level; // стартовая яркость
    // По умолчанию показываем режим цвета (HS), но поддерживаем и XY, и CT
    light_cfg.color_control.color_mode = (uint8_t)ColorControl::ColorMode::kCurrentHueAndCurrentSaturation;
    light_cfg.color_control.enhanced_color_mode = (uint8_t)ColorControl::ColorMode::kCurrentHueAndCurrentSaturation;
    light_cfg.color_control.color_temperature.startup_color_temperature_mireds = nullptr; // сохранять прошлое при ребуте

    endpoint_t *ep = extended_color_light::create(node, &light_cfg, ENDPOINT_FLAG_NONE, nullptr);

    // Явно включим все нужные фичи Color Control: HS + XY + CT
    {
        cluster_t *cc = cluster::get(ep, ColorControl::Id);
        // FeatureMap (битовая маска): 0-HS, 3-XY, 4-CT
        attribute_t *attr_feature = attribute::get(cc, ColorControl::Attributes::FeatureMap::Id);
        uint32_t feature_bits = (1u << 0) | (1u << 3) | (1u << 4);
        esp_matter_attr_val_t fval = esp_matter_uint32(feature_bits);
        attribute::set_val(attr_feature, &fval);
        // ColorCapabilities (та же маска, чаще читается контроллерами)
        attribute_t *attr_caps = attribute::get(cc, ColorControl::Attributes::ColorCapabilities::Id);
        uint16_t caps_bits = (1u << 0) | (1u << 3) | (1u << 4);
        esp_matter_attr_val_t cval = esp_matter_uint16(caps_bits);
        attribute::set_val(attr_caps, &cval);
        // Границы CT для корректной работы адаптива и слайдера температуры
        attribute_t *attr_ct_min = attribute::get(cc, ColorControl::Attributes::ColorTempPhysicalMinMireds::Id);
        attribute_t *attr_ct_max = attribute::get(cc, ColorControl::Attributes::ColorTempPhysicalMaxMireds::Id);
        esp_matter_attr_val_t vmin = esp_matter_uint16(153); // ~6500K
        esp_matter_attr_val_t vmax = esp_matter_uint16(500); // ~2000K
        attribute::set_val(attr_ct_min, &vmin);
        attribute::set_val(attr_ct_max, &vmax);
    }
    s_light_endpoint_id = endpoint::get_id(ep);
    ESP_LOGI(TAG, "Light endpoint id = %u", s_light_endpoint_id);

    // Запуск стека Matter
    ESP_ERROR_CHECK(esp_matter::start(nullptr));

    ESP_LOGI(TAG, "Booted. Откройте приложение-дом (Apple/Google/HA) и добавьте устройство по QR/коду из лога.");

    // Показать текущий цвет/яркость
    apply_to_strip();
}

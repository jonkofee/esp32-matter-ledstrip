/*
   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#include <esp_err.h>
#include <esp_log.h>
#include <nvs_flash.h>

#include <esp_matter.h>
#include <esp_matter_console.h>
#include <esp_matter_ota.h>
#include <esp_matter_attribute.h>

#include <math.h>

#include "led_strip.h"
#include "led_strip_rmt.h"

#define LED_STRIP_GPIO_PIN  2
#define LED_STRIP_LED_COUNT 144 * 2
// 10MHz resolution, 1 tick = 0.1us (led strip needs a high resolution)
#define LED_STRIP_RMT_RES_HZ  (10 * 1000 * 1000)

#if CHIP_DEVICE_CONFIG_ENABLE_THREAD
#include <platform/ESP32/OpenthreadLauncher.h>
#endif

#include <app/server/CommissioningWindowManager.h>
#include <app/server/Server.h>

static const char *TAG = "app_main";
uint16_t light_endpoint_id = 0;

using namespace esp_matter;
using namespace esp_matter::attribute;
using namespace esp_matter::endpoint;
using namespace chip::app::Clusters;

constexpr auto k_timeout_seconds = 300;

// ===================== WS2812B state & helpers =====================
static led_strip_handle_t s_strip = nullptr;

struct Rgb { uint8_t r, g, b; };
struct LightState {
    bool    on    = true;    // OnOff
    uint8_t level = 254;     // 0..254 (Matter CurrentLevel)
    Rgb     rgb   = {255, 180, 80};
};
static LightState g_light;

// --------- Color helpers (HS/XY -> RGB, independent from brightness level) ---------
static inline uint8_t clamp_u8(float x) { return (uint8_t) (x < 0 ? 0 : (x > 255 ? 255 : x)); }

static Rgb hsv_to_rgb_u8(uint8_t hue254, uint8_t sat254)
{
    // Hue: 0..254 -> 0..360°, Saturation: 0..254 -> 0..1, Value=1.0 (brightness handled separately)
    float H = (float)hue254 * 360.0f / 254.0f;
    float S = (float)sat254 / 254.0f;
    float V = 1.0f;

    float C = V * S;
    float X = C * (1.0f - fabsf(fmodf(H / 60.0f, 2.0f) - 1.0f));
    float m = V - C;

    float r1 = 0, g1 = 0, b1 = 0;
    if      (H < 60)  { r1 = C; g1 = X; b1 = 0; }
    else if (H < 120) { r1 = X; g1 = C; b1 = 0; }
    else if (H < 180) { r1 = 0; g1 = C; b1 = X; }
    else if (H < 240) { r1 = 0; g1 = X; b1 = C; }
    else if (H < 300) { r1 = X; g1 = 0; b1 = C; }
    else              { r1 = C; g1 = 0; b1 = X; }

    Rgb out;
    out.r = clamp_u8((r1 + m) * 255.0f);
    out.g = clamp_u8((g1 + m) * 255.0f);
    out.b = clamp_u8((b1 + m) * 255.0f);
    return out;
}

static inline float max3(float a, float b, float c) { return fmaxf(a, fmaxf(b, c)); }

static Rgb xy_to_rgb_u8(uint16_t x16, uint16_t y16)
{
    // Convert 0..65535 fixed-point x,y to CIE xy in 0..1
    float x = (float)x16 / 65535.0f;
    float y = (float)y16 / 65535.0f;

    // Basic sanity/clamping
    if (y < 1e-5f) y = 1e-5f;
    if (x < 0.0f) x = 0.0f;
    if (x > 0.9999f) x = 0.9999f;
    if (x + y > 0.9999f) { float s = 0.9999f / (x + y); x *= s; y *= s; }

    // Assume Y (luminance) = 1.0; brightness handled by LevelControl separately
    float Y = 1.0f;
    float X = (Y / y) * x;
    float Z = (Y / y) * (1.0f - x - y);

    // XYZ (D65) -> linear sRGB
    float r =  3.2406f * X - 1.5372f * Y - 0.4986f * Z;
    float g = -0.9689f * X + 1.8758f * Y + 0.0415f * Z;
    float b =  0.0557f * X - 0.2040f * Y + 1.0570f * Z;

    // Clamp negative and normalize so the max channel is 1.0
    r = r < 0 ? 0 : r; g = g < 0 ? 0 : g; b = b < 0 ? 0 : b;
    float m = max3(r, g, b);
    if (m > 0) { r /= m; g /= m; b /= m; }

    Rgb out;
    out.r = clamp_u8(r * 255.0f);
    out.g = clamp_u8(g * 255.0f);
    out.b = clamp_u8(b * 255.0f);
    return out;
}

static Rgb cct_mireds_to_rgb_u8(uint16_t mireds)
{
    // Convert Mireds to Kelvin and approximate to sRGB
    float m = (mireds == 0) ? 1.0f : (float)mireds;
    float K = 1000000.0f / m; // Kelvin
    if (K < 1000.0f) K = 1000.0f;
    if (K > 40000.0f) K = 40000.0f;
    float T = K / 100.0f;

    float R, G, B;
    if (T <= 66.0f) {
        R = 255.0f;
        G = 99.4708025861f * logf(T) - 161.1195681661f;
        if (T <= 19.0f)
            B = 0.0f;
        else
            B = 138.5177312231f * logf(T - 10.0f) - 305.0447927307f;
    } else {
        R = 329.698727446f * powf(T - 60.0f, -0.1332047592f);
        G = 288.1221695283f * powf(T - 60.0f, -0.0755148492f);
        B = 255.0f;
    }

    Rgb out;
    out.r = clamp_u8(R);
    out.g = clamp_u8(G);
    out.b = clamp_u8(B);
    return out;
}

static inline uint8_t scale_u8(uint8_t v, uint8_t lvl)
{
    return (uint16_t)v * (uint16_t)lvl / 254;
}

static void led_init()
{
    // LED strip general initialization, according to your led board design
    led_strip_config_t strip_config = {
        .strip_gpio_num = LED_STRIP_GPIO_PIN, // The GPIO that connected to the LED strip's data line
        .max_leds = LED_STRIP_LED_COUNT,      // The number of LEDs in the strip,
        .led_model = LED_MODEL_WS2812,        // LED strip model
        .color_component_format = LED_STRIP_COLOR_COMPONENT_FMT_GRB, // The color order of the strip: GRB
        .flags = {
            .invert_out = false, // don't invert the output signal
        }
    };

    // LED strip backend configuration: RMT
    led_strip_rmt_config_t rmt_config = {
        .clk_src = RMT_CLK_SRC_DEFAULT,        // different clock source can lead to different power consumption
        .resolution_hz = LED_STRIP_RMT_RES_HZ, // RMT counter clock frequency
        .mem_block_symbols = 0, // the memory block size used by the RMT channel
        .flags = {
            .with_dma = 0,     // Using DMA can improve performance when driving more LEDs
        }
    };
    ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_config, &rmt_config, &s_strip));
    ESP_LOGI(TAG, "Created LED strip object with RMT backend");
}

static void apply_light()
{
    if (!s_strip) return;

    if (!g_light.on || g_light.level == 0) {
        led_strip_clear(s_strip);
        return;
    }

    const uint8_t r = scale_u8(g_light.rgb.r, g_light.level);
    const uint8_t g = scale_u8(g_light.rgb.g, g_light.level);
    const uint8_t b = scale_u8(g_light.rgb.b, g_light.level);

    for (int i = 0; i < LED_STRIP_LED_COUNT; ++i) {
        led_strip_set_pixel(s_strip, i, r, g, b);
    }
    led_strip_refresh(s_strip);
}

// ===================== Matter event callback =====================
static void app_event_cb(const ChipDeviceEvent *event, intptr_t arg)
{
    switch (event->Type) {
    case chip::DeviceLayer::DeviceEventType::kInterfaceIpAddressChanged:
        ESP_LOGI(TAG, "Interface IP Address changed");
        break;

    case chip::DeviceLayer::DeviceEventType::kCommissioningComplete:
        ESP_LOGI(TAG, "Commissioning complete");
        break;

    case chip::DeviceLayer::DeviceEventType::kFailSafeTimerExpired:
        ESP_LOGI(TAG, "Commissioning failed, fail safe timer expired");
        break;

    case chip::DeviceLayer::DeviceEventType::kCommissioningSessionStarted:
        ESP_LOGI(TAG, "Commissioning session started");
        break;

    case chip::DeviceLayer::DeviceEventType::kCommissioningSessionStopped:
        ESP_LOGI(TAG, "Commissioning session stopped");
        break;

    case chip::DeviceLayer::DeviceEventType::kCommissioningWindowOpened:
        ESP_LOGI(TAG, "Commissioning window opened");
        break;

    case chip::DeviceLayer::DeviceEventType::kCommissioningWindowClosed:
        ESP_LOGI(TAG, "Commissioning window closed");
        break;

    case chip::DeviceLayer::DeviceEventType::kFabricRemoved: {
        ESP_LOGI(TAG, "Fabric removed successfully");
        if (chip::Server::GetInstance().GetFabricTable().FabricCount() == 0) {
            chip::CommissioningWindowManager & commissionMgr = chip::Server::GetInstance().GetCommissioningWindowManager();
            constexpr auto kTimeoutSeconds = chip::System::Clock::Seconds16(k_timeout_seconds);
            if (!commissionMgr.IsCommissioningWindowOpen()) {
                // Keep Wi‑Fi creds, advertise via DNS‑SD only
                CHIP_ERROR err = commissionMgr.OpenBasicCommissioningWindow(
                    kTimeoutSeconds, chip::CommissioningWindowAdvertisement::kDnssdOnly);
                if (err != CHIP_NO_ERROR) {
                    ESP_LOGE(TAG, "Failed to open commissioning window, err:%" CHIP_ERROR_FORMAT, err.Format());
                }
            }
        }
        break; }

    case chip::DeviceLayer::DeviceEventType::kFabricWillBeRemoved:
        ESP_LOGI(TAG, "Fabric will be removed");
        break;

    case chip::DeviceLayer::DeviceEventType::kFabricUpdated:
        ESP_LOGI(TAG, "Fabric is updated");
        break;

    case chip::DeviceLayer::DeviceEventType::kFabricCommitted:
        ESP_LOGI(TAG, "Fabric is committed");
        break;

    case chip::DeviceLayer::DeviceEventType::kBLEDeinitialized:
        ESP_LOGI(TAG, "BLE deinitialized and memory reclaimed");
        break;

    default:
        break;
    }
}

// ===================== Identify callback =====================
static esp_err_t app_identification_cb(identification::callback_type_t type, uint16_t endpoint_id, uint8_t effect_id,
                                       uint8_t effect_variant, void *priv_data)
{
    ESP_LOGI(TAG, "Identification callback: type:%u effect:%u variant:%u", type, effect_id, effect_variant);
    return ESP_OK;
}

// ===================== Attribute update callback =====================
static esp_err_t app_attribute_update_cb(attribute::callback_type_t type, uint16_t endpoint_id, uint32_t cluster_id,
                                         uint32_t attribute_id, esp_matter_attr_val_t *val, void *priv_data)
{
    if (endpoint_id != light_endpoint_id) return ESP_OK;

    if (type == PRE_UPDATE) {
        if (cluster_id == OnOff::Id && attribute_id == OnOff::Attributes::OnOff::Id) {
            g_light.on = val->val.b;
            apply_light();
        } else if (cluster_id == LevelControl::Id && attribute_id == LevelControl::Attributes::CurrentLevel::Id) {
            g_light.level = val->val.u8;
            apply_light();
        } else if (cluster_id == ColorControl::Id) {
            if (attribute_id == ColorControl::Attributes::CurrentHue::Id ||
                attribute_id == ColorControl::Attributes::CurrentSaturation::Id) {
                // HS update
                uint8_t h = 0, s = 0;
                esp_matter_attr_val_t vh, vs;
                if (auto *ah = attribute::get(light_endpoint_id, ColorControl::Id, ColorControl::Attributes::CurrentHue::Id)) {
                    attribute::get_val(ah, &vh); h = vh.val.u8;
                }
                if (auto *as = attribute::get(light_endpoint_id, ColorControl::Id, ColorControl::Attributes::CurrentSaturation::Id)) {
                    attribute::get_val(as, &vs); s = vs.val.u8;
                }
                g_light.rgb = hsv_to_rgb_u8(h, s);
                apply_light();
            } else if (attribute_id == ColorControl::Attributes::CurrentX::Id ||
                       attribute_id == ColorControl::Attributes::CurrentY::Id) {
                // XY update
                uint16_t x16 = 0, y16 = 0;
                esp_matter_attr_val_t vx, vy;
                if (auto *ax = attribute::get(light_endpoint_id, ColorControl::Id, ColorControl::Attributes::CurrentX::Id)) {
                    attribute::get_val(ax, &vx); x16 = vx.val.u16;
                }
                if (auto *ay = attribute::get(light_endpoint_id, ColorControl::Id, ColorControl::Attributes::CurrentY::Id)) {
                    attribute::get_val(ay, &vy); y16 = vy.val.u16;
                }
                g_light.rgb = xy_to_rgb_u8(x16, y16);
                apply_light();
            } else if (attribute_id == ColorControl::Attributes::ColorTemperatureMireds::Id) {
                // CCT update
                uint16_t mireds = 0;
                esp_matter_attr_val_t vt;
                if (auto *at = attribute::get(light_endpoint_id, ColorControl::Id, ColorControl::Attributes::ColorTemperatureMireds::Id)) {
                    attribute::get_val(at, &vt); mireds = vt.val.u16;
                }
                g_light.rgb = cct_mireds_to_rgb_u8(mireds);
                apply_light();
            }
        }
    }

    return ESP_OK;
}

extern "C" void app_main()
{
    /* Initialize the ESP NVS layer */
    nvs_flash_init();

    /* Initialize WS2812B driver */
    led_init();

    /* Create a Matter node and add the mandatory Root Node device type on endpoint 0 */
    node::config_t node_config;
    node_t *node = node::create(&node_config, app_attribute_update_cb, app_identification_cb);

    extended_color_light::config_t light_config;
    light_config.on_off.on_off = false; // default, then restored by StartUpOnOff=nullptr
    // Restore previous values on power-up:
    light_config.on_off.lighting.start_up_on_off = nullptr;                     // restore OnOff
    light_config.level_control.lighting.start_up_current_level = nullptr;       // restore Brightness
    light_config.color_control.color_temperature.startup_color_temperature_mireds = nullptr; // restore CCT

    // Prefer HS color mode
    light_config.color_control.color_mode = (uint8_t)ColorControl::ColorMode::kCurrentHueAndCurrentSaturation;
    light_config.color_control.enhanced_color_mode = (uint8_t)ColorControl::ColorMode::kCurrentHueAndCurrentSaturation;

    // Create light endpoint
    endpoint_t *endpoint = extended_color_light::create(node, &light_config, ENDPOINT_FLAG_NONE, nullptr);
    light_endpoint_id = endpoint::get_id(endpoint);

    // Persist frequently changing attributes with deferred writes (save flash)
    attribute::set_deferred_persistence(attribute::get(light_endpoint_id, OnOff::Id, OnOff::Attributes::OnOff::Id));
    attribute::set_deferred_persistence(attribute::get(light_endpoint_id, LevelControl::Id, LevelControl::Attributes::CurrentLevel::Id));
    // Defer XY/CCT too, plus HS (if controllers use HS)
    attribute::set_deferred_persistence(attribute::get(light_endpoint_id, ColorControl::Id, ColorControl::Attributes::CurrentHue::Id));
    attribute::set_deferred_persistence(attribute::get(light_endpoint_id, ColorControl::Id, ColorControl::Attributes::CurrentSaturation::Id));
    attribute::set_deferred_persistence(attribute::get(light_endpoint_id, ColorControl::Id, ColorControl::Attributes::CurrentX::Id));
    attribute::set_deferred_persistence(attribute::get(light_endpoint_id, ColorControl::Id, ColorControl::Attributes::CurrentY::Id));
    attribute::set_deferred_persistence(attribute::get(light_endpoint_id, ColorControl::Id, ColorControl::Attributes::ColorTemperatureMireds::Id));

#if CHIP_DEVICE_CONFIG_ENABLE_THREAD && CHIP_DEVICE_CONFIG_ENABLE_WIFI_STATION
    // Enable secondary network interface (optional)
    secondary_network_interface::config_t secondary_network_interface_config;
    endpoint = endpoint::secondary_network_interface::create(node, &secondary_network_interface_config, ENDPOINT_FLAG_NONE, nullptr);
#endif

#if CHIP_DEVICE_CONFIG_ENABLE_THREAD
    /* Set OpenThread platform config */
    esp_openthread_platform_config_t config = {
        .radio_config = ESP_OPENTHREAD_DEFAULT_RADIO_CONFIG(),
        .host_config = ESP_OPENTHREAD_DEFAULT_HOST_CONFIG(),
        .port_config = ESP_OPENTHREAD_DEFAULT_PORT_CONFIG(),
    };
    set_openthread_platform_config(&config);
#endif

    /* Matter start */
    esp_matter::start(app_event_cb);

    /* Read back restored attributes and apply to LEDs */
    if (auto *a = attribute::get(light_endpoint_id, OnOff::Id, OnOff::Attributes::OnOff::Id)) {
        esp_matter_attr_val_t v; attribute::get_val(a, &v); g_light.on = v.val.b; }
    if (auto *a = attribute::get(light_endpoint_id, LevelControl::Id, LevelControl::Attributes::CurrentLevel::Id)) {
        esp_matter_attr_val_t v; attribute::get_val(a, &v); g_light.level = v.val.u8; }
    // (optional) read HS/XY here and convert to RGB

    // Read color mode and color attributes, compute RGB
    if (auto *a_mode = attribute::get(light_endpoint_id, ColorControl::Id, ColorControl::Attributes::ColorMode::Id)) {
        esp_matter_attr_val_t vmode; attribute::get_val(a_mode, &vmode);
        if (vmode.val.u8 == (uint8_t)ColorControl::ColorMode::kCurrentHueAndCurrentSaturation) {
            esp_matter_attr_val_t vh, vs;
            if (auto *ah = attribute::get(light_endpoint_id, ColorControl::Id, ColorControl::Attributes::CurrentHue::Id))
                attribute::get_val(ah, &vh);
            if (auto *as = attribute::get(light_endpoint_id, ColorControl::Id, ColorControl::Attributes::CurrentSaturation::Id))
                attribute::get_val(as, &vs);
            uint8_t h = vh.val.u8;
            uint8_t s = vs.val.u8;
            g_light.rgb = hsv_to_rgb_u8(h, s);
        } else if (vmode.val.u8 == (uint8_t)ColorControl::ColorMode::kCurrentXAndCurrentY) {
            esp_matter_attr_val_t vx, vy;
            uint16_t x16 = 0, y16 = 0;
            if (auto *ax = attribute::get(light_endpoint_id, ColorControl::Id, ColorControl::Attributes::CurrentX::Id)) { attribute::get_val(ax, &vx); x16 = vx.val.u16; }
            if (auto *ay = attribute::get(light_endpoint_id, ColorControl::Id, ColorControl::Attributes::CurrentY::Id)) { attribute::get_val(ay, &vy); y16 = vy.val.u16; }
            g_light.rgb = xy_to_rgb_u8(x16, y16);
        } else if (vmode.val.u8 == (uint8_t)ColorControl::ColorMode::kColorTemperature) {
            esp_matter_attr_val_t vt;
            if (auto *at = attribute::get(light_endpoint_id, ColorControl::Id, ColorControl::Attributes::ColorTemperatureMireds::Id))
                attribute::get_val(at, &vt);
            g_light.rgb = cct_mireds_to_rgb_u8(vt.val.u16);
        }
    }

    apply_light();
}
#include <stdio.h>
#include <esp_wifi.h>
#include <esp_event_loop.h>
#include <esp_log.h>
#include <nvs_flash.h>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <homekit/homekit.h>
#include <homekit/characteristics.h>
#include "../wifi.h"

#include "esp32_digital_led_lib.h"

void on_wifi_ready();

esp_err_t event_handler(void *ctx, system_event_t *event)
{
    switch(event->event_id) {
        case SYSTEM_EVENT_STA_START:
            esp_wifi_connect();
            break;
        case SYSTEM_EVENT_STA_GOT_IP:
            on_wifi_ready();
            break;
        case SYSTEM_EVENT_STA_DISCONNECTED:
            esp_wifi_connect();
            break;
        default:
            break;
    }
    return ESP_OK;
}

static void wifi_init() {
    tcpip_adapter_init();
    ESP_ERROR_CHECK(esp_event_loop_init(event_handler, NULL));

    wifi_init_config_t wifi_init_config = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&wifi_init_config));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASSWORD,
        },
    };

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());
}

/*
 * Onboard LED for identify
 */

const int led_gpio = 2;
bool led_on = false;

void led_write(bool on) {
    gpio_set_level(led_gpio, on ? 1 : 0);
}

void led_init() {
    gpio_set_direction(led_gpio, GPIO_MODE_OUTPUT);
    led_write(led_on);
}

void led_identify_task(void *_args) {
    for (int i=0; i<3; i++) {
        for (int j=0; j<2; j++) {
            led_write(true);
            vTaskDelay(100 / portTICK_PERIOD_MS);
            led_write(false);
            vTaskDelay(100 / portTICK_PERIOD_MS);
        }

        vTaskDelay(250 / portTICK_PERIOD_MS);
    }

    led_write(led_on);

    vTaskDelete(NULL);
}

void led_identify(homekit_value_t _value) {
    xTaskCreate(led_identify_task, "LED identify", 512, NULL, 2, NULL);
}

/*
 * LED Strip setup
 */

// **Required** if debugging is enabled in library header
#if DEBUG_ESP32_DIGITAL_LED_LIB
    int digitalLeds_debugBufferSz = 1024;
    char *digitalLeds_debugBuffer = (char *)calloc(digitalLeds_debugBufferSz, sizeof(char));
#endif

strand_t STRANDS[] = { // Avoid using any of the strapping pins on the ESP32
    {.rmtChannel = 1, .gpioNum = 12, .ledType = LED_WS2813_V2, .brightLimit = 32, .numPixels = 8,
        .pixels = NULL, ._stateVars = NULL},
    {.rmtChannel = 0, .gpioNum = 13, .ledType = LED_WS2813_V2, .brightLimit = 32, .numPixels = 16,
        .pixels = NULL, ._stateVars = NULL},
};
int STRANDCNT = sizeof(STRANDS)/sizeof(STRANDS[0]);

#define HIGH 1
#define LOW 0
#define OUTPUT GPIO_MODE_OUTPUT
#define INPUT GPIO_MODE_INPUT

void gpioSetup(int gpioNum, int gpioMode, int gpioVal) {
    gpio_num_t gpioNumNative = (gpio_num_t)gpioNum;
    gpio_mode_t gpioModeNative = (gpio_mode_t)gpioMode;
    gpio_pad_select_gpio(gpioNumNative);
    gpio_set_direction(gpioNumNative, gpioModeNative);
    gpio_set_level(gpioNumNative, gpioVal);
}

/*
 * Color calculation
 */

typedef struct _lightState {
    bool on;
    float hue;
    float saturation;
    float brightness;
} LightState;

#include "math.h"
#define DEG_TO_RAD(X) (M_PI*(X)/180)

pixelColor_t calculate_color(LightState state) {
    int r, g, b, w;
    float cos_h, cos_1047_h;

    state.hue = fmod(state.hue, 360.0); // cycle H around to 0-360 degrees
    state.hue = 3.14159 * state.hue / (float)180.0; // Convert to radians.
    state.saturation = state.saturation > 0 ? (state.saturation < 1 ? state.saturation : 1) : 0; // clamp S and I to interval [0,1]
    state.brightness /= 100.0;
    state.brightness = state.brightness > 0 ? (state.brightness < 1 ? state.brightness : 1) : 0;

    if (state.hue < 2.09439) {
        cos_h = cos(state.hue);
        cos_1047_h = cos(1.047196667 - state.hue);
        r = state.saturation * 255 * state.brightness / 3 * (1 + cos_h / cos_1047_h);
        g = state.saturation * 255 * state.brightness / 3 * (1 + (1 - cos_h / cos_1047_h));
        b = 0;
        w = 255 * (1 - state.saturation) * state.brightness;
    } else if (state.hue < 4.188787) {
        state.hue = state.hue - 2.09439;
        cos_h = cos(state.hue);
        cos_1047_h = cos(1.047196667 - state.hue);
        g = state.saturation * 255 * state.brightness / 3 * (1 + cos_h / cos_1047_h);
        b = state.saturation * 255 * state.brightness / 3 * (1 + (1 - cos_h / cos_1047_h));
        r = 0;
        w = 255 * (1 - state.saturation) * state.brightness;
    } else {
        state.hue = state.hue - 4.188787;
        cos_h = cos(state.hue);
        cos_1047_h = cos(1.047196667 - state.hue);
        b = state.saturation * 255 * state.brightness / 3 * (1 + cos_h / cos_1047_h);
        r = state.saturation * 255 * state.brightness / 3 * (1 + (1 - cos_h / cos_1047_h));
        g = 0;
        w = 255 * (1 - state.saturation) * state.brightness;
    }

    return pixelFromRGBW(r, g, b, w);
}

void update_light(strand_t *strand, LightState state) {
    if (state.on == false) {
        digitalLeds_resetPixels(strand);
    } else {
        pixelColor_t px = calculate_color(state);
        for (uint16_t i = 0; i < strand->numPixels; i++) {
            strand->pixels[i] = px;
        }
        digitalLeds_updatePixels(strand);
    }    
}

/*
 * Floodlight
 */

LightState floodlight = {
    .on = true,
    .hue = 0,
    .saturation = 0,
    .brightness = 100
};

void update_floodlight() {
    update_light(&STRANDS[1], floodlight);
}

homekit_value_t get_floodlight_on() {
    return HOMEKIT_BOOL(floodlight.on);
}

void set_floodlight_on(homekit_value_t value) {
    floodlight.on = value.bool_value;
    update_floodlight();
}

homekit_value_t get_floodlight_hue() {
    return HOMEKIT_FLOAT(floodlight.hue);
}

void set_floodlight_hue(homekit_value_t value) {
    floodlight.hue = value.float_value;
    update_floodlight();
}

homekit_value_t get_floodlight_saturation() {
    return HOMEKIT_FLOAT(floodlight.saturation);
}

void set_floodlight_saturation(homekit_value_t value) {
    floodlight.saturation = value.float_value;
    update_floodlight();
}

homekit_value_t get_floodlight_brightness() {
    return HOMEKIT_INT(floodlight.brightness);
}

void set_floodlight_brightness(homekit_value_t value) {
    floodlight.brightness = value.int_value;
    floodlight.on = (floodlight.brightness > 0);
    update_floodlight();
}

/*
 * Ambilight
 */

LightState ambilight = {
    .on = true,
    .hue = 0,
    .saturation = 0,
    .brightness = 100
};

void update_ambilight() {
    update_light(&STRANDS[0], ambilight);
}

homekit_value_t get_ambilight_on() {
    return HOMEKIT_BOOL(ambilight.on);
}

void set_ambilight_on(homekit_value_t value) {
    ambilight.on = value.bool_value;
    update_ambilight();
}

homekit_value_t get_ambilight_hue() {
    return HOMEKIT_FLOAT(ambilight.hue);
}

void set_ambilight_hue(homekit_value_t value) {
    ambilight.hue = value.float_value;
    update_ambilight();
}

homekit_value_t get_ambilight_saturation() {
    return HOMEKIT_FLOAT(ambilight.saturation);
}

void set_ambilight_saturation(homekit_value_t value) {
    ambilight.saturation = value.float_value;
    update_ambilight();
}

homekit_value_t get_ambilight_brightness() {
    return HOMEKIT_INT(ambilight.brightness);
}

void set_ambilight_brightness(homekit_value_t value) {
    ambilight.brightness = value.int_value;
    ambilight.on = (ambilight.brightness > 0);
    update_ambilight();
}


/*
 * Accessory definition
 */

homekit_accessory_t *accessories[] = {
    HOMEKIT_ACCESSORY(.id=1, .category=homekit_accessory_category_lightbulb, .services=(homekit_service_t*[]){
        HOMEKIT_SERVICE(ACCESSORY_INFORMATION, .characteristics=(homekit_characteristic_t*[]){
            HOMEKIT_CHARACTERISTIC(NAME, "Luminaire"),
            HOMEKIT_CHARACTERISTIC(MANUFACTURER, "Dunkelstern"),
            HOMEKIT_CHARACTERISTIC(SERIAL_NUMBER, "037A2BABF19E"),
            HOMEKIT_CHARACTERISTIC(MODEL, "Luminaire"),
            HOMEKIT_CHARACTERISTIC(FIRMWARE_REVISION, "0.1"),
            HOMEKIT_CHARACTERISTIC(IDENTIFY, led_identify),
            NULL
        }),
        HOMEKIT_SERVICE(LIGHTBULB, .primary=true, .characteristics=(homekit_characteristic_t*[]){
            HOMEKIT_CHARACTERISTIC(NAME, "Floodlight"),
            HOMEKIT_CHARACTERISTIC(
                ON, false,
                .getter=get_floodlight_on,
                .setter=set_floodlight_on
            ),
            HOMEKIT_CHARACTERISTIC(
                BRIGHTNESS, false,
                .getter=get_floodlight_brightness,
                .setter=set_floodlight_brightness
            ),
            HOMEKIT_CHARACTERISTIC(
                HUE, false,
                .getter=get_floodlight_hue,
                .setter=set_floodlight_hue
            ),
            HOMEKIT_CHARACTERISTIC(
                SATURATION, false,
                .getter=get_floodlight_saturation,
                .setter=set_floodlight_saturation
            ),
            NULL
        }),
        HOMEKIT_SERVICE(LIGHTBULB, .primary=false, .characteristics=(homekit_characteristic_t*[]){
            HOMEKIT_CHARACTERISTIC(NAME, "Ambilight"),
            HOMEKIT_CHARACTERISTIC(
                ON, false,
                .getter=get_ambilight_on,
                .setter=set_ambilight_on
            ),
            HOMEKIT_CHARACTERISTIC(
                BRIGHTNESS, false,
                .getter=get_ambilight_brightness,
                .setter=set_ambilight_brightness
            ),
            HOMEKIT_CHARACTERISTIC(
                HUE, false,
                .getter=get_ambilight_hue,
                .setter=set_ambilight_hue
            ),
            HOMEKIT_CHARACTERISTIC(
                SATURATION, false,
                .getter=get_ambilight_saturation,
                .setter=set_ambilight_saturation
            ),
            NULL
        }),
        NULL
    }),
    NULL
};

homekit_server_config_t config = {
    .accessories = accessories,
    .password = "123-11-456"
};

void on_wifi_ready() {
    homekit_server_init(&config);
}

void app_main(void) {
    gpioSetup(12, OUTPUT, LOW);
    gpioSetup(13, OUTPUT, LOW);

    if (digitalLeds_initStrands(STRANDS, STRANDCNT)) {
        printf("Init FAILURE: halting\n");
        while (true) {};
    }
    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK( ret );

    wifi_init();
    led_init();
}

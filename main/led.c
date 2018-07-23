#define ENABLE_MQTT
#undef ENABLE_HOMEKIT

#include <stdio.h>
#include <esp_wifi.h>
#include <esp_event_loop.h>
#include <esp_log.h>
#include <nvs_flash.h>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "../wifi.h"
#include "esp32_digital_led_lib.h"

#ifdef ENABLE_HOMEKIT
#include <homekit/homekit.h>
#include <homekit/characteristics.h>
#endif /* ENABLE_HOMEKIT */

#ifdef ENABLE_MQTT
#include "mqtt_client.h"
static const char *TAG = "MQTT_CLIENT";
#endif /* ENABLE_MQTT */

#define max(a,b) \
   ({ __typeof__ (a) _a = (a); \
       __typeof__ (b) _b = (b); \
     _a > _b ? _a : _b; })

#define min(a,b) \
   ({ __typeof__ (a) _a = (a); \
       __typeof__ (b) _b = (b); \
     _a < _b ? _a : _b; })

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
#ifdef ENABLE_HOMEKIT

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

#endif /* ENABLE_HOMEKIT */

/*
 * MQTT setup
 */

#ifdef ENABLE_MQTT

static esp_err_t mqtt_event_handler(esp_mqtt_event_handle_t event);

#define MQTT_ID "luminaire"

const esp_mqtt_client_config_t mqtt_cfg = {
    .uri = "mqtt://192.168.2.84",
    .event_handle = mqtt_event_handler,
    .client_id = MQTT_ID,
    .username = "homeassistant",
    .password = "dunkelstern738"
    // .user_context = (void *)your_context
};

typedef enum _MQTT_Message{
    MQTT_MSG_AMBILIGHT_HUE         = 0,
    MQTT_MSG_AMBILIGHT_SATURATION  = 1,
    MQTT_MSG_AMBILIGHT_BRIGHTNESS  = 2,
    MQTT_MSG_AMBILIGHT_XY          = 3,
    MQTT_MSG_AMBILIGHT_SWITCH      = 4,
    MQTT_MSG_FLOODLIGHT_HUE        = 5,
    MQTT_MSG_FLOODLIGHT_SATURATION = 6,
    MQTT_MSG_FLOODLIGHT_BRIGHTNESS = 7,
    MQTT_MSG_FLOODLIGHT_XY         = 8,
    MQTT_MSG_FLOODLIGHT_SWITCH     = 9
} MQTT_Message;

#define NUM_MQTT_MSG 10

esp_mqtt_client_handle_t mqtt_client;

static void mqtt_republish(esp_mqtt_client_handle_t client, MQTT_Message typ);

#endif /* ENABLE_MQTT */

/*
 * LED Strip setup
 */

// **Required** if debugging is enabled in library header
#if DEBUG_ESP32_DIGITAL_LED_LIB
    int digitalLeds_debugBufferSz = 1024;
    char *digitalLeds_debugBuffer = (char *)calloc(digitalLeds_debugBufferSz, sizeof(char));
#endif

strand_t STRANDS[] = { // Avoid using any of the strapping pins on the ESP32
    {.rmtChannel = 1, .gpioNum = 12, .ledType = LED_SK6812W_V1, .brightLimit = 255, .numPixels = 8,
        .pixels = NULL, ._stateVars = NULL},
    {.rmtChannel = 0, .gpioNum = 13, .ledType = LED_SK6812W_V1, .brightLimit = 255, .numPixels = 16,
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
    float x;
    float y;
} LightState;

#include "math.h"
#define DEG_TO_RAD(X) (M_PI*(X)/180.0)

double exponential_in(double percentage) {
    return pow(2, 10 * (percentage - 1.0));
}

static pixelColor_t hsb_to_rgbw(LightState state) {
    int r, g, b, w;
    double cos_h, cos_1047_h;

    state.hue = fmod(state.hue, 360.0); // cycle H around to 0-360 degrees
    state.hue = 3.14159 * state.hue / (double)180.0; // Convert to radians.
    state.saturation /= 100.0;
    state.saturation = state.saturation > 0 ? (state.saturation < 1 ? state.saturation : 1) : 0; // clamp S and I to interval [0,1]
    state.brightness /= 100.0;
    state.brightness = state.brightness > 0 ? (state.brightness < 1 ? state.brightness : 1) : 0;

    if (state.hue < 2.09439) {
        cos_h = cos(state.hue);
        cos_1047_h = cos(1.047196667 - state.hue);
        r = state.saturation * 255.0 * state.brightness / 3.0 * (1.0 + cos_h / cos_1047_h);
        g = state.saturation * 255.0 * state.brightness / 3.0 * (1.0 + (1.0 - cos_h / cos_1047_h));
        b = 0;
        w = 255.0 * exponential_in(1.0 - state.saturation) * state.brightness;
    } else if (state.hue < 4.188787) {
        state.hue = state.hue - 2.09439;
        cos_h = cos(state.hue);
        cos_1047_h = cos(1.047196667 - state.hue);
        g = state.saturation * 255.0 * state.brightness / 3.0 * (1.0 + cos_h / cos_1047_h);
        b = state.saturation * 255.0 * state.brightness / 3.0 * (1.0 + (1.0 - cos_h / cos_1047_h));
        r = 0;
        w = 255 * exponential_in(1.0 - state.saturation) * state.brightness;
    } else {
        state.hue = state.hue - 4.188787;
        cos_h = cos(state.hue);
        cos_1047_h = cos(1.047196667 - state.hue);
        b = state.saturation * 255.0 * state.brightness / 3.0 * (1.0 + cos_h / cos_1047_h);
        r = state.saturation * 255.0 * state.brightness / 3.0 * (1.0 + (1.0 - cos_h / cos_1047_h));
        g = 0;
        w = 255.0 * exponential_in(1.0 - state.saturation) * state.brightness;
    }

    return pixelFromRGBW(r, g, b, w);
}

# if 0
static pixelColor_t hsv_to_rgb(LightState state) {
    int r, g, b;
    double cos_h, cos_1047_h;

    state.hue = fmod(state.hue, 360.0); // cycle H around to 0-360 degrees
    state.hue = 3.14159 * state.hue / (double)180.0; // Convert to radians.
    state.saturation /= 100.0;
    state.saturation = state.saturation > 0 ? (state.saturation < 1 ? state.saturation : 1) : 0; // clamp S and I to interval [0,1]
    state.brightness /= 100.0;
    state.brightness = state.brightness > 0 ? (state.brightness < 1 ? state.brightness : 1) : 0;

    if (state.hue < 2.09439) {
        cos_h = cos(state.hue);
        cos_1047_h = cos(1.047196667 - state.hue);
        r = state.saturation * 255.0 * (1.0 + cos_h / cos_1047_h);
        g = state.saturation * 255.0 * (1.0 + (1.0 - cos_h / cos_1047_h));
        b = 0;
    } else if (state.hue < 4.188787) {
        state.hue = state.hue - 2.09439;
        cos_h = cos(state.hue);
        cos_1047_h = cos(1.047196667 - state.hue);
        g = state.saturation * 255.0 * (1.0 + cos_h / cos_1047_h);
        b = state.saturation * 255.0 * (1.0 + (1.0 - cos_h / cos_1047_h));
        r = 0;
    } else {
        state.hue = state.hue - 4.188787;
        cos_h = cos(state.hue);
        cos_1047_h = cos(1.047196667 - state.hue);
        b = state.saturation * 255.0 * (1.0 + cos_h / cos_1047_h);
        r = state.saturation * 255.0 * (1.0 + (1.0 - cos_h / cos_1047_h));
        g = 0;
    }

    return pixelFromRGB(r, g, b);
}
#endif

static pixelColor_t xyb_to_rgb(float vX, float vY, float vB) {
    double brightness = vB / 100.0;
    if (brightness == 0) {
        return pixelFromRGB(0, 0, 0);
    }

    double Y = brightness;

    if (vY == 0) {
        vY += 0.00000000001;
    }

    double X = (Y / vY) * vX;
    double Z = (Y / vY) * (1.0 - vX - vY);

    // Convert to RGB using Wide RGB D65 conversion.
    double r = X * 1.656492 - Y * 0.354851 - Z * 0.255038;
    double g = -X * 0.707196 + Y * 1.655397 + Z * 0.036152;
    double b = X * 0.051713 - Y * 0.121364 + Z * 1.011530;

    // Apply reverse gamma correction.
    r = (r <= 0.0031308) ? (12.92 * r) : ((1.0 + 0.055) * pow(r, (1.0 / 2.4)) - 0.055); 
    g = (g <= 0.0031308) ? (12.92 * g) : ((1.0 + 0.055) * pow(g, (1.0 / 2.4)) - 0.055); 
    b = (b <= 0.0031308) ? (12.92 * b) : ((1.0 + 0.055) * pow(b, (1.0 / 2.4)) - 0.055); 

    // Bring all negative components to zero.
    r = max(0, r);
    g = max(0, g);
    b = max(0, b);

    // If one component is greater than 1, weight components by that value.
    double max_component = max(r, max(g, b));
    if (max_component > 1.0) {
        r /= max_component;
        g /= max_component;
        b /= max_component;
    }

    return pixelFromRGB(r * 255.0, g * 255.0, b * 255.0);
}

#if 0
typedef struct _XYBColor {
    float x;
    float y;
    float b;
} XYBColor;

static XYBColor rgb_to_xyb(pixelColor_t rgb) {
    XYBColor xyb = {0};

    if (rgb.r + rgb.b + rgb.b == 0) {
        return xyb;
    }
    
    double R = (double)rgb.r / 255.0;
    double B = (double)rgb.b / 255.0;
    double G = (double)rgb.g / 255.0;

    // Gamma correction
    R = (R > 0.04045) ? pow((R + 0.055) / (1.0 + 0.055), 2.4) : (R / 12.92);
    G = (G > 0.04045) ? pow((G + 0.055) / (1.0 + 0.055), 2.4) : (G / 12.92);
    B = (B > 0.04045) ? pow((B + 0.055) / (1.0 + 0.055), 2.4) : (B / 12.92);

    // Wide RGB D65 conversion formula
    double X = R * 0.664511 + G * 0.154324 + B * 0.162028;
    double Y = R * 0.283881 + G * 0.668433 + B * 0.047685;
    double Z = R * 0.000088 + G * 0.072310 + B * 0.986039;

    // Convert XYZ to xy
    xyb.x = round(X / (X + Y + Z) * 1000.0) / 1000.0;
    xyb.y = round(Y / (X + Y + Z) * 1000.0) / 1000.0;

    // Brightness
    xyb.y = round((Y > 1.0) ? 1000.0 : Y * 1000.0) / 1000.0;

    return xyb;
}
#endif

typedef struct _HSVColor {
    float h;
    float s;
    float v;
} HSVColor;

static HSVColor rgb_to_hsv(pixelColor_t color) {
    double delta, m;
    double h = 0, s, v;

    m = min(min(color.r, color.g), color.b);
    v = max(max(color.r, color.g), color.b);
    delta = v - m;

    if (v == 0.0) {
        s = 0;
    } else {
        s = delta / v;
    }

    if (s == 0) {
        h = 0.0;
    } else {
        if (color.r == v) {
            h = (double)(color.g - color.b) / delta;
        } else if (color.g == v) {
            h = 2.0 + (double)(color.b - color.r) / delta;
        } else if (color.b == v) {
            h = 4.0 + (double)(color.r - color.g) / delta;
        }

        h *= 60.0;

        if (h < 0.0) {
            h = h + 360.0;
        }
    }

    HSVColor hsv = {
        .h = h,
        .s = s,
        .v = v / 255.0
    };

    return hsv;
}

void update_light(strand_t *strand, LightState state) {
    if (state.on == false) {
        digitalLeds_resetPixels(strand);
    } else {
        pixelColor_t px = hsb_to_rgbw(state);
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
    .brightness = 100,
    .x = 0,
    .y = 0
};

void update_floodlight() {
    update_light(&STRANDS[1], floodlight);
    printf("floodlight: %s, h = %f, s = %f, b = %f\n",
        floodlight.on ? "on" : "off",
        floodlight.hue,
        floodlight.saturation,
        floodlight.brightness
    );
}

#ifdef ENABLE_HOMEKIT
homekit_value_t get_floodlight_on() {
    return HOMEKIT_BOOL(floodlight.on);
}

void set_floodlight_on(homekit_value_t value) {
    floodlight.on = value.bool_value;
    update_floodlight();
    mqtt_republish(mqtt_client, MQTT_MSG_FLOODLIGHT_SWITCH);
}

homekit_value_t get_floodlight_hue() {
    return HOMEKIT_FLOAT(floodlight.hue);
    return HOMEKIT_FLOAT(floodlight.hue);
}

void set_floodlight_hue(homekit_value_t value) {
    floodlight.hue = value.float_value;
    update_floodlight();
    mqtt_republish(mqtt_client, MQTT_MSG_FLOODLIGHT_HUE);
}

homekit_value_t get_floodlight_saturation() {
    return HOMEKIT_FLOAT(floodlight.saturation);
}

void set_floodlight_saturation(homekit_value_t value) {
    floodlight.saturation = value.float_value;
    update_floodlight();
    mqtt_republish(mqtt_client, MQTT_MSG_FLOODLIGHT_SATURATION);
}

homekit_value_t get_floodlight_brightness() {
    return HOMEKIT_INT(floodlight.brightness);
}

void set_floodlight_brightness(homekit_value_t value) {
    floodlight.brightness = value.int_value;
    floodlight.on = (floodlight.brightness > 0);
    update_floodlight();
    mqtt_republish(mqtt_client, MQTT_MSG_FLOODLIGHT_BRIGHTNESS);
}

#endif /* ENABLE_HOMEKIT */

/*
 * Ambilight
 */

LightState ambilight = {
    .on = true,
    .hue = 0,
    .saturation = 0,
    .brightness = 100,
    .x = 0,
    .y = 0
};

void update_ambilight() {
    update_light(&STRANDS[0], ambilight);
    printf("ambilight: %s, h = %f, s = %f, b = %f\n",
        ambilight.on ? "on" : "off",
        ambilight.hue,
        ambilight.saturation,
        ambilight.brightness
    );

}

#ifdef ENABLE_HOMEKIT

homekit_value_t get_ambilight_on() {
    return HOMEKIT_BOOL(ambilight.on);
}

void set_ambilight_on(homekit_value_t value) {
    ambilight.on = value.bool_value;
    update_ambilight();
    mqtt_republish(mqtt_client, MQTT_MSG_AMBILIGHT_SWITCH);
}

homekit_value_t get_ambilight_hue() {
    return HOMEKIT_FLOAT(ambilight.hue);
}

void set_ambilight_hue(homekit_value_t value) {
    ambilight.hue = value.float_value;
    update_ambilight();
    mqtt_republish(mqtt_client, MQTT_MSG_AMBILIGHT_HUE);
}

homekit_value_t get_ambilight_saturation() {
    return HOMEKIT_FLOAT(ambilight.saturation);
}

void set_ambilight_saturation(homekit_value_t value) {
    ambilight.saturation = value.float_value;
    update_ambilight();
    mqtt_republish(mqtt_client, MQTT_MSG_AMBILIGHT_SATURATION);
}

homekit_value_t get_ambilight_brightness() {
    return HOMEKIT_INT(ambilight.brightness);
}

void set_ambilight_brightness(homekit_value_t value) {
    ambilight.brightness = value.int_value;
    ambilight.on = (ambilight.brightness > 0);
    update_ambilight();
    mqtt_republish(mqtt_client, MQTT_MSG_AMBILIGHT_BRIGHTNESS);
}

#endif /* ENABLE_HOMEKIT */

/*
 * Accessory definition
 */

#ifdef ENABLE_HOMEKIT
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

#endif /* ENABLE_HOMEKIT */

/*
 * MQTT event handler
 */

#ifdef ENABLE_MQTT

static void mqtt_republish(esp_mqtt_client_handle_t client, MQTT_Message typ) {
    char *data = NULL;
    char *endpoint = NULL;
    switch (typ) {
        case MQTT_MSG_AMBILIGHT_HUE:
            asprintf(&data, "%d", (int)ambilight.hue);
            endpoint = MQTT_ID "/ambilight/hue/status";
            break;
        case MQTT_MSG_AMBILIGHT_SATURATION:
            asprintf(&data, "%d", (int)ambilight.saturation);
            endpoint = MQTT_ID "/ambilight/saturation/status";
            break;
        case MQTT_MSG_AMBILIGHT_XY: {
            asprintf(&data, "%0.3f,%0.3f", ambilight.x, ambilight.y);
            endpoint = MQTT_ID "/ambilight/xy/status";
            break;
        }
        case MQTT_MSG_AMBILIGHT_BRIGHTNESS:
            asprintf(&data, "%d", (int)ambilight.brightness);
            endpoint = MQTT_ID "/ambilight/brightness/status";
            break;
        case MQTT_MSG_AMBILIGHT_SWITCH:
            asprintf(&data, "%s", ambilight.on ? "ON" : "OFF");
            endpoint = MQTT_ID "/ambilight/switch/status";
            break;

        case MQTT_MSG_FLOODLIGHT_HUE:
            asprintf(&data, "%d", (int)floodlight.hue);
            endpoint = MQTT_ID "/floodlight/hue/status";
            break;
        case MQTT_MSG_FLOODLIGHT_SATURATION:
            asprintf(&data, "%d", (int)floodlight.saturation);
            endpoint = MQTT_ID "/floodlight/saturation/status";
            break;        
        case MQTT_MSG_FLOODLIGHT_XY: {
            asprintf(&data, "%0.3f,%0.3f", floodlight.x, floodlight.y);
            endpoint = MQTT_ID "/floodlight/xy/status";
            break;
        }
        case MQTT_MSG_FLOODLIGHT_BRIGHTNESS:
            asprintf(&data, "%d", (int)floodlight.brightness);
            endpoint = MQTT_ID "/floodlight/brightness/status";
            break;
        case MQTT_MSG_FLOODLIGHT_SWITCH:
            asprintf(&data, "%s", floodlight.on ? "ON" : "OFF");
            endpoint = MQTT_ID "/floodlight/switch/status";
            break;
    }
    if (data != NULL) {
        int msg_id = esp_mqtt_client_publish(client, endpoint, data, 0, 1, 1);
        ESP_LOGI(TAG, "sent publish successful, msg_id=%d", msg_id);
        free(data);
    }
}

static void update_xy(LightState *state, char *xy) {
    char *yptr = NULL;
    float x = strtof(xy, &yptr);
    float y = strtof(yptr + 1, NULL);

    pixelColor_t rgb = xyb_to_rgb(x, y, state->brightness);
    HSVColor hsv = rgb_to_hsv(rgb);

    state->hue = hsv.h;
    state->saturation = hsv.s * 100.0;
    state->x = x;
    state->y = y;
}

static void mqtt_update(esp_mqtt_client_handle_t client, char *topic, int topic_len, char *data, int data_len) {
    char *converted_data;
    asprintf(&converted_data, "%.*s", data_len, data);

    if (strncmp(topic, MQTT_ID "/ambilight/switch/set", topic_len) == 0) {
        ambilight.on = (strncasecmp(data, "ON", data_len) == 0);
        update_ambilight();
        mqtt_republish(client, MQTT_MSG_AMBILIGHT_SWITCH);
    } else if (strncmp(topic, MQTT_ID "/floodlight/switch/set", topic_len) == 0) {
        floodlight.on = (strncasecmp(data, "ON", data_len) == 0);
        update_floodlight();
        mqtt_republish(client, MQTT_MSG_FLOODLIGHT_SWITCH);
    } else if (strncmp(topic, MQTT_ID "/floodlight/hue/set", topic_len) == 0) {
        floodlight.hue = atof(converted_data);
        update_floodlight();
        mqtt_republish(client, MQTT_MSG_FLOODLIGHT_HUE);
        mqtt_republish(client, MQTT_MSG_FLOODLIGHT_XY);
    } else if (strncmp(topic, MQTT_ID "/floodlight/saturation/set", topic_len) == 0) {
        floodlight.saturation = atof(converted_data);
        update_floodlight();
        mqtt_republish(client, MQTT_MSG_FLOODLIGHT_SATURATION);
        mqtt_republish(client, MQTT_MSG_FLOODLIGHT_XY);
    } else if (strncmp(topic, MQTT_ID "/floodlight/xy/set", topic_len) == 0) {
        update_xy(&floodlight, converted_data);
        update_floodlight();
        mqtt_republish(client, MQTT_MSG_FLOODLIGHT_XY);
        mqtt_republish(client, MQTT_MSG_FLOODLIGHT_HUE);
        mqtt_republish(client, MQTT_MSG_FLOODLIGHT_SATURATION);
    } else if (strncmp(topic, MQTT_ID "/floodlight/brightness/set", topic_len) == 0) {
        floodlight.brightness = atof(converted_data);
        update_floodlight();
        mqtt_republish(client, MQTT_MSG_FLOODLIGHT_BRIGHTNESS);
    } else if (strncmp(topic, MQTT_ID "/ambilight/hue/set", topic_len) == 0) {
        ambilight.hue = atof(converted_data);
        update_ambilight();
        mqtt_republish(client, MQTT_MSG_AMBILIGHT_HUE);
        mqtt_republish(client, MQTT_MSG_AMBILIGHT_XY);
    } else if (strncmp(topic, MQTT_ID "/ambilight/saturation/set", topic_len) == 0) {
        ambilight.saturation = atof(converted_data);
        update_ambilight();
        mqtt_republish(client, MQTT_MSG_AMBILIGHT_SATURATION);
        mqtt_republish(client, MQTT_MSG_AMBILIGHT_XY);
    } else if (strncmp(topic, MQTT_ID "/ambilight/xy/set", topic_len) == 0) {
        update_xy(&ambilight, converted_data);
        update_ambilight();
        mqtt_republish(client, MQTT_MSG_AMBILIGHT_XY);
        mqtt_republish(client, MQTT_MSG_AMBILIGHT_HUE);
        mqtt_republish(client, MQTT_MSG_AMBILIGHT_SATURATION);
    } else if (strncmp(topic, MQTT_ID "/ambilight/brightness/set", topic_len) == 0) {
        ambilight.brightness = atof(converted_data);
        update_ambilight();
        mqtt_republish(client, MQTT_MSG_AMBILIGHT_BRIGHTNESS);
    }
    free(converted_data);
}

static esp_err_t mqtt_event_handler(esp_mqtt_event_handle_t event) {
    esp_mqtt_client_handle_t client = event->client;
    static int msgs[NUM_MQTT_MSG] = {0};
    // your_context_t *context = event->context;
    switch (event->event_id) {
        case MQTT_EVENT_CONNECTED:
            ESP_LOGI(TAG, "MQTT_EVENT_CONNECTED");
            msgs[MQTT_MSG_FLOODLIGHT_HUE] = esp_mqtt_client_subscribe(client, MQTT_ID "/floodlight/hue/set", 1);
            vTaskDelay(100 / portTICK_PERIOD_MS);
            msgs[MQTT_MSG_FLOODLIGHT_SATURATION] = esp_mqtt_client_subscribe(client, MQTT_ID "/floodlight/saturation/set", 1);
            vTaskDelay(100 / portTICK_PERIOD_MS);
            msgs[MQTT_MSG_FLOODLIGHT_BRIGHTNESS] = esp_mqtt_client_subscribe(client, MQTT_ID "/floodlight/brightness/set", 1);
            vTaskDelay(100 / portTICK_PERIOD_MS);
            msgs[MQTT_MSG_FLOODLIGHT_XY] = esp_mqtt_client_subscribe(client, MQTT_ID "/floodlight/xy/set", 1);
            vTaskDelay(100 / portTICK_PERIOD_MS);
            msgs[MQTT_MSG_FLOODLIGHT_SWITCH] = esp_mqtt_client_subscribe(client, MQTT_ID "/floodlight/switch/set", 1);
            vTaskDelay(100 / portTICK_PERIOD_MS);
            msgs[MQTT_MSG_AMBILIGHT_HUE] = esp_mqtt_client_subscribe(client, MQTT_ID "/ambilight/hue/set", 1);
            vTaskDelay(100 / portTICK_PERIOD_MS);
            msgs[MQTT_MSG_AMBILIGHT_SATURATION] = esp_mqtt_client_subscribe(client, MQTT_ID "/ambilight/saturation/set", 1);
            vTaskDelay(100 / portTICK_PERIOD_MS);
            msgs[MQTT_MSG_AMBILIGHT_BRIGHTNESS] = esp_mqtt_client_subscribe(client, MQTT_ID "/ambilight/brightness/set", 1);
            vTaskDelay(100 / portTICK_PERIOD_MS);
            msgs[MQTT_MSG_AMBILIGHT_XY] = esp_mqtt_client_subscribe(client, MQTT_ID "/ambilight/xy/set", 1);
            vTaskDelay(100 / portTICK_PERIOD_MS);
            msgs[MQTT_MSG_AMBILIGHT_SWITCH] = esp_mqtt_client_subscribe(client, MQTT_ID "/ambilight/switch/set", 1);
            vTaskDelay(100 / portTICK_PERIOD_MS);
            break;
        case MQTT_EVENT_DISCONNECTED:
            ESP_LOGI(TAG, "MQTT_EVENT_DISCONNECTED");
            break;

        case MQTT_EVENT_SUBSCRIBED: {
            ESP_LOGI(TAG, "MQTT_EVENT_SUBSCRIBED, msg_id=%d", event->msg_id);
            int typ = -1;
            for (int i = 0; i < NUM_MQTT_MSG; i++) {
                if (event->msg_id == msgs[i]) {
                    typ = i;
                    break;
                }
            }
            mqtt_republish(client, typ);
            break;
        }
        case MQTT_EVENT_UNSUBSCRIBED:
            ESP_LOGI(TAG, "MQTT_EVENT_UNSUBSCRIBED, msg_id=%d", event->msg_id);
            break;
        case MQTT_EVENT_PUBLISHED:
            ESP_LOGI(TAG, "MQTT_EVENT_PUBLISHED, msg_id=%d", event->msg_id);
            break;
        case MQTT_EVENT_DATA:
            ESP_LOGI(TAG, "MQTT_EVENT_DATA");
            mqtt_update(client, event->topic, event->topic_len, event->data, event->data_len);
            break;
        case MQTT_EVENT_ERROR:
            ESP_LOGI(TAG, "MQTT_EVENT_ERROR");
            break;
    }
    return ESP_OK;
}

#endif /* ENABLE_MQTT */

/*
 * APP Main
 */

void on_wifi_ready() {
#ifdef ENABLE_HOMEKIT
    homekit_server_init(&config);
#endif /* ENABLE_HOMEKIT */

#ifdef ENABLE_MQTT
    mqtt_client = esp_mqtt_client_init(&mqtt_cfg);
    esp_mqtt_client_start(mqtt_client);
#endif /* ENABLE_MQTT */
}

void app_main(void) {
    gpioSetup(12, OUTPUT, LOW);
    gpioSetup(13, OUTPUT, LOW);

    if (digitalLeds_initStrands(STRANDS, STRANDCNT)) {
        printf("Init FAILURE: halting\n");
        while (true) {};
    }

    update_ambilight();
    update_floodlight();

    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK( ret );

    wifi_init();
#ifdef ENABLE_HOMEKIT
    led_init();
#endif /* ENABLE_HOMEKIT */
}

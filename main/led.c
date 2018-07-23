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
#include "mqtt_client.h"

static const char *TAG = "MQTT_CLIENT";

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
 * MQTT setup
 */

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
    MQTT_MSG_AMBILIGHT_RGB         = 0,
    MQTT_MSG_AMBILIGHT_BRIGHTNESS  = 1,
    MQTT_MSG_AMBILIGHT_SWITCH      = 2,
    MQTT_MSG_FLOODLIGHT_RGB        = 3,
    MQTT_MSG_FLOODLIGHT_BRIGHTNESS = 4,
    MQTT_MSG_FLOODLIGHT_SWITCH     = 5
} MQTT_Message;

#define NUM_MQTT_MSG 6

esp_mqtt_client_handle_t mqtt_client;

static void mqtt_republish(esp_mqtt_client_handle_t client, MQTT_Message typ);

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
} LightState;

#include "math.h"
#define DEG_TO_RAD(X) (M_PI*(X)/180)

float exponential_in(float percentage) {
    return pow(2, 10 * (percentage - 1.0));
}

pixelColor_t calculate_color(LightState state) {
    int r, g, b, w;
    float cos_h, cos_1047_h;

    state.hue = fmod(state.hue, 360.0); // cycle H around to 0-360 degrees
    state.hue = 3.14159 * state.hue / (float)180.0; // Convert to radians.
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

    printf("r = %d, g = %d, b = %d, w = %d\n", r,g,b,w);
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
    printf("floodlight: %s, h = %f, s = %f, b = %f\n",
        floodlight.on ? "on" : "off",
        floodlight.hue,
        floodlight.saturation,
        floodlight.brightness
    );
}

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
}

void set_floodlight_hue(homekit_value_t value) {
    floodlight.hue = value.float_value;
    update_floodlight();
    mqtt_republish(mqtt_client, MQTT_MSG_FLOODLIGHT_RGB);
}

homekit_value_t get_floodlight_saturation() {
    return HOMEKIT_FLOAT(floodlight.saturation);
}

void set_floodlight_saturation(homekit_value_t value) {
    floodlight.saturation = value.float_value;
    update_floodlight();
    mqtt_republish(mqtt_client, MQTT_MSG_FLOODLIGHT_RGB);
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
    printf("ambilight: %s, h = %f, s = %f, b = %f\n",
        ambilight.on ? "on" : "off",
        ambilight.hue,
        ambilight.saturation,
        ambilight.brightness
    );

}

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
    mqtt_republish(mqtt_client, MQTT_MSG_AMBILIGHT_RGB);
}

homekit_value_t get_ambilight_saturation() {
    return HOMEKIT_FLOAT(ambilight.saturation);
}

void set_ambilight_saturation(homekit_value_t value) {
    ambilight.saturation = value.float_value;
    update_ambilight();
    mqtt_republish(mqtt_client, MQTT_MSG_AMBILIGHT_RGB);
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

/*
 * MQTT event handler
 */


static char *get_rgb_string(LightState state) {
    char *data;
    pixelColor_t px = calculate_color(state);
    asprintf(&data, "%d, %d, %d", px.r, px.g, px.b);
    return data;
}

static void mqtt_republish(esp_mqtt_client_handle_t client, MQTT_Message typ) {
    char *data = NULL;
    char *endpoint = NULL;
    switch (typ) {
        case MQTT_MSG_AMBILIGHT_RGB:
            data = get_rgb_string(ambilight);
            endpoint = MQTT_ID "/ambilight/rgb/status";
            break;
        case MQTT_MSG_AMBILIGHT_BRIGHTNESS:
            asprintf(&data, "%d", (int)ambilight.brightness);
            endpoint = MQTT_ID "/ambilight/brightness/status";
            break;
        case MQTT_MSG_AMBILIGHT_SWITCH:
            asprintf(&data, "%s", ambilight.on ? "ON" : "OFF");
            endpoint = MQTT_ID "/ambilight/switch/status";
            break;

        case MQTT_MSG_FLOODLIGHT_RGB:
            data = get_rgb_string(floodlight);
            endpoint = MQTT_ID "/floodlight/rgb/status";
            break;
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

static void mqtt_update(char *topic, int topic_len, char *data, int data_len) {
    if (strncmp(topic, MQTT_ID "/ambilight/switch/set", topic_len) == 0) {
        ambilight.on = (strncasecmp(data, "ON", data_len) == 0);
        update_ambilight();
    }
    if (strncmp(topic, MQTT_ID "/floodlight/switch/set", topic_len) == 0) {
        floodlight.on = (strncasecmp(data, "ON", data_len) == 0);
        update_floodlight();
    }
}

static esp_err_t mqtt_event_handler(esp_mqtt_event_handle_t event) {
    esp_mqtt_client_handle_t client = event->client;
    static int msgs[NUM_MQTT_MSG] = {0};
    // your_context_t *context = event->context;
    switch (event->event_id) {
        case MQTT_EVENT_CONNECTED:
            ESP_LOGI(TAG, "MQTT_EVENT_CONNECTED");
            msgs[MQTT_MSG_FLOODLIGHT_RGB] = esp_mqtt_client_subscribe(client, MQTT_ID "/floodlight/rgb/set", 1);
            vTaskDelay(100 / portTICK_PERIOD_MS);
            msgs[MQTT_MSG_FLOODLIGHT_BRIGHTNESS] = esp_mqtt_client_subscribe(client, MQTT_ID "/floodlight/brightness/set", 1);
            vTaskDelay(100 / portTICK_PERIOD_MS);
            msgs[MQTT_MSG_FLOODLIGHT_SWITCH] = esp_mqtt_client_subscribe(client, MQTT_ID "/floodlight/switch/set", 1);
            vTaskDelay(100 / portTICK_PERIOD_MS);
            msgs[MQTT_MSG_AMBILIGHT_RGB] = esp_mqtt_client_subscribe(client, MQTT_ID "/ambilight/rgb/set", 1);
            vTaskDelay(100 / portTICK_PERIOD_MS);
            msgs[MQTT_MSG_AMBILIGHT_BRIGHTNESS] = esp_mqtt_client_subscribe(client, MQTT_ID "/ambilight/brightness/set", 1);
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
            printf("TOPIC=%.*s\r\n", event->topic_len, event->topic);
            printf("DATA=%.*s\r\n", event->data_len, event->data);
            mqtt_update(event->topic, event->topic_len, event->data, event->data_len);
            break;
        case MQTT_EVENT_ERROR:
            ESP_LOGI(TAG, "MQTT_EVENT_ERROR");
            break;
    }
    return ESP_OK;
}

/*
 * APP Main
 */

void on_wifi_ready() {
    homekit_server_init(&config);

    mqtt_client = esp_mqtt_client_init(&mqtt_cfg);
    esp_mqtt_client_start(mqtt_client);
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
    led_init();
}

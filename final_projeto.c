/* -------------------------------------------------------------------------------------------------------------------------------------
/ Projeto: Painéis de Comunicação alternativa com sensores ambientais (MQTT + DNS + SENSORS)
/ Autor: Moisés Lourenço
/ Data: 04/02/2026
/----------------------------------------------------------------------------------------------------------------------------------------
*/

#include <string.h>
#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "lwip/dns.h"
#include "lwip/apps/mqtt.h"

// FreeRTOS
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

// Sensores e Hardware
#include "hardware/pwm.h"
#include "hardware/i2c.h"
#include "aht10.h"
#include "ssd1306.h"
#include "bh1750.h"

// ================= CONFIGURAÇÕES =================
#define WIFI_SSID       "Daniel"
#define WIFI_PASSWORD   "Samuel02"
#define MQTT_BROKER     "broker.emqx.io"
#ifndef MQTT_PORT
#define MQTT_PORT 1883
#endif
#define CLIENT_ID       "pico_moises_001"

#define I2C_PORT0 i2c0
#define I2C_SDA0 0
#define I2C_SCL0 1
#define I2C_PORT1 i2c1
#define I2C_SDA1 14
#define I2C_SCL1 15

#define BUZZER_PIN 10
#define BUTTON1_PIN 6  
#define BUTTON2_PIN 5  
#define BUTTON3_PIN 19 
#define BUTTON4_PIN 20 

// ====== VARIÁVEIS GLOBAIS E ESTRUTURAS ======
typedef struct {
    float temperatura;
    float umidade;
    float lux;
} sensor_data_t;

typedef enum {
    EVENT_NONE, EVENT_FOOD, EVENT_WATER, EVENT_BATHROOM, EVENT_PLAY
} button_event_t;

QueueHandle_t xSensorQueue;
QueueHandle_t xButtonEventQueue;
static sensor_data_t last_readings = {0}; // Armazena última leitura para o MQTT

static mqtt_client_t* mqtt_client;
static ip_addr_t broker_ip;

// --- Protótipos ---
void task_wifi(void *pvParameters);
void task_sensors(void *pvParameters);
void task_display(void *pvParameters);
void task_button_processor(void *pvParameters);
void init_hardware();
int i2c_write(uint8_t addr, const uint8_t *data, uint16_t len);
int i2c_read(uint8_t addr, uint8_t *data, uint16_t len);
void delay_ms(uint32_t ms);



/* --- FUNÇÕES DE MEMÓRIA ESTÁTICA PARA FREERTOS --- */
void vApplicationGetIdleTaskMemory(StaticTask_t **ppxIdleTaskTCBBuffer, 
                                   StackType_t **ppxIdleTaskStackBuffer, 
                                   uint32_t *pulIdleTaskStackSize) {
    static StaticTask_t xIdleTaskTCB;
    static StackType_t uxIdleTaskStack[configMINIMAL_STACK_SIZE];
    *ppxIdleTaskTCBBuffer = &xIdleTaskTCB;
    *ppxIdleTaskStackBuffer = uxIdleTaskStack;
    *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
}

void vApplicationGetTimerTaskMemory(StaticTask_t **ppxTimerTaskTCBBuffer, 
                                    StackType_t **ppxTimerTaskStackBuffer, 
                                    uint32_t *pulTimerTaskStackSize) {
    static StaticTask_t xTimerTaskTCB;
    static StackType_t uxTimerTaskStack[configTIMER_TASK_STACK_DEPTH];
    *ppxTimerTaskTCBBuffer = &xTimerTaskTCB;
    *ppxTimerTaskStackBuffer = uxTimerTaskStack;
    *pulTimerTaskStackSize = configTIMER_TASK_STACK_DEPTH;
}

// ================= CALLBACKS MQTT =================
static void mqtt_connection_cb(mqtt_client_t *client, void *arg, mqtt_connection_status_t status) {
    if (status == MQTT_CONNECT_ACCEPTED) {
        printf("MQTT: Conectado ao Broker!\n");
    } else {
        printf("MQTT: Falha na conexão (%d)\n", status);
    }
}

void publish_message(const char *topic, const char *payload) {
    if (mqtt_client && mqtt_client_is_connected(mqtt_client)) {
        err_t err = mqtt_publish(mqtt_client, topic, payload, strlen(payload), 0, 0, NULL, NULL);
        if (err == ERR_OK) printf("Publicado [%s]: %s\n", topic, payload);
    } else {
        printf("MQTT: Não conectado. Falha ao publicar.\n");
    }
}

// ================= TAREFA WI-FI E MQTT =================
void task_wifi(void *pvParameters) {
    // 1. Inicializa Wi-Fi
    if (cyw43_arch_init_with_country(CYW43_COUNTRY_BRAZIL)) {
        printf("Erro ao iniciar hardware Wi-Fi\n");
        vTaskDelete(NULL);
    }
    cyw43_arch_enable_sta_mode();

    // 2. Conecta ao Ponto de Acesso
    printf("Conectando ao Wi-Fi: %s...\n", WIFI_SSID);
    while (cyw43_arch_wifi_connect_timeout_ms(WIFI_SSID, WIFI_PASSWORD, CYW43_AUTH_WPA2_AES_PSK, 30000) != 0) {
        printf("Aguardando Wi-Fi...\n");
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
    printf("Wi-Fi Conectado!\n");

    // 3. Resolve o DNS do Broker
    printf("Resolvendo IP do Broker: %s\n", MQTT_BROKER);
    broker_ip.addr = 0; // Reseta o IP
    while (broker_ip.addr == 0) {
        err_t err = dns_gethostbyname(MQTT_BROKER, &broker_ip, NULL, NULL);
        if (err == ERR_OK) break; // Resolvido imediatamente (cache)
        if (err == ERR_INPROGRESS) {
            // Aguarda o callback do sistema (em segundo plano)
            vTaskDelay(pdMS_TO_TICKS(1000));
        } else {
            printf("Erro no DNS. Tentando novamente...\n");
            vTaskDelay(pdMS_TO_TICKS(2000));
        }
    }
    printf("IP Resolvido: %s\n", ipaddr_ntoa(&broker_ip));

    // 4. Inicia o Cliente MQTT
    mqtt_client = mqtt_client_new();
    struct mqtt_connect_client_info_t ci = {
        .client_id = CLIENT_ID,
        .keep_alive = 60
    };

    printf("Iniciando conexão MQTT...\n");
    mqtt_client_connect(mqtt_client, &broker_ip, MQTT_PORT, mqtt_connection_cb, NULL, &ci);

    // Loop de monitoramento (Blink LED)
    while (true) {
        cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);
        vTaskDelay(pdMS_TO_TICKS(100));
        cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 0);
        vTaskDelay(pdMS_TO_TICKS(3000)); // Pisca a cada 3s se estiver rodando
    }
}
// ================= TAREFA BOTÕES (COM DADOS SENSORES) =================
void task_button_processor(void *pvParameters) {
    button_event_t event;
    char payload[50];
     char payload1[50];
      char payload2[50];
    while (true) {
        if (xQueueReceive(xButtonEventQueue, &event, portMAX_DELAY)) {
            pwm_set_gpio_level(BUZZER_PIN, 2048); 

          snprintf(payload, sizeof(payload), "%.1f", last_readings.temperatura);
          snprintf(payload1, sizeof(payload1), "%.1f", last_readings.umidade);
          snprintf(payload2, sizeof(payload2), "%.1f", last_readings.lux);
            switch (event) {
                case EVENT_FOOD:     
                    publish_message("quarto/beber", "beber");
                    publish_message("quarto/temperatura", payload);
                    publish_message("quarto/umidade", payload1);
                    publish_message("quarto/lux", payload2);
                    break;
                case EVENT_WATER:
                    publish_message("quarto/comer", "comer");
                    publish_message("quarto/temperatura", payload);
                    publish_message("quarto/umidade", payload1);
                    publish_message("quarto/lux", payload2);
                    break;
                case EVENT_BATHROOM: 
                    publish_message("quarto/brincar", "brincar");
                   publish_message("quarto/temperatura", payload);
                    publish_message("quarto/umidade", payload1);
                    publish_message("quarto/lux", payload2);
                    break;
                case EVENT_PLAY:
                    publish_message("quarto/dormir", "dormir");
                    publish_message("quarto/temperatura", payload);
                    publish_message("quarto/umidade", payload1);
                    publish_message("quarto/lux", payload2);
                    break;
                default: break;
            }

            vTaskDelay(pdMS_TO_TICKS(500));
            pwm_set_gpio_level(BUZZER_PIN, 0);
        }
    }
}

// ================= TAREFA SENSORES =================
void task_sensors(void *pvParameters) {
    AHT10_Handle aht10 = { .iface = {i2c_write, i2c_read, delay_ms} };
    AHT10_Init(&aht10);
    bh1750_init(I2C_PORT0);

    while (true) {
        sensor_data_t data;
        if (AHT10_ReadTemperatureHumidity(&aht10, &data.temperatura, &data.umidade)) {
            data.lux = bh1750_read_lux(I2C_PORT0);
            last_readings = data; // Atualiza global para o MQTT
            xQueueSend(xSensorQueue, &data, 0);
        }
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}

// ================= TAREFA DISPLAY =================
void task_display(void *pvParameters) {
    sensor_data_t data;
    char str[20];
    while (true) {
        if (xQueueReceive(xSensorQueue, &data, portMAX_DELAY)) {
            ssd1306_clear();
            ssd1306_draw_string(32, 0, "MONITOR");
            snprintf(str, sizeof(str), "T: %.1f C", data.temperatura); ssd1306_draw_string(0, 16, str);
            snprintf(str, sizeof(str), "U: %.1f %%", data.umidade); ssd1306_draw_string(0, 32, str);
            snprintf(str, sizeof(str), "L: %.1f lx", data.lux); ssd1306_draw_string(0, 48, str);
            ssd1306_show();
        }
    }
}

// ================= INTERRUPÇÃO E HARDWARE =================
void gpio_callback(uint gpio, uint32_t events) {
    static uint32_t last_time = 0;
    uint32_t now = to_ms_since_boot(get_absolute_time());
    if (now - last_time < 250) return;
    last_time = now;

    button_event_t ev = (gpio == BUTTON1_PIN) ? EVENT_FOOD :
                        (gpio == BUTTON2_PIN) ? EVENT_WATER :
                        (gpio == BUTTON3_PIN) ? EVENT_BATHROOM :
                        (gpio == BUTTON4_PIN) ? EVENT_PLAY : EVENT_NONE;

    if (ev != EVENT_NONE) {
        xQueueSendFromISR(xButtonEventQueue, &ev, NULL);
    }
}

void init_hardware() {
    // I2C 1 - Display
    i2c_init(I2C_PORT1, 400000);
    gpio_set_function(I2C_SDA1, GPIO_FUNC_I2C); 
    gpio_set_function(I2C_SCL1, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA1); 
    gpio_pull_up(I2C_SCL1);

    // I2C 0 - Sensores
    i2c_init(I2C_PORT0, 100000);
    gpio_set_function(I2C_SDA0, GPIO_FUNC_I2C); 
    gpio_set_function(I2C_SCL0, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA0); 
    gpio_pull_up(I2C_SCL0);

    // PWM Buzzer
    gpio_set_function(BUZZER_PIN, GPIO_FUNC_PWM);
    uint slice_num = pwm_gpio_to_slice_num(BUZZER_PIN);
    pwm_config config = pwm_get_default_config(); 
    pwm_init(slice_num, &config, true);
    pwm_set_gpio_level(BUZZER_PIN, 0);

    // BOTÕES - CORRIGIDO AQUI
    uint btns[] = {BUTTON1_PIN, BUTTON2_PIN, BUTTON3_PIN, BUTTON4_PIN};
    for(int i = 0; i < 4; i++) {
        gpio_init(btns[i]); 
        gpio_set_dir(btns[i], GPIO_IN); // Garante que é entrada
        gpio_pull_up(btns[i]);
        // Note: Apenas a variável btns[i] correta agora:
        gpio_set_irq_enabled_with_callback(btns[i], GPIO_IRQ_EDGE_FALL, true, &gpio_callback);
    }
}
int main() {
    stdio_init_all();
    init_hardware();
    ssd1306_init(I2C_PORT1);

    xSensorQueue = xQueueCreate(5, sizeof(sensor_data_t));
    xButtonEventQueue = xQueueCreate(10, sizeof(button_event_t));

    xTaskCreate(task_wifi, "WiFi", 2048, NULL, 1, NULL);
    xTaskCreate(task_sensors, "Sensors", 1024, NULL, 2, NULL);
    xTaskCreate(task_display, "Display", 1024, NULL, 1, NULL);
    xTaskCreate(task_button_processor, "Buttons", 2048, NULL, 3, NULL);

    vTaskStartScheduler();
    while(1);
}

// Implementações auxiliares I2C
int i2c_write(uint8_t addr, const uint8_t *data, uint16_t len) { return i2c_write_blocking(I2C_PORT0, addr, data, len, false) < 0 ? -1 : 0; }
int i2c_read(uint8_t addr, uint8_t *data, uint16_t len) { return i2c_read_blocking(I2C_PORT0, addr, data, len, false) < 0 ? -1 : 0; }
void delay_ms(uint32_t ms) { vTaskDelay(pdMS_TO_TICKS(ms)); }
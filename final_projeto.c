/* -------------------------------------------------------------------------------------------------------------------------------------
/ Projeto: Paineis de Comunicação alternativa com sensores ambientais.
/ Descrição: Este código lê a temperatura e umidade do sensor AHT10 e exibe os dados em um display OLED SSD1306 e
/ uma mensagem de alerta quando a umidade chega a 70% ou a temperatura fica abaixo de 20 °C.
/ Bibliotecas: Aht10, Ssd1306
/ Autor: MOisés Lourenço
/ Data de Criação: 04/02/2026
/----------------------------------------------------------------------------------------------------------------------------------------
*/

// Wi-Fi Pico W
#include "lwip/dns.h"
#include "lwip/netdb.h"
#include "lwip/ip_addr.h"
#include "pico/cyw43_arch.h"
#include "lwipopts.h"
#include "lwip/apps/mqtt.h"


// FreeRTOS
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "event_groups.h"

// Raspberry Pi Pico
#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "hardware/clocks.h"
#include "hardware/i2c.h"

// sensores e display
#include "aht10.h"
#include "ssd1306.h"
#include "bh1750.h"

// I2C usado: I2C0 SENSOR AHT10 E SENSOR BH1750
#define I2C_PORT0 i2c0
#define I2C_SDA0 0
#define I2C_SCL0 1

#define I2C_PORT1 i2c1
#define I2C_SDA1 14
#define I2C_SCL1 15

// --- Novas Definições ---
#define BUZZER_PIN 10
#define BUTTON1_PIN 6  // Comida
#define BUTTON2_PIN 5  // Água
#define BUTTON3_PIN 19  // Banheiro
#define BUTTON4_PIN 20  // Brincar


// ================= CONFIGURAÇÕES =================
#define WIFI_SSID       "WI-FI_P4"
#define WIFI_PASSWORD   "thais_p4"

// Configurações do Broker
#define MQTT_BROKER "broker.emqx.io"
#ifdef MQTT_PORT
#undef MQTT_PORT
#endif
#define MQTT_PORT 1883
#define CLIENT_ID "mqttx_ad0e1b87"


// ====== FreeRTOS Static Memory ======
void vApplicationGetIdleTaskMemory(StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize) {
    static StaticTask_t xIdleTaskTCB;
    static StackType_t uxIdleTaskStack[configMINIMAL_STACK_SIZE];
    *ppxIdleTaskTCBBuffer = &xIdleTaskTCB; *ppxIdleTaskStackBuffer = uxIdleTaskStack; *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
}
void vApplicationGetTimerTaskMemory(StaticTask_t **ppxTimerTaskTCBBuffer, StackType_t **ppxTimerTaskStackBuffer, uint32_t *pulTimerTaskStackSize) {
    static StaticTask_t xTimerTaskTCB;
    static StackType_t uxTimerTaskStack[configTIMER_TASK_STACK_DEPTH];
    *ppxTimerTaskTCBBuffer = &xTimerTaskTCB; *ppxTimerTaskStackBuffer = uxTimerTaskStack; *pulTimerTaskStackSize = configTIMER_TASK_STACK_DEPTH;
}

// Estrutura para passar dados entre tarefas
typedef struct {
    float temperatura;
    float umidade;
    float lux;
} sensor_data_t;

// Enumeração para identificar os eventos dos botões
typedef enum {
    EVENT_NONE,
    EVENT_FOOD,
    EVENT_WATER,
    EVENT_BATHROOM,
    EVENT_PLAY
} button_event_t;

// Handle da Fila
QueueHandle_t xSensorQueue;
QueueHandle_t xButtonEventQueue;

// --- Protótipos ---
void task_button_processor(void *pvParameters);
void gpio_callback(uint gpio, uint32_t events);
void init_hardware_buttons();
void task_wifi(void *pvParameters);
void task_sensors(void *pvParameters);
void task_display(void *pvParameters);
int i2c_write(uint8_t addr, const uint8_t *data, uint16_t len);
int i2c_read(uint8_t addr, uint8_t *data, uint16_t len);
void delay_ms(uint32_t ms);

// Estrutura do cliente MQTT
static mqtt_client_t *mqtt_client;
static ip_addr_t broker_ip;

// Callback de conexão
static void mqtt_connection_cb(mqtt_client_t *client, void *arg, mqtt_connection_status_t status) {
    if (status == MQTT_CONNECT_ACCEPTED) {
        printf("MQTT: Conectado com sucesso!\n");
    } else {
        printf("MQTT: Falha na conexão (status %d). Tentando novamente...\n", status);
    }
}

// Função para publicar mensagem
void publish_message(const char *topic, const char *payload) {
    if (mqtt_client_is_connected(mqtt_client)) {
        err_t err = mqtt_publish(mqtt_client, topic, payload, strlen(payload), 0, 0, NULL, NULL);
        if (err == ERR_OK) {
            printf("MQTT: Publicado em [%s]: %s\n", topic, payload);
        }
    } else {
        printf("MQTT: Erro - Cliente não conectado\n");
    }
}

int main() {
    stdio_init_all();

    // Inicializa I2C OLED
    i2c_init(I2C_PORT1, 400 * 1000); // 100 kHz
    gpio_set_function(I2C_SDA1, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL1, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA1);
    gpio_pull_up(I2C_SCL1); 

    // Inicializa I2C  AHT10
    i2c_init(I2C_PORT0, 100000);
    gpio_set_function(I2C_SDA0, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL0, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA0);
    gpio_pull_up(I2C_SCL0);

     // I2C BH1750
    i2c_init(I2C_PORT0, 100 * 1000);
    gpio_set_function(I2C_SDA0, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL0, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA0);
    gpio_pull_up(I2C_SCL0);
    bh1750_init(I2C_PORT0);
    sleep_ms(200);

    sleep_ms(100);  // Aguarda estabilização do I2C
    ssd1306_init(I2C_PORT1);
    sleep_ms(100);  // Aguarda inicialização do display
    ssd1306_clear();
    ssd1306_draw_string(32, 0, "Sensor AHT10");
    ssd1306_show();


    xButtonEventQueue = xQueueCreate(10, sizeof(button_event_t));

    if (xButtonEventQueue != NULL) {
         init_hardware_buttons();
    
    // Tarefa que processará os botões e futuramente enviará MQTT
         xTaskCreate(task_button_processor, "Button_Task", 2048, NULL, 3, NULL);
    }

    // Criação da Fila (Suporta 5 medições)
    xSensorQueue = xQueueCreate(5, sizeof(sensor_data_t));

    if (xSensorQueue != NULL) {
        // Tarefas
        xTaskCreate(task_wifi, "WiFi_Task", 1024, NULL, 1, NULL);
        xTaskCreate(task_sensors, "Sensors_Task", 1024, NULL, 2, NULL);
        xTaskCreate(task_display, "Display_Task", 1024, NULL, 1, NULL);

        vTaskStartScheduler();
    }

    // Define estrutura do sensor
    AHT10_Handle aht10 = {
        .iface = {
            .i2c_write = i2c_write,
            .i2c_read = i2c_read,
            .delay_ms = delay_ms
        }
    };

  while(1) {}

}

/// ================= TAREFAS =================

void task_sensors(void *pvParameters) {
    bh1750_init(I2C_PORT0);
    
    AHT10_Handle aht10 = {
        .iface = { .i2c_write = i2c_write, .i2c_read = i2c_read, .delay_ms = delay_ms }
    };

    if (!AHT10_Init(&aht10)) {
        printf("Erro AHT10\n");
        vTaskDelete(NULL);
    }

    sensor_data_t data;
    while (true) {
        if (AHT10_ReadTemperatureHumidity(&aht10, &data.temperatura, &data.umidade)) {
            data.lux = bh1750_read_lux(I2C_PORT0);
            // Envia os dados para a fila
            xQueueSend(xSensorQueue, &data, portMAX_DELAY);
        }
        vTaskDelay(pdMS_TO_TICKS(2000)); // Lê a cada 2 segundos
    }
}

void task_display(void *pvParameters) {
    ssd1306_init(I2C_PORT1);
    sensor_data_t received_data;
    char str[16];

    while (true) {
        // Espera dados da fila
        if (xQueueReceive(xSensorQueue, &received_data, portMAX_DELAY)) {
            ssd1306_clear();
            ssd1306_draw_string(32, 0, "MONITOR");
            
            snprintf(str, sizeof(str), "Temp: %.1f C", received_data.temperatura);
            ssd1306_draw_string(0, 16, str);
            
            snprintf(str, sizeof(str), "Umi: %.1f %%", received_data.umidade);
            ssd1306_draw_string(0, 32, str);
            
            snprintf(str, sizeof(str), "Luz: %.1f lx", received_data.lux);
            ssd1306_draw_string(0, 48, str);
            
            ssd1306_show();
        }
    }
}

void task_wifi(void *pvParameters) {
    if (cyw43_arch_init()) {
        vTaskDelete(NULL);
    }
    cyw43_arch_enable_sta_mode();
    
    printf("Conectando ao Wi-Fi...\n");
    if (cyw43_arch_wifi_connect_timeout_ms(WIFI_SSID, WIFI_PASSWORD, CYW43_AUTH_WPA2_AES_PSK, 30000) == 0) {
        printf("Wi-Fi OK!\n");

        // Inicializa cliente MQTT
        mqtt_client = mqtt_client_new();
        
        // Resolve o IP do Broker DNS
        printf("Resolvendo IP do Broker...\n");
        err_t err = dns_gethostbyname(MQTT_BROKER, &broker_ip, NULL, NULL);
        
        // Nota: Em um código de produção, você usaria um callback para o DNS. 
        // Aqui, forçamos uma espera curta para simplificar.
        vTaskDelay(pdMS_TO_TICKS(2000)); 

        struct mqtt_connect_client_info_t ci;
        memset(&ci, 0, sizeof(ci));
        ci.client_id = CLIENT_ID;
        ci.keep_alive = 60;

        mqtt_client_connect(mqtt_client, &broker_ip, MQTT_PORT, mqtt_connection_cb, NULL, &ci);
    }

    while (true) {
        cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);
        vTaskDelay(pdMS_TO_TICKS(100));
        cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 0);
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}
// ================= TAREFA DE PROCESSAMENTO =================

void task_button_processor(void *pvParameters) {
    button_event_t event;
    
    while (true) {
        if (xQueueReceive(xButtonEventQueue, &event, portMAX_DELAY)) {
            
            pwm_set_gpio_level(BUZZER_PIN, 2048); 
            
            switch (event) {
                case EVENT_FOOD:
                    publish_message("quarto/comer", "Solicitação: Comida");
                    break;
                case EVENT_WATER:
                    publish_message("quarto/beber", "Solicitação: Agua");
                    break;
                case EVENT_BATHROOM:
                    publish_message("quarto/dormir", "Solicitação: Banheiro/Dormir");
                    break;
                case EVENT_PLAY:
                    publish_message("quarto/brincar", "Solicitação: Brincar");
                    break;
                default:
                    break;
            }

            vTaskDelay(pdMS_TO_TICKS(500));
            pwm_set_gpio_level(BUZZER_PIN, 0);
        }
    }
}

// ================= TRATAMENTO DE INTERRUPÇÃO (ISR) =================

void gpio_callback(uint gpio, uint32_t events) {
    static uint32_t last_interrupt_time = 0;
    uint32_t current_time = to_ms_since_boot(get_absolute_time());
    
    // Debounce de 250ms
    if (current_time - last_interrupt_time < 250) return;
    last_interrupt_time = current_time;

    button_event_t event = EVENT_NONE;

    if (gpio == BUTTON1_PIN) event = EVENT_FOOD;
    else if (gpio == BUTTON2_PIN) event = EVENT_WATER;
    else if (gpio == BUTTON3_PIN) event = EVENT_BATHROOM;
    else if (gpio == BUTTON4_PIN) event = EVENT_PLAY;

    if (event != EVENT_NONE) {
        // Envia para a fila a partir de uma interrupção (FromISR)
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        xQueueSendFromISR(xButtonEventQueue, &event, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}
// ================= CONFIGURAÇÃO DE HARDWARE =================

void init_hardware_buttons() {
    // Configura Buzzer
    gpio_set_function(BUZZER_PIN, GPIO_FUNC_PWM);
    uint slice_num = pwm_gpio_to_slice_num(BUZZER_PIN);
    pwm_config config = pwm_get_default_config();
    pwm_init(slice_num, &config, true);
    pwm_set_gpio_level(BUZZER_PIN, 0);

    // Configura Botões com Pull-up e Interrupção
    const uint buttons[] = {BUTTON1_PIN, BUTTON2_PIN, BUTTON3_PIN, BUTTON4_PIN};
    
    for(int i=0; i<4; i++) {
        gpio_init(buttons[i]);
        gpio_set_dir(buttons[i], GPIO_IN);
        gpio_pull_up(buttons[i]);
        // Habilita interrupção na descida (ao pressionar)
        gpio_set_irq_enabled_with_callback(buttons[i], GPIO_IRQ_EDGE_FALL, true, &gpio_callback);
    }
}
// ================= CALLBACKS I2C =================
int i2c_write(uint8_t addr, const uint8_t *data, uint16_t len) {
    return i2c_write_blocking(I2C_PORT0, addr, data, len, false) < 0 ? -1 : 0;
}

int i2c_read(uint8_t addr, uint8_t *data, uint16_t len) {
    return i2c_read_blocking(I2C_PORT0, addr, data, len, false) < 0 ? -1 : 0;
}

void delay_ms(uint32_t ms) {
    vTaskDelay(pdMS_TO_TICKS(ms));
}
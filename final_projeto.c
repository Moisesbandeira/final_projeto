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
#include "pico/cyw43_arch.h"
#include "lwipopts.h"

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
#define WIFI_SSID       "brisa-4338675"
#define WIFI_PASSWORD   "abqkbrvg"



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

// // Tarefa Wi-Fi
// void task_wifi(void *pvParameters) 
// {
//     if (cyw43_arch_init()) 
//     {
//         printf("Falha ao inicializar Wi-Fi\n");
//         vTaskDelete(NULL);
//     }

//     cyw43_arch_enable_sta_mode();
//     printf("Conectando ao Wi-Fi...\n");

//     if (cyw43_arch_wifi_connect_timeout_ms(
//             WIFI_SSID, WIFI_PASSWORD,
//             CYW43_AUTH_WPA2_AES_PSK, 30000) != 0) 
//     {
//         printf("Falha ao conectar no Wi-Fi\n");
//     } 
//     else 
//     {
//         printf("Wi-Fi conectado com sucesso!\n");
//     }
//     while (true) 
//     {
//         cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);
//         vTaskDelay(pdMS_TO_TICKS(500));
//         cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 0);
//         vTaskDelay(pdMS_TO_TICKS(500));
//     }
// }
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
    
    if (cyw43_arch_wifi_connect_timeout_ms(WIFI_SSID, WIFI_PASSWORD, CYW43_AUTH_WPA2_AES_PSK, 30000) == 0) {
        printf("Wi-Fi OK!\n");
    }

    while (true) {
        cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);
        vTaskDelay(pdMS_TO_TICKS(100));
        cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 0);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
// ================= TAREFA DE PROCESSAMENTO =================

void task_button_processor(void *pvParameters) {
    button_event_t event;
    
    while (true) {
        // Fica bloqueado esperando um evento de botão na fila
        if (xQueueReceive(xButtonEventQueue, &event, portMAX_DELAY)) {
            
            // Ativa o Buzzer (feedback sonoro)
            pwm_set_gpio_level(BUZZER_PIN, 2048); 
            
            switch (event) {
                case EVENT_FOOD:
                    printf("[MQTT] Enviando: Quero Comida\n");
                    // Futuramente: mqtt_publish("pico/ajuda", "comida");
                    break;
                case EVENT_WATER:
                    printf("[MQTT] Enviando: Quero Água\n");
                    break;
                case EVENT_BATHROOM:
                    printf("[MQTT] Enviando: Preciso ir ao Banheiro\n");
                    break;
                case EVENT_PLAY:
                    printf("[MQTT] Enviando: Quero Brincar\n");
                    break;
                default:
                    break;
            }

            vTaskDelay(pdMS_TO_TICKS(500)); // Duração do bip
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
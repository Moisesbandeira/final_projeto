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

// ================= CONFIGURAÇÕES =================
#define WIFI_SSID       "Gesilane"
#define WIFI_PASSWORD   "bruxxf6d"


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


// assinatura  das funções I2C
int i2c_write(uint8_t addr, const uint8_t *data, uint16_t len);
int i2c_read(uint8_t addr, uint8_t *data, uint16_t len);
void delay_ms(uint32_t ms);

// Tarefa Wi-Fi
void task_wifi(void *pvParameters) 
{
    if (cyw43_arch_init()) 
    {
        printf("Falha ao inicializar Wi-Fi\n");
        vTaskDelete(NULL);
    }

    cyw43_arch_enable_sta_mode();
    printf("Conectando ao Wi-Fi...\n");

    if (cyw43_arch_wifi_connect_timeout_ms(
            WIFI_SSID, WIFI_PASSWORD,
            CYW43_AUTH_WPA2_AES_PSK, 30000) != 0) 
    {
        printf("Falha ao conectar no Wi-Fi\n");
    } 
    else 
    {
        printf("Wi-Fi conectado com sucesso!\n");
    }
    while (true) 
    {
        cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);
        vTaskDelay(pdMS_TO_TICKS(500));
        cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 0);
        vTaskDelay(pdMS_TO_TICKS(500));
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

    // Define estrutura do sensor
    AHT10_Handle aht10 = {
        .iface = {
            .i2c_write = i2c_write,
            .i2c_read = i2c_read,
            .delay_ms = delay_ms
        }
    };

    printf("Inicializando AHT10...\n");
    if (!AHT10_Init(&aht10)) {
        printf("Falha na inicialização do sensor!\n");
        ssd1306_clear();
        ssd1306_draw_string(32, 0, "Sensor AHT10");
        ssd1306_draw_string(23, 30, "Falha no AHT10");
        ssd1306_show();
        while (1) sleep_ms(1000);
    }

    // Cria tarefa Wi-Fi e inicia Agedador de tarefas
   // xTaskCreate(task_wifi, "WiFi_Task", 1024, NULL, 1, NULL);
///   vTaskStartScheduler();
 ///  while(1) {}
    
    while (1) {
        float temp, hum, lux;
        if (AHT10_ReadTemperatureHumidity(&aht10, &temp, &hum)) {
            lux = bh1750_read_lux(I2C_PORT0);
            printf("Temperatura: %.2f °C | Umidade: %.2f %% | Luz: %.2f lux\n", temp, hum, lux);
        } else {
            printf("Falha na leitura dos dados!\n");
        }
        ssd1306_clear();
        // Organiza labels e valores em colunas (label à esquerda, valor à direita)
        char temp_str[16];
        char hum_str[16];
        char lux_str[16];
        snprintf(temp_str, sizeof(temp_str), "%.2f C", temp);
        snprintf(hum_str, sizeof(hum_str), "%.2f %%", hum);
        snprintf(lux_str, sizeof(lux_str), "%.2f lx", lux);
        // Título (opcional)
        ssd1306_draw_string(32, 0, "Sensores");
        // Linhas de dados (Y: 16, 32, 48)
        ssd1306_draw_string(0, 16, "Temperatura:");
        ssd1306_draw_string(90, 16, temp_str);
        ssd1306_draw_string(0, 32, "Umidade:");
        ssd1306_draw_string(90, 32, hum_str);
        ssd1306_draw_string(0, 48, "Luz:");
        ssd1306_draw_string(90, 48, lux_str);
        ssd1306_show();

        sleep_ms(1000);
    }
}
    // Função para escrita I2C (para o sensor AHT10 em I2C_PORT0)
int i2c_write(uint8_t addr, const uint8_t *data, uint16_t len) {
    int result = i2c_write_blocking(I2C_PORT0, addr, data, len, false);
    return result < 0 ? -1 : 0;
}

// Função para leitura I2C (para o sensor AHT10 em I2C_PORT0)
int i2c_read(uint8_t addr, uint8_t *data, uint16_t len) {
    int result = i2c_read_blocking(I2C_PORT0, addr, data, len, false);
    return result < 0 ? -1 : 0;
}

// Função para delay
void delay_ms(uint32_t ms) {
    sleep_ms(ms);
}


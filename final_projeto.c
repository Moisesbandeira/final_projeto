// Projeto_Final: BH1750 + AHT10 + OLED 128x64 + ESP8266 + aplicativo android + protocolo MQTT
// Autor: Moisés Lourenço / Felipe 
// Data: 24/01/2026
// Descrição: A caixa de comunicação alternativa Lê os sensores de luminosidade, temperatura e umidade, bem como a interação da criança com a mesma
//           e envia todas estas informações a um servidor que fica disponivel ao cuidador em um dispositivo móvel android via aplicativo. e mostra as 
//           informações no display OLED 128x64.

#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "ssd1306.h"
#include "bh1750.h"
#include "aht10.h"

// ===== Configuração de I2C =====
// I2C0: BH1750 (sensor de luminosidade)
#define I2C_PORT_BH1750 i2c0       
#define SDA_BH1750 0
#define SCL_BH1750 1

// I2C1: AHT10 (temperatura/umidade) + SSD1306 (display OLED)
#define I2C_PORT_DISPLAY i2c1
#define SDA_DISPLAY 14
#define SCL_DISPLAY 15

// Aliases para compatibilidade com código existente
#define I2C_PORT_SENSOR I2C_PORT_BH1750
#define SDA_SENSOR SDA_BH1750
#define SCL_SENSOR SCL_BH1750
#define I2C_PORT_OLED I2C_PORT_DISPLAY
#define SDA_OLED SDA_DISPLAY
#define SCL_OLED SCL_DISPLAY

// ===== Wrappers I2C para AHT10 (i2c1) =====
static int aht10_i2c_write(uint8_t addr, const uint8_t *data, uint16_t len) {
    // Função wrapper para compatibilidade com driver AHT10
    // Retorna 0 em sucesso, -1 em erro
    // Usando I2C_PORT_DISPLAY (i2c1) para AHT10
    int ret = i2c_write_blocking(I2C_PORT_DISPLAY, addr, (uint8_t *)data, len, false);
    if (ret < 0) {
        printf("AHT10 I2C: escrita falhou para addr 0x%02X, ret=%d\n", addr, ret);
        return -1;
    }
    if (ret != len) {
        printf("AHT10 I2C: escrita parcial (esperado %d, escreveu %d)\n", len, ret);
        return -1;
    }
    return 0;
}

static int aht10_i2c_read(uint8_t addr, uint8_t *data, uint16_t len) {
    // Função wrapper para compatibilidade com driver AHT10
    // Retorna 0 em sucesso, -1 em erro
    // Usando I2C_PORT_DISPLAY (i2c1) para AHT10
    int ret = i2c_read_blocking(I2C_PORT_DISPLAY, addr, data, len, false);
    if (ret < 0) {
        printf("AHT10 I2C: leitura falhou para addr 0x%02X, ret=%d\n", addr, ret);
        return -1;
    }
    if (ret != len) {
        printf("AHT10 I2C: leitura parcial (esperado %d, leu %d)\n", len, ret);
        return -1;
    }
    return 0;
}

static void aht10_delay_ms(uint32_t ms) {
    sleep_ms(ms);
}

int main() {
    stdio_init_all();
    sleep_ms(500);
    printf("\n===== INICIANDO SISTEMA =====\n");
    
    // ===== I2C0: BH1750 (Sensor de Luminosidade) =====
    printf("\n[1/3] I2C0 (BH1750 - addr 0x23)...\n");
    i2c_init(I2C_PORT_SENSOR, 100 * 1000);
    gpio_set_function(SDA_SENSOR, GPIO_FUNC_I2C);
    gpio_set_function(SCL_SENSOR, GPIO_FUNC_I2C);
    gpio_pull_up(SDA_SENSOR);
    gpio_pull_up(SCL_SENSOR);
    sleep_ms(200);
    
    bh1750_init(I2C_PORT_SENSOR);
    sleep_ms(300);
    printf("  ✓ BH1750 OK\n");

    // ===== I2C1: AHT10 + SSD1306 =====
    printf("\n[2/3] I2C1 (AHT10 + SSD1306)...\n");
    i2c_init(I2C_PORT_OLED, 100 * 1000);
    gpio_set_function(SDA_OLED, GPIO_FUNC_I2C);
    gpio_set_function(SCL_OLED, GPIO_FUNC_I2C);
    gpio_pull_up(SDA_OLED);
    gpio_pull_up(SCL_OLED);
    sleep_ms(200);

    // ===== SCANNER I2C - Diagnostic =====
    sleep_ms(500);
    printf("\n");
    printf("################################\n");
    printf("# SCANNER I2C - DIAGNOSTIC    #\n");
    printf("################################\n");
    
    printf("\n[SCAN] I2C0 (GPIO 0,1 - BH1750 esperado 0x23):\n");
    int found_i2c0 = 0;
    for (int addr = 0x03; addr < 0x78; addr++) {
        int ret = i2c_write_blocking(I2C_PORT_SENSOR, addr, NULL, 0, false);
        if (ret >= 0) {
            printf("  FOUND: 0x%02X\n", addr);
            found_i2c0++;
        }
    }
    if (found_i2c0 == 0) printf("  (nenhum dispositivo encontrado)\n");
    
    printf("\n[SCAN] I2C1 (GPIO 14,15 - AHT10 0x38 + SSD1306 0x3C esperados):\n");
    int found_i2c1 = 0;
    for (int addr = 0x03; addr < 0x78; addr++) {
        int ret = i2c_write_blocking(I2C_PORT_OLED, addr, NULL, 0, false);
        if (ret >= 0) {
            printf("  FOUND: 0x%02X\n", addr);
            found_i2c1++;
        }
    }
    if (found_i2c1 == 0) printf("  (nenhum dispositivo encontrado)\n");
    
    printf("\n[TESTE] Tentando AHT10 em endereços alternativos:\n");
    uint8_t test_cmd[] = {0xBA}; // Reset command
    for (int test_addr = 0x38; test_addr <= 0x39; test_addr++) {
        int ret = i2c_write_blocking(I2C_PORT_OLED, test_addr, test_cmd, 1, false);
        printf("  AHT10 em 0x%02X: ret=%d\n", test_addr, ret);
    }
    
    printf("\n################################\n");
    printf("# FIM DO SCANNER             #\n");
    printf("################################\n\n");

    // Inicializa AHT10
    AHT10_Handle aht10_dev;
    aht10_dev.iface.i2c_write = aht10_i2c_write;
    aht10_dev.iface.i2c_read = aht10_i2c_read;
    aht10_dev.iface.delay_ms = aht10_delay_ms;
    
    if (!AHT10_Init(&aht10_dev)) {
        printf("  ✗ AVISO: AHT10 init falhou\n");
    } else {
        printf("  ✓ AHT10 OK\n");
    }
    sleep_ms(300);

    // Inicializa SSD1306
    ssd1306_init(I2C_PORT_OLED);
    sleep_ms(200);
    printf("  ✓ SSD1306 OK\n");
    
    printf("\n===== SISTEMA PRONTO =====\n\n");
    
    // Tela inicial
    ssd1306_clear();
    ssd1306_draw_string(10, 0, "BH1750 + AHT10");
    ssd1306_show();
    sleep_ms(2000);
    
    while (true) {
        // Lê luminosidade (BH1750)
        float lux = bh1750_read_lux(I2C_PORT_SENSOR);
        printf("BH1750: ");
        if (lux < 0) {
            printf("leitura falhou\n");
            lux = 0.0f;
        } else {
            printf("%.1f lux\n", lux);
        }
        
        // Aguarda antes de ler AHT10 (timing importante)
        sleep_ms(500);
        
        // Tenta ler temperatura e umidade (AHT10)
        float temperature = 0.0f, humidity = 0.0f;
        bool aht_ok = false;
        
        // Tenta ler AHT10 3 vezes antes de desistir
        for (int retry = 0; retry < 3; retry++) {
            aht_ok = AHT10_ReadTemperatureHumidity(&aht10_dev, &temperature, &humidity);
            if (aht_ok) break;
            sleep_ms(100);
        }
        
        printf("AHT10: ");
        if (aht_ok) {
            printf("%.1fC, %.1f%%\n", temperature, humidity);
        } else {
            printf("leitura falhou\n");
            temperature = 0.0f;
            humidity = 0.0f;
        }
        
        // Aguarda antes de atualizar display
        sleep_ms(500);
      
        // Exibir no OLED
        ssd1306_clear();
        ssd1306_draw_string(20, 0, "Sensores");
        
        char line1[24], line2[24], line3[24];
        snprintf(line1, sizeof(line1), "Luz: %.1f lux", lux);
        snprintf(line2, sizeof(line2), "Tmp: %.1fC", temperature);
        snprintf(line3, sizeof(line3), "Umi: %.1f%%", humidity);
        
        ssd1306_draw_string(6, 15, line1);
        ssd1306_draw_string(6, 30, line2);
        ssd1306_draw_string(6, 45, line3);
        
        ssd1306_show();
        
        // Aguarda antes do próximo ciclo
        sleep_ms(1000);
   }
}

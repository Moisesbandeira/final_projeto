#include "aht10.h"
#include <stdio.h>

// Função interna para enviar comando ao AHT10
// Envia comando de 3 bytes (comando + 2 argumentos) via I2C
static bool aht10_write_command(AHT10_Handle *dev, uint8_t cmd, uint8_t arg1, uint8_t arg2) {
    uint8_t buf[3] = { cmd, arg1, arg2 };
    // Escreve os 3 bytes via I2C para o endereço do AHT10
    int ret = dev->iface.i2c_write(AHT10_I2C_ADDRESS, buf, 3);
    printf("AHT10: escreve cmd 0x%02X com args 0x%02X 0x%02X -> ret=%d\n", cmd, arg1, arg2, ret);
    return ret == 0;
}

// Inicializa o sensor AHT10
// 1. Faz reset suave do sensor
// 2. Aguarda 20ms para que o sensor reinicie
// 3. Envia comando de inicialização com parâmetros de calibração
bool AHT10_Init(AHT10_Handle *dev) {
    if (!dev) return false;
    printf("AHT10: Iniciando inicialização\n");
    
    // Primeiro, faz reset suave do sensor
    if (!AHT10_SoftReset(dev)) {
        printf("AHT10: Reset suave falhou\n");
        return false;
    }
    dev->iface.delay_ms(20); // Aguarda 20ms pós-reset
    
    // Envia comando de inicialização com parâmetros de calibração
    dev->initialized = aht10_write_command(dev, AHT10_CMD_INITIALIZE, 0x08, 0x00);
    dev->iface.delay_ms(10); // Aguarda 10ms pós-inicialização
    printf("AHT10: Inicialização %s\n", dev->initialized ? "sucesso" : "falhou");
    return dev->initialized;
}
// Reinicia (reset suave) o sensor AHT10
// Envia comando de reset e aguarda 20ms para completar
bool AHT10_SoftReset(AHT10_Handle *dev) {
    if (!dev) return false;
    uint8_t cmd = AHT10_CMD_RESET; // Comando 0xBA
    // Envia comando de reset via I2C (1 byte apenas)
    int ret = dev->iface.i2c_write(AHT10_I2C_ADDRESS, &cmd, 1);
    printf("AHT10: Reset suave -> ret=%d\n", ret);
    dev->iface.delay_ms(20); // Aguarda 20ms para o reset completar
    return ret == 0;
}
// Verifica se o sensor está ocupado (bit 7 do status = 1 significa ocupado)
bool AHT10_IsBusy(AHT10_Handle *dev) {
    uint8_t status = 0;
    // Lê byte de status do sensor
    if (dev->iface.i2c_read(AHT10_I2C_ADDRESS, &status, 1) != 0) return true;
    // Bit 7 (0x80) = 1 significa que está em conversão (ocupado)
    return (status & 0x80) != 0;
}
// Lê temperatura (°C) e umidade (%) do sensor AHT10
// Retorna true se leitura foi bem-sucedida, false caso contrário
bool AHT10_ReadTemperatureHumidity(AHT10_Handle *dev, float *temperature, float *humidity) {
    // Verifica se sensor está inicializado
    if (!dev || !dev->initialized) {
        printf("AHT10: Sensor não inicializado ou handle inválido\n");
        return false;
    }

    // Envia comando de medição ao sensor
    if (!aht10_write_command(dev, AHT10_CMD_MEASURE, 0x33, 0x00)) {
        printf("AHT10: Comando de medição falhou\n");
        return false;
    }
    // Aguarda 120ms para a conversão ser concluída (aumentado para mais segurança)
    dev->iface.delay_ms(120);

    // Lê 6 bytes de resposta contendo status + dados de umidade/temperatura
    uint8_t raw[6];
    int ret = dev->iface.i2c_read(AHT10_I2C_ADDRESS, raw, 6);
    printf("AHT10: Lê 6 bytes -> ret=%d\n", ret);
    if (ret != 0) {
        printf("AHT10: Leitura falhou\n");
        return false;
    }
    
    // Exibe dados brutos para debug
    printf("AHT10: Dados brutos: 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X\n", 
           raw[0], raw[1], raw[2], raw[3], raw[4], raw[5]);

    // Verifica se sensor ainda está ocupado (bit 7 = 1)
    if ((raw[0] & 0x80) != 0) {
        printf("AHT10: Sensor ainda ocupado, aguardando mais...\n");
        dev->iface.delay_ms(50);
        
        // Tenta ler novamente
        ret = dev->iface.i2c_read(AHT10_I2C_ADDRESS, raw, 6);
        if (ret != 0 || (raw[0] & 0x80) != 0) {
            printf("AHT10: Falha na segunda tentativa\n");
            return false;
        }
    }

    // Extrai valores brutos de umidade e temperatura do buffer
    // Umidade: bits 19-0 de raw[1..3]
    uint32_t raw_hum = ((uint32_t)(raw[1]) << 12) | ((uint32_t)(raw[2]) << 4) | (raw[3] >> 4);
    // Temperatura: bits 19-0 de raw[3..5]
    uint32_t raw_temp = (((uint32_t)(raw[3] & 0x0F)) << 16) | ((uint32_t)(raw[4]) << 8) | raw[5];

    printf("AHT10: Umidade bruta=%lu, Temperatura bruta=%lu\n", raw_hum, raw_temp);

    // Converte valores brutos para valores reais conforme datasheet
    // Umidade: (raw_hum / 2^20) * 100
    *humidity = (raw_hum * 100.0f) / 1048576.0f;
    // Temperatura: ((raw_temp / 2^20) * 200) - 50
    *temperature = ((raw_temp * 200.0f) / 1048576.0f) - 50.0f;
    
    printf("AHT10: Temperatura convertida=%.1f°C, Umidade convertida=%.1f%%\n", *temperature, *humidity);

    return true;
}

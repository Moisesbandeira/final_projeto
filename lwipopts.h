#ifndef _LWIPOPTS_H
#define _LWIPOPTS_H

// Como estamos usando FreeRTOS, não queremos o modo NO_SYS
#define NO_SYS                      0
#define LWIP_SOCKET                 0
#define LWIP_NETCONN                0

// --- ADICIONE ESTAS LINHAS PARA CORRIGIR O ERRO DE COMPILAÇÃO ---
#define LWIP_NETIF_HOSTNAME         1
#define LWIP_DHCP                   1
// -------------------------------------------------------------

#define LWIP_TIMEVAL_PRIVATE        0

// Ajustes para o FreeRTOS
#define SYS_LIGHTWEIGHT_PROT        1
#define MEM_ALIGNMENT               4

// ESSENCIAL: Diz ao lwIP para usar os hooks do FreeRTOS
#define TCPIP_THREAD_STACKSIZE      1024
#define TCPIP_THREAD_PRIO           (configMAX_PRIORITIES - 2)
#define TCPIP_MBOX_SIZE             8
#define DEFAULT_TCP_RECVMBOX_SIZE   8
#define DEFAULT_UDP_RECVMBOX_SIZE   8
#define DEFAULT_RAW_RECVMBOX_SIZE   8
#define DEFAULT_ACCEPTMBOX_SIZE     8

// --- HABILITAR DNS E PROTOCOLOS NECESSÁRIOS ---
#define LWIP_DNS                    1
#define LWIP_UDP                    1  // DNS requer UDP
#define LWIP_TCP                    1  // MQTT requer TCP
#define LWIP_IGMP                   1  // Bom para descoberta de rede

// Configurações extras de DNS para estabilidade
#define DNS_TABLE_SIZE              2
#define DNS_MAX_RETRANSMISSION      4

// Aumenta o número de conexões TCP simultâneas permitidas
#ifndef MEMP_NUM_TCP_PCB
#define MEMP_NUM_TCP_PCB 16
#endif

// Aumenta o número de buffers de requisições HTTP
#ifndef MEMP_NUM_HTTPC_STATE
#define MEMP_NUM_HTTPC_STATE 8
#endif

// Reduz o tempo que uma conexão morta fica ocupando memória
#ifndef TCP_MSL
#define TCP_MSL 500UL
#endif

#endif
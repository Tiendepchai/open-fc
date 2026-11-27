/*
 * OPEN-FC MINIMAL v1.0 — esp32-wifi MAVLink2 UDP <-> UART1 Telemetry Bridge
 *
 * Safety boundary (AGENTS.md §1.1 & §4.4):
 * - Transparent telemetry bridge ONLY; never generates commands or drives actuators.
 * - Handles UART framing/overflow errors and Wi-Fi disconnects without blocking.
 * - Never logs Wi-Fi credentials.
 */

#include <errno.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "driver/uart.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "lwip/inet.h"
#include "lwip/sockets.h"
#include "nvs_flash.h"

#include "mavlink_framer.h"

static const char *TAG = "openfc_wifi";

#define UART_RX_BUF_SIZE 2048
#define UART_TX_BUF_SIZE 2048
#define UART_EVT_QUEUE_LEN 20
/* Full Ethernet IPv4 MTU headroom so coalesced multi-packet GCS UDP datagrams are not truncated */
#define UDP_RX_BUF_SIZE 1500
#define UART_IDLE_RESET_MS 100
#define BRIDGE_TASK_PRIO 5

typedef struct {
    int sock;
    uint32_t sock_gen;
    bool sock_reset_req;
    struct sockaddr_in peer_addr;
    bool peer_learned;
    uint32_t uart_err_count;
    uint32_t uart_tx_err_count;
    uint32_t udp_tx_drop_count;
    uint32_t udp_rx_reject_count;
    uint32_t udp_sock_recover_count;
    openfc_mavlink_framer_t framer;
} bridge_state_t;

static bridge_state_t g_bridge;
static portMUX_TYPE g_bridge_mux = portMUX_INITIALIZER_UNLOCKED;
static QueueHandle_t g_uart_queue = NULL;

static void wifi_event_handler(void *arg, esp_event_base_t base, int32_t id, void *data) {
    (void)arg;
    (void)data;
    if (base == WIFI_EVENT) {
        if (id == WIFI_EVENT_STA_START) {
            esp_err_t err = esp_wifi_connect();
            if (err != ESP_OK) {
                ESP_LOGW(TAG, "Wi-Fi STA initial connect failed: %d", (int)err);
            }
        } else if (id == WIFI_EVENT_STA_DISCONNECTED) {
            ESP_LOGW(TAG, "Wi-Fi STA disconnected; scheduling reconnect");
            esp_err_t err = esp_wifi_connect();
            if (err != ESP_OK) {
                ESP_LOGW(TAG, "Wi-Fi STA reconnect failed: %d", (int)err);
            }
        } else if (id == WIFI_EVENT_AP_STACONNECTED) {
            ESP_LOGI(TAG, "GCS station connected to SoftAP");
        } else if (id == WIFI_EVENT_AP_STADISCONNECTED) {
            ESP_LOGW(TAG, "GCS station disconnected from SoftAP");
        }
    } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        ESP_LOGI(TAG, "Wi-Fi STA obtained IP address");
    }
}

static void init_wifi(void) {
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, NULL));

    wifi_config_t wifi_cfg;
    memset(&wifi_cfg, 0, sizeof(wifi_cfg));

#if CONFIG_OPENFC_WIFI_MODE_AP
    esp_netif_t *ap_netif = esp_netif_create_default_wifi_ap();
    configASSERT(ap_netif != NULL);
    strncpy((char *)wifi_cfg.ap.ssid, CONFIG_OPENFC_WIFI_SSID, sizeof(wifi_cfg.ap.ssid) - 1U);
    wifi_cfg.ap.ssid_len = (uint8_t)strlen((const char *)wifi_cfg.ap.ssid);
    wifi_cfg.ap.max_connection = 2;
    if (strlen(CONFIG_OPENFC_WIFI_PASS) >= 8U) {
        strncpy((char *)wifi_cfg.ap.password, CONFIG_OPENFC_WIFI_PASS, sizeof(wifi_cfg.ap.password) - 1U);
        wifi_cfg.ap.authmode = WIFI_AUTH_WPA2_PSK;
    } else {
        wifi_cfg.ap.password[0] = '\0';
        wifi_cfg.ap.authmode = WIFI_AUTH_OPEN;
        ESP_LOGW(TAG, "SoftAP password < 8 chars; using OPEN auth (set local WPA2 password)");
    }
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_cfg));
#else
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL, NULL));
    esp_netif_t *sta_netif = esp_netif_create_default_wifi_sta();
    configASSERT(sta_netif != NULL);
    strncpy((char *)wifi_cfg.sta.ssid, CONFIG_OPENFC_WIFI_SSID, sizeof(wifi_cfg.sta.ssid) - 1U);
    strncpy((char *)wifi_cfg.sta.password, CONFIG_OPENFC_WIFI_PASS, sizeof(wifi_cfg.sta.password) - 1U);
    wifi_cfg.sta.threshold.authmode = (strlen(CONFIG_OPENFC_WIFI_PASS) > 0U)
                                          ? WIFI_AUTH_WPA2_PSK
                                          : WIFI_AUTH_OPEN;
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_cfg));
#endif

    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_LOGI(TAG, "Wi-Fi started (credentials omitted from logs)");
}

static void init_uart(void) {
    const uart_port_t port = (uart_port_t)CONFIG_OPENFC_UART_PORT;
    const uart_config_t cfg = {
        .baud_rate = CONFIG_OPENFC_UART_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    ESP_ERROR_CHECK(uart_driver_install(port, UART_RX_BUF_SIZE, UART_TX_BUF_SIZE,
                                        UART_EVT_QUEUE_LEN, &g_uart_queue, 0));
    ESP_ERROR_CHECK(uart_param_config(port, &cfg));
    ESP_ERROR_CHECK(uart_set_pin(port, CONFIG_OPENFC_UART_TX_GPIO, CONFIG_OPENFC_UART_RX_GPIO,
                                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
}

static void init_default_peer(void) {
    struct sockaddr_in peer;
    memset(&peer, 0, sizeof(peer));
    peer.sin_family = AF_INET;
    peer.sin_port = htons((uint16_t)CONFIG_OPENFC_GCS_UDP_PORT);
    if (inet_pton(AF_INET, CONFIG_OPENFC_GCS_IP, &peer.sin_addr) != 1) {
        ESP_LOGW(TAG, "Invalid default GCS IP config; waiting for dynamic GCS peer packet");
        peer.sin_addr.s_addr = 0U;
    }
    taskENTER_CRITICAL(&g_bridge_mux);
    g_bridge.peer_addr = peer;
    g_bridge.peer_learned = false;
    taskEXIT_CRITICAL(&g_bridge_mux);
}

static int open_udp_socket(void) {
    int s = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    if (s < 0) {
        ESP_LOGE(TAG, "UDP socket() failed: errno=%d", errno);
        return -1;
    }

    int reuse = 1;
    (void)setsockopt(s, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));

    struct timeval tv = {.tv_sec = 0, .tv_usec = 20000};
    if (setsockopt(s, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv)) < 0) {
        ESP_LOGW(TAG, "UDP SO_RCVTIMEO failed: errno=%d", errno);
    }

    struct sockaddr_in bind_addr;
    memset(&bind_addr, 0, sizeof(bind_addr));
    bind_addr.sin_family = AF_INET;
    bind_addr.sin_port = htons((uint16_t)CONFIG_OPENFC_LOCAL_UDP_PORT);
    bind_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    if (bind(s, (struct sockaddr *)&bind_addr, sizeof(bind_addr)) < 0) {
        ESP_LOGE(TAG, "UDP bind() failed: errno=%d", errno);
        close(s);
        return -1;
    }
    return s;
}

static void uart_to_udp_task(void *arg) {
    (void)arg;
    const uart_port_t port = (uart_port_t)CONFIG_OPENFC_UART_PORT;
    uint8_t rx_chunk[256];
    uart_event_t evt;

    for (;;) {
        if (xQueueReceive(g_uart_queue, &evt, pdMS_TO_TICKS(UART_IDLE_RESET_MS)) != pdTRUE) {
            /* Discard stalled partial packet after UART idle gap so next real packet is not corrupted */
            if (g_bridge.framer.pos > 0U) {
                openfc_mavlink_framer_reset(&g_bridge.framer);
            }
            continue;
        }
        switch (evt.type) {
            case UART_DATA: {
                size_t remaining = evt.size;
                while (remaining > 0U) {
                    size_t to_read = (remaining < sizeof(rx_chunk)) ? remaining : sizeof(rx_chunk);
                    int n = uart_read_bytes(port, rx_chunk, to_read, pdMS_TO_TICKS(10));
                    if (n <= 0) {
                        break;
                    }
                    size_t n_read = (size_t)n;
                    remaining = (n_read >= remaining) ? 0U : (remaining - n_read);
                    for (size_t i = 0U; i < n_read; ++i) {
                        size_t pkt_len = 0U;
                        if (openfc_mavlink_framer_feed(&g_bridge.framer, rx_chunk[i], &pkt_len)) {
                            int sock_snap;
                            uint32_t sock_gen_snap;
                            struct sockaddr_in peer_snap;
                            taskENTER_CRITICAL(&g_bridge_mux);
                            sock_snap = g_bridge.sock;
                            sock_gen_snap = g_bridge.sock_gen;
                            peer_snap = g_bridge.peer_addr;
                            taskEXIT_CRITICAL(&g_bridge_mux);

                            if (sock_snap >= 0 && peer_snap.sin_addr.s_addr != 0U && peer_snap.sin_port != 0U) {
                                int sent = sendto(sock_snap, g_bridge.framer.buf, pkt_len, MSG_DONTWAIT,
                                                  (const struct sockaddr *)&peer_snap, sizeof(peer_snap));
                                if (sent < 0) {
                                    int err = errno;
                                    g_bridge.udp_tx_drop_count++;
                                    if (err == EBADF || err == ENOTSOCK) {
                                        taskENTER_CRITICAL(&g_bridge_mux);
                                        if (g_bridge.sock == sock_snap && g_bridge.sock_gen == sock_gen_snap) {
                                            g_bridge.sock_reset_req = true;
                                        }
                                        taskEXIT_CRITICAL(&g_bridge_mux);
                                    }
                                } else if ((size_t)sent != pkt_len) {
                                    g_bridge.udp_tx_drop_count++;
                                }
                            } else {
                                g_bridge.udp_tx_drop_count++;
                            }
                        }
                    }
                }
                break;
            }
            case UART_FIFO_OVF:
            case UART_BUFFER_FULL:
            case UART_FRAME_ERR:
            case UART_PARITY_ERR:
            case UART_BREAK:
                g_bridge.uart_err_count++;
                uart_flush_input(port);
                xQueueReset(g_uart_queue);
                openfc_mavlink_framer_reset(&g_bridge.framer);
                break;
            default:
                break;
        }
    }
}

static void udp_to_uart_task(void *arg) {
    (void)arg;
    const uart_port_t port = (uart_port_t)CONFIG_OPENFC_UART_PORT;
    /* Single task instance: keep 1500 B MTU buffer off the 4 KB FreeRTOS task stack */
    static uint8_t udp_buf[UDP_RX_BUF_SIZE];

    for (;;) {
        int sock_cur;
        bool need_reset;
        taskENTER_CRITICAL(&g_bridge_mux);
        sock_cur = g_bridge.sock;
        need_reset = g_bridge.sock_reset_req;
        taskEXIT_CRITICAL(&g_bridge_mux);

        if (sock_cur < 0 || need_reset) {
            taskENTER_CRITICAL(&g_bridge_mux);
            int old_sock = g_bridge.sock;
            g_bridge.sock = -1;
            g_bridge.sock_gen++;
            g_bridge.sock_reset_req = false;
            taskEXIT_CRITICAL(&g_bridge_mux);

            if (old_sock >= 0) {
                /* Allow any concurrent MSG_DONTWAIT sendto() on Core 0 to return before closing fd */
                vTaskDelay(pdMS_TO_TICKS(20));
                close(old_sock);
            }

            int new_sock = open_udp_socket();
            if (new_sock < 0) {
                vTaskDelay(pdMS_TO_TICKS(500));
                continue;
            }

            taskENTER_CRITICAL(&g_bridge_mux);
            g_bridge.sock = new_sock;
            g_bridge.sock_gen++;
            taskEXIT_CRITICAL(&g_bridge_mux);
            g_bridge.udp_sock_recover_count++;
            ESP_LOGI(TAG, "UDP socket ready (recovery_count=%lu)",
                     (unsigned long)g_bridge.udp_sock_recover_count);
            sock_cur = new_sock;
        }

        struct sockaddr_in src;
        socklen_t srclen = (socklen_t)sizeof(src);
        int n = recvfrom(sock_cur, udp_buf, sizeof(udp_buf), 0, (struct sockaddr *)&src, &srclen);
        if (n < 0) {
            int err = errno;
            if (err == EAGAIN || err == EWOULDBLOCK || err == EINTR) {
                continue;
            }
            if (err == ENOMEM || err == ENOBUFS) {
                vTaskDelay(pdMS_TO_TICKS(10));
                continue;
            }
            ESP_LOGW(TAG, "UDP recvfrom() error=%d; scheduling socket recovery", err);
            taskENTER_CRITICAL(&g_bridge_mux);
            g_bridge.sock_reset_req = true;
            taskEXIT_CRITICAL(&g_bridge_mux);
            vTaskDelay(pdMS_TO_TICKS(200));
            continue;
        }
        if (n == 0) {
            g_bridge.udp_rx_reject_count++;
            continue;
        }
        if (!openfc_mavlink_validate_datagram(udp_buf, (size_t)n)) {
            g_bridge.udp_rx_reject_count++;
            continue;
        }
        if (srclen >= (socklen_t)sizeof(src) && src.sin_family == AF_INET && src.sin_port != 0U) {
            taskENTER_CRITICAL(&g_bridge_mux);
            g_bridge.peer_addr = src;
            g_bridge.peer_learned = true;
            taskEXIT_CRITICAL(&g_bridge_mux);
        }
        int written = uart_write_bytes(port, udp_buf, (size_t)n);
        if (written < 0 || written != n) {
            g_bridge.uart_tx_err_count++;
        }
    }
}

void app_main(void) {
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    memset(&g_bridge, 0, sizeof(g_bridge));
    g_bridge.sock = -1;
    openfc_mavlink_framer_init(&g_bridge.framer);

    init_uart();
    init_wifi();
    init_default_peer();
    g_bridge.sock = open_udp_socket();

    BaseType_t ok_tx = xTaskCreatePinnedToCore(
        uart_to_udp_task, "uart_to_udp", 4096, NULL, BRIDGE_TASK_PRIO, NULL, 0);
    BaseType_t ok_rx = xTaskCreatePinnedToCore(
        udp_to_uart_task, "udp_to_uart", 4096, NULL, BRIDGE_TASK_PRIO, NULL, 1);
    configASSERT(ok_tx == pdPASS);
    configASSERT(ok_rx == pdPASS);

    ESP_LOGI(TAG, "OPEN-FC Wi-Fi MAVLink2 bridge running (UART%d @ %d bps)",
             CONFIG_OPENFC_UART_PORT, CONFIG_OPENFC_UART_BAUD);
}

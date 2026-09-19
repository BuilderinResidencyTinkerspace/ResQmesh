/**
 * @file receiver.cpp
 * @brief Wireless Flight Control Receiver Subsystem with ESP-NOW Swarm Support.
 *
 * Implements ESP-NOW multi-agent mesh packet reception, broadcast heartbeat
 * transmission, command execution, and legacy packet fallback.
 */

#include "receiver.h"
#include <cstring>
#include <cstdlib>
#include <algorithm>

#if defined(ESP_PLATFORM)
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_idf_version.h"

#if (RECEIVER_MODE_WIFI == 1)
#include <WiFi.h>
#include <esp_http_server.h>
#include "web_ui.h"
#else
#include "esp_wifi.h"
#include "esp_now.h"
#endif

static const char *TAG = "RECEIVER";
#else
#include <cstdio>
#define ESP_LOGI(tag, fmt, ...) printf("[INFO][%s] " fmt "\n", tag, ##__VA_ARGS__)
#define ESP_LOGW(tag, fmt, ...) printf("[WARN][%s] " fmt "\n", tag, ##__VA_ARGS__)
#define ESP_LOGE(tag, fmt, ...) printf("[ERROR][%s] " fmt "\n", tag, ##__VA_ARGS__)
static const char *TAG = "RECEIVER_HOST";
#endif

Receiver *Receiver::instance_ = nullptr;

#if defined(ESP_PLATFORM) && (RECEIVER_MODE_WIFI == 1)
static httpd_handle_t s_httpd = NULL;

static esp_err_t http_index_handler(httpd_req_t *req) {
    httpd_resp_set_type(req, "text/html");
    httpd_resp_set_hdr(req, "Cache-Control", "no-cache, no-store, must-revalidate");
    httpd_resp_send(req, WEB_UI_HTML, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

static esp_err_t ws_control_handler(httpd_req_t *req) {
    if (req->method == HTTP_GET) {
        return ESP_OK;
    }

    httpd_ws_frame_t ws_pkt;
    uint8_t rx_buf[256];
    memset(&ws_pkt, 0, sizeof(ws_pkt));
    ws_pkt.type = HTTPD_WS_TYPE_TEXT;
    ws_pkt.payload = rx_buf;

    esp_err_t ret = httpd_ws_recv_frame(req, &ws_pkt, sizeof(rx_buf) - 1);
    if (ret != ESP_OK) {
        return ret;
    }

    rx_buf[ws_pkt.len] = '\0';

    char tx_buf[128] = {0};
    Receiver *rx = Receiver::getInstance();
    if (rx) {
        rx->processWebSocketFrame((const char *)rx_buf, ws_pkt.len, tx_buf, sizeof(tx_buf));
    }

    if (tx_buf[0] != '\0') {
        httpd_ws_frame_t resp_pkt;
        memset(&resp_pkt, 0, sizeof(resp_pkt));
        resp_pkt.type = HTTPD_WS_TYPE_TEXT;
        resp_pkt.payload = (uint8_t *)tx_buf;
        resp_pkt.len = strlen(tx_buf);
        return httpd_ws_send_frame(req, &resp_pkt);
    }

    return ESP_OK;
}

static bool start_web_server() {
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.server_port = 80;
    config.max_open_sockets = 4;
    config.lru_purge_enable = true;

    if (httpd_start(&s_httpd, &config) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start HTTP server!");
        return false;
    }

    httpd_uri_t uri_index = {
        .uri = "/",
        .method = HTTP_GET,
        .handler = http_index_handler,
        .user_ctx = NULL,
        .is_websocket = false
    };
    httpd_register_uri_handler(s_httpd, &uri_index);

    httpd_uri_t uri_ws = {
        .uri = "/ws",
        .method = HTTP_GET,
        .handler = ws_control_handler,
        .user_ctx = NULL,
        .is_websocket = true
    };
    httpd_register_uri_handler(s_httpd, &uri_ws);

    ESP_LOGI(TAG, "Web UI & WebSocket server active on port 80");
    return true;
}
#endif

#if defined(ESP_PLATFORM) && (RECEIVER_MODE_WIFI == 0)
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
static void espnow_recv_wrapper(const esp_now_recv_info_t *recv_info, const uint8_t *data, int len) {
    const uint8_t *mac = recv_info ? recv_info->src_addr : nullptr;
    Receiver::onDataRecv(mac, data, len);
}
#else
static void espnow_recv_wrapper(const uint8_t *mac, const uint8_t *data, int len) {
    Receiver::onDataRecv(mac, data, len);
}
#endif
#endif

Receiver::Receiver()
    : last_packet_time_us_(0),
      last_sequence_(0),
      packets_received_(0),
      packets_lost_(0),
      is_connected_(false),
      is_first_packet_(true),
      node_id_(SWARM_DEFAULT_NODE_ID),
      role_(SWARM_ROLE_FOLLOWER),
      tx_sequence_(0),
      pending_cmd_(CMD_SWARM_NONE),
      has_pending_cmd_(false),
      telem_batt_v_(3.85f),
      telem_pitch_(0.0f),
      telem_roll_(0.0f),
      telem_loop_hz_(500.0f) {
    memset(&latest_packet_, 0, sizeof(latest_packet_));
    memset(pending_cmd_payload_, 0, sizeof(pending_cmd_payload_));
    strncpy(telem_state_, "DISARMED", sizeof(telem_state_));
    instance_ = this;
}

Receiver::~Receiver() {
#if defined(ESP_PLATFORM)
#if (RECEIVER_MODE_WIFI == 1)
    if (s_httpd) {
        httpd_stop(s_httpd);
        s_httpd = NULL;
    }
    WiFi.softAPdisconnect(true);
#else
    esp_now_deinit();
#endif
#endif
    if (instance_ == this) {
        instance_ = nullptr;
    }
}

uint16_t Receiver::computeCRC(const uint8_t *data, size_t length) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < length; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (int b = 0; b < 8; b++) {
            if (crc & 0x8000) {
                crc = (crc << 1) ^ 0x1021; // CCITT polynomial
            } else {
                crc = (crc << 1);
            }
        }
    }
    return crc;
}

bool Receiver::init() {
#if defined(ESP_PLATFORM)
#if (RECEIVER_MODE_WIFI == 1)
    ESP_LOGI(TAG, "Initializing Wi-Fi SoftAP for Phone Control: SSID='%s'", WIFI_AP_SSID);

    WiFi.mode(WIFI_AP);
    IPAddress local_ip(192, 168, 4, 1);
    IPAddress gateway(192, 168, 4, 1);
    IPAddress subnet(255, 255, 255, 0);
    WiFi.softAPConfig(local_ip, gateway, subnet);

    bool ap_ok = WiFi.softAP(WIFI_AP_SSID, WIFI_AP_PASSWORD, WIFI_AP_CHANNEL, 0, WIFI_MAX_CLIENTS);
    if (!ap_ok) {
        ESP_LOGE(TAG, "Failed to start Wi-Fi SoftAP!");
        return false;
    }

    ESP_LOGI(TAG, "SoftAP online! Connect phone to SSID '%s' -> navigate to http://192.168.4.1", WIFI_AP_SSID);
    return start_web_server();

#else
    ESP_LOGI(TAG, "Initializing ESP-NOW Swarm Node (Node ID: %d, Channel: %d)...", node_id_, ESPNOW_WIFI_CHANNEL);

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_err_t err = esp_wifi_init(&cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_wifi_init failed: 0x%x", err);
        return false;
    }

    esp_wifi_set_storage(WIFI_STORAGE_RAM);
    esp_wifi_set_mode(WIFI_MODE_STA);
    esp_wifi_start();
    esp_wifi_set_channel(ESPNOW_WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE);

    err = esp_now_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_now_init failed: 0x%x", err);
        return false;
    }

    err = esp_now_register_recv_cb(espnow_recv_wrapper);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_now_register_recv_cb failed: 0x%x", err);
        return false;
    }

    // Register 2.4GHz Broadcast Peer (FF:FF:FF:FF:FF:FF) for Swarm Heartbeats & Commands
    static const uint8_t s_broadcast_mac[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
    esp_now_peer_info_t peer_info = {};
    memcpy(peer_info.peer_addr, s_broadcast_mac, 6);
    peer_info.channel = ESPNOW_WIFI_CHANNEL;
    peer_info.encrypt = false;

    if (esp_now_is_peer_exist(s_broadcast_mac) == false) {
        esp_err_t p_err = esp_now_add_peer(&peer_info);
        if (p_err != ESP_OK) {
            ESP_LOGW(TAG, "Failed to register ESP-NOW broadcast peer: 0x%x", p_err);
        } else {
            ESP_LOGI(TAG, "ESP-NOW broadcast peer registered successfully.");
        }
    }

    ESP_LOGI(TAG, "ESP-NOW Swarm Subsystem online on Channel %d. Ready for multi-agent mesh.", ESPNOW_WIFI_CHANNEL);
    return true;
#endif
#else
    return true;
#endif
}

void Receiver::setTelemetry(float batt_v, float pitch, float roll, const char *state_str, float loop_hz) {
    telem_batt_v_ = batt_v;
    telem_pitch_  = pitch;
    telem_roll_   = roll;
    telem_loop_hz_= loop_hz;
    if (state_str) {
        strncpy(telem_state_, state_str, sizeof(telem_state_) - 1);
        telem_state_[sizeof(telem_state_) - 1] = '\0';
    }
}

void Receiver::processWebSocketFrame(const char *json_str, size_t len, char *resp_buf, size_t max_resp_len) {
    if (!json_str || len == 0) return;

    float t = 0.0f;
    int y = 0, p = 0, r = 0, a = 0, calib = 0;

    const char *p_t = strstr(json_str, "\"t\":");
    if (p_t) t = (float)atof(p_t + 4);

    const char *p_y = strstr(json_str, "\"y\":");
    if (p_y) y = atoi(p_y + 4);

    const char *p_p = strstr(json_str, "\"p\":");
    if (p_p) p = atoi(p_p + 4);

    const char *p_r = strstr(json_str, "\"r\":");
    if (p_r) r = atoi(p_r + 4);

    const char *p_a = strstr(json_str, "\"a\":");
    if (p_a) a = atoi(p_a + 4);

    const char *p_c = strstr(json_str, "\"calib\":");
    if (p_c) calib = atoi(p_c + 8);

    latest_packet_.throttle = (uint16_t)std::max(0.0f, std::min(1000.0f, t));
    latest_packet_.yaw      = (int16_t)std::max(-500, std::min(500, y));
    latest_packet_.pitch    = (int16_t)std::max(-500, std::min(500, p));
    latest_packet_.roll     = (int16_t)std::max(-500, std::min(500, r));
    latest_packet_.arm      = (uint8_t)a;
    latest_packet_.mode     = (calib == 1) ? MODE_CALIB : MODE_ANGLE;
    latest_packet_.sequence++;

#if defined(ESP_PLATFORM)
    last_packet_time_us_ = esp_timer_get_time();
#else
    last_packet_time_us_ += 25000;
#endif
    is_connected_ = true;
    packets_received_++;

    if (resp_buf && max_resp_len > 0) {
        snprintf(resp_buf, max_resp_len,
            "{\"b\":%.2f,\"p\":%.1f,\"r\":%.1f,\"s\":\"%s\",\"l\":%.0f,\"a\":%d}",
            telem_batt_v_,
            telem_pitch_,
            telem_roll_,
            telem_state_,
            telem_loop_hz_,
            (latest_packet_.arm == 1 ? 1 : 0)
        );
    }
}

void Receiver::processRawPacket(const ControlPacket &pkt, int64_t now_us) {
    if (!is_first_packet_) {
        uint32_t expected_seq = last_sequence_ + 1;
        if (pkt.sequence > expected_seq) {
            packets_lost_ += (pkt.sequence - expected_seq);
        }
    } else {
        is_first_packet_ = false;
    }

    last_sequence_ = pkt.sequence;
    latest_packet_ = pkt;
    last_packet_time_us_ = now_us;
    packets_received_++;
    is_connected_ = true;
}

void Receiver::processSwarmPacket(const uint8_t *data, size_t len, int64_t now_us) {
    if (!data || len < sizeof(SwarmHeader)) return;

    const SwarmHeader *hdr = reinterpret_cast<const SwarmHeader *>(data);
    if (hdr->magic[0] != SWARM_MAGIC_0 || hdr->magic[1] != SWARM_MAGIC_1) {
        return;
    }

    switch (hdr->msg_type) {
        case SWARM_MSG_CONTROL: {
            if (len != sizeof(SwarmControlPacket)) return;
            const SwarmControlPacket *cp = reinterpret_cast<const SwarmControlPacket *>(data);

            // Check if packet is targeted to our node or broadcast to all
            if (cp->header.target_id != node_id_ && cp->header.target_id != SWARM_NODE_BROADCAST) {
                return; // Addressed to a different drone in the swarm
            }

            // Verify CRC
            uint16_t computed_crc = computeCRC(data, sizeof(SwarmControlPacket) - sizeof(uint16_t));
            if (computed_crc != cp->crc) {
                ESP_LOGW(TAG, "Swarm Control CRC error: 0x%04X != 0x%04X", computed_crc, cp->crc);
                return;
            }

            // Map into standard control packet
            ControlPacket pkt;
            pkt.throttle = cp->throttle;
            pkt.roll     = cp->roll;
            pkt.pitch    = cp->pitch;
            pkt.yaw      = cp->yaw;
            pkt.arm      = cp->arm;
            pkt.mode     = cp->mode;
            pkt.sequence = cp->header.sequence;
            pkt.crc      = cp->crc;
            processRawPacket(pkt, now_us);
            break;
        }

        case SWARM_MSG_HEARTBEAT: {
            if (len != sizeof(SwarmHeartbeatPacket)) return;
            const SwarmHeartbeatPacket *hb = reinterpret_cast<const SwarmHeartbeatPacket *>(data);

            // Verify CRC
            uint16_t computed_crc = computeCRC(data, sizeof(SwarmHeartbeatPacket) - sizeof(uint16_t));
            if (computed_crc != hb->crc) {
                return;
            }

            // Update decentralized peer mesh table
            peer_table_.updatePeer(*hb, now_us);
            break;
        }

        case SWARM_MSG_COMMAND: {
            if (len != sizeof(SwarmCommandPacket)) return;
            const SwarmCommandPacket *cmd = reinterpret_cast<const SwarmCommandPacket *>(data);

            if (cmd->header.target_id != node_id_ && cmd->header.target_id != SWARM_NODE_BROADCAST) {
                return;
            }

            uint16_t computed_crc = computeCRC(data, sizeof(SwarmCommandPacket) - sizeof(uint16_t));
            if (computed_crc != cmd->crc) {
                ESP_LOGW(TAG, "Swarm Command CRC error: 0x%04X != 0x%04X", computed_crc, cmd->crc);
                return;
            }

            pending_cmd_ = (SwarmCommandId)cmd->cmd_id;
            memcpy(pending_cmd_payload_, cmd->payload, sizeof(pending_cmd_payload_));
            has_pending_cmd_ = true;

            // Direct safety enforcement on urgent swarm commands
            if (pending_cmd_ == CMD_SWARM_EMERGENCY_STOP) {
                latest_packet_.arm = 2;
                latest_packet_.throttle = 0;
                last_packet_time_us_ = now_us;
                is_connected_ = true;
            } else if (pending_cmd_ == CMD_SWARM_DISARM_ALL) {
                latest_packet_.arm = 0;
                latest_packet_.throttle = 0;
                last_packet_time_us_ = now_us;
                is_connected_ = true;
            } else if (pending_cmd_ == CMD_SWARM_ARM_ALL) {
                latest_packet_.arm = 1;
                last_packet_time_us_ = now_us;
                is_connected_ = true;
            }
            break;
        }

        default:
            break;
    }
}

void Receiver::onDataRecv(const uint8_t *mac_addr, const uint8_t *data, int len) {
    if (!instance_ || !data || len <= 0) return;

#if defined(ESP_PLATFORM)
    int64_t now_us = esp_timer_get_time();
#else
    int64_t now_us = 0;
#endif

    // 1. Check for ResQmesh Swarm Protocol framing ('R', 'Q')
    if ((size_t)len >= sizeof(SwarmHeader) && data[0] == SWARM_MAGIC_0 && data[1] == SWARM_MAGIC_1) {
        instance_->processSwarmPacket(data, (size_t)len, now_us);
        return;
    }

    // 2. Fallback: Legacy 16-byte ControlPacket compatibility
    if ((size_t)len == sizeof(ControlPacket)) {
        const ControlPacket *pkt = reinterpret_cast<const ControlPacket *>(data);
        uint16_t computed_crc = computeCRC(data, sizeof(ControlPacket) - sizeof(uint16_t));
        if (computed_crc != pkt->crc) {
            ESP_LOGW(TAG, "Legacy CRC mismatch: 0x%04X != 0x%04X", computed_crc, pkt->crc);
            return;
        }
        instance_->processRawPacket(*pkt, now_us);
        return;
    }

    ESP_LOGW(TAG, "Unknown packet received, size=%d bytes", len);
}

void Receiver::injectPacket(const ControlPacket &packet, int64_t now_us) {
    processRawPacket(packet, now_us);
}

void Receiver::injectSwarmPacket(const uint8_t *data, size_t len, int64_t now_us) {
    processSwarmPacket(data, len, now_us);
}

bool Receiver::sendHeartbeat(uint8_t state, uint16_t batt_mv, uint8_t batt_pct,
                             float roll, float pitch, float yaw_rate, bool armed, uint32_t uptime_s) {
    SwarmHeartbeatPacket pkt;
    pkt.header.magic[0] = SWARM_MAGIC_0;
    pkt.header.magic[1] = SWARM_MAGIC_1;
    pkt.header.msg_type = SWARM_MSG_HEARTBEAT;
    pkt.header.target_id = SWARM_NODE_BROADCAST;
    pkt.header.sender_id = node_id_;
    pkt.header.sequence  = ++tx_sequence_;

    pkt.role             = role_;
    pkt.state            = state;
    pkt.battery_mv       = batt_mv;
    pkt.battery_pct      = batt_pct;
    pkt.roll_deg_x10     = (int16_t)(roll * 10.0f);
    pkt.pitch_deg_x10    = (int16_t)(pitch * 10.0f);
    pkt.yaw_rate_dps_x10 = (int16_t)(yaw_rate * 10.0f);
    pkt.armed            = armed ? 1 : 0;
    pkt.uptime_s         = uptime_s;
    pkt.crc              = computeCRC((const uint8_t *)&pkt, sizeof(pkt) - sizeof(uint16_t));

#if defined(ESP_PLATFORM) && (RECEIVER_MODE_WIFI == 0)
    static const uint8_t bcast_mac[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
    esp_err_t err = esp_now_send(bcast_mac, (const uint8_t *)&pkt, sizeof(pkt));
    return (err == ESP_OK);
#else
    return true;
#endif
}

bool Receiver::sendSwarmCommand(uint8_t cmd_id, uint8_t target_id, const uint8_t *payload, size_t payload_len) {
    SwarmCommandPacket pkt;
    pkt.header.magic[0] = SWARM_MAGIC_0;
    pkt.header.magic[1] = SWARM_MAGIC_1;
    pkt.header.msg_type = SWARM_MSG_COMMAND;
    pkt.header.target_id = target_id;
    pkt.header.sender_id = node_id_;
    pkt.header.sequence  = ++tx_sequence_;

    pkt.cmd_id = cmd_id;
    memset(pkt.payload, 0, sizeof(pkt.payload));
    if (payload && payload_len > 0) {
        size_t cp_len = (payload_len > sizeof(pkt.payload)) ? sizeof(pkt.payload) : payload_len;
        memcpy(pkt.payload, payload, cp_len);
    }
    pkt.crc = computeCRC((const uint8_t *)&pkt, sizeof(pkt) - sizeof(uint16_t));

#if defined(ESP_PLATFORM) && (RECEIVER_MODE_WIFI == 0)
    static const uint8_t bcast_mac[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
    esp_err_t err = esp_now_send(bcast_mac, (const uint8_t *)&pkt, sizeof(pkt));
    return (err == ESP_OK);
#else
    return true;
#endif
}

bool Receiver::hasPendingSwarmCommand(SwarmCommandId &cmd, uint8_t *payload) {
    if (!has_pending_cmd_) return false;
    cmd = pending_cmd_;
    if (payload) {
        memcpy(payload, pending_cmd_payload_, sizeof(pending_cmd_payload_));
    }
    has_pending_cmd_ = false;
    pending_cmd_ = CMD_SWARM_NONE;
    return true;
}

void Receiver::clearPendingSwarmCommand() {
    has_pending_cmd_ = false;
    pending_cmd_ = CMD_SWARM_NONE;
}

bool Receiver::update(ReceiverData &data, int64_t now_us) {
    int64_t timeout_us = (int64_t)RECEIVER_TIMEOUT_MS * 1000LL;

    // Strict watchdog: if signal drops or node is isolated, trip failsafe
    if (last_packet_time_us_ == 0 || (now_us - last_packet_time_us_) > timeout_us) {
        is_connected_ = false;

        data.throttle         = 0.0f;
        data.roll_angle       = 0.0f;
        data.pitch_angle      = 0.0f;
        data.yaw_rate         = 0.0f;
        data.arm_command      = false;
        data.emergency_stop   = false;
        data.mode             = MODE_ANGLE;
        data.sequence         = last_sequence_;
        data.packets_received = packets_received_;
        data.packets_lost     = packets_lost_;
        data.is_connected     = false;
        return false;
    }

    is_connected_ = true;

    // Map throttle: [0, 1000] -> [0.0, 1000.0]
    data.throttle = (float)latest_packet_.throttle;

    // Map roll: [-500, +500] -> [-STICK_ANGLE_MAX_DEG, +STICK_ANGLE_MAX_DEG]
    float norm_roll = (float)latest_packet_.roll / 500.0f;
    norm_roll = std::max(-1.0f, std::min(1.0f, norm_roll));
    data.roll_angle = norm_roll * STICK_ANGLE_MAX_DEG;

    // Map pitch: [-500, +500] -> [-STICK_ANGLE_MAX_DEG, +STICK_ANGLE_MAX_DEG]
    float norm_pitch = (float)latest_packet_.pitch / 500.0f;
    norm_pitch = std::max(-1.0f, std::min(1.0f, norm_pitch));
    data.pitch_angle = norm_pitch * STICK_ANGLE_MAX_DEG;

    // Map yaw: [-500, +500] -> [-STICK_YAW_RATE_MAX_DPS, +STICK_YAW_RATE_MAX_DPS]
    float norm_yaw = (float)latest_packet_.yaw / 500.0f;
    norm_yaw = std::max(-1.0f, std::min(1.0f, norm_yaw));
    data.yaw_rate = norm_yaw * STICK_YAW_RATE_MAX_DPS;

    data.arm_command    = (latest_packet_.arm == 1);
    data.emergency_stop = (latest_packet_.arm == 2);
    data.mode           = (FlightMode)latest_packet_.mode;
    data.sequence       = latest_packet_.sequence;
    data.packets_received = packets_received_;
    data.packets_lost   = packets_lost_;
    data.is_connected   = true;

    return true;
}

#include "Tab5LocalNode.h"

#include "comms/PacketClient.h"
#include "comms/PacketServer.h"
#include "graphics/common/LoRaPresets.h"
#include "mesh-pb-constants.h"
#include "meshtastic/admin.pb.h"
#include "meshtastic/channel.pb.h"
#include "meshtastic/connection_status.pb.h"
#include "meshtastic/config.pb.h"
#include "meshtastic/device_ui.pb.h"
#include "meshtastic/mesh.pb.h"
#include "meshtastic/module_config.pb.h"
#include "LittleFS.h"
#include "util/ILog.h"
#include "util/Packet.h"
#if defined(ARDUINO_ARCH_ESP32) || defined(ESP_PLATFORM)
#include <esp_system.h>
#endif
#include <mbedtls/aes.h>
#include <Arduino.h>
#include <RadioLib.h>
#include <SPI.h>
#include <algorithm>
#include <cstring>
#include <memory>
#include <vector>

namespace {

constexpr uint8_t kLoRaSck = 5;
constexpr uint8_t kLoRaMiso = 19;
constexpr uint8_t kLoRaMosi = 18;
constexpr uint8_t kLoRaCs = 33;
constexpr uint8_t kLoRaIrq = 6;
constexpr int8_t kLoRaRst = -1;
constexpr uint8_t kLoRaBusy = 46;
constexpr uint8_t kLoRaRfSwitch = 14;
constexpr uint8_t kRfSwitchRxLevel = HIGH;
constexpr uint8_t kRfSwitchTxLevel = LOW;
constexpr bool kEnableRfPathSweepTx = false;
constexpr bool kEnableDualRfSwitchTxProbe = false;
constexpr bool kPrimaryRfUsesDio2AutoSwitch = true;
constexpr bool kFallbackRfUsesDio2AutoSwitch = false;
constexpr bool kEnableAltHashCompatTx = false;
constexpr int8_t kMaxTab5TxPowerDbm = 22;

constexpr float kDefaultFrequencyMhz = 868.0f;
constexpr float kDefaultBandwidthKhz = 125.0f;
constexpr uint8_t kDefaultSpreadFactor = 12;
constexpr uint8_t kDefaultCodingRate = 8;
constexpr uint8_t kDefaultSyncWord = 0x2B;
constexpr int8_t kDefaultTxPowerDbm = 22;
constexpr uint8_t kDefaultPreamble = 16;

constexpr uint32_t kFallbackLocalNodeNum = 0x0A0B0C0D;
constexpr char kLongNamePrefix[] = "Tab5L";
constexpr char kEnvName[] = "tab5-mui";
constexpr char kPrimaryChannelName[] = "Primary";
constexpr char kFirmwareVersion[] = "2.7.17-tab5";
constexpr char kSilentRingtone[] = "";
constexpr uint32_t kAirProtocolVersion = 1;
constexpr uint16_t kMaxAirPayload = 240;
constexpr char kStateFilePath[] = "/tab5-node-state.bin";
constexpr char kMessagesDirPath[] = "/messages";
constexpr uint32_t kStateMagic = 0x54354C53;
constexpr uint16_t kStateVersionV1 = 1;
constexpr uint16_t kStateVersion = 2;
constexpr size_t kMaxChannels = 8;
constexpr uint8_t kDefaultPsk[16] = {0xD4, 0xF1, 0xBB, 0x3A, 0x20, 0x29, 0x07, 0x59,
                                     0xF0, 0xBC, 0xFF, 0xAB, 0xCF, 0x4E, 0x69, 0x01};

constexpr uint8_t kPacketFlagsHopLimitMask = 0x07;
constexpr uint8_t kPacketFlagsWantAckMask = 0x08;
constexpr uint8_t kPacketFlagsViaMqttMask = 0x10;
constexpr uint8_t kPacketFlagsHopStartMask = 0xE0;
constexpr uint8_t kPacketFlagsHopStartShift = 5;
constexpr uint8_t kBitfieldOkToMqttMask = 1u << 0;
constexpr uint8_t kBitfieldWantResponseMask = 1u << 1;
constexpr bool kEnableRawTxProbe = false;

volatile bool gRadioRxPending = false;
volatile bool gRadioTxDonePending = false;

#pragma pack(push, 1)
struct PacketHeader {
    uint32_t to;
    uint32_t from;
    uint32_t id;
    uint8_t flags;
    uint8_t channel;
    uint8_t next_hop;
    uint8_t relay_node;
};
#pragma pack(pop)

static_assert(sizeof(PacketHeader) == 16, "PacketHeader size mismatch");

struct ExpandedChannelKey {
    uint8_t bytes[32] = {0};
    size_t length = 0;
    bool valid = false;
};

uint8_t xorHash(const uint8_t *data, size_t len)
{
    uint8_t hash = 0;
    for (size_t i = 0; i < len; ++i) {
        hash ^= data[i];
    }
    return hash;
}

void logHexDump(const char *prefix, const uint8_t *data, size_t len)
{
    if (!prefix) {
        prefix = "HEX";
    }

    if (!data || len == 0) {
        ILOG_INFO("%s hex: <empty>", prefix);
        return;
    }

    char line[96] = {0};
    for (size_t offset = 0; offset < len; offset += 16) {
        const size_t chunkLen = std::min<size_t>(16, len - offset);
        size_t pos = static_cast<size_t>(std::snprintf(line, sizeof(line), "%s hex [%03u]:", prefix,
                                                       static_cast<unsigned>(offset)));
        for (size_t i = 0; i < chunkLen && pos + 4 < sizeof(line); ++i) {
            pos += static_cast<size_t>(std::snprintf(line + pos, sizeof(line) - pos, " %02X", data[offset + i]));
        }
        ILOG_INFO("%s", line);
    }
}

bool tryGetPresetRadioParameters(meshtastic_Config_LoRaConfig_ModemPreset preset, float &bandwidthKhz, uint8_t &spreadFactor,
                                 uint8_t &codingRate)
{
    switch (preset) {
    case meshtastic_Config_LoRaConfig_ModemPreset_LONG_FAST:
        bandwidthKhz = 250.0f;
        spreadFactor = 11;
        codingRate = 5;
        return true;
    case meshtastic_Config_LoRaConfig_ModemPreset_LONG_SLOW:
        bandwidthKhz = 125.0f;
        spreadFactor = 12;
        codingRate = 8;
        return true;
    case meshtastic_Config_LoRaConfig_ModemPreset_VERY_LONG_SLOW:
        bandwidthKhz = 62.0f;
        spreadFactor = 12;
        codingRate = 8;
        return true;
    case meshtastic_Config_LoRaConfig_ModemPreset_MEDIUM_SLOW:
        bandwidthKhz = 250.0f;
        spreadFactor = 10;
        codingRate = 7;
        return true;
    case meshtastic_Config_LoRaConfig_ModemPreset_MEDIUM_FAST:
        bandwidthKhz = 250.0f;
        spreadFactor = 9;
        codingRate = 5;
        return true;
    case meshtastic_Config_LoRaConfig_ModemPreset_SHORT_SLOW:
        bandwidthKhz = 250.0f;
        spreadFactor = 8;
        codingRate = 5;
        return true;
    case meshtastic_Config_LoRaConfig_ModemPreset_SHORT_FAST:
        bandwidthKhz = 250.0f;
        spreadFactor = 7;
        codingRate = 5;
        return true;
    case meshtastic_Config_LoRaConfig_ModemPreset_LONG_MODERATE:
        bandwidthKhz = 125.0f;
        spreadFactor = 11;
        codingRate = 8;
        return true;
    case meshtastic_Config_LoRaConfig_ModemPreset_SHORT_TURBO:
        bandwidthKhz = 500.0f;
        spreadFactor = 7;
        codingRate = 5;
        return true;
    case meshtastic_Config_LoRaConfig_ModemPreset_LONG_TURBO:
        bandwidthKhz = 500.0f;
        spreadFactor = 11;
        codingRate = 5;
        return true;
    default:
        return false;
    }
}

uint32_t localNodeNum();
void buildNodeId(char *out, size_t outSize);

void initNonce(uint32_t fromNode, uint64_t packetId, uint8_t nonce[16])
{
    std::memset(nonce, 0, 16);
    std::memcpy(nonce, &packetId, sizeof(uint64_t));
    std::memcpy(nonce + sizeof(uint64_t), &fromNode, sizeof(uint32_t));
}

bool cryptAesCtr(const uint8_t *key, size_t keyLength, uint32_t fromNode, uint64_t packetId, uint8_t *bytes, size_t length)
{
    if (keyLength == 0 || length == 0) {
        return true;
    }

    mbedtls_aes_context aes;
    mbedtls_aes_init(&aes);

    const int rc = mbedtls_aes_setkey_enc(&aes, key, static_cast<unsigned>(keyLength * 8));
    if (rc != 0) {
        mbedtls_aes_free(&aes);
        return false;
    }

    uint8_t nonceCounter[16] = {0};
    uint8_t streamBlock[16] = {0};
    uint8_t scratch[kMaxAirPayload] = {0};
    size_t ncOff = 0;
    initNonce(fromNode, packetId, nonceCounter);
    std::memcpy(scratch, bytes, length);

    const int cryptRc = mbedtls_aes_crypt_ctr(&aes, length, &ncOff, nonceCounter, streamBlock, scratch, bytes);
    mbedtls_aes_free(&aes);
    return cryptRc == 0;
}

bool applyPacketLayerDefaults(SX1262 &radio, int8_t txPowerDbm)
{
    int state = radio.setOutputPower(txPowerDbm);
    if (state != RADIOLIB_ERR_NONE) {
        ILOG_WARN("SX1262 setOutputPower failed: %d", state);
        return false;
    }

    state = radio.setCurrentLimit(140);
    if (state != RADIOLIB_ERR_NONE) {
        ILOG_WARN("SX1262 setCurrentLimit failed: %d", state);
        return false;
    }

    state = radio.setPreambleLength(kDefaultPreamble);
    if (state != RADIOLIB_ERR_NONE) {
        ILOG_WARN("SX1262 setPreambleLength failed: %d", state);
        return false;
    }

    state = radio.setSyncWord(kDefaultSyncWord);
    if (state != RADIOLIB_ERR_NONE) {
        ILOG_WARN("SX1262 setSyncWord failed: %d", state);
        return false;
    }

    state = radio.setCRC(2);
    if (state != RADIOLIB_ERR_NONE) {
        ILOG_WARN("SX1262 setCRC failed: %d", state);
        return false;
    }

    return true;
}

void generateLocalNames(char *shortName, size_t shortNameSize, char *longName, size_t longNameSize)
{
    char generatedShort[5] = {0};
    std::snprintf(generatedShort, sizeof(generatedShort), "%04lX", static_cast<unsigned long>(localNodeNum() & 0xFFFF));

    if (shortName && shortNameSize > 0) {
        std::snprintf(shortName, shortNameSize, "%s", generatedShort);
    }
    if (longName && longNameSize > 0) {
        std::snprintf(longName, longNameSize, "%s %s", kLongNamePrefix, generatedShort);
    }
}

uint32_t localNodeNum()
{
    static uint32_t nodeNum = 0;
    if (nodeNum != 0) {
        return nodeNum;
    }

#if defined(ARDUINO_ARCH_ESP32) || defined(ESP_PLATFORM)
    uint64_t mac = ESP.getEfuseMac();
    uint32_t derived = static_cast<uint32_t>(mac & 0xFFFFFFFFu);
    if (derived == 0 || derived == 0xFFFFFFFFu) {
        derived = kFallbackLocalNodeNum;
    }
    nodeNum = derived;
#else
    nodeNum = kFallbackLocalNodeNum;
#endif
    return nodeNum;
}

void buildNodeId(char *out, size_t outSize)
{
    std::snprintf(out, outSize, "!%08lX", static_cast<unsigned long>(localNodeNum()));
}

struct PersistedStateHeader {
    uint32_t magic = kStateMagic;
    uint16_t version = kStateVersion;
    uint16_t reserved = 0;
};

struct PersistedState {
    PersistedStateHeader header;
    meshtastic_User localUser = meshtastic_User_init_default;
    meshtastic_DeviceUIConfig deviceUiConfig = meshtastic_DeviceUIConfig_init_default;
    meshtastic_Config_DeviceConfig deviceConfig = meshtastic_Config_DeviceConfig_init_default;
    meshtastic_Config_PositionConfig positionConfig = meshtastic_Config_PositionConfig_init_default;
    meshtastic_Config_PowerConfig powerConfig = meshtastic_Config_PowerConfig_init_default;
    meshtastic_Config_NetworkConfig networkConfig = meshtastic_Config_NetworkConfig_init_default;
    meshtastic_Config_DisplayConfig displayConfig = meshtastic_Config_DisplayConfig_init_default;
    meshtastic_Config_LoRaConfig loraConfig = meshtastic_Config_LoRaConfig_init_default;
    meshtastic_Config_BluetoothConfig bluetoothConfig = meshtastic_Config_BluetoothConfig_init_default;
    meshtastic_Config_SecurityConfig securityConfig = meshtastic_Config_SecurityConfig_init_default;
    meshtastic_Channel channels[kMaxChannels] = {};
    meshtastic_ModuleConfig telemetryConfig = meshtastic_ModuleConfig_init_default;
    meshtastic_ModuleConfig cannedMessageConfig = meshtastic_ModuleConfig_init_default;
    char currentRingtone[231] = {0};
};

struct PersistedStateV1 {
    PersistedStateHeader header;
    meshtastic_User localUser = meshtastic_User_init_default;
    meshtastic_DeviceUIConfig deviceUiConfig = meshtastic_DeviceUIConfig_init_default;
    meshtastic_Config_DeviceConfig deviceConfig = meshtastic_Config_DeviceConfig_init_default;
    meshtastic_Config_PositionConfig positionConfig = meshtastic_Config_PositionConfig_init_default;
    meshtastic_Config_PowerConfig powerConfig = meshtastic_Config_PowerConfig_init_default;
    meshtastic_Config_NetworkConfig networkConfig = meshtastic_Config_NetworkConfig_init_default;
    meshtastic_Config_DisplayConfig displayConfig = meshtastic_Config_DisplayConfig_init_default;
    meshtastic_Config_LoRaConfig loraConfig = meshtastic_Config_LoRaConfig_init_default;
    meshtastic_Config_BluetoothConfig bluetoothConfig = meshtastic_Config_BluetoothConfig_init_default;
    meshtastic_Config_SecurityConfig securityConfig = meshtastic_Config_SecurityConfig_init_default;
    meshtastic_Channel primaryChannel = meshtastic_Channel_init_default;
    meshtastic_ModuleConfig telemetryConfig = meshtastic_ModuleConfig_init_default;
    meshtastic_ModuleConfig cannedMessageConfig = meshtastic_ModuleConfig_init_default;
    char currentRingtone[231] = {0};
};

void onRadioReceive()
{
    gRadioRxPending = true;
}

void onRadioTransmitDone()
{
    gRadioTxDonePending = true;
}

class Tab5StandaloneClient final : public PacketClient
{
  public:
    bool isStandalone(void) override { return true; }
};

struct KnownNode {
    uint32_t num = 0;
    meshtastic_User user = meshtastic_User_init_default;
    uint32_t lastHeard = 0;
    float snr = 0.0f;
};

struct RecentAirPacket {
    uint32_t from = 0;
    uint32_t id = 0;
    uint32_t seenAtMs = 0;
};

struct PendingAckTx {
    meshtastic_MeshPacket packet = meshtastic_MeshPacket_init_default;
    uint8_t retriesDone = 0;
    uint32_t lastSendMs = 0;
};

enum class RfPathMode {
    AutoHigh,
    ManualHigh,
    ManualLow,
};

constexpr uint32_t kReliableRetryIntervalMs = 1500;
constexpr uint8_t kReliableMaxRetries = 3;
constexpr uint32_t kInteractiveNodeInfoMinIntervalMs = 60000;

const char *portNumName(meshtastic_PortNum portnum)
{
    switch (portnum) {
    case meshtastic_PortNum_TEXT_MESSAGE_APP:
        return "TEXT";
    case meshtastic_PortNum_ROUTING_APP:
        return "ROUTING";
    case meshtastic_PortNum_ADMIN_APP:
        return "ADMIN";
    case meshtastic_PortNum_NODEINFO_APP:
        return "NODEINFO";
    case meshtastic_PortNum_POSITION_APP:
        return "POSITION";
    case meshtastic_PortNum_TELEMETRY_APP:
        return "TELEMETRY";
    case meshtastic_PortNum_TRACEROUTE_APP:
        return "TRACEROUTE";
    case meshtastic_PortNum_NEIGHBORINFO_APP:
        return "NEIGHBOR";
    case meshtastic_PortNum_RANGE_TEST_APP:
        return "RANGE";
    default:
        return "OTHER";
    }
}

enum class PendingSystemAction {
    None,
    Reboot,
    Shutdown,
    FactoryReset,
    NodeDbReset,
};

class Tab5LocalNodeImpl
{
  public:
    Tab5LocalNodeImpl() : radio(new Module(kLoRaCs, kLoRaIrq, kLoRaRst, kLoRaBusy)) {}

    bool begin()
    {
        if (started) {
            return true;
        }

        buildDefaults();
        loadState();
        packetServer.reset(PacketServer::init());
        packetClient.init();

        pinMode(kLoRaRfSwitch, OUTPUT);
        digitalWrite(kLoRaRfSwitch, HIGH);
        SPI.begin(kLoRaSck, kLoRaMiso, kLoRaMosi);

        int state = radio.begin(kDefaultFrequencyMhz, kDefaultBandwidthKhz, kDefaultSpreadFactor, kDefaultCodingRate,
                                kDefaultSyncWord, kDefaultTxPowerDbm, kDefaultPreamble, 3.0f, true);
        if (state != RADIOLIB_ERR_NONE) {
            ILOG_ERROR("SX1262 init failed: %d", state);
            return false;
        }

        if (!applyPacketLayerDefaults(radio, effectiveTxPowerDbm())) {
            ILOG_ERROR("SX1262 packet layer init failed");
            return false;
        }
        applyRfStrategy(kPrimaryRfUsesDio2AutoSwitch, false);
        radio.setPacketReceivedAction(onRadioReceive);
        state = radio.startReceive();
        if (state != RADIOLIB_ERR_NONE) {
            ILOG_ERROR("SX1262 startReceive failed: %d", state);
            return false;
        }

        ILOG_INFO("SX1262 started: sync=0x%02x preamble=%u defaultFreq=%.4fMHz defaultBw=%.1fkHz defaultSf=%u defaultCr=%u txPower=%d",
                  kDefaultSyncWord, kDefaultPreamble, kDefaultFrequencyMhz, kDefaultBandwidthKhz, kDefaultSpreadFactor,
                  kDefaultCodingRate, kDefaultTxPowerDbm);

        started = true;
        applyLoRaConfigToRadio();
        broadcastLocalNodeInfo("boot");
        return true;
    }

    void taskHandler()
    {
        if (!started) {
            return;
        }

        maybeBroadcastLocalNodeInfo();
        processPendingAckRetries();

        runPendingSystemAction();

        processUiQueue();

        if (gRadioRxPending) {
            gRadioRxPending = false;
            receiveFromRadio();
        }

        runPendingSystemAction();
    }

    IClientBase &client() { return packetClient; }

  private:
    const meshtastic_Channel &activePrimaryChannel() const
    {
        for (size_t i = 0; i < kMaxChannels; ++i) {
            if (channels[i].role == meshtastic_Channel_Role_PRIMARY) {
                return channels[i];
            }
        }
        return channels[0];
    }

    void applyLocalUserDefaults(bool preserveLongName)
    {
        char generatedShort[sizeof(localUser.short_name)] = {0};
        char generatedLong[sizeof(localUser.long_name)] = {0};
        generateLocalNames(generatedShort, sizeof(generatedShort), generatedLong, sizeof(generatedLong));

        if (localUser.id[0] == '\0') {
            buildNodeId(localUser.id, sizeof(localUser.id));
        }
        if (!preserveLongName || localUser.long_name[0] == '\0') {
            std::snprintf(localUser.long_name, sizeof(localUser.long_name), "%s", generatedLong);
        }
        std::snprintf(localUser.short_name, sizeof(localUser.short_name), "%s", generatedShort);
        localUser.hw_model = meshtastic_HardwareModel_PRIVATE_HW;
        localUser.role = deviceConfig.role;
    }

    void sendRebooted()
    {
        auto &from = prepareFromRadio(meshtastic_FromRadio_rebooted_tag);
        from.rebooted = true;
        queueFromRadio(from);
    }

    void scheduleSystemAction(PendingSystemAction action, int32_t seconds)
    {
        const int32_t delaySeconds = std::max(seconds, static_cast<int32_t>(0));
        pendingSystemAction = action;
        pendingSystemActionAtMs = millis() + static_cast<uint32_t>(delaySeconds) * 1000U;
    }

    void clearPersistedMessages()
    {
        File dir = LittleFS.open(kMessagesDirPath);
        if (!dir || !dir.isDirectory()) {
            return;
        }

        while (true) {
            File entry = dir.openNextFile();
            if (!entry) {
                break;
            }

            String path = entry.path();
            const bool isDir = entry.isDirectory();
            entry.close();

            if (isDir) {
                LittleFS.rmdir(path.c_str());
            } else {
                LittleFS.remove(path.c_str());
            }
        }

        dir.close();
        LittleFS.rmdir(kMessagesDirPath);
    }

    void runPendingSystemAction()
    {
        if (pendingSystemAction == PendingSystemAction::None) {
            return;
        }

        const int32_t remaining = static_cast<int32_t>(pendingSystemActionAtMs - millis());
        if (remaining > 0) {
            return;
        }

        const PendingSystemAction action = pendingSystemAction;
        pendingSystemAction = PendingSystemAction::None;

        switch (action) {
        case PendingSystemAction::FactoryReset:
            ILOG_INFO("Performing factory reset");
            LittleFS.remove(kStateFilePath);
            clearPersistedMessages();
            buildDefaults();
            saveState();
            break;
        case PendingSystemAction::NodeDbReset:
            ILOG_INFO("Performing node database reset");
            knownNodes.clear();
            updateSelfNode();
            myInfo.nodedb_count = 1;
            break;
        case PendingSystemAction::Reboot:
        case PendingSystemAction::Shutdown:
        case PendingSystemAction::None:
            break;
        }

        if (action == PendingSystemAction::Shutdown) {
            ILOG_INFO("Entering deep sleep");
            packetClient.disconnect();
#if defined(ARDUINO_ARCH_ESP32) || defined(ESP_PLATFORM)
            esp_deep_sleep_start();
#endif
            return;
        }

        if (action == PendingSystemAction::Reboot || action == PendingSystemAction::FactoryReset ||
            action == PendingSystemAction::NodeDbReset) {
            ILOG_INFO("Restarting Tab5 standalone node");
            packetClient.disconnect();
#if defined(ARDUINO_ARCH_ESP32) || defined(ESP_PLATFORM)
            esp_restart();
#endif
        }
    }

    const char *effectivePrimaryChannelName() const
    {
        const auto &primary = activePrimaryChannel();
        if (primary.settings.name[0] != '\0') {
            return primary.settings.name;
        }
        if (loraConfig.use_preset) {
            return LoRaPresets::modemPresetToString(loraConfig.modem_preset);
        }
        return "Custom";
    }

    const char *hashPrimaryChannelName() const
    {
        const auto &primary = activePrimaryChannel();
        // Official Meshtastic hashes an unnamed default preset channel using the
        // derived preset display name (for example LongFast), not an empty string.
        if (loraConfig.use_preset) {
            const char *presetName = LoRaPresets::modemPresetToString(loraConfig.modem_preset);
            if (primary.settings.name[0] == '\0') {
                return presetName;
            }
        }

        // Explicit channel names hash exactly as stored on the channel.
        return primary.settings.name;
    }

    ExpandedChannelKey expandedPrimaryKey() const
    {
        ExpandedChannelKey key;
        const auto &primary = activePrimaryChannel();
        if (!primary.has_settings || primary.role == meshtastic_Channel_Role_DISABLED) {
            return key;
        }

        key.length = primary.settings.psk.size;
        std::memcpy(key.bytes, primary.settings.psk.bytes, key.length);

        if (key.length == 0) {
            key.valid = true;
            return key;
        }

        if (key.length == 1) {
            const uint8_t pskIndex = key.bytes[0];
            if (pskIndex == 0) {
                key.length = 0;
                key.valid = true;
                return key;
            }

            std::memcpy(key.bytes, kDefaultPsk, sizeof(kDefaultPsk));
            key.bytes[sizeof(kDefaultPsk) - 1] = static_cast<uint8_t>(key.bytes[sizeof(kDefaultPsk) - 1] + pskIndex - 1);
            key.length = sizeof(kDefaultPsk);
            key.valid = true;
            return key;
        }

        if (key.length < sizeof(kDefaultPsk)) {
            std::memset(key.bytes + key.length, 0, sizeof(kDefaultPsk) - key.length);
            key.length = sizeof(kDefaultPsk);
        }

        key.valid = key.length == 16 || key.length == 32;
        return key;
    }

    uint8_t primaryChannelHash() const
    {
        const ExpandedChannelKey key = expandedPrimaryKey();
        return xorHash(reinterpret_cast<const uint8_t *>(hashPrimaryChannelName()), std::strlen(hashPrimaryChannelName())) ^
               xorHash(key.bytes, key.length);
    }

    uint8_t currentAirChannelHash() const
    {
        // Always transmit with the local channel hash. Learned hashes are kept
        // for diagnostics only and must not steer TX onto a foreign network.
        return primaryChannelHash();
    }

    uint8_t legacyPresetNameChannelHash() const
    {
        const ExpandedChannelKey key = expandedPrimaryKey();
        return xorHash(reinterpret_cast<const uint8_t *>(effectivePrimaryChannelName()), std::strlen(effectivePrimaryChannelName())) ^
               xorHash(key.bytes, key.length);
    }

    uint32_t effectiveChannelNum() const
    {
        if (!loraConfig.use_preset) {
            return loraConfig.channel_num;
        }

        if (loraConfig.channel_num > 0) {
            return loraConfig.channel_num;
        }

        return LoRaPresets::getDefaultSlot(loraConfig.region, loraConfig.modem_preset, effectivePrimaryChannelName());
    }

    void computeActiveRadioParameters(float &frequency, float &bandwidthKhz, uint8_t &spreadFactor, uint8_t &codingRate,
                                      uint32_t &channelNum) const
    {
        frequency = kDefaultFrequencyMhz;
        bandwidthKhz = kDefaultBandwidthKhz;
        spreadFactor = kDefaultSpreadFactor;
        codingRate = kDefaultCodingRate;
        channelNum = effectiveChannelNum();

        if (loraConfig.use_preset) {
            uint32_t numChannels = LoRaPresets::getNumChannels(loraConfig.region, loraConfig.modem_preset);
            if (numChannels > 0) {
                if (channelNum == 0 || channelNum > numChannels) {
                    channelNum = LoRaPresets::getDefaultSlot(loraConfig.region, loraConfig.modem_preset, effectivePrimaryChannelName());
                }
                frequency = LoRaPresets::getRadioFreq(loraConfig.region, loraConfig.modem_preset, channelNum) +
                            loraConfig.frequency_offset;
                if (!tryGetPresetRadioParameters(loraConfig.modem_preset, bandwidthKhz, spreadFactor, codingRate)) {
                    bandwidthKhz = LoRaPresets::getBandwidth(loraConfig.modem_preset) * 1000.0f;
                }
            }
        } else {
            frequency = loraConfig.override_frequency + loraConfig.frequency_offset;
            bandwidthKhz = loraConfig.bandwidth > 0 ? loraConfig.bandwidth : kDefaultBandwidthKhz;
            spreadFactor = loraConfig.spread_factor > 0 ? loraConfig.spread_factor : kDefaultSpreadFactor;
            codingRate = loraConfig.coding_rate > 0 ? loraConfig.coding_rate : kDefaultCodingRate;
        }
    }

    void logLoRaConfigSummary(const char *reason) const
    {
        float frequency = 0.0f;
        float bandwidthKhz = 0.0f;
        uint8_t spreadFactor = 0;
        uint8_t codingRate = 0;
        uint32_t channelNum = 0;
        computeActiveRadioParameters(frequency, bandwidthKhz, spreadFactor, codingRate, channelNum);

        const ExpandedChannelKey key = expandedPrimaryKey();
        const int8_t effectiveTxPower = effectiveTxPowerDbm();
        ILOG_INFO(
            "LoRa %s: region=%u usePreset=%u preset=%u(%s) channel=%lu freq=%.4fMHz bw=%.1fkHz sf=%u cr=%u txPower=%d sync=0x%02x hop=%u txEnabled=%u offset=%.4f name='%s' keyLen=%u keyHash=0x%02x airHash=0x%02x",
            reason, loraConfig.region, loraConfig.use_preset ? 1 : 0, loraConfig.modem_preset,
            LoRaPresets::modemPresetToString(loraConfig.modem_preset), static_cast<unsigned long>(channelNum), frequency,
            bandwidthKhz, spreadFactor, codingRate, effectiveTxPower,
            kDefaultSyncWord, loraConfig.hop_limit, loraConfig.tx_enabled ? 1 : 0, loraConfig.frequency_offset,
            effectivePrimaryChannelName(), static_cast<unsigned>(key.length), xorHash(key.bytes, key.length),
            currentAirChannelHash());
    }

    int8_t effectiveTxPowerDbm() const
    {
        const int8_t configuredTxPower = loraConfig.tx_power ? loraConfig.tx_power : kDefaultTxPowerDbm;
        return std::min<int8_t>(configuredTxPower, kMaxTab5TxPowerDbm);
    }

    void logPacketSummary(const char *prefix, const meshtastic_MeshPacket &packet, size_t encodedLen = 0) const
    {
        const meshtastic_PortNum portnum =
            packet.which_payload_variant == meshtastic_MeshPacket_decoded_tag ? packet.decoded.portnum : meshtastic_PortNum_UNKNOWN_APP;
        const uint32_t dataDest =
            packet.which_payload_variant == meshtastic_MeshPacket_decoded_tag ? packet.decoded.dest : 0;
        const uint32_t dataSource =
            packet.which_payload_variant == meshtastic_MeshPacket_decoded_tag ? packet.decoded.source : 0;
        ILOG_INFO(
            "%s pkt: id=0x%08lx from=0x%08lx to=0x%08lx port=%u(%s) wantAck=%u hopLimit=%u hopStart=%u nextHop=0x%02x relay=0x%02x dataDest=0x%08lx dataSource=0x%08lx payload=%u encoded=%u",
            prefix, static_cast<unsigned long>(packet.id), static_cast<unsigned long>(packet.from),
            static_cast<unsigned long>(packet.to), static_cast<unsigned>(portnum), portNumName(portnum), packet.want_ack ? 1 : 0,
            packet.hop_limit, packet.hop_start, packet.next_hop, packet.relay_node,
            static_cast<unsigned long>(dataDest), static_cast<unsigned long>(dataSource),
            packet.which_payload_variant == meshtastic_MeshPacket_decoded_tag ? static_cast<unsigned>(packet.decoded.payload.size) : 0,
            static_cast<unsigned>(encodedLen));
    }

    void logRawAirHeader(const char *prefix, const PacketHeader &header, size_t payloadLen, int state, float snr, int32_t rssi) const
    {
        ILOG_INFO(
            "%s air: state=%d len=%u from=0x%08lx to=0x%08lx id=0x%08lx flags=0x%02x chHash=0x%02x nextHop=0x%02x relay=0x%02x rssi=%ld snr=%.1f expectedHash=0x%02x learned=%s0x%02x",
            prefix, state, static_cast<unsigned>(payloadLen), static_cast<unsigned long>(header.from),
            static_cast<unsigned long>(header.to), static_cast<unsigned long>(header.id), header.flags, header.channel,
            header.next_hop, header.relay_node, static_cast<long>(rssi), snr, primaryChannelHash(),
            hasLearnedChannelHash ? "" : "n/a ", learnedChannelHash);
    }

    bool encodePacketForAir(const meshtastic_MeshPacket &packet, uint8_t *out, size_t &outLen)
    {
        if (packet.which_payload_variant != meshtastic_MeshPacket_decoded_tag) {
            return false;
        }

        ExpandedChannelKey key = expandedPrimaryKey();
        if (!key.valid) {
            ILOG_WARN("Primary channel key is invalid");
            return false;
        }

                // Meshtastic text traffic mirrors packet.to into Data.dest, including
                // broadcast 0xffffffff. Keep source unset for local TX to match upstream.
        meshtastic_Data encodedData = packet.decoded;
        if (encodedData.portnum == meshtastic_PortNum_TEXT_MESSAGE_APP && encodedData.dest == 0) {
            encodedData.dest = packet.to;
        }
        if (encodedData.has_bitfield && encodedData.bitfield == 0) {
            encodedData.has_bitfield = false;
        }
        uint8_t plainData[kMaxAirPayload] = {0};
        const size_t payloadLen = pb_encode_to_bytes(plainData, sizeof(plainData), &meshtastic_Data_msg, &encodedData);
        if (payloadLen == 0) {
            return false;
        }

        uint8_t encrypted[kMaxAirPayload] = {0};
        std::memcpy(encrypted, plainData, payloadLen);

        const uint32_t airPacketId = packet.id;

        if (!cryptAesCtr(key.bytes, key.length, packet.from, airPacketId, encrypted, payloadLen)) {
            ILOG_WARN("Failed to encrypt MeshPacket payload");
            return false;
        }

        PacketHeader header{};
        header.to = packet.to;
        header.from = packet.from;
        header.id = airPacketId;

        uint8_t hopLimit = packet.hop_limit ? packet.hop_limit : loraConfig.hop_limit;
        if (hopLimit > 7) {
            hopLimit = 3;
        }

        uint8_t hopStart = packet.hop_start ? packet.hop_start : hopLimit;
        if (hopStart > 7) {
            hopStart = hopLimit;
        }
        header.flags = hopLimit | (packet.want_ack ? kPacketFlagsWantAckMask : 0) |
                       (packet.via_mqtt ? kPacketFlagsViaMqttMask : 0) |
                       static_cast<uint8_t>((hopStart << kPacketFlagsHopStartShift) & kPacketFlagsHopStartMask);
        header.channel = currentAirChannelHash();
        header.next_hop = packet.next_hop;
        header.relay_node = packet.relay_node ? packet.relay_node : static_cast<uint8_t>(packet.from & 0xFF);

        outLen = sizeof(header) + payloadLen;
        std::memcpy(out, &header, sizeof(header));
        std::memcpy(out + sizeof(header), encrypted, payloadLen);
        return true;
    }

    bool decodePacketFromAir(const uint8_t *payload, size_t payloadLen, float snr, int32_t rssi, meshtastic_MeshPacket &packet)
    {
        if (payloadLen < sizeof(PacketHeader)) {
            return false;
        }

        PacketHeader header{};
        std::memcpy(&header, payload, sizeof(header));
        if (header.from == 0) {
            return false;
        }

        const size_t encryptedLen = payloadLen - sizeof(PacketHeader);
        if (encryptedLen > sizeof(packet.decoded.payload.bytes)) {
            return false;
        }

        ExpandedChannelKey key = expandedPrimaryKey();
        if (!key.valid) {
            ILOG_WARN("Drop air packet: invalid primary channel key");
            return false;
        }

        const uint8_t expectedHash = primaryChannelHash();
        const bool hashMatches = (header.channel == expectedHash) || (hasLearnedChannelHash && header.channel == learnedChannelHash);
        if (!hashMatches) {
            ILOG_WARN("Air channel hash mismatch: rx=0x%02x expected=0x%02x learned=%s0x%02x (trying decode fallback)",
                      header.channel, expectedHash, hasLearnedChannelHash ? "" : "n/a ", learnedChannelHash);
        }

        uint8_t decodedBytes[kMaxAirPayload] = {0};
        std::memcpy(decodedBytes, payload + sizeof(PacketHeader), encryptedLen);
        if (!cryptAesCtr(key.bytes, key.length, header.from, header.id, decodedBytes, encryptedLen)) {
            ILOG_WARN("Failed to decrypt MeshPacket payload");
            return false;
        }

        packet = meshtastic_MeshPacket_init_default;
        packet.from = header.from;
        packet.to = header.to;
        packet.id = header.id;
        packet.channel = 0;
        packet.hop_limit = header.flags & kPacketFlagsHopLimitMask;
        packet.hop_start = (header.flags & kPacketFlagsHopStartMask) >> kPacketFlagsHopStartShift;
        packet.want_ack = (header.flags & kPacketFlagsWantAckMask) != 0;
        packet.via_mqtt = (header.flags & kPacketFlagsViaMqttMask) != 0;
        packet.next_hop = packet.hop_start == 0 ? 0 : header.next_hop;
        packet.relay_node = packet.hop_start == 0 ? 0 : header.relay_node;
        packet.rx_snr = snr;
        packet.rx_rssi = rssi;
        packet.rx_time = millis() / 1000;
        packet.which_payload_variant = meshtastic_MeshPacket_decoded_tag;
        packet.decoded = meshtastic_Data_init_default;

        if (!pb_decode_from_bytes(decodedBytes, encryptedLen, &meshtastic_Data_msg, &packet.decoded)) {
            ILOG_WARN("Drop air packet: protobuf decode failed (len=%u)", static_cast<unsigned>(encryptedLen));
            return false;
        }

        if (packet.decoded.has_bitfield) {
            packet.decoded.want_response = packet.decoded.want_response ||
                                           ((packet.decoded.bitfield & kBitfieldWantResponseMask) != 0);
        }

        if (!hashMatches) {
            learnedChannelHash = header.channel;
            hasLearnedChannelHash = true;
            ILOG_INFO("Learned remote air channel hash: 0x%02x", learnedChannelHash);
        }

        return true;
    }

    void buildDefaults()
    {
        localUser = meshtastic_User_init_default;
        buildNodeId(localUser.id, sizeof(localUser.id));
        localUser.hw_model = meshtastic_HardwareModel_PRIVATE_HW;
        localUser.role = meshtastic_Config_DeviceConfig_Role_CLIENT;

        myInfo = meshtastic_MyNodeInfo_init_default;
        myInfo.my_node_num = localNodeNum();
        myInfo.reboot_count = 1;
        myInfo.min_app_version = 0;
        myInfo.nodedb_count = 1;
        std::snprintf(myInfo.pio_env, sizeof(myInfo.pio_env), "%s", kEnvName);
        myInfo.firmware_edition = meshtastic_FirmwareEdition_VANILLA;
        myInfo.device_id.size = 8;
        std::memcpy(myInfo.device_id.bytes, "TAB5L001", 8);

        metadata = meshtastic_DeviceMetadata_init_default;
        std::snprintf(metadata.firmware_version, sizeof(metadata.firmware_version), "%s", kFirmwareVersion);
        metadata.device_state_version = 1;
        metadata.canShutdown = true;
        metadata.hasBluetooth = false;
        metadata.hasWifi = false;
        metadata.hasEthernet = false;
        metadata.role = meshtastic_Config_DeviceConfig_Role_CLIENT;
        metadata.hw_model = meshtastic_HardwareModel_PRIVATE_HW;
        metadata.hasRemoteHardware = false;
        metadata.hasPKC = false;

        deviceUiConfig = meshtastic_DeviceUIConfig_init_default;
        deviceUiConfig.screen_brightness = 75;
        deviceUiConfig.screen_timeout = 60;
        deviceUiConfig.theme = meshtastic_Theme_DARK;
        deviceUiConfig.alert_enabled = true;
        deviceUiConfig.banner_enabled = true;
        deviceUiConfig.language = meshtastic_Language_ENGLISH;
        deviceUiConfig.screen_rgb_color = 0x3DDA83;
        deviceUiConfig.compass_mode = meshtastic_CompassMode_DYNAMIC;

        deviceConfig = meshtastic_Config_DeviceConfig_init_default;
        deviceConfig.role = meshtastic_Config_DeviceConfig_Role_CLIENT;
        deviceConfig.serial_enabled = true;
        deviceConfig.node_info_broadcast_secs = 900;

        applyLocalUserDefaults(false);

        positionConfig = meshtastic_Config_PositionConfig_init_default;
        positionConfig.gps_enabled = false;
        positionConfig.position_broadcast_secs = 900;
        positionConfig.position_broadcast_smart_enabled = false;

        powerConfig = meshtastic_Config_PowerConfig_init_default;
        powerConfig.wait_bluetooth_secs = 0;

        networkConfig = meshtastic_Config_NetworkConfig_init_default;
        networkConfig.wifi_enabled = false;
        networkConfig.eth_enabled = false;

        displayConfig = meshtastic_Config_DisplayConfig_init_default;
        displayConfig.screen_on_secs = UINT32_MAX;
        displayConfig.auto_screen_carousel_secs = 0;
        displayConfig.displaymode = meshtastic_Config_DisplayConfig_DisplayMode_DEFAULT;
        displayConfig.use_long_node_name = true;

        loraConfig = meshtastic_Config_LoRaConfig_init_default;
        loraConfig.use_preset = true;
        loraConfig.modem_preset = meshtastic_Config_LoRaConfig_ModemPreset_LONG_SLOW;
        loraConfig.bandwidth = static_cast<uint16_t>(kDefaultBandwidthKhz);
        loraConfig.spread_factor = kDefaultSpreadFactor;
        loraConfig.coding_rate = kDefaultCodingRate;
        loraConfig.region = meshtastic_Config_LoRaConfig_RegionCode_EU_868;
        loraConfig.hop_limit = 3;
        loraConfig.tx_enabled = true;
        loraConfig.tx_power = kDefaultTxPowerDbm;
        loraConfig.channel_num = 0;
        loraConfig.sx126x_rx_boosted_gain = true;
        loraConfig.override_frequency = kDefaultFrequencyMhz;

        bluetoothConfig = meshtastic_Config_BluetoothConfig_init_default;
        bluetoothConfig.enabled = false;

        securityConfig = meshtastic_Config_SecurityConfig_init_default;
        securityConfig.serial_enabled = true;
        securityConfig.admin_channel_enabled = true;

        for (size_t i = 0; i < kMaxChannels; ++i) {
            channels[i] = meshtastic_Channel_init_default;
            channels[i].index = i;
            channels[i].role = meshtastic_Channel_Role_DISABLED;
            channels[i].has_settings = false;
        }
        channels[0].has_settings = true;
        channels[0].role = meshtastic_Channel_Role_PRIMARY;
        channels[0].settings.channel_num = 0;
        channels[0].settings.psk.size = 1;
        channels[0].settings.psk.bytes[0] = 1;
        channels[0].settings.name[0] = '\0';
        channels[0].settings.uplink_enabled = true;
        channels[0].settings.downlink_enabled = true;

        telemetryConfig = meshtastic_ModuleConfig_init_default;
        telemetryConfig.which_payload_variant = meshtastic_ModuleConfig_telemetry_tag;
        telemetryConfig.payload_variant.telemetry = meshtastic_ModuleConfig_TelemetryConfig_init_default;

        cannedMessageConfig = meshtastic_ModuleConfig_init_default;
        cannedMessageConfig.which_payload_variant = meshtastic_ModuleConfig_canned_message_tag;
        cannedMessageConfig.payload_variant.canned_message = meshtastic_ModuleConfig_CannedMessageConfig_init_default;

        connectionStatus = meshtastic_DeviceConnectionStatus_init_default;
        connectionStatus.has_serial = true;
        std::snprintf(currentRingtone, sizeof(currentRingtone), "%s", kSilentRingtone);

        knownNodes.clear();
        KnownNode selfNode;
        selfNode.num = localNodeNum();
        selfNode.user = localUser;
        selfNode.lastHeard = millis() / 1000;
        knownNodes.push_back(selfNode);
    }

    void normalizeState()
    {
        applyLocalUserDefaults(true);

        if (loraConfig.region == meshtastic_Config_LoRaConfig_RegionCode_UNSET) {
            loraConfig.region = meshtastic_Config_LoRaConfig_RegionCode_EU_868;
        }
        if (loraConfig.use_preset && loraConfig.channel_num == 0) {
            loraConfig.channel_num = LoRaPresets::getDefaultSlot(loraConfig.region, loraConfig.modem_preset, effectivePrimaryChannelName());
        }
        if (!loraConfig.use_preset && loraConfig.override_frequency == 0.0f) {
            loraConfig.override_frequency = kDefaultFrequencyMhz;
        }
        if (loraConfig.tx_power == 0) {
            loraConfig.tx_power = kDefaultTxPowerDbm;
        }

        int primaryIdx = -1;
        for (size_t i = 0; i < kMaxChannels; ++i) {
            channels[i].index = i;
            if (channels[i].role == meshtastic_Channel_Role_PRIMARY) {
                if (primaryIdx < 0) {
                    primaryIdx = static_cast<int>(i);
                } else {
                    channels[i].role = meshtastic_Channel_Role_SECONDARY;
                }
            }
        }

        if (primaryIdx < 0) {
            primaryIdx = 0;
            channels[0].role = meshtastic_Channel_Role_PRIMARY;
        }

        auto &primary = channels[primaryIdx];
        primary.has_settings = true;
        if (primary.settings.psk.size == 0) {
            primary.settings.psk.size = 1;
            primary.settings.psk.bytes[0] = 1;
        }
        if (std::strcmp(primary.settings.name, kPrimaryChannelName) == 0) {
            primary.settings.name[0] = '\0';
        }
        if (loraConfig.use_preset &&
            std::strcmp(primary.settings.name, LoRaPresets::modemPresetToString(loraConfig.modem_preset)) == 0) {
            primary.settings.name[0] = '\0';
        }

        if (currentRingtone[0] == '\0') {
            std::snprintf(currentRingtone, sizeof(currentRingtone), "%s", kSilentRingtone);
        }

        metadata.role = deviceConfig.role;
        localUser.role = deviceConfig.role;
        updateSelfNode();
    }

    bool loadState()
    {
        if (!LittleFS.exists(kStateFilePath)) {
            normalizeState();
            return false;
        }

        File file = LittleFS.open(kStateFilePath, FILE_READ);
        if (!file) {
            ILOG_WARN("Failed to open state file for read");
            normalizeState();
            return false;
        }

        PersistedStateHeader header;
        const size_t headerRead = file.read(reinterpret_cast<uint8_t *>(&header), sizeof(header));
        file.close();

        if (headerRead != sizeof(header) || header.magic != kStateMagic) {
            ILOG_WARN("Ignoring invalid persisted state header");
            normalizeState();
            return false;
        }

        if (header.version == kStateVersionV1) {
            PersistedStateV1 oldState;
            file = LittleFS.open(kStateFilePath, FILE_READ);
            if (!file) {
                ILOG_WARN("Failed to reopen state file for v1 migration");
                normalizeState();
                return false;
            }
            const size_t bytesRead = file.read(reinterpret_cast<uint8_t *>(&oldState), sizeof(oldState));
            file.close();
            if (bytesRead != sizeof(oldState)) {
                ILOG_WARN("Ignoring truncated v1 persisted state");
                normalizeState();
                return false;
            }

            localUser = oldState.localUser;
            deviceUiConfig = oldState.deviceUiConfig;
            deviceConfig = oldState.deviceConfig;
            positionConfig = oldState.positionConfig;
            powerConfig = oldState.powerConfig;
            networkConfig = oldState.networkConfig;
            displayConfig = oldState.displayConfig;
            loraConfig = oldState.loraConfig;
            bluetoothConfig = oldState.bluetoothConfig;
            securityConfig = oldState.securityConfig;
            for (size_t i = 0; i < kMaxChannels; ++i) {
                channels[i] = meshtastic_Channel_init_default;
                channels[i].index = i;
                channels[i].role = meshtastic_Channel_Role_DISABLED;
                channels[i].has_settings = false;
            }
            channels[0] = oldState.primaryChannel;
            channels[0].index = 0;
            if (channels[0].role == meshtastic_Channel_Role_DISABLED) {
                channels[0].role = meshtastic_Channel_Role_PRIMARY;
            }
            telemetryConfig = oldState.telemetryConfig;
            cannedMessageConfig = oldState.cannedMessageConfig;
            std::memcpy(currentRingtone, oldState.currentRingtone, sizeof(currentRingtone));

            normalizeState();
            saveState();
            ILOG_INFO("Migrated persisted Tab5 node state v1 -> v2");
            return true;
        }

        PersistedState state;
        file = LittleFS.open(kStateFilePath, FILE_READ);
        if (!file) {
            ILOG_WARN("Failed to reopen state file for v2 load");
            normalizeState();
            return false;
        }
        const size_t bytesRead = file.read(reinterpret_cast<uint8_t *>(&state), sizeof(state));
        file.close();
        if (bytesRead != sizeof(state) || state.header.version != kStateVersion) {
            ILOG_WARN("Ignoring invalid persisted state payload");
            normalizeState();
            return false;
        }

        localUser = state.localUser;
        deviceUiConfig = state.deviceUiConfig;
        deviceConfig = state.deviceConfig;
        positionConfig = state.positionConfig;
        powerConfig = state.powerConfig;
        networkConfig = state.networkConfig;
        displayConfig = state.displayConfig;
        loraConfig = state.loraConfig;
        bluetoothConfig = state.bluetoothConfig;
        securityConfig = state.securityConfig;
        for (size_t i = 0; i < kMaxChannels; ++i) {
            channels[i] = state.channels[i];
        }
        telemetryConfig = state.telemetryConfig;
        cannedMessageConfig = state.cannedMessageConfig;
        std::memcpy(currentRingtone, state.currentRingtone, sizeof(currentRingtone));

        normalizeState();
        ILOG_INFO("Loaded persisted Tab5 node state");
        return true;
    }

    bool saveState()
    {
        normalizeState();

        PersistedState state;
        state.localUser = localUser;
        state.deviceUiConfig = deviceUiConfig;
        state.deviceConfig = deviceConfig;
        state.positionConfig = positionConfig;
        state.powerConfig = powerConfig;
        state.networkConfig = networkConfig;
        state.displayConfig = displayConfig;
        state.loraConfig = loraConfig;
        state.bluetoothConfig = bluetoothConfig;
        state.securityConfig = securityConfig;
        for (size_t i = 0; i < kMaxChannels; ++i) {
            state.channels[i] = channels[i];
        }
        state.telemetryConfig = telemetryConfig;
        state.cannedMessageConfig = cannedMessageConfig;
        std::memcpy(state.currentRingtone, currentRingtone, sizeof(currentRingtone));

        File file = LittleFS.open(kStateFilePath, "w");
        if (!file) {
            ILOG_ERROR("Failed to open state file for write");
            return false;
        }

        const size_t expectedSize = sizeof(state);
        const size_t bytesWritten = file.write(reinterpret_cast<const uint8_t *>(&state), expectedSize);
        file.flush();
        file.close();

        if (bytesWritten != expectedSize) {
            ILOG_ERROR("Failed to persist full state (%u/%u)", static_cast<unsigned>(bytesWritten),
                       static_cast<unsigned>(expectedSize));
            return false;
        }

        ILOG_INFO("Persisted Tab5 node state");
        return true;
    }

    void processUiQueue()
    {
        while (packetServer && packetServer->hasData()) {
            auto packet = packetServer->receivePacket();
            if (!packet) {
                break;
            }

            auto *typed = static_cast<DataPacket<meshtastic_ToRadio> *>(packet.get());
            handleToRadio(typed->getData());
        }
    }

    void handleToRadio(const meshtastic_ToRadio &to)
    {
        switch (to.which_payload_variant) {
        case meshtastic_ToRadio_want_config_id_tag:
            sendFullConfig(to.want_config_id);
            break;
        case meshtastic_ToRadio_packet_tag:
            handleMeshPacket(to.packet);
            break;
        case meshtastic_ToRadio_heartbeat_tag:
            break;
        case meshtastic_ToRadio_disconnect_tag:
            break;
        default:
            ILOG_WARN("Unhandled ToRadio variant: %u", to.which_payload_variant);
            break;
        }
    }

    void handleMeshPacket(meshtastic_MeshPacket packet)
    {
        if (packet.from == 0) {
            packet.from = localNodeNum();
        }
        if (packet.id == 0) {
            packet.id = nextMeshPacketId++;
        }

        if (packet.to == UINT32_MAX || packet.to == 0) {
            packet.want_ack = false;
        }

        if (packet.which_payload_variant == meshtastic_MeshPacket_decoded_tag &&
            packet.decoded.portnum == meshtastic_PortNum_ADMIN_APP &&
            (packet.to == 0 || packet.to == localNodeNum())) {
            logPacketSummary("LOCAL-ADMIN", packet);
            handleAdminPacket(packet);
            return;
        }

        if (packet.want_ack && packet.to != 0 && packet.to != UINT32_MAX && packet.to != localNodeNum()) {
            queueReliablePacket(packet);
        }

        maybeBroadcastNodeInfoBeforeText(packet);
        transmitMeshPacket(packet);
    }

    void handleAdminPacket(const meshtastic_MeshPacket &packet)
    {
        scratchAdminMessage = meshtastic_AdminMessage_init_default;
        if (!pb_decode_from_bytes(packet.decoded.payload.bytes, packet.decoded.payload.size, &meshtastic_AdminMessage_msg,
                                  &scratchAdminMessage)) {
            ILOG_WARN("Admin payload decode failed");
            return;
        }

        switch (scratchAdminMessage.which_payload_variant) {
        case meshtastic_AdminMessage_get_channel_request_tag:
            sendChannel(channels[scratchAdminMessage.get_channel_request < kMaxChannels ? scratchAdminMessage.get_channel_request : 0]);
            break;
        case meshtastic_AdminMessage_get_owner_request_tag:
            sendNodeInfo(knownNodes.front());
            break;
        case meshtastic_AdminMessage_get_config_request_tag:
            sendConfigResponse(scratchAdminMessage.get_config_request);
            break;
        case meshtastic_AdminMessage_get_module_config_request_tag:
            sendModuleConfigResponse(scratchAdminMessage.get_module_config_request);
            break;
        case meshtastic_AdminMessage_get_device_metadata_request_tag:
            sendMetadata();
            break;
        case meshtastic_AdminMessage_get_ringtone_request_tag:
            sendRingtoneResponse(packet);
            break;
        case meshtastic_AdminMessage_get_device_connection_status_request_tag:
            sendConnectionStatusResponse(packet);
            break;
        case meshtastic_AdminMessage_get_ui_config_request_tag:
            sendDeviceUiConfig();
            break;
        case meshtastic_AdminMessage_set_owner_tag:
            if (scratchAdminMessage.set_owner.long_name[0] != '\0') {
                std::snprintf(localUser.long_name, sizeof(localUser.long_name), "%s", scratchAdminMessage.set_owner.long_name);
            }
            applyLocalUserDefaults(true);
            updateSelfNode();
            saveState();
            sendNodeInfo(knownNodes.front());
            break;
        case meshtastic_AdminMessage_set_channel_tag:
        {
            const uint32_t idx = scratchAdminMessage.set_channel.index < kMaxChannels ? scratchAdminMessage.set_channel.index : 0;
            channels[idx] = scratchAdminMessage.set_channel;
            channels[idx].index = idx;
            if (channels[idx].role == meshtastic_Channel_Role_PRIMARY) {
                for (size_t i = 0; i < kMaxChannels; ++i) {
                    if (i != idx && channels[i].role == meshtastic_Channel_Role_PRIMARY) {
                        channels[i].role = meshtastic_Channel_Role_SECONDARY;
                    }
                }
            }
            saveState();
            sendChannel(channels[idx]);
            applyLoRaConfigToRadio();
            break;
        }
        case meshtastic_AdminMessage_set_config_tag:
            applyConfig(scratchAdminMessage.set_config);
            saveState();
            sendConfigResponse(configTypeFor(scratchAdminMessage.set_config));
            break;
        case meshtastic_AdminMessage_set_module_config_tag:
            applyModuleConfig(scratchAdminMessage.set_module_config);
            saveState();
            sendModuleConfig(scratchAdminMessage.set_module_config);
            break;
        case meshtastic_AdminMessage_set_ringtone_message_tag:
            std::snprintf(currentRingtone, sizeof(currentRingtone), "%s", scratchAdminMessage.set_ringtone_message);
            saveState();
            sendRingtoneResponse(packet);
            break;
        case meshtastic_AdminMessage_store_ui_config_tag:
            deviceUiConfig = scratchAdminMessage.store_ui_config;
            saveState();
            sendDeviceUiConfig();
            break;
        case meshtastic_AdminMessage_reboot_seconds_tag:
            sendRebooted();
            scheduleSystemAction(PendingSystemAction::Reboot, scratchAdminMessage.reboot_seconds);
            break;
        case meshtastic_AdminMessage_reboot_ota_seconds_tag:
            sendRebooted();
            scheduleSystemAction(PendingSystemAction::Reboot, scratchAdminMessage.reboot_ota_seconds);
            break;
        case meshtastic_AdminMessage_shutdown_seconds_tag:
            scheduleSystemAction(PendingSystemAction::Shutdown, scratchAdminMessage.shutdown_seconds);
            break;
        case meshtastic_AdminMessage_factory_reset_config_tag:
            sendRebooted();
            scheduleSystemAction(PendingSystemAction::FactoryReset, 0);
            break;
        case meshtastic_AdminMessage_nodedb_reset_tag:
            sendRebooted();
            scheduleSystemAction(PendingSystemAction::NodeDbReset, 0);
            break;
        default:
            ILOG_WARN("Unhandled admin variant: %u", scratchAdminMessage.which_payload_variant);
            break;
        }
    }

    void applyConfig(const meshtastic_Config &config)
    {
        switch (config.which_payload_variant) {
        case meshtastic_Config_device_tag:
            deviceConfig = config.payload_variant.device;
            metadata.role = deviceConfig.role;
            localUser.role = deviceConfig.role;
            updateSelfNode();
            break;
        case meshtastic_Config_position_tag:
            positionConfig = config.payload_variant.position;
            break;
        case meshtastic_Config_power_tag:
            powerConfig = config.payload_variant.power;
            break;
        case meshtastic_Config_network_tag:
            networkConfig = config.payload_variant.network;
            break;
        case meshtastic_Config_display_tag:
            displayConfig = config.payload_variant.display;
            break;
        case meshtastic_Config_lora_tag:
            loraConfig = config.payload_variant.lora;
            if (loraConfig.region == meshtastic_Config_LoRaConfig_RegionCode_UNSET) {
                loraConfig.region = meshtastic_Config_LoRaConfig_RegionCode_EU_868;
            }
            if (loraConfig.use_preset && loraConfig.channel_num == 0) {
                loraConfig.channel_num = LoRaPresets::getDefaultSlot(loraConfig.region, loraConfig.modem_preset,
                                                                     effectivePrimaryChannelName());
            }
            if (!loraConfig.use_preset && loraConfig.override_frequency == 0.0f) {
                loraConfig.override_frequency = kDefaultFrequencyMhz;
            }
            applyLoRaConfigToRadio();
            break;
        case meshtastic_Config_bluetooth_tag:
            bluetoothConfig = config.payload_variant.bluetooth;
            break;
        case meshtastic_Config_security_tag:
            securityConfig = config.payload_variant.security;
            break;
        case meshtastic_Config_device_ui_tag:
            deviceUiConfig = config.payload_variant.device_ui;
            sendDeviceUiConfig();
            break;
        default:
            break;
        }
    }

    void applyModuleConfig(const meshtastic_ModuleConfig &config)
    {
        switch (config.which_payload_variant) {
        case meshtastic_ModuleConfig_telemetry_tag:
            telemetryConfig = config;
            break;
        case meshtastic_ModuleConfig_canned_message_tag:
            cannedMessageConfig = config;
            break;
        default:
            break;
        }
    }

    meshtastic_AdminMessage_ConfigType configTypeFor(const meshtastic_Config &config) const
    {
        switch (config.which_payload_variant) {
        case meshtastic_Config_device_tag:
            return meshtastic_AdminMessage_ConfigType_DEVICE_CONFIG;
        case meshtastic_Config_position_tag:
            return meshtastic_AdminMessage_ConfigType_POSITION_CONFIG;
        case meshtastic_Config_power_tag:
            return meshtastic_AdminMessage_ConfigType_POWER_CONFIG;
        case meshtastic_Config_network_tag:
            return meshtastic_AdminMessage_ConfigType_NETWORK_CONFIG;
        case meshtastic_Config_display_tag:
            return meshtastic_AdminMessage_ConfigType_DISPLAY_CONFIG;
        case meshtastic_Config_lora_tag:
            return meshtastic_AdminMessage_ConfigType_LORA_CONFIG;
        case meshtastic_Config_bluetooth_tag:
            return meshtastic_AdminMessage_ConfigType_BLUETOOTH_CONFIG;
        case meshtastic_Config_security_tag:
            return meshtastic_AdminMessage_ConfigType_SECURITY_CONFIG;
        case meshtastic_Config_device_ui_tag:
            return meshtastic_AdminMessage_ConfigType_DEVICEUI_CONFIG;
        default:
            return meshtastic_AdminMessage_ConfigType_DEVICE_CONFIG;
        }
    }

    void sendFullConfig(uint32_t configId)
    {
        sendDeviceUiConfig();
        sendMyInfo();
        updateSelfNode();
        for (const auto &node : knownNodes) {
            sendNodeInfo(node);
        }
        sendConfigResponse(meshtastic_AdminMessage_ConfigType_DEVICE_CONFIG);
        sendConfigResponse(meshtastic_AdminMessage_ConfigType_POSITION_CONFIG);
        sendConfigResponse(meshtastic_AdminMessage_ConfigType_POWER_CONFIG);
        sendConfigResponse(meshtastic_AdminMessage_ConfigType_NETWORK_CONFIG);
        sendConfigResponse(meshtastic_AdminMessage_ConfigType_DISPLAY_CONFIG);
        sendConfigResponse(meshtastic_AdminMessage_ConfigType_LORA_CONFIG);
        sendConfigResponse(meshtastic_AdminMessage_ConfigType_BLUETOOTH_CONFIG);
        sendConfigResponse(meshtastic_AdminMessage_ConfigType_SECURITY_CONFIG);
        sendModuleConfig(telemetryConfig);
        sendModuleConfig(cannedMessageConfig);
        for (size_t i = 0; i < kMaxChannels; ++i) {
            sendChannel(channels[i]);
        }
        sendMetadata();

        meshtastic_FromRadio from = meshtastic_FromRadio_init_default;
        from.which_payload_variant = meshtastic_FromRadio_config_complete_id_tag;
        from.config_complete_id = configId;
        queueFromRadio(from);
    }

    void sendDeviceUiConfig()
    {
        auto &from = prepareFromRadio(meshtastic_FromRadio_deviceuiConfig_tag);
        from.deviceuiConfig = deviceUiConfig;
        queueFromRadio(from);
    }

    void sendMyInfo()
    {
        myInfo.nodedb_count = static_cast<uint16_t>(knownNodes.size());
        auto &from = prepareFromRadio(meshtastic_FromRadio_my_info_tag);
        from.my_info = myInfo;
        queueFromRadio(from);
    }

    void sendNodeInfo(const KnownNode &node)
    {
        auto &from = prepareFromRadio(meshtastic_FromRadio_node_info_tag);
        from.node_info = meshtastic_NodeInfo_init_default;
        from.node_info.num = node.num;
        from.node_info.has_user = true;
        from.node_info.user = node.user;
        from.node_info.last_heard = node.lastHeard;
        from.node_info.snr = node.snr;
        queueFromRadio(from);
    }

    void maybeBroadcastLocalNodeInfo()
    {
        const uint32_t intervalSecs = deviceConfig.node_info_broadcast_secs;
        if (intervalSecs == 0) {
            return;
        }

        const uint32_t nowMs = millis();
        const uint32_t intervalMs = intervalSecs * 1000UL;
        if (lastNodeInfoBroadcastMs != 0 && (nowMs - lastNodeInfoBroadcastMs) < intervalMs) {
            return;
        }

        broadcastLocalNodeInfo("periodic");
    }

    void maybeBroadcastNodeInfoBeforeText(const meshtastic_MeshPacket &packet)
    {
        if (packet.which_payload_variant != meshtastic_MeshPacket_decoded_tag ||
            packet.decoded.portnum != meshtastic_PortNum_TEXT_MESSAGE_APP) {
            return;
        }

        const uint32_t nowMs = millis();
        if (lastNodeInfoBroadcastMs != 0 &&
            (nowMs - lastNodeInfoBroadcastMs) < kInteractiveNodeInfoMinIntervalMs) {
            return;
        }

        ILOG_INFO("Broadcast NODEINFO before TEXT: textId=0x%08lx", static_cast<unsigned long>(packet.id));
        broadcastLocalNodeInfo("pre-text");
    }

    void broadcastLocalNodeInfo(const char *reason)
    {
        meshtastic_User userPayload = localUser;
        buildNodeId(userPayload.id, sizeof(userPayload.id));

        scratchTxPacket = meshtastic_MeshPacket_init_default;
        scratchTxPacket.from = localNodeNum();
        scratchTxPacket.to = UINT32_MAX;
        scratchTxPacket.id = nextMeshPacketId++;
        scratchTxPacket.want_ack = false;
        scratchTxPacket.which_payload_variant = meshtastic_MeshPacket_decoded_tag;
        scratchTxPacket.decoded = meshtastic_Data_init_default;
        scratchTxPacket.decoded.portnum = meshtastic_PortNum_NODEINFO_APP;

        const size_t payloadLen =
            pb_encode_to_bytes(scratchTxPacket.decoded.payload.bytes, sizeof(scratchTxPacket.decoded.payload.bytes),
                               &meshtastic_User_msg, &userPayload);
        if (payloadLen == 0) {
            ILOG_WARN("NODEINFO encode failed");
            return;
        }

        scratchTxPacket.decoded.payload.size = payloadLen;
        ILOG_INFO("Broadcast NODEINFO (%s): id=0x%08lx", reason, static_cast<unsigned long>(scratchTxPacket.id));
        transmitMeshPacket(scratchTxPacket);
        lastNodeInfoBroadcastMs = millis();
    }

    void sendMetadata()
    {
        auto &from = prepareFromRadio(meshtastic_FromRadio_metadata_tag);
        from.metadata = metadata;
        queueFromRadio(from);
    }

    void sendChannel(const meshtastic_Channel &channel)
    {
        auto &from = prepareFromRadio(meshtastic_FromRadio_channel_tag);
        from.channel = channel;
        queueFromRadio(from);
    }

    void sendModuleConfigResponse(meshtastic_AdminMessage_ModuleConfigType type)
    {
        switch (type) {
        case meshtastic_AdminMessage_ModuleConfigType_TELEMETRY_CONFIG:
            sendModuleConfig(telemetryConfig);
            break;
        case meshtastic_AdminMessage_ModuleConfigType_CANNEDMSG_CONFIG:
            sendModuleConfig(cannedMessageConfig);
            break;
        default:
            break;
        }
    }

    void sendModuleConfig(const meshtastic_ModuleConfig &module)
    {
        auto &from = prepareFromRadio(meshtastic_FromRadio_moduleConfig_tag);
        from.moduleConfig = module;
        queueFromRadio(from);
    }

    void sendConfigResponse(meshtastic_AdminMessage_ConfigType type)
    {
        auto &from = prepareFromRadio(meshtastic_FromRadio_config_tag);
        from.config = meshtastic_Config_init_default;

        switch (type) {
        case meshtastic_AdminMessage_ConfigType_DEVICE_CONFIG:
            from.config.which_payload_variant = meshtastic_Config_device_tag;
            from.config.payload_variant.device = deviceConfig;
            break;
        case meshtastic_AdminMessage_ConfigType_POSITION_CONFIG:
            from.config.which_payload_variant = meshtastic_Config_position_tag;
            from.config.payload_variant.position = positionConfig;
            break;
        case meshtastic_AdminMessage_ConfigType_POWER_CONFIG:
            from.config.which_payload_variant = meshtastic_Config_power_tag;
            from.config.payload_variant.power = powerConfig;
            break;
        case meshtastic_AdminMessage_ConfigType_NETWORK_CONFIG:
            from.config.which_payload_variant = meshtastic_Config_network_tag;
            from.config.payload_variant.network = networkConfig;
            break;
        case meshtastic_AdminMessage_ConfigType_DISPLAY_CONFIG:
            from.config.which_payload_variant = meshtastic_Config_display_tag;
            from.config.payload_variant.display = displayConfig;
            break;
        case meshtastic_AdminMessage_ConfigType_LORA_CONFIG:
            from.config.which_payload_variant = meshtastic_Config_lora_tag;
            from.config.payload_variant.lora = loraConfig;
            break;
        case meshtastic_AdminMessage_ConfigType_BLUETOOTH_CONFIG:
            from.config.which_payload_variant = meshtastic_Config_bluetooth_tag;
            from.config.payload_variant.bluetooth = bluetoothConfig;
            break;
        case meshtastic_AdminMessage_ConfigType_SECURITY_CONFIG:
            from.config.which_payload_variant = meshtastic_Config_security_tag;
            from.config.payload_variant.security = securityConfig;
            break;
        case meshtastic_AdminMessage_ConfigType_DEVICEUI_CONFIG:
            from.config.which_payload_variant = meshtastic_Config_device_ui_tag;
            from.config.payload_variant.device_ui = deviceUiConfig;
            break;
        default:
            return;
        }

        queueFromRadio(from);
    }

    void queueMeshPacket(const meshtastic_MeshPacket &packet)
    {
        auto &from = prepareFromRadio(meshtastic_FromRadio_packet_tag);
        from.packet = packet;
        queueFromRadio(from);
    }

    meshtastic_FromRadio &prepareFromRadio(pb_size_t payloadVariant)
    {
        scratchFromRadio = meshtastic_FromRadio_init_default;
        scratchFromRadio.which_payload_variant = payloadVariant;
        return scratchFromRadio;
    }

    void queueFromRadio(meshtastic_FromRadio &from)
    {
        from.id = nextFromRadioId++;
        packetServer->sendPacket(DataPacket<meshtastic_FromRadio>(from.id, from));
    }

    bool shouldSendRoutingAck(const meshtastic_MeshPacket &packet) const
    {
        if (!packet.want_ack || packet.from == 0 || packet.from == localNodeNum()) {
            return false;
        }

        return packet.to == localNodeNum() || packet.to == UINT32_MAX || packet.to == 0;
    }

    void sendRoutingAck(const meshtastic_MeshPacket &request, meshtastic_Routing_Error error = meshtastic_Routing_Error_NONE)
    {
        meshtastic_Routing routing = meshtastic_Routing_init_default;
        routing.which_variant = meshtastic_Routing_error_reason_tag;
        routing.error_reason = error;

        scratchTxPacket = meshtastic_MeshPacket_init_default;
        scratchTxPacket.from = localNodeNum();
        scratchTxPacket.to = request.from;
        scratchTxPacket.channel = request.channel;
        scratchTxPacket.id = nextMeshPacketId++;
        scratchTxPacket.want_ack = false;
        scratchTxPacket.which_payload_variant = meshtastic_MeshPacket_decoded_tag;
        scratchTxPacket.decoded = meshtastic_Data_init_default;
        scratchTxPacket.decoded.portnum = meshtastic_PortNum_ROUTING_APP;
        scratchTxPacket.decoded.request_id = request.id;
        scratchTxPacket.decoded.dest = request.from;
        scratchTxPacket.decoded.source = localNodeNum();

        const size_t encoded =
            pb_encode_to_bytes(scratchTxPacket.decoded.payload.bytes, sizeof(scratchTxPacket.decoded.payload.bytes),
                               &meshtastic_Routing_msg, &routing);
        if (encoded == 0) {
            ILOG_WARN("Routing ACK encode failed for request 0x%08lx", static_cast<unsigned long>(request.id));
            return;
        }

        scratchTxPacket.decoded.payload.size = encoded;
        logPacketSummary("TX-ACK", scratchTxPacket);
        transmitMeshPacket(scratchTxPacket);
    }

    void queueReliablePacket(const meshtastic_MeshPacket &packet)
    {
        for (auto &pending : pendingAckTxPackets) {
            if (pending.packet.id == packet.id && pending.packet.to == packet.to) {
                pending.packet = packet;
                pending.retriesDone = 0;
                pending.lastSendMs = millis();
                return;
            }
        }

        PendingAckTx pending;
        pending.packet = packet;
        pending.retriesDone = 0;
        pending.lastSendMs = millis();
        pendingAckTxPackets.push_back(pending);
    }

    bool isRoutingAckForPending(const meshtastic_MeshPacket &packet)
    {
        if (packet.which_payload_variant != meshtastic_MeshPacket_decoded_tag) {
            return false;
        }
        if (packet.decoded.portnum != meshtastic_PortNum_ROUTING_APP) {
            return false;
        }
        if (packet.decoded.request_id == 0) {
            return false;
        }

        const uint32_t requestId = packet.decoded.request_id;
        const uint32_t ackFrom = packet.from;
        for (auto it = pendingAckTxPackets.begin(); it != pendingAckTxPackets.end(); ++it) {
            if (it->packet.id == requestId && it->packet.to == ackFrom) {
                ILOG_INFO("Reliable TX acked: id=0x%08lx to=0x%08lx retries=%u", static_cast<unsigned long>(requestId),
                          static_cast<unsigned long>(ackFrom), static_cast<unsigned>(it->retriesDone));
                pendingAckTxPackets.erase(it);
                return true;
            }
        }

        return false;
    }

    void processPendingAckRetries()
    {
        if (pendingAckTxPackets.empty()) {
            return;
        }

        const uint32_t nowMs = millis();
        for (auto it = pendingAckTxPackets.begin(); it != pendingAckTxPackets.end();) {
            if ((nowMs - it->lastSendMs) < kReliableRetryIntervalMs) {
                ++it;
                continue;
            }

            if (it->retriesDone >= kReliableMaxRetries) {
                ILOG_WARN("Reliable TX timeout: id=0x%08lx to=0x%08lx retries=%u", static_cast<unsigned long>(it->packet.id),
                          static_cast<unsigned long>(it->packet.to), static_cast<unsigned>(it->retriesDone));
                it = pendingAckTxPackets.erase(it);
                continue;
            }

            ++it->retriesDone;
            it->lastSendMs = nowMs;
            ILOG_INFO("Reliable TX retry %u/%u: id=0x%08lx to=0x%08lx", static_cast<unsigned>(it->retriesDone),
                      static_cast<unsigned>(kReliableMaxRetries), static_cast<unsigned long>(it->packet.id),
                      static_cast<unsigned long>(it->packet.to));
            transmitMeshPacket(it->packet);
            ++it;
        }
    }

    bool transmitBuffer(const char *tag, const uint8_t *data, size_t len, bool useDio2AutoSwitch)
    {
        gRadioTxDonePending = false;
        radio.clearPacketSentAction();
        radio.setPacketSentAction(onRadioTransmitDone);

        int state = radio.startTransmit(data, len);
        if (state != RADIOLIB_ERR_NONE) {
            radio.clearPacketSentAction();
            ILOG_WARN("SX1262 startTransmit failed path=%s tag=%s: %d", useDio2AutoSwitch ? "auto" : "manual",
                      tag ? tag : "tx", state);
            return false;
        }

        const uint32_t timeoutMs = 5 + static_cast<uint32_t>((radio.getTimeOnAir(len) * 5) / 1000);
        const uint32_t startMs = millis();
        while (!gRadioTxDonePending) {
            delay(1);
            if ((millis() - startMs) > timeoutMs) {
                radio.clearPacketSentAction();
                radio.finishTransmit();
                ILOG_WARN("SX1262 tx timeout path=%s tag=%s after %lu ms", useDio2AutoSwitch ? "auto" : "manual",
                          tag ? tag : "tx", static_cast<unsigned long>(timeoutMs));
                return false;
            }
        }

        state = radio.finishTransmit();
        radio.clearPacketSentAction();
        if (state != RADIOLIB_ERR_NONE) {
            ILOG_WARN("SX1262 finishTransmit failed path=%s tag=%s: %d", useDio2AutoSwitch ? "auto" : "manual",
                      tag ? tag : "tx", state);
            return false;
        }

        return true;
    }

    const char *rfPathModeName(RfPathMode mode) const
    {
        switch (mode) {
        case RfPathMode::AutoHigh:
            return "auto-rf";
        case RfPathMode::ManualHigh:
            return "manual-high";
        case RfPathMode::ManualLow:
            return "manual-low";
        }

        return "unknown";
    }

    void applyTxRfPathMode(RfPathMode mode)
    {
        switch (mode) {
        case RfPathMode::AutoHigh:
            radio.setDio2AsRfSwitch(true);
            digitalWrite(kLoRaRfSwitch, HIGH);
            break;
        case RfPathMode::ManualHigh:
            radio.setDio2AsRfSwitch(false);
            digitalWrite(kLoRaRfSwitch, HIGH);
            break;
        case RfPathMode::ManualLow:
            radio.setDio2AsRfSwitch(false);
            digitalWrite(kLoRaRfSwitch, LOW);
            break;
        }
    }

    bool transmitBufferWithRfMode(const char *tag, const uint8_t *data, size_t len, RfPathMode mode)
    {
        applyTxRfPathMode(mode);
        delayMicroseconds(200);

        return transmitBuffer(tag, data, len, mode == RfPathMode::AutoHigh);
    }

    void transmitMeshPacket(const meshtastic_MeshPacket &packet)
    {
        uint8_t payload[255] = {0};
        size_t encoded = 0;
        if (!encodePacketForAir(packet, payload, encoded)) {
            ILOG_WARN("Meshtastic air encode failed");
            return;
        }

        gRadioRxPending = false;
        gRadioTxDonePending = false;
        radio.clearPacketReceivedAction();
        radio.clearPacketSentAction();
        logLoRaConfigSummary("TX");

        const bool primaryPathAuto = kPrimaryRfUsesDio2AutoSwitch;
        const bool fallbackPathEnabled = kEnableDualRfSwitchTxProbe && (kFallbackRfUsesDio2AutoSwitch != kPrimaryRfUsesDio2AutoSwitch);
        bool primaryOk = false;
        if (kEnableRfPathSweepTx) {
            primaryOk = transmitBufferWithRfMode("primary-auto", payload, encoded, RfPathMode::AutoHigh);
            transmitBufferWithRfMode("primary-manual-high", payload, encoded, RfPathMode::ManualHigh);
            transmitBufferWithRfMode("primary-manual-low", payload, encoded, RfPathMode::ManualLow);
        } else {
            primaryOk = transmitBuffer("primary", payload, encoded, primaryPathAuto);
        }

        if (kEnableAltHashCompatTx && primaryOk && packet.to == UINT32_MAX && encoded >= sizeof(PacketHeader)) {
            const uint8_t primaryHash = primaryChannelHash();
            uint8_t altHash = 0;
            bool shouldSendAltHash = false;
            if (hasLearnedChannelHash && learnedChannelHash != primaryHash) {
                altHash = learnedChannelHash;
                shouldSendAltHash = true;
            } else {
                const uint8_t legacyHash = legacyPresetNameChannelHash();
                if (legacyHash != primaryHash) {
                    altHash = legacyHash;
                    shouldSendAltHash = true;
                }
            }

            if (shouldSendAltHash) {
                PacketHeader altHeader{};
                std::memcpy(&altHeader, payload, sizeof(altHeader));
                altHeader.channel = altHash;

                uint8_t altPayload[255] = {0};
                std::memcpy(altPayload, payload, encoded);
                std::memcpy(altPayload, &altHeader, sizeof(altHeader));

                transmitBuffer("alt-hash", altPayload, encoded, primaryPathAuto);
                if (fallbackPathEnabled) {
                    transmitBuffer("fallback-primary", payload, encoded, kFallbackRfUsesDio2AutoSwitch);
                    transmitBuffer("fallback-alt-hash", altPayload, encoded, kFallbackRfUsesDio2AutoSwitch);
                }
            }
        } else if (fallbackPathEnabled) {
            transmitBuffer("fallback-primary", payload, encoded, kFallbackRfUsesDio2AutoSwitch);
        }

        if (kEnableRawTxProbe &&
            packet.which_payload_variant == meshtastic_MeshPacket_decoded_tag &&
            packet.decoded.portnum == meshtastic_PortNum_TEXT_MESSAGE_APP) {
            char probe[48] = {0};
            std::snprintf(probe, sizeof(probe), "TAB5-PROBE-%08lX", static_cast<unsigned long>(packet.id));
            const uint8_t *probeBytes = reinterpret_cast<const uint8_t *>(probe);
            const size_t probeLen = std::strlen(probe);
            if (transmitBuffer("raw-probe", probeBytes, probeLen, primaryPathAuto)) {
                ILOG_INFO("SX1262 raw probe transmit ok path=%s: '%s'", primaryPathAuto ? "auto" : "manual", probe);
            }
            if (fallbackPathEnabled) {
                if (transmitBuffer("fallback-raw-probe", probeBytes, probeLen, kFallbackRfUsesDio2AutoSwitch)) {
                    ILOG_INFO("SX1262 raw probe transmit ok path=%s: '%s'",
                              kFallbackRfUsesDio2AutoSwitch ? "auto" : "manual", probe);
                }
            }
        }

        applyRfStrategy(kPrimaryRfUsesDio2AutoSwitch, false);
        const int state = radio.startReceive();
        radio.setPacketReceivedAction(onRadioReceive);
        gRadioRxPending = false;
        if (state != RADIOLIB_ERR_NONE) {
            ILOG_WARN("SX1262 startReceive failed after transmit: %d", state);
        }
    }

    void receiveFromRadio()
    {
        uint8_t payload[255] = {0};
        size_t packetLength = radio.getPacketLength(true);
        if (packetLength == 0 || packetLength > sizeof(payload)) {
            ILOG_WARN("Ignoring invalid SX1262 packet length: %u", static_cast<unsigned>(packetLength));
            radio.startReceive();
            return;
        }

        int state = radio.readData(payload, packetLength);
        float snr = radio.getSNR();
        int32_t rssi = lround(radio.getRSSI());
        ILOG_INFO("SX1262 rx event: state=%d len=%u rssi=%ld snr=%.1f", state, static_cast<unsigned>(packetLength),
                  static_cast<long>(rssi), snr);

        if (state == RADIOLIB_ERR_NONE) {
            if (decodePacketFromAir(payload, packetLength, snr, rssi, scratchRxPacket)) {
                isRoutingAckForPending(scratchRxPacket);
                if (shouldSendRoutingAck(scratchRxPacket)) {
                    sendRoutingAck(scratchRxPacket);
                }
                if (isDuplicateAirPacket(scratchRxPacket)) {
                    ILOG_INFO("Drop duplicate air packet: from=0x%08lx id=0x%08lx", static_cast<unsigned long>(scratchRxPacket.from),
                              static_cast<unsigned long>(scratchRxPacket.id));
                } else {
                    rememberAirPacket(scratchRxPacket);
                    updateKnownNode(scratchRxPacket.from, snr);
                    queueMeshPacket(scratchRxPacket);
                }
            } else {
                ILOG_WARN("Meshtastic air decode failed");
            }
        } else if (state == RADIOLIB_ERR_CRC_MISMATCH) {
            ILOG_WARN("SX1262 CRC mismatch: len=%u rssi=%ld snr=%.1f", static_cast<unsigned>(packetLength),
                      static_cast<long>(rssi), snr);
        } else {
            ILOG_WARN("SX1262 receive failed: %d", state);
        }

        radio.startReceive();
    }

    void updateKnownNode(uint32_t nodeNum, float snr)
    {
        if (nodeNum == 0) {
            return;
        }

        auto it = std::find_if(knownNodes.begin(), knownNodes.end(), [nodeNum](const KnownNode &node) { return node.num == nodeNum; });
        if (it == knownNodes.end()) {
            KnownNode node;
            node.num = nodeNum;
            node.user = meshtastic_User_init_default;
            std::snprintf(node.user.id, sizeof(node.user.id), "!%08lX", static_cast<unsigned long>(nodeNum));
            std::snprintf(node.user.long_name, sizeof(node.user.long_name), "Node %04lX",
                          static_cast<unsigned long>(nodeNum & 0xFFFF));
            std::snprintf(node.user.short_name, sizeof(node.user.short_name), "%04lX",
                          static_cast<unsigned long>(nodeNum & 0xFFFF));
            node.user.hw_model = meshtastic_HardwareModel_UNSET;
            node.user.role = meshtastic_Config_DeviceConfig_Role_CLIENT;
            node.lastHeard = millis() / 1000;
            node.snr = snr;
            knownNodes.push_back(node);
            sendNodeInfo(node);
            return;
        }

        it->lastHeard = millis() / 1000;
        it->snr = snr;
    }

    bool isDuplicateAirPacket(const meshtastic_MeshPacket &packet)
    {
        const uint32_t nowMs = millis();
        constexpr uint32_t kDuplicateWindowMs = 30000;

        for (auto it = recentAirPackets.begin(); it != recentAirPackets.end();) {
            if (nowMs - it->seenAtMs > kDuplicateWindowMs) {
                it = recentAirPackets.erase(it);
                continue;
            }

            if (it->from == packet.from && it->id == packet.id) {
                return true;
            }

            ++it;
        }

        return false;
    }

    void applyRfStrategy(bool useDio2AutoSwitch, bool txPhase)
    {
        radio.setDio2AsRfSwitch(useDio2AutoSwitch);
        if (useDio2AutoSwitch) {
            // Sample-code compatible mode: external RF switch power held high.
            digitalWrite(kLoRaRfSwitch, HIGH);
        } else {
            // Manual GPIO mode: explicit TX/RX levels for board-level RF switch.
            digitalWrite(kLoRaRfSwitch, txPhase ? kRfSwitchTxLevel : kRfSwitchRxLevel);
        }
    }

    void rememberAirPacket(const meshtastic_MeshPacket &packet)
    {
        const uint32_t nowMs = millis();
        constexpr size_t kRecentAirPacketLimit = 32;

        recentAirPackets.push_back({packet.from, packet.id, nowMs});
        if (recentAirPackets.size() > kRecentAirPacketLimit) {
            recentAirPackets.erase(recentAirPackets.begin(), recentAirPackets.begin() + (recentAirPackets.size() - kRecentAirPacketLimit));
        }
    }

    void updateSelfNode()
    {
        if (knownNodes.empty()) {
            return;
        }
        knownNodes.front().user = localUser;
        knownNodes.front().lastHeard = millis() / 1000;
    }

    void applyLoRaConfigToRadio()
    {
        if (!started) {
            return;
        }

        applyRfStrategy(kPrimaryRfUsesDio2AutoSwitch, false);
        gRadioRxPending = false;
        int state = radio.standby();
        if (state != RADIOLIB_ERR_NONE) {
            ILOG_WARN("SX1262 standby failed before reconfig: %d", state);
        }

        float frequency = kDefaultFrequencyMhz;
        float bandwidthKhz = kDefaultBandwidthKhz;
        uint8_t spreadFactor = kDefaultSpreadFactor;
        uint8_t codingRate = kDefaultCodingRate;

        if (loraConfig.use_preset) {
            uint32_t numChannels = LoRaPresets::getNumChannels(loraConfig.region, loraConfig.modem_preset);
            if (numChannels == 0) {
                ILOG_WARN("Invalid LoRa preset/region combination: region=%u preset=%u", loraConfig.region,
                          loraConfig.modem_preset);
                return;
            }
        }

        uint32_t channelNum = 0;
        computeActiveRadioParameters(frequency, bandwidthKhz, spreadFactor, codingRate, channelNum);
        if (loraConfig.use_preset && channelNum > 0) {
            loraConfig.channel_num = channelNum;
        }

        if ((state = radio.setFrequency(frequency)) != RADIOLIB_ERR_NONE) {
            ILOG_WARN("SX1262 setFrequency failed: %d", state);
        }
        if ((state = radio.setBandwidth(bandwidthKhz)) != RADIOLIB_ERR_NONE) {
            ILOG_WARN("SX1262 setBandwidth failed: %d", state);
        }
        if ((state = radio.setSpreadingFactor(spreadFactor)) != RADIOLIB_ERR_NONE) {
            ILOG_WARN("SX1262 setSpreadingFactor failed: %d", state);
        }
        if ((state = radio.setCodingRate(codingRate)) != RADIOLIB_ERR_NONE) {
            ILOG_WARN("SX1262 setCodingRate failed: %d", state);
        }
        if (!applyPacketLayerDefaults(radio, effectiveTxPowerDbm())) {
            ILOG_WARN("SX1262 packet layer reconfigure incomplete");
        }

        applyRfStrategy(kPrimaryRfUsesDio2AutoSwitch, false);

        hasLearnedChannelHash = false;
        learnedChannelHash = 0;
        logLoRaConfigSummary("apply");

        state = radio.startReceive();
        if (state != RADIOLIB_ERR_NONE) {
            ILOG_WARN("SX1262 startReceive failed after reconfig: %d", state);
        } else {
            ILOG_INFO("SX1262 reconfigured: freq=%.4f MHz bw=%.1f kHz sf=%u cr=%u sync=0x%02x chHash=0x%02x", frequency,
                      bandwidthKhz, spreadFactor, codingRate, kDefaultSyncWord, primaryChannelHash());
        }
    }

    void sendRingtoneResponse(const meshtastic_MeshPacket &request)
    {
        scratchAdminResponse = meshtastic_AdminMessage_init_default;
        scratchAdminResponse.which_payload_variant = meshtastic_AdminMessage_get_ringtone_response_tag;
        std::snprintf(scratchAdminResponse.get_ringtone_response, sizeof(scratchAdminResponse.get_ringtone_response), "%s",
                      currentRingtone);
        sendAdminPacketResponse(request, scratchAdminResponse);
    }

    void sendConnectionStatusResponse(const meshtastic_MeshPacket &request)
    {
        scratchAdminResponse = meshtastic_AdminMessage_init_default;
        scratchAdminResponse.which_payload_variant = meshtastic_AdminMessage_get_device_connection_status_response_tag;
        scratchAdminResponse.get_device_connection_status_response = connectionStatus;
        sendAdminPacketResponse(request, scratchAdminResponse);
    }

    void sendAdminPacketResponse(const meshtastic_MeshPacket &request, const meshtastic_AdminMessage &response)
    {
        scratchTxPacket = meshtastic_MeshPacket_init_default;
        scratchTxPacket.from = localNodeNum();
        scratchTxPacket.to = request.from ? request.from : localNodeNum();
        scratchTxPacket.channel = request.channel;
        scratchTxPacket.which_payload_variant = meshtastic_MeshPacket_decoded_tag;
        scratchTxPacket.decoded = meshtastic_Data_init_default;
        scratchTxPacket.decoded.portnum = meshtastic_PortNum_ADMIN_APP;
        scratchTxPacket.decoded.request_id = request.id;
        scratchTxPacket.id = nextMeshPacketId++;

        size_t encoded = pb_encode_to_bytes(scratchTxPacket.decoded.payload.bytes, sizeof(scratchTxPacket.decoded.payload.bytes),
                                            &meshtastic_AdminMessage_msg, &response);
        if (encoded == 0) {
            ILOG_WARN("Admin response encode failed");
            return;
        }

        scratchTxPacket.decoded.payload.size = encoded;
        queueMeshPacket(scratchTxPacket);
    }

    bool started = false;
    uint32_t nextFromRadioId = 1;
    uint32_t nextMeshPacketId = 1;
    Tab5StandaloneClient packetClient;
    std::unique_ptr<PacketServer> packetServer;
    SX1262 radio;

    meshtastic_User localUser = meshtastic_User_init_default;
    meshtastic_MyNodeInfo myInfo = meshtastic_MyNodeInfo_init_default;
    meshtastic_DeviceMetadata metadata = meshtastic_DeviceMetadata_init_default;
    meshtastic_DeviceUIConfig deviceUiConfig = meshtastic_DeviceUIConfig_init_default;
    meshtastic_Config_DeviceConfig deviceConfig = meshtastic_Config_DeviceConfig_init_default;
    meshtastic_Config_PositionConfig positionConfig = meshtastic_Config_PositionConfig_init_default;
    meshtastic_Config_PowerConfig powerConfig = meshtastic_Config_PowerConfig_init_default;
    meshtastic_Config_NetworkConfig networkConfig = meshtastic_Config_NetworkConfig_init_default;
    meshtastic_Config_DisplayConfig displayConfig = meshtastic_Config_DisplayConfig_init_default;
    meshtastic_Config_LoRaConfig loraConfig = meshtastic_Config_LoRaConfig_init_default;
    meshtastic_Config_BluetoothConfig bluetoothConfig = meshtastic_Config_BluetoothConfig_init_default;
    meshtastic_Config_SecurityConfig securityConfig = meshtastic_Config_SecurityConfig_init_default;
    meshtastic_Channel channels[kMaxChannels] = {};
    meshtastic_ModuleConfig telemetryConfig = meshtastic_ModuleConfig_init_default;
    meshtastic_ModuleConfig cannedMessageConfig = meshtastic_ModuleConfig_init_default;
    meshtastic_DeviceConnectionStatus connectionStatus = meshtastic_DeviceConnectionStatus_init_default;
    std::vector<KnownNode> knownNodes;
    std::vector<RecentAirPacket> recentAirPackets;
    std::vector<PendingAckTx> pendingAckTxPackets;
    uint32_t lastNodeInfoBroadcastMs = 0;
    bool hasLearnedChannelHash = false;
    uint8_t learnedChannelHash = 0;
    char currentRingtone[231] = {0};

    meshtastic_AdminMessage scratchAdminMessage = meshtastic_AdminMessage_init_default;
    meshtastic_AdminMessage scratchAdminResponse = meshtastic_AdminMessage_init_default;
    meshtastic_FromRadio scratchFromRadio = meshtastic_FromRadio_init_default;
    meshtastic_MeshPacket scratchRxPacket = meshtastic_MeshPacket_init_default;
    meshtastic_MeshPacket scratchTxPacket = meshtastic_MeshPacket_init_default;
    PendingSystemAction pendingSystemAction = PendingSystemAction::None;
    uint32_t pendingSystemActionAtMs = 0;
};

} // namespace

class Tab5LocalNode::Impl : public Tab5LocalNodeImpl
{
};

Tab5LocalNode &Tab5LocalNode::instance()
{
    static Tab5LocalNode node;
    return node;
}

Tab5LocalNode::Tab5LocalNode() : impl(new Impl()) {}

bool Tab5LocalNode::begin()
{
    return impl->begin();
}

void Tab5LocalNode::taskHandler()
{
    impl->taskHandler();
}

IClientBase &Tab5LocalNode::client()
{
    return impl->client();
}
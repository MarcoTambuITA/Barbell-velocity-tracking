// ════════════════════════════════════════════════════════════════
// ble_vbt_service.h — VBT BLE Service for Arduino Nano 33 BLE Rev2
// Low-latency BLE stack using ArduinoBLE
// ════════════════════════════════════════════════════════════════
#ifndef BLE_VBT_SERVICE_H
#define BLE_VBT_SERVICE_H

#include <ArduinoBLE.h>

// ────────────────────────────────────────────────────────────────
// Custom 128-bit UUIDs (generated, no conflict with standard GATT)
// ────────────────────────────────────────────────────────────────
#define VBT_SERVICE_UUID        "a7e5f8b0-3c14-4b92-8d7e-6f1a2b9c0e34"
#define VELOCITY_CHAR_UUID      "a7e5f8b1-3c14-4b92-8d7e-6f1a2b9c0e34"
#define REP_STATS_CHAR_UUID     "a7e5f8b2-3c14-4b92-8d7e-6f1a2b9c0e34"

// ────────────────────────────────────────────────────────────────
// RepData Struct — packed binary payload for repStatsChar
// ────────────────────────────────────────────────────────────────
// 4 floats × 4 bytes = 16 bytes total (fits within 20-byte MTU)
struct __attribute__((packed)) RepData {
    float meanConcentricVelocity;   // m/s
    float peakConcentricVelocity;   // m/s
    float timeUnderTension;         // seconds
    float rangeOfMotion;            // meters
};

// ────────────────────────────────────────────────────────────────
// BLE Objects
// ────────────────────────────────────────────────────────────────

// Service
BLEService vbtService(VBT_SERVICE_UUID);

// Characteristics
//   velocityChar: Float (4 bytes), Read + Notify
BLEFloatCharacteristic velocityChar(VELOCITY_CHAR_UUID, BLERead | BLENotify);

//   repStatsChar: 20-byte array, Read + Notify
BLECharacteristic repStatsChar(REP_STATS_CHAR_UUID, BLERead | BLENotify, 20);

// ────────────────────────────────────────────────────────────────
// BLE Initialization — call this from setup()
// ────────────────────────────────────────────────────────────────
bool initBLE() {
    if (!BLE.begin()) {
        Serial.println("BLE: init FAILED");
        return false;
    }

    // ── Low-Latency Connection Parameters ──
    // min/max interval in units of 1.25ms → 7.5ms / 15ms
    // slave latency 0 = respond every interval (no skipping)
    // supervision timeout in units of 10ms → 2000ms
    BLE.setConnectionInterval(16, 32);  // 20-40 ms. can change back to 7.5ms – 15ms by changing the code to (6, 12)
    // Note: actual interval is negotiated with the central device

    // ── Device Name & Advertised Service ──
    BLE.setLocalName("VBT-Nano33");
    BLE.setAdvertisedService(vbtService);

    // ── Attach Characteristics to Service ──
    vbtService.addCharacteristic(velocityChar);
    vbtService.addCharacteristic(repStatsChar);

    // ── Register Service ──
    BLE.addService(vbtService);

    // ── Set Initial Values ──
    velocityChar.writeValue(0.0f);

    uint8_t emptyStats[20] = {0};
    repStatsChar.writeValue(emptyStats, 20);

    // ── Start Advertising ──
    BLE.advertise();

    Serial.println("BLE: VbtService advertising as 'VBT-Nano33'");
    return true;
}

// ────────────────────────────────────────────────────────────────
// updateRepStats — Write rep metrics to BLE as raw binary
// ────────────────────────────────────────────────────────────────
void updateRepStats(RepData newData) {
    repStatsChar.writeValue(
        reinterpret_cast<const uint8_t*>(&newData),
        sizeof(RepData)
    );
}

#endif // BLE_VBT_SERVICE_H

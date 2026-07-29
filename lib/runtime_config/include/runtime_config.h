#pragma once

#include <Arduino.h>

struct RuntimePidConfig {
    float kp;
    float ki;
    float kd;
    uint16_t periodMs;
    float alphaD;
    float integralLimit;
    float minRetargetFrac;
    float uNeutral;
};

struct RuntimeBalanceConfig {
    uint32_t holdMs;
    float stopPressureDeltaKpa;
    uint8_t stopPressureSamples;
    uint16_t samplePeriodMs;
};

struct RuntimeMotorConfig {
    uint32_t maxSpeed;
    uint32_t maxAcceleration;
    uint32_t homingSpeed;
    uint32_t testSpeed;
};

class RuntimeConfigManager {
public:
    void begin();

    const RuntimePidConfig& pid() const { return _pid; }
    const RuntimeBalanceConfig& balance() const { return _balance; }
    const RuntimeMotorConfig& motor() const { return _motor; }

    bool setPidConfig(const RuntimePidConfig& config);
    bool setBalanceConfig(const RuntimeBalanceConfig& config);
    bool setMotorConfig(const RuntimeMotorConfig& config);

    bool validatePidConfig(const RuntimePidConfig& config) const;
    bool validateBalanceConfig(const RuntimeBalanceConfig& config) const;
    bool validateMotorConfig(const RuntimeMotorConfig& config) const;

    void applyPidConfig() const;
    void applyMotorConfig() const;

    void formatPidConfigJson(char* buffer, size_t size) const;
    void formatBalanceConfigJson(char* buffer, size_t size) const;
    void formatMotorConfigJson(char* buffer, size_t size) const;

private:
    RuntimePidConfig _pid{};
    RuntimeBalanceConfig _balance{};
    RuntimeMotorConfig _motor{};

    void loadPidConfig();
    void loadBalanceConfig();
    void loadMotorConfig();
    bool savePidConfig() const;
    bool saveBalanceConfig() const;
    bool saveMotorConfig() const;
};

extern RuntimeConfigManager runtimeConfig;

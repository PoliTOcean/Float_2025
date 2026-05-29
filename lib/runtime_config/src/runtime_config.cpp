#include "runtime_config.h"

#include <Preferences.h>
#include <math.h>

#include "DebugSerial.h"
#include "config.h"
#include "motor.h"
#include "pid.h"

namespace {
constexpr char NVS_NAMESPACE[] = "float_runtime";
constexpr uint32_t PID_MAGIC = 0x50494432;     // PID2
constexpr uint32_t BALANCE_MAGIC = 0x42414C31; // BAL1
constexpr uint32_t MOTOR_MAGIC = 0x4D4F5431;   // MOT1

constexpr uint32_t MOTOR_SPEED_MIN = 10;
constexpr uint32_t MOTOR_SPEED_MAX = 5000;
constexpr uint32_t MOTOR_ACCEL_MIN = 10;
constexpr uint32_t MOTOR_ACCEL_MAX = 10000;

template <typename T>
struct StoredConfig {
    uint32_t magic;
    T config;
};

RuntimePidConfig defaultPidConfig() {
    return {
        PID_KP_DEFAULT,
        PID_KI_DEFAULT,
        PID_KD_DEFAULT,
        PID_PERIOD_DEFAULT_MS,
        PID_ALPHA_D_DEFAULT,
        PID_INTEGRAL_LIMIT,
        PID_MIN_RETARGET_FRAC,
        PID_U_NEUTRAL,
    };
}

RuntimeBalanceConfig defaultBalanceConfig() {
    return {
        5000,
        BALANCE_STOP_PRESSURE_DELTA_KPA,
        BALANCE_STOP_PRESSURE_SAMPLES,
        BALANCE_PRESSURE_SAMPLE_PERIOD_MS,
    };
}

RuntimeMotorConfig defaultMotorConfig() {
    return {
        MOTOR_MAX_SPEED,
        MOTOR_MAX_ACCELERATION,
        MOTOR_HOMING_SPEED,
        MOTOR_MAX_SPEED,
    };
}

bool finiteNonNegative(float value) {
    return isfinite(value) && value >= 0.0f;
}
}

RuntimeConfigManager runtimeConfig;

void RuntimeConfigManager::begin() {
    _pid = defaultPidConfig();
    _balance = defaultBalanceConfig();
    _motor = defaultMotorConfig();

    loadPidConfig();
    loadBalanceConfig();
    loadMotorConfig();
    applyPidConfig();
}

bool RuntimeConfigManager::validatePidConfig(const RuntimePidConfig& config) const {
    return isfinite(config.kp) &&
           isfinite(config.ki) &&
           isfinite(config.kd) &&
           config.periodMs >= 20 &&
           config.periodMs <= 500 &&
           config.alphaD >= 0.05f &&
           config.alphaD <= 1.0f &&
           isfinite(config.integralLimit) &&
           config.integralLimit > 0.0f &&
           finiteNonNegative(config.minRetargetFrac) &&
           finiteNonNegative(config.uNeutral);
}

bool RuntimeConfigManager::validateBalanceConfig(const RuntimeBalanceConfig& config) const {
    return config.holdMs <= 60000 &&
           isfinite(config.stopPressureDeltaKpa) &&
           config.stopPressureDeltaKpa >= 0.1f &&
           config.stopPressureDeltaKpa <= 50.0f &&
           config.stopPressureSamples >= 1 &&
           config.stopPressureSamples <= 20 &&
           config.samplePeriodMs >= 20 &&
           config.samplePeriodMs <= 1000;
}

bool RuntimeConfigManager::validateMotorConfig(const RuntimeMotorConfig& config) const {
    return config.maxSpeed >= MOTOR_SPEED_MIN &&
           config.maxSpeed <= MOTOR_SPEED_MAX &&
           config.maxAcceleration >= MOTOR_ACCEL_MIN &&
           config.maxAcceleration <= MOTOR_ACCEL_MAX &&
           config.homingSpeed >= MOTOR_SPEED_MIN &&
           config.homingSpeed <= MOTOR_SPEED_MAX &&
           config.testSpeed >= MOTOR_SPEED_MIN &&
           config.testSpeed <= MOTOR_SPEED_MAX;
}

bool RuntimeConfigManager::setPidConfig(const RuntimePidConfig& config) {
    if (!validatePidConfig(config)) {
        Debug.println("PID config rejected");
        return false;
    }
    const RuntimePidConfig previous = _pid;
    _pid = config;
    const bool saved = savePidConfig();
    if (!saved) {
        _pid = previous;
        applyPidConfig();
        Debug.println("PID config save failed");
        return false;
    }
    applyPidConfig();
    Debug.printf("PID config: Kp=%.4f Ki=%.4f Kd=%.4f period=%u alpha=%.3f "
                 "integral=%.3f retarget=%.5f neutral=%.4f\n",
                 _pid.kp, _pid.ki, _pid.kd, _pid.periodMs, _pid.alphaD,
                 _pid.integralLimit, _pid.minRetargetFrac, _pid.uNeutral);
    return saved;
}

bool RuntimeConfigManager::setBalanceConfig(const RuntimeBalanceConfig& config) {
    if (!validateBalanceConfig(config)) {
        Debug.println("Balance config rejected");
        return false;
    }
    const RuntimeBalanceConfig previous = _balance;
    _balance = config;
    const bool saved = saveBalanceConfig();
    if (!saved) {
        _balance = previous;
        Debug.println("Balance config save failed");
        return false;
    }
    Debug.printf("Balance config: hold=%lu delta=%.2f samples=%u period=%u\n",
                 static_cast<unsigned long>(_balance.holdMs),
                 _balance.stopPressureDeltaKpa,
                 _balance.stopPressureSamples,
                 _balance.samplePeriodMs);
    return saved;
}

bool RuntimeConfigManager::setMotorConfig(const RuntimeMotorConfig& config) {
    if (!validateMotorConfig(config)) {
        Debug.println("Motor config rejected");
        return false;
    }
    const RuntimeMotorConfig previous = _motor;
    _motor = config;
    const bool saved = saveMotorConfig();
    if (!saved) {
        _motor = previous;
        applyMotorConfig();
        Debug.println("Motor config save failed");
        return false;
    }
    applyMotorConfig();
    Debug.printf("Motor config: max=%lu accel=%lu homing=%lu test=%lu\n",
                 static_cast<unsigned long>(_motor.maxSpeed),
                 static_cast<unsigned long>(_motor.maxAcceleration),
                 static_cast<unsigned long>(_motor.homingSpeed),
                 static_cast<unsigned long>(_motor.testSpeed));
    return saved;
}

void RuntimeConfigManager::applyPidConfig() const {
    pidController.Kp = _pid.kp;
    pidController.Ki = _pid.ki;
    pidController.Kd = _pid.kd;
    pidController.periodMs = _pid.periodMs;
    pidController.alphaD = _pid.alphaD;
    pidController.integralLimit = _pid.integralLimit;
    pidController.minRetargetFrac = _pid.minRetargetFrac;
    pidController.uNeutral = _pid.uNeutral;
}

void RuntimeConfigManager::applyMotorConfig() const {
    ::motor.setMaxSpeed(_motor.maxSpeed);
    ::motor.setAcceleration(_motor.maxAcceleration);
}

void RuntimeConfigManager::formatPidConfigJson(char* buffer, size_t size) const {
    snprintf(buffer, size,
             "{\"kp\":%.6f,\"ki\":%.6f,\"kd\":%.6f,"
             "\"period_ms\":%u,\"alpha_d\":%.6f,"
             "\"integral_limit\":%.6f,\"min_retarget_frac\":%.6f,"
             "\"u_neutral\":%.6f}",
             _pid.kp, _pid.ki, _pid.kd,
             _pid.periodMs, _pid.alphaD,
             _pid.integralLimit, _pid.minRetargetFrac, _pid.uNeutral);
}

void RuntimeConfigManager::formatBalanceConfigJson(char* buffer, size_t size) const {
    snprintf(buffer, size,
             "{\"hold_ms\":%lu,\"stop_delta_kpa\":%.6f,"
             "\"stop_samples\":%u,\"sample_period_ms\":%u}",
             static_cast<unsigned long>(_balance.holdMs),
             _balance.stopPressureDeltaKpa,
             _balance.stopPressureSamples,
             _balance.samplePeriodMs);
}

void RuntimeConfigManager::formatMotorConfigJson(char* buffer, size_t size) const {
    snprintf(buffer, size,
             "{\"max_speed\":%lu,\"max_accel\":%lu,"
             "\"homing_speed\":%lu,\"test_speed\":%lu}",
             static_cast<unsigned long>(_motor.maxSpeed),
             static_cast<unsigned long>(_motor.maxAcceleration),
             static_cast<unsigned long>(_motor.homingSpeed),
             static_cast<unsigned long>(_motor.testSpeed));
}

void RuntimeConfigManager::loadPidConfig() {
    Preferences prefs;
    if (!prefs.begin(NVS_NAMESPACE, true)) {
        return;
    }
    StoredConfig<RuntimePidConfig> stored{};
    const size_t read = prefs.getBytes("pid", &stored, sizeof(stored));
    prefs.end();
    if (read == sizeof(stored) &&
        stored.magic == PID_MAGIC &&
        validatePidConfig(stored.config)) {
        _pid = stored.config;
    }
}

void RuntimeConfigManager::loadBalanceConfig() {
    Preferences prefs;
    if (!prefs.begin(NVS_NAMESPACE, true)) {
        return;
    }
    StoredConfig<RuntimeBalanceConfig> stored{};
    const size_t read = prefs.getBytes("balance", &stored, sizeof(stored));
    prefs.end();
    if (read == sizeof(stored) &&
        stored.magic == BALANCE_MAGIC &&
        validateBalanceConfig(stored.config)) {
        _balance = stored.config;
    }
}

void RuntimeConfigManager::loadMotorConfig() {
    Preferences prefs;
    if (!prefs.begin(NVS_NAMESPACE, true)) {
        return;
    }
    StoredConfig<RuntimeMotorConfig> stored{};
    const size_t read = prefs.getBytes("motor", &stored, sizeof(stored));
    prefs.end();
    if (read == sizeof(stored) &&
        stored.magic == MOTOR_MAGIC &&
        validateMotorConfig(stored.config)) {
        _motor = stored.config;
    }
}

bool RuntimeConfigManager::savePidConfig() const {
    Preferences prefs;
    if (!prefs.begin(NVS_NAMESPACE, false)) {
        return false;
    }
    StoredConfig<RuntimePidConfig> stored{PID_MAGIC, _pid};
    const size_t written = prefs.putBytes("pid", &stored, sizeof(stored));
    prefs.end();
    return written == sizeof(stored);
}

bool RuntimeConfigManager::saveBalanceConfig() const {
    Preferences prefs;
    if (!prefs.begin(NVS_NAMESPACE, false)) {
        return false;
    }
    StoredConfig<RuntimeBalanceConfig> stored{BALANCE_MAGIC, _balance};
    const size_t written = prefs.putBytes("balance", &stored, sizeof(stored));
    prefs.end();
    return written == sizeof(stored);
}

bool RuntimeConfigManager::saveMotorConfig() const {
    Preferences prefs;
    if (!prefs.begin(NVS_NAMESPACE, false)) {
        return false;
    }
    StoredConfig<RuntimeMotorConfig> stored{MOTOR_MAGIC, _motor};
    const size_t written = prefs.putBytes("motor", &stored, sizeof(stored));
    prefs.end();
    return written == sizeof(stored);
}

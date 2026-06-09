#pragma once

#include <AP_GpsSpoofDetect/AP_GpsSpoofDetect_config.h>

#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>
#include <AP_Math/AP_Math.h>
#include <AP_GPS/AP_GPS.h>
#include <AP_InertialSensor/AP_InertialSensor.h>
#include <AP_AHRS/AP_AHRS.h>
#include <AP_Logger/AP_Logger.h>

#if AP_GPSPOOFDETECT_ENABLED

class AP_GpsSpoofDetect {
public:
    AP_GpsSpoofDetect();

    CLASS_NO_COPY(AP_GpsSpoofDetect);

    static const struct AP_Param::GroupInfo var_info[];

    static AP_GpsSpoofDetect *get_singleton() {
        return _singleton;
    }

    void init();
    void update();

    enum class State : uint8_t {
        NOMINAL = 0,
        SUSPICIOUS = 1,
        CONFIRMED = 2
    };

    State get_state() const { return _state; }
    float get_score() const { return _score; }
    uint8_t get_feature_flags() const { return _feature_flags; }
    float get_f1_score() const { return _f1_score; }
    float get_f2_score() const { return _f2_score; }
    float get_f3_score() const { return _f3_score; }
    float get_f4_score() const { return _f4_score; }
    bool healthy() const;

    // Public for testing
    void store_sample(const Vector3f &v_gps, const Vector3f &v_ekf, float hAcc, float sAcc);
    void compute_feature_vel_divergence(const Vector3f &v_gps, const Vector3f &v_ekf, float sAcc);
    void compute_feature_variance_trend(AP_AHRS &ahrs);
    void compute_feature_accel_consistency(const AP_InertialSensor &ins, AP_AHRS &ahrs, const Vector3f &v_gps);
    void compute_feature_accuracy_consistency(const Vector3f &v_gps, const Vector3f &v_ekf, float hAcc, float sAcc);
    void fuse_and_update_state();

    // Test setters
    void set_threshold_low(float val) { _threshold_low.set(val); }
    void set_threshold_high(float val) { _threshold_high.set(val); }
    void set_weight1(float val) { _weight1.set(val); }
    void set_weight2(float val) { _weight2.set(val); }
    void set_weight3(float val) { _weight3.set(val); }
    void set_weight4(float val) { _weight4.set(val); }
    void set_enable(uint8_t val) { _enable.set(val); }
    void set_innov_thresh(float val) { _innov_thresh.set(val); }
    void set_cusum_k(float val) { _cusum_k.set(val); }
    void set_action(uint8_t val) { _action.set(val); }
    void set_state(State val) { _state = val; }
    void set_f1_score(float val) { _f1_score = val; }
    void set_f2_score(float val) { _f2_score = val; }
    void set_f3_score(float val) { _f3_score = val; }
    void set_f4_score(float val) { _f4_score = val; }

private:
    friend class TestGpsSpoofDetect;

    static AP_GpsSpoofDetect *_singleton;

    // Parameters
    AP_Int8 _enable;
    AP_Float _innov_thresh;
    AP_Float _cusum_k;
    AP_Float _win_short;
    AP_Float _win_med;
    AP_Float _win_long;
    AP_Float _weight1;
    AP_Float _weight2;
    AP_Float _weight3;
    AP_Float _weight4;
    AP_Float _threshold_low;
    AP_Float _threshold_high;
    AP_Int8 _action;
    AP_Int8 _log_level;

    // State machine
    State _state;
    State _previous_state;
    float _score;
    uint8_t _feature_flags;
    uint32_t _state_change_ms;
    uint32_t _last_warn_ms;

    // CUSUM accumulators (one per feature: vel_divergence, variance, accel, accuracy)
    float _cusum_pos[4];

    // Sample buffer for velocities
    static constexpr uint16_t BUFFER_SIZE = 100;
    struct SampleBuffer {
        Vector3f v_gps;
        Vector3f v_ekf;
        float hAcc;
        float sAcc;
    } _sample_buffer[BUFFER_SIZE];
    uint16_t _buffer_idx;
    uint16_t _buffer_count;

    // Feature scores
    float _f1_score;
    float _f2_score;
    float _f3_score;
    float _f4_score;

    // For acceleration feature
    Vector3f _v_gps_prev;
    Vector3f _a_gps_filt;
    uint32_t _last_accel_ms;

    // Hysteresis counters
    uint8_t _suspicious_count;
    uint8_t _confirmed_count;
    uint8_t _nominal_count;

    // Initialization flag
    bool _initialized;

    // Helper methods
    void emit_log();
    void emit_gcs_warning();
    void reset_buffers();
};

namespace AP {
    AP_GpsSpoofDetect *gps_spoof_detect();
};

#else
class AP_GpsSpoofDetect {
public:
    void init() {}
    void update() {}
    static AP_GpsSpoofDetect *get_singleton() { return nullptr; }
};
#endif  // AP_GPSPOOFDETECT_ENABLED

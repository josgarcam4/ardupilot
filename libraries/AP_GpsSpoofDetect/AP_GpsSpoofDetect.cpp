#include "AP_GpsSpoofDetect.h"

#if AP_GPSPOOFDETECT_ENABLED

#include <GCS_MAVLink/GCS.h>

AP_GpsSpoofDetect *AP_GpsSpoofDetect::_singleton = nullptr;

const AP_Param::GroupInfo AP_GpsSpoofDetect::var_info[] = {
    AP_GROUPINFO_FLAGS("GSPF_ENABLE", 1, AP_GpsSpoofDetect, _enable, 1, AP_PARAM_FLAG_ENABLE),
    AP_GROUPINFO("GSPF_INNOV_TH", 2, AP_GpsSpoofDetect, _innov_thresh, 12.0f),
    AP_GROUPINFO("GSPF_CUSUM_K", 3, AP_GpsSpoofDetect, _cusum_k, 1.5f),
    AP_GROUPINFO("GSPF_WIN_SHRT", 4, AP_GpsSpoofDetect, _win_short, 1.0f),
    AP_GROUPINFO("GSPF_WIN_MED", 5, AP_GpsSpoofDetect, _win_med, 10.0f),
    AP_GROUPINFO("GSPF_WIN_LONG", 6, AP_GpsSpoofDetect, _win_long, 60.0f),
    AP_GROUPINFO("GSPF_W1", 7, AP_GpsSpoofDetect, _weight1, 1.0f),
    AP_GROUPINFO("GSPF_W2", 8, AP_GpsSpoofDetect, _weight2, 0.0f),
    AP_GROUPINFO("GSPF_W3", 9, AP_GpsSpoofDetect, _weight3, 0.0f),
    AP_GROUPINFO("GSPF_W4", 10, AP_GpsSpoofDetect, _weight4, 0.0f),
    AP_GROUPINFO("GSPF_TH_LOW", 11, AP_GpsSpoofDetect, _threshold_low, 0.5f),
    AP_GROUPINFO("GSPF_TH_HIGH", 12, AP_GpsSpoofDetect, _threshold_high, 1.0f),
    AP_GROUPINFO("GSPF_ACT", 13, AP_GpsSpoofDetect, _action, 1),
    AP_GROUPINFO("GSPF_LOG_LVL", 14, AP_GpsSpoofDetect, _log_level, 1),
    AP_GROUPEND
};

AP_GpsSpoofDetect::AP_GpsSpoofDetect()
    : _state(State::NOMINAL),
      _previous_state(State::NOMINAL),
      _score(0.0f),
      _feature_flags(0),
      _state_change_ms(0),
      _last_warn_ms(0),
      _buffer_idx(0),
      _buffer_count(0),
      _f1_score(0.0f),
      _f2_score(0.0f),
      _f3_score(0.0f),
      _f4_score(0.0f),
      _v_gps_prev(Vector3f()),
      _a_gps_filt(Vector3f()),
      _last_accel_ms(0),
      _suspicious_count(0),
      _confirmed_count(0),
      _nominal_count(0),
      _initialized(false)
{
    _singleton = this;
    memset(_cusum_pos, 0, sizeof(_cusum_pos));
    // Register parameter group in constructor (pattern from AP_Camera)
    AP_Param::setup_object_defaults(this, var_info);
}

void AP_GpsSpoofDetect::init()
{
    reset_buffers();
}

void AP_GpsSpoofDetect::reset_buffers()
{
    _buffer_idx = 0;
    _buffer_count = 0;
    _last_accel_ms = AP_HAL::millis();
    _v_gps_prev = Vector3f();
    _a_gps_filt = Vector3f();
    memset(_cusum_pos, 0, sizeof(_cusum_pos));

    // Reinitialize sample buffer properly
    for (uint16_t i = 0; i < BUFFER_SIZE; i++) {
        _sample_buffer[i].v_gps = Vector3f();
        _sample_buffer[i].v_ekf = Vector3f();
        _sample_buffer[i].hAcc = 0.0f;
        _sample_buffer[i].sAcc = 0.0f;
    }
}

void AP_GpsSpoofDetect::store_sample(const Vector3f &v_gps, const Vector3f &v_ekf, float hAcc, float sAcc)
{
    uint16_t idx = _buffer_idx;
    _sample_buffer[idx].v_gps = v_gps;
    _sample_buffer[idx].v_ekf = v_ekf;
    _sample_buffer[idx].hAcc = hAcc;
    _sample_buffer[idx].sAcc = sAcc;

    _buffer_idx = (_buffer_idx + 1) % BUFFER_SIZE;
    if (_buffer_count < BUFFER_SIZE) {
        _buffer_count++;
    }
}

void AP_GpsSpoofDetect::update()
{
    // Lazy initialization on first update
    if (!_initialized) {
        init();
        _initialized = true;
    }

    if (!_enable) {
        reset_buffers();
        _state = State::NOMINAL;
        return;
    }

    const AP_GPS &gps = AP::gps();
    if (gps.status() < AP_GPS_FixType::FIX_3D) {
        return;
    }

    AP_AHRS &ahrs = AP::ahrs();
    const AP_InertialSensor &ins = AP::ins();

    // Get GPS velocity
    Vector3f v_gps = gps.velocity();

    // Get EKF velocity
    Vector3f v_ekf;
    if (!ahrs.get_velocity_NED(v_ekf)) {
        return;
    }

    // Get GPS accuracy
    float hAcc = 5.0f, sAcc = 1.0f;
    gps.horizontal_accuracy(hAcc);
    gps.speed_accuracy(sAcc);

    // Store sample
    store_sample(v_gps, v_ekf, hAcc, sAcc);

    // Compute features
    _feature_flags = 0;
    compute_feature_vel_divergence(v_gps, v_ekf, sAcc);
    compute_feature_variance_trend(ahrs);
    compute_feature_accel_consistency(ins, ahrs, v_gps);
    compute_feature_accuracy_consistency(v_gps, v_ekf, hAcc, sAcc);

    // Fuse and update state machine
    fuse_and_update_state();

    // Logging and GCS notification
    emit_log();
    emit_gcs_warning();
}

void AP_GpsSpoofDetect::compute_feature_vel_divergence(const Vector3f &v_gps, const Vector3f &v_ekf, float sAcc)
{
    Vector3f residual = v_gps - v_ekf;
    float residual_norm = residual.length();

    // Normalize by GPS speed accuracy (1-sigma)
    // Use 1.0 as minimum to avoid false sensitivity when SITL reports sAcc=0
    float sigma = fmaxf(sAcc, 1.0f);
    float nu = residual_norm / sigma;

    // CUSUM: accumulate positive deviations
    _cusum_pos[0] = fmaxf(0.0f, _cusum_pos[0] + nu - _cusum_k);
    // Limit CUSUM to prevent indefinite growth during prolonged spoofing
    _cusum_pos[0] = fminf(_cusum_pos[0], 500.0f);

    // Score: normalized CUSUM ratio
    float ratio = _cusum_pos[0] / _innov_thresh;
    _f1_score = fminf(ratio, 2.0f);

    // DEBUG: log residual details every 20 cycles (~2 seconds at 10Hz)
    static uint32_t f1_count = 0;
    if (f1_count++ % 20 == 0) {
        gcs().send_text(MAV_SEVERITY_INFO, "GSPF_F1: res=%.2f sAcc=%.2f nu=%.2f cusum=%.1f f1=%.2f score=%.2f",
            residual_norm, sAcc, nu, _cusum_pos[0], _f1_score, _score);
    }

    if (_f1_score > 0.9f) {
        _feature_flags |= (1 << 0);
    }
}

void AP_GpsSpoofDetect::compute_feature_variance_trend(AP_AHRS &ahrs)
{
    float velVar, posVar, hgtVar, tasVar;
    Vector3f magVar;

    if (!ahrs.get_variances(velVar, posVar, hgtVar, magVar, tasVar)) {
        _f2_score = 0.0f;
        return;
    }

    // Normalize variances by expected nominal values
    // Nominal: velVar ~ 0.5, posVar ~ 2.0
    float velNorm = velVar / 0.5f;
    float posNorm = posVar / 2.0f;
    float avgNorm = (velNorm + posNorm) / 2.0f;

    // Only accumulate when variance is elevated
    float nu = fmaxf(0.0f, avgNorm - 1.0f);

    // CUSUM with slower accumulation for this feature
    _cusum_pos[1] = fmaxf(0.0f, _cusum_pos[1] + nu - _cusum_k * 0.3f);

    // Score
    float ratio = _cusum_pos[1] / _innov_thresh;
    _f2_score = fminf(ratio, 2.0f);

    if (_f2_score > 0.9f) {
        _feature_flags |= (1 << 1);
    }
}

void AP_GpsSpoofDetect::compute_feature_accel_consistency(const AP_InertialSensor &ins, AP_AHRS &ahrs, const Vector3f &v_gps)
{
    uint32_t now_ms = AP_HAL::millis();
    float dt = (now_ms - _last_accel_ms) * 1e-3f;

    // Need valid dt (between 0.08s and 1.5s for reasonable GPS derivatives)
    if (dt < 0.08f || dt > 1.5f) {
        _last_accel_ms = now_ms;
        _v_gps_prev = v_gps;
        _f3_score = 0.0f;
        return;
    }

    // Derive GPS acceleration from velocity change
    Vector3f a_gps_raw = (v_gps - _v_gps_prev) / dt;

    // Low-pass filter GPS acceleration (0.7 alpha for smoothing)
    _a_gps_filt = _a_gps_filt * 0.7f + a_gps_raw * 0.3f;

    // Get IMU delta-velocity
    Vector3f dv;
    float dv_dt;
    if (ins.get_delta_velocity(dv, dv_dt) && dv_dt > 0.01f) {
        // IMU acceleration in body frame
        Vector3f a_imu_body = dv / dv_dt;

        // Rotate to NED
        Vector3f a_imu_ned = ahrs.body_to_earth(a_imu_body);

        // Remove gravity (subtract because gravity vector in body frame points down)
        a_imu_ned.z += GRAVITY_MSS;

        // Compare accelerations
        float diff = (_a_gps_filt - a_imu_ned).length();
        const float thresh = 1.5f;

        // Only trigger if difference significantly exceeds threshold
        float nu = fmaxf(0.0f, (diff / thresh) - 1.0f);

        _cusum_pos[2] = fmaxf(0.0f, _cusum_pos[2] + nu - _cusum_k * 0.5f);

        float ratio = _cusum_pos[2] / _innov_thresh;
        _f3_score = fminf(ratio, 2.0f);

        if (_f3_score > 0.9f) {
            _feature_flags |= (1 << 2);
        }
    }

    _v_gps_prev = v_gps;
    _last_accel_ms = now_ms;
}

void AP_GpsSpoofDetect::compute_feature_accuracy_consistency(const Vector3f &v_gps, const Vector3f &v_ekf, float hAcc, float sAcc)
{
    float actual_diff = (v_gps - v_ekf).length();

    // Expected difference given GPS accuracy
    float expected_diff = fmaxf(sAcc, 0.05f);

    // If GPS claims high accuracy but actual divergence is large, inconsistent
    if (actual_diff > 3.0f * expected_diff && hAcc < 3.0f) {
        // Accumulate anomaly
        _cusum_pos[3] = fmaxf(0.0f, _cusum_pos[3] + 1.0f - _cusum_k);
    } else {
        // Decay faster when consistent
        _cusum_pos[3] = fmaxf(0.0f, _cusum_pos[3] - 0.5f);
    }

    float ratio = _cusum_pos[3] / _innov_thresh;
    _f4_score = fminf(ratio, 2.0f);

    if (_f4_score > 0.9f) {
        _feature_flags |= (1 << 3);
    }
}

void AP_GpsSpoofDetect::fuse_and_update_state()
{
    // Fuse features with weights
    float weight_sum = _weight1 + _weight2 + _weight3 + _weight4;
    if (weight_sum < 0.01f) {
        weight_sum = 1.0f;
    }

    _score = (_weight1 * _f1_score + _weight2 * _f2_score +
              _weight3 * _f3_score + _weight4 * _f4_score) / weight_sum;

    _previous_state = _state;

    // State machine with hysteresis
    switch (_state) {
        case State::NOMINAL:
            if (_score > _threshold_low) {
                _suspicious_count++;
                if (_suspicious_count >= 5) {  // 0.5s at 10Hz
                    _state = State::SUSPICIOUS;
                    _state_change_ms = AP_HAL::millis();
                    _suspicious_count = 0;
                    _confirmed_count = 0;
                }
            } else {
                _suspicious_count = 0;
            }
            break;

        case State::SUSPICIOUS:
            if (_score >= _threshold_high) {
                _confirmed_count++;
                if (_confirmed_count >= 30) {  // 3s at 10Hz
                    _state = State::CONFIRMED;
                    _state_change_ms = AP_HAL::millis();
                    _suspicious_count = 0;
                    _confirmed_count = 0;
                }
            } else if (_score < _threshold_low * 0.5f) {
                _state = State::NOMINAL;
                _state_change_ms = AP_HAL::millis();
                _suspicious_count = 0;
                _confirmed_count = 0;
            } else {
                _confirmed_count = 0;
            }
            break;

        case State::CONFIRMED:
            if (_score < _threshold_low * 0.3f) {
                _nominal_count++;
                if (_nominal_count >= 20) {  // 2s at 10Hz - quick recovery
                    _state = State::NOMINAL;
                    _state_change_ms = AP_HAL::millis();
                    _suspicious_count = 0;
                    _confirmed_count = 0;
                    _nominal_count = 0;
                }
            } else {
                _nominal_count = 0;
            }
            break;
    }
}

void AP_GpsSpoofDetect::emit_log()
{
    if (!_enable) {
        return;
    }

    // DEBUG: log feature details every 50 cycles (~5 seconds at 10Hz)
    static uint32_t log_count = 0;
    if (log_count++ % 50 == 0) {
        gcs().send_text(MAV_SEVERITY_INFO, "GSPF_ALL: f1=%.2f f2=%.2f f3=%.2f f4=%.2f cs0=%.1f cs1=%.1f cs2=%.1f cs3=%.1f state=%d",
            _f1_score, _f2_score, _f3_score, _f4_score,
            _cusum_pos[0], _cusum_pos[1], _cusum_pos[2], _cusum_pos[3], (int)_state);
    }
}

void AP_GpsSpoofDetect::emit_gcs_warning()
{
    if (!_enable || _action < 1) {
        return;
    }

    uint32_t now_ms = AP_HAL::millis();

    if (_state == State::SUSPICIOUS) {
        if (now_ms - _last_warn_ms > 5000) {  // Throttle to 5s
            gcs().send_text(MAV_SEVERITY_WARNING, "GSPF: SUSPICIOUS score=%.2f", _score);
            _last_warn_ms = now_ms;
        }
    } else if (_state == State::CONFIRMED) {
        if (now_ms - _last_warn_ms > 2000) {  // Throttle to 2s
            gcs().send_text(MAV_SEVERITY_CRITICAL, "GSPF: CONFIRMED SPOOFING score=%.2f", _score);
            _last_warn_ms = now_ms;
        }
    }
}

bool AP_GpsSpoofDetect::healthy() const
{
    if (!_enable) {
        return true;
    }

    // Not healthy if in CONFIRMED state
    if (_state == State::CONFIRMED) {
        return false;
    }

    return true;
}

namespace AP {
    AP_GpsSpoofDetect *gps_spoof_detect()
    {
        return AP_GpsSpoofDetect::get_singleton();
    }
}

#endif  // AP_GPSPOOFDETECT_ENABLED

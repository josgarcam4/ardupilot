#include <AP_gtest.h>
#include <AP_GpsSpoofDetect/AP_GpsSpoofDetect.h>
#include <AP_Math/AP_Math.h>

class TestGpsSpoofDetect : public ::testing::Test {
protected:
    AP_GpsSpoofDetect detector;

    void SetUp() override {
        detector.init();
    }
};

// Test CUSUM accumulation with residual data
TEST_F(TestGpsSpoofDetect, CUSUM_Nominal) {
    // Simulate nominal operation: small random residuals
    for (int i = 0; i < 100; i++) {
        Vector3f v_gps(0.1f, 0.05f, -0.1f);  // Tiny variations
        Vector3f v_ekf(0.0f, 0.0f, 0.0f);
        detector.store_sample(v_gps, v_ekf, 2.0f, 0.5f);
    }
    // With small residuals, CUSUM shouldn't accumulate significantly
    EXPECT_LT(detector.get_score(), 0.3f);
    EXPECT_EQ(detector.get_state(), AP_GpsSpoofDetect::State::NOMINAL);
}

// Test state transition from NOMINAL to SUSPICIOUS
TEST_F(TestGpsSpoofDetect, StateTransition_Nominal_To_Suspicious) {
    detector.set_threshold_low(0.5f);
    detector.set_weight1(1.0f);  // Only use feature 1

    // Simulate persistent residual (spoofing)
    for (int i = 0; i < 50; i++) {
        Vector3f v_gps(1.0f, 0.5f, 0.3f);  // Large persistent residual
        Vector3f v_ekf(0.0f, 0.0f, 0.0f);
        detector.store_sample(v_gps, v_ekf, 5.0f, 1.0f);

        // Manually call the feature computation
        float sAcc = 1.0f;
        detector.compute_feature_vel_divergence(v_gps, v_ekf, sAcc);
        detector.fuse_and_update_state();

        // After ~5 iterations, should transition to SUSPICIOUS
        if (i >= 4) {
            EXPECT_EQ(detector.get_state(), AP_GpsSpoofDetect::State::SUSPICIOUS);
            break;
        }
    }
}

// Test hysteresis: stay SUSPICIOUS until score drops significantly
TEST_F(TestGpsSpoofDetect, Hysteresis_Suspicious) {
    detector.set_threshold_low(0.5f);
    detector.set_threshold_high(1.0f);
    detector.set_weight1(1.0f);

    // First, elevate score to enter SUSPICIOUS
    for (int i = 0; i < 10; i++) {
        Vector3f v_gps(1.5f, 0.5f, 0.3f);
        Vector3f v_ekf(0.0f, 0.0f, 0.0f);
        detector.store_sample(v_gps, v_ekf, 5.0f, 1.0f);
        detector.compute_feature_vel_divergence(v_gps, v_ekf, 1.0f);
        detector.fuse_and_update_state();
    }

    EXPECT_EQ(detector.get_state(), AP_GpsSpoofDetect::State::SUSPICIOUS);

    // Now drop score slightly but not below threshold_low * 0.5
    for (int i = 0; i < 10; i++) {
        Vector3f v_gps(0.3f, 0.1f, 0.05f);  // Smaller residual
        Vector3f v_ekf(0.0f, 0.0f, 0.0f);
        detector.store_sample(v_gps, v_ekf, 2.0f, 0.5f);
        detector.compute_feature_vel_divergence(v_gps, v_ekf, 0.5f);
        detector.fuse_and_update_state();
    }

    // Should still be SUSPICIOUS (hysteresis)
    EXPECT_EQ(detector.get_state(), AP_GpsSpoofDetect::State::SUSPICIOUS);
}

// Test velocity divergence feature
TEST_F(TestGpsSpoofDetect, VelocityDivergence_ZeroResidual) {
    Vector3f v_gps(1.0f, 2.0f, 3.0f);
    Vector3f v_ekf(1.0f, 2.0f, 3.0f);  // Identical
    detector.compute_feature_vel_divergence(v_gps, v_ekf, 1.0f);
    EXPECT_FLOAT_EQ(detector.get_f1_score(), 0.0f);
}

TEST_F(TestGpsSpoofDetect, VelocityDivergence_LargeResidual) {
    Vector3f v_gps(5.0f, 0.0f, 0.0f);
    Vector3f v_ekf(0.0f, 0.0f, 0.0f);
    float sAcc = 1.0f;
    detector.compute_feature_vel_divergence(v_gps, v_ekf, sAcc);
    // Residual norm = 5.0, normalized by sAcc=1.0 → nu=5.0
    // CUSUM: 0 + 5.0 - 0.5 = 4.5
    // Score = 4.5 / 5.0 = 0.9 (capped at 0.9)
    EXPECT_GT(detector.get_f1_score(), 0.5f);
}

// Test accuracy consistency feature
TEST_F(TestGpsSpoofDetect, AccuracyConsistency_Good) {
    Vector3f v_gps(1.0f, 0.5f, 0.0f);
    Vector3f v_ekf(0.9f, 0.4f, 0.0f);
    float hAcc = 3.0f;
    float sAcc = 0.5f;
    // Actual diff ~ 0.14m/s, expected diff ~ 0.5m/s
    // 0.14 < 3*0.5, so no anomaly
    detector.compute_feature_accuracy_consistency(v_gps, v_ekf, hAcc, sAcc);
    EXPECT_LT(detector.get_f4_score(), 0.5f);
}

TEST_F(TestGpsSpoofDetect, AccuracyConsistency_Inconsistent) {
    Vector3f v_gps(10.0f, 0.0f, 0.0f);
    Vector3f v_ekf(0.0f, 0.0f, 0.0f);
    float hAcc = 2.0f;  // Claims high accuracy
    float sAcc = 0.5f;
    // Actual diff = 10m/s > 3*0.5 = 1.5 and hAcc < 3, so anomaly!
    detector.compute_feature_accuracy_consistency(v_gps, v_ekf, hAcc, sAcc);
    EXPECT_GT(detector.get_f4_score(), 0.5f);  // Should accumulate in CUSUM
}

// Test feature weighting in score fusion
TEST_F(TestGpsSpoofDetect, ScoreFusion_AllFeaturesZero) {
    detector.set_f1_score(0.0f);
    detector.set_f2_score(0.0f);
    detector.set_f3_score(0.0f);
    detector.set_f4_score(0.0f);
    detector.set_weight1(0.25f);
    detector.set_weight2(0.25f);
    detector.set_weight3(0.25f);
    detector.set_weight4(0.25f);
    detector.fuse_and_update_state();
    EXPECT_FLOAT_EQ(detector.get_score(), 0.0f);
}

TEST_F(TestGpsSpoofDetect, ScoreFusion_OneFeatureHigh) {
    detector.set_f1_score(2.0f);
    detector.set_f2_score(0.0f);
    detector.set_f3_score(0.0f);
    detector.set_f4_score(0.0f);
    detector.set_weight1(1.0f);
    detector.set_weight2(0.0f);
    detector.set_weight3(0.0f);
    detector.set_weight4(0.0f);
    detector.fuse_and_update_state();
    EXPECT_FLOAT_EQ(detector.get_score(), 2.0f);
}

// Test healthy() method
TEST_F(TestGpsSpoofDetect, HealthyNominal) {
    detector.set_enable(1);
    detector.set_state(AP_GpsSpoofDetect::State::NOMINAL);
    EXPECT_TRUE(detector.healthy());
}

TEST_F(TestGpsSpoofDetect, HealthyNotConfirmed) {
    detector.set_enable(1);
    detector.set_state(AP_GpsSpoofDetect::State::CONFIRMED);
    EXPECT_FALSE(detector.healthy());
}

TEST_F(TestGpsSpoofDetect, HealthyDisabled) {
    detector.set_enable(0);
    detector.set_state(AP_GpsSpoofDetect::State::CONFIRMED);
    EXPECT_TRUE(detector.healthy());  // Disabled = healthy
}

AP_GTEST_MAIN()

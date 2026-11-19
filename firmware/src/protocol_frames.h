#pragma once
#include <stdint.h>

enum class MsgType : uint8_t {
    SENSOR_50HZ      = 0x01,
    GESTURE_RESULT   = 0x02,
    ACTUATOR_100HZ   = 0x03,
    TAMPER_ALERT     = 0x04,
    HIL_INJECT_FRAME = 0x05,
    POST_BOOT_REPORT = 0x06,
    VISION_2FA_SYNC  = 0x07,
    HEALTH_EXTENDED  = 0x08
};

enum class FaultCode : uint8_t {
    E00_NOMINAL                 = 0,
    E01_I2C_BUS_FAULT           = 1,
    E02_VOLTAGE_ABNORMAL_042    = 2,
    E03_MOTOR_STALL_OVERCURRENT = 3,
    E04_FEEDBACK_MAX_AGE_150MS  = 4,
    E05_TAMPER_ATTACK_LOCKDOWN  = 5,
    E06_SENSOR_DIVERGENCE       = 6,
    E07_POST_OR_OTA_FAIL        = 7,
    E08_HARDWARE_ESTOP_ACTIVE   = 8
};

#pragma pack(push, 1)
struct FrameHeader {
    uint16_t sync;        // 0xAA55
    uint8_t  version;     // 0x01
    uint8_t  msg_type;
    uint16_t seq_num;
    uint64_t t_cap_us;
    uint8_t  payload_len;
};

struct SensorFrame50Hz {
    float   us_x_mm;
    float   us_y_mm;
    float   tof_x_mm;
    float   tof_y_mm;
    float   kalman_x_mm;
    float   kalman_y_mm;
    float   kalman_vx;
    float   kalman_vy;
    float   temp_c;
    uint8_t mahalanobis_flags;
    uint8_t particle_spread_mm;
};

struct GestureInferPayload {
    uint8_t  pred_class;
    int32_t  logits[7];
    int32_t  margin_int32;
    float    bio_cos_sim;
    float    ood_recon_mse;
    int8_t   vec_120[120];
    uint32_t infer_us;
    uint8_t  accepted;
};

struct ActuatorTelemetry100Hz {
    float   target_mm;
    float   pos_mm;
    float   vel_mm_s;
    float   current_ma;
    int16_t pwm_permille;
    int16_t ff_actuatornet;
    uint8_t fsm_state;
    uint8_t fault_flags;
};

struct TamperAlertFrame {
    uint8_t tamper_class;
    float   score;
    float   peak_accel_g;
    float   peak_gyro_dps;
    float   rms_fft_10_40hz;
    uint8_t latched_nvs;
    uint8_t pad[6];
};

struct HilInjectFrame {
    float    fake_us_x_mm;
    float    fake_us_y_mm;
    float    fake_tof_x_mm;
    float    fake_tof_y_mm;
    float    fake_imu_g;
    uint32_t override_mask;
};

struct PostBootReport {
    uint8_t  vectors_passed;
    uint8_t  vectors_total;
    uint16_t i2c_ack_mask;
    float    gyro_var_max;
    uint8_t  flash_mode_ok;
    uint16_t psram_kb;
    uint32_t fw_crc32;
    uint8_t  reserved;
};

struct Vision2FASyncFrame {
    int32_t  aruco_id;
    uint8_t  badge_color_hsv;
    uint8_t  yolo_person_count;
    uint8_t  tailgating_alert;
    float    pose_dist_mm;
    uint8_t  valid_token;
    uint64_t nonce;
};

struct HealthExtendedFrame {
    float    v_mains_v;
    float    v_batt_v;
    float    v_bus_v;
    float    v_5v_est;
    float    temp_c;
    float    rh_pct;
    uint8_t  ats_source;
    uint8_t  fault_code;
    uint16_t stack_hwm_bytes;
};
#pragma pack(pop)

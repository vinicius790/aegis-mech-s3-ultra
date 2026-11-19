#pragma once
// Contrato Hardware v3.0 — ESP32-S3-WROOM-1-N16R8
// Fonte unica de verdade de pinos, constantes mecanicas e protecoes

#include <stdint.h>

namespace AegisPins {
    template <int PIN>
    constexpr int validate_safe_gpio() {
        static_assert(!(PIN >= 26 && PIN <= 37), "FATAL: GPIO 26..37 reservados p/ PSRAM/Flash N16R8!");
        static_assert(PIN != 0 && PIN != 3 && PIN != 45 && PIN != 46, "FATAL: Strapping pin de boot proibido!");
        static_assert(PIN != 19 && PIN != 20, "FATAL: USB-OTG D-/D+ reservado!");
        static_assert(PIN != 43 && PIN != 44, "FATAL: UART0 TX/RX reservado!");
        return PIN;
    }

    // Monitoramento ATS Dual Power (ADC1 12-bit + BAT54S clamp)
    constexpr int V_MAINS_SENSE    = validate_safe_gpio<1>();   // ADC1_CH0 (39k / 10k -> fator 4.90)
    constexpr int V_UPS_BATT_SENSE = validate_safe_gpio<2>();   // ADC1_CH1 (22k / 10k -> fator 3.20)

    // Sensores Ultrassonicos HC-SR04 (RMT TX/RX IDF 4.4)
    constexpr int US_X_TRIG        = validate_safe_gpio<4>();
    constexpr int US_X_ECHO        = validate_safe_gpio<5>();   // Divisor 1.2k / 2.2k obrigatorio (3.235V max)
    constexpr int US_Y_TRIG        = validate_safe_gpio<6>();
    constexpr int US_Y_ECHO        = validate_safe_gpio<7>();   // Divisor 1.2k / 2.2k obrigatorio (3.235V max)

    // Barramento I2C Fast-Mode 400 kHz + XSHUT VL53L1X
    constexpr int I2C_SDA          = validate_safe_gpio<8>();   // Pull-up 4.7k -> 3.3V
    constexpr int I2C_SCL          = validate_safe_gpio<9>();   // Pull-up 4.7k -> 3.3V
    constexpr int TOF_X_XSHUT      = validate_safe_gpio<10>();  // Boot 0x29 -> 0x30
    constexpr int TOF_Y_XSHUT      = validate_safe_gpio<11>();  // Boot 0x29 -> 0x31

    // Inercial & Termometro
    constexpr int IMU_INT          = validate_safe_gpio<12>();
    constexpr int DS18B20_DQ       = validate_safe_gpio<13>();  // Pull-up 4.7k -> 3.3V

    // Potencia & Atuacao (DRV8871 + PC817/IRLZ44N/1N5819 + Fins de Curso)
    constexpr int MOTOR_IN1        = validate_safe_gpio<14>();  // Pull-down 10k externo
    constexpr int MOTOR_IN2        = validate_safe_gpio<15>();  // Pull-down 10k externo
    constexpr int SOLENOID_CTRL    = validate_safe_gpio<16>();  // Pull-down 10k -> 330R -> PC817
    constexpr int LIMIT_OPEN_SW    = validate_safe_gpio<17>();  // Pull-up 10k + RC 1k/100nF
    constexpr int LIMIT_CLOSE_SW   = validate_safe_gpio<18>();  // Pull-up 10k + RC 1k/100nF

    // HMI & Sinalizacao
    constexpr int BTN_MODE         = validate_safe_gpio<21>();
    constexpr int WS2812_RING      = validate_safe_gpio<38>();  // Resistor serie 330R
    constexpr int BUZZER_PWM       = validate_safe_gpio<39>();  // 1k -> Base 2N2222
    constexpr int ONBOARD_RGB      = validate_safe_gpio<48>();

    // Expansoes & E-Stop Hardwired (via GPIO Matrix)
    constexpr int AUX_UART1_TX     = validate_safe_gpio<40>();  // ESP32-CAM 2FA
    constexpr int AUX_UART1_RX     = validate_safe_gpio<41>();  // ESP32-CAM 2FA
    constexpr int ESTOP_SENSE_IN   = validate_safe_gpio<42>();  // INPUT_PULLUP; HIGH = E-Stop ativo
    constexpr int AUX_SERVO_PWM    = validate_safe_gpio<47>();  // Bandeja 2-DOF / HX711_SCK

    // Fator Mecanico Cremalheira m=1.25 / Pinhao Z=16 (D_prim = 20.0 mm)
    constexpr float AS5600_MM_PER_DEG = 0.174532925f;
    constexpr float BOLT_OPEN_MM      = 28.0f;

    static_assert(MOTOR_IN1 != MOTOR_IN2, "FATAL: Pinos de Ponte-H duplicados!");
    static_assert(US_X_ECHO != US_Y_ECHO && US_X_TRIG != US_Y_TRIG, "FATAL: Colisao pinos ultrassom!");
    static_assert(V_MAINS_SENSE != V_UPS_BATT_SENSE && AUX_UART1_TX != AUX_UART1_RX, "FATAL: Colisao ATS/UART!");
}

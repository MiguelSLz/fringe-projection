#pragma once
#include <string>
#include <chrono>
#include <cstdint>
#include <vector>
#include <memory>
#include <atomic>  
#include <thread>  
#include "StereoConfig.hpp"
#include "pwm_trigger.hpp"

struct gpiod_chip;
struct gpiod_line;

namespace stereo {

class GpioController {
public:
    GpioController();
    ~GpioController();

    bool init(const GPIOTriggerConfig& trigCfg, const ZNCCConfig& znccCfg, const EncoderConfig& encodCfg);
    void release();

    // Trigger functionality
    bool generatePulse(unsigned int duration_us = 100);

    // ZNCC functionality
    void setLaser(bool on);
    void moveMotor(float angle);

    bool isInitialized() const { return m_initialized; }

private:
    struct gpiod_chip* m_chip0{nullptr};
    
    // Trigger lines (from chip0)
    std::vector<struct gpiod_line*> m_trigger_lines;
    
    // ZNCC lines
    struct gpiod_line* m_laser_line{nullptr};
    
    // Motor DC via PWM
    std::unique_ptr<PwmTrigger> m_pwmA;    // pino 13
    std::unique_ptr<PwmTrigger> m_pwmB;    // pino 18
    std::string m_pwmChip_a;               // ex: "/sys/class/pwm/pwmchip0"
    std::string m_pwmChip_b;               // ex: "/sys/class/pwm/pwmchip2"
    int m_pwmChannelA{0};
    int m_pwmChannelB{0};
    double m_pwmFrequencyHz{1000.0};
    double m_pwmDutyPercent{0.0};
    bool m_activeLow{false};
    bool m_initialized{false};
    int m_delay_us{3500};

    // Encoder
    std::vector<struct gpiod_line*> m_encoder_lines;
    int m_encoder_pulses_per_rev{2100};

    // Controle da Thread do Encoder
    void encoderThreadLoop();                  
    std::thread m_encoder_thread;                   // obj
    std::atomic<bool> m_encoder_running{false};     // flag para parar o loop
    std::atomic<int32_t> m_encoder_count{0};        // contador de pulsos
};

} // namespace stereo
#include "GpioController.hpp"
#include <iostream>
#include <unordered_map>
#include <thread>
#include <cmath>
#include <algorithm>

#ifdef HAS_GPIOD
#include <gpiod.h>
#endif

namespace stereo {

const std::unordered_map<int, int> ORIN_NANO_PIN_MAP = {
    {7,  144}, {11, 112}, {12, 50},  {13, 122},
    {15, 85},  {16, 126}, {18, 125}, {19, 135},
    {21, 134}, {22, 123}, {23, 133}, {24, 136},
    {26, 137}, {29, 105}, {31, 106}, {32, 41},
    {33, 43},  {35, 53},  {36, 113}, {37, 124},
    {38, 52},  {40, 51}
};
const std::unordered_map<int, int> ORIN_AGX_PIN_MAP = {
    {7,  106}, {11, 112}, {12, 50},  {13, 108},
    {15, 85},  {16, 9}, {18, 43}, {19, 135},
    {21, 134}, {22, 96}, {23, 133}, {24, 136},
    {26, 137}, {29, 1}, {31, 0}, {32, 8},
    {33, 2},  {35, 53},  {36, 113}, {37, 3},
    {38, 52},  {40, 51}
};

static int mapHeaderPinToGpio(int headerPin, const std::string& model) {
    if (model == "JETSON_ORIN_NANO_NX") {
        auto it = ORIN_NANO_PIN_MAP.find(headerPin);
        if (it != ORIN_NANO_PIN_MAP.end()) return it->second;
    } else if (model == "JETSON_AGX_ORIN") {
        auto it = ORIN_AGX_PIN_MAP.find(headerPin);
        if (it != ORIN_AGX_PIN_MAP.end()) return it->second;
    }
    return headerPin;
}


GpioController::~GpioController() {
    release();
}

bool GpioController::init(const GPIOTriggerConfig& trigCfg, const ZNCCConfig& znccCfg, const EncoderConfig& encodCfg) {
    release();

    // pegando os valores da configuração para os atributos do GpioController
    m_activeLow = trigCfg.activeLow;
    m_pwmChip_a = znccCfg.pwmChipA;
    m_pwmChip_b = znccCfg.pwmChipB;
    m_pwmChannelA = znccCfg.pwmChannel;
    m_pwmChannelB = znccCfg.pwmChannel;
    m_pwmFrequencyHz = znccCfg.pwmFrequencyHz;
    m_pwmDutyPercent = znccCfg.pwmDutyPercent;

    // setup motor
    m_pwmA = std::make_unique<PwmTrigger>(m_pwmChip_a,m_pwmChannelA,m_pwmFrequencyHz, m_pwmDutyPercent);
    m_pwmB = std::make_unique<PwmTrigger>(m_pwmChip_b,m_pwmChannelB,m_pwmFrequencyHz, m_pwmDutyPercent);
    m_pwmA->start();
    m_pwmB->start();

#ifdef HAS_GPIOD
    m_chip0 = gpiod_chip_open_by_name("gpiochip0");
    if (!m_chip0) {
        std::cerr << "[GpioController] Error: Failed to open gpiochip0\n";
        return false;
    }

    // 1. Setup Trigger Pins (on chip0)
    for (int pin : trigCfg.pins) {
        int line_offset = pin;
        if (trigCfg.pinType == "HEADER_PIN") {
            line_offset = mapHeaderPinToGpio(pin, trigCfg.jetsonModel);
        }
        struct gpiod_line* line = gpiod_chip_get_line(m_chip0, line_offset);
        if (line) {
            if (gpiod_line_request_output(line, "StereoTrigger", m_activeLow ? 1 : 0) == 0) {
                m_trigger_lines.push_back(line);
                std::cout << "[GpioController] Trigger line " << line_offset << " configured.\n";
            }
        }
    }

    // 2. Setup Laser Pin (on chip0)
    m_laser_line = gpiod_chip_get_line(m_chip0, znccCfg.laserPin);
    if (m_laser_line) {
        if (gpiod_line_request_output(m_laser_line, "ZNCC_Laser", 0) == 0) {
            std::cout << "[GpioController] Laser line " << znccCfg.laserPin << " configured.\n";
        } else {
            m_laser_line = nullptr;
        }
    }

    // 3. Setup Encoder Pins
    if (encodCfg.pins.size() >= 2) {
        int pinA = encodCfg.pins[0];
        int pinB = encodCfg.pins[1]; 
        if (encodCfg.pinType == "HEADER_PIN") {
            pinA = mapHeaderPinToGpio(pinA, encodCfg.jetsonModel);
            pinB = mapHeaderPinToGpio(pinB, encodCfg.jetsonModel);
        }
        struct gpiod_line* lineA = gpiod_chip_get_line(m_chip0, pinA);
        struct gpiod_line* lineB = gpiod_chip_get_line(m_chip0, pinB);
        if (!lineA || !lineB) {
            std::cerr << "[GpioController] Error: Failed to get encoder lines from chip0 (A=" 
                      << pinA << ", B=" << pinB << ")\n";
            return false;
        }
        // Canal A: interrupções por hardware em AMBAS as bordas (subida e descida)
        if (gpiod_line_request_both_edges_events(lineA, "Encoder_A") < 0) {
            std::cerr << "[GpioController] Error: Failed to request edge events on Encoder line A (" 
                      << pinA << ")\n";
            return false;
        }
        // Canal B: entrada normal (apenas lê o nível no instante da borda de A)
        if (gpiod_line_request_input(lineB, "Encoder_B") < 0) {
            std::cerr << "[GpioController] Error: Failed to request input on Encoder line B (" 
                      << pinB << ")\n";
            gpiod_line_release(lineA);
            return false;
        }
        m_encoder_lines.push_back(lineA); // m_encoder_lines[0] = Canal A
        m_encoder_lines.push_back(lineB); // m_encoder_lines[1] = Canal B

        std::cout << "[GpioController] Encoder lines configuradas: A=" 
                  << pinA << " (interrupt), B=" << pinB << " (level)\n";

        // 4. Start Encoder Thread
        m_encoder_count = 0;
        m_encoder_running = true;
        m_encoder_thread = std::thread(&GpioController::encoderThreadLoop, this);
        std::cout << "[GpioController] Thread do encoder iniciada.\n";

    } else {
        std::cerr << "[GpioController] Warning: EncoderConfig requires at least 2 pins (A and B).\n";
    }

    m_initialized = true;
    return true;
#else
    std::cout << "[GpioController] HAS_GPIOD not defined. Mock mode.\n";
    m_initialized = true;
    return true;
#endif



}

void GpioController::release() {
#ifdef HAS_GPIOD

    // 1. Para a thread do encoder primeiro
    if (m_encoder_running.load()) {
        m_encoder_running.store(false);
        if (m_encoder_thread.joinable()) {
            m_encoder_thread.join();
        }
    }
    for (auto line : m_encoder_lines) {
        if (line) gpiod_line_release(line);
    }   
    m_encoder_lines.clear();

    for (auto line : m_trigger_lines) {
        if (line) gpiod_line_release(line);
    }
    m_trigger_lines.clear();

    if (m_laser_line) {
        gpiod_line_set_value(m_laser_line, 0);
        gpiod_line_release(m_laser_line);
        m_laser_line = nullptr;
    }

    if (m_chip0) {
        gpiod_chip_close(m_chip0);
        m_chip0 = nullptr;
    }
#endif

    m_initialized = false;
}

bool GpioController::generatePulse(unsigned int duration_us) {
    if (!m_initialized) return false;

#ifdef HAS_GPIOD
    int high_val = m_activeLow ? 0 : 1;
    int low_val  = m_activeLow ? 1 : 0;

    for (auto line : m_trigger_lines) {
        gpiod_line_set_value(line, high_val);
    }
    std::this_thread::sleep_for(std::chrono::microseconds(duration_us));
    for (auto line : m_trigger_lines) {
        gpiod_line_set_value(line, low_val);
    }
#else
    std::cout << "[GpioController] Mock Trigger Pulse\n";
#endif
    return true;
}

void GpioController::setLaser(bool on) {
    if (!m_initialized) return;
#ifdef HAS_GPIOD
    if(m_laser_line) {
        gpiod_line_set_value(m_laser_line, on ? 1 : 0);
    }
#else
    std::cout << "[GpioController] Mock Laser " << (on ? "ON" : "OFF") << "\n";
#endif
}

void GpioController::moveMotor(float angle) {

    if (!m_initialized) return;
    

    /*
    if (!m_initialized) return;
    int steps = static_cast<int>((angle / 360.0f) * m_steps_per_rev);
    if (steps == 0) {
#ifdef HAS_GPIOD
        for (auto line : m_motor_lines) {
            gpiod_line_set_value(line, 0);
        }
#endif
        return;
    }

    bool clockwise = (steps > 0);
    steps = std::abs(steps);
    
    std::vector<std::vector<int>> current_seq = m_step_sequence;
    if (!clockwise) {
        std::reverse(current_seq.begin(), current_seq.end());
    }

    for (int i = 0; i < steps; ++i) {
        const auto& step = current_seq[i % current_seq.size()];
#ifdef HAS_GPIOD
        for (size_t j = 0; j < m_motor_lines.size() && j < step.size(); ++j) {
            gpiod_line_set_value(m_motor_lines[j], step[j]);
        }
#endif
        std::this_thread::sleep_for(std::chrono::microseconds(m_delay_us));
    }*/
}

void GpioController::encoderThreadLoop() {
    
    if (m_encoder_lines.size() != 2) return;

    struct gpiod_line* lineA = m_encoder_lines[0];
    struct gpiod_line* lineB = m_encoder_lines[1];

    const struct timespec timeout = {0, 100000000}; // 100ms timeout para checar m_encoder_running
    struct gpiod_line_event event;

    while (m_encoder_running.load()){
        
        int evento = gpiod_line_event_wait(lineA, &timeout); // =1 quando acontece um evento

        if (evento==1){
            if (gpiod_line_event_read(lineA, &event) == 0) { // leitura da borda bem sucedida
                
                int val_b = gpiod_line_get_value(lineB);
                
                // m_encoder_count aumenta com o sentido horario do motor
                if (event.event_type == GPIOD_LINE_EVENT_RISING_EDGE){
                    if (val_b == 0) m_encoder_count++;
                    else m_encoder_count--;
                } else if (event.event_type == GPIOD_LINE_EVENT_FALLING_EDGE){
                    if (val_b == 1) m_encoder_count++;
                    else m_encoder_count--;
                }

            }   
        }
    }

}

} // namespace stereo

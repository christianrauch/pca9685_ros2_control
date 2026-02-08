#ifndef RPY_PWM_HAT_PCA9685_H
#define RPY_PWM_HAT_PCA9685_H

#include <string>
#include <cstdint>

namespace PiPCA9685 {

class PCA9685 {
public:
    explicit PCA9685(const std::string &device = "/dev/i2c-1", int address = 0x40);
    ~PCA9685();

    void set_pwm_freq(const double freq_hz);

    void set_pwm(const int channel, const uint16_t on, const uint16_t off);

    void set_pwm_ms(const int channel, const double ms);

private:
    int bus_fd;

    // Default frequency pulled from PCA9685 datasheet.
    double frequency = 200.0;

};

}  // namespace PiPCA9685

#endif //RPY_PWM_HAT_PCA9685_H

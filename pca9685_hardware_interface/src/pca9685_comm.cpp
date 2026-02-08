#include "pca9685_hardware_interface/pca9685_comm.h"
#include <unistd.h>
#include <cmath>

#include <system_error>

#include <sys/ioctl.h>
#include <fcntl.h>
extern "C" {
#include <linux/i2c-dev.h>
#include <i2c/smbus.h>
}

namespace PiPCA9685 {

// Registers/etc:
constexpr uint8_t R_MODE1            = 0x00;
constexpr uint8_t R_MODE2            = 0x01;
constexpr uint8_t R_PRESCALE         = 0xFE;
constexpr uint8_t R_LED0_ON_L        = 0x06;
constexpr uint8_t R_LED0_ON_H        = 0x07;
constexpr uint8_t R_LED0_OFF_L       = 0x08;
constexpr uint8_t R_LED0_OFF_H       = 0x09;

// Bits:
constexpr uint8_t SLEEP              = 0x10;

PCA9685::PCA9685(const std::string &device, int address) {
  bus_fd = open(device.c_str(), O_RDWR);
  if(bus_fd < 0) {
    throw std::system_error(errno, std::system_category(), "Could not open i2c bus.");
  }

  if(ioctl(bus_fd, I2C_SLAVE, address) < 0) {
    throw std::system_error(errno, std::system_category(), "Could not set peripheral address.");
  }

  i2c_smbus_write_byte_data(bus_fd, R_MODE2, 0b0000'0100); // OUTDRV
  usleep(5'000);
  auto mode1_val = i2c_smbus_read_byte_data(bus_fd, R_MODE1);
  mode1_val &= ~SLEEP;
  i2c_smbus_write_byte_data(bus_fd, R_MODE1, mode1_val);
  usleep(5'000);
}

PCA9685::~PCA9685() {
  close(bus_fd);
}

void PCA9685::set_pwm_freq(const double freq_hz) {
  frequency = freq_hz;

  auto prescaleval = 2.5e7; //    # 25MHz
  prescaleval /= 4096.0; //       # 12-bit
  prescaleval /= freq_hz;
  prescaleval -= 1.0;

  auto prescale = static_cast<int>(std::round(prescaleval));

  const auto oldmode = i2c_smbus_read_byte_data(bus_fd, R_MODE1);

  auto newmode = (oldmode & 0x7F) | SLEEP;

  i2c_smbus_write_byte_data(bus_fd, R_MODE1, newmode);
  i2c_smbus_write_byte_data(bus_fd, R_PRESCALE, prescale);
  i2c_smbus_write_byte_data(bus_fd, R_MODE1, oldmode);
  usleep(5'000);
  i2c_smbus_write_byte_data(bus_fd, R_MODE1, oldmode | 0b1000'0000);
}

void PCA9685::set_pwm(const int channel, const uint16_t on, const uint16_t off) {
  const auto channel_offset = 4 * channel;
  i2c_smbus_write_byte_data(bus_fd, R_LED0_ON_L+channel_offset, on & 0xFF);
  i2c_smbus_write_byte_data(bus_fd, R_LED0_ON_H+channel_offset, on >> 8);
  i2c_smbus_write_byte_data(bus_fd, R_LED0_OFF_L+channel_offset, off & 0xFF);
  i2c_smbus_write_byte_data(bus_fd, R_LED0_OFF_H+channel_offset, off >> 8);
}

void PCA9685::set_pwm_ms(const int channel, const double ms) {
  auto period_ms = 1000.0 / frequency;
  auto bits_per_ms = 4096 / period_ms;
  auto bits = ms * bits_per_ms;
  set_pwm(channel, 0, bits);
}

}  // namespace PiPCA9685

#ifndef PCA9685_HARDWARE_INTERFACE__PCA9685_SYSTEM_HPP_
#define PCA9685_HARDWARE_INTERFACE__PCA9685_SYSTEM_HPP_

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "pca9685_hardware_interface/visibility_control.h"

#include <sys/ioctl.h>
#include <fcntl.h>
extern "C" {
#include <linux/i2c-dev.h>
#include <i2c/smbus.h>
}


class PCA9685 {
public:
  explicit PCA9685(const std::string &device = "/dev/i2c-1", int address = 0x40) {
    bus_fd = open(device.c_str(), O_RDWR);
    if(bus_fd < 0) {
      throw std::system_error(errno, std::system_category(), "Could not open i2c bus.");
    }

    if(ioctl(bus_fd, I2C_SLAVE, address) < 0) {
      throw std::system_error(errno, std::system_category(), "Could not set peripheral address.");
    }

    // OUTDRV: totem pole structure
    i2c_smbus_write_byte_data(bus_fd, R_MODE2, 0b0000'0100);
    usleep(5'000);
  }

  ~PCA9685() {
    close(bus_fd);
  }

  void set_pwm_freq(const double freq_hz) {
    frequency = freq_hz;

    auto prescaleval = 2.5e7; //    # 25MHz
    prescaleval /= 4096.0; //       # 12-bit
    prescaleval /= freq_hz;
    prescaleval -= 1.0;

    auto prescale = static_cast<int>(std::round(prescaleval));

    const auto oldmode = i2c_smbus_read_byte_data(bus_fd, R_MODE1);

    auto newmode = (oldmode & 0x7F) | 0b0001'0000;

    i2c_smbus_write_byte_data(bus_fd, R_MODE1, newmode);
    i2c_smbus_write_byte_data(bus_fd, R_PRESCALE, prescale);
    i2c_smbus_write_byte_data(bus_fd, R_MODE1, oldmode);
    usleep(5'000);
    i2c_smbus_write_byte_data(bus_fd, R_MODE1, oldmode | 0b1000'0000);
  }

  void set_pwm(const int channel, const uint16_t on, const uint16_t off) {
    const auto channel_offset = 4 * channel;
    i2c_smbus_write_byte_data(bus_fd, R_LED0_ON_L+channel_offset, on & 0xFF);
    i2c_smbus_write_byte_data(bus_fd, R_LED0_ON_H+channel_offset, on >> 8);
    i2c_smbus_write_byte_data(bus_fd, R_LED0_OFF_L+channel_offset, off & 0xFF);
    i2c_smbus_write_byte_data(bus_fd, R_LED0_OFF_H+channel_offset, off >> 8);
  }

  void set_pwm_ms(const int channel, const double ms) {
    auto period_ms = 1000.0 / frequency;
    auto bits_per_ms = 4096 / period_ms;
    auto bits = ms * bits_per_ms;
    set_pwm(channel, 0, bits);
  }

private:
  int bus_fd;

  // Default frequency pulled from PCA9685 datasheet.
  double frequency = 200.0;

  static constexpr uint8_t R_MODE1            = 0x00;
  static constexpr uint8_t R_MODE2            = 0x01;
  static constexpr uint8_t R_PRESCALE         = 0xFE;
  static constexpr uint8_t R_LED0_ON_L        = 0x06;
  static constexpr uint8_t R_LED0_ON_H        = 0x07;
  static constexpr uint8_t R_LED0_OFF_L       = 0x08;
  static constexpr uint8_t R_LED0_OFF_H       = 0x09;

};


namespace pca9685_hardware_interface
{
class Pca9685SystemHardware : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(Pca9685SystemHardware);

  PCA9685_HARDWARE_INTERFACE_PUBLIC
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareComponentInterfaceParams & params) override;

  PCA9685_HARDWARE_INTERFACE_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  PCA9685_HARDWARE_INTERFACE_PUBLIC
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  PCA9685_HARDWARE_INTERFACE_PUBLIC
  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  PCA9685_HARDWARE_INTERFACE_PUBLIC
  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  PCA9685_HARDWARE_INTERFACE_PUBLIC
  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  PCA9685_HARDWARE_INTERFACE_PUBLIC
  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  std::vector<double> hw_commands_;
  PCA9685 pca;
  double command_to_duty_cycle(double command);
};

}  // namespace pca9685_hardware_interface

#endif  // PCA9685_HARDWARE_INTERFACE__PCA9685_SYSTEM_HPP_

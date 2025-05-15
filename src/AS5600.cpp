// Copyright 2025 Pavel Suprunov
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "foc/encoder/AS5600.hpp"

#include <algorithm>
#include <cstring>
#include <esp_log.h>

namespace {

auto const TAG = "AS5600";

[[maybe_unused]] auto const DEVICE_ADDRESS = 0x36;     ///< Адрес устройства
[[maybe_unused]] auto const REGISTER_ZMCO = 0x00;      ///< Количество записей
[[maybe_unused]] auto const REGISTER_ZPOS = 0x01;      ///< Минимальное положение
[[maybe_unused]] auto const REGISTER_MPOS = 0x03;      ///< Максимальное положение
[[maybe_unused]] auto const REGISTER_MANG = 0x05;      ///< Диапазон значений
[[maybe_unused]] auto const REGISTER_CONF = 0x07;      ///<  Конфигурация
[[maybe_unused]] auto const REGISTER_STATUS = 0x0B;    ///< Статус
[[maybe_unused]] auto const REGISTER_RAW_ANGLE = 0x0C; ///< Угол (сырое значение)
[[maybe_unused]] auto const REGISTER_ANGLE = 0x0E;     ///< Угол (значение после фильтрации)
[[maybe_unused]] auto const REGISTER_AGC = 0x1A;       ///< Automatic Gain Control
[[maybe_unused]] auto const REGISTER_MAGNITUDE = 0x1B; ///< Магнитуда
[[maybe_unused]] auto const REGISTER_BURN = 0xFF;      ///< Запись в ПЗУ

template <typename T> auto convertBytesToData(std::vector<std::uint8_t> bytes) -> T {
  T struct_{};

  std::ranges::reverse(bytes);
  std::memcpy(&struct_, bytes.data(), bytes.size());

  return struct_;
}

template <typename T> auto convertDataToBytes(T struct_) -> std::vector<uint8_t> {
  std::vector<uint8_t> bytes(sizeof(T));

  std::memcpy(bytes.data(), &struct_, sizeof(T));
  std::ranges::reverse(bytes);

  return bytes;
}

} // namespace

namespace foc {

auto AS5600::create(i2c::Master::Pin const sda, i2c::Master::Pin const scl, i2c::Master::Port const port) -> std::expected<AS5600::Pointer, AS5600::Error> {
  return AS5600::create(sda, scl, port, DEVICE_ADDRESS);
}

auto AS5600::create(i2c::Master::Pin const sda, i2c::Master::Pin const scl, i2c::Master::Port const port, i2c::Device::Address const address) -> std::expected<Pointer, Error> {
  auto bus = i2c::Master::create(sda, scl, port);
  if (not bus) {
    return std::unexpected(AS5600::Error::I2C_BUS_CREATED_ERROR);
  }

  auto device = (*bus)->createDevice(address);
  if (not device) {
    return std::unexpected(AS5600::Error::I2C_DEVICE_CREATED_ERROR);
  }

  return AS5600::Pointer(new AS5600(std::move(*bus), std::move(*device)));
}

AS5600::AS5600(i2c::Master::Pointer bus_, i2c::Device::Pointer device_) noexcept : bus(std::move(bus_)), device(std::move(device_)) {
  m_status = getStatus();
  ESP_LOGI(TAG, "Magnet low: %d", m_status.magnetLow);
  ESP_LOGI(TAG, "Magnet high: %d", m_status.magnetHigh);
  ESP_LOGI(TAG, "Magnet detected: %d", m_status.magnetDetected);

  m_configuration = getConfiguration();
  ESP_LOGI(TAG, "Power mode: %d", m_configuration.powerMode);
  ESP_LOGI(TAG, "Hysteresis: %d", m_configuration.hysteresis);
  ESP_LOGI(TAG, "Output stage: %d", m_configuration.outputStage);
  ESP_LOGI(TAG, "PWM Frequency: %d", m_configuration.pwmFrequency);
  ESP_LOGI(TAG, "Slow filter: %d", m_configuration.slowFilter);
  ESP_LOGI(TAG, "Fast filter threshold: %d", m_configuration.fastFilterThreshold);
  ESP_LOGI(TAG, "Watchdog: %d", m_configuration.watchdog);
}

void AS5600::init() { EncoderBase::init(); }

[[maybe_unused]] auto AS5600::setPowerMode(AS5600::PowerMode powerMode) -> void {
  m_configuration.powerMode = powerMode;

  auto const configurationData = convertDataToBytes(m_configuration);

  device->write(REGISTER_CONF, configurationData);
}

[[maybe_unused]] auto AS5600::setHysteresis(AS5600::Hysteresis hysteresis) -> void {
  m_configuration.hysteresis = hysteresis;

  auto const configurationData = convertDataToBytes(m_configuration);

  device->write(REGISTER_CONF, configurationData);
}

[[maybe_unused]] auto AS5600::setOutputStage(AS5600::OutputStage outputStage) -> void {
  m_configuration.outputStage = outputStage;

  auto const configurationData = convertDataToBytes(m_configuration);

  device->write(REGISTER_CONF, configurationData);
}

[[maybe_unused]] auto AS5600::setPWMFrequency(AS5600::PWMFrequency pwmFrequency) -> void {
  m_configuration.pwmFrequency = pwmFrequency;

  auto const configurationData = convertDataToBytes(m_configuration);

  device->write(REGISTER_CONF, configurationData);
}

[[maybe_unused]] auto AS5600::setSlowFilter(AS5600::SlowFilter slowFilter) -> void {
  m_configuration.slowFilter = slowFilter;

  auto const configurationData = convertDataToBytes(m_configuration);

  device->write(REGISTER_CONF, configurationData);
}

[[maybe_unused]] auto AS5600::setFastFilterThreshold(AS5600::FastFilterThreshold fastFilterThreshold) -> void {
  m_configuration.fastFilterThreshold = fastFilterThreshold;

  auto const configurationData = convertDataToBytes(m_configuration);

  device->write(REGISTER_CONF, configurationData);
}

[[maybe_unused]] auto AS5600::setWatchdog(bool enable) -> void {
  m_configuration.watchdog = enable;

  auto const configurationData = convertDataToBytes(m_configuration);

  device->write(REGISTER_CONF, configurationData);
}

auto AS5600::getStatus() -> AS5600::Status {
  auto const statusData = device->read(REGISTER_STATUS);

  m_status = convertBytesToData<AS5600::Status>(statusData);

  return m_status;
}

auto AS5600::getConfiguration() -> AS5600::Configuration {
  auto const configurationData = device->read(REGISTER_CONF, 2);

  m_configuration = convertBytesToData<AS5600::Configuration>(configurationData);

  return m_configuration;
}

auto AS5600::getMechanicalAngle() -> float { return EncoderBase::getMechanicalAngle(); }

auto AS5600::getAngle() -> float { return EncoderBase::getAngle(); }

auto AS5600::getVelocity() -> float { return EncoderBase::getVelocity(); }

auto AS5600::getFullRotations() -> AS5600::Rotations { return EncoderBase::getFullRotations(); }

auto AS5600::getRotations() -> AS5600::Rotations { return AS5600::Rotations(); }

auto AS5600::getPreciseAngle() -> AS5600::PreciseAngle { return EncoderBase::getPreciseAngle(); }

auto AS5600::update() -> void { EncoderBase::update(); }

auto AS5600::needsSearch() -> int { return EncoderBase::needsSearch(); }

auto AS5600::getSensorAngle() -> float {
  auto const angleData = device->read(REGISTER_ANGLE, 2);

  return convertBytesToData<AS5600::Angle>(angleData);
}

} // namespace foc

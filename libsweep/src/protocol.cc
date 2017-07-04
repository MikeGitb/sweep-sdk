#include <chrono>
#include <thread>

#include "protocol.hpp"

namespace sweep {
namespace protocol {

void write_command(serial::device_s serial, const uint8_t cmd[2]) {
  SWEEP_ASSERT(serial);
  SWEEP_ASSERT(cmd);

  cmd_packet_s packet;
  packet.cmdByte1 = cmd[0];
  packet.cmdByte2 = cmd[1];
  packet.cmdParamTerm = '\n';

  // pause for 2ms, so the device is never bombarded with back to back commands
  std::this_thread::sleep_for(std::chrono::milliseconds(2));

  serial::device_write(serial, &packet, sizeof(packet));
}

void write_command_with_arguments(serial::device_s serial, const uint8_t cmd[2], const uint8_t arg[2]) {
  SWEEP_ASSERT(serial);
  SWEEP_ASSERT(cmd);
  SWEEP_ASSERT(arg);

  cmd_param_packet_s packet;
  packet.cmdByte1 = cmd[0];
  packet.cmdByte2 = cmd[1];
  packet.cmdParamByte1 = arg[0];
  packet.cmdParamByte2 = arg[1];
  packet.cmdParamTerm = '\n';

  serial::device_write(serial, &packet, sizeof(packet));
}

namespace {
template <class Msg>
Msg read_response(serial::device_s serial, const uint8_t cmd[2]) {
  Msg response;
  serial::device_read(serial, &response, sizeof(response));

  const uint8_t checksum = response.calc_checksum();
  if (checksum != response.cmdSum)
    throw error{"invalid response header checksum"};

  const bool ok = response.cmdByte1 == cmd[0] && response.cmdByte2 == cmd[1];
  if (!ok)
    throw error{"invalid header response commands"};

  return response;
}
} // namespace

response_header_s read_response_header(serial::device_s serial, const uint8_t cmd[2]) {
  SWEEP_ASSERT(serial);
  SWEEP_ASSERT(cmd);

  return read_response<response_header_s>(serial, cmd);
}

response_param_s read_response_param(serial::device_s serial, const uint8_t cmd[2]) {
  SWEEP_ASSERT(serial);
  SWEEP_ASSERT(cmd);

  return read_response<response_param_s>(serial, cmd);
}

response_scan_packet_s read_response_scan(serial::device_s serial) {
  SWEEP_ASSERT(serial);

  response_scan_packet_s response;
  serial::device_read(serial, &response, sizeof(response));

  const uint8_t checksum = response.calc_checksum();
  if (checksum != response.checksum)
    throw error{"invalid scan response commands"};

  return response;
}

response_info_motor_ready_s read_response_info_motor_ready(serial::device_s serial) {
  SWEEP_ASSERT(serial);

  response_info_motor_ready_s response;
  serial::device_read(serial, &response, sizeof(response));

  const bool ok = response.cmdByte1 == MOTOR_READY[0] && response.cmdByte2 == MOTOR_READY[1];
  if (!ok)
    throw error{"invalid motor ready response commands"};

  return response;
}

response_info_motor_speed_s read_response_info_motor_speed(serial::device_s serial) {
  SWEEP_ASSERT(serial);

  response_info_motor_speed_s response;
  serial::device_read(serial, &response, sizeof(response));

  const bool ok = response.cmdByte1 == MOTOR_INFORMATION[0] && response.cmdByte2 == MOTOR_INFORMATION[1];
  if (!ok)
    throw error{"invalid motor info response commands"};

  return response;
}

response_info_sample_rate_s read_response_info_sample_rate(sweep::serial::device_s serial) {
  SWEEP_ASSERT(serial);

  response_info_sample_rate_s response;
  serial::device_read(serial, &response, sizeof(response));

  const bool ok = response.cmdByte1 == SAMPLE_RATE_INFORMATION[0] && response.cmdByte2 == SAMPLE_RATE_INFORMATION[1];
  if (!ok)
    throw error{"invalid sample rate info response commands"};

  return response;
}

} // ns protocol
} // ns sweep

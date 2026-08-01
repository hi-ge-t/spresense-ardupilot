/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * This is a deliberately small, read-only MAVLink endpoint. It identifies as
 * ArduPilot/Copter so a GCS can exercise the future transport boundary, but it
 * does not link Copter flight control and must not be treated as flight-ready.
 */

#include "m1_gcs_protocol.h"

#include <string.h>

struct m1_parameter
{
  const char id[16];
  float value;
};

static const struct m1_parameter g_parameters[M1_GCS_PARAMETER_COUNT] =
{
  {"M1_OUT_EN", 0.0f},
  {"M1_GNSS_REQ", 1.0f},
  {"M1_GNSS_RAM", 1.0f},
  {"M1_STAGE", 1.0f},
};

static int m1_gcs_target_matches(uint8_t target_system,
                                 uint8_t target_component)
{
  return (target_system == 0u || target_system == M1_GCS_SYSTEM_ID) &&
         (target_component == 0u ||
          target_component == M1_GCS_COMPONENT_ID);
}

static int m1_gcs_send_message(struct m1_gcs_protocol *protocol,
                               mavlink_message_t *message)
{
  uint8_t frame[MAVLINK_MAX_PACKET_LEN];
  const uint16_t length = mavlink_msg_to_send_buffer(frame, message);
  const int result = protocol->send(protocol->send_context, frame, length);

  if (result == 0)
    {
      protocol->transmitted_messages++;
    }

  return result;
}

static int m1_gcs_send_parameter(struct m1_gcs_protocol *protocol,
                                 uint16_t index)
{
  mavlink_message_t message;

  if (index >= M1_GCS_PARAMETER_COUNT)
    {
      return -1;
    }

  mavlink_msg_param_value_pack_status(
    M1_GCS_SYSTEM_ID, M1_GCS_COMPONENT_ID, &protocol->tx_status, &message,
    g_parameters[index].id, g_parameters[index].value, MAV_PARAM_TYPE_REAL32,
    M1_GCS_PARAMETER_COUNT, index);
  return m1_gcs_send_message(protocol, &message);
}

static int m1_gcs_find_parameter(const char id[16])
{
  uint16_t index;

  for (index = 0u; index < M1_GCS_PARAMETER_COUNT; index++)
    {
      if (memcmp(id, g_parameters[index].id, sizeof(g_parameters[index].id)) ==
          0)
        {
          return (int)index;
        }
    }

  return -1;
}

static int m1_gcs_send_autopilot_version(struct m1_gcs_protocol *protocol)
{
  static const uint8_t flight_custom_version[8] =
    {'M', '1', 'G', 'C', 'S', '0', '0', '1'};
  static const uint8_t zero8[8] = {0};
  static const uint8_t zero18[18] = {0};
  const uint64_t capabilities = MAV_PROTOCOL_CAPABILITY_PARAM_FLOAT |
                                MAV_PROTOCOL_CAPABILITY_MAVLINK2;
  const uint32_t flight_sw_version = (4u << 24) | (7u << 16);
  mavlink_message_t message;

  mavlink_msg_autopilot_version_pack_status(
    M1_GCS_SYSTEM_ID, M1_GCS_COMPONENT_ID, &protocol->tx_status, &message,
    capabilities, flight_sw_version, 0u, 0u, 1u, flight_custom_version, zero8,
    zero8, 0x054cu, 0u, 0u, zero18);
  return m1_gcs_send_message(protocol, &message);
}

static int m1_gcs_send_command_ack(struct m1_gcs_protocol *protocol,
                                   uint16_t command, uint8_t result,
                                   const mavlink_message_t *request)
{
  mavlink_message_t message;

  mavlink_msg_command_ack_pack_status(
    M1_GCS_SYSTEM_ID, M1_GCS_COMPONENT_ID, &protocol->tx_status, &message,
    command, result, UINT8_MAX, 0, request->sysid, request->compid);
  return m1_gcs_send_message(protocol, &message);
}

static void m1_gcs_handle_command_long(struct m1_gcs_protocol *protocol,
                                       const mavlink_message_t *message)
{
  mavlink_command_long_t command;

  mavlink_msg_command_long_decode(message, &command);
  if (!m1_gcs_target_matches(command.target_system,
                             command.target_component))
    {
      return;
    }

  if (command.command == MAV_CMD_COMPONENT_ARM_DISARM)
    {
      protocol->arm_reject_count++;
      (void)m1_gcs_send_command_ack(protocol, command.command,
                                    MAV_RESULT_DENIED, message);
      return;
    }

  if ((command.command == MAV_CMD_REQUEST_MESSAGE &&
       (uint32_t)command.param1 == MAVLINK_MSG_ID_AUTOPILOT_VERSION) ||
      command.command == MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES)
    {
      (void)m1_gcs_send_autopilot_version(protocol);
      (void)m1_gcs_send_command_ack(protocol, command.command,
                                    MAV_RESULT_ACCEPTED, message);
      return;
    }

  (void)m1_gcs_send_command_ack(protocol, command.command,
                                MAV_RESULT_UNSUPPORTED, message);
}

static void m1_gcs_handle_parameter_request_list(
  struct m1_gcs_protocol *protocol, const mavlink_message_t *message)
{
  mavlink_param_request_list_t request;
  uint16_t index;

  mavlink_msg_param_request_list_decode(message, &request);
  if (!m1_gcs_target_matches(request.target_system, request.target_component))
    {
      return;
    }

  for (index = 0u; index < M1_GCS_PARAMETER_COUNT; index++)
    {
      (void)m1_gcs_send_parameter(protocol, index);
    }
}

static void m1_gcs_handle_parameter_request_read(
  struct m1_gcs_protocol *protocol, const mavlink_message_t *message)
{
  mavlink_param_request_read_t request;
  int index;

  mavlink_msg_param_request_read_decode(message, &request);
  if (!m1_gcs_target_matches(request.target_system, request.target_component))
    {
      return;
    }

  index = request.param_index;
  if (index < 0)
    {
      index = m1_gcs_find_parameter(request.param_id);
    }
  if (index >= 0 && index < (int)M1_GCS_PARAMETER_COUNT)
    {
      (void)m1_gcs_send_parameter(protocol, (uint16_t)index);
    }
}

static void m1_gcs_handle_parameter_set(struct m1_gcs_protocol *protocol,
                                        const mavlink_message_t *message)
{
  mavlink_param_set_t request;
  int index;

  mavlink_msg_param_set_decode(message, &request);
  if (!m1_gcs_target_matches(request.target_system, request.target_component))
    {
      return;
    }

  protocol->parameter_write_reject_count++;
  index = m1_gcs_find_parameter(request.param_id);
  if (index >= 0)
    {
      (void)m1_gcs_send_parameter(protocol, (uint16_t)index);
    }
}

static void m1_gcs_handle_message(struct m1_gcs_protocol *protocol,
                                  const mavlink_message_t *message)
{
  switch (message->msgid)
    {
      case MAVLINK_MSG_ID_COMMAND_LONG:
        m1_gcs_handle_command_long(protocol, message);
        break;
      case MAVLINK_MSG_ID_PARAM_REQUEST_LIST:
        m1_gcs_handle_parameter_request_list(protocol, message);
        break;
      case MAVLINK_MSG_ID_PARAM_REQUEST_READ:
        m1_gcs_handle_parameter_request_read(protocol, message);
        break;
      case MAVLINK_MSG_ID_PARAM_SET:
        m1_gcs_handle_parameter_set(protocol, message);
        break;
      default:
        break;
    }
}

void m1_gcs_protocol_init(struct m1_gcs_protocol *protocol,
                          m1_gcs_send_fn send, void *send_context)
{
  memset(protocol, 0, sizeof(*protocol));
  protocol->send = send;
  protocol->send_context = send_context;
}

void m1_gcs_protocol_receive(struct m1_gcs_protocol *protocol,
                             const uint8_t *bytes, size_t length)
{
  size_t offset;

  for (offset = 0u; offset < length; offset++)
    {
      mavlink_message_t message;
      mavlink_status_t status;
      const uint8_t framing = mavlink_frame_char_buffer(
        &protocol->rx_buffer, &protocol->rx_status, bytes[offset], &message,
        &status);

      if (framing == MAVLINK_FRAMING_OK)
        {
          protocol->received_messages++;
          m1_gcs_handle_message(protocol, &message);
        }
    }
}

int m1_gcs_protocol_send_heartbeat(struct m1_gcs_protocol *protocol)
{
  mavlink_message_t message;

  mavlink_msg_heartbeat_pack_status(
    M1_GCS_SYSTEM_ID, M1_GCS_COMPONENT_ID, &protocol->tx_status, &message,
    MAV_TYPE_QUADROTOR, MAV_AUTOPILOT_ARDUPILOTMEGA,
    MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 0u, MAV_STATE_STANDBY);
  return m1_gcs_send_message(protocol, &message);
}

int m1_gcs_protocol_send_boot_status(struct m1_gcs_protocol *protocol)
{
  static const char text[50] = "SPRESENSE M1 OUTPUT DISABLED";
  mavlink_message_t message;

  mavlink_msg_statustext_pack_status(
    M1_GCS_SYSTEM_ID, M1_GCS_COMPONENT_ID, &protocol->tx_status, &message,
    MAV_SEVERITY_WARNING, text, 0u, 0u);
  return m1_gcs_send_message(protocol, &message);
}

uint32_t m1_gcs_physical_write_count(void)
{
  return 0u;
}

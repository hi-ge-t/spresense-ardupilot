/* SPDX-License-Identifier: GPL-3.0-or-later */

#include "m1_gcs_protocol.h"

#include <assert.h>
#include <stdio.h>
#include <string.h>

struct capture
{
  uint8_t bytes[4096];
  size_t length;
};

static int capture_send(void *context, const uint8_t *bytes, size_t length)
{
  struct capture *capture = context;

  assert(length <= sizeof(capture->bytes) - capture->length);
  memcpy(&capture->bytes[capture->length], bytes, length);
  capture->length += length;
  return 0;
}

static size_t decode_capture(struct capture *capture,
                             mavlink_message_t *messages,
                             size_t message_capacity)
{
  mavlink_message_t rx_buffer = {0};
  mavlink_status_t rx_status = {0};
  size_t count = 0u;
  size_t offset;

  for (offset = 0u; offset < capture->length; offset++)
    {
      mavlink_message_t message;
      mavlink_status_t status;
      if (mavlink_frame_char_buffer(&rx_buffer, &rx_status,
                                    capture->bytes[offset], &message,
                                    &status) == MAVLINK_FRAMING_OK)
        {
          assert(count < message_capacity);
          messages[count++] = message;
        }
    }
  capture->length = 0u;
  return count;
}

static void deliver_message(struct m1_gcs_protocol *protocol,
                            mavlink_message_t *message)
{
  uint8_t frame[MAVLINK_MAX_PACKET_LEN];
  const uint16_t length = mavlink_msg_to_send_buffer(frame, message);

  m1_gcs_protocol_receive(protocol, frame, length);
}

int main(void)
{
  struct capture capture = {0};
  struct m1_gcs_protocol protocol;
  mavlink_status_t gcs_tx = {0};
  mavlink_message_t messages[8];
  mavlink_message_t request;
  size_t count;

  m1_gcs_protocol_init(&protocol, capture_send, &capture);
  assert(m1_gcs_protocol_send_heartbeat(&protocol) == 0);
  count = decode_capture(&capture, messages, 8u);
  assert(count == 1u);
  assert(messages[0].msgid == MAVLINK_MSG_ID_HEARTBEAT);
  {
    mavlink_heartbeat_t heartbeat;
    mavlink_msg_heartbeat_decode(&messages[0], &heartbeat);
    assert(heartbeat.type == MAV_TYPE_QUADROTOR);
    assert(heartbeat.autopilot == MAV_AUTOPILOT_ARDUPILOTMEGA);
    assert((heartbeat.base_mode & MAV_MODE_FLAG_SAFETY_ARMED) == 0u);
  }

  mavlink_msg_command_long_pack_status(
    255u, MAV_COMP_ID_MISSIONPLANNER, &gcs_tx, &request,
    M1_GCS_SYSTEM_ID, M1_GCS_COMPONENT_ID, MAV_CMD_REQUEST_MESSAGE, 0u,
    (float)MAVLINK_MSG_ID_AUTOPILOT_VERSION, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
    0.0f);
  deliver_message(&protocol, &request);
  count = decode_capture(&capture, messages, 8u);
  assert(count == 2u);
  assert(messages[0].msgid == MAVLINK_MSG_ID_AUTOPILOT_VERSION);
  assert(messages[1].msgid == MAVLINK_MSG_ID_COMMAND_ACK);
  {
    mavlink_autopilot_version_t version;
    mavlink_command_ack_t ack;
    mavlink_msg_autopilot_version_decode(&messages[0], &version);
    mavlink_msg_command_ack_decode(&messages[1], &ack);
    assert((version.flight_sw_version >> 24) == 4u);
    assert(((version.flight_sw_version >> 16) & 0xffu) == 7u);
    assert(memcmp(version.flight_custom_version, "M1GCS001", 8u) == 0);
    assert(ack.command == MAV_CMD_REQUEST_MESSAGE);
    assert(ack.result == MAV_RESULT_ACCEPTED);
  }

  mavlink_msg_param_request_list_pack_status(
    255u, MAV_COMP_ID_MISSIONPLANNER, &gcs_tx, &request, M1_GCS_SYSTEM_ID,
    M1_GCS_COMPONENT_ID);
  deliver_message(&protocol, &request);
  count = decode_capture(&capture, messages, 8u);
  assert(count == M1_GCS_PARAMETER_COUNT);
  {
    size_t index;
    for (index = 0u; index < count; index++)
      {
        mavlink_param_value_t parameter;
        mavlink_msg_param_value_decode(&messages[index], &parameter);
        assert(parameter.param_count == M1_GCS_PARAMETER_COUNT);
        assert(parameter.param_index == index);
      }
  }

  mavlink_msg_command_long_pack_status(
    255u, MAV_COMP_ID_MISSIONPLANNER, &gcs_tx, &request,
    M1_GCS_SYSTEM_ID, M1_GCS_COMPONENT_ID, MAV_CMD_COMPONENT_ARM_DISARM, 0u,
    1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
  deliver_message(&protocol, &request);
  count = decode_capture(&capture, messages, 8u);
  assert(count == 1u);
  {
    mavlink_command_ack_t ack;
    mavlink_msg_command_ack_decode(&messages[0], &ack);
    assert(ack.command == MAV_CMD_COMPONENT_ARM_DISARM);
    assert(ack.result == MAV_RESULT_DENIED);
  }
  assert(protocol.arm_reject_count == 1u);
  assert(m1_gcs_physical_write_count() == 0u);

  puts("spresense_m1_gcs_protocol=PASS heartbeat=ardupilotmega params=4 arm=DENIED physical_writes=0");
  return 0;
}

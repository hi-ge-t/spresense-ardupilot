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
  const char output_parameter_id[16] = "M1_OUT_EN";
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
#if defined(CONFIG_SPRESENSE_M1_GNSS_RUNTIME_REQUIRED) || \
    defined(CONFIG_SPRESENSE_M1_PWBIMU_REQUIRED)
    assert(heartbeat.system_status == MAV_STATE_CRITICAL);
#else
    assert(heartbeat.system_status == MAV_STATE_STANDBY);
#endif
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
#ifdef CONFIG_SPRESENSE_M1_GNSS_RUNTIME_REQUIRED
    assert(memcmp(version.flight_custom_version, "M1PGN001", 8u) == 0);
#elif defined(CONFIG_SPRESENSE_M1_PWBIMU_REQUIRED)
    assert(memcmp(version.flight_custom_version, "M1PIM001", 8u) == 0);
#else
    assert(memcmp(version.flight_custom_version, "M1GCS001", 8u) == 0);
#endif
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
#ifdef CONFIG_SPRESENSE_M1_PWBIMU_REQUIRED
        if (index == 4u)
          {
            assert(memcmp(parameter.param_id, "M1_GNSS_OK", 10u) == 0);
            assert(parameter.param_value == 0.0f);
          }
        if (index == 5u)
          {
            assert(memcmp(parameter.param_id, "M1_GNSS_ERR", 11u) == 0);
            assert(parameter.param_value == 0.0f);
          }
        if (index == 6u)
          {
            assert(memcmp(parameter.param_id, "M1_IMU_REQ", 10u) == 0);
            assert(parameter.param_value == 1.0f);
          }
        if (index == 7u)
          {
            assert(memcmp(parameter.param_id, "M1_IMU_OK", 9u) == 0);
            assert(parameter.param_value == 0.0f);
          }
#endif
      }
  }

#ifdef CONFIG_SPRESENSE_M1_PWBIMU_REQUIRED
  m1_gcs_protocol_set_gnss_result(&protocol, 0);
  assert(m1_gcs_protocol_send_heartbeat(&protocol) == 0);
  count = decode_capture(&capture, messages, 8u);
  assert(count == 1u);
  {
    mavlink_heartbeat_t heartbeat;
    mavlink_msg_heartbeat_decode(&messages[0], &heartbeat);
    assert(heartbeat.system_status == MAV_STATE_CRITICAL);
  }

  m1_gcs_protocol_set_pwbimu_ready(&protocol, 1);
  assert(m1_gcs_protocol_send_heartbeat(&protocol) == 0);
  count = decode_capture(&capture, messages, 8u);
  assert(count == 1u);
  {
    mavlink_heartbeat_t heartbeat;
    mavlink_msg_heartbeat_decode(&messages[0], &heartbeat);
    assert(heartbeat.system_status == MAV_STATE_STANDBY);
    assert((heartbeat.base_mode & MAV_MODE_FLAG_SAFETY_ARMED) == 0u);
  }

  mavlink_msg_param_request_list_pack_status(
    255u, MAV_COMP_ID_MISSIONPLANNER, &gcs_tx, &request, M1_GCS_SYSTEM_ID,
    M1_GCS_COMPONENT_ID);
  deliver_message(&protocol, &request);
  count = decode_capture(&capture, messages, 8u);
  assert(count == M1_GCS_PARAMETER_COUNT);
  {
    mavlink_param_value_t gnss_parameter;
    mavlink_param_value_t gnss_error_parameter;
    mavlink_param_value_t imu_parameter;
    mavlink_msg_param_value_decode(&messages[4], &gnss_parameter);
    mavlink_msg_param_value_decode(&messages[5], &gnss_error_parameter);
    mavlink_msg_param_value_decode(&messages[7], &imu_parameter);
    assert(memcmp(gnss_parameter.param_id, "M1_GNSS_OK", 10u) == 0);
    assert(gnss_parameter.param_value == 1.0f);
    assert(memcmp(gnss_error_parameter.param_id, "M1_GNSS_ERR", 11u) == 0);
    assert(gnss_error_parameter.param_value == 0.0f);
    assert(memcmp(imu_parameter.param_id, "M1_IMU_OK", 9u) == 0);
    assert(imu_parameter.param_value == 1.0f);
  }
#endif

  mavlink_msg_param_set_pack_status(
    255u, MAV_COMP_ID_MISSIONPLANNER, &gcs_tx, &request, M1_GCS_SYSTEM_ID,
    M1_GCS_COMPONENT_ID, output_parameter_id, 1.0f,
    MAV_PARAM_TYPE_REAL32);
  deliver_message(&protocol, &request);
  count = decode_capture(&capture, messages, 8u);
  assert(count == 1u);
  {
    mavlink_param_value_t parameter;
    mavlink_msg_param_value_decode(&messages[0], &parameter);
    assert(memcmp(parameter.param_id, "M1_OUT_EN", 9u) == 0);
    assert(parameter.param_value == 0.0f);
  }
  assert(protocol.parameter_write_reject_count == 1u);

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

  printf("spresense_m1_gcs_protocol=PASS heartbeat=ardupilotmega "
         "params=%u arm=DENIED physical_writes=0\n",
         (unsigned int)M1_GCS_PARAMETER_COUNT);
  return 0;
}

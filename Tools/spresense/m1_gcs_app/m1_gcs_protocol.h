/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * Output-disabled MAVLink protocol surface for the Spresense M1 bring-up.
 */

#pragma once

#include <stddef.h>
#include <stdint.h>

#include <ardupilotmega/mavlink.h>

#ifdef __cplusplus
extern "C" {
#endif

#define M1_GCS_SYSTEM_ID 1u
#define M1_GCS_COMPONENT_ID MAV_COMP_ID_AUTOPILOT1
#define M1_GCS_PARAMETER_COUNT 4u

typedef int (*m1_gcs_send_fn)(void *context, const uint8_t *bytes,
                              size_t length);

struct m1_gcs_protocol
{
  mavlink_message_t rx_buffer;
  mavlink_status_t rx_status;
  mavlink_status_t tx_status;
  m1_gcs_send_fn send;
  void *send_context;
  uint32_t received_messages;
  uint32_t transmitted_messages;
  uint32_t arm_reject_count;
  uint32_t parameter_write_reject_count;
};

void m1_gcs_protocol_init(struct m1_gcs_protocol *protocol,
                          m1_gcs_send_fn send, void *send_context);
void m1_gcs_protocol_receive(struct m1_gcs_protocol *protocol,
                             const uint8_t *bytes, size_t length);
int m1_gcs_protocol_send_heartbeat(struct m1_gcs_protocol *protocol);
int m1_gcs_protocol_send_boot_status(struct m1_gcs_protocol *protocol);
uint32_t m1_gcs_physical_write_count(void);

#ifdef __cplusplus
}
#endif

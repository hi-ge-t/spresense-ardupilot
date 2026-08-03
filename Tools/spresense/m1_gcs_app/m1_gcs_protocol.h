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
#ifdef CONFIG_SPRESENSE_M1_GNSS_RUNTIME_REQUIRED
#define M1_GCS_PARAMETER_COUNT 8u
#elif defined(CONFIG_SPRESENSE_M1_PWBIMU_REQUIRED)
#define M1_GCS_PARAMETER_COUNT 6u
#else
#define M1_GCS_PARAMETER_COUNT 4u
#endif

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
  float parameter_values[M1_GCS_PARAMETER_COUNT];
#ifdef CONFIG_SPRESENSE_M1_GNSS_RUNTIME_REQUIRED
  uint8_t gnss_ready;
#endif
#ifdef CONFIG_SPRESENSE_M1_PWBIMU_REQUIRED
  uint8_t pwbimu_ready;
#endif
};

void m1_gcs_protocol_init(struct m1_gcs_protocol *protocol,
                          m1_gcs_send_fn send, void *send_context);
void m1_gcs_protocol_receive(struct m1_gcs_protocol *protocol,
                             const uint8_t *bytes, size_t length);
int m1_gcs_protocol_send_heartbeat(struct m1_gcs_protocol *protocol);
int m1_gcs_protocol_send_boot_status(struct m1_gcs_protocol *protocol);
void m1_gcs_protocol_set_gnss_result(struct m1_gcs_protocol *protocol,
                                    int result);
void m1_gcs_protocol_set_pwbimu_ready(struct m1_gcs_protocol *protocol,
                                     int ready);
uint32_t m1_gcs_physical_write_count(void);

#ifdef __cplusplus
}
#endif

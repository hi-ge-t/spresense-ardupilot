/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * Linker-map markers for the Application SRAM and GNSS RAM feasibility gate.
 * The runtime touch keeps every section live; it is not a latency benchmark.
 */

#include <nuttx/config.h>
#include <nuttx/compiler.h>

#include "m1_gcs_memory_layout.h"

#include <stdbool.h>
#include <arch/chip/gnssram.h>

#include <stddef.h>
#include <stdint.h>

#define M1_GCS_LAYOUT_WORDS 16u

static const uint32_t g_m1_gcs_app_rodata[M1_GCS_LAYOUT_WORDS] =
{
  0x41505000u, 0x41505001u, 0x41505002u, 0x41505003u,
  0x41505004u, 0x41505005u, 0x41505006u, 0x41505007u,
  0x41505008u, 0x41505009u, 0x4150500au, 0x4150500bu,
  0x4150500cu, 0x4150500du, 0x4150500eu, 0x4150500fu,
};

static uint32_t g_m1_gcs_app_data = 0x4d314150u;
static uint32_t g_m1_gcs_app_bss;

extern const uint32_t g_m1_gcs_gnss_rodata[M1_GCS_LAYOUT_WORDS];

uint32_t g_m1_gcs_gnss_data[M1_GCS_LAYOUT_WORDS] GNSSRAM_DATA =
{
  0x474e4441u, 0x54410001u,
};

uint32_t g_m1_gcs_gnss_bss[M1_GCS_LAYOUT_WORDS] GNSSRAM_BSS;

static uint32_t m1_gcs_gnss_checksum(const uint32_t *words, size_t count)
  GNSSRAM_CODE __attribute__((noinline));

static uint32_t m1_gcs_gnss_checksum(const uint32_t *words, size_t count)
{
  uint32_t checksum = 2166136261u;
  size_t index;

  for (index = 0u; index < count; index++)
    {
      checksum ^= words[index];
      checksum *= 16777619u;
    }
  return checksum;
}

uint32_t m1_gcs_memory_layout_touch(void)
{
  g_m1_gcs_app_bss = g_m1_gcs_app_data ^ g_m1_gcs_gnss_data[0];
  g_m1_gcs_gnss_bss[0] = g_m1_gcs_app_rodata[0];
  return m1_gcs_gnss_checksum(g_m1_gcs_gnss_rodata,
                              M1_GCS_LAYOUT_WORDS) ^
         g_m1_gcs_app_bss ^ g_m1_gcs_gnss_bss[0];
}

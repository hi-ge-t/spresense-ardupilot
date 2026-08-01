/* SPDX-License-Identifier: GPL-3.0-or-later */

#include <nuttx/config.h>
#include <nuttx/compiler.h>

#include <stdint.h>

#define M1_GCS_LAYOUT_WORDS 16u

const uint32_t g_m1_gcs_gnss_rodata[M1_GCS_LAYOUT_WORDS]
  locate_data(".gnssram.data") =
{
  0x474e5353u, 0x524f0001u, 0x524f0002u, 0x524f0003u,
  0x524f0004u, 0x524f0005u, 0x524f0006u, 0x524f0007u,
  0x524f0008u, 0x524f0009u, 0x524f000au, 0x524f000bu,
  0x524f000cu, 0x524f000du, 0x524f000eu, 0x524f000fu,
};

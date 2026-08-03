/* SPDX-License-Identifier: GPL-3.0-or-later */

#include <nuttx/config.h>

#include "m1_gcs_runtime.h"

#include <sched.h>
#include <unistd.h>

static int m1_gcs_boot_task(int argc, char *argv[])
{
  usleep(1000u * 1000u);
  return m1_gcs_runtime_main(argc, argv);
}

int spresense_main(int argc, char *argv[])
{
  const pid_t task = task_create(
    "m1-gcs", CONFIG_SPRESENSE_M1_GCS_PRIORITY,
    CONFIG_SPRESENSE_M1_GCS_STACKSIZE, m1_gcs_boot_task, NULL);

  (void)argc;
  (void)argv;
  if (task < 0)
    {
      for (;;)
        {
          sleep(1);
        }
    }

  return 0;
}

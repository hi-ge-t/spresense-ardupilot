/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * Bounded, sensor-only smoke probe for Sony's CXD5602PWBIMU character
 * driver. This module has no actuator or physical-output path.
 */

#include <nuttx/config.h>

#include "m1_pwbimu_probe.h"

#include <nuttx/sensors/cxd5602pwbimu.h>

#include <errno.h>
#include <fcntl.h>
#include <poll.h>
#include <stdint.h>
#include <sys/ioctl.h>
#include <unistd.h>

#define M1_PWBIMU_SAMPLE_RATE_HZ 60
#define M1_PWBIMU_ACCEL_RANGE_G 2
#define M1_PWBIMU_GYRO_RANGE_DPS 125
#define M1_PWBIMU_FIFO_THRESHOLD 1
#define M1_PWBIMU_TIMEOUT_MS 1000

static int m1_pwbimu_ioctl(int fd, int request, unsigned long argument)
{
  if (ioctl(fd, request, argument) != 0)
    {
      return errno == 0 ? -EIO : -errno;
    }

  return 0;
}

int m1_pwbimu_probe_once(void)
{
  cxd5602pwbimu_range_t range;
  cxd5602pwbimu_data_t sample;
  struct pollfd descriptor;
  int enabled = 0;
  int result = 0;
  int fd;

  fd = open(CONFIG_SPRESENSE_M1_PWBIMU_DEVICE, O_RDONLY);
  if (fd < 0)
    {
      return errno == 0 ? -ENODEV : -errno;
    }

  range.accel = M1_PWBIMU_ACCEL_RANGE_G;
  range.gyro = M1_PWBIMU_GYRO_RANGE_DPS;
  result = m1_pwbimu_ioctl(fd, SNIOC_SSAMPRATE,
                           M1_PWBIMU_SAMPLE_RATE_HZ);
  if (result == 0)
    {
      result = m1_pwbimu_ioctl(fd, SNIOC_SDRANGE,
                               (unsigned long)(uintptr_t)&range);
    }
  if (result == 0)
    {
      result = m1_pwbimu_ioctl(fd, SNIOC_SFIFOTHRESH,
                               M1_PWBIMU_FIFO_THRESHOLD);
    }
  if (result == 0)
    {
      result = m1_pwbimu_ioctl(fd, SNIOC_ENABLE, 1u);
      enabled = result == 0;
    }

  descriptor.fd = fd;
  descriptor.events = POLLIN;
  descriptor.revents = 0;
  if (result == 0)
    {
      int poll_result;

      do
        {
          poll_result = poll(&descriptor, 1, M1_PWBIMU_TIMEOUT_MS);
        }
      while (poll_result < 0 && errno == EINTR);

      if (poll_result == 0)
        {
          result = -ETIMEDOUT;
        }
      else if (poll_result < 0)
        {
          result = errno == 0 ? -EIO : -errno;
        }
      else if ((descriptor.revents & POLLIN) == 0)
        {
          result = -EIO;
        }
    }
  if (result == 0)
    {
      ssize_t length;

      do
        {
          length = read(fd, &sample, sizeof(sample));
        }
      while (length < 0 && errno == EINTR);
      if (length != (ssize_t)sizeof(sample))
        {
          result = length < 0 && errno != 0 ? -errno : -EIO;
        }
    }

  if (enabled && m1_pwbimu_ioctl(fd, SNIOC_ENABLE, 0u) != 0 && result == 0)
    {
      result = -EIO;
    }
  if (close(fd) != 0 && result == 0)
    {
      result = errno == 0 ? -EIO : -errno;
    }

  return result;
}

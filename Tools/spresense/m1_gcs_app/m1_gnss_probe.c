/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * Bounded, sensor-only smoke probe for Sony's CXD5610 GNSS Add-on
 * character driver. This module has no actuator or physical-output path.
 */

#include <nuttx/config.h>

#include "m1_gnss_probe.h"

#include <arch/chip/gnss.h>

#include <errno.h>
#include <fcntl.h>
#include <poll.h>
#include <stdint.h>
#include <string.h>
#include <sys/ioctl.h>
#include <unistd.h>

#define M1_GNSS_TIMEOUT_MS 15000

static struct cxd56_gnss_positiondata2_s g_m1_gnss_sample;

static int m1_gnss_ioctl(int fd, int request, unsigned long argument)
{
  if (ioctl(fd, request, argument) < 0)
    {
      return errno == 0 ? -EIO : -errno;
    }

  return 0;
}

int m1_gnss_probe_once(void)
{
  struct pollfd descriptor;
  char version[CXD56_GNSS_VERSION_MAXLEN];
  int started = 0;
  int result = 0;
  int fd;

  fd = open(CONFIG_SPRESENSE_M1_GNSS_DEVICE, O_RDONLY);
  if (fd < 0)
    {
      return errno == 0 ? -ENODEV : -errno;
    }

  result = m1_gnss_ioctl(fd, CXD56_GNSS_IOCTL_WAKEUP, 0u);
  memset(version, 0, sizeof(version));
  if (result == 0)
    {
      result = m1_gnss_ioctl(
        fd, CXD56_GNSS_IOCTL_GET_VERSION,
        (unsigned long)(uintptr_t)version);
    }
  if (result == 0 && version[0] == '\0')
    {
      result = -ENODEV;
    }
  if (result == 0)
    {
      result = m1_gnss_ioctl(fd, CXD56_GNSS_IOCTL_START,
                             CXD56_GNSS_STMOD_HOT);
      started = result == 0;
    }

  descriptor.fd = fd;
  descriptor.events = POLLIN;
  descriptor.revents = 0;
  if (result == 0)
    {
      int poll_result;

      do
        {
          poll_result = poll(&descriptor, 1, M1_GNSS_TIMEOUT_MS);
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
          length = read(fd, &g_m1_gnss_sample, sizeof(g_m1_gnss_sample));
        }
      while (length < 0 && errno == EINTR);
      if (length != (ssize_t)sizeof(g_m1_gnss_sample))
        {
          result = length < 0 && errno != 0 ? -errno : -EIO;
        }
    }

  if (started &&
      m1_gnss_ioctl(fd, CXD56_GNSS_IOCTL_STOP, 0u) != 0 && result == 0)
    {
      result = -EIO;
    }
  if (close(fd) != 0 && result == 0)
    {
      result = errno == 0 ? -EIO : -errno;
    }

  return result;
}

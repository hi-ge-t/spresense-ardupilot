/* SPDX-License-Identifier: GPL-3.0-or-later */

#include <nuttx/config.h>

#include "m1_gcs_protocol.h"
#include "m1_gcs_memory_layout.h"
#include "m1_gcs_runtime.h"
#ifdef CONFIG_SPRESENSE_M1_PWBIMU_REQUIRED
#include "m1_pwbimu_probe.h"
#endif

#include <errno.h>
#include <fcntl.h>
#include <stdint.h>
#include <string.h>
#include <termios.h>
#include <time.h>
#include <unistd.h>

#define M1_GCS_TX_CAPACITY 2048u
#define M1_GCS_READ_SIZE 128u
#define M1_GCS_READ_DRAIN_LIMIT 32u
#define M1_GCS_HEARTBEAT_INTERVAL_MS 1000u
#define M1_GCS_REOPEN_INTERVAL_MS 250u

struct m1_gcs_transport
{
  int fd;
  uint8_t tx[M1_GCS_TX_CAPACITY];
  size_t tx_head;
  size_t tx_tail;
  size_t tx_count;
};

static uint64_t m1_gcs_monotonic_ms(void)
{
  struct timespec now;

  if (clock_gettime(CLOCK_MONOTONIC, &now) != 0)
    {
      return 0u;
    }

  return (uint64_t)now.tv_sec * 1000u + (uint64_t)now.tv_nsec / 1000000u;
}

static int m1_gcs_configure_serial(int fd)
{
  struct termios config;

  if (tcgetattr(fd, &config) != 0)
    {
      return -errno;
    }

  config.c_iflag = 0;
  config.c_oflag = 0;
  config.c_lflag = 0;
  config.c_cflag = CS8 | CREAD | CLOCAL;
  config.c_cc[VMIN] = 0;
  config.c_cc[VTIME] = 0;
  if (cfsetispeed(&config, B115200) != 0 ||
      cfsetospeed(&config, B115200) != 0 ||
      tcsetattr(fd, TCSANOW, &config) != 0)
    {
      return -errno;
    }

  return 0;
}

static int m1_gcs_open_serial(void)
{
  int flags = O_RDWR | O_NONBLOCK;
  int fd;

#ifdef O_NOCTTY
  flags |= O_NOCTTY;
#endif
  fd = open(CONFIG_SPRESENSE_M1_GCS_DEVICE, flags);
  if (fd < 0)
    {
      return -1;
    }
  if (m1_gcs_configure_serial(fd) != 0)
    {
      (void)close(fd);
      return -1;
    }

  return fd;
}

static int m1_gcs_enqueue(void *context, const uint8_t *bytes, size_t length)
{
  struct m1_gcs_transport *transport = context;
  size_t offset;

  if (length > M1_GCS_TX_CAPACITY - transport->tx_count)
    {
      return -ENOSPC;
    }

  for (offset = 0u; offset < length; offset++)
    {
      transport->tx[transport->tx_head] = bytes[offset];
      transport->tx_head = (transport->tx_head + 1u) % M1_GCS_TX_CAPACITY;
    }
  transport->tx_count += length;
  return 0;
}

static int m1_gcs_flush_tx(struct m1_gcs_transport *transport)
{
  while (transport->tx_count > 0u)
    {
      size_t contiguous = M1_GCS_TX_CAPACITY - transport->tx_tail;
      ssize_t written;

      if (contiguous > transport->tx_count)
        {
          contiguous = transport->tx_count;
        }
      written = write(transport->fd, &transport->tx[transport->tx_tail],
                      contiguous);
      if (written > 0)
        {
          transport->tx_tail =
            (transport->tx_tail + (size_t)written) % M1_GCS_TX_CAPACITY;
          transport->tx_count -= (size_t)written;
          continue;
        }
      if (written < 0 && errno == EINTR)
        {
          continue;
        }
      if (written < 0 && (errno == EAGAIN || errno == EWOULDBLOCK))
        {
          return 0;
        }
      return -1;
    }

  return 0;
}

static int m1_gcs_drain_rx(struct m1_gcs_transport *transport,
                           struct m1_gcs_protocol *protocol)
{
  uint8_t buffer[M1_GCS_READ_SIZE];
  unsigned int pass;

  for (pass = 0u; pass < M1_GCS_READ_DRAIN_LIMIT; pass++)
    {
      const ssize_t received = read(transport->fd, buffer, sizeof(buffer));

      if (received > 0)
        {
          m1_gcs_protocol_receive(protocol, buffer, (size_t)received);
          continue;
        }
      if (received == 0)
        {
          return 0;
        }
      if (errno == EINTR)
        {
          continue;
        }
      if (errno == EAGAIN || errno == EWOULDBLOCK)
        {
          return 0;
        }
      return -1;
    }

  return 0;
}

int m1_gcs_runtime_main(int argc, char *argv[])
{
  struct m1_gcs_transport transport;
  struct m1_gcs_protocol protocol;
  uint64_t next_heartbeat_ms = 0u;
  uint64_t next_open_ms = 0u;
  volatile uint32_t memory_layout_evidence;

  (void)argc;
  (void)argv;
  memset(&transport, 0, sizeof(transport));
  transport.fd = -1;
  memory_layout_evidence = m1_gcs_memory_layout_touch();
  (void)memory_layout_evidence;
  m1_gcs_protocol_init(&protocol, m1_gcs_enqueue, &transport);
#ifdef CONFIG_SPRESENSE_M1_PWBIMU_REQUIRED
  m1_gcs_protocol_set_pwbimu_ready(&protocol,
                                   m1_pwbimu_probe_once() == 0);
#endif

  for (;;)
    {
      const uint64_t now_ms = m1_gcs_monotonic_ms();

      if (transport.fd < 0 && now_ms >= next_open_ms)
        {
          transport.fd = m1_gcs_open_serial();
          next_open_ms = now_ms + M1_GCS_REOPEN_INTERVAL_MS;
          if (transport.fd >= 0)
            {
              transport.tx_head = 0u;
              transport.tx_tail = 0u;
              transport.tx_count = 0u;
              next_heartbeat_ms = now_ms;
              (void)m1_gcs_protocol_send_boot_status(&protocol);
            }
        }

      if (transport.fd >= 0)
        {
          int serial_ok = 1;

          if (m1_gcs_drain_rx(&transport, &protocol) != 0)
            {
              serial_ok = 0;
            }
          if (now_ms >= next_heartbeat_ms)
            {
              (void)m1_gcs_protocol_send_heartbeat(&protocol);
              next_heartbeat_ms = now_ms + M1_GCS_HEARTBEAT_INTERVAL_MS;
            }
          if (serial_ok && m1_gcs_flush_tx(&transport) != 0)
            {
              serial_ok = 0;
            }
          if (!serial_ok)
            {
              (void)close(transport.fd);
              transport.fd = -1;
              next_open_ms = now_ms + M1_GCS_REOPEN_INTERVAL_MS;
            }
        }

      usleep(10000u);
    }

  return 1;
}

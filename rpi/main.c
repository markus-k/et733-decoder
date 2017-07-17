/*
 * This program is a mqtt feeder for my bbq thermometer
 *
 * Copyright (C) 2017  Markus Kasten
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <errno.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/timerfd.h>
#include <linux/types.h>
#include <linux/spi/spidev.h>

#include <mosquitto.h>

#define BBQ_READOUT_FLAG_STARTING       0
struct bbq_readout {
  uint16_t magic;
  int16_t temp1;
  int16_t temp2;
  uint8_t flags;
};

struct bbq_conf {
  char *mqtt_host;
  uint16_t mqtt_port;
};

volatile sig_atomic_t sigint_received = 0;

int spi_open() {
  int fd;
  int ret;
  uint32_t tmp;
  const char *device = "/dev/spidev0.0";

  fd = open(device, O_RDWR);
  if (fd < 0) {
    perror("Can't open SPI device");
    abort();
  }

  tmp = 50000; // 50kHz
  ret = ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &tmp);
  if (ret == -1) {
    perror("Setting SPI speed failed");
  }

  tmp = 8;
  ret = ioctl(fd, SPI_IOC_WR_BITS_PER_WORD, &tmp);
  if (ret == -1) {
    perror("Setting SPI word bits failed");
  }

  return fd;
}

void spi_close(int fd) {
  close(fd);
}

void spi_receive(int fd, size_t len, void *rx_buf) {
  int ret;

  ret = read(fd, rx_buf, len);
  if (ret < 0) {
    fprintf(stderr, "Reading from SPI device failed: %d", ret);
  }
}

void set_timer_interval(int fd, int interval) {
  struct itimerspec itval;

  itval.it_interval.tv_sec = interval;
  itval.it_interval.tv_nsec = interval * 1e6;
  itval.it_value.tv_sec = interval;
  itval.it_value.tv_nsec = interval * 1e6;

  timerfd_settime(fd, 0, &itval, NULL);
}

void setup_timer(int *fd) {
  *fd = timerfd_create(CLOCK_REALTIME, 0);

  set_timer_interval(*fd, 5);
}

void destroy_timer(int fd) {
  close(fd);
}

int timer_wait(int fd) {
  int ret;
  uint64_t missed;

  ret = read(fd, &missed, sizeof(missed));
  if (ret == -1) {
    if (errno == EINTR) {
      return 1;
    } else {
      perror("read() on timer_fd failed");
    }
  }
  return 0;
}

void handle_sigint() {
  sigint_received = 1;
  fprintf(stderr, "Interrupted\n");
}

void connect_callback(struct mosquitto *mosq, void *userdata, int result) {
  if (!result) {
    printf("Connected.\n");
  } else {
    fprintf(stderr, "Connection failed.\n");
  }
}

void disconnect_callback(struct mosquitto *mosq, void *userdata, int result) {
  if (!result) {
    printf("Disconnected.\n");
  } else {
    fprintf(stderr, "Unexpected disconnect.\n");
  }
}

void log_callback(struct mosquitto *mosq, void *userdata, int level, const char *str) {
  printf("mosquitto: %s\n", str);
}

void read_readout(int spi_fd, struct bbq_readout *readout) {
  uint8_t rx_buf[sizeof(struct bbq_readout)];

  spi_receive(spi_fd, sizeof(struct bbq_readout), rx_buf);

  readout->magic = rx_buf[0] | (uint16_t)rx_buf[1] << 8;
  readout->temp1 = (rx_buf[2] | (uint16_t)rx_buf[3] << 8) - 532;
  readout->temp2 = (rx_buf[4] | (uint16_t)rx_buf[5] << 8) - 532;
  readout->flags = rx_buf[6];
  printf("Magic: %x\n", readout->magic);
}

void publish_readout(struct mosquitto *mosq, struct bbq_readout *readout) {
  char buf[8];
  int bytes;

  bytes = sprintf(buf, "%d", readout->temp1);
  mosquitto_publish(mosq, NULL, "temp1", bytes, buf, 0, 1);
  bytes = sprintf(buf, "%d", readout->temp2);
  mosquitto_publish(mosq, NULL, "temp2", bytes, buf, 0, 1);
}

void do_readout(int spi_fd, struct mosquitto *mosq) {
  struct bbq_readout readout;

  read_readout(spi_fd, &readout);

  publish_readout(mosq, &readout);
}

int main(int argc, char **argv) {
  struct mosquitto *mosq = NULL;
  int spi_fd;
  int timer_fd;

  struct sigaction int_handler = { .sa_handler = handle_sigint };
  sigaction(SIGINT, &int_handler, 0);

  mosquitto_lib_init();
  mosq = mosquitto_new(NULL, true, NULL);
  if (!mosq) {
    fprintf(stderr, "mosquitto_new() failed.\n");
    return 1;
  }

  mosquitto_log_callback_set(mosq, log_callback);
  mosquitto_connect_callback_set(mosq, connect_callback);
  mosquitto_disconnect_callback_set(mosq, disconnect_callback);

  spi_fd = spi_open();

  setup_timer(&timer_fd);

  mosquitto_connect(mosq, "localhost", 1883, 10);

  mosquitto_loop_start(mosq);

  do {
    do_readout(spi_fd, mosq);
  } while (!timer_wait(timer_fd));

  mosquitto_disconnect(mosq);

  mosquitto_loop_stop(mosq, false);

  destroy_timer(timer_fd);

  spi_close(spi_fd);

  mosquitto_destroy(mosq);
  mosquitto_lib_cleanup();

  return 0;
}

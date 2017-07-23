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
#include <string.h>
#include <signal.h>
#include <unistd.h>
#include <getopt.h>
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
  uint16_t sender_id;
  uint8_t flags;
};

struct bbq_conf {
  char *mqtt_host;
  uint16_t mqtt_port;
  char *mqtt_topic;
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

  set_timer_interval(*fd, 1);
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
  readout->sender_id = rx_buf[6] | (uint16_t)rx_buf[7] << 8;
  readout->flags = rx_buf[8];
  printf("Magic: %x\n", readout->magic);
}

char *make_topic(struct bbq_conf *conf, const char *name) {
  char *res = malloc(strlen(conf->mqtt_topic) + strlen(name) + 1);

  strcpy(res, conf->mqtt_topic);
  strcat(res, name);

  return res;
}

void publish_readout(struct bbq_conf *conf, struct mosquitto *mosq, struct bbq_readout *readout) {
  char buf[8];
  char *topic;
  int bytes;

  topic = make_topic(conf, "temp1");
  bytes = sprintf(buf, "%d", readout->temp1);
  mosquitto_publish(mosq, NULL, topic, bytes, buf, 0, 1);
  free(topic);

  topic = make_topic(conf, "temp2");
  bytes = sprintf(buf, "%d", readout->temp2);
  mosquitto_publish(mosq, NULL, topic, bytes, buf, 0, 1);
  free(topic);
}

void do_readout(struct bbq_conf *conf, int spi_fd, struct mosquitto *mosq) {
  struct bbq_readout readout;

  read_readout(spi_fd, &readout);

  publish_readout(conf, mosq, &readout);
}

void print_help(int argc, char **argv) {
  printf("Usage: %s [OPTIONS]\n"
	 "BBQ thermometer to mqtt bridge\n"
	 "\n"
	 "  -h, --help                  print this help\n"
	 "  -H, --mqtt-host HOST        MQTT host\n"
	 "  -p, --mqtt-port PORT        MQTT port\n"
	 "  -t, --mqtt-topic PATH       MQTT topic prefix\n"
	 , argv[0]);
}

void parse_opts(int argc, char **argv, struct bbq_conf *conf) {
  int opt, opt_i;
  static struct option long_opts[] = {
    {"help",       no_argument,       0, 'h'},
    {"mqtt-host",  required_argument, 0, 'H'},
    {"mqtt-port",  required_argument, 0, 'p'},
    {"mqtt-topic", required_argument, 0, 't'},
    {0,            0,                 0,  0 }
  };
  const char *short_opts = "hH:p:t:";

  while ((opt = getopt_long(argc, argv, short_opts, long_opts, &opt_i)) != -1) {
    switch (opt) {
    case 'h':
      print_help(argc, argv);
      exit(0);
      break;
    case 'H':
      conf->mqtt_host = realloc(conf->mqtt_host, strlen(optarg) + 1);
      strcpy(conf->mqtt_host, optarg);
      break;
    case 'p':
      conf->mqtt_port = atoi(optarg);
      break;
    case 't':
      conf->mqtt_topic = realloc(conf->mqtt_topic, strlen(optarg) + 1);
      strcpy(conf->mqtt_topic, optarg);
      break;
    default:
      print_help(argc, argv);
      exit(1);
    }
  }
}

int main(int argc, char **argv) {
  struct bbq_conf conf;
  struct mosquitto *mosq = NULL;
  int spi_fd;
  int timer_fd;

  conf.mqtt_host = strdup("localhost");
  conf.mqtt_port = 1883;
  conf.mqtt_topic = strdup("");

  struct sigaction int_handler = { .sa_handler = handle_sigint };
  sigaction(SIGINT, &int_handler, 0);

  parse_opts(argc, argv, &conf);

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

  mosquitto_connect(mosq, conf.mqtt_host, conf.mqtt_port, 10);

  mosquitto_loop_start(mosq);

  do {
    do_readout(&conf, spi_fd, mosq);
  } while (!timer_wait(timer_fd));

  mosquitto_disconnect(mosq);

  mosquitto_loop_stop(mosq, false);

  destroy_timer(timer_fd);

  spi_close(spi_fd);

  mosquitto_destroy(mosq);
  mosquitto_lib_cleanup();

  free(conf.mqtt_host);
  free(conf.mqtt_topic);

  return 0;
}

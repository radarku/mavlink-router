/*
 * This file is part of the MAVLink Router project
 *
 * Copyright (C) 2020 Auterion AG. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#pragma once

#include <limits>
#include <list>
#include <string>
#include <vector>

#include <common/log.h>
#include <common/conf_file.h>
#include "logendpoint.h"

#define DEFAULT_BAUDRATE 115200U
#define DEFAULT_RETRY_TCP_TIMEOUT 5

enum endpoint_type { Tcp, Uart, Udp, Unknown };
enum mavlink_dialect { Auto, Common, Ardupilotmega };

struct endpoint_config {
    std::string name;
    enum endpoint_type type;
    // The following are only meaningful for network endpoints.
    std::string address;
    unsigned long port = 0;
    int retry_timeout = 0;
    bool eavesdropping = false;
    // The following are only meaningful for serial endpoints.
    std::string device;
    std::vector<unsigned long> bauds;
    // The following apply to all endpoints.
    bool flowcontrol = false;
    std::string filter;
};

struct options {
    std::list<endpoint_config> endpoints;
    std::string conf_file_name;
    std::string conf_dir;
    unsigned long tcp_port = std::numeric_limits<unsigned long>::max();
    bool report_msg_statistics = false;
    std::string logs_dir;
    LogMode log_mode = LogMode::always;
    int debug_log_level = (int)Log::Level::INFO;
    enum mavlink_dialect mavlink_dialect = Auto;
    unsigned long min_free_space = 0;
    unsigned long max_log_files = 0;
};

/* Adds tdp endpoint to configuration structure. */
int add_tcp_endpoint_address(const char *name, size_t name_len, const char *ip,
                            long unsigned port, int timeout, options* opt);

/* Adds udp endpoint to configuration structure. */
int add_endpoint_address(const char *name, size_t name_len, const char *ip,
                         long unsigned port, bool eavesdropping, const char *filter,
                         options* opt);

/* Adds uart endpoint to configuration structure. */
int add_uart_endpoint(const char *name, size_t name_len, const char *uart_device,
                      const char *bauds, bool flowcontrol, options* opt);

/* Transfers contents of parsed config file to the options structure. */
int parse_confs(ConfFile &conf, options* opt);

int log_level_from_str(const char *str);

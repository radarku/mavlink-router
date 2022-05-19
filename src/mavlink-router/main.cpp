/*
 * This file is part of the MAVLink Router project
 *
 * Copyright (C) 2016  Intel Corporation. All rights reserved.
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

#include <assert.h>
#include <dirent.h>
#include <getopt.h>
#include <limits.h>
#include <stddef.h>
#include <stdio.h>
#include <sys/stat.h>
#include <string>
#include <regex>

#include <common/conf_file.h>
#include <common/dbg.h>
#include <common/log.h>
#include <common/util.h>
#include <src/common/log.h>
#include <src/common/util.h>

#include "comm.h"
#include "endpoint.h"
#include "mainloop.h"

#define MAVLINK_TCP_PORT 5760
#define DEFAULT_BAUDRATE 115200U
#define DEFAULT_CONFFILE "/etc/mavlink-router/main.conf"
#define DEFAULT_CONF_DIR "/etc/mavlink-router/config.d"
#define DEFAULT_RETRY_TCP_TIMEOUT 5

static struct options opt = {
    .endpoints = nullptr,
    .conf_file_name = nullptr,
    .conf_dir = nullptr,
    .tcp_port = ULONG_MAX,
    .report_msg_statistics = false,
    .logs_dir = nullptr,
    .log_mode = LogMode::always,
    .debug_log_level = (int)Log::Level::INFO,
    .mavlink_dialect = Auto,
    .min_free_space = 0,
    .max_log_files = 0,
    .heartbeat = false
};

static const struct option long_options[] = {
    { "endpoints",              required_argument,  NULL,   'e' },
    { "conf-file",              required_argument,  NULL,   'c' },
    { "conf-dir" ,              required_argument,  NULL,   'd' },
    { "report_msg_statistics",  no_argument,        NULL,   'r' },
    { "tcp-port",               required_argument,  NULL,   't' },
    { "tcp-endpoint",           required_argument,  NULL,   'p' },
    { "log",                    required_argument,  NULL,   'l' },
    { "debug-log-level",        required_argument,  NULL,   'g' },
    { "heartbeat"      ,        no_argument,        NULL,   'b' },
    { "verbose",                no_argument,        NULL,   'v' },
    { "version",                no_argument,        NULL,   'V' },
    { }
};

static const char* short_options = "he:rt:c:d:l:p:g:bvV";

static void help(FILE *fp) {
    fprintf(fp,
            "%s [OPTIONS...] [<uart>|<udp_address>]\n\n"
            "  <uart>                       UART device (<device>[:<baudrate>]) that will be routed\n"
            "  <udp_address>                UDP address (<ip>:<port>) that will be routed\n"
            "  -e --endpoint <ip[:port]>    Add UDP endpoint to communicate port is optional\n"
            "                               and in case it's not given it starts in 14550 and\n"
            "                               continues increasing not to collide with previous\n"
            "                               ports\n"
            "  -p --tcp-endpoint <ip:port>  Add TCP endpoint client, which will connect to given\n"
            "                               address\n"
            "  -r --report_msg_statistics   Report message statistics\n"
            "  -t --tcp-port <port>         Port in which mavlink-router will listen for TCP\n"
            "                               connections. Pass 0 to disable TCP listening.\n"
            "                               Default port 5760\n"
            "  -c --conf-file <file>        .conf file with configurations for mavlink-router.\n"
            "  -d --conf-dir <dir>          Directory where to look for .conf files overriding\n"
            "                               default conf file.\n"
            "  -l --log <directory>         Enable Flight Stack logging\n"
            "  -g --debug-log-level <level> Set debug log level. Levels are\n"
            "                               <error|warning|info|debug>\n"
            "  -b --heartbeat               Broadcast log status as heartbeat when logging is enabled\n"
            "  -v --verbose                 Verbose. Same as --debug-log-level=debug\n"
            "  -V --version                 Show version\n"
            "  -h --help                    Print this message\n"
            , program_invocation_short_name);
}

static unsigned long find_next_endpoint_port(const char *ip)
{
    unsigned long port = 14550U;

    while (true) {
        struct endpoint_config *conf;

        for (conf = opt.endpoints; conf; conf = conf->next) {
            if (conf->type == Udp && streq(conf->address, ip) && conf->port == port) {
                port++;
                break;
            }
        }
        if (!conf)
            break;
    }

    return port;
}

static int split_on_last_colon(const char *str, char **base, unsigned long *number)
{
    char *colonstr;

    *base = strdup(str);
    colonstr = strrchr(*base, ':');
    *number = ULONG_MAX;

    if (colonstr != nullptr) {
        *colonstr = '\0';
        if (safe_atoul(colonstr + 1, number) < 0) {
            free(*base);
            return -EINVAL;
        }
    }

    return 0;
}

static int validate_ip(const char* ip)
{
    std::string ip_addr(ip);

    std::regex ipv4_regex("(\\d{1,3}\\.\\d{1,3}\\.\\d{1,3}\\.\\d{1,3})");
#ifdef ENABLE_IPV6
    std::regex ipv6_regex("\\[(([a-f\\d]{0,4}:)+[a-f\\d]{0,4})\\]"); // simplyfied pattern

    if (!std::regex_match(ip_addr, ipv4_regex) && !std::regex_match(ip_addr, ipv6_regex)) {
#else
    if (!std::regex_match(ip_addr, ipv4_regex)) {
#endif
        return -EINVAL;
    }

    return 0;
}

static int log_level_from_str(const char *str)
{
    if (strcaseeq(str, "error"))
        return (int)Log::Level::ERROR;
    if (strcaseeq(str, "warning"))
        return (int)Log::Level::WARNING;
    if (strcaseeq(str, "info"))
        return (int)Log::Level::INFO;
    if (strcaseeq(str, "debug"))
        return (int)Log::Level::DEBUG;

    return -EINVAL;
}

static int add_tcp_endpoint_address(const char *name, size_t name_len, const char *ip,
                                    long unsigned port, int timeout)
{
    int ret;

    struct endpoint_config *conf
        = (struct endpoint_config *)calloc(1, sizeof(struct endpoint_config));
    assert_or_return(conf, -ENOMEM);
    conf->type = Tcp;
    conf->port = ULONG_MAX;
    conf->dropout_percentage = 0;

    if (!conf->name && name) {
        conf->name = strndup(name, name_len);
        if (!conf->name) {
            ret = -ENOMEM;
            goto fail;
        }
    }

    if (ip) {
        free(conf->address);
        conf->address = strdup(ip);
        if (!conf->address) {
            ret = -ENOMEM;
            goto fail;
        }
    }

    if (!conf->address) {
        ret = -EINVAL;
        goto fail;
    }

    if (port != ULONG_MAX) {
        conf->port = port;
    }

    if (conf->port == ULONG_MAX) {
        ret = -EINVAL;
        goto fail;
    }

    conf->retry_timeout = timeout;

    conf->next = opt.endpoints;
    opt.endpoints = conf;

    return 0;

fail:
    free(conf->address);
    free(conf->name);
    free(conf);

    return ret;
}

static int add_udp_endpoint_address(const char *name, size_t name_len, const char *ip,
                                    long unsigned port, bool eavesdropping, const char *filter,
                                    int coalesce_bytes, int coalesce_ms, const char *coalesce_nodelay,
                                    uint32_t dropout_percentage)
{
    int ret;

    struct endpoint_config *conf
        = (struct endpoint_config *)calloc(1, sizeof(struct endpoint_config));
    assert_or_return(conf, -ENOMEM);
    assert_or_return(ip != nullptr, -EINVAL);
    conf->type = Udp;
    conf->port = ULONG_MAX;
    conf->dropout_percentage = dropout_percentage;

    if (name) {
        conf->name = strndup(name, name_len);
        if (!conf->name) {
            ret = -ENOMEM;
            goto fail;
        }
    }

    conf->address = strdup(ip);
    if (!conf->address) {
        ret = -ENOMEM;
        goto fail;
    }

    if (filter) {
        conf->filter = strdup(filter);
        if (!conf->filter) {
            ret = -ENOMEM;
            goto fail;
        }
    }

    if (port != ULONG_MAX) {
        conf->port = port;
    }

    if (conf->port == ULONG_MAX) {
        conf->port = find_next_endpoint_port(conf->address);
    }

    conf->eavesdropping = eavesdropping;
    conf->coalesce_bytes = coalesce_bytes;
    conf->coalesce_ms = coalesce_ms;

    if (coalesce_nodelay) {
        conf->coalesce_nodelay = strdup(coalesce_nodelay);
        if (!conf->coalesce_nodelay) {
            ret = -ENOMEM;
            goto fail;
        }
    }

    conf->next = opt.endpoints;
    opt.endpoints = conf;

    return 0;

fail:
    free(conf->address);
    free(conf->name);
    free(conf);

    return ret;
}

static std::vector<unsigned long> *strlist_to_ul(const char *list,
                                                 const char *listname,
                                                 const char *delim,
                                                 unsigned long default_value)
{
    char *s, *tmp_str;
    std::unique_ptr<std::vector<unsigned long>> v{new std::vector<unsigned long>()};

    if (!list || list[0] == '\0') {
        v->push_back(default_value);
        return v.release();
    }

    tmp_str = strdup(list);
    if (!tmp_str) {
        return nullptr;
    }

    s = strtok(tmp_str, delim);
    while (s) {
        unsigned long l;
        if (safe_atoul(s, &l) < 0) {
            log_error("Invalid %s %s", listname, s);
            goto error;
        }
        v->push_back(l);
        s = strtok(NULL, delim);
    }

    free(tmp_str);

    if (!v->size()) {
        log_error("No valid %s on %s", listname, list);
        return nullptr;
    }

    return v.release();

error:
    free(tmp_str);
    return nullptr;
}

static int add_uart_endpoint(const char *name, size_t name_len, const char *uart_device,
                             const char *bauds, bool flowcontrol, uint32_t dropout_percentage)
{
    int ret;

    struct endpoint_config *conf
        = (struct endpoint_config *)calloc(1, sizeof(struct endpoint_config));
    assert_or_return(conf, -ENOMEM);
    conf->type = Uart;
    conf->dropout_percentage = dropout_percentage;

    if (name) {
        conf->name = strndup(name, name_len);
        if (!conf->name) {
            ret = -ENOMEM;
            goto fail;
        }
    }

    conf->device = strdup(uart_device);
    if (!conf->device) {
        ret = -ENOMEM;
        goto fail;
    }

    conf->bauds = strlist_to_ul(bauds, "baud", ",", DEFAULT_BAUDRATE);
    if (!conf->bauds) {
        ret = -EINVAL;
        goto fail;
    }

    conf->flowcontrol = flowcontrol;

    conf->next = opt.endpoints;
    opt.endpoints = conf;

    return 0;

fail:
    free(conf->device);
    free(conf->name);
    free(conf);

    return ret;
}

static bool pre_parse_argv(int argc, char *argv[])
{
    // This function parses only conf-file and conf-dir from
    // command line, so we can read the conf files.
    // parse_argv will then parse all other options, overriding
    // config files definitions

    int c;

    while ((c = getopt_long(argc, argv, short_options, long_options, NULL)) >= 0) {
        switch (c) {
        case 'c': {
            opt.conf_file_name = optarg;
            break;
        }
        case 'd': {
            opt.conf_dir = optarg;
            break;
        }
        case 'V':
            puts(PACKAGE " version " VERSION);
            return false;
        }
    }

    // Reset getopt*
    optind = 1;

    return true;
}

static int parse_argv(int argc, char *argv[])
{
    int c;
    struct stat st;

    assert(argc >= 0);
    assert(argv);

    while ((c = getopt_long(argc, argv, short_options, long_options, NULL)) >= 0) {
        switch (c) {
        case 'h':
            help(stdout);
            return 0;
        case 'e': {
            char *ip;
            unsigned long port;

            if (split_on_last_colon(optarg, &ip, &port) < 0) {
                log_error("Invalid port in argument: %s", optarg);
                help(stderr);
                return -EINVAL;
            }
            if (validate_ip(ip) < 0) {
                log_error("Invalid IP address in argument: %s", optarg);
                help(stderr);
                return -EINVAL;
            }

            add_udp_endpoint_address(NULL, 0, ip, port, false, NULL, 0, 0, NULL, 0);
            free(ip);
            break;
        }
        case 'r': {
            opt.report_msg_statistics = true;
            break;
        }
        case 't': {
            if (safe_atoul(optarg, &opt.tcp_port) < 0) {
                log_error("Invalid argument for tcp-port = %s", optarg);
                help(stderr);
                return -EINVAL;
            }
            break;
        }
        case 'l': {
            opt.logs_dir = strdup(optarg);
            break;
        }
        case 'g': {
            int lvl = log_level_from_str(optarg);
            if (lvl == -EINVAL) {
                log_error("Invalid argument for debug-log-level = %s", optarg);
                help(stderr);
                return -EINVAL;
            }
            opt.debug_log_level = lvl;
            break;
        }
        case 'b': {
            opt.heartbeat = true;
            break;
        }
        case 'v': {
            opt.debug_log_level = (int)Log::Level::DEBUG;
            break;
        }
        case 'p': {
            char *ip;
            unsigned long port;

            if (split_on_last_colon(optarg, &ip, &port) < 0) {
                log_error("Invalid port in argument: %s", optarg);
                help(stderr);
                return -EINVAL;
            }
            if (port == ULONG_MAX) {
                log_error("Missing port in argument: %s", optarg);
                free(ip);
                help(stderr);
                return -EINVAL;
            }
            if (validate_ip(ip) < 0) {
                log_error("Invalid IP address in argument: %s", optarg);
                help(stderr);
                return -EINVAL;
            }

            add_tcp_endpoint_address(NULL, 0, ip, port, DEFAULT_RETRY_TCP_TIMEOUT);
            free(ip);
            break;
        }
        case 'c':
        case 'd':
        case 'V':
            break; // These options were parsed on pre_parse_argv
        case '?':
        default:
            help(stderr);
            return -EINVAL;
        }
    }

    /* positional arguments */
    while (optind < argc) {
        // UDP and UART master endpoints are of the form:
        // UDP: <ip>:<port> UART: <device>[:<baudrate>]
        char *base;
        unsigned long number;

        if (split_on_last_colon(argv[optind], &base, &number) < 0) {
            log_error("Invalid argument %s", argv[optind]);
            help(stderr);
            return -EINVAL;
        }

        if (stat(base, &st) == -1 || !S_ISCHR(st.st_mode)) {
            if (number == ULONG_MAX) {
                log_error("Invalid argument for UDP port = %s", argv[optind]);
                help(stderr);
                free(base);
                return -EINVAL;
            }
            if (validate_ip(base) < 0) {
                log_error("Invalid IP address in argument: %s", argv[optind]);
                help(stderr);
                return -EINVAL;
            }

            add_udp_endpoint_address(NULL, 0, base, number, true, NULL, 0, 0, NULL, 0);
        } else {
            const char *bauds = number != ULONG_MAX ? base + strlen(base) + 1 : NULL;
            int ret = add_uart_endpoint(NULL, 0, base, bauds, false, 0);
            if (ret < 0) {
                free(base);
                return ret;
            }
        }
        free(base);
        optind++;
    }

    return 2;
}

static const char *get_conf_file_name()
{
    char *s;

    if (opt.conf_file_name)
        return opt.conf_file_name;

    s = getenv("MAVLINK_ROUTERD_CONF_FILE");
    if (s)
        return s;

    return DEFAULT_CONFFILE;
}

static const char *get_conf_dir()
{
    char *s;

    if (opt.conf_dir)
        return opt.conf_dir;

    s = getenv("MAVLINK_ROUTERD_CONF_DIR");
    if (s)
        return s;

    return DEFAULT_CONF_DIR;
}

static int parse_mavlink_dialect(const char *val, size_t val_len, void *storage, size_t storage_len)
{
    assert(val);
    assert(storage);
    assert(val_len);

    enum mavlink_dialect *dialect = (enum mavlink_dialect *)storage;

    if (storage_len < sizeof(options::mavlink_dialect))
        return -ENOBUFS;
    if (val_len > INT_MAX)
        return -EINVAL;

    if (memcaseeq(val, val_len, "auto", sizeof("auto") - 1)) {
        *dialect = Auto;
    } else if (memcaseeq(val, val_len, "common", sizeof("common") - 1)) {
        *dialect = Common;
    } else if (memcaseeq(val, val_len, "ardupilotmega", sizeof("ardupilotmega") - 1)) {
        *dialect = Ardupilotmega;
    } else {
        log_error("Invalid argument for MavlinkDialect = %.*s", (int)val_len, val);
        return -EINVAL;
    }

    return 0;
}

#define MAX_LOG_LEVEL_SIZE 10
static int parse_log_level(const char *val, size_t val_len, void *storage, size_t storage_len)
{
    assert(val);
    assert(storage);
    assert(val_len);

    if (storage_len < sizeof(options::debug_log_level))
        return -ENOBUFS;
    if (val_len > MAX_LOG_LEVEL_SIZE)
        return -EINVAL;

    const char *log_level = strndupa(val, val_len);
    int lvl = log_level_from_str(log_level);
    if (lvl == -EINVAL) {
        log_error("Invalid argument for DebugLogLevel = %s", log_level);
        return -EINVAL;
    }
    *((int *)storage) = lvl;

    return 0;
}
#undef MAX_LOG_LEVEL_SIZE

#define MAX_LOG_MODE_SIZE 20
static int parse_log_mode(const char *val, size_t val_len, void *storage, size_t storage_len)
{
    assert(val);
    assert(storage);
    assert(val_len);

    if (storage_len < sizeof(options::log_mode))
        return -ENOBUFS;
    if (val_len > MAX_LOG_MODE_SIZE)
        return -EINVAL;

    const char *log_mode_str = strndupa(val, val_len);
    LogMode log_mode;
    if (strcaseeq(log_mode_str, "always"))
        log_mode = LogMode::always;
    else if (strcaseeq(log_mode_str, "while-armed"))
        log_mode = LogMode::while_armed;
    else {
        log_error("Invalid argument for LogMode = %s", log_mode_str);
        return -EINVAL;
    }
    *((LogMode *)storage) = log_mode;

    return 0;
}
#undef MAX_LOG_MODE_SIZE


static int parse_mode(const char *val, size_t val_len, void *storage, size_t storage_len)
{
    assert(val);
    assert(storage);
    assert(val_len);

    if (storage_len < sizeof(bool))
        return -ENOBUFS;
    if (val_len > INT_MAX)
        return -EINVAL;

    bool *eavesdropping = (bool *)storage;
    if (memcaseeq(val, val_len, "normal", sizeof("normal") - 1)) {
        *eavesdropping = false;
    } else if (memcaseeq(val, val_len, "eavesdropping", sizeof("eavesdropping") - 1)) {
        *eavesdropping = true;
    } else {
        log_error("Unknown 'mode' key: %.*s", (int)val_len, val);
        return -EINVAL;
    }

    return 0;
}

static int parse_confs(ConfFile &conf)
{
    int ret;
    size_t offset;
    struct ConfFile::section_iter iter;
    const char *pattern;

    static const ConfFile::OptionsTable option_table[] = {
        {"TcpServerPort", false, ConfFile::parse_ul, OPTIONS_TABLE_STRUCT_FIELD(options, tcp_port)},
        {"ReportStats", false, ConfFile::parse_bool,
         OPTIONS_TABLE_STRUCT_FIELD(options, report_msg_statistics)},
        {"MavlinkDialect", false, parse_mavlink_dialect,
         OPTIONS_TABLE_STRUCT_FIELD(options, mavlink_dialect)},
        {"Log", false, ConfFile::parse_str_dup, OPTIONS_TABLE_STRUCT_FIELD(options, logs_dir)},
        {"LogMode", false, parse_log_mode, OPTIONS_TABLE_STRUCT_FIELD(options, log_mode)},
        {"DebugLogLevel", false, parse_log_level,
         OPTIONS_TABLE_STRUCT_FIELD(options, debug_log_level)},
        {"MinFreeSpace", false, ConfFile::parse_ul,
         OPTIONS_TABLE_STRUCT_FIELD(options, min_free_space)},
        {"MaxLogFiles", false, ConfFile::parse_ul,
         OPTIONS_TABLE_STRUCT_FIELD(options, max_log_files)},
    };

    struct option_uart {
        char *device;
        char *bauds;
        bool flowcontrol;
        unsigned long dropout_percentage;
    };
    static const ConfFile::OptionsTable option_table_uart[] = {
        {"baud",        false,  ConfFile::parse_str_dup,    OPTIONS_TABLE_STRUCT_FIELD(option_uart, bauds)},
        {"device",      true,   ConfFile::parse_str_dup,    OPTIONS_TABLE_STRUCT_FIELD(option_uart, device)},
        {"FlowControl", false,  ConfFile::parse_bool,       OPTIONS_TABLE_STRUCT_FIELD(option_uart, flowcontrol)},
        {"DropoutPercentage",false, ConfFile::parse_ul,     OPTIONS_TABLE_STRUCT_FIELD(option_uart, dropout_percentage)},
    };

    struct option_udp {
        char *addr;
        bool eavesdropping;
        unsigned long port;
        char *filter;
        unsigned long coalesce_bytes;
        unsigned long coalesce_ms;
        char *coalesce_nodelay;
        unsigned long dropout_percentage;
    };
    static const ConfFile::OptionsTable option_table_udp[] = {
        {"address",         true,   ConfFile::parse_str_dup,    OPTIONS_TABLE_STRUCT_FIELD(option_udp, addr)},
        {"mode",            true,   parse_mode,                 OPTIONS_TABLE_STRUCT_FIELD(option_udp, eavesdropping)},
        {"port",            false,  ConfFile::parse_ul,         OPTIONS_TABLE_STRUCT_FIELD(option_udp, port)},
        {"filter",          false,  ConfFile::parse_str_dup,    OPTIONS_TABLE_STRUCT_FIELD(option_udp, filter)},
        {"CoalesceBytes",   false,  ConfFile::parse_ul,         OPTIONS_TABLE_STRUCT_FIELD(option_udp, coalesce_bytes)},
        {"CoalesceMs",      false,  ConfFile::parse_ul,         OPTIONS_TABLE_STRUCT_FIELD(option_udp, coalesce_ms)},
        {"CoalesceNoDelay", false,  ConfFile::parse_str_dup,    OPTIONS_TABLE_STRUCT_FIELD(option_udp, coalesce_nodelay)},
        {"DropoutPercentage",false, ConfFile::parse_ul,         OPTIONS_TABLE_STRUCT_FIELD(option_udp, dropout_percentage)},
    };

    struct option_tcp {
        char *addr;
        unsigned long port;
        int timeout;
    };
    static const ConfFile::OptionsTable option_table_tcp[] = {
        {"address",         true,   ConfFile::parse_str_dup,    OPTIONS_TABLE_STRUCT_FIELD(option_tcp, addr)},
        {"port",            true,   ConfFile::parse_ul,         OPTIONS_TABLE_STRUCT_FIELD(option_tcp, port)},
        {"RetryTimeout",    false,  ConfFile::parse_i,          OPTIONS_TABLE_STRUCT_FIELD(option_tcp, timeout)},
    };

    ret = conf.extract_options("General", option_table, ARRAY_SIZE(option_table), &opt);
    if (ret < 0)
        return ret;

    iter = {};
    pattern = "uartendpoint *";
    offset = strlen(pattern) - 1;
    while (conf.get_sections(pattern, &iter) == 0) {
        struct option_uart opt_uart = {nullptr, nullptr};
        ret = conf.extract_options(&iter, option_table_uart, ARRAY_SIZE(option_table_uart),
                                   &opt_uart);
        if (ret == 0)
            ret = add_uart_endpoint(iter.name + offset, iter.name_len - offset, opt_uart.device,
                                    opt_uart.bauds, opt_uart.flowcontrol, opt_uart.dropout_percentage);
        free(opt_uart.device);
        free(opt_uart.bauds);
        if (ret < 0)
            return ret;
    }

    iter = {};
    pattern = "udpendpoint *";
    offset = strlen(pattern) - 1;
    while (conf.get_sections(pattern, &iter) == 0) {
        struct option_udp opt_udp = {nullptr, false, ULONG_MAX, nullptr, 0, 0, nullptr};
        ret = conf.extract_options(&iter, option_table_udp, ARRAY_SIZE(option_table_udp), &opt_udp);
        if (ret == 0) {
            if (opt_udp.eavesdropping && opt_udp.port == ULONG_MAX) {
                log_error("Expected 'port' key for section %.*s", (int)iter.name_len, iter.name);
                ret = -EINVAL;
            } else {
                    ret = add_udp_endpoint_address(iter.name + offset, iter.name_len - offset, opt_udp.addr,
                                                   opt_udp.port, opt_udp.eavesdropping, opt_udp.filter, opt_udp.coalesce_bytes,
                                                   opt_udp.coalesce_ms, opt_udp.coalesce_nodelay, opt_udp.dropout_percentage);
            }
        }

        free(opt_udp.addr);
        free(opt_udp.coalesce_nodelay);
        free(opt_udp.filter);
        if (ret < 0)
            return ret;
    }

    iter = {};
    pattern = "tcpendpoint *";
    offset = strlen(pattern) - 1;
    while (conf.get_sections(pattern, &iter) == 0) {
        struct option_tcp opt_tcp = {nullptr, ULONG_MAX, DEFAULT_RETRY_TCP_TIMEOUT};
        ret = conf.extract_options(&iter, option_table_tcp, ARRAY_SIZE(option_table_tcp), &opt_tcp);

        if (ret == 0) {
                ret = add_tcp_endpoint_address(iter.name + offset, iter.name_len - offset, opt_tcp.addr,
                                               opt_tcp.port, opt_tcp.timeout);
        }
        free(opt_tcp.addr);
        if (ret < 0)
            return ret;
    }

    return 0;
}

static int cmpstr(const void *s1, const void *s2)
{
    return strcmp(*(const char **)s1, *(const char **)s2);
}

static int parse_conf_files()
{
    DIR *dir;
    struct dirent *ent;
    const char *filename, *dirname;
    int ret = 0;
    char *files[128] = {};
    int i = 0, j = 0;
    ConfFile conf;

    // First, open default conf file
    filename = get_conf_file_name();
    ret = conf.parse(filename);

    // If there's no default conf file, everything is good
    if (ret < 0 && ret != -ENOENT) {
        return ret;
    }

    dirname = get_conf_dir();
    // Then, parse all files on configuration directory
    dir = opendir(dirname);
    if (!dir)
        return parse_confs(conf);

    while ((ent = readdir(dir))) {
        char path[PATH_MAX];
        struct stat st;

        ret = snprintf(path, sizeof(path), "%s/%s", dirname, ent->d_name);
        if (ret >= (int)sizeof(path)) {
            log_error("Couldn't open directory %s", dirname);
            ret = -EINVAL;
            goto fail;
        }
        if (stat(path, &st) < 0 || !S_ISREG(st.st_mode)) {
            continue;
        }
        files[i] = strdup(path);
        if (!files[i]) {
            ret = -ENOMEM;
            goto fail;
        }
        i++;

        if ((size_t)i > sizeof(files) / sizeof(*files)) {
            log_warning("Too many files on %s. Not all of them will be considered", dirname);
            break;
        }
    }

    qsort(files, (size_t)i, sizeof(char *), cmpstr);

    for (j = 0; j < i; j++) {
        ret = conf.parse(files[j]);
        if (ret < 0)
            goto fail;
        free(files[j]);
    }

    closedir(dir);

    return parse_confs(conf);
fail:
    while (j < i) {
        free(files[j++]);
    }

    closedir(dir);

    return ret;
}

/*
 * Frees dynamically allocated strings in options struct. This code was
 * extracted from Mainloop (where it does not belong at all) and simply
 * moved here verbatim as a "slightly" better place.
 *
 * XXX: it should go to proper encapsulation in config system; it should
 * not use C memory management.
 */
static void free_endpoints_options_strings(struct options* opts)
{
    for (auto e = opts->endpoints; e;) {
        auto next = e->next;
        if (e->type == Udp || e->type == Tcp) {
            free(e->address);
            free(e->coalesce_nodelay);
        } else {
            free(e->device);
            delete e->bauds;
        }
        free(e->filter);
        free(e->name);
        free(e);
        e = next;
    }
}

int main(int argc, char *argv[])
{
    Mainloop mainloop;

    Log::open();

    if (!pre_parse_argv(argc, argv)) {
        Log::close();
        return 0;
    }

    if (parse_conf_files() < 0)
        goto close_log;

    if (parse_argv(argc, argv) != 2)
        goto close_log;

    Log::set_max_level((Log::Level) opt.debug_log_level);

    dbg("Cmd line and options parsed");

    if (opt.tcp_port == ULONG_MAX)
        opt.tcp_port = MAVLINK_TCP_PORT;

    if (!mainloop.add_endpoints(mainloop, &opt))
        goto endpoint_error;

    mainloop.loop();

    free_endpoints_options_strings(&opt);

    free(opt.logs_dir);

    Log::close();

    return 0;

endpoint_error:
    free_endpoints_options_strings(&opt);

    free(opt.logs_dir);

close_log:
    Log::close();
    return EXIT_FAILURE;
}

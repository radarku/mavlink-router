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

#include <algorithm>
#include <string>
#include <vector>

#include <common/conf_file.h>
#include <common/dbg.h>
#include <common/log.h>
#include <common/util.h>

#include "comm.h"
#include "endpoint.h"
#include "mainloop.h"

#define MAVLINK_TCP_PORT 5760
#define DEFAULT_CONFFILE "/etc/mavlink-router/main.conf"
#define DEFAULT_CONF_DIR "/etc/mavlink-router/config.d"

static const struct option long_options[] = {
    { "endpoints",              required_argument,  NULL,   'e' },
    { "conf-file",              required_argument,  NULL,   'c' },
    { "conf-dir" ,              required_argument,  NULL,   'd' },
    { "report_msg_statistics",  no_argument,        NULL,   'r' },
    { "tcp-port",               required_argument,  NULL,   't' },
    { "tcp-endpoint",           required_argument,  NULL,   'p' },
    { "log",                    required_argument,  NULL,   'l' },
    { "debug-log-level",        required_argument,  NULL,   'g' },
    { "verbose",                no_argument,        NULL,   'v' },
    { "version",                no_argument,        NULL,   'V' },
    { }
};

static const char* short_options = "he:rt:c:d:l:p:g:vV";

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
            "  -v --verbose                 Verbose. Same as --debug-log-level=debug\n"
            "  -V --version                 Show version\n"
            "  -h --help                    Print this message\n"
            , program_invocation_short_name);
}

static int split_on_colon(const char *str, char **base, unsigned long *number)
{
    char *colonstr;

    *base = strdup(str);
    colonstr = strchrnul(*base, ':');
    *number = ULONG_MAX;

    if (*colonstr != '\0') {
        *colonstr = '\0';
        if (safe_atoul(colonstr + 1, number) < 0) {
            free(*base);
            return -EINVAL;
        }
    }

    return 0;
}

static bool pre_parse_argv(int argc, char *argv[], options* opt)
{
    // This function parses only conf-file and conf-dir from
    // command line, so we can read the conf files.
    // parse_argv will then parse all other options, overriding
    // config files definitions

    int c;

    while ((c = getopt_long(argc, argv, short_options, long_options, NULL)) >= 0) {
        switch (c) {
        case 'c': {
            opt->conf_file_name = optarg;
            break;
        }
        case 'd': {
            opt->conf_dir = optarg;
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

static int parse_argv(int argc, char *argv[], options* opt)
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

            if (split_on_colon(optarg, &ip, &port) < 0) {
                log_error("Invalid port in argument: %s", optarg);
                help(stderr);
                return -EINVAL;
            }

            add_endpoint_address(NULL, 0, ip, port, false, NULL, opt);
            free(ip);
            break;
        }
        case 'r': {
            opt->report_msg_statistics = true;
            break;
        }
        case 't': {
            if (safe_atoul(optarg, &opt->tcp_port) < 0) {
                log_error("Invalid argument for tcp-port = %s", optarg);
                help(stderr);
                return -EINVAL;
            }
            break;
        }
        case 'l': {
            opt->logs_dir = optarg;
            break;
        }
        case 'g': {
            int lvl = log_level_from_str(optarg);
            if (lvl == -EINVAL) {
                log_error("Invalid argument for debug-log-level = %s", optarg);
                help(stderr);
                return -EINVAL;
            }
            opt->debug_log_level = lvl;
            break;
        }
        case 'v': {
            opt->debug_log_level = (int)Log::Level::DEBUG;
            break;
        }
        case 'p': {
            char *ip;
            unsigned long port;

            if (split_on_colon(optarg, &ip, &port) < 0) {
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

            add_tcp_endpoint_address(NULL, 0, ip, port, DEFAULT_RETRY_TCP_TIMEOUT, opt);
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

        if (split_on_colon(argv[optind], &base, &number) < 0) {
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

            add_endpoint_address(NULL, 0, base, number, true, NULL, opt);
        } else {
            const char *bauds = number != ULONG_MAX ? base + strlen(base) + 1 : NULL;
            int ret = add_uart_endpoint(NULL, 0, base, bauds, false, opt);
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

static std::string get_conf_file_name(const options& opt)
{
    if (!opt.conf_file_name.empty()) {
        return opt.conf_file_name;
    } else {
        const char* s = getenv("MAVLINK_ROUTERD_CONF_FILE");
        if (s) {
            return s;
        }
    }

    return DEFAULT_CONFFILE;
}

static std::string get_conf_dir(const options& opt)
{
    if (!opt.conf_dir.empty()) {
        return opt.conf_dir;
    } else {
        const char* s = getenv("MAVLINK_ROUTERD_CONF_DIR");
        if (s) {
            return s;
        }
    }

    return DEFAULT_CONF_DIR;
}

static int parse_conf_files(options* opt)
{
    DIR *dir;
    struct dirent *ent;
    int ret = 0;
    std::vector<std::string> files;
    ConfFile conf;

    // First, open default conf file
    ret = conf.parse(get_conf_file_name(*opt).c_str());

    // If there's no default conf file, everything is good
    if (ret < 0 && ret != -ENOENT) {
        return ret;
    }

    std::string dirname = get_conf_dir(*opt);
    // Then, parse all files on configuration directory
    dir = opendir(dirname.c_str());
    if (!dir)
        return parse_confs(conf, opt);

    while ((ent = readdir(dir))) {
        std::string path = dirname + "/" + ent->d_name;
        struct stat st;
        if (stat(path.c_str(), &st) < 0 || !S_ISREG(st.st_mode)) {
            continue;
        }
        files.push_back(path);
    }
    closedir(dir);

    std::sort(files.begin(), files.end());

    for (const auto& file : files) {
        ret = conf.parse(file.c_str());
        if (ret < 0) {
            return ret;
        }
    }

    return parse_confs(conf, opt);
}

int main(int argc, char *argv[])
{
    struct options opt;

    Mainloop mainloop;

    Log::open();

    if (!pre_parse_argv(argc, argv, &opt)) {
        Log::close();
        return 0;
    }

    if (parse_conf_files(&opt) < 0)
        goto close_log;

    if (parse_argv(argc, argv, &opt) != 2)
        goto close_log;

    Log::set_max_level((Log::Level) opt.debug_log_level);

    dbg("Cmd line and options parsed");

    if (opt.tcp_port == ULONG_MAX)
        opt.tcp_port = MAVLINK_TCP_PORT;

    if (!mainloop.add_endpoints(mainloop, &opt))
        goto endpoint_error;

    mainloop.loop();

    Log::close();

    return 0;

endpoint_error:
close_log:
    Log::close();
    return EXIT_FAILURE;
}

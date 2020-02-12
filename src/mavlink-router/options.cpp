#include "options.h"

#include <limits.h>

#include <src/common/util.h>

namespace {

int parse_mode(const char *val, size_t val_len, void *storage, size_t storage_len)
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

int parse_log_mode(const char *val, size_t val_len, void *storage, size_t storage_len)
{
    static constexpr size_t MAX_LOG_MODE_SIZE = 20;
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

int parse_log_level(const char *val, size_t val_len, void *storage, size_t storage_len)
{
    static constexpr size_t MAX_LOG_LEVEL_SIZE = 10;
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

int parse_mavlink_dialect(const char *val, size_t val_len, void *storage, size_t storage_len)
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

unsigned long find_next_endpoint_port(const struct options& opt, const char *ip)
{
    unsigned long port = 14550U;

    for (const auto& conf : opt.endpoints) {
        if (conf.type == Udp && conf.address == ip && conf.port == port) {
            port++;
            break;
        }
    }

    return port;
}

}  // namespace

int add_tcp_endpoint_address(const char *name, size_t name_len, const char *ip,
                            long unsigned port, int timeout, options* opt)
{
    endpoint_config conf;
    conf.type = Tcp;
    conf.port = ULONG_MAX;

    conf.name = name ? std::string(name, name + name_len) : "";
    conf.address = ip;

    conf.port = (port == ULONG_MAX) ? 0 : port;
    conf.retry_timeout = timeout;

    // Originally this was a singly linked list, defined to do front insertion:
    // Config items used to appear in reverse order for that reason. That is
    // a bit non-intuitive that config items appear in reverse order, but it
    // really just preserves previous behavior to avoid risk of breaking things.
    opt->endpoints.push_front(conf);

    return 0;
}

int add_endpoint_address(const char *name, size_t name_len, const char *ip,
                         long unsigned port, bool eavesdropping, const char *filter,
                         options* opt)
{
    endpoint_config conf;
    conf.type = Udp;
    conf.port = ULONG_MAX;
    conf.name = name ? std::string(name, name + name_len) : "";
    conf.address = ip;
    if (filter) {
        conf.filter = filter;
    }

    conf.port =
        (port == ULONG_MAX) ?
        find_next_endpoint_port(*opt, ip) :
        port;

    conf.eavesdropping = eavesdropping;

    // See comment above about push_front.
    opt->endpoints.push_front(conf);

    return 0;
}

namespace {

std::vector<unsigned long> strlist_to_ul(const char *list,
                                         const char *listname,
                                         const char *delim,
                                         unsigned long default_value)
{
    char *s, *tmp_str;
    std::vector<unsigned long> v;

    if (!list || list[0] == '\0') {
        v.push_back(default_value);
        return v;
    }

    std::vector<char> tmp_storage(list, list + strlen(list) + 1);
    tmp_str = &tmp_storage[0];
    if (!tmp_str) {
        return {};
    }

    s = strtok(tmp_str, delim);
    while (s) {
        unsigned long l;
        if (safe_atoul(s, &l) < 0) {
            log_error("Invalid %s %s", listname, s);
            v.clear();
            break;
        }
        v.push_back(l);
        s = strtok(NULL, delim);
    }

    free(tmp_str);

    if (!v.size()) {
        log_error("No valid %s on %s", listname, list);
    }

    return v;
}

}  // namespace

int add_uart_endpoint(const char *name, size_t name_len, const char *uart_device,
                      const char *bauds, bool flowcontrol, options* opt)
{
    endpoint_config conf;
    conf.type = Uart;

    conf.name = name ? std::string(name, name + name_len) : "";
    conf.device = uart_device;
    conf.bauds = strlist_to_ul(bauds, "baud", ",", DEFAULT_BAUDRATE);
    conf.flowcontrol = flowcontrol;
    // See comment above about push_front.
    opt->endpoints.push_front(conf);

    return 0;
}

int parse_confs(ConfFile &conf, options* opt)
{
    int ret;
    size_t offset;
    struct ConfFile::section_iter iter;
    const char *pattern;

    struct option_general {
        unsigned long tcp_port;
        bool report_msg_statistics;
        enum mavlink_dialect mavlink_dialect;
        char* logs_dir;
        LogMode log_mode;
        int debug_log_level;
        unsigned long min_free_space;
        unsigned long max_log_files;
    };

    static const ConfFile::OptionsTable option_table[] = {
        {"TcpServerPort", false, ConfFile::parse_ul, OPTIONS_TABLE_STRUCT_FIELD(option_general, tcp_port)},
        {"ReportStats", false, ConfFile::parse_bool,
         OPTIONS_TABLE_STRUCT_FIELD(option_general, report_msg_statistics)},
        {"MavlinkDialect", false, parse_mavlink_dialect,
         OPTIONS_TABLE_STRUCT_FIELD(option_general, mavlink_dialect)},
        {"Log", false, ConfFile::parse_str_dup, OPTIONS_TABLE_STRUCT_FIELD(option_general, logs_dir)},
        {"LogMode", false, parse_log_mode, OPTIONS_TABLE_STRUCT_FIELD(option_general, log_mode)},
        {"DebugLogLevel", false, parse_log_level,
         OPTIONS_TABLE_STRUCT_FIELD(option_general, debug_log_level)},
        {"MinFreeSpace", false, ConfFile::parse_ul,
         OPTIONS_TABLE_STRUCT_FIELD(option_general, min_free_space)},
        {"MaxLogFiles", false, ConfFile::parse_ul,
         OPTIONS_TABLE_STRUCT_FIELD(option_general, max_log_files)},
    };

    struct option_uart {
        char *device;
        char *bauds;
        bool flowcontrol;
    };
    static const ConfFile::OptionsTable option_table_uart[] = {
        {"baud",        false,  ConfFile::parse_str_dup,    OPTIONS_TABLE_STRUCT_FIELD(option_uart, bauds)},
        {"device",      true,   ConfFile::parse_str_dup,    OPTIONS_TABLE_STRUCT_FIELD(option_uart, device)},
        {"FlowControl", false,  ConfFile::parse_bool,       OPTIONS_TABLE_STRUCT_FIELD(option_uart, flowcontrol)},
    };

    struct option_udp {
        char *addr;
        bool eavesdropping;
        unsigned long port;
        char *filter;
    };
    static const ConfFile::OptionsTable option_table_udp[] = {
        {"address", true,   ConfFile::parse_str_dup,    OPTIONS_TABLE_STRUCT_FIELD(option_udp, addr)},
        {"mode",    true,   parse_mode,                 OPTIONS_TABLE_STRUCT_FIELD(option_udp, eavesdropping)},
        {"port",    false,  ConfFile::parse_ul,         OPTIONS_TABLE_STRUCT_FIELD(option_udp, port)},
        {"filter",  false,  ConfFile::parse_str_dup,    OPTIONS_TABLE_STRUCT_FIELD(option_udp, filter)},
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

    struct option_general opt_general{};
    ret = conf.extract_options("General", option_table, ARRAY_SIZE(option_table), &opt_general);
    if (ret < 0) {
        return ret;
    }

    opt->tcp_port = opt_general.tcp_port;
    opt->report_msg_statistics = opt_general.report_msg_statistics;
    opt->mavlink_dialect = opt_general.mavlink_dialect;
    opt->logs_dir = opt_general.logs_dir ? opt_general.logs_dir : "";
    opt->log_mode = opt_general.log_mode;
    opt->debug_log_level = opt_general.debug_log_level;
    opt->min_free_space = opt_general.min_free_space;
    opt->max_log_files = opt_general.max_log_files;
    free(opt_general.logs_dir);

    iter = {};
    pattern = "uartendpoint *";
    offset = strlen(pattern) - 1;
    while (conf.get_sections(pattern, &iter) == 0) {
        struct option_uart opt_uart = {nullptr, nullptr};
        ret = conf.extract_options(&iter, option_table_uart, ARRAY_SIZE(option_table_uart),
                                   &opt_uart);
        if (ret == 0)
            ret = add_uart_endpoint(iter.name + offset, iter.name_len - offset, opt_uart.device,
                                    opt_uart.bauds, opt_uart.flowcontrol, opt);
        free(opt_uart.device);
        free(opt_uart.bauds);
        if (ret < 0)
            return ret;
    }

    iter = {};
    pattern = "udpendpoint *";
    offset = strlen(pattern) - 1;
    while (conf.get_sections(pattern, &iter) == 0) {
        struct option_udp opt_udp = {nullptr, false, ULONG_MAX};
        ret = conf.extract_options(&iter, option_table_udp, ARRAY_SIZE(option_table_udp), &opt_udp);
        if (ret == 0) {
            if (opt_udp.eavesdropping && opt_udp.port == ULONG_MAX) {
                log_error("Expected 'port' key for section %.*s", (int)iter.name_len, iter.name);
                ret = -EINVAL;
            } else {
                ret = add_endpoint_address(iter.name + offset, iter.name_len - offset, opt_udp.addr,
                                           opt_udp.port, opt_udp.eavesdropping, opt_udp.filter, opt);
            }
        }

        free(opt_udp.addr);
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
                                           opt_tcp.port, opt_tcp.timeout, opt);
        }
        free(opt_tcp.addr);
        if (ret < 0)
            return ret;
    }

    return 0;
}

int log_level_from_str(const char *str)
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

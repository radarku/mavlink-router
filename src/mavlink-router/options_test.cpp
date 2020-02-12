#include "options.h"

#include <cstring>

#include <gtest/gtest.h>

class OptionsTest : public ::testing::Test {
public:
    options parse_string(const char* s)
    {
        ConfFile conf;
        conf.parse_buffer(s, std::strlen(s), "foo");

        options opt;
        EXPECT_EQ(0, parse_confs(conf, &opt));
        return opt;
    }

    static const endpoint_config*
    lookup_endpoint_by_name(const options& opt, const char* name)
    {
        for (const auto& ep : opt.endpoints) {
            if (ep.name == name) {
                return &ep;
            }
        }
        return nullptr;
    }
};

TEST_F(OptionsTest, empty)
{
    options opt = parse_string("");
    EXPECT_TRUE(opt.endpoints.empty());
}

TEST_F(OptionsTest, general)
{
    options opt = parse_string(
        "[General]\n"
        "Log=/tmp/foo\n"
        "TcpServerPort=1234\n");
    EXPECT_TRUE(opt.endpoints.empty());
    EXPECT_EQ("/tmp/foo", opt.logs_dir);
    EXPECT_EQ(1234, opt.tcp_port);
}

TEST_F(OptionsTest, endpoints)
{
    options opt = parse_string(
        "[udpendpoint ep1]\n"
        "mode=eavesdropping\n"
        "address=127.0.0.1\n"
        "port=1234\n"
        "filter=42,43\n"
        "[tcpendpoint ep2]\n"
        "address=127.0.0.1\n"
        "port=5678\n"
        "[uartendpoint ep3]\n"
        "device=/dev/ttyS0\n");
    EXPECT_EQ(3, opt.endpoints.size());

    const auto* ep1 = lookup_endpoint_by_name(opt, "ep1");
    EXPECT_EQ("127.0.0.1", ep1->address);
    EXPECT_EQ(1234, ep1->port);
    EXPECT_EQ(true, ep1->eavesdropping);
    EXPECT_EQ("42,43", ep1->filter);

    const auto* ep2 = lookup_endpoint_by_name(opt, "ep2");
    EXPECT_EQ("127.0.0.1", ep2->address);
    EXPECT_EQ(5678, ep2->port);

    const auto* ep3 = lookup_endpoint_by_name(opt, "ep3");
    EXPECT_EQ("/dev/ttyS0", ep3->device);
}

#include "location/nmea_parser.h"
#include "location/serial_reader.h"

#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <functional>
#include <gtest/gtest.h>
#include <pty.h>
#include <string>
#include <thread>
#include <unistd.h>

using namespace std::chrono;

static std::string sentence(const std::string & body)
{
    unsigned char checksum = 0;
    for (char value : body) checksum ^= static_cast<unsigned char>(value);
    char suffix[8];
    std::snprintf(suffix, sizeof(suffix), "*%02X\r\n", checksum);
    return "$" + body + suffix;
}

static std::string gga(int quality)
{
    return sentence("GNGGA,080158.30,2929.39408860,N,10634.05546410,E," +
        std::to_string(quality) + ",25,1.0,256.5516,M,-38.0892,M,,");
}

TEST(NmeaQualityTest, ClearsFixedFlagWhenSolutionBecomesFloat)
{
    RTKData data;
    ASSERT_TRUE(NMEAParser::parseSentence(gga(4), data));
    ASSERT_TRUE(data.is_rtk_fixed);
    ASSERT_TRUE(NMEAParser::parseSentence(gga(5), data));
    EXPECT_FALSE(data.is_rtk_fixed);
    EXPECT_TRUE(data.is_rtk_float);
}

TEST(NmeaQualityTest, ClearsFloatFlagWhenSolutionBecomesStandalone)
{
    RTKData data;
    ASSERT_TRUE(NMEAParser::parseSentence(gga(5), data));
    ASSERT_TRUE(NMEAParser::parseSentence(gga(1), data));
    EXPECT_FALSE(data.is_rtk_float);
    EXPECT_FALSE(data.is_rtk_fixed);
    EXPECT_TRUE(data.data_valid);
    EXPECT_EQ(data.rtk_status, "GPS");
}

TEST(NmeaQualityTest, ClearsFixedFlagWhenFixIsLost)
{
    RTKData data;
    ASSERT_TRUE(NMEAParser::parseSentence(gga(4), data));
    ASSERT_TRUE(NMEAParser::parseSentence(gga(0), data));
    EXPECT_FALSE(data.is_rtk_fixed);
    EXPECT_FALSE(data.data_valid);
}

TEST(NmeaQualityTest, GgaWithoutCoordinatesCannotReuseOldValidPosition)
{
    RTKData data;
    ASSERT_TRUE(NMEAParser::parseSentence(gga(4), data));
    ASSERT_TRUE(NMEAParser::parseSentence(
        sentence("GNGGA,080159.30,,,,,4,25,1.0,256.5,M,-38.0,M,,"), data));
    EXPECT_FALSE(data.data_valid);
}

TEST(NmeaQualityTest, RmcWithoutCoordinatesCannotReuseOldValidPosition)
{
    RTKData data;
    ASSERT_TRUE(NMEAParser::parseSentence(gga(4), data));
    ASSERT_TRUE(NMEAParser::parseSentence(
        sentence("GNRMC,080159.30,A,,,,,0.0,0.0,171026,2.6,W,A,C"), data));
    EXPECT_FALSE(data.data_valid);
}

class SerialReaderTest : public ::testing::Test
{
protected:
    void SetUp() override
    {
        char pattern[] = "/tmp/rtk_serial_test_XXXXXX";
        const char * created = mkdtemp(pattern);
        ASSERT_NE(created, nullptr);
        directory = created;
        link = directory + "/rtk-by-id";
        master = createPort();
        ASSERT_GE(master, 0);
        ASSERT_EQ(symlink(slave_name.c_str(), link.c_str()), 0);
        ASSERT_TRUE(reader.initialize(link, 115200));
        reader.startReading();
    }

    void TearDown() override
    {
        reader.stopReading();
        if (master >= 0) close(master);
        unlink(link.c_str());
        rmdir(directory.c_str());
    }

    int createPort()
    {
        int next_master = -1;
        int slave = -1;
        char name[128] = {};
        if (openpty(&next_master, &slave, name, nullptr, nullptr) != 0) return -1;
        close(slave);
        slave_name = name;
        return next_master;
    }

    bool waitFor(const std::function<bool()> & condition, milliseconds timeout = milliseconds(500))
    {
        const auto deadline = steady_clock::now() + timeout;
        do {
            if (condition()) return true;
            std::this_thread::sleep_for(milliseconds(10));
        } while (steady_clock::now() < deadline);
        return condition();
    }

    void send(const std::string & message)
    {
        ASSERT_EQ(write(master, message.data(), message.size()), static_cast<ssize_t>(message.size()));
    }

    void establishPosition()
    {
        send(gga(1));
        ASSERT_TRUE(waitFor([this] { return reader.getLatestData().data_valid; }));
    }

    SerialReader reader;
    int master = -1;
    std::string directory;
    std::string link;
    std::string slave_name;
};

TEST_F(SerialReaderTest, FreshStandalonePositionIsAvailable)
{
    establishPosition();
    const auto data = reader.getLatestData();
    EXPECT_TRUE(data.data_valid);
    EXPECT_NEAR(data.position.latitude, 29.4899014766667, 1e-10);
    EXPECT_NEAR(data.position.longitude, 106.567591068333, 1e-10);
    EXPECT_EQ(data.rtk_status, "GPS");
}

TEST_F(SerialReaderTest, PositionExpiresWithoutNewMeasurements)
{
    establishPosition();
    std::this_thread::sleep_for(milliseconds(1200));
    EXPECT_FALSE(reader.getLatestData().data_valid);
    EXPECT_EQ(reader.getStatus(), "NO DATA");
}

TEST_F(SerialReaderTest, HeadingMessagesDoNotRefreshPositionTimestamp)
{
    establishPosition();
    const auto stamp = reader.getLatestData().timestamp;
    send(sentence("GNHDT,123.4,T"));
    ASSERT_TRUE(waitFor([this] { return reader.getLatestData().head_angle_true == 123.4; }));
    EXPECT_EQ(reader.getLatestData().timestamp, stamp);
}

TEST_F(SerialReaderTest, HeadingTrafficDoesNotKeepStalePositionValid)
{
    establishPosition();
    const auto deadline = steady_clock::now() + milliseconds(1200);
    while (steady_clock::now() < deadline) {
        send(sentence("GNHDT,123.4,T"));
        std::this_thread::sleep_for(milliseconds(40));
    }
    EXPECT_FALSE(reader.getLatestData().data_valid);
}

TEST_F(SerialReaderTest, DisconnectImmediatelyInvalidatesCachedPosition)
{
    establishPosition();
    close(master);
    master = -1;
    EXPECT_TRUE(waitFor([this] { return !reader.isConnected(); }));
    EXPECT_FALSE(reader.getLatestData().data_valid);
}

TEST_F(SerialReaderTest, ReconnectsUsingStableDeviceLinkAfterPortChanges)
{
    establishPosition();
    const int old_master = master;
    master = createPort();
    ASSERT_GE(master, 0);
    ASSERT_EQ(unlink(link.c_str()), 0);
    ASSERT_EQ(symlink(slave_name.c_str(), link.c_str()), 0);
    close(old_master);
    const auto deadline = steady_clock::now() + seconds(7);
    bool received = false;
    while (steady_clock::now() < deadline) {
        const auto message = gga(4);
        if (write(master, message.data(), message.size()) < 0) {
            std::this_thread::sleep_for(milliseconds(50));
            continue;
        }
        const auto data = reader.getLatestData();
        if (data.data_valid && data.fix_quality == 4) {
            received = true;
            break;
        }
        std::this_thread::sleep_for(milliseconds(50));
    }
    EXPECT_TRUE(received);
}

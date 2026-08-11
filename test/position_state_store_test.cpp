#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>
#include <string>

#include <unistd.h>

#include "candle_ros2/position_state_store.hpp"

class PositionStateStoreTest : public ::testing::Test
{
  protected:
    void SetUp() override
    {
        directory = std::filesystem::temp_directory_path() /
                    ("candle_position_state_test_" + std::to_string(::getpid()) + "_" +
                     std::to_string(counter++));
        std::filesystem::create_directories(directory);
        path = directory / "state.json";
    }

    void TearDown() override
    {
        std::filesystem::remove_all(directory);
    }

    static inline int counter = 0;
    std::filesystem::path directory;
    std::filesystem::path path;
};

TEST_F(PositionStateStoreTest, RoundTripsVersionedDriveStateAtomically)
{
    PositionStateStore writer(path.string());
    writer.set(342,
               PersistedPositionState{
                   0.452,
                   -0.176,
                   0.63,
                   0.62831853,
                   0.63,
                   7,
                   123456789,
                   true,
               });

    std::string error;
    ASSERT_TRUE(writer.saveAtomic(&error)) << error;
    EXPECT_FALSE(std::filesystem::exists(path.string() + ".tmp"));

    PositionStateStore reader(path.string());
    ASSERT_TRUE(reader.load(&error)) << error;
    const auto restored = reader.get(342);
    ASSERT_TRUE(restored.has_value());
    EXPECT_DOUBLE_EQ(restored->logicalPosition, 0.452);
    EXPECT_DOUBLE_EQ(restored->rawPosition, -0.176);
    EXPECT_DOUBLE_EQ(restored->logicalTarget, 0.63);
    EXPECT_EQ(restored->calibrationGeneration, 7U);
    EXPECT_TRUE(restored->homed);
}

TEST_F(PositionStateStoreTest, RejectsCorruptStateWithoutReturningPartialData)
{
    {
        std::ofstream output(path);
        output << "{\"version\":1,\"drives\":{\"342\":";
    }

    PositionStateStore store(path.string());
    std::string error;
    EXPECT_FALSE(store.load(&error));
    EXPECT_FALSE(error.empty());
    EXPECT_FALSE(store.get(342).has_value());
}

TEST_F(PositionStateStoreTest, AtomicSaveReplacesCorruptDestination)
{
    {
        std::ofstream output(path);
        output << "corrupt";
    }

    PositionStateStore store(path.string());
    store.set(343,
              PersistedPositionState{
                  0.1, 0.1, 0.2, 0.62831853, 0.63, 1, 1000, true});
    std::string error;
    ASSERT_TRUE(store.saveAtomic(&error)) << error;

    PositionStateStore reloaded(path.string());
    ASSERT_TRUE(reloaded.load(&error)) << error;
    EXPECT_TRUE(reloaded.get(343).has_value());
}

TEST_F(PositionStateStoreTest, IgnoresAgeButRejectsConfigurationMismatch)
{
    PersistedPositionState state{
        0.1, 0.1, 0.2, 0.62831853, 0.63, 1, 100000, true};
    EXPECT_TRUE(isPersistedPositionStateCompatible(
        state, 0.62831853, 0.63));
    state.timestampUnixMs = 1;
    EXPECT_TRUE(isPersistedPositionStateCompatible(
        state, 0.62831853, 0.63));
    EXPECT_FALSE(isPersistedPositionStateCompatible(
        state, 0.70000000, 0.63));
    EXPECT_FALSE(isPersistedPositionStateCompatible(
        state, 0.62831853, 0.50));
}

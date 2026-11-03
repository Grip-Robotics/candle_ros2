#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <functional>
#include <future>
#include <thread>

#include "MD.hpp"
#include "candle.hpp"
#include "candle_frame_adapter.hpp"
#include "crc.hpp"
#include "pds_protocol.hpp"

namespace
{
using Bytes = std::vector<u8>;
using BusError = mab::I_CommunicationInterface::Error_t;
using AdapterError = mab::CANdleFrameAdapter::Error_t;
using MdError = mab::MD::Error_t;

void checksum(Bytes& bytes)
{
    const size_t offset = bytes.size() - sizeof(u32);
    const u32 crc = Crc::calcCrc(reinterpret_cast<const char*>(bytes.data()), offset);
    for (size_t i = 0; i < sizeof(u32); ++i)
        bytes[offset + i] = static_cast<u8>(crc >> (8 * i));
}

class ScriptedBus : public mab::I_CommunicationInterface
{
  public:
    std::function<std::pair<Bytes, BusError>(Bytes)> reply;
    std::atomic<size_t> requests{0};
    Error_t connect() override { return OK; }
    Error_t disconnect() override { return OK; }
    Error_t transfer(Bytes, u32) override { return OK; }
    std::pair<Bytes, Error_t> transfer(Bytes request, u32, size_t) override
    {
        ++requests;
        if (reply)
            return reply(std::move(request));
        return {{2, 1, 'r', 4, 5, 2}, OK};
    }
};

class MdTransport : public ::testing::Test
{
  protected:
    ScriptedBus* bus{};
    std::unique_ptr<mab::Candle> candle;
    void SetUp() override
    {
        Logger::g_m_verbosity = Logger::Verbosity_E::SILENT;
        auto connection = std::make_unique<ScriptedBus>();
        bus = connection.get();
        candle = std::make_unique<mab::Candle>(mab::CAN_DATARATE_1M, std::move(connection));
        ASSERT_EQ(candle->init(), mab::candleTypes::Error_t::OK);
    }
    void syncReply(Bytes payload, BusError error = BusError::OK)
    {
        bus->reply = [payload, error](Bytes) { return std::make_pair(payload, error); };
    }
    void asyncReply(Bytes payload)
    {
        bus->reply = [payload](Bytes packed)
        {
            for (u8 frame = 0; frame < packed[2]; ++frame)
            {
                const size_t offset = 3 + frame * mab::CANdleFrame::DTO_SIZE;
                packed[offset + 4] = static_cast<u8>(payload.size());
                std::copy(payload.begin(), payload.end(), packed.begin() + offset + 6);
            }
            checksum(packed);
            return std::make_pair(packed, BusError::OK);
        };
    }
};

TEST_F(MdTransport, RejectsTimeoutWithAcknowledgementBytes)
{
    syncReply({4, 1, 0xA0, 0}, BusError::TIMEOUT);
    mab::MD md(113, candle.get());
    EXPECT_EQ(md.disable(), MdError::TRANSFER_FAILED);
}

TEST_F(MdTransport, RejectsTruncatedSynchronousWritesWithoutThrowing)
{
    mab::MD md(113, candle.get());
    for (const Bytes reply : {Bytes{}, Bytes{4}, Bytes{4, 1}, Bytes{4, 1, 0xA0}})
    {
        syncReply(reply);
        EXPECT_EQ(md.disable(), MdError::TRANSFER_FAILED);
    }
}

TEST_F(MdTransport, AcceptsSupportedWriteHeaders)
{
    mab::MD md(113, candle.get());
    for (const u8 header : {0xA0, 0x42})
    {
        syncReply({4, 1, header, 0});
        EXPECT_EQ(md.disable(), MdError::OK);
    }
}

TEST_F(MdTransport, RejectsWrongWriteHeader)
{
    syncReply({4, 1, 0x41, 0});
    mab::MD md(113, candle.get());
    EXPECT_EQ(md.disable(), MdError::TRANSFER_FAILED);
}

TEST_F(MdTransport, RejectsTruncatedReadsAndPreservesCachedValues)
{
    mab::MD md(113, candle.get());
    mab::MDRegisters_S registers;
    registers.mainEncoderPosition = 1.25f;
    for (const Bytes reply : {Bytes{}, Bytes{4}, Bytes{4, 1}, Bytes{4, 1, 0x41}})
    {
        syncReply(reply);
        EXPECT_EQ(md.readRegisters(registers.mainEncoderPosition), MdError::TRANSFER_FAILED);
        EXPECT_FLOAT_EQ(registers.mainEncoderPosition.value, 1.25f);
    }
}

TEST_F(MdTransport, ValidatesEntireReadBeforeUpdatingAnyRegister)
{
    mab::MD md(113, candle.get());
    mab::MDRegisters_S registers;
    registers.mainEncoderPosition = 1.25f;
    registers.motorTorque = 2.5f;
    auto incoming = registers.mainEncoderPosition;
    incoming = 0.125f;
    const auto first = incoming.getSerializedRegister();
    Bytes reply{4, 1, 0x41, 0};
    reply.insert(reply.end(), first->begin(), first->end());
    reply.insert(reply.end(), {0xFF, 0xFF, 0, 0, 0, 0});
    syncReply(reply);
    EXPECT_EQ(md.readRegisters(registers.mainEncoderPosition, registers.motorTorque), MdError::TRANSFER_FAILED);
    EXPECT_FLOAT_EQ(registers.mainEncoderPosition.value, 1.25f);
    EXPECT_FLOAT_EQ(registers.motorTorque.value, 2.5f);
}

TEST_F(MdTransport, AcceptsLegacyAndCurrentReadHeaders)
{
    mab::MD md(113, candle.get());
    mab::MDRegisters_S registers;
    registers.mainEncoderPosition = 0.125f;
    const auto incoming = *registers.mainEncoderPosition.getSerializedRegister();
    for (const u8 header : {0, 0x41})
    {
        Bytes reply{4, 1, header, 0};
        reply.insert(reply.end(), incoming.begin(), incoming.end());
        syncReply(reply);
        registers.mainEncoderPosition = 1.25f;
        EXPECT_EQ(md.readRegisters(registers.mainEncoderPosition), MdError::OK);
        EXPECT_FLOAT_EQ(registers.mainEncoderPosition.value, 0.125f);
    }
}

TEST_F(MdTransport, AsyncWriteRequiresMdAcknowledgement)
{
    mab::MD md(113, candle.get());
    mab::MDRegisters_S registers;
    registers.state = 64;
    asyncReply({0x41, 0});
    EXPECT_EQ(md.writeRegistersAsync(registers.state).get(), MdError::TRANSFER_FAILED);
}

TEST_F(MdTransport, AsyncWriteAcceptsLegacyAcknowledgement)
{
    mab::MD md(113, candle.get());
    mab::MDRegisters_S registers;
    registers.state = 64;
    asyncReply({0xA0, 0});
    EXPECT_EQ(md.writeRegistersAsync(registers.state).get(), MdError::OK);
}

TEST_F(MdTransport, AsyncRegisterAccessRejectsWrongAccessBeforeTransfer)
{
    mab::MD md(113, candle.get());
    mab::MDRegisters_S registers;
    const size_t requests = bus->requests;
    EXPECT_EQ(md.writeRegistersAsync(registers.mainEncoderPosition).get(), MdError::REQUEST_INVALID);
    EXPECT_EQ(md.readRegistersAsync(registers.runZero).get(), MdError::REQUEST_INVALID);
    EXPECT_EQ(bus->requests, requests);
}

TEST_F(MdTransport, NullCandleAsyncRequestsFailWithoutDereferencing)
{
    mab::MD md(113, nullptr);
    mab::MDRegisters_S registers;
    EXPECT_EQ(md.writeRegistersAsync(registers.state).get(), MdError::REQUEST_INVALID);
    EXPECT_EQ(md.readRegistersAsync(registers.mainEncoderPosition).get(), MdError::REQUEST_INVALID);
}

TEST_F(MdTransport, TruncatedCandleResetFailsWithoutThrowing)
{
    syncReply({});
    EXPECT_NE(candle->reset(), mab::candleTypes::Error_t::OK);
}

TEST_F(MdTransport, SerializesConcurrentBusTransactions)
{
    std::atomic<int> active{0};
    std::atomic<bool> overlapped{false};
    bus->reply = [&](Bytes request)
    {
        if (++active != 1)
            overlapped = true;
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
        --active;
        if (request[0] == mab::CANdleFrame::DTO_PARSE_ID)
        {
            for (u8 frame = 0; frame < request[2]; ++frame)
            {
                const size_t offset = 3 + frame * mab::CANdleFrame::DTO_SIZE;
                request[offset + 4] = 2;
                request[offset + 6] = 0xA0;
                request[offset + 7] = 0;
            }
            checksum(request);
            return std::make_pair(request, BusError::OK);
        }
        return std::make_pair(Bytes{4, 1, 0xA0, 0}, BusError::OK);
    };
    mab::MD md(113, candle.get());
    mab::MDRegisters_S registers;
    registers.state = 64;
    auto asynchronous = md.writeRegistersAsync(registers.state);
    EXPECT_EQ(md.disable(), MdError::OK);
    EXPECT_EQ(asynchronous.get(), MdError::OK);
    EXPECT_FALSE(overlapped);
}

class PackedTransport : public ::testing::Test
{
  protected:
    std::shared_ptr<std::function<void()>> notify = std::make_shared<std::function<void()>>([] {});
    mab::CANdleFrameAdapter adapter{notify};
    std::pair<Bytes, AdapterError> exchange(const std::function<void(Bytes&)>& change)
    {
        auto response = std::async(std::launch::async, [&] { return adapter.accumulateFrame(113, {0xA0, 0}, 10); });
        const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(100);
        while (adapter.getCount() == 0 && std::chrono::steady_clock::now() < deadline)
            std::this_thread::yield();
        if (adapter.getCount() == 0)
        {
            ADD_FAILURE() << "Frame did not queue";
            return response.get();
        }
        auto packet = adapter.getPackedFrame();
        change(packet.first);
        adapter.parsePackedFrame(packet.first, packet.second.load());
        return response.get();
    }
};

TEST_F(PackedTransport, GoodPacketReturnsRequestedPayload)
{
    auto result = exchange([](Bytes&) {});
    EXPECT_EQ(result.second, AdapterError::OK);
    EXPECT_EQ(result.first, (Bytes{0xA0, 0}));
}

TEST_F(PackedTransport, EveryTruncatedPacketFailsWithoutOutOfBoundsAccess)
{
    const size_t expected = 3 + mab::CANdleFrame::DTO_SIZE + 4;
    for (size_t length = 0; length < expected; ++length)
    {
        const auto result = exchange([length](Bytes& bytes) { bytes.resize(length); });
        EXPECT_NE(result.second, AdapterError::OK) << length;
        EXPECT_TRUE(result.first.empty());
    }
}

TEST_F(PackedTransport, CorruptChecksumFailsTheWaitingRequest)
{
    auto result = exchange([](Bytes& bytes) { bytes.back() ^= 1; });
    EXPECT_EQ(result.second, AdapterError::INVALID_BUS_FRAME);
}

TEST_F(PackedTransport, SequenceZeroFailsWithoutUnderflow)
{
    auto result = exchange([](Bytes& bytes) { bytes[8] = 0; checksum(bytes); });
    EXPECT_EQ(result.second, AdapterError::INVALID_CANDLE_FRAME);
}

TEST_F(PackedTransport, WrongCanIdCannotSatisfyAnotherDriveRequest)
{
    auto result = exchange([](Bytes& bytes) { bytes[3] = 111; checksum(bytes); });
    EXPECT_EQ(result.second, AdapterError::INVALID_CANDLE_FRAME);
}

TEST_F(PackedTransport, OversizedInnerPayloadCannotOverflowFrameStorage)
{
    auto result = exchange([](Bytes& bytes) { bytes[7] = 255; checksum(bytes); });
    EXPECT_EQ(result.second, AdapterError::INVALID_CANDLE_FRAME);
}

TEST_F(PackedTransport, OversizedRequestsCannotWrapTheirLengthOrExhaustSlots)
{
    for (size_t attempt = 0; attempt < 2 * mab::CANdleFrameAdapter::FRAME_BUFFER_SIZE; ++attempt)
        EXPECT_EQ(adapter.accumulateFrame(113, Bytes(256, 1), 10).second, AdapterError::INVALID_BUS_FRAME);
    EXPECT_EQ(adapter.getCount(), 0);
    EXPECT_EQ(exchange([](Bytes&) {}).second, AdapterError::OK);
}

TEST_F(PackedTransport, EmptyOrBroadcastRequestsAreRejected)
{
    EXPECT_EQ(adapter.accumulateFrame(113, {}, 10).second, AdapterError::INVALID_BUS_FRAME);
    EXPECT_EQ(adapter.accumulateFrame(0, {1}, 10).second, AdapterError::INVALID_BUS_FRAME);
    EXPECT_EQ(adapter.getCount(), 0);
}
TEST_F(PackedTransport, SevenConcurrentRepliesStayAssociatedWithTheirDrives)
{
    std::vector<std::future<std::pair<Bytes, AdapterError>>> results;
    for (u16 id = 111; id < 118; ++id)
        results.emplace_back(std::async(std::launch::async, [&, id]
            { return adapter.accumulateFrame(id, {static_cast<u8>(id), 0}, 10); }));
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(100);
    while (adapter.getCount() != 7 && std::chrono::steady_clock::now() < deadline)
        std::this_thread::yield();
    ASSERT_EQ(adapter.getCount(), 7);
    auto packet = adapter.getPackedFrame();
    EXPECT_EQ(adapter.parsePackedFrame(packet.first, packet.second.load()), AdapterError::OK);
    for (size_t index = 0; index < results.size(); ++index)
    {
        const auto result = results[index].get();
        EXPECT_EQ(result.second, AdapterError::OK);
        EXPECT_EQ(result.first, (Bytes{static_cast<u8>(111 + index), 0}));
    }
}

TEST_F(PackedTransport, DuplicateSequenceCannotAcknowledgeTwoRequests)
{
    auto first = std::async(std::launch::async, [&] { return adapter.accumulateFrame(111, {1}, 10); });
    auto second = std::async(std::launch::async, [&] { return adapter.accumulateFrame(113, {2}, 10); });
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(100);
    while (adapter.getCount() != 2 && std::chrono::steady_clock::now() < deadline)
        std::this_thread::yield();
    ASSERT_EQ(adapter.getCount(), 2);
    auto packet = adapter.getPackedFrame();
    packet.first[8 + mab::CANdleFrame::DTO_SIZE] = packet.first[8];
    checksum(packet.first);
    EXPECT_EQ(adapter.parsePackedFrame(packet.first, packet.second.load()), AdapterError::INVALID_CANDLE_FRAME);
    EXPECT_NE(first.get().second, AdapterError::OK);
    EXPECT_NE(second.get().second, AdapterError::OK);
}

TEST(PdsTransport, RejectsEveryTruncatedPropertyReply)
{
    mab::PropertyGetMessage message(mab::moduleType_E::POWER_STAGE, mab::socketIndex_E::SOCKET_1);
    message.addProperty(mab::propertyId_E::BUS_VOLTAGE);
    Bytes valid{0, 1, 0, 0xE0, 0x5B, 0, 0};
    for (size_t length = 0; length < valid.size(); ++length)
        EXPECT_NE(message.parseResponse(valid.data(), length), mab::PdsMessage::error_E::OK) << length;
    EXPECT_EQ(message.parseResponse(valid.data(), valid.size()), mab::PdsMessage::error_E::OK);
    u32 voltage = 0;
    EXPECT_EQ(message.getProperty(mab::propertyId_E::BUS_VOLTAGE, &voltage), mab::PdsMessage::error_E::OK);
    EXPECT_EQ(voltage, 23520u);
}

TEST(PdsTransport, FailedPropertyReadDoesNotLeaveStaleOrPartialResults)
{
    mab::PropertyGetMessage message(mab::moduleType_E::BRAKE_RESISTOR, mab::socketIndex_E::SOCKET_1);
    message.addProperty(mab::propertyId_E::ENABLE);
    Bytes valid{0, 1, 0, 1};
    ASSERT_EQ(message.parseResponse(valid.data(), valid.size()), mab::PdsMessage::error_E::OK);
    u32 enabled = 0;
    ASSERT_EQ(message.getProperty(mab::propertyId_E::ENABLE, &enabled), mab::PdsMessage::error_E::OK);
    Bytes failed{0, 1, 1, 1};
    EXPECT_EQ(message.parseResponse(failed.data(), failed.size()), mab::PdsMessage::error_E::RESPONSE_STATUS_ERROR);
    EXPECT_NE(message.getProperty(mab::propertyId_E::ENABLE, &enabled), mab::PdsMessage::error_E::OK);
}

}

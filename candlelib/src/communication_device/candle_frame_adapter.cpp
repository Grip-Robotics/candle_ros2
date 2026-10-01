#include "candle_frame_adapter.hpp"
#include "algorithm"
#include "chrono"
#include "crc.hpp"
namespace mab
{
    std::pair<std::vector<u8>, CANdleFrameAdapter::Error_t> CANdleFrameAdapter::accumulateFrame(
        const canId_t canId, const std::vector<u8>& data, const u16 timeout100us)
    {
        // Validate before narrowing the payload length or consuming a slot.
        if (canId == 0 || data.empty() || data.size() > CANdleFrame::DATA_MAX_LENGTH)
            return {{}, Error_t::INVALID_BUS_FRAME};
        auto success = m_sem.try_acquire_for(READER_TIMEOUT);
        if (!success)
            return {{}, Error_t::READER_TIMEOUT};

        std::unique_lock lock(m_mutex);
        const size_t seqIdx = m_count;
        CANdleFrame cf;
        u8 buf[CANdleFrame::DTO_SIZE] = {0};
        cf.init(canId, static_cast<u8>(seqIdx + 1), timeout100us);
        if (cf.addData(data.data(), static_cast<u8>(data.size())) != CANdleFrame::Error_t::OK)
        {
            m_sem.release();
            return {{}, Error_t::INVALID_BUS_FRAME};
        }
        ++m_count;
        cf.serialize(buf);
        const u64 thisFrameIdx = m_frameIndex;
        m_packedFrames[thisFrameIdx].insert(
            m_packedFrames[thisFrameIdx].end(), buf, buf + cf.DTO_SIZE);
        auto& entry = m_pendingBatches[thisFrameIdx];
        if (!entry)
            entry = std::make_shared<PendingBatch>();
        auto batch = entry;
        batch->targetIds[seqIdx] = canId;
        ++batch->readers;
        ++batch->count;
        if (auto func = m_requestTransfer.lock())
            (*func)();

        const bool received = batch->ready.wait_for(lock, READER_TIMEOUT,
                                                    [&] { return batch->received; });
        auto result = received ? std::move(batch->results[seqIdx])
                               : std::make_pair(std::vector<u8>{}, Error_t::READER_TIMEOUT);
        if (--batch->readers == 0 && batch->received)
            m_pendingBatches.erase(thisFrameIdx);
        return result;
    }

    std::pair<std::vector<u8>, std::atomic<u64>> CANdleFrameAdapter::getPackedFrame()
    {
        std::unique_lock lock(m_mutex);

        auto m_packedFrameIt = m_packedFrames.find(m_frameIndex);

        if (m_packedFrameIt == m_packedFrames.end())
        {
            m_log.warn("Expected frame is empty!");
        }

        m_log.debug("Sending frame with idx: %u", m_frameIndex.load());
        std::vector<u8> packedFrame = m_packedFrames[m_frameIndex];
        m_packedFrames.erase(m_frameIndex++);
        u8 count                     = m_count;
        m_count                      = 0;
        m_packedFrames[m_frameIndex] = std::vector<u8>({CANdleFrame::DTO_PARSE_ID, 0x1, 0x0});
        m_sem.release(count);

        lock.unlock();

        auto countIter      = packedFrame.begin() + 2 /*PARSE_ID + ACK*/;
        *countIter          = count;
        u32 calculatedCRC32 = Crc::calcCrc((const char*)packedFrame.data(), packedFrame.size());
        packedFrame.push_back(calculatedCRC32);
        packedFrame.push_back(calculatedCRC32 >> 8);
        packedFrame.push_back(calculatedCRC32 >> 16);
        packedFrame.push_back(calculatedCRC32 >> 24);
        return std::make_pair(packedFrame, m_frameIndex - 1);
    }

    CANdleFrameAdapter::Error_t CANdleFrameAdapter::parsePackedFrame(
        const std::vector<u8>& packedFrames, std::atomic<u64> idx)
    {
        std::unique_lock lock(m_mutex);

        const auto found = m_pendingBatches.find(idx);
        if (found == m_pendingBatches.end())
            return Error_t::FRAME_LOST;
        auto batch = found->second;
        const auto finish = [&](Error_t error)
        {
            batch->received = true;
            batch->ready.notify_all();
            if (batch->readers == 0)
                m_pendingBatches.erase(idx);
            return error;
        };
        for (auto& result : batch->results)
            result = {{}, Error_t::INVALID_BUS_FRAME};

        constexpr size_t headerSize = 3;
        constexpr size_t checksumSize = sizeof(u32);
        if (packedFrames.size() < headerSize + checksumSize ||
            packedFrames[0] != CANdleFrame::DTO_PARSE_ID || packedFrames[1] != 1)
            return finish(Error_t::INVALID_BUS_FRAME);
        const u8 count = packedFrames[2];
        const size_t expectedSize = headerSize + count * CANdleFrame::DTO_SIZE + checksumSize;
        if (count > FRAME_BUFFER_SIZE || count != batch->count || packedFrames.size() != expectedSize)
            return finish(Error_t::INVALID_BUS_FRAME);

        const size_t checksumOffset = expectedSize - checksumSize;
        const u32 readCRC32 = static_cast<u32>(packedFrames[checksumOffset]) |
            (static_cast<u32>(packedFrames[checksumOffset + 1]) << 8) |
            (static_cast<u32>(packedFrames[checksumOffset + 2]) << 16) |
            (static_cast<u32>(packedFrames[checksumOffset + 3]) << 24);
        if (readCRC32 != Crc::calcCrc(reinterpret_cast<const char*>(packedFrames.data()), checksumOffset))
            return finish(Error_t::INVALID_BUS_FRAME);

        std::array<bool, FRAME_BUFFER_SIZE> seen{};
        Error_t error = Error_t::OK;
        for (size_t frame = 0; frame < count; ++frame)
        {
            CANdleFrame cf;
            cf.deserialize(packedFrames.data() + headerSize + frame * CANdleFrame::DTO_SIZE);
            if (cf.sequenceNo() == 0 || cf.sequenceNo() > count)
            {
                for (auto& result : batch->results)
                    result = {{}, Error_t::INVALID_CANDLE_FRAME};
                return finish(Error_t::INVALID_CANDLE_FRAME);
            }
            const size_t slot = cf.sequenceNo() - 1;
            if (seen[slot])
            {
                for (auto& result : batch->results)
                    result = {{}, Error_t::INVALID_CANDLE_FRAME};
                return finish(Error_t::INVALID_CANDLE_FRAME);
            }
            seen[slot] = true;
            if (!cf.isValid() || cf.canId() != batch->targetIds[slot])
            {
                batch->results[slot] = {{}, Error_t::INVALID_CANDLE_FRAME};
                error = Error_t::INVALID_CANDLE_FRAME;
                continue;
            }
            batch->results[slot] = {std::vector<u8>(cf.data(), cf.data() + cf.length()), Error_t::OK};
        }
        // Expired callers can leave entries behind when the USB transport never
        // produced a response. They have no live waiters and can be discarded.
        for (auto it = m_pendingBatches.begin(); it != m_pendingBatches.end();)
        {
            if (it->first != idx && it->second->readers == 0 &&
                it->first + DEPRECATION_FRAME_COUNT < m_frameIndex)
                it = m_pendingBatches.erase(it);
            else
                ++it;
        }
        return finish(error);
    }
}  // namespace mab

#pragma once
#include "mab_def.hpp"
#include "mab_types.hpp"
#include <cstring>
#include <type_traits>
namespace mab
{
    /// @brief DTO struct to be sent via CANdle buses
    struct CANdleFrameDTO
    {
        canId_t canId          = 0;
        u16     timeout        = 0;  // 100 us
        u8      length         = 0;  // data length in bytes
        u8      sequenceNumber = 0;  // start at 1 for the first frame in the package
                                     // and increment by 1 for each frame
        u8 data[64] = {0};
    };

    /// @brief Helper for dealing with CANdle frame DTO
    struct CANdleFrame
    {
      private:
        CANdleFrameDTO frameDTO = {0};

      public:
        static constexpr u8     DTO_PARSE_ID    = 15;
        static constexpr u8     DATA_MAX_LENGTH = 64;
        static constexpr size_t DTO_SIZE =
            sizeof(CANdleFrameDTO::canId) + sizeof(CANdleFrameDTO::timeout) +
            sizeof(CANdleFrameDTO::length) + sizeof(CANdleFrameDTO::sequenceNumber) +
            (sizeof(CANdleFrameDTO::data) / sizeof(*CANdleFrameDTO::data));

        enum class Error_t
        {
            UNKNOWN,
            OK,
            MSG_TOO_LONG
        };
        inline void clear()
        {
            frameDTO = {0};
        }
        inline void init(canId_t                           canId,
                         decltype(frameDTO.sequenceNumber) sequenceNumber,
                         decltype(frameDTO.timeout)        timeout)
        {
            clear();
            frameDTO.canId          = canId;
            frameDTO.sequenceNumber = sequenceNumber;
            frameDTO.timeout        = timeout;
        }
        inline bool isValid() const
        {
            return frameDTO.canId != 0 && frameDTO.length != 0 &&
                   frameDTO.length <= sizeof(frameDTO.data);
        }

        inline bool isValid(decltype(frameDTO.sequenceNumber) expectedSeqNum) const
        {
            return frameDTO.canId != 0 && frameDTO.length != 0 &&
                   frameDTO.length <= sizeof(frameDTO.data) &&
                   frameDTO.sequenceNumber == expectedSeqNum;
        }

        inline void serialize(void* buffer) const
        {
            std::memcpy(buffer, &frameDTO.canId, sizeof(frameDTO.canId));
            buffer                                      = (u8*)buffer + sizeof(frameDTO.canId);
            std::memcpy(buffer, &frameDTO.timeout, sizeof(frameDTO.timeout));
            buffer                                      = (u8*)buffer + sizeof(frameDTO.timeout);
            std::memcpy(buffer, &frameDTO.length, sizeof(frameDTO.length));
            buffer                                      = (u8*)buffer + sizeof(frameDTO.length);
            std::memcpy(buffer, &frameDTO.sequenceNumber, sizeof(frameDTO.sequenceNumber));
            buffer = (u8*)buffer + sizeof(frameDTO.sequenceNumber);
            std::memcpy(buffer, frameDTO.data, frameDTO.length);
        }
        inline void deserialize(const void* buffer)
        {
            clear();
            std::memcpy(&frameDTO.canId, buffer, sizeof(frameDTO.canId));
            buffer                  = (u8*)buffer + sizeof(frameDTO.canId);
            std::memcpy(&frameDTO.timeout, buffer, sizeof(frameDTO.timeout));
            buffer                  = (u8*)buffer + sizeof(frameDTO.timeout);
            std::memcpy(&frameDTO.length, buffer, sizeof(frameDTO.length));
            buffer                  = (u8*)buffer + sizeof(frameDTO.length);
            std::memcpy(&frameDTO.sequenceNumber, buffer, sizeof(frameDTO.sequenceNumber));
            buffer                  = (u8*)buffer + sizeof(frameDTO.sequenceNumber);
            // Preserve the invalid length for isValid(), but never copy beyond
            // the fixed CAN payload before the caller can validate it.
            if (frameDTO.length <= sizeof(frameDTO.data))
                std::memcpy(frameDTO.data, buffer, frameDTO.length);
        }
        inline Error_t addData(const void* data, decltype(frameDTO.length) size)
        {
            if (frameDTO.length + size > sizeof(frameDTO.data))
                return Error_t::MSG_TOO_LONG;
            std::memcpy(frameDTO.data + frameDTO.length, data, size);
            frameDTO.length += size;
            return Error_t::OK;
        }
        inline void clearData()
        {
            std::memset(frameDTO.data, 0, sizeof(frameDTO.data));
            frameDTO.length = 0;
        }
        inline const std::remove_cvref_t<decltype(frameDTO.data[0])>* data() const
        {
            return &frameDTO.data[0];
        }
        inline decltype(frameDTO.sequenceNumber) sequenceNo() const
        {
            return frameDTO.sequenceNumber;
        }
        inline decltype(frameDTO.canId) canId() const
        {
            return frameDTO.canId;
        }
        inline decltype(frameDTO.length) length() const
        {
            return frameDTO.length;
        }
        inline decltype(frameDTO.timeout) timeout() const
        {
            return frameDTO.timeout;
        }
    };
}  // namespace mab

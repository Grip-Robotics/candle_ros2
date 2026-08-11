#include "candle_ros2/position_state_store.hpp"

#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <limits>
#include <stdexcept>
#include <system_error>
#include <utility>

#include <fcntl.h>
#include <nlohmann/json.hpp>
#include <unistd.h>

using json = nlohmann::json;

PositionStateStore::PositionStateStore(std::string path) : m_path(expandUserPath(std::move(path)))
{
}

bool PositionStateStore::load(std::string* error)
{
    m_states.clear();
    if (!std::filesystem::exists(m_path))
        return true;

    try
    {
        std::ifstream input(m_path);
        if (!input)
            throw std::runtime_error("could not open file");

        const json root = json::parse(input);
        if (root.at("version").get<int>() != FORMAT_VERSION)
            throw std::runtime_error("unsupported format version");

        for (const auto& [idText, value] : root.at("drives").items())
        {
            const auto parsedId = std::stoul(idText);
            if (parsedId > std::numeric_limits<std::uint16_t>::max())
                throw std::runtime_error("CAN ID is out of range");

            PersistedPositionState state;
            state.logicalPosition      = value.at("logical_position").get<double>();
            state.rawPosition          = value.at("raw_position").get<double>();
            state.logicalTarget        = value.at("logical_target").get<double>();
            state.wrapPeriodRad        = value.at("wrap_period_rad").get<double>();
            state.travelRad            = value.at("travel_rad").get<double>();
            state.calibrationGeneration =
                value.at("calibration_generation").get<std::uint64_t>();
            state.timestampUnixMs = value.at("timestamp_unix_ms").get<std::int64_t>();
            state.homed           = value.at("homed").get<bool>();
            m_states.emplace(static_cast<std::uint16_t>(parsedId), state);
        }
        return true;
    }
    catch (const std::exception& exception)
    {
        if (error)
            *error = exception.what();
        m_states.clear();
        return false;
    }
}

bool PositionStateStore::saveAtomic(std::string* error) const
{
    try
    {
        const std::filesystem::path destination(m_path);
        if (destination.has_parent_path())
            std::filesystem::create_directories(destination.parent_path());

        json drives = json::object();
        for (const auto& [id, state] : m_states)
        {
            drives[std::to_string(id)] = {
                {"logical_position", state.logicalPosition},
                {"raw_position", state.rawPosition},
                {"logical_target", state.logicalTarget},
                {"wrap_period_rad", state.wrapPeriodRad},
                {"travel_rad", state.travelRad},
                {"calibration_generation", state.calibrationGeneration},
                {"timestamp_unix_ms", state.timestampUnixMs},
                {"homed", state.homed},
            };
        }

        const json root = {{"version", FORMAT_VERSION}, {"drives", drives}};
        const std::filesystem::path temporary = destination.string() + ".tmp";
        {
            std::ofstream output(temporary, std::ios::trunc);
            if (!output)
                throw std::runtime_error("could not open temporary file");
            output << root.dump(2) << '\n';
            output.flush();
            if (!output)
                throw std::runtime_error("could not flush temporary file");
        }

        const int fileDescriptor = ::open(temporary.c_str(), O_RDONLY);
        if (fileDescriptor < 0)
            throw std::runtime_error("could not open temporary file for sync");
        const int syncResult = ::fsync(fileDescriptor);
        ::close(fileDescriptor);
        if (syncResult != 0)
            throw std::runtime_error("could not sync temporary file");

        std::error_code renameError;
        std::filesystem::rename(temporary, destination, renameError);
        if (renameError)
        {
            std::filesystem::remove(destination, renameError);
            renameError.clear();
            std::filesystem::rename(temporary, destination, renameError);
        }
        if (renameError)
            throw std::runtime_error("could not atomically replace state file");

        if (destination.has_parent_path())
        {
            const int directoryDescriptor =
                ::open(destination.parent_path().c_str(), O_RDONLY | O_DIRECTORY);
            if (directoryDescriptor >= 0)
            {
                ::fsync(directoryDescriptor);
                ::close(directoryDescriptor);
            }
        }
        return true;
    }
    catch (const std::exception& exception)
    {
        if (error)
            *error = exception.what();
        return false;
    }
}

void PositionStateStore::set(std::uint16_t id, const PersistedPositionState& state)
{
    m_states[id] = state;
}

std::optional<PersistedPositionState> PositionStateStore::get(std::uint16_t id) const
{
    const auto state = m_states.find(id);
    if (state == m_states.end())
        return std::nullopt;
    return state->second;
}

void PositionStateStore::erase(std::uint16_t id)
{
    m_states.erase(id);
}

const std::string& PositionStateStore::path() const
{
    return m_path;
}

std::string PositionStateStore::expandUserPath(std::string path) const
{
    if (path == "~" || path.rfind("~/", 0) == 0)
    {
        const char* home = std::getenv("HOME");
        if (home)
            path.replace(0, 1, home);
    }
    return path;
}

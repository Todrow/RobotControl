#include "systems/rpi/link/socket_utils.h"

#include "systems/rpi/link/map_link.h"

#include <chrono>
#include <cstdio>
#include <vector>

#include "map_protocol.h"

namespace {

// Twice a second. The map is for the operator's eyes, not for control, and a
// 400x400 grid does not change meaningfully faster than that.
constexpr auto kMapPeriod = std::chrono::milliseconds(500);
// How long to wait on an incoming mode byte before going back to sending. Short
// enough that a button press is acted on within one frame.
constexpr auto kPollTimeout = std::chrono::milliseconds(20);

// Grid -> the three-state cells the wire format carries. The log-odds detail is
// deliberately dropped: the operator needs to see wall / floor / unseen, and
// sending 160 KB of signed bytes instead of a few KB of runs would not tell them
// anything more.
std::vector<uint8_t> flatten(const OccupancyGrid& grid) {
    std::vector<uint8_t> cells;
    cells.reserve(static_cast<size_t>(OccupancyGrid::kCells) * OccupancyGrid::kCells);
    for (int cy = 0; cy < OccupancyGrid::kCells; ++cy) {
        for (int cx = 0; cx < OccupancyGrid::kCells; ++cx) {
            switch (grid.cell(cx, cy)) {
                case Cell::Free: cells.push_back(mapproto::CELL_FREE); break;
                case Cell::Occupied: cells.push_back(mapproto::CELL_OCCUPIED); break;
                case Cell::Unknown: cells.push_back(mapproto::CELL_UNKNOWN); break;
            }
        }
    }
    return cells;
}

std::vector<uint8_t> buildFrame(const MapSnapshot& snapshot, bool exploring) {
    const std::vector<mapproto::CellRun> runs = mapproto::encodeRuns(flatten(snapshot.grid));

    mapproto::MapHeader header{};
    header.pose_x = snapshot.pose.x;
    header.pose_y = snapshot.pose.y;
    header.pose_theta = snapshot.pose.theta;
    header.match_score = snapshot.match_score;
    header.resolution = OccupancyGrid::kResolution;
    header.cells = static_cast<uint16_t>(OccupancyGrid::kCells);
    header.runs = static_cast<uint32_t>(runs.size());
    header.exploring = exploring ? 1 : 0;

    const size_t payload = sizeof(header) + runs.size() * sizeof(mapproto::CellRun);
    mapproto::FrameHeader frame{mapproto::MSG_MAP, static_cast<uint32_t>(payload)};

    std::vector<uint8_t> bytes;
    bytes.resize(sizeof(frame) + payload);
    size_t offset = 0;
    std::memcpy(bytes.data() + offset, &frame, sizeof(frame));
    offset += sizeof(frame);
    std::memcpy(bytes.data() + offset, &header, sizeof(header));
    offset += sizeof(header);
    if (!runs.empty())
        std::memcpy(bytes.data() + offset, runs.data(), runs.size() * sizeof(mapproto::CellRun));
    return bytes;
}

}  // namespace

void runMapLink(const RobotOptions& options, RobotState& state) {
    net::Socket server = net::listenOn(options.map_port);
    if (!server) {
        std::fprintf(stderr, "[map] cannot listen on %u (socket error %d)\n",
                     static_cast<unsigned>(options.map_port), net::lastError());
        state.abort();
        return;
    }
    std::printf("[map] autonomy channel on %u; map twice a second\n",
                static_cast<unsigned>(options.map_port));

    while (state.running.load()) {
        net::Socket client;
        std::string peer;
        const auto accepted =
            net::acceptClient(server.get(), client, peer,
                              net::Clock::now() + std::chrono::milliseconds(200), state.running);
        if (accepted == net::IoResult::Timeout) continue;
        if (accepted == net::IoResult::Stopped) break;
        if (accepted != net::IoResult::Ok) {
            std::fprintf(stderr, "[map] accept failed (socket error %d)\n", net::lastError());
            state.abort();
            break;
        }
        std::printf("[map] operator connected: %s\n", peer.c_str());

        auto next_map = net::Clock::now();
        while (state.running.load()) {
            // Mode requests first: a button press should not wait behind a map.
            mapproto::FrameHeader header{};
            const auto received = net::recvExact(client.get(), &header, sizeof(header),
                                                 net::Clock::now() + kPollTimeout, state.running);
            if (received == net::IoResult::Closed || received == net::IoResult::Error) break;
            if (received == net::IoResult::Stopped) break;
            if (received == net::IoResult::Ok) {
                if (header.type != mapproto::MSG_MODE || header.length != 1) {
                    std::fprintf(stderr, "[map] unexpected frame type %u length %u; closing\n",
                                 static_cast<unsigned>(header.type),
                                 static_cast<unsigned>(header.length));
                    break;
                }
                uint8_t wanted = 0;
                const auto body =
                    net::recvExact(client.get(), &wanted, sizeof(wanted),
                                   net::Clock::now() + std::chrono::seconds(1), state.running);
                if (body != net::IoResult::Ok) break;
                // A request, not a decision: the control loop reads this and
                // decides, so the operator taking the stick still wins.
                state.explore_requested.store(wanted != 0);
                std::printf("[map] operator requested %s\n", wanted ? "EXPLORE" : "MANUAL");
            }

            if (net::Clock::now() < next_map) continue;
            next_map = net::Clock::now() + kMapPeriod;

            const auto snapshot = state.map.snapshot();
            if (!snapshot) continue;  // SLAM has not produced a map yet.
            const std::vector<uint8_t> bytes =
                buildFrame(*snapshot, state.mode.load() == ControlMode::Explore);
            const auto sent =
                net::sendExact(client.get(), bytes.data(), bytes.size(),
                               net::Clock::now() + std::chrono::seconds(2), state.running);
            if (sent != net::IoResult::Ok) {
                if (sent == net::IoResult::Timeout)
                    std::fprintf(stderr, "[map] could not send a full map within 2 s\n");
                break;
            }
        }
        std::printf("[map] operator disconnected: %s%s\n", peer.c_str(),
                    state.mode.load() == ControlMode::Explore ? " (still exploring)" : "");
    }
}

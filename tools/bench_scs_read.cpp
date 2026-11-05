// 读取 SCS 舵机反馈的耗时基准。
//
// 复刻 FeetechServoHardwareInterface::read() 的线上行为：对每个 id 顺序执行一次
// READ 0x38..0x46（15 字节），不做 SYNC READ（SCS 协议没有定义）。只读，不使能力矩，
// 不写 EEPROM，因此在机器人上运行不会让关节动作。
//
// 用法：
//   bench_scs_read --port /dev/ttyUSB0 --ids 1-10
//   bench_scs_read --port /dev/ttyUSB0 --scan
//   bench_scs_read --port /dev/ttyUSB0 --ids 1,2,3 --iters 500 --json
//
// 输出每个周期的 min/median/p95/max，以及逐关节平均耗时。真实硬件的数字才有意义；
// pty 模拟器只能验证工具本身。

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <numeric>
#include <optional>
#include <sstream>
#include <string>
#include <string_view>
#include <vector>

#include "feetech_servo_ros2_control/scs_protocol.hpp"
#include "feetech_servo_ros2_control/scs_register_map.hpp"
#include "feetech_servo_ros2_control/scs_serial_port.hpp"

namespace {

    struct Options {
        std::string port;
        int baud_rate = 1'000'000;
        std::vector<uint8_t> ids;
        int iterations = 200;
        int warmup = 20;
        int timeout_ms = 10;
        bool scan = false;
        bool json = false;
    };

    void usage(const char* argv0) {
        std::cerr << "usage: " << argv0 << " --port DEVICE [--ids 1-10 | --scan]\n"
            << "       [--baud 1000000] [--iters 200] [--warmup 20]\n"
            << "       [--timeout-ms 10] [--json]\n";
    }

    std::optional<std::vector<uint8_t>> parse_ids(const std::string& text) {
        std::vector<uint8_t> ids;
        std::stringstream stream(text);
        std::string token;
        while (std::getline(stream, token, ',')) {
            if (token.empty()) {
                return std::nullopt;
            }
            const auto dash = token.find('-');
            int first = 0;
            int last = 0;
            try {
                if (dash == std::string::npos) {
                    first = last = std::stoi(token);
                }
                else {
                    first = std::stoi(token.substr(0, dash));
                    last = std::stoi(token.substr(dash + 1));
                }
            }
            catch (const std::exception&) {
                return std::nullopt;
            }
            if (first < 0 || last < first || last > 253) {
                return std::nullopt;
            }
            for (int id = first; id <= last; ++id) {
                ids.push_back(static_cast<uint8_t>(id));
            }
        }
        return ids.empty() ? std::nullopt : std::optional<std::vector<uint8_t>>(std::move(ids));
    }

    bool parse_args(const int argc, char** argv, Options& options) {
        for (int i = 1; i < argc; ++i) {
            const std::string_view arg = argv[i];
            const auto value = [&]() -> std::string_view {
                if (i + 1 >= argc) {
                    std::cerr << "missing value for " << arg << '\n';
                    std::exit(2);
                }
                return argv[++i];
            };
            if (arg == "--port") {
                options.port = value();
            }
            else if (arg == "--baud") {
                options.baud_rate = std::stoi(std::string(value()));
            }
            else if (arg == "--ids") {
                auto parsed = parse_ids(std::string(value()));
                if (!parsed) {
                    std::cerr << "invalid --ids (example: 1-10 or 1,2,3)\n";
                    return false;
                }
                options.ids = std::move(*parsed);
            }
            else if (arg == "--iters") {
                options.iterations = std::stoi(std::string(value()));
            }
            else if (arg == "--warmup") {
                options.warmup = std::stoi(std::string(value()));
            }
            else if (arg == "--timeout-ms") {
                options.timeout_ms = std::stoi(std::string(value()));
            }
            else if (arg == "--scan") {
                options.scan = true;
            }
            else if (arg == "--json") {
                options.json = true;
            }
            else if (arg == "--help" || arg == "-h") {
                usage(argv[0]);
                std::exit(0);
            }
            else {
                std::cerr << "unknown argument: " << arg << '\n';
                usage(argv[0]);
                return false;
            }
        }
        if (options.port.empty()) {
            std::cerr << "--port is required\n";
            return false;
        }
        if (!options.scan && options.ids.empty()) {
            std::cerr << "either --ids or --scan is required\n";
            return false;
        }
        if (options.iterations <= 0 || options.warmup < 0 || options.timeout_ms <= 0) {
            std::cerr << "--iters and --timeout-ms must be positive, --warmup non-negative\n";
            return false;
        }
        return true;
    }

    struct Stats {
        double min_ms = 0.0;
        double median_ms = 0.0;
        double p95_ms = 0.0;
        double max_ms = 0.0;
        double mean_ms = 0.0;
    };

    Stats summarize(std::vector<double> samples) {
        std::ranges::sort(samples);
        const auto at = [&](const double fraction) {
            if (samples.empty()) {
                return 0.0;
            }
            const auto index = static_cast<size_t>(std::llround(fraction * static_cast<double>(samples.size() - 1)));
            return samples[std::min(index, samples.size() - 1)];
        };
        Stats stats;
        stats.min_ms = samples.front();
        stats.median_ms = at(0.50);
        stats.p95_ms = at(0.95);
        stats.max_ms = samples.back();
        stats.mean_ms = std::accumulate(samples.begin(), samples.end(), 0.0) /
            static_cast<double>(samples.size());
        return stats;
    }

    std::vector<uint8_t> scan_ids(const feetech_servo_ros2_control::ScsProtocol& protocol) {
        std::vector<uint8_t> found;
        for (int id = 0; id <= 253; ++id) {
            if (protocol.ping(static_cast<uint8_t>(id))) {
                found.push_back(static_cast<uint8_t>(id));
            }
        }
        return found;
    }

} // namespace

int main(int argc, char** argv) {
    using namespace feetech_servo_ros2_control;
    using Clock = std::chrono::steady_clock;

    Options options;
    if (!parse_args(argc, argv, options)) {
        usage(argv[0]);
        return 2;
    }

    ScsSerialPort port;
    if (const auto result = port.open(options.port, options.baud_rate); !result) {
        std::cerr << "open failed: " << result.error() << '\n';
        return 1;
    }
    ScsProtocol protocol(port, std::chrono::milliseconds(options.timeout_ms));

    if (options.scan) {
        std::cerr << "scanning ids 0..253 at " << options.baud_rate << " baud...\n";
        options.ids = scan_ids(protocol);
        if (options.ids.empty()) {
            std::cerr << "no servo answered; check port, baud rate and power\n";
            return 1;
        }
        std::cerr << "found " << options.ids.size() << " servo(s):";
        for (const auto id : options.ids) {
            std::cerr << ' ' << static_cast<unsigned>(id);
        }
        std::cerr << '\n';
    }

    std::cerr << "reading " << options.ids.size() << " servo(s) at " << options.baud_rate
        << " baud, timeout " << options.timeout_ms << " ms, " << options.iterations
        << " measured cycle(s)\n";

    // 一次读的缓冲区与硬件接口完全一致：0x38..0x46 共 15 字节。
    std::array<uint8_t, kFeedbackLength> feedback{};
    std::vector<size_t> read_counts(options.ids.size(), 0);
    std::vector<double> read_totals_us(options.ids.size(), 0.0);
    std::vector<uint16_t> last_position(options.ids.size(), 0);

    const auto read_one = [&](const size_t index) {
        const auto start = Clock::now();
        const auto result = protocol.read_registers(options.ids[index], kRegisterPresentPosition, feedback);
        const auto elapsed = std::chrono::duration<double, std::micro>(Clock::now() - start).count();
        if (!result) {
            return result;
        }
        read_totals_us[index] += elapsed;
        ++read_counts[index];
        last_position[index] = decode_word(feedback.data());
        return result;
    };

    for (int warm = 0; warm < options.warmup; ++warm) {
        for (size_t i = 0; i < options.ids.size(); ++i) {
            if (const auto result = read_one(i); !result) {
                std::cerr << "warmup read failed for id " << static_cast<unsigned>(options.ids[i])
                    << ": " << result.error() << '\n';
                return 1;
            }
        }
    }

    std::vector<double> cycles_ms;
    cycles_ms.reserve(static_cast<size_t>(options.iterations));
    for (int iteration = 0; iteration < options.iterations; ++iteration) {
        const auto start = Clock::now();
        for (size_t i = 0; i < options.ids.size(); ++i) {
            if (const auto result = read_one(i); !result) {
                std::cerr << "read failed for id " << static_cast<unsigned>(options.ids[i])
                    << ": " << result.error() << '\n';
                return 1;
            }
        }
        cycles_ms.push_back(std::chrono::duration<double, std::milli>(Clock::now() - start).count());
    }

    const auto [min_ms, median_ms, p95_ms, max_ms, mean_ms] = summarize(cycles_ms);
    const double total_bytes_per_cycle =
        static_cast<double>(options.ids.size()) * (8.0 + 21.0); // READ 请求 + 15 字节应答帧

    if (options.json) {
        std::cout << std::fixed << std::setprecision(3)
            << "{\"servos\":" << options.ids.size()
            << ",\"baud\":" << options.baud_rate
            << ",\"iterations\":" << options.iterations
            << ",\"timeout_ms\":" << options.timeout_ms
            << ",\"wire_bytes_per_cycle\":" << total_bytes_per_cycle
            << R"(,"cycle_ms":{"min":)" << min_ms
            << ",\"median\":" << median_ms
            << ",\"p95\":" << p95_ms
            << ",\"max\":" << max_ms
            << ",\"mean\":" << mean_ms
            << "},\"per_servo_us\":[";
        for (size_t i = 0; i < options.ids.size(); ++i) {
            if (i != 0) {
                std::cout << ',';
            }
            const double mean = read_counts[i] == 0 ? 0.0 : read_totals_us[i] / static_cast<double>(read_counts[i]);
            std::cout << "{\"id\":" << static_cast<unsigned>(options.ids[i])
                << ",\"mean_us\":" << mean
                << ",\"position\":" << last_position[i] << '}';
        }
        std::cout << "]}\n";
        return 0;
    }

    std::cout << std::fixed << std::setprecision(3);
    std::cout << "servos:               " << options.ids.size() << '\n'
        << "baud:                 " << options.baud_rate << '\n'
        << "iterations:           " << options.iterations << '\n'
        << "wire bytes / cycle:   " << total_bytes_per_cycle << '\n'
        << "cycle min:            " << min_ms << " ms\n"
        << "cycle median:         " << median_ms << " ms\n"
        << "cycle p95:            " << p95_ms << " ms\n"
        << "cycle max:            " << max_ms << " ms\n"
        << "cycle mean:           " << mean_ms << " ms\n"
        << "at 50 Hz budget:      " << (median_ms <= 20.0 ? "fits median" : "exceeds median")
        << " (" << median_ms / 20.0 * 100.0 << "% used)\n"
        << "per-servo mean:       ";
    for (size_t i = 0; i < options.ids.size(); ++i) {
        const double mean = read_counts[i] == 0 ? 0.0 : read_totals_us[i] / static_cast<double>(read_counts[i]);
        std::cout << static_cast<unsigned>(options.ids[i]) << '=' << mean << "us ";
    }
    std::cout << '\n';
    return 0;
}

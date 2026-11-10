// 通过官方 SDK 测试设备流，打印少量描述信息和丢包统计，不保存载荷。
#include <xdsdk/xdsdk_stream.h>
#include <xdsdk/xdsdk_control.h>
#include <atomic>
#include <chrono>
#include <cstdio>
#include <thread>

int main(int argc, char** argv) {
    if (argc != 2) { std::fprintf(stderr, "usage: probe_xdsdk_live <ip>\n"); return 2; }
    xdsdk::ControlClient ctl;
    xdsdk::ControlClient::Options options;
    options.camera_ip = argv[1];
    auto open_status = ctl.open(options);
    xdsdk::xdp::PreviewConfig pc{};
    auto config_status = ctl.get_preview_config(&pc);
    std::printf("control_open=%d preview_status=%d mode=%u data_mask=%u port=%u\n",
        int(open_status), int(config_status), unsigned(pc.mode), unsigned(pc.data_mask), unsigned(pc.port));
    if (open_status != xdsdk::Status::Ok || config_status != xdsdk::Status::Ok) return 3;
    xdsdk::xdp::ImgConfig image_config{};
    auto image_status = ctl.get_img_config(&image_config);
    std::printf("image_status=%d ret_code=%u width=%u height=%u mirrors=%u,%u\n",
        int(image_status), unsigned(image_config.ret_code), unsigned(image_config.width),
        unsigned(image_config.height), unsigned(image_config.mirror[0]), unsigned(image_config.mirror[1]));
    const auto original = pc;
    pc.data_mask |= xdsdk::xdp::PreviewDataImu;
    auto set_status = ctl.set_preview_config(&pc);
    std::printf("enable_imu_status=%d ret_code=%u\n", int(set_status), unsigned(pc.ret_code));
    if (set_status != xdsdk::Status::Ok || pc.ret_code != xdsdk::xdp::RetOk) return 4;
    struct Restore {
        xdsdk::ControlClient& client;
        xdsdk::xdp::PreviewConfig config;
        ~Restore() { auto s = client.set_preview_config(&config); std::printf("restore_status=%d ret_code=%u\n", int(s), unsigned(config.ret_code)); }
    } restore{ctl, original};
    xdsdk::StreamConfig cfg;
    cfg.port = pc.port ? pc.port : xdsdk::xdp::kDefaultStreamPort;
    cfg.camera_address = argv[1];
    cfg.options.delivery = xdsdk::DeliveryMode::Callback;
    xdsdk::StreamSource source(cfg);
    std::atomic<unsigned> frames{0};
    std::atomic<unsigned> other_frames{0};
    auto status = source.start([&](const xdsdk::Frame& frame) {
        auto n = ++frames;
        if (frame.desc().sink_type != 4 && frame.desc().sink_type != 5 && ++other_frames <= 3) {
            std::printf("other_payload sink=%u bytes=%llu hex=", unsigned(frame.desc().sink_type), (unsigned long long)frame.size());
            const auto* bytes = reinterpret_cast<const unsigned char*>(frame.data());
            for (size_t i = 0; i < frame.size() && i < 96; ++i) std::printf("%02x", bytes[i]);
            std::printf("\n");
        }
        if (n <= 30 || n % 100 == 0) {
            const auto& d = frame.desc();
            std::printf("frame=%u sink=%u seq=%llu ts=%llu bytes=%llu\n", n,
                unsigned(d.sink_type), (unsigned long long)d.sequence,
                (unsigned long long)d.device_ts, (unsigned long long)frame.size());
        }
    });
    if (status != xdsdk::Status::Ok) { std::printf("start_error=%d\n", int(status)); return 1; }
    std::this_thread::sleep_for(std::chrono::seconds(8));
    source.stop();
    auto s = source.stream_stats();
    std::printf("datagrams=%llu bad=%llu crc=%llu started=%llu completed=%llu images=%llu dropped=%llu frames=%u\n",
        (unsigned long long)s.datagrams_received, (unsigned long long)s.bad_packets,
        (unsigned long long)s.crc_errors, (unsigned long long)s.frames_started,
        (unsigned long long)s.frames_completed, (unsigned long long)s.images_completed,
        (unsigned long long)s.images_dropped, frames.load());
    return frames ? 0 : 1;
}

// 被动观察 SDK 是否能解析本机转发的 VS030 原始 UDP 包；不控制设备、不落盘图像。
#include <xdsdk/xdsdk_stream.h>
#include <atomic>
#include <chrono>
#include <cstdio>
#include <thread>

int main() {
    xdsdk::StreamConfig config;
    config.bind_address="127.0.0.1";
    config.port=45685;
    config.camera_address="";  // 纯接收，不发送推流或控制命令。
    config.options.delivery=xdsdk::DeliveryMode::Callback;
    xdsdk::StreamSource source(config);
    std::atomic<unsigned> frames{0};
    auto status=source.start([&](const xdsdk::Frame& f){
        ++frames;
        if(frames.load()<=3)
            std::printf("sdk_frame sink=%u seq=%llu device_ts=%llu bytes=%llu\n",
                f.desc().sink_type,(unsigned long long)f.desc().sequence,
                (unsigned long long)f.desc().device_ts,(unsigned long long)f.size());
    });
    if(status!=xdsdk::Status::Ok){std::printf("start_error=%d\n",int(status));return 1;}
    std::puts("SDK_LISTENING");std::fflush(stdout);
    std::this_thread::sleep_for(std::chrono::seconds(14));
    source.stop();auto stats=source.stream_stats();
    std::printf("datagrams=%llu bad=%llu completed=%llu frames=%u\n",
        (unsigned long long)stats.datagrams_received,(unsigned long long)stats.bad_packets,
        (unsigned long long)stats.images_completed,frames.load());
    return 0;
}

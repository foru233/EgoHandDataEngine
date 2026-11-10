// 独立进程隔离厂商 SDK：stdout 仅传二进制帧，stderr 用于诊断。
#include <xdsdk/xdsdk_stream.h>
#include <xdsdk/xdsdk_control.h>
#include <cstdio>
#include <cstdint>
#include <atomic>
#include <mutex>
#include <condition_variable>
#include <deque>
#include <vector>
#include <algorithm>
#include <thread>
#include <chrono>
#include <io.h>
#include <fcntl.h>

#pragma pack(push,1)
// IPC v2: preserve SDK-internal capture/arrival clocks separately from raw device_ts.
// These SDK timestamps are diagnostic only; they are NOT silently used as VIO exposure time.
struct Record {
    uint32_t magic;
    uint16_t version;
    uint16_t header_size;
    uint32_t sink;
    uint32_t size;
    uint64_t sequence;
    uint64_t device_ts;
    uint64_t capture_ts_ns;
    uint64_t arrival_ts_ns;
};
#pragma pack(pop)
static_assert(sizeof(Record) == 48, "worker IPC v2 header must be 48 bytes");
constexpr uint16_t kRecordVersion = 2;
int main(int argc,char** argv) {
    if (argc!=2) return 2;
    _setmode(_fileno(stdout),_O_BINARY);
    xdsdk::ControlClient ctl; xdsdk::ControlClient::Options options; options.camera_ip=argv[1];
    if (ctl.open(options)!=xdsdk::Status::Ok) return 3;
    if (ctl.start_heartbeat([](const xdsdk::xdp::HeartbeatResponse&){})!=xdsdk::Status::Ok) return 8;
    xdsdk::xdp::PreviewConfig pc{}; xdsdk::xdp::ImgConfig image{};
    if (ctl.get_preview_config(&pc)!=xdsdk::Status::Ok || pc.ret_code!=0 ||
        ctl.get_img_config(&image)!=xdsdk::Status::Ok || image.ret_code!=0) return 4;
    if (pc.mode!=0 || !(pc.data_mask & xdsdk::xdp::PreviewDataH265) ||
        image.width!=1600 || image.height!=1200 || image.mirror[0] || image.mirror[1]) {
        std::fprintf(stderr,"unsupported camera configuration mode=%u mask=%u size=%ux%u mirror=%u/%u\n",
            pc.mode,pc.data_mask,image.width,image.height,image.mirror[0],image.mirror[1]); return 5;
    }
    const auto original=pc;
    const bool changed=(pc.data_mask & xdsdk::xdp::PreviewDataImu)==0;
    if (changed) {
        pc.data_mask |= xdsdk::xdp::PreviewDataImu;
        if (ctl.set_preview_config(&pc)!=xdsdk::Status::Ok || pc.ret_code!=0) return 7;
    }
    struct Restore {
        xdsdk::ControlClient& ctl; xdsdk::xdp::PreviewConfig pc; bool changed;
        ~Restore(){if(changed) {auto status=ctl.set_preview_config(&pc); if(status!=xdsdk::Status::Ok || pc.ret_code!=0) std::fprintf(stderr,"preview config restore failed\n");}}
    } restore{ctl,original,changed};
    xdsdk::StreamConfig cfg; cfg.port=pc.port?pc.port:xdsdk::xdp::kDefaultStreamPort;
    cfg.camera_address=argv[1]; cfg.options.delivery=xdsdk::DeliveryMode::Callback;
    cfg.recv_buffer_bytes=32u*1024u*1024u; cfg.options.buffer_count=64;
    xdsdk::StreamSource source(cfg); std::atomic<bool> failed{false}; std::mutex mutex;
    std::condition_variable wake;
    struct Item { Record header; std::vector<uint8_t> data; };
    std::deque<Item> pending; std::atomic<unsigned> dropped_video{0};
    std::thread writer([&] {
        for (;;) {
            Item item;
            { std::unique_lock<std::mutex> lock(mutex); wake.wait_for(lock,std::chrono::milliseconds(100),[&]{return failed || !pending.empty();});
              if (pending.empty()) {if(failed)break;continue;}
              item=std::move(pending.front()); pending.pop_front(); }
            if(std::fwrite(&item.header,sizeof(item.header),1,stdout)!=1 ||
               (item.header.size && std::fwrite(item.data.data(),1,item.data.size(),stdout)!=item.data.size()) || std::fflush(stdout)!=0) {failed=true;break;}
        }
    });
    auto status=source.start([&](const xdsdk::Frame& frame) {
        const auto& d=frame.desc();
        if (d.sink_type!=4 && d.sink_type!=5 && d.sink_type!=7) return;
        if (!frame.size()) return;
        if (frame.size()>8u*1024u*1024u) {failed=true;return;}
        Record rec{0x58444652u,kRecordVersion,uint16_t(sizeof(Record)),d.sink_type,uint32_t(frame.size()),d.sequence,d.device_ts,d.capture_ts_ns,d.arrival_ts_ns};
        const auto* first=reinterpret_cast<const uint8_t*>(frame.data());
        Item item{rec,std::vector<uint8_t>(first,first+frame.size())};
        std::lock_guard<std::mutex> guard(mutex);
        if(pending.size()>=128){
            // 不静默丢失任何相机或 IMU 数据：让上层检测采集故障并标记录制不完整。
            failed=true;std::fprintf(stderr,"sdk output queue overflow\n");return;
        }
        pending.push_back(std::move(item));wake.notify_one();
    });
    if(status!=xdsdk::Status::Ok) {failed=true;wake.notify_all();writer.join();return 6;}
    // 父进程关闭 stdin 表示终止；在主线程 stop 避免 SDK 回调死锁。
    std::thread input([&]{ while(std::getchar()!=EOF){} failed=true; });
    while(!failed) std::this_thread::sleep_for(std::chrono::milliseconds(100));
    source.stop();wake.notify_all();writer.join();
    // 先关闭输出，让接收方看见 EOF 并关闭 stdin，避免故障时双方互等。
    std::fclose(stdout);input.join();
    const auto stats=source.stream_stats();
    std::fprintf(stderr,"sdk stats datagrams=%llu bad=%llu crc=%llu images_dropped=%llu frames_lost=%llu video_queue_drops=%u\n",
        (unsigned long long)stats.datagrams_received,(unsigned long long)stats.bad_packets,
        (unsigned long long)stats.crc_errors,(unsigned long long)stats.images_dropped,(unsigned long long)stats.frames_lost,dropped_video.load());
    return 0;
}

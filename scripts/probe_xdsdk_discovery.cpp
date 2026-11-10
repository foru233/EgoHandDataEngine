// 使用厂商 SDK 的广播发现，仅读取设备信息，不修改相机设置。
#include <xdsdk/xdsdk_control.h>
#include <cstdio>
#include <vector>

int main() {
    std::vector<xdsdk::ControlClient::ProbeResult> devices;
    auto status = xdsdk::ControlClient::probe(&devices, 3000);
    std::printf("status=%d devices=%zu\n", int(status), devices.size());
    for (const auto& device : devices) {
        std::printf("address=%s port=%u model=%s firmware=%s\n",
            device.address.c_str(), device.port,
            xdsdk::xdp::fixed_str(device.info.dev_type, sizeof(device.info.dev_type)).c_str(),
            xdsdk::xdp::version_str(device.info.version).c_str());
    }
    return status == xdsdk::Status::Ok ? 0 : 1;
}

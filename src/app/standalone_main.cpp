/**
 * 非 ROS 可执行入口。
 *
 *   small_point_lio_standalone <配置文件.yaml>
 *
 * 配置文件和 ROS 那边用的是同一份 yaml。里面 `transport:` 写 standalone 即可。
 */

#include "common/log.h"
#include "transport/config_loader.h"
#include "transport/transport.h"

#include <cstdio>

int main(int argc, char **argv) {
    if (argc != 2) {
        std::fprintf(stderr, "用法: %s <配置文件.yaml>\n", argv[0]);
        return 2;
    }

    small_point_lio::Parameters parameters;
    small_point_lio::transport::TransportConfig transport_config;
    transport_config.kind = "standalone";
    std::string error;
    if (!small_point_lio::transport::load_config_file(argv[1], parameters, transport_config, error)) {
        std::fprintf(stderr, "读取配置失败: %s\n", error.c_str());
        return 2;
    }

    std::unique_ptr<small_point_lio::transport::ITransport> transport =
            small_point_lio::transport::make_transport(transport_config, parameters, error);
    if (!transport) {
        std::fprintf(stderr, "%s\n", error.c_str());
        return 2;
    }
    if (!transport->start()) {
        return 1;
    }
    return transport->spin();
}

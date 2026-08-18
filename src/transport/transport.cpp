#include "transport/transport.h"

#ifdef SPL_WITH_STANDALONE
#include "transport/standalone/standalone_transport.h"
#endif
#ifdef SPL_WITH_ROS2
#include "transport/ros2/ros_transport.hpp"
#endif

namespace small_point_lio::transport {

    std::vector<std::string> available_transports() {
        std::vector<std::string> names;
#ifdef SPL_WITH_ROS2
        names.emplace_back("ros2");
#endif
#ifdef SPL_WITH_STANDALONE
        names.emplace_back("standalone");
#endif
        return names;
    }

    std::unique_ptr<ITransport> make_transport(const TransportConfig &transport_config,
                                               const Parameters &parameters,
                                               std::string &error) {
        error.clear();
#ifdef SPL_WITH_STANDALONE
        if (transport_config.kind == "standalone") {
            return std::make_unique<StandaloneTransport>(transport_config, parameters);
        }
#endif
#ifdef SPL_WITH_ROS2
        if (transport_config.kind == "ros2") {
            return std::make_unique<RosTransport>(transport_config, parameters);
        }
#endif
        std::string names;
        for (const auto &name: available_transports()) {
            if (!names.empty()) {
                names += ", ";
            }
            names += name;
        }
        if (names.empty()) {
            names = "（这个构建没有编入任何通信层）";
        }
        error = "配置里的 transport = `" + transport_config.kind +
                "` 在本次构建里不可用。可用的是: " + names;
        return nullptr;
    }

}// namespace small_point_lio::transport

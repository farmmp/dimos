// Copyright 2026 Dimensional Inc.
// SPDX-License-Identifier: Apache-2.0
//
// Gazebo Harmonic native module for the dimos NativeModule framework.
//
// Embeds gz::sim::Server in-process (running on a background thread) and
// bridges gz-transport sensor topics out to LCM (dimos messages), and
// LCM cmd_vel back into gz-transport.
//
// CLI args (passed by the Python NativeModule wrapper):
//   --cmd_vel <lcm-channel>           input  (geometry_msgs/Twist)
//   --terrain_map <lcm-channel>       input  (PointCloud2, ignored)
//   --odometry <lcm-channel>          output (nav_msgs/Odometry)
//   --registered_scan <lcm-channel>   output (sensor_msgs/PointCloud2)
//   --color_image <lcm-channel>       output (sensor_msgs/Image)
//   --semantic_image <lcm-channel>    output (declared, no publisher)
//   --camera_info <lcm-channel>       output (sensor_msgs/CameraInfo)
//   --world <abs-path-to-sdf>         world file (embedded gz::sim::Server)
//   --gz_cmd_vel  <gz-topic>          gz topic for outgoing cmd_vel  (default /model/dimos_bot/cmd_vel)
//   --gz_odom     <gz-topic>          gz topic for incoming odometry (default /model/dimos_bot/odometry)
//   --gz_lidar    <gz-topic>          gz topic for incoming PointCloudPacked (default /lidar/points)
//   --gz_camera   <gz-topic>          gz topic for incoming Image            (default /camera)
//   --gz_caminfo  <gz-topic>          gz topic for incoming CameraInfo       (default /camera_info)
//   --frame_id <str>                  default "base_link"
//   --headless    true|false          unused in embedded mode (kept for API parity)
//   --gz_sim_binary <path>            unused in embedded mode (kept for API parity)

#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>

#include <lcm/lcm-cpp.hpp>

#include <gz/common/Console.hh>
#include <gz/sim/Server.hh>
#include <gz/sim/ServerConfig.hh>
#include <gz/transport/Node.hh>
#include <gz/msgs/twist.pb.h>
#include <gz/msgs/odometry.pb.h>
#include <gz/msgs/image.pb.h>
#include <gz/msgs/camera_info.pb.h>
#include <gz/msgs/pointcloud_packed.pb.h>

#include "dimos_native_module.hpp"

#include "geometry_msgs/Twist.hpp"
#include "geometry_msgs/Pose.hpp"
#include "geometry_msgs/PoseWithCovariance.hpp"
#include "geometry_msgs/TwistWithCovariance.hpp"
#include "nav_msgs/Odometry.hpp"
#include "sensor_msgs/CameraInfo.hpp"
#include "sensor_msgs/Image.hpp"
#include "sensor_msgs/PointCloud2.hpp"
#include "sensor_msgs/PointField.hpp"

using dimos::make_header;
using dimos::time_from_seconds;

static std::atomic<bool> g_running{true};

static void on_signal(int) {
    g_running.store(false);
}

static double now_seconds() {
    using namespace std::chrono;
    return duration<double>(steady_clock::now().time_since_epoch()).count();
}

// gz-msgs pixel-format enum → ROS encoding string (subset that matters for our world).
static std::string gz_pixel_format_to_ros(int pf) {
    switch (pf) {
        case 1: return "l8";       // L_INT8
        case 2: return "l16";      // L_INT16
        case 3: return "rgb8";     // RGB_INT8
        case 6: return "bgr8";     // BGR_INT8
        case 7: return "rgba8";    // RGBA_INT8
        case 8: return "bgra8";    // BGRA_INT8
        default: return "rgb8";
    }
}

// ---------------------------------------------------------------------------
// gz → LCM converters
// ---------------------------------------------------------------------------

static void on_gz_odometry(const gz::msgs::Odometry& msg,
                           lcm::LCM& bus,
                           const std::string& channel,
                           const std::string& frame_id) {
    nav_msgs::Odometry out;
    double ts = msg.has_header() && msg.header().has_stamp()
                    ? msg.header().stamp().sec() + msg.header().stamp().nsec() * 1e-9
                    : now_seconds();
    out.header = make_header(frame_id, ts);
    out.child_frame_id = "odom";
    out.pose.pose.position.x = msg.pose().position().x();
    out.pose.pose.position.y = msg.pose().position().y();
    out.pose.pose.position.z = msg.pose().position().z();
    out.pose.pose.orientation.x = msg.pose().orientation().x();
    out.pose.pose.orientation.y = msg.pose().orientation().y();
    out.pose.pose.orientation.z = msg.pose().orientation().z();
    out.pose.pose.orientation.w = msg.pose().orientation().w();
    for (int i = 0; i < 36; ++i) out.pose.covariance[i] = 0.0;
    out.twist.twist.linear.x = msg.twist().linear().x();
    out.twist.twist.linear.y = msg.twist().linear().y();
    out.twist.twist.linear.z = msg.twist().linear().z();
    out.twist.twist.angular.x = msg.twist().angular().x();
    out.twist.twist.angular.y = msg.twist().angular().y();
    out.twist.twist.angular.z = msg.twist().angular().z();
    for (int i = 0; i < 36; ++i) out.twist.covariance[i] = 0.0;
    bus.publish(channel, &out);
}

static void on_gz_image(const gz::msgs::Image& msg,
                        lcm::LCM& bus,
                        const std::string& channel,
                        const std::string& frame_id) {
    sensor_msgs::Image out;
    double ts = msg.has_header() && msg.header().has_stamp()
                    ? msg.header().stamp().sec() + msg.header().stamp().nsec() * 1e-9
                    : now_seconds();
    out.header = make_header(frame_id + "/camera", ts);
    out.height = static_cast<int32_t>(msg.height());
    out.width = static_cast<int32_t>(msg.width());
    out.encoding = gz_pixel_format_to_ros(msg.pixel_format_type());
    out.is_bigendian = 0;
    out.step = msg.step() > 0 ? static_cast<int32_t>(msg.step())
                              : static_cast<int32_t>(msg.width() * 3);
    out.data_length = static_cast<int32_t>(msg.data().size());
    out.data.assign(msg.data().begin(), msg.data().end());
    bus.publish(channel, &out);
}

static void on_gz_caminfo(const gz::msgs::CameraInfo& msg,
                          lcm::LCM& bus,
                          const std::string& channel,
                          const std::string& frame_id) {
    sensor_msgs::CameraInfo out;
    double ts = msg.has_header() && msg.header().has_stamp()
                    ? msg.header().stamp().sec() + msg.header().stamp().nsec() * 1e-9
                    : now_seconds();
    out.header = make_header(frame_id + "/camera", ts);
    out.height = static_cast<int32_t>(msg.height());
    out.width = static_cast<int32_t>(msg.width());
    out.distortion_model = "plumb_bob";
    if (msg.has_distortion() && msg.distortion().k_size() > 0) {
        out.D_length = msg.distortion().k_size();
        out.D.assign(msg.distortion().k().begin(), msg.distortion().k().end());
    } else {
        out.D_length = 5;
        out.D = {0.0, 0.0, 0.0, 0.0, 0.0};
    }
    for (int i = 0; i < 9; ++i) {
        out.K[i] = (msg.has_intrinsics() && i < msg.intrinsics().k_size())
                       ? msg.intrinsics().k(i)
                       : 0.0;
        out.R[i] = (i < msg.rectification_matrix_size()) ? msg.rectification_matrix(i)
                                                         : (i % 4 == 0 ? 1.0 : 0.0);
    }
    for (int i = 0; i < 12; ++i) {
        out.P[i] = (msg.has_projection() && i < msg.projection().p_size())
                       ? msg.projection().p(i)
                       : 0.0;
    }
    out.binning_x = 0;
    out.binning_y = 0;
    out.roi.x_offset = 0;
    out.roi.y_offset = 0;
    out.roi.height = 0;
    out.roi.width = 0;
    out.roi.do_rectify = 0;
    bus.publish(channel, &out);
}

static void on_gz_pointcloud(const gz::msgs::PointCloudPacked& msg,
                             lcm::LCM& bus,
                             const std::string& channel,
                             const std::string& frame_id) {
    sensor_msgs::PointCloud2 out;
    double ts = msg.has_header() && msg.header().has_stamp()
                    ? msg.header().stamp().sec() + msg.header().stamp().nsec() * 1e-9
                    : now_seconds();
    out.header = make_header(frame_id + "/lidar", ts);
    out.height = static_cast<int32_t>(msg.height());
    out.width = static_cast<int32_t>(msg.width());
    out.is_bigendian = msg.is_bigendian() ? 1 : 0;
    out.is_dense = msg.is_dense() ? 1 : 0;
    out.point_step = static_cast<int32_t>(msg.point_step());
    out.row_step = static_cast<int32_t>(msg.row_step());
    out.fields_length = msg.field_size();
    out.fields.resize(out.fields_length);
    for (int i = 0; i < out.fields_length; ++i) {
        out.fields[i].name = msg.field(i).name();
        out.fields[i].offset = static_cast<int32_t>(msg.field(i).offset());
        out.fields[i].datatype = static_cast<uint8_t>(msg.field(i).datatype());
        out.fields[i].count = static_cast<int32_t>(msg.field(i).count());
    }
    out.data_length = static_cast<int32_t>(msg.data().size());
    out.data.assign(msg.data().begin(), msg.data().end());
    bus.publish(channel, &out);
}

// ---------------------------------------------------------------------------
// LCM → gz forwarder
// ---------------------------------------------------------------------------

class CmdVelForwarder {
public:
    CmdVelForwarder(gz::transport::Node::Publisher pub) : pub_(std::move(pub)) {}

    void operator()(const lcm::ReceiveBuffer*, const std::string&,
                    const geometry_msgs::Twist* msg) {
        gz::msgs::Twist out;
        out.mutable_linear()->set_x(msg->linear.x);
        out.mutable_linear()->set_y(msg->linear.y);
        out.mutable_linear()->set_z(msg->linear.z);
        out.mutable_angular()->set_x(msg->angular.x);
        out.mutable_angular()->set_y(msg->angular.y);
        out.mutable_angular()->set_z(msg->angular.z);
        pub_.Publish(out);
    }

private:
    gz::transport::Node::Publisher pub_;
};

// ---------------------------------------------------------------------------
// main
// ---------------------------------------------------------------------------

int main(int argc, char** argv) {
    std::signal(SIGINT, on_signal);
    std::signal(SIGTERM, on_signal);

    dimos::NativeModule cli(argc, argv);

    // Crank up gz log verbosity so embedded server problems are visible.
    gz::common::Console::SetVerbosity(3);

    const std::string ch_cmd_vel       = cli.arg("cmd_vel");
    const std::string ch_odometry      = cli.arg("odometry");
    const std::string ch_pointcloud    = cli.arg("registered_scan");
    const std::string ch_image         = cli.arg("color_image");
    const std::string ch_caminfo       = cli.arg("camera_info");

    const std::string gz_cmd_vel = cli.arg("gz_cmd_vel", "/model/dimos_bot/cmd_vel");
    const std::string gz_odom    = cli.arg("gz_odom",    "/model/dimos_bot/odometry");
    const std::string gz_lidar   = cli.arg("gz_lidar",   "/lidar/points");
    const std::string gz_camera  = cli.arg("gz_camera",  "/camera");
    const std::string gz_caminfo = cli.arg("gz_caminfo", "/camera_info");

    const std::string frame_id = cli.arg("frame_id", "base_link");

    // Spin up the embedded gz sim server (background thread) before
    // subscribing, so publishers are advertised by the time we wire up.
    std::unique_ptr<gz::sim::Server> server;
    if (cli.has("world")) {
        gz::sim::ServerConfig sc;
        if (!sc.SetSdfFile(cli.arg("world"))) {
            std::cerr << "[gazebo_native] SetSdfFile failed for "
                      << cli.arg("world") << std::endl;
            return 1;
        }
        // Honor the --headless flag (default true). With Gazebo Ionic this
        // tells the embedded Server's render setup to use surfaceless EGL
        // rather than try to open a GLX window.
        const std::string headless_arg = cli.arg("headless", "true");
        const bool headless = headless_arg == "true" || headless_arg == "1";
        sc.SetHeadlessRendering(headless);
        server = std::make_unique<gz::sim::Server>(sc);
        // _blocking=false runs simulation on a background thread.
        // _paused=false starts simulation immediately.
        if (!server->Run(false /*blocking*/, 0 /*iterations=infinite*/, false /*paused*/)) {
            std::cerr << "[gazebo_native] server Run() returned false" << std::endl;
            return 1;
        }
        std::cerr << "[gazebo_native] embedded gz::sim::Server started on "
                  << cli.arg("world") << std::endl;
        // Let publishers get up.
        std::this_thread::sleep_for(std::chrono::milliseconds(1500));
    }

    lcm::LCM bus;
    if (!bus.good()) {
        std::cerr << "[gazebo_native] LCM init failed" << std::endl;
        return 1;
    }

    gz::transport::Node node;

    // gz → LCM subscriptions
    if (!ch_odometry.empty()) {
        bool ok = node.Subscribe<gz::msgs::Odometry>(gz_odom,
            [&bus, ch_odometry, frame_id](const gz::msgs::Odometry& m) {
                static std::atomic<int> n{0};
                if (n.fetch_add(1) < 3) {
                    std::cerr << "[gazebo_native] gz odom #" << n.load()
                              << " → LCM " << ch_odometry << std::endl;
                }
                on_gz_odometry(m, bus, ch_odometry, frame_id);
            });
        std::cerr << "[gazebo_native] subscribed to " << gz_odom
                  << " → LCM " << ch_odometry << " (ok=" << ok << ")" << std::endl;
    }
    if (!ch_pointcloud.empty()) {
        node.Subscribe<gz::msgs::PointCloudPacked>(gz_lidar,
            [&bus, ch_pointcloud, frame_id](const gz::msgs::PointCloudPacked& m) {
                on_gz_pointcloud(m, bus, ch_pointcloud, frame_id);
            });
    }
    if (!ch_image.empty()) {
        node.Subscribe<gz::msgs::Image>(gz_camera,
            [&bus, ch_image, frame_id](const gz::msgs::Image& m) {
                on_gz_image(m, bus, ch_image, frame_id);
            });
    }
    if (!ch_caminfo.empty()) {
        node.Subscribe<gz::msgs::CameraInfo>(gz_caminfo,
            [&bus, ch_caminfo, frame_id](const gz::msgs::CameraInfo& m) {
                on_gz_caminfo(m, bus, ch_caminfo, frame_id);
            });
    }

    // LCM cmd_vel → gz
    auto pub = node.Advertise<gz::msgs::Twist>(gz_cmd_vel);
    if (!pub) {
        std::cerr << "[gazebo_native] failed to advertise " << gz_cmd_vel << std::endl;
    }
    CmdVelForwarder fwd(pub);
    if (!ch_cmd_vel.empty()) {
        bus.subscribe(ch_cmd_vel, &CmdVelForwarder::operator(), &fwd);
    }

    std::cerr << "[gazebo_native] running. lcm in: " << ch_cmd_vel
              << "  out: odom=" << ch_odometry
              << " scan=" << ch_pointcloud
              << " img="  << ch_image
              << " cinfo="<< ch_caminfo << std::endl;

    // Main loop: pump LCM at ~100Hz until SIGTERM. Don't poll
    // server->Running() — it can briefly be false during world load and
    // also during normal idle states, which would cause spurious exits.
    while (g_running.load()) {
        bus.handleTimeout(10);
    }

    if (server) {
        server->Stop();
        server.reset();
    }

    return 0;
}

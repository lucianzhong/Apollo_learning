load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_binary(
    name = "libApolloCyber.so",
    linkshared = True,
    srcs = [
        "CyberReaderControl.cpp",
        "CyberReaderEstimate.cpp",
        "CyberReaderTrajectory.cpp",
        "CyberWriterChassis.cpp",
        "CyberWriterContiRadar.cpp",
        "CyberWriterGps.cpp",
        "CyberWriterImage.cpp",
        "CyberWriterImu.cpp",
        "CyberWriterInsStat.cpp",
        "CyberWriterObstacles.cpp",
        "CyberWriterPointCloud.cpp",
        "CyberWriterRoutingReq.cpp",
        "CyberWriterTrafficLight.cpp",
    ],
    includes = ['.'],

    copts = [
        "-DMODULE_NAME=\\\"bridge\\\"",
    ],
    linkopts=[
        "-Wl,-Bsymbolic"
    ],
    deps = [
        "//cyber",
        "//modules/drivers/proto:sensor_proto",
        "//modules/drivers/gnss/proto:gnss_proto",
        "//modules/localization/proto:localization_proto",
        "//modules/localization/proto:imu_proto",
        "//modules/localization/proto:gps_proto",
        "//modules/perception/proto:perception_proto",
        "//modules/planning/proto:planning_proto",
        "//modules/routing/proto:routing_proto",
        "//modules/control/proto:control_proto",
    ],
)
cpplint()

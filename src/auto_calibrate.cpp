/**
 * Standalone multi-lidar extrinsic calibration tool.
 *
 * Subscribes to N lidar point cloud DDS topics, captures multiple frames,
 * pre-filters, runs coarse-to-fine GICP with best-of-K frame selection,
 * optionally refines all transforms jointly for N>2 lidars, and
 * prints/writes corrected mount_tf extrinsics.
 *
 * Works with any robot — only needs a config JSON listing lidar names,
 * DDS topics, and initial mount transforms.
 *
 * Usage:
 *   ./multi_lidar_calibrate [OPTIONS]
 *
 * Example:
 *   ./multi_lidar_calibrate --config calibration_config.json --ref front_lidar
 *   ./multi_lidar_calibrate --num-frames 10 --max-fitness 0.2 --write-config
 *   ./multi_lidar_calibrate --save-aligned merged.pcd
 */

#include <multi_lidar_calibration/Calibration.hpp>

#include <message_manager/fastdds_transport.hpp>
#include <message_manager/publisher_options.hpp>
#include <message_manager/type_id.hpp>
#include <message_manager/types.hpp>

#include <pcl/io/pcd_io.h>

#include <nlohmann/json.hpp>

#include <chrono>
#include <cstring>
#include <fstream>
#include <iostream>
#include <map>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace mlc = multi_lidar_calibration;
using PointT  = mlc::PointT;
using Cloud   = mlc::Cloud;
using json    = nlohmann::json;

// ---------------------------------------------------------------------------
// Per-lidar info parsed from config
// ---------------------------------------------------------------------------
struct LidarInfo {
    std::string name;
    std::string topic;
    double translation[3]{};
    double rpy_deg[3]{};
};

static std::vector<LidarInfo> loadConfig(const std::string& path, int& domain_id) {
    std::vector<LidarInfo> result;
    std::ifstream f(path);
    if (!f.is_open()) {
        std::cerr << "Error: cannot open " << path << "\n";
        return result;
    }
    json j;
    f >> j;

    if (j.contains("dds") && j["dds"].contains("domain_id"))
        domain_id = j["dds"]["domain_id"].get<int>();

    if (!j.contains("lidars") || !j["lidars"].is_array()) return result;

    for (const auto& entry : j["lidars"]) {
        LidarInfo li;
        if (entry.contains("name"))
            li.name = entry["name"].get<std::string>();
        if (entry.contains("topic"))
            li.topic = entry["topic"].get<std::string>();
        if (entry.contains("mount_tf")) {
            const auto& m = entry["mount_tf"];
            if (m.contains("translation") && m["translation"].size() == 3)
                for (int i = 0; i < 3; ++i)
                    li.translation[i] = m["translation"][i].get<double>();
            if (m.contains("rotation_rpy_deg") && m["rotation_rpy_deg"].size() == 3)
                for (int i = 0; i < 3; ++i)
                    li.rpy_deg[i] = m["rotation_rpy_deg"][i].get<double>();
        }
        result.push_back(li);
    }
    return result;
}

// ---------------------------------------------------------------------------
// DDS PointCloud2 -> PCL conversion
// ---------------------------------------------------------------------------
static Cloud::Ptr ddsToCloud(const sensor_msgs::msg::dds_::PointCloud2_8MB_& msg) {
    const auto& fields = msg.fields();
    const auto& data   = msg.data();
    uint32_t point_step = msg.point_step();
    uint32_t n_points   = msg.width() * msg.height();

    int off_x = -1, off_y = -1, off_z = -1, off_i = -1;
    for (uint8_t fi = 0; fi < msg.num_fields(); ++fi) {
        const auto& fld = fields[fi];
        const auto& name = fld.name();
        if      (name == "x")         off_x = static_cast<int>(fld.offset());
        else if (name == "y")         off_y = static_cast<int>(fld.offset());
        else if (name == "z")         off_z = static_cast<int>(fld.offset());
        else if (name == "intensity") off_i = static_cast<int>(fld.offset());
    }

    if (off_x < 0 || off_y < 0 || off_z < 0) return nullptr;

    auto cloud = Cloud::Ptr(new Cloud);
    cloud->reserve(n_points);

    for (uint32_t i = 0; i < n_points; ++i) {
        const uint8_t* pt_ptr = data.data() + static_cast<size_t>(i) * point_step;
        PointT pt;
        std::memcpy(&pt.x, pt_ptr + off_x, sizeof(float));
        std::memcpy(&pt.y, pt_ptr + off_y, sizeof(float));
        std::memcpy(&pt.z, pt_ptr + off_z, sizeof(float));
        pt.intensity = 0.0f;
        if (off_i >= 0) std::memcpy(&pt.intensity, pt_ptr + off_i, sizeof(float));

        if (!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z))
            continue;
        cloud->push_back(pt);
    }
    return cloud;
}

// ---------------------------------------------------------------------------
// Usage
// ---------------------------------------------------------------------------
static void printUsage(const char* prog) {
    std::cerr
        << "Standalone multi-lidar extrinsic calibration tool.\n\n"
        << "Usage:\n"
        << "  " << prog << " [OPTIONS]\n\n"
        << "Options:\n"
        << "  --config <path>        Calibration config JSON (default: calibration_config.json)\n"
        << "  --ref <name>           Reference lidar — kept fixed (default: first in config)\n"
        << "  --timeout <seconds>    Capture timeout per lidar (default: 30)\n"
        << "  --num-frames <K>       Frames to capture per lidar for best-of-K (default: 5)\n"
        << "  --voxel <metres>       VoxelGrid leaf size for fine GICP pass (default: 0.1)\n"
        << "  --max-iter <n>         Max GICP iterations per pass (default: 100)\n"
        << "  --corr-dist <metres>   Max correspondence distance (default: 2.0)\n"
        << "  --min-range <metres>   Remove points closer than this (default: 0.5)\n"
        << "  --max-range <metres>   Remove points farther than this (default: 100.0)\n"
        << "  --max-fitness <val>    Reject GICP results above this fitness (default: 0.3)\n"
        << "  --refine-iters <n>     Joint refinement iterations for N>2 (default: 10)\n"
        << "  --save-aligned <path>  Save merged aligned cloud to PCD\n"
        << "  --write-config         Write corrected mount_tf back to config file\n"
        << "  -h, --help             Show this message\n\n"
        << "Config format:\n"
        << "  {\n"
        << "    \"dds\": { \"domain_id\": 0 },\n"
        << "    \"lidars\": [\n"
        << "      {\n"
        << "        \"name\": \"front_lidar\",\n"
        << "        \"topic\": \"/front_lidar/points\",\n"
        << "        \"mount_tf\": {\n"
        << "          \"translation\": [2.0, 0.0, 0.0],\n"
        << "          \"rotation_rpy_deg\": [33, 0, 90]\n"
        << "        }\n"
        << "      }\n"
        << "    ]\n"
        << "  }\n\n"
        << "Requires lidars to be publishing PointCloud2 on DDS topics.\n";
}

// ---------------------------------------------------------------------------
// main
// ---------------------------------------------------------------------------
int main(int argc, char** argv) {
    std::string config_path = "calibration_config.json";
    std::string ref_name;
    int    capture_timeout = 30;
    int    num_frames      = 5;
    float  voxel_size      = 0.1f;
    int    max_iter        = 100;
    float  corr_dist       = 2.0f;
    float  min_range       = 0.5f;
    float  max_range       = 100.0f;
    float  max_fitness     = 0.3f;
    int    refine_iters    = 10;
    std::string save_aligned_path;
    bool   write_config    = false;

    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if ((arg == "--config" || arg == "-c") && i + 1 < argc)
            config_path = argv[++i];
        else if ((arg == "--ref" || arg == "-r") && i + 1 < argc)
            ref_name = argv[++i];
        else if (arg == "--timeout" && i + 1 < argc)
            capture_timeout = std::stoi(argv[++i]);
        else if (arg == "--num-frames" && i + 1 < argc)
            num_frames = std::stoi(argv[++i]);
        else if (arg == "--voxel" && i + 1 < argc)
            voxel_size = std::stof(argv[++i]);
        else if (arg == "--max-iter" && i + 1 < argc)
            max_iter = std::stoi(argv[++i]);
        else if (arg == "--corr-dist" && i + 1 < argc)
            corr_dist = std::stof(argv[++i]);
        else if (arg == "--min-range" && i + 1 < argc)
            min_range = std::stof(argv[++i]);
        else if (arg == "--max-range" && i + 1 < argc)
            max_range = std::stof(argv[++i]);
        else if (arg == "--max-fitness" && i + 1 < argc)
            max_fitness = std::stof(argv[++i]);
        else if (arg == "--refine-iters" && i + 1 < argc)
            refine_iters = std::stoi(argv[++i]);
        else if (arg == "--save-aligned" && i + 1 < argc)
            save_aligned_path = argv[++i];
        else if (arg == "--write-config")
            write_config = true;
        else if (arg == "--help" || arg == "-h") {
            printUsage(argv[0]);
            return 0;
        } else {
            std::cerr << "Unknown argument: " << arg << "\n\n";
            printUsage(argv[0]);
            return 1;
        }
    }

    if (num_frames < 1) num_frames = 1;

    // ---- Load config ----
    int domain_id = 0;
    auto lidars = loadConfig(config_path, domain_id);
    if (lidars.size() < 2) {
        std::cerr << "Error: need at least 2 lidars in config, found " << lidars.size() << "\n";
        return 1;
    }

    if (ref_name.empty()) ref_name = lidars[0].name;

    bool ref_found = false;
    for (const auto& li : lidars)
        if (li.name == ref_name) { ref_found = true; break; }
    if (!ref_found) {
        std::cerr << "Error: reference lidar '" << ref_name << "' not in config\n";
        return 1;
    }

    std::cout << "=== Multi-Lidar Calibration ===\n"
              << "Config          : " << config_path << "\n"
              << "Reference lidar : " << ref_name << "\n"
              << "Capture timeout : " << capture_timeout << " s\n"
              << "Frames per lidar: " << num_frames << "\n"
              << "Voxel size      : " << voxel_size << " m\n"
              << "Max GICP iters  : " << max_iter << "\n"
              << "Corr distance   : " << corr_dist << " m\n"
              << "Range filter    : [" << min_range << ", " << max_range << "] m\n"
              << "Max fitness     : " << max_fitness << "\n"
              << "Refine iters    : " << refine_iters << "\n"
              << "Lidars          : " << lidars.size() << "\n\n";

    // ---- Set up DDS and subscribe to all topics ----
    auto dds = std::make_shared<message_manager::FastDDSTransport>(
        message_manager::FastDDSTransport::Config{
            .domain_id        = domain_id,
            .participant_name = "multi_lidar_calibrate",
            .enable_shm       = true,
        });

    if (!dds->initialize()) {
        std::cerr << "Error: DDS init failed\n";
        return 1;
    }

    // Capture K frames per lidar
    std::mutex mu;
    std::map<std::string, std::vector<Cloud::Ptr>> captured_frames;

    for (const auto& li : lidars) {
        std::string name  = li.name;
        std::string topic = li.topic;
        int K = num_frames;

        std::cout << "Subscribing to " << topic << " (" << name << ")...\n";

        auto sub = dds->registerSubscriber(
            topic,
            message_manager::type_id<sensor_msgs::msg::dds_::PointCloud2_8MB_>,
            [&mu, &captured_frames, name, K](const void* ptr, const std::type_index&) {
                std::lock_guard<std::mutex> lk(mu);
                auto& frames = captured_frames[name];
                if (static_cast<int>(frames.size()) >= K) return;

                const auto& msg = *static_cast<const sensor_msgs::msg::dds_::PointCloud2_8MB_*>(ptr);
                auto cloud = ddsToCloud(msg);
                if (cloud && !cloud->empty()) {
                    frames.push_back(cloud);
                    std::cout << "  [" << name << "] frame " << frames.size()
                              << "/" << K << " — " << cloud->size() << " points\n";
                }
            },
            message_manager::SubscriberOptions().transport<message_manager::FastDDSTransport>(
                {.qos_depth = 5, .transient_local = false, .reliable = false}));

        if (!sub) {
            std::cerr << "Error: failed to subscribe to " << topic << "\n";
            dds->shutdown();
            return 1;
        }
    }

    // ---- Wait for all captures ----
    std::cout << "\nWaiting for point clouds (" << num_frames << " frames each, timeout "
              << capture_timeout << "s)...\n";
    auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(capture_timeout);

    while (std::chrono::steady_clock::now() < deadline) {
        {
            std::lock_guard<std::mutex> lk(mu);
            bool all_done = true;
            for (const auto& li : lidars) {
                if (static_cast<int>(captured_frames[li.name].size()) < num_frames) {
                    all_done = false;
                    break;
                }
            }
            if (all_done) break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    dds->shutdown();

    // Check what we got
    {
        std::lock_guard<std::mutex> lk(mu);
        int captured_count = 0;
        for (const auto& li : lidars)
            if (!captured_frames[li.name].empty()) ++captured_count;

        if (captured_count < 2) {
            std::cerr << "\nError: only captured " << captured_count << " / "
                      << lidars.size() << " lidars. Make sure they are publishing.\n";
            for (const auto& li : lidars)
                if (captured_frames[li.name].empty())
                    std::cerr << "  MISSING: " << li.name << " (" << li.topic << ")\n";
            return 1;
        }
        if (!captured_frames.count(ref_name) || captured_frames[ref_name].empty()) {
            std::cerr << "\nError: reference lidar '" << ref_name << "' not captured\n";
            return 1;
        }
        if (captured_count < static_cast<int>(lidars.size())) {
            std::cerr << "\nWarning: only captured " << captured_count << " / "
                      << lidars.size() << " lidars. Continuing with available data.\n";
            for (const auto& li : lidars)
                if (captured_frames[li.name].empty())
                    std::cerr << "  MISSING: " << li.name << " (" << li.topic << ")\n";
        }
    }

    std::cout << "\nAll captures done.\n";

    // ---- Pre-filter and prepare data ----
    struct LidarData {
        std::string name;
        std::vector<Cloud::Ptr> filtered_frames;
        Eigen::Matrix4d T_initial;
    };
    std::vector<LidarData> lidar_data;

    std::cout << "\n--- Pre-filtering ---\n";
    for (const auto& li : lidars) {
        if (captured_frames[li.name].empty()) continue;

        LidarData ld;
        ld.name = li.name;
        ld.T_initial = mlc::mountTfToMatrix(li.translation, li.rpy_deg);

        for (size_t fi = 0; fi < captured_frames[li.name].size(); ++fi) {
            auto raw = captured_frames[li.name][fi];
            auto filtered = mlc::preFilterCloud(raw, min_range, max_range);
            std::cout << "[" << li.name << " frame " << fi << "] "
                      << raw->size() << " raw -> " << filtered->size() << " filtered\n";
            if (!filtered->empty())
                ld.filtered_frames.push_back(filtered);
        }

        if (ld.filtered_frames.empty()) {
            std::cerr << "Warning: [" << li.name << "] all frames empty after filtering — skipping\n";
            continue;
        }
        lidar_data.push_back(std::move(ld));
    }

    if (lidar_data.size() < 2) {
        std::cerr << "Error: fewer than 2 lidars with valid data after filtering\n";
        return 1;
    }

    // Find reference index
    int ref_idx = -1;
    for (int i = 0; i < static_cast<int>(lidar_data.size()); ++i)
        if (lidar_data[i].name == ref_name) { ref_idx = i; break; }
    if (ref_idx < 0) {
        std::cerr << "Error: reference lidar lost after filtering\n";
        return 1;
    }

    // Pick the reference frame with the most points
    const auto& ref = lidar_data[ref_idx];
    Cloud::ConstPtr ref_cloud = ref.filtered_frames[0];
    for (size_t fi = 1; fi < ref.filtered_frames.size(); ++fi)
        if (ref.filtered_frames[fi]->size() > ref_cloud->size())
            ref_cloud = ref.filtered_frames[fi];
    std::cout << "\n[" << ref.name << "] reference cloud: " << ref_cloud->size() << " points\n";

    // ---- Best-of-K coarse-to-fine GICP alignment ----
    std::cout << "\n======================================\n"
              << "  Best-of-K Coarse-to-Fine GICP\n"
              << "======================================\n\n";

    struct CalibResult {
        std::string name;
        Eigen::Matrix4d T_corrected;
        double translation[3];
        double rpy_deg[3];
        double fitness;
        bool converged;
        bool rejected;
    };
    std::vector<CalibResult> results;

    // Reference stays fixed
    {
        CalibResult cr;
        cr.name = ref.name;
        cr.T_corrected = ref.T_initial;
        mlc::matrixToMountTf(ref.T_initial, cr.translation, cr.rpy_deg);
        cr.fitness   = 0.0;
        cr.converged = true;
        cr.rejected  = false;
        results.push_back(cr);
    }

    for (int i = 0; i < static_cast<int>(lidar_data.size()); ++i) {
        if (i == ref_idx) continue;
        const auto& src = lidar_data[i];

        std::cout << "[GICP] aligning " << src.name << " -> " << ref.name
                  << " (best of " << src.filtered_frames.size() << " frames)...\n";

        Eigen::Matrix4d T_guess = ref.T_initial.inverse() * src.T_initial;

        mlc::GicpResult best;
        int best_frame = -1;

        for (size_t fi = 0; fi < src.filtered_frames.size(); ++fi) {
            auto result = mlc::runCoarseToFineGicp(
                src.filtered_frames[fi], ref_cloud,
                voxel_size, max_iter, corr_dist,
                T_guess.cast<float>());

            std::cout << "  frame " << fi << ": converged=" << (result.converged ? "Y" : "N")
                      << "  fitness=" << result.fitness << "\n";

            if (result.converged && result.fitness < best.fitness) {
                best = result;
                best_frame = static_cast<int>(fi);
            }
        }

        CalibResult cr;
        cr.name      = src.name;
        cr.converged = best.converged;
        cr.fitness   = best.fitness;
        cr.rejected  = false;

        if (best.converged && best.fitness <= max_fitness) {
            Eigen::Matrix4d T_src_to_ref = best.transform;
            cr.T_corrected = ref.T_initial * T_src_to_ref;
            mlc::matrixToMountTf(cr.T_corrected, cr.translation, cr.rpy_deg);

            std::cout << "  BEST frame " << best_frame << ":\n"
                      << "    fitness     : " << cr.fitness << "\n"
                      << "    corrected t : [" << cr.translation[0] << ", "
                      << cr.translation[1] << ", " << cr.translation[2] << "]\n"
                      << "    corrected rpy: [" << cr.rpy_deg[0] << ", "
                      << cr.rpy_deg[1] << ", " << cr.rpy_deg[2] << "] deg\n";

            const LidarInfo* orig = nullptr;
            for (const auto& li : lidars)
                if (li.name == src.name) { orig = &li; break; }
            if (orig) {
                std::cout << "    delta t     : ("
                          << cr.translation[0] - orig->translation[0] << ", "
                          << cr.translation[1] - orig->translation[1] << ", "
                          << cr.translation[2] - orig->translation[2] << ") m\n"
                          << "    delta rpy   : ("
                          << cr.rpy_deg[0] - orig->rpy_deg[0] << ", "
                          << cr.rpy_deg[1] - orig->rpy_deg[1] << ", "
                          << cr.rpy_deg[2] - orig->rpy_deg[2] << ") deg\n";
            }
        } else {
            cr.T_corrected = src.T_initial;
            mlc::matrixToMountTf(src.T_initial, cr.translation, cr.rpy_deg);
            cr.rejected = true;

            if (!best.converged)
                std::cout << "  REJECTED: GICP did not converge — keeping original mount_tf\n";
            else
                std::cout << "  REJECTED: fitness " << best.fitness
                          << " > threshold " << max_fitness << " — keeping original mount_tf\n";
        }
        std::cout << "\n";
        results.push_back(cr);
    }

    // ---- Iterative multi-lidar refinement for N > 2 ----
    int non_rejected = 0;
    for (const auto& cr : results)
        if (!cr.rejected) ++non_rejected;

    if (non_rejected > 2 && refine_iters > 0) {
        std::cout << "======================================\n"
                  << "  Joint Multi-Lidar Refinement (" << non_rejected << " lidars)\n"
                  << "======================================\n\n";

        std::vector<mlc::LidarPose> poses;
        for (const auto& cr : results) {
            if (cr.rejected) continue;
            mlc::LidarPose lp;
            lp.name = cr.name;
            lp.T    = cr.T_corrected;
            lp.is_reference = (cr.name == ref_name);

            for (const auto& ld : lidar_data) {
                if (ld.name == cr.name) {
                    lp.cloud = ld.filtered_frames[0];
                    for (size_t fi = 1; fi < ld.filtered_frames.size(); ++fi)
                        if (ld.filtered_frames[fi]->size() > lp.cloud->size())
                            lp.cloud = ld.filtered_frames[fi];
                    break;
                }
            }
            poses.push_back(lp);
        }

        auto refined = mlc::iterativeMultiLidarRefinement(
            poses, voxel_size, max_iter, corr_dist, max_fitness, refine_iters);

        for (auto& cr : results) {
            if (cr.rejected) continue;
            for (const auto& rp : refined) {
                if (rp.name == cr.name) {
                    cr.T_corrected = rp.T;
                    mlc::matrixToMountTf(rp.T, cr.translation, cr.rpy_deg);
                    break;
                }
            }
        }
        std::cout << "\n";
    }

    // ---- Print config-ready output ----
    std::cout << "======================================\n"
              << "  Calibrated mount_tf values\n"
              << "======================================\n\n";

    for (const auto& cr : results) {
        std::string status = cr.rejected ? " (REJECTED — original)" : "";
        std::cout << "\"" << cr.name << "\"" << status << ": {\n"
                  << "  \"mount_tf\": {\n"
                  << "    \"translation\": ["
                  << cr.translation[0] << ", " << cr.translation[1] << ", " << cr.translation[2] << "],\n"
                  << "    \"rotation_rpy_deg\": ["
                  << cr.rpy_deg[0] << ", " << cr.rpy_deg[1] << ", " << cr.rpy_deg[2] << "]\n"
                  << "  }\n"
                  << "}\n\n";
    }

    // ---- Write config back ----
    if (write_config) {
        int writable = 0;
        for (const auto& cr : results)
            if (!cr.rejected) ++writable;

        if (writable <= 1) {
            std::cerr << "Error: all non-reference calibrations were rejected — "
                      << "refusing to write config.\n";
        } else {
            std::ifstream fin(config_path);
            if (!fin.is_open()) {
                std::cerr << "Error: cannot re-read " << config_path << " for writing\n";
                return 1;
            }
            json j;
            fin >> j;
            fin.close();

            for (auto& entry : j["lidars"]) {
                std::string name = entry.value("name", "");
                for (const auto& cr : results) {
                    if (cr.name != name || !cr.converged || cr.rejected) continue;
                    entry["mount_tf"]["translation"] = {cr.translation[0], cr.translation[1], cr.translation[2]};
                    entry["mount_tf"]["rotation_rpy_deg"] = {cr.rpy_deg[0], cr.rpy_deg[1], cr.rpy_deg[2]};
                    break;
                }
            }

            std::ofstream fout(config_path);
            if (!fout.is_open()) {
                std::cerr << "Error: cannot write " << config_path << "\n";
                return 1;
            }
            fout << j.dump(2) << "\n";
            fout.close();
            std::cout << "Updated " << config_path << " with calibrated mount_tf values.\n";

            for (const auto& cr : results)
                if (cr.rejected)
                    std::cout << "  Skipped " << cr.name << " (rejected)\n";
        }
    }

    // ---- Save merged aligned cloud ----
    if (!save_aligned_path.empty()) {
        Cloud merged;
        for (const auto& ld : lidar_data) {
            const CalibResult* cr = nullptr;
            for (const auto& r : results)
                if (r.name == ld.name) { cr = &r; break; }
            if (!cr) continue;
            for (const auto& frame : ld.filtered_frames) {
                Cloud transformed;
                pcl::transformPointCloud(*frame, transformed, cr->T_corrected.cast<float>());
                merged += transformed;
            }
        }
        if (pcl::io::savePCDFileBinary(save_aligned_path, merged) < 0) {
            std::cerr << "Error: failed to save " << save_aligned_path << "\n";
        } else {
            std::cout << "Saved aligned cloud (" << merged.size() << " points) to "
                      << save_aligned_path << "\n";
        }
    }

    return 0;
}

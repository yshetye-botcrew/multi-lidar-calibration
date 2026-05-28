#pragma once

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/registration/gicp.h>
#include <pcl/common/transforms.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <cmath>
#include <iostream>
#include <string>
#include <vector>

namespace multi_lidar_calibration {

using PointT = pcl::PointXYZI;
using Cloud  = pcl::PointCloud<PointT>;

// ---------------------------------------------------------------------------
// Result types
// ---------------------------------------------------------------------------

struct GicpResult {
    Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();
    double fitness   = std::numeric_limits<double>::max();
    bool   converged = false;
};

struct LidarPose {
    std::string      name;
    Cloud::ConstPtr  cloud;
    Eigen::Matrix4d  T = Eigen::Matrix4d::Identity();
    bool             is_reference = false;
};

// ---------------------------------------------------------------------------
// Transform helpers  (ZYX intrinsic Euler angles <-> 4x4 matrix)
// ---------------------------------------------------------------------------

inline Eigen::Matrix4d mountTfToMatrix(const double t[3], const double rpy_deg[3]) {
    const double r = rpy_deg[0] * M_PI / 180.0;
    const double p = rpy_deg[1] * M_PI / 180.0;
    const double y = rpy_deg[2] * M_PI / 180.0;
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    T.block<3, 3>(0, 0) =
        (Eigen::AngleAxisd(y, Eigen::Vector3d::UnitZ()) *
         Eigen::AngleAxisd(p, Eigen::Vector3d::UnitY()) *
         Eigen::AngleAxisd(r, Eigen::Vector3d::UnitX()))
            .toRotationMatrix();
    T(0, 3) = t[0];
    T(1, 3) = t[1];
    T(2, 3) = t[2];
    return T;
}

inline void matrixToMountTf(const Eigen::Matrix4d& T, double t[3], double rpy_deg[3]) {
    t[0] = T(0, 3);
    t[1] = T(1, 3);
    t[2] = T(2, 3);
    const Eigen::Matrix3d R = T.block<3, 3>(0, 0);
    rpy_deg[0] = std::atan2(R(2, 1), R(2, 2)) * 180.0 / M_PI;
    rpy_deg[1] = std::atan2(-R(2, 0), std::sqrt(R(2, 1) * R(2, 1) + R(2, 2) * R(2, 2))) * 180.0 / M_PI;
    rpy_deg[2] = std::atan2(R(1, 0), R(0, 0)) * 180.0 / M_PI;
}

// ---------------------------------------------------------------------------
// Pre-filtering
// ---------------------------------------------------------------------------

inline Cloud::Ptr rangeFilter(Cloud::ConstPtr input, float min_range, float max_range) {
    auto output = Cloud::Ptr(new Cloud);
    output->reserve(input->size());
    const float min_sq = min_range * min_range;
    const float max_sq = max_range * max_range;
    for (const auto& pt : *input) {
        float d2 = pt.x * pt.x + pt.y * pt.y + pt.z * pt.z;
        if (d2 >= min_sq && d2 <= max_sq)
            output->push_back(pt);
    }
    return output;
}

inline Cloud::Ptr statisticalOutlierFilter(Cloud::ConstPtr input, int mean_k = 50,
                                           double stddev_mul = 1.0) {
    if (input->empty()) return Cloud::Ptr(new Cloud);
    auto output = Cloud::Ptr(new Cloud);
    pcl::StatisticalOutlierRemoval<PointT> sor;
    sor.setInputCloud(input);
    sor.setMeanK(mean_k);
    sor.setStddevMulThresh(stddev_mul);
    sor.filter(*output);
    return output;
}

inline Cloud::Ptr preFilterCloud(Cloud::ConstPtr input, float min_range, float max_range,
                                 int sor_mean_k = 50, double sor_stddev_mul = 1.0) {
    auto ranged = rangeFilter(input, min_range, max_range);
    return statisticalOutlierFilter(ranged, sor_mean_k, sor_stddev_mul);
}

// ---------------------------------------------------------------------------
// Downsampling
// ---------------------------------------------------------------------------

inline Cloud::Ptr downsample(Cloud::ConstPtr input, float leaf_size) {
    auto output = Cloud::Ptr(new Cloud);
    if (input->empty()) return output;
    pcl::VoxelGrid<PointT> vg;
    vg.setInputCloud(input);
    vg.setLeafSize(leaf_size, leaf_size, leaf_size);
    vg.filter(*output);
    return output;
}

// ---------------------------------------------------------------------------
// Single-pass GICP  (expects pre-filtered clouds; downsamples internally)
// ---------------------------------------------------------------------------

inline GicpResult runSingleGicp(Cloud::ConstPtr source, Cloud::ConstPtr target,
                                float voxel_size, int max_iter, float corr_dist,
                                const Eigen::Matrix4f& initial_guess) {
    GicpResult result;
    auto src_down = downsample(source, voxel_size);
    auto tgt_down = downsample(target, voxel_size);

    if (src_down->size() < 30 || tgt_down->size() < 30) {
        std::cerr << "  Warning: too few points after downsampling ("
                  << src_down->size() << " src, " << tgt_down->size() << " tgt)\n";
        return result;
    }

    pcl::GeneralizedIterativeClosestPoint<PointT, PointT> gicp;
    gicp.setMaximumIterations(max_iter);
    gicp.setTransformationEpsilon(1e-8);
    gicp.setMaxCorrespondenceDistance(corr_dist);
    gicp.setInputSource(src_down);
    gicp.setInputTarget(tgt_down);

    Cloud aligned;
    gicp.align(aligned, initial_guess);

    result.transform = gicp.getFinalTransformation().cast<double>();
    result.fitness   = gicp.getFitnessScore();
    result.converged = gicp.hasConverged();
    return result;
}

// ---------------------------------------------------------------------------
// Coarse-to-fine GICP  (4x voxel coarse pass, then 1x fine pass)
// ---------------------------------------------------------------------------

inline GicpResult runCoarseToFineGicp(Cloud::ConstPtr source, Cloud::ConstPtr target,
                                      float voxel_size, int max_iter, float corr_dist,
                                      const Eigen::Matrix4f& initial_guess) {
    auto coarse = runSingleGicp(source, target,
                                voxel_size * 4.0f, max_iter, corr_dist * 2.0f,
                                initial_guess);

    Eigen::Matrix4f fine_guess = coarse.converged
        ? coarse.transform.cast<float>()
        : initial_guess;

    return runSingleGicp(source, target, voxel_size, max_iter, corr_dist, fine_guess);
}

// ---------------------------------------------------------------------------
// Iterative multi-lidar refinement (joint optimization for N > 2)
//
// Each non-reference lidar is repeatedly aligned against the merged cloud
// of all other lidars until the transforms converge.
// ---------------------------------------------------------------------------

inline std::vector<LidarPose> iterativeMultiLidarRefinement(
    std::vector<LidarPose> poses,
    float voxel_size, int max_iter, float corr_dist, float max_fitness,
    int refinement_iters = 10, double convergence_threshold = 1e-4)
{
    if (poses.size() <= 2) return poses;

    for (int iter = 0; iter < refinement_iters; ++iter) {
        double max_delta = 0.0;

        for (size_t i = 0; i < poses.size(); ++i) {
            if (poses[i].is_reference) continue;

            Cloud::Ptr merged_target(new Cloud);
            for (size_t j = 0; j < poses.size(); ++j) {
                if (j == i) continue;
                Cloud transformed;
                pcl::transformPointCloud(*poses[j].cloud, transformed,
                                         poses[j].T.cast<float>());
                *merged_target += transformed;
            }

            merged_target = downsample(merged_target, voxel_size);

            auto result = runCoarseToFineGicp(
                poses[i].cloud, merged_target,
                voxel_size, max_iter, corr_dist,
                poses[i].T.cast<float>());

            if (result.converged && result.fitness <= max_fitness) {
                double translation_change =
                    (result.transform.block<3, 1>(0, 3) - poses[i].T.block<3, 1>(0, 3)).norm();
                max_delta = std::max(max_delta, translation_change);
                poses[i].T = result.transform;
            }
        }

        std::cout << "  [refinement iter " << iter << "] max delta = " << max_delta << " m\n";
        if (max_delta < convergence_threshold) {
            std::cout << "  Converged after " << (iter + 1) << " iterations.\n";
            break;
        }
    }
    return poses;
}

}  // namespace multi_lidar_calibration

#include <multi_lidar_calibration/Calibration.hpp>
#include <gtest/gtest.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <cmath>
#include <random>

namespace mlc = multi_lidar_calibration;
using PointT  = mlc::PointT;
using Cloud   = mlc::Cloud;

namespace {

Cloud::Ptr makeSurface(float x_min, float x_max, float y_min, float y_max,
                       float step, float z_amplitude = 0.3f, unsigned seed = 42) {
    auto cloud = Cloud::Ptr(new Cloud);
    std::mt19937 rng(seed);
    std::normal_distribution<float> noise(0.0f, 0.005f);
    for (float x = x_min; x <= x_max; x += step) {
        for (float y = y_min; y <= y_max; y += step) {
            PointT pt;
            pt.x = x + noise(rng);
            pt.y = y + noise(rng);
            pt.z = z_amplitude * std::sin(x * 2.0f) * std::cos(y * 2.0f) + noise(rng);
            pt.intensity = 50.0f;
            cloud->push_back(pt);
        }
    }
    return cloud;
}

Eigen::Matrix4f makeTransform(float tx, float ty, float tz, float yaw_deg) {
    Eigen::Matrix4f T = Eigen::Matrix4f::Identity();
    float rad = yaw_deg * static_cast<float>(M_PI) / 180.0f;
    T.block<3, 3>(0, 0) = Eigen::AngleAxisf(rad, Eigen::Vector3f::UnitZ()).toRotationMatrix();
    T(0, 3) = tx;
    T(1, 3) = ty;
    T(2, 3) = tz;
    return T;
}

}  // namespace

// ---------------------------------------------------------------------------
// Transform roundtrip
// ---------------------------------------------------------------------------

TEST(TransformHelpers, Roundtrip_Identity) {
    double t[3] = {0, 0, 0};
    double rpy[3] = {0, 0, 0};
    auto M = mlc::mountTfToMatrix(t, rpy);
    EXPECT_TRUE(M.isIdentity(1e-12));

    double t2[3], rpy2[3];
    mlc::matrixToMountTf(M, t2, rpy2);
    for (int i = 0; i < 3; ++i) {
        EXPECT_NEAR(t2[i], 0.0, 1e-12);
        EXPECT_NEAR(rpy2[i], 0.0, 1e-12);
    }
}

TEST(TransformHelpers, Roundtrip_PureTranslation) {
    double t[3] = {1.5, -2.3, 0.7};
    double rpy[3] = {0, 0, 0};
    auto M = mlc::mountTfToMatrix(t, rpy);

    double t2[3], rpy2[3];
    mlc::matrixToMountTf(M, t2, rpy2);
    for (int i = 0; i < 3; ++i) {
        EXPECT_NEAR(t2[i], t[i], 1e-10);
        EXPECT_NEAR(rpy2[i], 0.0, 1e-10);
    }
}

TEST(TransformHelpers, Roundtrip_Combined) {
    double t[3] = {2.0, -0.4, 0.1};
    double rpy[3] = {15.0, -30.0, 45.0};
    auto M = mlc::mountTfToMatrix(t, rpy);

    double t2[3], rpy2[3];
    mlc::matrixToMountTf(M, t2, rpy2);
    for (int i = 0; i < 3; ++i) {
        EXPECT_NEAR(t2[i], t[i], 1e-10);
        EXPECT_NEAR(rpy2[i], rpy[i], 1e-10);
    }
}

TEST(TransformHelpers, Roundtrip_MultipleAngles) {
    double cases[][3] = {
        {0, 0, 90}, {45, 0, 0}, {0, 45, 0}, {10, 20, 30}, {-15, 60, -45}
    };
    double t[3] = {1, 2, 3};
    for (const auto& rpy : cases) {
        auto M = mlc::mountTfToMatrix(t, rpy);
        double t2[3], rpy2[3];
        mlc::matrixToMountTf(M, t2, rpy2);
        for (int i = 0; i < 3; ++i) {
            EXPECT_NEAR(t2[i], t[i], 1e-10);
            EXPECT_NEAR(rpy2[i], rpy[i], 1e-10);
        }
    }
}

// ---------------------------------------------------------------------------
// Range filter
// ---------------------------------------------------------------------------

TEST(PreFilter, RangeFilter_RemovesOutOfRange) {
    auto cloud = Cloud::Ptr(new Cloud);

    // Close points (distance ~0.17m)
    for (int i = 0; i < 10; ++i) {
        PointT pt; pt.x = 0.1f; pt.y = 0.1f; pt.z = 0.1f; pt.intensity = 0;
        cloud->push_back(pt);
    }
    // In-range points (distance = 5m)
    for (int i = 0; i < 50; ++i) {
        PointT pt; pt.x = 3.0f; pt.y = 4.0f; pt.z = 0.0f; pt.intensity = 0;
        cloud->push_back(pt);
    }
    // Far points (distance ~173m)
    for (int i = 0; i < 10; ++i) {
        PointT pt; pt.x = 100.0f; pt.y = 100.0f; pt.z = 100.0f; pt.intensity = 0;
        cloud->push_back(pt);
    }

    auto filtered = mlc::rangeFilter(cloud, 0.5f, 100.0f);
    EXPECT_EQ(filtered->size(), 50u);
}

TEST(PreFilter, RangeFilter_EmptyInput) {
    auto cloud = Cloud::Ptr(new Cloud);
    auto filtered = mlc::rangeFilter(cloud, 0.5f, 100.0f);
    EXPECT_TRUE(filtered->empty());
}

// ---------------------------------------------------------------------------
// Statistical outlier removal
// ---------------------------------------------------------------------------

TEST(PreFilter, StatisticalOutlierFilter_RemovesOutliers) {
    auto cloud = Cloud::Ptr(new Cloud);
    std::mt19937 rng(123);
    std::normal_distribution<float> dist(5.0f, 0.1f);

    for (int i = 0; i < 500; ++i) {
        PointT pt;
        pt.x = dist(rng); pt.y = dist(rng); pt.z = dist(rng); pt.intensity = 0;
        cloud->push_back(pt);
    }

    for (int i = 0; i < 10; ++i) {
        PointT pt;
        pt.x = 50.0f + static_cast<float>(i); pt.y = 50.0f; pt.z = 50.0f; pt.intensity = 0;
        cloud->push_back(pt);
    }

    EXPECT_EQ(cloud->size(), 510u);
    auto filtered = mlc::statisticalOutlierFilter(cloud, 30, 1.0);
    EXPECT_GT(filtered->size(), 400u);
    EXPECT_LT(filtered->size(), 510u);
}

// ---------------------------------------------------------------------------
// PreFilterCloud end-to-end
// ---------------------------------------------------------------------------

TEST(PreFilter, PreFilterCloud_CombinesBoth) {
    auto cloud = Cloud::Ptr(new Cloud);
    std::mt19937 rng(99);
    std::normal_distribution<float> dist(5.0f, 0.05f);

    for (int i = 0; i < 300; ++i) {
        PointT pt;
        pt.x = dist(rng); pt.y = dist(rng); pt.z = dist(rng); pt.intensity = 0;
        cloud->push_back(pt);
    }
    for (int i = 0; i < 20; ++i) {
        PointT pt; pt.x = 0.01f; pt.y = 0.01f; pt.z = 0.01f; pt.intensity = 0;
        cloud->push_back(pt);
    }
    PointT outlier; outlier.x = 30.0f; outlier.y = 30.0f; outlier.z = 30.0f; outlier.intensity = 0;
    cloud->push_back(outlier);

    auto filtered = mlc::preFilterCloud(cloud, 0.5f, 80.0f, 30, 1.0);
    EXPECT_GT(filtered->size(), 250u);
    EXPECT_LE(filtered->size(), 301u);
}

// ---------------------------------------------------------------------------
// Downsample
// ---------------------------------------------------------------------------

TEST(Downsample, ReducesPointCount) {
    auto cloud = makeSurface(-5, 5, -5, 5, 0.02f);
    EXPECT_GT(cloud->size(), 10000u);

    auto down = mlc::downsample(cloud, 0.5f);
    EXPECT_GT(down->size(), 10u);
    EXPECT_LT(down->size(), cloud->size());
}

TEST(Downsample, EmptyInput) {
    auto cloud = Cloud::Ptr(new Cloud);
    auto down = mlc::downsample(cloud, 0.1f);
    EXPECT_TRUE(down->empty());
}

// ---------------------------------------------------------------------------
// Single-pass GICP: recover a known small transform
// ---------------------------------------------------------------------------

TEST(Gicp, SinglePass_RecoverKnownTransform) {
    auto source = makeSurface(-3, 3, -3, 3, 0.05f, 0.5f, 10);
    Eigen::Matrix4f T_true = makeTransform(0.3f, 0.2f, 0.05f, 3.0f);

    Cloud::Ptr target(new Cloud);
    pcl::transformPointCloud(*source, *target, T_true);

    auto result = mlc::runSingleGicp(
        source, target, 0.1f, 100, 2.0f, Eigen::Matrix4f::Identity());

    ASSERT_TRUE(result.converged);
    EXPECT_LT(result.fitness, 0.1);

    Eigen::Vector3f t_err = result.transform.cast<float>().block<3,1>(0,3) - T_true.block<3,1>(0,3);
    EXPECT_LT(t_err.norm(), 0.15f);
}

// ---------------------------------------------------------------------------
// Coarse-to-fine GICP: recover a larger transform
// ---------------------------------------------------------------------------

TEST(Gicp, CoarseToFine_RecoverLargerTransform) {
    auto source = makeSurface(-5, 5, -5, 5, 0.05f, 0.5f, 20);
    Eigen::Matrix4f T_true = makeTransform(0.5f, 0.3f, 0.1f, 5.0f);

    Cloud::Ptr target(new Cloud);
    pcl::transformPointCloud(*source, *target, T_true);

    auto result = mlc::runCoarseToFineGicp(
        source, target, 0.1f, 100, 2.0f, Eigen::Matrix4f::Identity());

    ASSERT_TRUE(result.converged);
    EXPECT_LT(result.fitness, 0.1);

    Eigen::Vector3f t_err = result.transform.cast<float>().block<3,1>(0,3) - T_true.block<3,1>(0,3);
    EXPECT_LT(t_err.norm(), 0.15f);
}

TEST(Gicp, CoarseToFine_BetterThanSinglePass) {
    auto source = makeSurface(-5, 5, -5, 5, 0.05f, 0.5f, 30);
    Eigen::Matrix4f T_true = makeTransform(1.0f, 0.5f, 0.0f, 8.0f);

    Cloud::Ptr target(new Cloud);
    pcl::transformPointCloud(*source, *target, T_true);

    auto single = mlc::runSingleGicp(
        source, target, 0.1f, 100, 2.0f, Eigen::Matrix4f::Identity());
    auto ctf = mlc::runCoarseToFineGicp(
        source, target, 0.1f, 100, 2.0f, Eigen::Matrix4f::Identity());

    ASSERT_TRUE(ctf.converged);
    EXPECT_LE(ctf.fitness, single.fitness + 0.01);
}

// ---------------------------------------------------------------------------
// Fitness threshold gate
// ---------------------------------------------------------------------------

TEST(FitnessGate, HighFitnessOnNonOverlappingClouds) {
    auto cloud_a = makeSurface(-3, -1, -3, -1, 0.1f, 0.3f, 40);
    auto cloud_b = makeSurface(10, 12, 10, 12, 0.1f, 0.3f, 41);

    auto result = mlc::runSingleGicp(
        cloud_a, cloud_b, 0.2f, 50, 5.0f, Eigen::Matrix4f::Identity());

    EXPECT_GT(result.fitness, 0.3);
}

// ---------------------------------------------------------------------------
// Iterative multi-lidar refinement (3 clouds)
// ---------------------------------------------------------------------------

TEST(Refinement, ThreeClouds_ImprovesAlignment) {
    auto base = makeSurface(-5, 5, -5, 5, 0.05f, 0.5f, 50);

    Eigen::Matrix4d T_B_true = Eigen::Matrix4d::Identity();
    T_B_true(0, 3) = 1.0;
    Eigen::Matrix4d T_C_true = Eigen::Matrix4d::Identity();
    T_C_true(1, 3) = 1.0;

    Cloud::Ptr cloud_B(new Cloud), cloud_C(new Cloud);
    pcl::transformPointCloud(*base, *cloud_B, T_B_true.inverse().cast<float>());
    pcl::transformPointCloud(*base, *cloud_C, T_C_true.inverse().cast<float>());

    Eigen::Matrix4d T_B_init = T_B_true;
    T_B_init(0, 3) += 0.15;
    Eigen::Matrix4d T_C_init = T_C_true;
    T_C_init(1, 3) += 0.15;

    std::vector<mlc::LidarPose> poses = {
        {"A", base,    Eigen::Matrix4d::Identity(), true},
        {"B", cloud_B, T_B_init,                    false},
        {"C", cloud_C, T_C_init,                    false},
    };

    auto refined = mlc::iterativeMultiLidarRefinement(
        poses, 0.1f, 100, 2.0f, 1.0f, 5, 1e-4);

    double err_B_before = (T_B_init.block<3,1>(0,3) - T_B_true.block<3,1>(0,3)).norm();
    double err_C_before = (T_C_init.block<3,1>(0,3) - T_C_true.block<3,1>(0,3)).norm();

    double err_B_after = 1e9, err_C_after = 1e9;
    for (const auto& p : refined) {
        if (p.name == "B")
            err_B_after = (p.T.block<3,1>(0,3) - T_B_true.block<3,1>(0,3)).norm();
        if (p.name == "C")
            err_C_after = (p.T.block<3,1>(0,3) - T_C_true.block<3,1>(0,3)).norm();
    }

    EXPECT_LT(err_B_after, err_B_before);
    EXPECT_LT(err_C_after, err_C_before);
}

TEST(Refinement, TwoLidars_ReturnsUnchanged) {
    auto cloud = makeSurface(-2, 2, -2, 2, 0.1f);

    std::vector<mlc::LidarPose> poses = {
        {"A", cloud, Eigen::Matrix4d::Identity(), true},
        {"B", cloud, Eigen::Matrix4d::Identity(), false},
    };

    auto refined = mlc::iterativeMultiLidarRefinement(
        poses, 0.1f, 50, 2.0f, 1.0f, 5);

    EXPECT_EQ(refined.size(), 2u);
    EXPECT_TRUE(refined[0].T.isApprox(Eigen::Matrix4d::Identity(), 1e-10));
}

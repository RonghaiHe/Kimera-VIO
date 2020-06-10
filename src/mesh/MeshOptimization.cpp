/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   MeshOptimization.cpp
 * @brief  Optimizes vertices of a 3D mesh given depth data on a projective
 * setting (depth map, rgb-d, lidar).
 * @author Antoni Rosinol
 */

#include "kimera-vio/mesh/MeshOptimization.h"

#include <string>
#include <vector>

#include <Eigen/Core>

#include <opencv2/opencv.hpp>
// To convert from/to eigen
#include <opencv2/core/eigen.hpp>

#include <gtsam/geometry/Cal3_S2.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/linear/GaussianFactorGraph.h>
#include <gtsam/nonlinear/Marginals.h>

#include "kimera-vio/common/vio_types.h"
#include "kimera-vio/frontend/StereoCamera.h"
#include "kimera-vio/mesh/Mesh.h"
#include "kimera-vio/mesh/MeshOptimization-definitions.h"
#include "kimera-vio/mesh/MeshUtils.h"
#include "kimera-vio/mesh/Mesher-definitions.h"
#include "kimera-vio/utils/Macros.h"
#include "kimera-vio/utils/UtilsOpenCV.h"
#include "kimera-vio/visualizer/OpenCvVisualizer3D.h"

namespace VIO {

MeshOptimization::MeshOptimization(const MeshOptimizerType& solver_type,
                                   const MeshColorType& mesh_color_type,
                                   StereoCamera::Ptr stereo_camera,
                                   OpenCvVisualizer3D::Ptr visualizer)
    : visualizer_(visualizer),
      mesh_optimizer_type_(solver_type),
      stereo_camera_(stereo_camera),
      window_("Mesh Optimization"),
      mesh_color_type_(mesh_color_type) {
  window_.setBackgroundColor(cv::viz::Color::white());
  window_.setFullScreen(true);
}

MeshOptimizationOutput::UniquePtr MeshOptimization::spinOnce(
    const MeshOptimizationInput& input) {
  return solveOptimalMesh(input.noisy_point_cloud, input.mesh_2d);
}

void MeshOptimization::draw2dMeshOnImg(const Mesh2D& mesh_2d,
                                       cv::Mat* img,
                                       const cv::viz::Color& color,
                                       const size_t& thickness,
                                       const int line_type) {
  CHECK_NOTNULL(img);
  CHECK_EQ(mesh_2d.getMeshPolygonDimension(), 3u);
  CHECK_GT(mesh_2d.getNumberOfPolygons(), 0u);
  // Draw the pixel on the image
  Mesh2D::Polygon polygon;
  for (size_t k = 0u; k < mesh_2d.getNumberOfPolygons(); k++) {
    CHECK(mesh_2d.getPolygon(k, &polygon));
    const Vertex2D& v0 = polygon.at(0).getVertexPosition();
    const Vertex2D& v1 = polygon.at(1).getVertexPosition();
    const Vertex2D& v2 = polygon.at(2).getVertexPosition();
    cv::line(*img, v0, v1, color, thickness, line_type);
    cv::line(*img, v1, v2, color, thickness, line_type);
    cv::line(*img, v2, v0, color, thickness, line_type);
  }
}

void MeshOptimization::draw3dMesh(const std::string& id,
                                  const Mesh3D& mesh_3d,
                                  bool display_as_wireframe,
                                  const double& opacity) {
  cv::Mat vertices_mesh;
  cv::Mat polygons_mesh;
  mesh_3d.getVerticesMeshToMat(&vertices_mesh);
  mesh_3d.getPolygonsMeshToMat(&polygons_mesh);
  cv::Mat colors_mesh = mesh_3d.getColorsMesh().t();  // Note the transpose.
  if (colors_mesh.empty()) {
    colors_mesh = cv::Mat(1u,
                          mesh_3d.getNumberOfUniqueVertices(),
                          CV_8UC3,
                          cv::viz::Color::yellow());
  }

  // Build visual mesh
  cv::viz::Mesh cv_mesh;
  cv_mesh.cloud = vertices_mesh.t();
  cv_mesh.polygons = polygons_mesh;
  cv_mesh.colors = colors_mesh;

  // Build widget mesh
  cv::viz::WMesh widget_cv_mesh(cv_mesh);
  widget_cv_mesh.setRenderingProperty(cv::viz::SHADING, cv::viz::SHADING_FLAT);
  widget_cv_mesh.setRenderingProperty(cv::viz::AMBIENT, 0);
  widget_cv_mesh.setRenderingProperty(cv::viz::LIGHTING, 1);
  widget_cv_mesh.setRenderingProperty(cv::viz::OPACITY, opacity);
  if (display_as_wireframe) {
    widget_cv_mesh.setRenderingProperty(cv::viz::REPRESENTATION,
                                        cv::viz::REPRESENTATION_WIREFRAME);
  }
  window_.showWidget(id.c_str(), widget_cv_mesh);
}

void MeshOptimization::collectTriangleDataPointsFast(
    const cv::Mat& noisy_point_cloud,
    const Mesh2D& mesh_2d,
    TriangleToDatapoints* triangles_to_datapoints_xyz,
    TriangleToPixels* triangles_to_datapoints_pixels,
    size_t* number_of_valid_datapoints) {
  // Assumes ordered point cloud
  const size_t& img_height = noisy_point_cloud.rows;
  const size_t& img_width = noisy_point_cloud.cols;
  const size_t& n_polys = mesh_2d.getNumberOfPolygons();
  CHECK_GT(n_polys, 0u);
  CHECK_NOTNULL(triangles_to_datapoints_xyz)->reserve(n_polys);
  CHECK_NOTNULL(triangles_to_datapoints_pixels)->reserve(n_polys);
  *CHECK_NOTNULL(number_of_valid_datapoints) = 0u;

  Mesh2D::Polygon polygon;
  for (size_t k = 0u; k < n_polys; k++) {
    CHECK(mesh_2d.getPolygon(k, &polygon));
    const Vertex2D& vtx1 = polygon.at(0).getVertexPosition();
    const Vertex2D& vtx2 = polygon.at(1).getVertexPosition();
    const Vertex2D& vtx3 = polygon.at(2).getVertexPosition();

    // Loop over pixels inside the bounding box of the current triangle

    // 1. Find bounding box of triangle
    float xmin = min3(vtx1.x, vtx2.x, vtx3.x);
    float ymin = min3(vtx1.y, vtx2.y, vtx3.y);

    float xmax = max3(vtx1.x, vtx2.x, vtx3.x);
    float ymax = max3(vtx1.y, vtx2.y, vtx3.y);

    // Discard triangle out of screen
    // WARNING bcs of cv::Point2f convention x is the width...
    if (xmin > img_width - 1 || xmax < 0 || ymin > img_height - 1 || ymax < 0) {
      LOG(ERROR) << "Triangle out of screen!:"
                 << "xmin: " << xmin << '\n'
                 << "xmax: " << xmax << '\n'
                 << "ymin: " << ymin << '\n'
                 << "ymax: " << ymax << '\n'
                 << "img_height" << img_height - 1 << '\n'
                 << "img_width" << img_width - 1;
      continue;
    }

    // be careful xmin/xmax/ymin/ymax can be negative. Don't cast to uint32_t
    uint32_t x0 = std::max(int32_t(0), (int32_t)(std::floor(xmin)));
    uint32_t x1 = std::min(int32_t(img_width) - 1, (int32_t)(std::floor(xmax)));
    uint32_t y0 = std::max(int32_t(0), (int32_t)(std::floor(ymin)));
    uint32_t y1 =
        std::min(int32_t(img_height) - 1, (int32_t)(std::floor(ymax)));

    // 2. Loop over pixels in the bounding box
    /// Cache stuff
    float x12 = vtx1.x - vtx2.x;
    float y12 = vtx1.y - vtx2.y;
    float x23 = vtx2.x - vtx3.x;
    float y23 = vtx2.y - vtx3.y;
    float x31 = vtx3.x - vtx1.x;
    float y31 = vtx3.y - vtx1.y;
    for (uint32_t u = x0; u <= x1; ++u) {
      for (uint32_t v = y0; v <= y1; ++v) {
        // Check that pixel is in triangle
        float d1, d2, d3;
        bool has_neg, has_pos;

        d1 = (u - vtx2.x) * y12 - x12 * (v - vtx2.y);
        d2 = (u - vtx3.x) * y23 - x23 * (v - vtx3.y);
        d3 = (u - vtx1.x) * y31 - x31 * (v - vtx1.y);

        has_neg = (d1 < 0) || (d2 < 0) || (d3 < 0);
        has_pos = (d1 > 0) || (d2 > 0) || (d3 > 0);

        KeypointCV pixel = cv::Point2f(u, v);
        CHECK_EQ(!(has_neg && has_pos),
                 pointInTriangle(pixel, vtx1, vtx2, vtx3));
        if (!(has_neg && has_pos)) {
          // Point in triangle
          const cv::Point3f& lmk = noisy_point_cloud.at<cv::Point3f>(pixel);
          if (isValidPoint(lmk) && lmk.z <= kMaxZ && lmk.z >= kMinZ) {
            (*triangles_to_datapoints_xyz)[k].push_back(lmk);
            (*triangles_to_datapoints_pixels)[k].push_back(pixel);
            ++(*number_of_valid_datapoints);
          }
        }
      }  // loop over y
    }    // loop over x
  }
}

void MeshOptimization::collectTriangleDataPoints(
    const cv::Mat& noisy_point_cloud,
    const Mesh2D& mesh_2d,
    TriangleToDatapoints* corresp,
    TriangleToPixels* pixel_corresp,
    size_t* number_of_valid_datapoints) {
  CHECK_NOTNULL(corresp);
  CHECK_NOTNULL(pixel_corresp);
  *CHECK_NOTNULL(number_of_valid_datapoints) = 0u;
  CHECK(stereo_camera_);

  for (size_t v = 0u; v < noisy_point_cloud.rows; ++v) {
    for (size_t u = 0u; u < noisy_point_cloud.cols; ++u) {
      // 1. Project pointcloud to image (color img with projections)
      // aka get pixel coordinates for all points in pointcloud.
      // TODO(Toni): the projection of all points could be greatly optimized
      // by
      // appending all points into a big matrix and performing dense
      // multiplication.
      KeypointCV pixel = cv::Point2f(u, v);
      const cv::Point3f& lmk = noisy_point_cloud.at<cv::Point3f>(pixel);
      if (isValidPoint(lmk) && lmk.z <= kMaxZ && lmk.z >= kMinZ) {
        // Convert xyz to global coords (as they are given in camera coords).
        const gtsam::Pose3& left_cam_rect_pose =
            stereo_camera_->getBodyPoseLeftCamRect();
        // transform from left cam frame of reference to body because the
        // stereo camera projection function expects landmarks in the body
        // frame of reference!
        const gtsam::Point3& pt_body = left_cam_rect_pose.transformFrom(
            gtsam::Point3(lmk.x, lmk.y, lmk.z));
        LandmarkCV lmk_cv;
        lmk_cv.x = pt_body.x();
        lmk_cv.y = pt_body.y();
        lmk_cv.z = pt_body.z();

        KeypointCV left_pixel;
        KeypointCV right_pixel;
        stereo_camera_->project(lmk_cv, &left_pixel, &right_pixel);
        CHECK_NEAR(left_pixel.x, static_cast<double>(u), 0.001);
        CHECK_NEAR(left_pixel.y, static_cast<double>(v), 0.001);

        if (visualizer_) {
          // drawPixelOnImg(left_pixel, img_, cv::viz::Color::green(), 1u);
        }

        // 2. Generate correspondences btw points and triangles.
        // For each triangle in 2d Mesh
        // TODO(Toni): this can be greatly optimized by going on a per triangle
        // fashion and using halfplane checks on all points.
        Mesh2D::Polygon polygon;
        for (size_t k = 0; k < mesh_2d.getNumberOfPolygons(); k++) {
          CHECK(mesh_2d.getPolygon(k, &polygon));
          if (pointInTriangle(left_pixel,
                              polygon.at(0).getVertexPosition(),
                              polygon.at(1).getVertexPosition(),
                              polygon.at(2).getVertexPosition())) {
            // A point should only be in one triangle, once found, we can stop
            // looping over the 2d mesh.
            (*corresp)[k].push_back(lmk);
            (*pixel_corresp)[k].push_back(left_pixel);
            ++(*number_of_valid_datapoints);
            break;
          }
        }
      }
    }
  }
}

MeshOptimizationOutput::UniquePtr MeshOptimization::solveOptimalMesh(
    const cv::Mat& noisy_point_cloud,
    const Mesh2D& mesh_2d) {
  CHECK_GT(mesh_2d.getNumberOfPolygons(), 0);
  CHECK_GT(mesh_2d.getNumberOfUniqueVertices(), 0);
  CHECK_EQ(noisy_point_cloud.channels(), 3u);
  CHECK(stereo_camera_);

  // Need to visualizeScene again because the image of the camera frustum
  // was updated
  if (visualizer_) {
    drawPointCloud("Noisy Point Cloud",
                   noisy_point_cloud,
                   UtilsOpenCV::gtsamPose3ToCvAffine3d(
                       stereo_camera_->getBodyPoseLeftCamRect()));
    // draw2dMeshOnImg(img_, mesh_2d);
    drawScene(stereo_camera_->getBodyPoseLeftCamRect(),
              stereo_camera_->getStereoCalib()->calibration());
    // spinDisplay();
  }

  /// Step 1: Collect all datapoints that fall within triangle
  LOG(INFO) << "Collecting triangle data points.";
  TriangleToDatapoints triangles_to_datapoints_xyz;  // In left_cam_rect coords
  TriangleToPixels triangles_to_datapoints_pixels;
  size_t number_of_valid_datapoints = 0;
  collectTriangleDataPointsFast(noisy_point_cloud,
                                mesh_2d,
                                &triangles_to_datapoints_xyz,
                                &triangles_to_datapoints_pixels,
                                &number_of_valid_datapoints);

  CHECK_GT(number_of_valid_datapoints, 3u);
  CHECK_GT(triangles_to_datapoints_xyz.size(), 0u);
  LOG_IF(ERROR,
         triangles_to_datapoints_xyz.size() != mesh_2d.getNumberOfPolygons())
      << "Every triangle should have some data points! (or maybe not "
         "really)";
  CHECK_EQ(triangles_to_datapoints_xyz.size(),
           triangles_to_datapoints_pixels.size());

  /// Step 2: Solve for the ys and build the Y matrix incrementally by
  /// looping over all triangles.
  LOG(INFO) << "Building optimization problem.";
  // This matrix has as columns the landmark ids, and as rows the ys of each
  // datapoint, with non-zeros only where a datapoint is associated to a
  // lmk.
  typedef std::unordered_map<Mesh2D::VertexId, Vertex3D> VtxIdToBearingVector;
  typedef std::unordered_map<Mesh2D::VertexId, Vertex2D> VtxIdToPixels;
  VtxIdToBearingVector vtx_ids_to_bearing_vectors;
  VtxIdToPixels vtx_ids_to_pixels;

  // Mesh that will hold the reconstructed one
  Mesh3D reconstructed_mesh;

  // Create empty graph
  gtsam::GaussianFactorGraph factor_graph;

  // For each triangle in 2d Mesh (as long as non-degenerate).
  Mesh2D::Polygon polygon_2d;
  LOG_IF(ERROR,
         mesh_2d.getNumberOfPolygons() != triangles_to_datapoints_xyz.size())
      << "There are some undetermined triangles in the 2d mesh.";
  // VIT this needs to be static or out of the for loop above!
  // otw we overwrite previous rows in Y.
  std::map<gtsam::Key, size_t> vertex_supports;  //! Only for visualization.
  for (size_t tri_idx = 0; tri_idx < mesh_2d.getNumberOfPolygons(); tri_idx++) {
    CHECK(mesh_2d.getPolygon(tri_idx, &polygon_2d));
    CHECK_EQ(polygon_2d.size(), 3);

    /// Step 2.1: Build bearing vector matrix of triangle and collect
    /// landmark
    /// ids.
    // List of landmark ids associated to the triangle vertices
    std::array<Mesh2D::VertexId, 3> vtx_ids;
    // 1 row with 3 columns: each column is a cv::Point3f
    // TODO(Toni): we are recalculating bearing vectors all the time
    // we should be calculating these only once...
    cv::Mat_<Vertex3D> triangle_bearing_vectors(1, 3);
    size_t col = 0u;
    for (const auto& vtx : polygon_2d) {
      // Calculate bearing vectors from mesh 2d
      // vtx is pixel, get in bearing vectors?
      const Vertex2D& vtx_pixel = vtx.getVertexPosition();
      // This gets you the bearing vector in body coordinates
      cv::Point3f cv_bearing_vector_body_frame;
      getBearingVectorFrom2DPixel(vtx_pixel, &cv_bearing_vector_body_frame);

      if (visualizer_) {
        drawArrow(cv::Point3d(UtilsOpenCV::gtsamVector3ToCvMat(
                      stereo_camera_->getBodyPoseLeftCamRect().translation())),
                  cv_bearing_vector_body_frame,
                  "r" + std::to_string(tri_idx * 3 + col),
                  false,
                  0.001,
                  0.001,
                  cv::viz::Color::red());
      }
      Mesh2D::VertexId vtx_id;
      CHECK(mesh_2d.getVtxIdForLmkId(vtx.getLmkId(), &vtx_id));
      triangle_bearing_vectors[0][col] = cv_bearing_vector_body_frame;
      vtx_ids_to_bearing_vectors[vtx_id] = cv_bearing_vector_body_frame;
      vtx_ids_to_pixels[vtx_id] = vtx_pixel;
      vtx_ids[col] = vtx_id;
      col++;
    }

    /// Step 2.2: For each datapoint in current triangle solve for the ys
    /// and build big Y matrix
    // TODO(Toni): check for other degenerate configs such as all points
    // on a
    // line, or same spot or... We should use regularization to deal with
    // these
    // or maybe not if its neighbour triangles are fine.
    std::vector<cv::Point3f> triangle_datapoints_xyz_left_rect_cam_frame =
        triangles_to_datapoints_xyz[tri_idx];
    //! Pixels associated to a triangle that have a depth value (datapoint,
    //! measurements)
    KeypointsCV triangle_datapoints_pixel =
        triangles_to_datapoints_pixels[tri_idx];
    CHECK_EQ(triangle_datapoints_xyz_left_rect_cam_frame.size(),
             triangle_datapoints_pixel.size());

    // Skip under-constrained since we do not have enough info to solve Ay=b
    if (triangle_datapoints_xyz_left_rect_cam_frame.size() < 3) {
      // ALTHOUGH WE COULD JUST USE AS PRIOR THE CURRENT DEPTH SAMPLE...
      LOG(ERROR) << "Degenerate case optimization problem, we need more than 3 "
                    "datapoints: offending triangle idx: "
                 << tri_idx;
      continue;
    }

    LOG(INFO) << "Adding " << triangle_datapoints_xyz_left_rect_cam_frame.size()
              << " datapoints to triangle with idx: " << tri_idx;
    switch (mesh_optimizer_type_) {
      case MeshOptimizerType::kGtsamMesh: {
        // Build factor graph on a per triangle basis
        for (size_t i = 0u; i < triangle_datapoints_pixel.size(); i++) {
          const KeypointCV& pixel = triangle_datapoints_pixel[i];
          const cv::Point3f& lmk =
              triangle_datapoints_xyz_left_rect_cam_frame[i];

          // In principle, datapoints here are all valid, as filtered by the
          // collect data points function
          double inv_depth_meas = 1.0 / std::sqrt(lmk.dot(lmk));
          // These should not be recomputed but cached when computing triangle
          // rasterization pixels...
          BaryCoord b0, b1, b2;
          if (!barycentricCoordinates(vtx_ids_to_pixels[vtx_ids[0]],
                                      vtx_ids_to_pixels[vtx_ids[1]],
                                      vtx_ids_to_pixels[vtx_ids[2]],
                                      pixel,
                                      &b0,
                                      &b1,
                                      &b2)) {
            // This fails for datapoints on the vertices, which I guess is good?
            // Although we are discarding the actual depth of this point!
            LOG(ERROR) << "Query pixel: " << pixel << '\n'
                       << "vtx1 pixel: " << vtx_ids_to_pixels[vtx_ids[0]]
                       << '\n'
                       << "vtx2 pixel: " << vtx_ids_to_pixels[vtx_ids[1]]
                       << '\n'
                       << "vtx3 pixel: " << vtx_ids_to_pixels[vtx_ids[2]]
                       << '\n'
                       << "Outside triangle ";
          }

          //! Construct ternary factors  and them to factor graph
          gtsam::Key i1(vtx_ids[0]);
          gtsam::Matrix11 A1(b0);  //! barycentric coordinates of vtx 0
          gtsam::Key i2(vtx_ids[1]);
          gtsam::Matrix11 A2(b1);  //! barycentric coordinates of vtx 1
          gtsam::Key i3(vtx_ids[2]);
          gtsam::Matrix11 A3(b2);  //! barycentric coordinates of vtx 2
          //! Inverse depth of datapoint
          gtsam::Vector1 b(inv_depth_meas);
          gtsam::SharedDiagonal noise_model_input =
              gtsam::noiseModel::Diagonal::Sigmas(
                  gtsam::Vector1(kDepthMeasNoiseSigma));

          //! one per data point influencing three variables
          factor_graph += gtsam::JacobianFactor(
              i1, A1, i2, A2, i3, A3, b, noise_model_input);

          //! Count the number of points that support the vertices (only for
          //! support visualization of the mesh).
          vertex_supports[i1] += 1;
          vertex_supports[i2] += 1;
          vertex_supports[i3] += 1;
        }
      } break;
      default: { LOG(FATAL) << "Unknown mesh optimization type."; } break;
    }

    // check that the non-zero entries correspond to the adjacency
    // between
    // of the 2d mesh on a per row basis. I.e. each row should have
    // non-zero
    // entries only for cols (vtx_ids) which correspond to vtx_ids of an
    // actual triangle.
  }

  /// At this point you could drop columns (lmk_ids) for those that do
  /// not
  /// have enough measurements (although even with 0 measurements you
  /// should
  /// be able to constraint if neihbors have enough measurements).

  LOG(INFO) << "Solving optimization problem.";
  switch (mesh_optimizer_type_) {
    case MeshOptimizerType::kGtsamMesh: {
      if (kUseSpringEnergies) {
        //! Add spring energies for this triangle, but don't duplicate
        //! springs! Hence, use adjacency matrix to know where to put the
        //! springs.
        cv::Mat adjacency_matrix = mesh_2d.getAdjacencyMatrix();
        const gtsam::Vector1 kSpringRestLength(0);
        constexpr double kSpringConstant = 1.0;
        const gtsam::Matrix11 A1(kSpringConstant);
        const gtsam::Matrix11 A2(-1.0 * kSpringConstant);
        const gtsam::SharedDiagonal kSpringNoiseModel =
            gtsam::noiseModel::Diagonal::Sigmas(
                gtsam::Vector1(kSpringNoiseSigma));
        // ASSUMEs that vtx ids are the indices of the adjacency matrix!
        for (size_t i = 0u; i < adjacency_matrix.rows; i++) {
          gtsam::Key i1(i);
          for (size_t j = 0u; j < adjacency_matrix.cols; j++) {
            if (j < i) {
              if (adjacency_matrix.at<uint8_t>(i, j) == 1u) {
                // Vertices are connected!
                gtsam::Key i2(j);
                factor_graph += gtsam::JacobianFactor(
                    i1, A1, i2, A2, kSpringRestLength, kSpringNoiseModel);
              }
            } else {
              CHECK_EQ(j, i);
              // The matrix is symmetric, avoid adding duplicated springs.
              break;
            }
          }
        }
      }

      // Solve linear factor graph Ax=b...
      // optimize the graph
      gtsam::VectorValues actual =
          factor_graph.optimize(boost::none, gtsam::EliminateQR);
      actual.print("Values after optimization");

      gtsam::VectorValues hessian = factor_graph.hessianDiagonal();

      // Find the max std deviation, just for visualization of variances
      // later.
      // double max_inv_depth = -std::numeric_limits<double>::max();
      // for (const auto& kv : actual) {
      //  if (kv.second[0] > max_inv_depth) max_inv_depth = kv.second[0];
      //}
      // double max_inv_variance_of_inv_depth =
      //    -std::numeric_limits<double>::max();
      // for (const auto& kv : hessian) {
      //  if (kv.second[0] > max_inv_depth) max_inv_depth = kv.second[0];
      //}
      // const double& max_variance_of_inv_depth =
      //    1.0 / max_inv_variance_of_inv_depth;
      // const double& max_variance_of_depth =
      //    max_variance_of_inv_depth *
      //    (1.0 / std::pow(max_inv_depth, 2));
      // const double& max_std_deviation = std::sqrt(max_variance_of_depth);

      size_t max_vertex_support = 0u;
      for (auto it = vertex_supports.cbegin(); it != vertex_supports.cend();
           ++it) {
        if (it->second > max_vertex_support) {
          max_vertex_support = it->second;
        }
      }

      // Add new polygons to reconstructed mesh
      Mesh2D::Polygon poly_2d;
      for (size_t k = 0u; k < mesh_2d.getNumberOfPolygons(); k++) {
        CHECK(mesh_2d.getPolygon(k, &poly_2d));
        Mesh3D::Polygon poly_3d;
        poly_3d.reserve(poly_2d.size());
        bool add_poly = true;
        for (const Mesh2D::VertexType& vtx_2d : poly_2d) {
          const LandmarkId& lmk_id = vtx_2d.getLmkId();
          Mesh2D::VertexId vtx_id;
          CHECK(mesh_2d.getVtxIdForLmkId(lmk_id, &vtx_id));
          if (!actual.exists(gtsam::Key(vtx_id))) {
            LOG(ERROR) << "vtx_id: " << vtx_id << " is not in optimization.";
            add_poly = false;
            break;
          }

          const double& inv_depth = actual.at(gtsam::Key(vtx_id))[0];
          if (std::isinf(inv_depth)) {
            LOG(ERROR) << "vtx_id: " << vtx_id << " goes to +/-inf.";
            add_poly = false;
            break;
          }

          //! Calculate depth estimation variance
          // TODO(Toni): check that these divisions are not on 0;
          const double& inv_variance_of_inv_depth =
              hessian.at(gtsam::Key(vtx_id))[0];
          const double& variance_of_inv_depth = 1.0 / inv_variance_of_inv_depth;
          const double& variance_of_depth =
              variance_of_inv_depth * (1.0 / std::pow(inv_depth, 2));

          //! Calculate depth estimation
          const double& depth = 1.0 / inv_depth;
          const cv::Point3f& cv_bearing_vector_body_frame =
              vtx_ids_to_bearing_vectors[vtx_id];
          const Vertex3D& lmk = depth * cv_bearing_vector_body_frame;

          //! Plot confidence intervals on pixel rays.
          const double& std_deviation = std::sqrt(variance_of_depth);
          const Vertex3D& lmk_max =
              (depth + std_deviation) * cv_bearing_vector_body_frame;
          const Vertex3D& lmk_min =
              (depth - std_deviation) * cv_bearing_vector_body_frame;
          //! Display cylinder oriented with ray (the width is irrelevant)
          //! only the distance along the ray is meaningful!
          //! TODO(Toni): this is in world coordinates!! Not in cam coords!
          //! But right-now everything is the same...
          drawCylinder("Variance for Lmk: " + std::to_string(lmk_id),
                       lmk_max,
                       lmk_min,
                       0.01,
                       30,
                       cv::viz::Color::azure());

          //! Add new vertex to polygon
          //! Color with covariance bgr:
          static constexpr double kScaleStdDeviation = 0.1;
          cv::viz::Color vtx_color = cv::viz::Color::black();
          switch (mesh_color_type_) {
            case MeshColorType::kVertexFlatColor: {
              // Use color of each pixel where the landmark is
              switch (mesh_count_ % 5) {
                case 0:
                  vtx_color = cv::viz::Color::red();
                  break;
                case 1:
                  vtx_color = cv::viz::Color::apricot();
                  break;
                case 2:
                  vtx_color = cv::viz::Color::purple();
                  break;
                case 3:
                  vtx_color = cv::viz::Color::brown();
                  break;
                case 4:
                  vtx_color = cv::viz::Color::pink();
                  break;
              }
            } break;
            case MeshColorType::kVertexRGB: {
              // Use textures in the image
            } break;
            case MeshColorType::kVertexDepthVariance: {
              // Use variances of the vertices
              vtx_color = cv::Scalar(
                  0,
                  0,
                  std::round(std_deviation / kScaleStdDeviation * 255.0),
                  255);
            } break;
            case MeshColorType::kVertexSupport: {
              // Use the number of datapoints that support this vertex
              vtx_color =
                  cv::Scalar(std::round(vertex_supports[gtsam::Key(vtx_id)] /
                                        max_vertex_support * 255.0),
                             0,
                             0,
                             255);
            } break;
            default: { LOG(FATAL) << "Unrecognized mesh color type."; }
          }
          poly_3d.push_back(Mesh3D::VertexType(lmk_id, lmk, vtx_color));
        }
        if (add_poly) {
          reconstructed_mesh.addPolygonToMesh(poly_3d);
        } else {
          LOG(WARNING) << "Non-reconstructed poly: " << k;
        }
      }

      break;
    }
    default: { LOG(FATAL) << "Unknown mesh optimization type."; } break;
  }

  // Display reconstructed mesh.
  if (visualizer_) {
    LOG(INFO) << "Drawing optimized reconstructed mesh...";
    draw3dMesh("Reconstructed Mesh " + std::to_string(mesh_count_),
               reconstructed_mesh,
               false,
               0.9);
    // spinDisplay();
  }
  MeshOptimizationOutput::UniquePtr output =
      VIO::make_unique<MeshOptimizationOutput>();
  output->optimized_mesh_3d = reconstructed_mesh;
  mesh_count_++;
  return output;
}

cv::Point2f MeshOptimization::generatePixelFromLandmarkGivenCamera(
    const cv::Point3f& lmk,
    const gtsam::Pose3& extrinsics,
    const gtsam::Cal3_S2& intrinsics) {
  // Project 3D landmarks to camera image plane and get pixels.
  // (Sub-)Pixel coordinates
  gtsam::Vector3 v(lmk.x, lmk.y, lmk.z);
  gtsam::Point3 pixel = intrinsics.K() * extrinsics.transformTo(v);
  // cv::Point has inverted row/col wrt to cv::Mat!
  CHECK_GT(pixel.z(), 0.0);
  return cv::Point2f(pixel.x() / pixel.z(), pixel.y() / pixel.z());
}

void MeshOptimization::getBearingVectorFrom2DPixel(
    const cv::Point2f& pixel,
    cv::Point3f* bearing_vector_body_frame) {
  CHECK_NOTNULL(bearing_vector_body_frame);
  CHECK(stereo_camera_ != nullptr);
  LandmarkCV lmk;
  // This in turn transforms the bearing vector to the stereo camera frame of
  // reference. Therefore, bearing vector is expressed in the body frame of ref.
  stereo_camera_->backProjectDepth(pixel, 1.0, &lmk);
  *bearing_vector_body_frame = lmk / std::sqrt(lmk.dot(lmk));
  CHECK_NEAR(bearing_vector_body_frame->dot(*bearing_vector_body_frame),
             1.0f,
             0.0001f);
}

void MeshOptimization::getBearingVectorFrom3DLmk(const gtsam::Pose3& extrinsics,
                                                 const cv::Point3f& lmk,
                                                 cv::Point3f* bearing_vector,
                                                 float* inverse_depth) {
  CHECK_NOTNULL(bearing_vector);
  CHECK_NOTNULL(inverse_depth);
  const gtsam::Point3& ray =
      extrinsics.transformTo(gtsam::Vector3(lmk.x, lmk.y, lmk.z));
  const double& norm = ray.norm();
  CHECK_GT(norm, 0.0);
  *inverse_depth = 1 / norm;
  // Divide ray by its length get the normalized bearing vector.
  const gtsam::Point3& bearing = *inverse_depth * ray;
  *bearing_vector = cv::Point3d(bearing.x(), bearing.y(), bearing.z());
}

float MeshOptimization::sign(const cv::Point2f& p1,
                             const cv::Point2f& p2,
                             const cv::Point2f& p3) {
  return (p1.x - p3.x) * (p2.y - p3.y) - (p2.x - p3.x) * (p1.y - p3.y);
}

bool MeshOptimization::pointInTriangle(const cv::Point2f& pt,
                                       const cv::Point2f& v1,
                                       const cv::Point2f& v2,
                                       const cv::Point2f& v3) {
  float d1, d2, d3;
  bool has_neg, has_pos;

  d1 = sign(pt, v1, v2);
  d2 = sign(pt, v2, v3);
  d3 = sign(pt, v3, v1);

  has_neg = (d1 < 0) || (d2 < 0) || (d3 < 0);
  has_pos = (d1 > 0) || (d2 > 0) || (d3 > 0);

  return !(has_neg && has_pos);
}

void MeshOptimization::drawPointCloud(const std::string& id,
                                      const cv::Mat& pointcloud,
                                      const cv::Affine3d& pose) {
  cv::Mat viz_cloud(0, 1, CV_32FC3, cv::Scalar(0));
  cv::Mat colors_pcl = cv::Mat(0, 0, CV_8UC3, cv::viz::Color::red());
  CHECK_EQ(img_.type(), CV_8UC1);
  if (pointcloud.rows != 1u || pointcloud.cols != 1u) {
    LOG(ERROR) << "Reshaping pointcloud!";
    cv::Mat_<cv::Point3f> flat_pcl = cv::Mat(1, 0, CV_32FC3);
    for (int32_t v = 0u; v < pointcloud.rows; ++v) {
      for (int32_t u = 0u; u < pointcloud.cols; ++u) {
        const cv::Point3f& lmk = pointcloud.at<cv::Point3f>(v, u);
        if (isValidPoint(lmk)) {
          flat_pcl.push_back(lmk);
          colors_pcl.push_back(cv::Vec3b::all(img_.at<uint8_t>(v, u)));
        }
      }
    }
    viz_cloud = flat_pcl;
  }
  cv::viz::WCloud cloud(viz_cloud, colors_pcl);
  cloud.setRenderingProperty(cv::viz::POINT_SIZE, 6);
  window_.showWidget(id, cloud, pose);
}

void MeshOptimization::drawCylinder(const std::string& id,
                                    const cv::Point3d& axis_point1,
                                    const cv::Point3d& axis_point2,
                                    const double& radius,
                                    const int& numsides,
                                    const cv::viz::Color& color) {
  cv::viz::WCylinder cylinder(
      axis_point1, axis_point2, radius, numsides, color);
  window_.showWidget(id, cylinder);
}

void MeshOptimization::drawScene(const gtsam::Pose3& extrinsics,
                                 const gtsam::Cal3_S2& intrinsics) {
  cv::Mat cv_extrinsics;
  cv::eigen2cv(extrinsics.matrix(), cv_extrinsics);
  cv::Affine3f cam_pose_real;
  cam_pose_real.matrix = cv_extrinsics;

  cv::Matx33d K;
  cv::eigen2cv(intrinsics.K(), K);
  cv::viz::WCameraPosition cpw(0.2);  // Coordinate axes
  static constexpr float kScaleFrustum = 1.0;
  cv::viz::WCameraPosition cpw_frustum(
      K, img_, kScaleFrustum);  // Camera frustum
  window_.showWidget("World Coordinates", cv::viz::WCoordinateSystem(0.5));
  window_.showWidget("Cam Coordinates", cpw, cam_pose_real);
  window_.showWidget("Cam Frustum", cpw_frustum, cam_pose_real);
}

void MeshOptimization::drawArrow(const cv::Point3f& from,
                                 const cv::Point3f& to,
                                 const std::string& id,
                                 const bool& with_text,
                                 const double& arrow_thickness,
                                 const double& text_thickness,
                                 const cv::viz::Color& color) {
  // Display 3D rays from cam origin to lmks.
  if (with_text) {
    window_.showWidget("Arrow Label " + id,
                       cv::viz::WText3D(id, to, text_thickness, true, color));
  }
  window_.showWidget("Arrow " + id,
                     cv::viz::WArrow(from, to, arrow_thickness, color));
}

void MeshOptimization::drawPixelOnImg(const cv::Point2f& pixel,
                                      const cv::Mat& img,
                                      const cv::viz::Color& color,
                                      const size_t& pixel_size) {
  // Draw the pixel on the image
  cv::circle(img, pixel, pixel_size, color, -1);
}

void MeshOptimization::spinDisplay() {
  // Display 3D window
  window_.spin();
}

}  // namespace VIO

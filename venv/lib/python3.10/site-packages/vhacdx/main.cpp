#include <iostream>
#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#define ENABLE_VHACD_IMPLEMENTATION 1
#define VHACD_DISABLE_THREADING 0
#include "VHACD.h"

#define STRINGIFY(x) #x
#define MACRO_STRINGIFY(x) STRINGIFY(x)

namespace py = pybind11;

// Return a list of convex hulls
// Each convex hull is a tuple of (vertices, indices)
// vertices is a numpy array of shape (n, 3)
// indices is a numpy array of shape (m,)
std::vector<std::pair<py::array_t<double>, py::array_t<uint32_t>>>
compute_vhacd(py::array_t<double> points, py::array_t<uint32_t> faces,
              uint32_t maxConvexHulls, uint32_t resolution,
              double minimumVolumePercentErrorAllowed,
              uint32_t maxRecursionDepth, bool shrinkWrap, std::string fillMode,
              uint32_t maxNumVerticesPerCH, bool asyncACD,
              uint32_t minEdgeLength, bool findBestPlane) {

  /*  read input arrays buffer_info */
  auto buf_points = points.request();
  auto buf_faces = faces.request();

  /*  variables */
  double *ptr_points = (double *)buf_points.ptr;
  uint32_t *ptr_faces = (uint32_t *)buf_faces.ptr;
  size_t num_points = buf_points.shape[0];
  size_t num_faces = buf_faces.shape[0] / 4;
  size_t num_triangle_indices = num_faces * 3;

  // Prepare our input arrays for VHACD
  uint32_t *triangles = new uint32_t[num_triangle_indices];
  for (uint32_t i = 0; i < num_faces; i++) {
    // We skip the first index of the face array, which is the number of
    // vertices in the face
    triangles[3 * i] = ptr_faces[4 * i + 1];
    triangles[3 * i + 1] = ptr_faces[4 * i + 2];
    triangles[3 * i + 2] = ptr_faces[4 * i + 3];
  }

  // Create the VHACD parameters
  VHACD::IVHACD::Parameters p;
  p.m_maxConvexHulls = maxConvexHulls;
  p.m_resolution = resolution;
  p.m_minimumVolumePercentErrorAllowed = minimumVolumePercentErrorAllowed;
  p.m_maxRecursionDepth = maxRecursionDepth;
  p.m_shrinkWrap = shrinkWrap;
  p.m_maxNumVerticesPerCH = maxNumVerticesPerCH;
  p.m_asyncACD = asyncACD;
  p.m_minEdgeLength = minEdgeLength;
  p.m_findBestPlane = findBestPlane;
  if (fillMode == "flood") {
    p.m_fillMode = VHACD::FillMode::FLOOD_FILL;
  } else if (fillMode == "raycast") {
    p.m_fillMode = VHACD::FillMode::RAYCAST_FILL;
  } else if (fillMode == "surface") {
    p.m_fillMode = VHACD::FillMode::SURFACE_ONLY;
  } else {
    printf("Invalid fill mode, only valid options are 'flood', 'raycast', and "
           "'surface'\n");
  }

#if VHACD_DISABLE_THREADING
  VHACD::IVHACD *iface = VHACD::CreateVHACD();
#else
  VHACD::IVHACD *iface =
      p.m_asyncACD ? VHACD::CreateVHACD_ASYNC() : VHACD::CreateVHACD();
#endif

  /// The main computation ///////////////////////////////////////////////
  iface->Compute(ptr_points, num_points, triangles, num_faces, p);

  while (!iface->IsReady()) {
    std::this_thread::sleep_for(std::chrono::nanoseconds(10000)); // s
  }

  // Build our output arrays from VHACD outputs
  std::vector<std::pair<py::array_t<double>, py::array_t<uint32_t>>> res;
  const int nConvexHulls = iface->GetNConvexHulls();
  res.reserve(nConvexHulls);

  if (nConvexHulls) {
    // Exporting Convex Decomposition results of convex hulls

    for (uint32_t i = 0; i < iface->GetNConvexHulls(); i++) {
      VHACD::IVHACD::ConvexHull ch;
      iface->GetConvexHull(i, ch);

      /*  allocate the output buffers */
      py::array_t<double> res_vertices =
          py::array_t<double>(ch.m_points.size() * 3);
      py::array_t<uint32_t> res_faces =
          py::array_t<uint32_t>(ch.m_triangles.size() * 3);

      py::buffer_info buf_res_vertices = res_vertices.request();
      py::buffer_info buf_res_faces = res_faces.request();

      double *ptr_res_vertices = (double *)buf_res_vertices.ptr;
      uint32_t *ptr_res_faces = (uint32_t *)buf_res_faces.ptr;

      for (uint32_t j = 0; j < ch.m_points.size(); j++) {
        const VHACD::Vertex &pos = ch.m_points[j];
        ptr_res_vertices[3 * j] = pos.mX;
        ptr_res_vertices[3 * j + 1] = pos.mY;
        ptr_res_vertices[3 * j + 2] = pos.mZ;
      }

      const uint32_t num_v = 3;
      for (uint32_t j = 0; j < ch.m_triangles.size(); j++) {
        uint32_t i1 = ch.m_triangles[j].mI0;
        uint32_t i2 = ch.m_triangles[j].mI1;
        uint32_t i3 = ch.m_triangles[j].mI2;
        ptr_res_faces[3 * j] = i1;
        ptr_res_faces[3 * j + 1] = i2;
        ptr_res_faces[3 * j + 2] = i3;
      }

      // Reshape the vertices array to be (n, 3)
      const long width = 3;
      res_vertices.resize({(long)ch.m_points.size(), width});
      // Resize faces to be (m, 3)
      res_faces.resize({(long)ch.m_triangles.size(), width});

      // Push on our list
      res.emplace_back(std::move(res_vertices), std::move(res_faces));
    }
  }
  return res;
}

/* Wrapping routines with PyBind */
PYBIND11_MODULE(vhacdx, m) {
  m.doc() =
      "Python bindings for the V-HACD algorithm"; // optional module docstring
  m.def("compute_vhacd", &compute_vhacd, "Compute convex hulls",
        py::arg("points"), py::arg("faces"), py::arg("maxConvexHulls") = 64,
        py::arg("resolution") = 400000,
        py::arg("minimumVolumePercentErrorAllowed") = 1.0,
        py::arg("maxRecursionDepth") = 10, py::arg("shrinkWrap") = true,
        py::arg("fillMode") = "flood", py::arg("maxNumVerticesPerCH") = 64,
        py::arg("asyncACD") = true, py::arg("minEdgeLength") = 2,
        py::arg("findBestPlane") = false);

#ifdef VERSION_INFO
  m.attr("__version__") = MACRO_STRINGIFY(VERSION_INFO);
#else
  m.attr("__version__") = "dev";
#endif
}

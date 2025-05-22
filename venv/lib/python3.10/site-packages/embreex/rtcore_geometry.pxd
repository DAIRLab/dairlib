# rtcore_geometry wrapper

from .rtcore_ray cimport RTCRay, RTCRay4, RTCRay8, RTCRay16
from .rtcore_scene cimport RTCScene
cimport cython
cimport numpy as np

cdef extern from "embree2/rtcore_geometry.h":
    cdef unsigned int RTC_INVALID_GEOMETRY_ID

    cdef enum RTCBufferType:
        RTC_INDEX_BUFFER
        RTC_VERTEX_BUFFER
        RTC_VERTEX_BUFFER0
        RTC_VERTEX_BUFFER1

        RTC_FACE_BUFFER
        RTC_LEVEL_BUFFER

        RTC_EDGE_CREASE_INDEX_BUFFER 
        RTC_EDGE_CREASE_WEIGHT_BUFFER 

        RTC_VERTEX_CREASE_INDEX_BUFFER 
        RTC_VERTEX_CREASE_WEIGHT_BUFFER 

        RTC_HOLE_BUFFER          

    cdef enum RTCMatrixType:
        RTC_MATRIX_ROW_MAJOR
        RTC_MATRIX_COLUMN_MAJOR
        RTC_MATRIX_COLUMN_MAJOR_ALIGNED16

    cdef enum RTCGeometryFlags:
        RTC_GEOMETRY_STATIC
        RTC_GEOMETRY_DEFORMABLE
        RTC_GEOMETRY_DYNAMIC

    cdef struct RTCBounds:
        float lower_x, lower_y, lower_z, align0
        float upper_x, upper_y, upper_z, align1

    ctypedef void (*RTCFilterFunc)(void* ptr, RTCRay& ray)
    ctypedef void (*RTCFilterFunc4)(void* ptr, RTCRay4& ray)
    ctypedef void (*RTCFilterFunc8)(void* ptr, RTCRay8& ray)
    ctypedef void (*RTCFilterFunc16)(void* ptr, RTCRay16& ray)

    ctypedef void (*RTCDisplacementFunc)(void* ptr, unsigned geomID, unsigned primID,
                                         const float* u, const float* v,
                                         const float* nx, const float* ny, const float* nz,
                                         float* px, float* py, float* pz, size_t N)

    unsigned rtcNewInstance(RTCScene target, RTCScene source)
    void rtcSetTransform(RTCScene scene, unsigned geomID,
                         RTCMatrixType layout, const float *xfm)
    unsigned rtcNewTriangleMesh(RTCScene scene, RTCGeometryFlags flags, 
                                size_t numTriangles, size_t numVertices,
                                size_t numTimeSteps)

    unsigned rtcNewSubdivisionMesh (RTCScene scene, RTCGeometryFlags flags,
                                    size_t numFaces, size_t numEdges,
                                    size_t numVertices, size_t numEdgeCreases,
                                    size_t numVertexCreases, size_t numHoles,
                                    size_t numTimeSteps)
    unsigned rtcNewHairGeometry (RTCScene scene, RTCGeometryFlags flags,
                                 size_t numCurves, size_t numVertices,
                                 size_t numTimeSteps)
    void rtcSetMask(RTCScene scene, unsigned geomID, int mask)
    void *rtcMapBuffer(RTCScene scene, unsigned geomID, RTCBufferType type)
    void rtcUnmapBuffer(RTCScene scene, unsigned geomID, RTCBufferType type)
    void rtcSetBuffer(RTCScene scene, unsigned geomID, RTCBufferType type,
                      void *ptr, size_t offset, size_t stride)
    void rtcEnable(RTCScene scene, unsigned geomID)
    void rtcUpdate(RTCScene scene, unsigned geomID)
    void rtcUpdateBuffer(RTCScene scene, unsigned geomID, RTCBufferType type)
    void rtcDisable(RTCScene scene, unsigned geomID)
    void rtcSetDisplacementFunction (RTCScene scene, unsigned geomID, RTCDisplacementFunc func, RTCBounds* bounds)
    void rtcSetIntersectionFilterFunction (RTCScene scene, unsigned geomID, RTCFilterFunc func)
    void rtcSetIntersectionFilterFunction4 (RTCScene scene, unsigned geomID, RTCFilterFunc4 func)
    void rtcSetIntersectionFilterFunction8 (RTCScene scene, unsigned geomID, RTCFilterFunc8 func)
    void rtcSetIntersectionFilterFunction16 (RTCScene scene, unsigned geomID, RTCFilterFunc16 func)
    void rtcSetOcclusionFilterFunction (RTCScene scene, unsigned geomID, RTCFilterFunc func)
    void rtcSetOcclusionFilterFunction4 (RTCScene scene, unsigned geomID, RTCFilterFunc4 func)
    void rtcSetOcclusionFilterFunction8 (RTCScene scene, unsigned geomID, RTCFilterFunc8 func)
    void rtcSetOcclusionFilterFunction16 (RTCScene scene, unsigned geomID, RTCFilterFunc16 func)
    void rtcSetUserData (RTCScene scene, unsigned geomID, void* ptr)
    void* rtcGetUserData (RTCScene scene, unsigned geomID)
    void rtcDeleteGeometry (RTCScene scene, unsigned geomID)


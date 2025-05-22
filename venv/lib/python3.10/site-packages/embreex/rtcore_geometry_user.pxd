# rtcore_geometry_user wrapper

#from libc.stdint cimport ssize_t, size_t
from .rtcore_ray cimport RTCRay, RTCRay4, RTCRay8, RTCRay16
from .rtcore_geometry cimport RTCBounds
from .rtcore_scene cimport RTCScene
cimport cython
cimport numpy as np

cdef extern from "embree2/rtcore_geometry_user.h":
    ctypedef void (*RTCBoundsFunc)(void* ptr, size_t item, RTCBounds& bounds_o)
    ctypedef void (*RTCIntersectFunc)(void* ptr, RTCRay& ray, size_t item)
    ctypedef void (*RTCIntersectFunc4)(const void* valid, void* ptr,
                                       RTCRay4& ray, size_t item)
    ctypedef void (*RTCIntersectFunc8)(const void* valid, void* ptr,
                                       RTCRay8& ray, size_t item)
    ctypedef void (*RTCIntersectFunc16)(const void* valid, void* ptr,
                                        RTCRay16& ray, size_t item)
    ctypedef void (*RTCOccludedFunc)(void* ptr, RTCRay& ray, size_t item)
    ctypedef void (*RTCOccludedFunc4)(const void* valid, void* ptr,
                                      RTCRay4& ray, size_t item)
    ctypedef void (*RTCOccludedFunc8)(const void* valid, void* ptr,
                                      RTCRay8& ray, size_t item)
    ctypedef void (*RTCOccludedFunc16)(const void* valid, void* ptr,
                                       RTCRay16& ray, size_t item)
    unsigned rtcNewUserGeometry(RTCScene scene, size_t numGeometries)
    void rtcSetBoundsFunction(RTCScene scene, unsigned geomID, RTCBoundsFunc bounds)
    void rtcSetIntersectFunction(RTCScene scene, unsigned geomID, RTCIntersectFunc intersect)
    void rtcSetIntersectFunction4(RTCScene scene, unsigned geomID, RTCIntersectFunc4 intersect4)
    void rtcSetIntersectFunction8(RTCScene scene, unsigned geomID, RTCIntersectFunc8 intersect8)
    void rtcSetIntersectFunction16(RTCScene scene, unsigned geomID, RTCIntersectFunc16 intersect16)
    void rtcSetOccludedFunction(RTCScene scene, unsigned geomID, RTCOccludedFunc occluded)
    void rtcSetOccludedFunction4(RTCScene scene, unsigned geomID, RTCOccludedFunc4 occluded4)
    void rtcSetOccludedFunction8(RTCScene scene, unsigned geomID, RTCOccludedFunc8 occluded8)
    void rtcSetOccludedFunction16(RTCScene scene, unsigned geomID, RTCOccludedFunc16 occluded16)

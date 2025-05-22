# rtcore_scene.pxd wrapper

cimport cython
cimport numpy as np
cimport rtcore as rtc
cimport rtcore_ray as rtcr

cdef extern from "embree2/rtcore_scene.h":

    ctypedef struct RTCRay
    ctypedef struct RTCRay4
    ctypedef struct RTCRay8
    ctypedef struct RTCRay16

    cdef enum RTCSceneFlags:
        RTC_SCENE_STATIC
        RTC_SCENE_DYNAMIC
        RTC_SCENE_COMPACT
        RTC_SCENE_COHERENT
        RTC_SCENE_INCOHERENT
        RTC_SCENE_HIGH_QUALITY
        RTC_SCENE_ROBUST

    cdef enum RTCAlgorithmFlags:
        RTC_INTERSECT1
        RTC_INTERSECT4
        RTC_INTERSECT8
        RTC_INTERSECT16

    # ctypedef void* RTCDevice
    ctypedef void* RTCScene

    RTCScene rtcNewScene(RTCSceneFlags flags, RTCAlgorithmFlags aflags)

    RTCScene rtcDeviceNewScene(rtc.RTCDevice device, RTCSceneFlags flags, RTCAlgorithmFlags aflags)

    ctypedef bint (*RTCProgressMonitorFunc)(void* ptr, const double n)

    void rtcSetProgressMonitorFunction(RTCScene scene, RTCProgressMonitorFunc func, void* ptr)

    void rtcCommit(RTCScene scene)

    void rtcCommitThread(RTCScene scene, unsigned int threadID, unsigned int numThreads)

    void rtcIntersect(RTCScene scene, RTCRay& ray)

    void rtcIntersect4(const void* valid, RTCScene scene, RTCRay4& ray)

    void rtcIntersect8(const void* valid, RTCScene scene, RTCRay8& ray)

    void rtcIntersect16(const void* valid, RTCScene scene, RTCRay16& ray)

    void rtcOccluded(RTCScene scene, RTCRay& ray)

    void rtcOccluded4(const void* valid, RTCScene scene, RTCRay4& ray)

    void rtcOccluded8(const void* valid, RTCScene scene, RTCRay8& ray)

    void rtcOccluded16(const void* valid, RTCScene scene, RTCRay16& ray)

    void rtcDeleteScene(RTCScene scene)

cdef class EmbreeScene:
    cdef RTCScene scene_i
    # Optional device used if not given, it should be as input of EmbreeScene
    cdef public int is_committed
    cdef rtc.EmbreeDevice device

cdef enum rayQueryType:
    intersect,
    occluded,
    distance

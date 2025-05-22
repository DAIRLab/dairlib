# rtcore.pxd wrapper

cimport cython
cimport numpy as np


cdef extern from "embree2/rtcore.h":
    cdef int RTCORE_VERSION_MAJOR
    cdef int RTCORE_VERSION_MINOR
    cdef int RTCORE_VERSION_PATCH

    void rtcInit(const char* cfg)
    void rtcExit()

    cdef enum RTCError:
        RTC_NO_ERROR
        RTC_UNKNOWN_ERROR
        RTC_INVALID_ARGUMENT
        RTC_INVALID_OPERATION
        RTC_OUT_OF_MEMORY
        RTC_UNSUPPORTED_CPU
        RTC_CANCELLED

    # typedef struct __RTCDevice {}* RTCDevice;
    ctypedef void* RTCDevice

    RTCDevice rtcNewDevice(const char* cfg)
    void rtcDeleteDevice(RTCDevice device)

    RTCError rtcGetError()
    ctypedef void (*RTCErrorFunc)(const RTCError code, const char* _str)
    void rtcSetErrorFunction(RTCErrorFunc func)

    # Embree 2.14.0-0
    void rtcDeviceSetErrorFunction(RTCDevice device, RTCErrorFunc func)

    # Embree 2.15.1
    ctypedef void (*RTCErrorFunc2)(void* userPtr, const RTCError code, const char* str)
    void rtcDeviceSetErrorFunction2(RTCDevice device, RTCErrorFunc2 func, void* userPtr)

    ctypedef bint RTCMemoryMonitorFunc(const ssize_t _bytes, const bint post)
    void rtcSetMemoryMonitorFunction(RTCMemoryMonitorFunc func)

cdef extern from "embree2/rtcore_ray.h":
    pass

cdef struct Vertex:
    float x, y, z, r

cdef struct Triangle:
    int v0, v1, v2

cdef struct Vec3f:
    float x, y, z

cdef void print_error(RTCError code)

cdef class EmbreeDevice:
    cdef RTCDevice device

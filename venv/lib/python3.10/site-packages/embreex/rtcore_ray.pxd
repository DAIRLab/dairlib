# rtcore_ray.pxd wrapper

cimport cython
cimport numpy as np

cdef extern from "embree2/rtcore_ray.h":
    # RTCORE_ALIGN(16)
    # This is for a *single* ray
    cdef struct RTCRay:
        # Ray data
        float org[3]
        float align0

        float dir[3]
        float align1

        float tnear
        float tfar

        float time
        int mask

        # Hit data
        float Ng[3]
        float align2

        float u
        float v

        int geomID
        int primID
        int instID

    # This is for a packet of 4 rays
    cdef struct RTCRay4:
        # Ray data
        float orgx[4]
        float orgy[4]
        float orgz[4]
        float align0

        float dirx[4]
        float diry[4]
        float dirz[4]

        float tnear[4]
        float tfar[4]

        float time[4]
        int mask[4]

        # Hit data
        float Ngx[4]
        float Ngy[4]
        float Ngz[4]

        float u[4]
        float v[4]

        int geomID[4]
        int primID[4]
        int instID[4]

    # This is for a packet of 8 rays
    cdef struct RTCRay8:
        # Ray data
        float orgx[8]
        float orgy[8]
        float orgz[8]
        float align0

        float dirx[8]
        float diry[8]
        float dirz[8]

        float tnear[8]
        float tfar[8]

        float time[8]
        int mask[8]

        # Hit data
        float Ngx[8]
        float Ngy[8]
        float Ngz[8]

        float u[8]
        float v[8]

        int geomID[8]
        int primID[8]
        int instID[8]

    # This is for a packet of 16 rays
    cdef struct RTCRay16:
        # Ray data
        float orgx[16]
        float orgy[16]
        float orgz[16]
        float align0

        float dirx[16]
        float diry[16]
        float dirz[16]

        float tnear[16]
        float tfar[16]

        float time[16]
        int mask[16]

        # Hit data
        float Ngx[16]
        float Ngy[16]
        float Ngz[16]

        float u[16]
        float v[16]

        int geomID[16]
        int primID[16]
        int instID[16]

#!/usr/bin/env python3
import sys
import os
import glob

def get_min_z_from_obj(obj_path):
    min_z = None
    with open(obj_path, 'r') as f:
        for line in f:
            if line.startswith('v '):
                parts = line.strip().split()
                if len(parts) == 4:
                    z = float(parts[3])
                    if min_z is None or z < min_z:
                        min_z = z
    return min_z

def get_max_z_from_obj(obj_path):
    max_z = None
    with open(obj_path, 'r') as f:
        for line in f:
            if line.startswith('v '):
                parts = line.strip().split()
                if len(parts) == 4:
                    z = float(parts[3])
                    if max_z is None or z > max_z:
                        max_z = z
    return max_z
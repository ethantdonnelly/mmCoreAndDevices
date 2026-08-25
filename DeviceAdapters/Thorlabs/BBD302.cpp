///////////////////////////////////////////////////////////////////////////////
// FILE:          BBD302.cpp
///////////////////////////////////////////////////////////////////////////////
// FILE:          BBD302.cpp
// PROJECT:       Micro-Manager
// SUBSYSTEM:     DeviceAdapters
//-----------------------------------------------------------------------------
// DESCRIPTION:   Thorlabs BBD302 XY-stage adapter
//
// LICENSE:       This file is distributed under the BSD license.
//
// AUTHORS:       Ethan Donnelly, 2026
//
// This adapter uses the Thorlabs Kinesis Benchtop Brushless Motor C API.
///////////////////////////////////////////////////////////////////////////////

#ifdef WIN32
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#endif

#include "BBD302.h"
#include "Thorlabs.h"
#include "Thorlabs.MotionControl.Benchtop.BrushlessMotor.h"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstring>
#include <limits>
#include <sstream>
#include <vector>
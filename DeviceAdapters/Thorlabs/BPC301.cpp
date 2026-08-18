///////////////////////////////////////////////////////////////////////////////
// FILE:          BPC301.cpp
///////////////////////////////////////////////////////////////////////////////
// FILE:          BPC301.cpp
// PROJECT:       Micro-Manager
// SUBSYSTEM:     DeviceAdapters
//-----------------------------------------------------------------------------
// DESCRIPTION:   Thorlabs BPC301 high-precision single-axis Z-stage adapter
//
// LICENSE:       This file is distributed under the BSD license.
//
// AUTHORS:       Ethan Donnelly, 2026
//
// This adapter uses the Thorlabs BPC301 Command Library.
///////////////////////////////////////////////////////////////////////////////

#ifdef WIN32
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#endif

#include "BPC301.h"
#include "Thorlabs.h"
#include "Thorlabs.MotionControl.Benchtop.Piezo.h"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstring>
#include <limits>
#include <sstream>
#include <vector>

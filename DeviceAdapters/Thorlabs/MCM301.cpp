///////////////////////////////////////////////////////////////////////////////
// FILE:          MCM301.cpp
// PROJECT:       Micro-Manager
// SUBSYSTEM:     DeviceAdapters
//-----------------------------------------------------------------------------
// DESCRIPTION:   Thorlabs device adapters: MCM301 Controller
//
// COPYRIGHT:     Thorlabs, 2011
//
// LICENSE:       This file is distributed under the BSD license.
//
//                This file is distributed in the hope that it will be useful,
//                but WITHOUT ANY WARRANTY; without even the implied warranty
//                of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
//
//                IN NO EVENT SHALL THE COPYRIGHT OWNER OR
//                CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
//                INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES.
//
// AUTHOR:        Nenad Amodaj, nenad@amodaj.com, 2011
//

#ifdef WIN32
	#include <windows.h>
#endif

#include "MCM301.h"
#include "Thorlabs.h"
#include <cstdio>
#include <string>
#include <math.h>
#include <sstream>






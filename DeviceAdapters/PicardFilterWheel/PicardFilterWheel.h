///////////////////////////////////////////////////////////////////////////////
// FILE:          PicardFilterWheel.h
// PROJECT:       Micro-Manager
// SUBSYSTEM:     DeviceAdapters
//-----------------------------------------------------------------------------
// DESCRIPTION:   The drivers for the Picard Industries USB filter wheel
//
// AUTHORS:       Ethan Donnelly, 2026
//
// LICENSE:       This file is distributed under the BSD license.
//                License text is included with the source distribution.
//
//                This file is distributed in the hope that it will be useful,
//                but WITHOUT ANY WARRANTY; without even the implied warranty
//                of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
//
//                IN NO EVENT SHALL THE COPYRIGHT OWNER OR
//                CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
//                INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES.
///////////////////////////////////////////////////////////////////////////////
// DEPENDENCIES
//
// Download Picard Filter Wheel software from Picard Industries:
// https://picardindustries.com/other/software-downloads/
//   - Unzip the file into the 3rdpartypublic directory of your Micro-Manager source tree.
//
// Expected file locations:
//   Include:  3rdpartypublic\Picard\USB Filter V1.4\PiUsbSDK\include
//   Lib:      3rdpartypublic\Picard\USB Filter V1.4\PiUsbSDK\lib\x64
//   Bin:      3rdpartypublic\Picard\USB Filter V1.4\PiUsbSDK\bin\x64
//
// Required Picard files:
//   - PiUsb.h
//   - PiUsb.lib
//
// Runtime DLLs:
//   - PiUsb.dll
///////////////////////////////////////////////////////////////////////////////

#ifndef _PICARDFILTERWHEEL_H_
#define _PICARDFILTERWHEEL_H_

#include "DeviceBase.h"

class CPicardFilterWheel :
	public CStateDeviceBase<CPicardFilterWheel>
{
public: 
	CPicardFilterWheel();
	~CPicardFilterWheel();

	// MMDevice API
    // ------------
	int Initialize();
	int Shutdown();

	void GetName(char* name) const;
	bool Busy();
	unsigned long GetNumberOfPositions() const 
	{
		return static_cast<unsigned long>(numPositions_); 
	}

	// Property action handlers 
	int OnState(
		MM::PropertyBase* property,
		MM::ActionType action);

	int OnSerialNumber(
		MM::PropertyBase* property,
		MM::ActionType action);

	int OnNumberOfPositions(
		MM::PropertyBase* property,
		MM::ActionType action);

private:
	int DiscoverSerialNumber();
	int serialNumber_;

	long numPositions_;

	bool initialized_;
	bool movePending_;

	int targetPicardPosition_;

	long position_;

	void* handle_;

	MM::MMTime moveStartTime_;
};

#endif //_PICARDFILTERWHEEL_H_

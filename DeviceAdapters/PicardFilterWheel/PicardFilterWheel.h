///////////////////////////////////////////////////////////////////////////////
// FILE:          PicardFilterWheel.h
// PROJECT:       Micro-Manager
// SUBSYSTEM:     DeviceAdapters
//-----------------------------------------------------------------------------
// DESCRIPTION:   The drivers for the Picard Industries USB filter wheel
//                Based on the CDemoStage and CDemoXYStage classes
//
// AUTHORS:       Johannes Schindelin, Luke Stuyvenberg, 2011 - 2014
//
// COPYRIGHT:     Board of Regents of the University of Wisconsin -- Madison,
//					Copyright (C) 2011 - 2014
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

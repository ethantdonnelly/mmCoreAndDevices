///////////////////////////////////////////////////////////////////////////////
// FILE:          PicardFilterWheel.cpp
// PROJECT:       Micro-Manager
// SUBSYSTEM:     DeviceAdapters
//-----------------------------------------------------------------------------
// DESCRIPTION:   The drivers required for the Picard Industries USB filter wheel
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

#include <cstdio>
#include <cstring>

#include "ModuleInterface.h"
#include "PiUsb.h"
#include "PicardFilterWheel.h"


///////////////////////////////////////////////////////////////////////////////
// Constants and helper functions
///////////////////////////////////////////////////////////////////////////////

namespace
{
    const char* g_DeviceName =
        "Picard Filter Wheel";

    const char* g_Keyword_SerialNumber =
        "Serial Number";

    const char* g_Keyword_NumberOfPositions =
        "Number of Positions";

    const int DEFAULT_SERIAL_NUMBER = -1;
    const long DEFAULT_NUMBER_OF_POSITIONS = 6;
    const long MAX_NUMBER_OF_POSITIONS = 16;

    // The Python implementation waits at least 10 ms after sending a move.
    const double BUSY_START_DELAY_MS = 10.0;

    const int ERR_NO_FILTER_WHEEL = 1102;


    int InterpretPiUsbError(int piError)
    {
        switch (piError)
        {
        case PI_NO_ERROR:
            return DEVICE_OK;

        case PI_DEVICE_NOT_FOUND:
        case PI_OBJECT_NOT_FOUND:
        case PI_INVALID_DEVICE_HANDLE:
            return DEVICE_NOT_CONNECTED;

        case PI_INVALID_PARAMETER:
            return DEVICE_INVALID_PROPERTY_VALUE;

        case PI_CANNOT_CREATE_OBJECT:
        case PI_READ_TIMEOUT:
        case PI_READ_THREAD_ABANDONED:
        case PI_READ_FAILED:
        case PI_WRITE_FAILED:
        default:
            return DEVICE_ERR;
        }
    }
}


///////////////////////////////////////////////////////////////////////////////
// Exported Micro-Manager module API
///////////////////////////////////////////////////////////////////////////////

MODULE_API void InitializeModuleData()
{
    RegisterDevice(
        g_DeviceName,
        MM::StateDevice,
        "Picard Industries USB Filter Wheel");
}


MODULE_API MM::Device* CreateDevice(const char* deviceName)
{
    if (deviceName == 0)
        return 0;

    if (strcmp(deviceName, g_DeviceName) == 0)
        return new CPicardFilterWheel();

    return 0;
}


MODULE_API void DeleteDevice(MM::Device* device)
{
    delete device;
}


///////////////////////////////////////////////////////////////////////////////
// CPicardFilterWheel implementation
///////////////////////////////////////////////////////////////////////////////

CPicardFilterWheel::CPicardFilterWheel() :
    numPositions_(DEFAULT_NUMBER_OF_POSITIONS),
    initialized_(false),
    movePending_(false),
    serialNumber_(DEFAULT_SERIAL_NUMBER),
    targetPicardPosition_(0),
    position_(0),
    handle_(0),
    moveStartTime_(0.0)
{
    InitializeDefaultErrorMessages();

    SetErrorText(
        ERR_NO_FILTER_WHEEL,
        "No compatible Picard filter wheel was found");

    // Serial number is selected before initialization.
    // A value of -1 means automatic discovery.
    CPropertyAction* action =
        new CPropertyAction(
            this,
            &CPicardFilterWheel::OnSerialNumber);

    CreateProperty(
        g_Keyword_SerialNumber,
        "-1",
        MM::Integer,
        false,
        action,
        true);

    // Standard wheels have six positions.
    // Picard supports custom wheels with up to 16.
    action =
        new CPropertyAction(
            this,
            &CPicardFilterWheel::OnNumberOfPositions);

    CreateProperty(
        g_Keyword_NumberOfPositions,
        "6",
        MM::Integer,
        false,
        action,
        true);

    SetPropertyLimits(
        g_Keyword_NumberOfPositions,
        1,
        MAX_NUMBER_OF_POSITIONS);
}


CPicardFilterWheel::~CPicardFilterWheel()
{
    Shutdown();
}


void CPicardFilterWheel::GetName(char* name) const
{
    CDeviceUtils::CopyLimitedString(
        name,
        g_DeviceName);
}


///////////////////////////////////////////////////////////////////////////////
// Pre-initialization property handlers
///////////////////////////////////////////////////////////////////////////////

int CPicardFilterWheel::OnSerialNumber(
    MM::PropertyBase* property,
    MM::ActionType action)
{
    if (action == MM::BeforeGet)
    {
        property->Set(
            static_cast<long>(serialNumber_));
    }
    else if (action == MM::AfterSet)
    {
        long value = serialNumber_;
        property->Get(value);

        if (value < -1)
            return DEVICE_INVALID_PROPERTY_VALUE;

        serialNumber_ = static_cast<int>(value);
    }

    return DEVICE_OK;
}


int CPicardFilterWheel::OnNumberOfPositions(
    MM::PropertyBase* property,
    MM::ActionType action)
{
    if (action == MM::BeforeGet)
    {
        property->Set(numPositions_);
    }
    else if (action == MM::AfterSet)
    {
        long value = numPositions_;
        property->Get(value);

        if (value < 1 ||
            value > MAX_NUMBER_OF_POSITIONS)
        {
            property->Set(numPositions_);
            return DEVICE_INVALID_PROPERTY_VALUE;
        }

        numPositions_ = value;
    }

    return DEVICE_OK;
}


///////////////////////////////////////////////////////////////////////////////
// Device discovery
///////////////////////////////////////////////////////////////////////////////

int CPicardFilterWheel::DiscoverSerialNumber()
{
    const int maxDevices = 16;

    int deviceCount = 0;
    int serialNumbers[maxDevices] = { 0 };

    int piError = piFindFilters(
        &deviceCount,
        serialNumbers,
        maxDevices);

    if (piError != PI_NO_ERROR)
        return InterpretPiUsbError(piError);

    if (deviceCount < 1)
        return ERR_NO_FILTER_WHEEL;

    // piFindFilters can also find compatible Picard gradient-wheel
    // and rotator devices because they share the same product ID.
    serialNumber_ = serialNumbers[0];

    char serialText[32];

    snprintf(
        serialText,
        sizeof(serialText),
        "%d",
        serialNumber_);

    SetProperty(
        g_Keyword_SerialNumber,
        serialText);

    if (deviceCount > 1)
    {
        LogMessage(
            "Multiple compatible Picard devices were found. "
            "Using the first device. Set Serial Number manually "
            "to select a different device.",
            false);
    }

    return DEVICE_OK;
}


///////////////////////////////////////////////////////////////////////////////
// Initialization and shutdown
///////////////////////////////////////////////////////////////////////////////

int CPicardFilterWheel::Initialize()
{
    if (initialized_)
        return DEVICE_OK;

    if (handle_ != 0)
        Shutdown();

    int result = DEVICE_OK;

    if (serialNumber_ == DEFAULT_SERIAL_NUMBER)
    {
        result = DiscoverSerialNumber();

        if (result != DEVICE_OK)
            return result;
    }

    int piError = PI_NO_ERROR;

    handle_ = piConnectFilter(
        &piError,
        serialNumber_);

    if (handle_ == 0 ||
        piError != PI_NO_ERROR)
    {
        char message[128];

        snprintf(
            message,
            sizeof(message),
            "Could not connect to Picard filter wheel %d "
            "(PiUsb error %d)",
            serialNumber_,
            piError);

        LogMessage(message, false);

        handle_ = 0;

        if (piError == PI_NO_ERROR)
            return ERR_NO_FILTER_WHEEL;

        return InterpretPiUsbError(piError);
    }

    // Obtain the current hardware position.
    int hardwarePosition = 0;

    piError = piGetFilterPosition(
        &hardwarePosition,
        handle_);

    if (piError != PI_NO_ERROR)
    {
        Shutdown();
        return InterpretPiUsbError(piError);
    }

    if (hardwarePosition >= 1 &&
        hardwarePosition <= numPositions_)
    {
        // Picard positions are one-based.
        // Micro-Manager states are zero-based.
        position_ = hardwarePosition - 1;
    }

    // Standard device properties.
    result = CreateProperty(
        MM::g_Keyword_Name,
        g_DeviceName,
        MM::String,
        true);

    if (result != DEVICE_OK)
    {
        Shutdown();
        return result;
    }

    result = CreateProperty(
        MM::g_Keyword_Description,
        "Picard Industries USB Filter Wheel",
        MM::String,
        true);

    if (result != DEVICE_OK)
    {
        Shutdown();
        return result;
    }

    // Create position names such as Filter-1 through Filter-6.
    char label[32];

    for (long index = 0;
        index < numPositions_;
        ++index)
    {
        snprintf(
            label,
            sizeof(label),
            "Filter-%ld",
            index + 1);

        SetPositionLabel(index, label);
    }

    // Numeric State property.
    CPropertyAction* action =
        new CPropertyAction(
            this,
            &CPicardFilterWheel::OnState);

    char initialState[32];

    snprintf(
        initialState,
        sizeof(initialState),
        "%ld",
        position_);

    result = CreateProperty(
        MM::g_Keyword_State,
        initialState,
        MM::Integer,
        false,
        action);

    if (result != DEVICE_OK)
    {
        Shutdown();
        return result;
    }

    result = SetPropertyLimits(
        MM::g_Keyword_State,
        0,
        numPositions_ - 1);

    if (result != DEVICE_OK)
    {
        Shutdown();
        return result;
    }

    // Human-readable Label property.
    action =
        new CPropertyAction(
            this,
            &CStateBase::OnLabel);

    result = CreateProperty(
        MM::g_Keyword_Label,
        "",
        MM::String,
        false,
        action);

    if (result != DEVICE_OK)
    {
        Shutdown();
        return result;
    }

    initialized_ = true;

    result = UpdateStatus();

    if (result != DEVICE_OK)
    {
        Shutdown();
        return result;
    }

    return DEVICE_OK;
}


int CPicardFilterWheel::Shutdown()
{
    if (handle_ != 0)
    {
        piDisconnectFilter(handle_);
        handle_ = 0;
    }

    initialized_ = false;
    movePending_ = false;
    targetPicardPosition_ = 0;

    return DEVICE_OK;
}


///////////////////////////////////////////////////////////////////////////////
// State property
///////////////////////////////////////////////////////////////////////////////

int CPicardFilterWheel::OnState(
    MM::PropertyBase* property,
    MM::ActionType action)
{
    if (handle_ == 0)
        return DEVICE_NOT_CONNECTED;

    if (action == MM::BeforeGet)
    {
        int hardwarePosition = 0;

        int piError = piGetFilterPosition(
            &hardwarePosition,
            handle_);

        if (piError != PI_NO_ERROR)
            return InterpretPiUsbError(piError);

        if (hardwarePosition >= 1 &&
            hardwarePosition <= numPositions_)
        {
            // While moving, the Picard wheel may briefly report
            // intermediate positive positions. Do not replace the
            // requested state unless it reached the destination.
            if (!movePending_ ||
                hardwarePosition == targetPicardPosition_)
            {
                position_ = hardwarePosition - 1;
            }

            if (movePending_ &&
                hardwarePosition == targetPicardPosition_)
            {
                movePending_ = false;
                targetPicardPosition_ = 0;
            }
        }

        // Position zero means the wheel is between slots.
        // Keep reporting the cached valid Micro-Manager state.
        property->Set(position_);
    }
    else if (action == MM::AfterSet)
    {
        long requestedPosition = position_;
        property->Get(requestedPosition);

        if (requestedPosition < 0 ||
            requestedPosition >= numPositions_)
        {
            property->Set(position_);
            return DEVICE_UNKNOWN_POSITION;
        }

        // Micro-Manager uses zero-based states.
        // Picard uses one-based physical positions.
        int picardDestination =
            static_cast<int>(requestedPosition + 1);

        int piError = piSetFilterPosition(
            picardDestination,
            handle_);

        if (piError != PI_NO_ERROR)
        {
            property->Set(position_);
            return InterpretPiUsbError(piError);
        }

        position_ = requestedPosition;
        targetPicardPosition_ = picardDestination;
        movePending_ = true;
        moveStartTime_ = GetCurrentMMTime();
    }

    return DEVICE_OK;
}


///////////////////////////////////////////////////////////////////////////////
// Busy state
///////////////////////////////////////////////////////////////////////////////

bool CPicardFilterWheel::Busy()
{
    if (handle_ == 0)
        return false;

    if (movePending_)
    {
        MM::MMTime elapsed =
            GetCurrentMMTime() - moveStartTime_;

        if (elapsed <
            MM::MMTime::fromMs(BUSY_START_DELAY_MS))
        {
            return true;
        }
    }

    int hardwarePosition = 0;

    int piError = piGetFilterPosition(
        &hardwarePosition,
        handle_);

    if (piError != PI_NO_ERROR)
    {
        LogMessage(
            "Unable to read the Picard filter-wheel position "
            "while checking Busy()",
            false);

        // Busy() cannot return an error code.
        return false;
    }

    if (!movePending_)
    {
        // Zero means the wheel is currently between positions.
        return hardwarePosition == 0;
    }

    if (hardwarePosition == targetPicardPosition_)
    {
        position_ = hardwarePosition - 1;
        targetPicardPosition_ = 0;
        movePending_ = false;
        return false;
    }

    // Position zero means the wheel is between slots.
    // A different positive value means it is passing another slot.
    return true;
}
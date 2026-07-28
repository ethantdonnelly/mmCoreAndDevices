///////////////////////////////////////////////////////////////////////////////
// FILE:          MCM301.cpp
///////////////////////////////////////////////////////////////////////////////
// FILE:          MCM301.cpp
// PROJECT:       Micro-Manager
// SUBSYSTEM:     DeviceAdapters
//-----------------------------------------------------------------------------
// DESCRIPTION:   Thorlabs MCM301 single-axis Z-stage adapter
//
// LICENSE:       This file is distributed under the BSD license.
//
// AUTHORS:       Ethan Donnelly, 2026
//
// This adapter uses the Thorlabs MCM301 Command Library.
///////////////////////////////////////////////////////////////////////////////

#ifdef WIN32
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#endif

#include "MCM301.h"
#include "Thorlabs.h"
#include "MCM301CommandLibrary.h"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstring>
#include <limits>
#include <sstream>
#include <vector>

namespace
{
    const char* const kDescription = "Thorlabs MCM301 single-axis Z stage";

    const char* const kPropControllerIdentifier = "ControllerIdentifier";
    const char* const kPropConnectedIdentifier = "ConnectedIdentifier";
    const char* const kPropSlot = "Slot";
    const char* const kPropInvertDirection = "InvertDirection";
    const char* const kPropHomeOnInitialize = "HomeOnInitialize";
    const char* const kPropHomeDirection = "HomeDirection";
    const char* const kPropUseCurrentAsOrigin = "UseCurrentPositionAsOrigin";
    const char* const kPropMinimumPositionUm = "MinimumPositionUm";
    const char* const kPropMaximumPositionUm = "MaximumPositionUm";
    const char* const kPropOpenTimeoutMs = "OpenTimeoutMs";
    const char* const kPropHomeTimeoutMs = "HomeTimeoutMs";
    const char* const kPropFirmwareVersion = "FirmwareVersion";
    const char* const kPropCpidVersion = "CPIDVersion";
    const char* const kPropStageType = "StageType";
    const char* const kPropStepSizeUm = "StepSizeUm";
    const char* const kPropStatusBits = "StatusBits";
    const char* const kPropCurrentEncoder = "CurrentEncoder";

    const char* const kYes = "Yes";
    const char* const kNo = "No";
    const char* const kAuto = "Auto";
    const char* const kUseControllerSetting = "Use Controller Setting";
    const char* const kClockwise = "Clockwise";
    const char* const kCounterClockwise = "Counter-Clockwise";

    // MCM301 GetMotStatus() bits documented by the supplied SDK header.
    const unsigned int kMovingClockwise = 0x10;
    const unsigned int kMovingCounterClockwise = 0x20;
    const unsigned int kJoggingClockwise = 0x40;
    const unsigned int kJoggingCounterClockwise = 0x80;
    const unsigned int kMotorConnected = 0x100;
    const unsigned int kHoming = 0x200;
    const unsigned int kMotionMask =
        kMovingClockwise |
        kMovingCounterClockwise |
        kJoggingClockwise |
        kJoggingCounterClockwise |
        kHoming;

    std::string Trim(const std::string& value)
    {
        std::string::size_type begin = 0;
        while (begin < value.size() &&
            std::isspace(static_cast<unsigned char>(value[begin])))
        {
            ++begin;
        }

        std::string::size_type end = value.size();
        while (end > begin &&
            std::isspace(static_cast<unsigned char>(value[end - 1])))
        {
            --end;
        }

        std::string result = value.substr(begin, end - begin);
        if (result.size() >= 2)
        {
            const char first = result[0];
            const char last = result[result.size() - 1];
            if ((first == '"' && last == '"') ||
                (first == '\'' && last == '\''))
            {
                result = result.substr(1, result.size() - 2);
            }
        }
        return result;
    }

    std::vector<std::string> FirstDeviceOpenCandidates(const std::string& listText)
    {
        std::string firstLine = listText;
        const std::string::size_type lineEnd = firstLine.find_first_of("\r\n");
        if (lineEnd != std::string::npos)
            firstLine = firstLine.substr(0, lineEnd);

        std::vector<std::string> fields;
        std::stringstream stream(firstLine);
        std::string field;
        while (std::getline(stream, field, ','))
            fields.push_back(Trim(field));

        std::vector<std::string> candidates;

        // The SDK header describes the second field as the serial number, which
        // is the documented Open() argument. Some SDK/sample combinations refer
        // to the first field as the COM/device identifier, so try that second.
        if (fields.size() > 1 && !fields[1].empty())
            candidates.push_back(fields[1]);
        if (!fields.empty() && !fields[0].empty())
        {
            if (candidates.empty() || candidates[0] != fields[0])
                candidates.push_back(fields[0]);
        }

        return candidates;
    }
}

MCM301Stage::MCM301Stage() :
    initialized_(false),
    handle_(-1),
    slot_(1),
    openTimeoutMs_(3000),
    homeTimeoutMs_(60000),
    requestedIdentifier_(kAuto),
    connectedIdentifier_(""),
    homeDirection_(kUseControllerSetting),
    invertDirection_(true),
    homeOnInitialize_(false),
    useCurrentPositionAsOrigin_(true),
    pendingHomeOrigin_(false),
    minimumPositionUm_(-100.0),
    maximumPositionUm_(100.0),
    originNm_(0.0),
    originEncoder_(0),
    stepSizeUm_(0.0),
    lastStatusBits_(0),
    lastSdkError_(0)
{
    InitializeDefaultErrorMessages();

    SetErrorText(ERR_MCM301_SDK_CALL_FAILED,
        "An MCM301 SDK call failed. Check the Core Log for the SDK operation and return code.");
    SetErrorText(ERR_MCM301_NO_CONTROLLER,
        "No MCM301 controller was returned by the SDK List() call.");
    SetErrorText(ERR_MCM301_OPEN_FAILED,
        "The MCM301 controller could not be opened. Close the Thorlabs GUI and verify the controller identifier.");
    SetErrorText(ERR_MCM301_INVALID_SLOT,
        "Invalid MCM301 slot. Use 1, 2, or 3. Legacy values 4, 5, and 6 map to 1, 2, and 3.");
    SetErrorText(ERR_MCM301_SLOT_NOT_READY,
        "The selected MCM301 slot reported a plug-and-play/configuration error.");
    SetErrorText(ERR_MCM301_MOTOR_NOT_CONNECTED,
        "The MCM301 did not report a recognized motor on the selected slot.");
    SetErrorText(ERR_MCM301_INVALID_LIMITS,
        "MinimumPositionUm must be less than MaximumPositionUm.");
    SetErrorText(ERR_MCM301_POSITION_OUT_OF_RANGE,
        "Requested Z position is outside the configured software test range.");
    SetErrorText(ERR_MCM301_ENCODER_OUT_OF_RANGE,
        "Requested encoder position is outside the 32-bit range accepted by the MCM301 SDK.");
    SetErrorText(ERR_MCM301_NOT_INITIALIZED,
        "The MCM301 Z stage is not initialized.");
    SetErrorText(ERR_MCM301_HOME_TIMEOUT,
        "The MCM301 stage did not finish homing before the configured timeout.");

    CreateProperty(MM::g_Keyword_Name,
        g_MCM301StageDeviceName, MM::String, true);
    CreateProperty(MM::g_Keyword_Description,
        kDescription, MM::String, true);

    CreateProperty(kPropControllerIdentifier,
        kAuto, MM::String, false, 0, true);

    CreateProperty(kPropSlot,
        "1", MM::Integer, false, 0, true);
    SetPropertyLimits(kPropSlot, 1, 6);

    CreateProperty(kPropInvertDirection,
        kYes, MM::String, false, 0, true);
    AddAllowedValue(kPropInvertDirection, kYes);
    AddAllowedValue(kPropInvertDirection, kNo);

    CreateProperty(kPropHomeOnInitialize,
        kNo, MM::String, false, 0, true);
    AddAllowedValue(kPropHomeOnInitialize, kYes);
    AddAllowedValue(kPropHomeOnInitialize, kNo);

    CreateProperty(kPropHomeDirection,
        kUseControllerSetting, MM::String, false, 0, true);
    AddAllowedValue(kPropHomeDirection, kUseControllerSetting);
    AddAllowedValue(kPropHomeDirection, kClockwise);
    AddAllowedValue(kPropHomeDirection, kCounterClockwise);

    CreateProperty(kPropUseCurrentAsOrigin,
        kYes, MM::String, false, 0, true);
    AddAllowedValue(kPropUseCurrentAsOrigin, kYes);
    AddAllowedValue(kPropUseCurrentAsOrigin, kNo);

    CreateProperty(kPropMinimumPositionUm,
        "-100.0", MM::Float, false, 0, true);
    CreateProperty(kPropMaximumPositionUm,
        "100.0", MM::Float, false, 0, true);

    CreateProperty(kPropOpenTimeoutMs,
        "3000", MM::Integer, false, 0, true);
    SetPropertyLimits(kPropOpenTimeoutMs, 1, 60000);

    CreateProperty(kPropHomeTimeoutMs,
        "60000", MM::Integer, false, 0, true);
    SetPropertyLimits(kPropHomeTimeoutMs, 1000, 300000);
}

MCM301Stage::~MCM301Stage()
{
    Shutdown();
}

void MCM301Stage::GetName(char* name) const
{
    CDeviceUtils::CopyLimitedString(name, g_MCM301StageDeviceName);
}

int MCM301Stage::Initialize()
{
    if (initialized_)
        return DEVICE_OK;

    long slotValue = 1;
    long openTimeoutValue = 3000;
    long homeTimeoutValue = 60000;
    std::string value;

    char textValue[MM::MaxStrLength] = { 0 };

    GetProperty(kPropControllerIdentifier, textValue);
    requestedIdentifier_ = textValue;

    GetProperty(kPropSlot, slotValue);
    GetProperty(kPropOpenTimeoutMs, openTimeoutValue);
    GetProperty(kPropHomeTimeoutMs, homeTimeoutValue);

    GetProperty(kPropHomeDirection, textValue);
    homeDirection_ = textValue;

    GetProperty(kPropMinimumPositionUm, minimumPositionUm_);
    GetProperty(kPropMaximumPositionUm, maximumPositionUm_);

    GetProperty(kPropInvertDirection, textValue);
    value = textValue;
    invertDirection_ = (value == kYes);

    GetProperty(kPropHomeOnInitialize, textValue);
    value = textValue;
    homeOnInitialize_ = (value == kYes);

    GetProperty(kPropUseCurrentAsOrigin, textValue);
    value = textValue;
    useCurrentPositionAsOrigin_ = (value == kYes);

    if (slotValue < 1 || slotValue > 6)
        return ERR_MCM301_INVALID_SLOT;
    if (minimumPositionUm_ >= maximumPositionUm_)
        return ERR_MCM301_INVALID_LIMITS;

    slot_ = static_cast<char>(slotValue);
    openTimeoutMs_ = static_cast<int>(openTimeoutValue);
    homeTimeoutMs_ = homeTimeoutValue;

    int ret = OpenController();
    if (ret != DEVICE_OK)
        return ret;

    unsigned int pnpStatus = 0;
    ret = CheckSdkResult(::GetPNPStatus(handle_, slot_, &pnpStatus),
        "GetPNPStatus");
    if (ret != DEVICE_OK)
    {
        Shutdown();
        return ret;
    }
    if (pnpStatus != 0)
    {
        std::ostringstream message;
        message << "MCM301 slot " << static_cast<int>(slot_)
            << " PNP status: 0x" << std::hex << pnpStatus;
        LogMessage(message.str(), false);
        Shutdown();
        return ERR_MCM301_SLOT_NOT_READY;
    }

    ret = CheckSdkResult(::SetChanEnableState(handle_, slot_, 1),
        "SetChanEnableState");
    if (ret != DEVICE_OK)
    {
        Shutdown();
        return ret;
    }

    int encoder = 0;
    unsigned int statusBits = 0;
    ret = ReadMotionStatus(encoder, statusBits);
    if (ret != DEVICE_OK)
    {
        Shutdown();
        return ret;
    }
    if ((statusBits & kMotorConnected) == 0)
    {
        Shutdown();
        return ERR_MCM301_MOTOR_NOT_CONNECTED;
    }

    if (homeDirection_ == kClockwise)
    {
        ret = CheckSdkResult(::SetHomeInfo(handle_, slot_, 0), "SetHomeInfo");
        if (ret != DEVICE_OK)
        {
            Shutdown();
            return ret;
        }
    }
    else if (homeDirection_ == kCounterClockwise)
    {
        ret = CheckSdkResult(::SetHomeInfo(handle_, slot_, 1), "SetHomeInfo");
        if (ret != DEVICE_OK)
        {
            Shutdown();
            return ret;
        }
    }

    if (useCurrentPositionAsOrigin_)
    {
        ret = CaptureCurrentAsOrigin();
        if (ret != DEVICE_OK)
        {
            Shutdown();
            return ret;
        }
    }
    else
    {
        originNm_ = 0.0;
        ret = CheckSdkResult(
            ::ConvertnmToEncoder(handle_, slot_, originNm_, &originEncoder_),
            "ConvertnmToEncoder(origin)");
        if (ret != DEVICE_OK)
        {
            Shutdown();
            return ret;
        }
    }

    ret = ComputeStepSize();
    if (ret != DEVICE_OK)
    {
        Shutdown();
        return ret;
    }

    char stageType[64] = { 0 };
    if (::GetSlotDeviceType(handle_, slot_, stageType,
        static_cast<int>(sizeof(stageType))) < 0)
    {
        std::strncpy(stageType, "Unknown", sizeof(stageType) - 1);
        stageType[sizeof(stageType) - 1] = '\0';
    }

    char firmware[3] = { 0 };
    char cpid[2] = { 0 };
    const int hardwareInfoResult =
        ::GetHardwareInfo(handle_, firmware, 3, cpid, 2);

    CreateProperty(kPropConnectedIdentifier,
        connectedIdentifier_.c_str(), MM::String, true);
    CreateProperty(kPropStageType, stageType, MM::String, true);
    CreateProperty(kPropStepSizeUm,
        CDeviceUtils::ConvertToString(stepSizeUm_), MM::Float, true);

    if (hardwareInfoResult >= 0)
    {
        std::ostringstream firmwareText;
        firmwareText << static_cast<int>(static_cast<unsigned char>(firmware[2]))
            << "."
            << static_cast<int>(static_cast<unsigned char>(firmware[1]))
            << "."
            << static_cast<int>(static_cast<unsigned char>(firmware[0]));
        CreateProperty(kPropFirmwareVersion,
            firmwareText.str().c_str(), MM::String, true);

        std::ostringstream cpidText;
        cpidText << static_cast<int>(static_cast<unsigned char>(cpid[0]))
            << "."
            << static_cast<int>(static_cast<unsigned char>(cpid[1]));
        CreateProperty(kPropCpidVersion,
            cpidText.str().c_str(), MM::String, true);
    }

    CPropertyAction* positionAction =
        new CPropertyAction(this, &MCM301Stage::OnPosition);
    CreateProperty(MM::g_Keyword_Position,
        "0.0", MM::Float, false, positionAction);
    SetPropertyLimits(MM::g_Keyword_Position,
        minimumPositionUm_, maximumPositionUm_);

    CPropertyAction* statusAction =
        new CPropertyAction(this, &MCM301Stage::OnStatusBits);
    CreateProperty(kPropStatusBits,
        "0", MM::Integer, true, statusAction);

    CPropertyAction* encoderAction =
        new CPropertyAction(this, &MCM301Stage::OnCurrentEncoder);
    CreateProperty(kPropCurrentEncoder,
        CDeviceUtils::ConvertToString(static_cast<long>(encoder)), MM::Integer, true, encoderAction);

    initialized_ = true;

    if (homeOnInitialize_)
    {
        ret = Home();
        if (ret != DEVICE_OK)
        {
            Shutdown();
            return ret;
        }

        ret = WaitUntilStopped(homeTimeoutMs_);
        if (ret != DEVICE_OK)
        {
            Shutdown();
            return ret;
        }
    }

    ret = UpdateStatus();
    if (ret != DEVICE_OK)
    {
        Shutdown();
        return ret;
    }

    std::ostringstream message;
    message << "Initialized MCM301 Z stage: identifier="
        << connectedIdentifier_
        << ", slot=" << static_cast<int>(slot_)
        << ", invert=" << (invertDirection_ ? "Yes" : "No")
        << ", limits=[" << minimumPositionUm_
        << ", " << maximumPositionUm_ << "] um";
    LogMessage(message.str(), false);

    return DEVICE_OK;
}

int MCM301Stage::Shutdown()
{
    int returnCode = DEVICE_OK;

    if (handle_ >= 0)
    {
        // Stop outstanding movement before releasing the SDK handle.
        ::MoveStop(handle_, slot_);

        const int closeResult = ::Close(handle_);
        if (closeResult < 0)
        {
            lastSdkError_ = closeResult;
            returnCode = ERR_MCM301_SDK_CALL_FAILED;
        }
    }

    handle_ = -1;
    initialized_ = false;
    pendingHomeOrigin_ = false;
    lastStatusBits_ = 0;
    connectedIdentifier_.clear();

    return returnCode;
}

bool MCM301Stage::Busy()
{
    if (!initialized_ || handle_ < 0)
        return false;

    int encoder = 0;
    unsigned int statusBits = 0;
    const int ret = ReadMotionStatus(encoder, statusBits);
    if (ret != DEVICE_OK)
    {
        LogMessage("MCM301 GetMotStatus failed while polling Busy().", false);
        return false;
    }

    const bool moving = StatusIndicatesMotion(statusBits);
    if (!moving && pendingHomeOrigin_)
    {
        if (CaptureCurrentAsOrigin() == DEVICE_OK)
            pendingHomeOrigin_ = false;
    }

    return moving;
}

int MCM301Stage::SetPositionUm(double positionUm)
{
    if (!initialized_)
        return ERR_MCM301_NOT_INITIALIZED;

    int ret = FinalizeHomeOriginIfComplete();
    if (ret != DEVICE_OK)
        return ret;

    if (Busy())
        return ERR_BUSY;

    if (!PositionWithinConfiguredLimits(positionUm))
        return ERR_MCM301_POSITION_OUT_OF_RANGE;

    const double physicalNm =
        originNm_ + DirectionSign() * positionUm * 1000.0;

    int targetEncoder = 0;
    ret = CheckSdkResult(
        ::ConvertnmToEncoder(handle_, slot_, physicalNm, &targetEncoder),
        "ConvertnmToEncoder(SetPositionUm)");
    if (ret != DEVICE_OK)
        return ret;

    ret = CheckSdkResult(
        ::MoveAbsolute(handle_, slot_, targetEncoder),
        "MoveAbsolute(SetPositionUm)");
    if (ret != DEVICE_OK)
        return ret;

    return DEVICE_OK;
}

int MCM301Stage::GetPositionUm(double& positionUm)
{
    if (!initialized_)
        return ERR_MCM301_NOT_INITIALIZED;

    int ret = FinalizeHomeOriginIfComplete();
    if (ret != DEVICE_OK)
        return ret;

    int encoder = 0;
    double physicalNm = 0.0;
    ret = ReadPhysicalPosition(encoder, physicalNm);
    if (ret != DEVICE_OK)
        return ret;

    positionUm = DirectionSign() * (physicalNm - originNm_) / 1000.0;
    return DEVICE_OK;
}

int MCM301Stage::SetPositionSteps(long steps)
{
    if (!initialized_)
        return ERR_MCM301_NOT_INITIALIZED;

    int ret = FinalizeHomeOriginIfComplete();
    if (ret != DEVICE_OK)
        return ret;

    if (Busy())
        return ERR_BUSY;

    const long long signedSteps = invertDirection_ ? -static_cast<long long>(steps)
        : static_cast<long long>(steps);
    const long long target64 =
        static_cast<long long>(originEncoder_) + signedSteps;

    if (target64 < static_cast<long long>(std::numeric_limits<int>::min()) ||
        target64 > static_cast<long long>(std::numeric_limits<int>::max()))
    {
        return ERR_MCM301_ENCODER_OUT_OF_RANGE;
    }

    const int targetEncoder = static_cast<int>(target64);
    double targetNm = 0.0;
    ret = CheckSdkResult(
        ::ConvertEncoderTonm(handle_, slot_, targetEncoder, &targetNm),
        "ConvertEncoderTonm(SetPositionSteps)");
    if (ret != DEVICE_OK)
        return ret;

    const double targetUm =
        DirectionSign() * (targetNm - originNm_) / 1000.0;
    if (!PositionWithinConfiguredLimits(targetUm))
        return ERR_MCM301_POSITION_OUT_OF_RANGE;

    return CheckSdkResult(
        ::MoveAbsolute(handle_, slot_, targetEncoder),
        "MoveAbsolute(SetPositionSteps)");
}

int MCM301Stage::GetPositionSteps(long& steps)
{
    if (!initialized_)
        return ERR_MCM301_NOT_INITIALIZED;

    int ret = FinalizeHomeOriginIfComplete();
    if (ret != DEVICE_OK)
        return ret;

    int encoder = 0;
    unsigned int statusBits = 0;
    ret = ReadMotionStatus(encoder, statusBits);
    if (ret != DEVICE_OK)
        return ret;

    const long long difference =
        static_cast<long long>(encoder) - static_cast<long long>(originEncoder_);
    const long long logicalSteps = invertDirection_ ? -difference : difference;

    if (logicalSteps < static_cast<long long>(std::numeric_limits<long>::min()) ||
        logicalSteps > static_cast<long long>(std::numeric_limits<long>::max()))
    {
        return ERR_MCM301_ENCODER_OUT_OF_RANGE;
    }

    steps = static_cast<long>(logicalSteps);
    return DEVICE_OK;
}

int MCM301Stage::SetOrigin()
{
    if (!initialized_)
        return ERR_MCM301_NOT_INITIALIZED;
    if (Busy())
        return ERR_BUSY;

    return CaptureCurrentAsOrigin();
}

int MCM301Stage::SetAdapterOriginUm(double newPositionUm)
{
    if (!initialized_)
        return ERR_MCM301_NOT_INITIALIZED;
    if (Busy())
        return ERR_BUSY;

    int currentEncoder = 0;
    double currentNm = 0.0;
    int ret = ReadPhysicalPosition(currentEncoder, currentNm);
    if (ret != DEVICE_OK)
        return ret;

    originNm_ = currentNm - DirectionSign() * newPositionUm * 1000.0;
    ret = CheckSdkResult(
        ::ConvertnmToEncoder(handle_, slot_, originNm_, &originEncoder_),
        "ConvertnmToEncoder(SetAdapterOriginUm)");
    if (ret != DEVICE_OK)
        return ret;

    return DEVICE_OK;
}

int MCM301Stage::GetLimits(double& lowerUm, double& upperUm)
{
    lowerUm = minimumPositionUm_;
    upperUm = maximumPositionUm_;
    return DEVICE_OK;
}

int MCM301Stage::Home()
{
    if (!initialized_)
        return ERR_MCM301_NOT_INITIALIZED;
    if (Busy())
        return ERR_BUSY;

    const int ret = CheckSdkResult(::Home(handle_, slot_), "Home");
    if (ret != DEVICE_OK)
        return ret;

    pendingHomeOrigin_ = true;
    return DEVICE_OK;
}

int MCM301Stage::Stop()
{
    if (!initialized_)
        return ERR_MCM301_NOT_INITIALIZED;

    const int ret = CheckSdkResult(::MoveStop(handle_, slot_), "MoveStop");
    if (ret == DEVICE_OK)
        pendingHomeOrigin_ = false;
    return ret;
}

int MCM301Stage::OnPosition(
    MM::PropertyBase* property, MM::ActionType action)
{
    if (action == MM::BeforeGet)
    {
        double positionUm = 0.0;
        const int ret = GetPositionUm(positionUm);
        if (ret != DEVICE_OK)
            return ret;
        property->Set(positionUm);
    }
    else if (action == MM::AfterSet)
    {
        double positionUm = 0.0;
        property->Get(positionUm);
        return SetPositionUm(positionUm);
    }

    return DEVICE_OK;
}

int MCM301Stage::OnStatusBits(
    MM::PropertyBase* property, MM::ActionType action)
{
    if (action == MM::BeforeGet)
    {
        int encoder = 0;
        unsigned int statusBits = 0;
        const int ret = ReadMotionStatus(encoder, statusBits);
        if (ret != DEVICE_OK)
            return ret;
        property->Set(static_cast<long>(statusBits));
    }

    return DEVICE_OK;
}

int MCM301Stage::OnCurrentEncoder(
    MM::PropertyBase* property, MM::ActionType action)
{
    if (action == MM::BeforeGet)
    {
        int encoder = 0;
        unsigned int statusBits = 0;
        const int ret = ReadMotionStatus(encoder, statusBits);
        if (ret != DEVICE_OK)
            return ret;
        property->Set(static_cast<long>(encoder));
    }

    return DEVICE_OK;
}

int MCM301Stage::OpenController()
{
    connectedIdentifier_.clear();

    if (requestedIdentifier_ != kAuto && !requestedIdentifier_.empty())
        return TryOpenIdentifier(requestedIdentifier_);

    char listBuffer[4096] = { 0 };
    const int count = ::List(listBuffer, static_cast<int>(sizeof(listBuffer)));
    if (count < 0)
        return CheckSdkResult(count, "List");
    if (count == 0)
        return ERR_MCM301_NO_CONTROLLER;

    const std::string listText(listBuffer);
    LogMessage(std::string("MCM301 List(): ") + listText, false);

    const std::vector<std::string> candidates =
        FirstDeviceOpenCandidates(listText);
    for (std::vector<std::string>::const_iterator it = candidates.begin();
        it != candidates.end(); ++it)
    {
        if (TryOpenIdentifier(*it) == DEVICE_OK)
            return DEVICE_OK;
    }

    return ERR_MCM301_OPEN_FAILED;
}

int MCM301Stage::TryOpenIdentifier(const std::string& identifier)
{
    if (identifier.empty())
        return ERR_MCM301_OPEN_FAILED;

    std::vector<char> mutableIdentifier(identifier.begin(), identifier.end());
    mutableIdentifier.push_back('\0');

    const int candidateHandle =
        ::Open(&mutableIdentifier[0], 115200, openTimeoutMs_);
    if (candidateHandle < 0)
    {
        lastSdkError_ = candidateHandle;
        std::ostringstream message;
        message << "MCM301 Open failed for identifier '" << identifier
            << "' with SDK return code " << candidateHandle;
        LogMessage(message.str(), false);
        return ERR_MCM301_OPEN_FAILED;
    }

    handle_ = candidateHandle;
    connectedIdentifier_ = identifier;
    return DEVICE_OK;
}

int MCM301Stage::ReadMotionStatus(
    int& encoder, unsigned int& statusBits)
{
    if (handle_ < 0)
        return ERR_MCM301_NOT_INITIALIZED;

    const int ret = CheckSdkResult(
        ::GetMotStatus(handle_, slot_, &encoder, &statusBits),
        "GetMotStatus");
    if (ret == DEVICE_OK)
        lastStatusBits_ = statusBits;
    return ret;
}

int MCM301Stage::ReadPhysicalPosition(int& encoder, double& positionNm)
{
    unsigned int statusBits = 0;
    int ret = ReadMotionStatus(encoder, statusBits);
    if (ret != DEVICE_OK)
        return ret;

    ret = CheckSdkResult(
        ::ConvertEncoderTonm(handle_, slot_, encoder, &positionNm),
        "ConvertEncoderTonm");
    return ret;
}

int MCM301Stage::CaptureCurrentAsOrigin()
{
    int encoder = 0;
    double positionNm = 0.0;
    const int ret = ReadPhysicalPosition(encoder, positionNm);
    if (ret != DEVICE_OK)
        return ret;

    originEncoder_ = encoder;
    originNm_ = positionNm;
    return ComputeStepSize();
}

int MCM301Stage::FinalizeHomeOriginIfComplete()
{
    if (!pendingHomeOrigin_)
        return DEVICE_OK;

    int encoder = 0;
    unsigned int statusBits = 0;
    int ret = ReadMotionStatus(encoder, statusBits);
    if (ret != DEVICE_OK)
        return ret;

    if (StatusIndicatesMotion(statusBits))
        return DEVICE_OK;

    ret = CaptureCurrentAsOrigin();
    if (ret == DEVICE_OK)
        pendingHomeOrigin_ = false;
    return ret;
}

int MCM301Stage::WaitUntilStopped(long timeoutMs)
{
    const MM::MMTime start = GetCurrentMMTime();

    while (true)
    {
        int encoder = 0;
        unsigned int statusBits = 0;
        int ret = ReadMotionStatus(encoder, statusBits);
        if (ret != DEVICE_OK)
            return ret;

        if (!StatusIndicatesMotion(statusBits))
        {
            if (pendingHomeOrigin_)
            {
                ret = CaptureCurrentAsOrigin();
                if (ret != DEVICE_OK)
                    return ret;
                pendingHomeOrigin_ = false;
            }
            return DEVICE_OK;
        }

        if ((GetCurrentMMTime() - start).getMsec() > timeoutMs)
        {
            ::MoveStop(handle_, slot_);
            pendingHomeOrigin_ = false;
            return ERR_MCM301_HOME_TIMEOUT;
        }

        CDeviceUtils::SleepMs(50);
    }
}

int MCM301Stage::CheckSdkResult(
    int sdkResult, const char* operation)
{
    if (sdkResult >= 0)
        return DEVICE_OK;

    lastSdkError_ = sdkResult;

    std::ostringstream message;
    message << "MCM301 SDK call " << operation
        << " failed with return code " << sdkResult;

    if (handle_ >= 0)
    {
        const int deviceErrorState = ::GetErrorState(handle_);
        message << "; GetErrorState=" << deviceErrorState;
    }

    LogMessage(message.str(), false);
    return ERR_MCM301_SDK_CALL_FAILED;
}

int MCM301Stage::ComputeStepSize()
{
    if (handle_ < 0)
        return ERR_MCM301_NOT_INITIALIZED;

    int neighborEncoder = originEncoder_;
    if (neighborEncoder < std::numeric_limits<int>::max())
        ++neighborEncoder;
    else if (neighborEncoder > std::numeric_limits<int>::min())
        --neighborEncoder;
    else
        return ERR_MCM301_ENCODER_OUT_OF_RANGE;

    double originPositionNm = 0.0;
    double neighborPositionNm = 0.0;

    int ret = CheckSdkResult(
        ::ConvertEncoderTonm(handle_, slot_, originEncoder_, &originPositionNm),
        "ConvertEncoderTonm(step origin)");
    if (ret != DEVICE_OK)
        return ret;

    ret = CheckSdkResult(
        ::ConvertEncoderTonm(handle_, slot_, neighborEncoder, &neighborPositionNm),
        "ConvertEncoderTonm(step neighbor)");
    if (ret != DEVICE_OK)
        return ret;

    stepSizeUm_ = std::fabs(neighborPositionNm - originPositionNm) / 1000.0;
    return DEVICE_OK;
}

bool MCM301Stage::StatusIndicatesMotion(unsigned int statusBits) const
{
    return (statusBits & kMotionMask) != 0;
}

bool MCM301Stage::PositionWithinConfiguredLimits(double positionUm) const
{
    return positionUm >= minimumPositionUm_ &&
        positionUm <= maximumPositionUm_;
}

double MCM301Stage::DirectionSign() const
{
    return invertDirection_ ? -1.0 : 1.0;
}








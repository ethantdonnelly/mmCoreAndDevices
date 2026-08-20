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
// This adapter uses the Thorlabs Kinesis Benchtop Piezo C API.
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

namespace
{
    const char* const kDescription =
        "Thorlabs BPC301 closed-loop piezo Z stage";

    const char* const kPropSerialNumber = "SerialNumber";
    const char* const kPropConnectedSerialNumber = "ConnectedSerialNumber";
    const char* const kPropInvertDirection = "InvertDirection";
    const char* const kPropZeroOnInitialize = "ZeroOnInitialize";
    const char* const kPropPollingIntervalMs = "PollingIntervalMs";
    const char* const kPropBusyDelayMs = "BusyDelayMs";
    const char* const kPropMaximumTravelUm = "MaximumTravelUm";
    const char* const kPropStepSizeUm = "StepSizeUm";
    const char* const kPropControlMode = "ControlMode";
    const char* const kPropStatusBits = "StatusBits";
    const long kZeroTimeoutMs = 100000;

    const char* const kYes = "Yes";
    const char* const kNo = "No";
    const char* const kAuto = "Auto";
    const char* const kClosedLoop = "ClosedLoop";

    // Thorlabs' native BPC example uses type ID 41 for BPC devices.
    const int kBPCDeviceTypeId = 41;

    // BPC301 is a single-channel controller.
    const short kDefaultChannel = 1;

    // Closed-loop position units used by PBC_GetPosition/PBC_SetPosition:
    // 0 .. 32767 corresponds to 0 .. 100% travel for set position.
    const double kFullScalePositionUnits = 32767.0;

    // PBC_GetStatusBits values documented in the supplied Kinesis header.
    const unsigned long kStatusActuatorConnected = 0x00000001UL;
    const unsigned long kStatusZeroed = 0x00000010UL;
    const unsigned long kStatusZeroing = 0x00000020UL;
    const unsigned long kStatusStrainGaugeConnected = 0x00000100UL;
    const unsigned long kStatusClosedLoop = 0x00000400UL;
    const unsigned long kStatusChannelEnabled = 0x80000000UL;

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

        return value.substr(begin, end - begin);
    }

    std::vector<std::string> ParseSerialNumbers(const std::string& listText)
    {
        std::vector<std::string> serialNumbers;
        std::stringstream stream(listText);
        std::string field;

        while (std::getline(stream, field, ','))
        {
            field = Trim(field);
            if (!field.empty())
                serialNumbers.push_back(field);
        }

        return serialNumbers;
    }
}

BPC301Stage::BPC301Stage() :
    initialized_(false),
    polling_(false),
    pendingZero_(false),
    moveCommandIssued_(false),
    channel_(kDefaultChannel),
    pollingIntervalMs_(250),
    busyDelayMs_(10),
    invertDirection_(false),
    zeroOnInitialize_(true),
    requestedSerialNumber_(kAuto),
    connectedSerialNumber_(""),
    maximumTravelUm_(0.0),
    originPhysicalUm_(0.0),
    stepSizeUm_(0.0),
    lastStatusBits_(0),
    lastSdkError_(0)
{
    InitializeDefaultErrorMessages();

    SetErrorText(ERR_BPC301_SDK_CALL_FAILED,
        "A BPC301 Kinesis SDK call failed. Check the Core Log for the SDK operation and return code.");
    SetErrorText(ERR_BPC301_NO_CONTROLLER,
        "No compatible Thorlabs BPC controller was found.");
    SetErrorText(ERR_BPC301_OPEN_FAILED,
        "The BPC301 controller could not be opened. Close Kinesis or other software using the controller and verify the serial number.");
    SetErrorText(ERR_BPC301_INVALID_CHANNEL,
        "Channel 1 is not valid for the connected BPC controller.");
    SetErrorText(ERR_BPC301_POLLING_FAILED,
        "The BPC301 polling loop could not be started.");
    SetErrorText(ERR_BPC301_ENABLE_FAILED,
        "The BPC301 channel could not be enabled.");
    SetErrorText(ERR_BPC301_CONTROL_MODE_FAILED,
        "The BPC301 could not be placed into closed-loop position control mode.");
    SetErrorText(ERR_BPC301_INVALID_MAX_TRAVEL,
        "The BPC301 reported zero or invalid maximum piezo travel. Check the connected actuator and Kinesis configuration.");
    SetErrorText(ERR_BPC301_POSITION_OUT_OF_RANGE,
        "Requested BPC301 position is outside the physical piezo travel range.");
    SetErrorText(ERR_BPC301_NOT_INITIALIZED,
        "The BPC301 Z stage is not initialized.");
    SetErrorText(ERR_BPC301_ZERO_TIMEOUT,
        "The BPC301 did not finish zeroing before the configured timeout.");

    CreateProperty(MM::g_Keyword_Name,
        g_BPC301StageDeviceName, MM::String, true);
    CreateProperty(MM::g_Keyword_Description,
        kDescription, MM::String, true);

    // Pre-initialization controller selection.
    CreateProperty(kPropSerialNumber,
        kAuto, MM::String, false, 0, true);

    CreateProperty(kPropInvertDirection,
        kNo, MM::String, false, 0, true);
    AddAllowedValue(kPropInvertDirection, kYes);
    AddAllowedValue(kPropInvertDirection, kNo);

    // Matches the behavior of the existing Python prototype.
    CreateProperty(kPropZeroOnInitialize,
        kYes, MM::String, false, 0, true);
    AddAllowedValue(kPropZeroOnInitialize, kYes);
    AddAllowedValue(kPropZeroOnInitialize, kNo);

    CreateProperty(kPropPollingIntervalMs,
        "250", MM::Integer, false, 0, true);
    SetPropertyLimits(kPropPollingIntervalMs, 10, 5000);

    // The Python prototype blocks Micro-Manager briefly after a position command.
    CreateProperty(kPropBusyDelayMs,
        "10", MM::Integer, false, 0, true);
    SetPropertyLimits(kPropBusyDelayMs, 0, 5000);
}

BPC301Stage::~BPC301Stage()
{
    Shutdown();
}

void BPC301Stage::GetName(char* name) const
{
    CDeviceUtils::CopyLimitedString(name, g_BPC301StageDeviceName);
}

int BPC301Stage::Initialize()
{
    if (initialized_)
        return DEVICE_OK;

    char textValue[MM::MaxStrLength] = { 0 };
    std::string value;

    long pollingInterval = 250;
    long busyDelay = 10;

    GetProperty(kPropSerialNumber, textValue);
    requestedSerialNumber_ = textValue;

    GetProperty(kPropInvertDirection, textValue);
    value = textValue;
    invertDirection_ = (value == kYes);

    GetProperty(kPropZeroOnInitialize, textValue);
    value = textValue;
    zeroOnInitialize_ = (value == kYes);

    GetProperty(kPropPollingIntervalMs, pollingInterval);
    GetProperty(kPropBusyDelayMs, busyDelay);

    pollingIntervalMs_ = static_cast<int>(pollingInterval);
    busyDelayMs_ = busyDelay;

    int ret = OpenController();
    if (ret != DEVICE_OK)
        return ret;

    // BPC301 is a single-channel controller. Validate channel 1 explicitly.
    if (!::PBC_IsChannelValid(connectedSerialNumber_.c_str(), channel_))
    {
        Shutdown();
        return ERR_BPC301_INVALID_CHANNEL;
    }

    // Load locally stored Kinesis settings when available. The official
    // native example does not require this call, so failure is logged but
    // is not fatal.
    if (!::PBC_LoadSettings(connectedSerialNumber_.c_str(), channel_))
    {
        LogMessage(
            "BPC301 PBC_LoadSettings returned false; continuing with controller settings.",
            false);
    }

    if (!::PBC_StartPolling(
        connectedSerialNumber_.c_str(), channel_, pollingIntervalMs_))
    {
        Shutdown();
        return ERR_BPC301_POLLING_FAILED;
    }
    polling_ = true;

    // Give the first poll time to populate cached device values.
    CDeviceUtils::SleepMs(pollingIntervalMs_);

    ret = CheckSdkResult(
        ::PBC_EnableChannel(connectedSerialNumber_.c_str(), channel_),
        "PBC_EnableChannel");
    if (ret != DEVICE_OK)
    {
        Shutdown();
        return ERR_BPC301_ENABLE_FAILED;
    }

    // The Micro-Manager Stage API is expressed in physical position (um),
    // therefore force closed-loop position mode. PBC_SetPosition is ignored
    // in open-loop mode according to the supplied Kinesis header.
    ret = CheckSdkResult(
        ::PBC_SetPositionControlMode(
            connectedSerialNumber_.c_str(),
            channel_,
            PZ_ControlModeTypes::PZ_CloseLoop),
        "PBC_SetPositionControlMode(PZ_CloseLoop)");
    if (ret != DEVICE_OK)
    {
        Shutdown();
        return ERR_BPC301_CONTROL_MODE_FAILED;
    }

    CDeviceUtils::SleepMs(pollingIntervalMs_);

    // Ask the controller for travel information, then read the cached value.
    if (!::PBC_RequestMaximumTravel(
        connectedSerialNumber_.c_str(), channel_))
    {
        LogMessage(
            "BPC301 PBC_RequestMaximumTravel returned false; attempting to use cached maximum travel.",
            false);
    }

    CDeviceUtils::SleepMs(pollingIntervalMs_);

    const unsigned short maximumTravelDeviceUnits =
        ::PBC_GetMaximumTravel(
            connectedSerialNumber_.c_str(), channel_);

    // The supplied Kinesis header documents maximum travel in 100 nm units.
    maximumTravelUm_ =
        static_cast<double>(maximumTravelDeviceUnits) * 0.1;

    if (!(maximumTravelUm_ > 0.0) ||
        !std::isfinite(maximumTravelUm_))
    {
        Shutdown();
        return ERR_BPC301_INVALID_MAX_TRAVEL;
    }

    stepSizeUm_ = maximumTravelUm_ / kFullScalePositionUnits;

    // Hardware physical zero corresponds to logical zero until the user
    // changes the software origin.
    originPhysicalUm_ = 0.0;

    if (zeroOnInitialize_)
    {
        ret = CheckSdkResult(
            ::PBC_SetZero(connectedSerialNumber_.c_str(), channel_),
            "PBC_SetZero(Initialize)");
        if (ret != DEVICE_OK)
        {
            Shutdown();
            return ret;
        }

        pendingZero_ = true;

        // Allow one polling interval before checking zeroing state so that
        // PBC_GetStatusBits is not reading stale pre-command status.
        CDeviceUtils::SleepMs(pollingIntervalMs_);

        ret = WaitForZeroComplete(kZeroTimeoutMs);
        if (ret != DEVICE_OK)
        {
            Shutdown();
            return ret;
        }
    }

    CreateProperty(kPropConnectedSerialNumber,
        connectedSerialNumber_.c_str(), MM::String, true);

    CreateProperty(kPropMaximumTravelUm,
        CDeviceUtils::ConvertToString(maximumTravelUm_),
        MM::Float, true);

    CreateProperty(kPropStepSizeUm,
        CDeviceUtils::ConvertToString(stepSizeUm_),
        MM::Float, true);

    CreateProperty(kPropControlMode,
        kClosedLoop, MM::String, true);

    CPropertyAction* positionAction =
        new CPropertyAction(this, &BPC301Stage::OnPosition);
    CreateProperty(MM::g_Keyword_Position,
        "0.0", MM::Float, false, positionAction);

    CPropertyAction* statusAction =
        new CPropertyAction(this, &BPC301Stage::OnStatusBits);
    CreateProperty(kPropStatusBits,
        "0", MM::Integer, true, statusAction);

    initialized_ = true;

    UpdatePositionPropertyLimits();

    ret = UpdateStatus();
    if (ret != DEVICE_OK)
    {
        Shutdown();
        return ret;
    }

    unsigned long statusBits = 0;
    ReadStatusBits(statusBits);

    std::ostringstream message;
    message << "Initialized BPC301 Z stage: serial="
        << connectedSerialNumber_
        << ", channel=" << channel_
        << ", maxTravel=" << maximumTravelUm_ << " um"
        << ", stepSize=" << stepSizeUm_ << " um"
        << ", invert=" << (invertDirection_ ? "Yes" : "No")
        << ", zeroOnInitialize=" << (zeroOnInitialize_ ? "Yes" : "No")
        << ", status=0x" << std::hex << statusBits;

    LogMessage(message.str(), false);

    if ((statusBits & kStatusStrainGaugeConnected) == 0)
    {
        LogMessage(
            "BPC301 warning: status does not currently report strain-gauge feedback connected.",
            false);
    }

    if ((statusBits & kStatusClosedLoop) == 0)
    {
        LogMessage(
            "BPC301 warning: status does not currently report closed-loop mode.",
            false);
    }

    return DEVICE_OK;
}

int BPC301Stage::Shutdown()
{
    if (!connectedSerialNumber_.empty())
    {
        if (polling_)
        {
            ::PBC_StopPolling(
                connectedSerialNumber_.c_str(), channel_);
        }

        // Do not disable the channel here. Disabling removes power from the
        // piezo actuator; simply closing communications is less disruptive.
        ::PBC_Close(connectedSerialNumber_.c_str());
    }

    initialized_ = false;
    polling_ = false;
    pendingZero_ = false;
    moveCommandIssued_ = false;

    connectedSerialNumber_.clear();
    maximumTravelUm_ = 0.0;
    stepSizeUm_ = 0.0;
    originPhysicalUm_ = 0.0;
    lastStatusBits_ = 0;

    return DEVICE_OK;
}

bool BPC301Stage::Busy()
{
    if (!initialized_)
        return false;

    if (pendingZero_)
    {
        unsigned long statusBits = 0;
        if (ReadStatusBits(statusBits) == DEVICE_OK)
        {
            if ((statusBits & kStatusZeroing) != 0)
                return true;

            // Once zeroing is complete, the hardware zero is the adapter zero.
            if ((statusBits & kStatusZeroed) != 0)
            {
                pendingZero_ = false;
                originPhysicalUm_ = 0.0;
                UpdatePositionPropertyLimits();
            }
        }
    }

    if (moveCommandIssued_ && busyDelayMs_ > 0)
    {
        const double elapsedMs =
            (GetCurrentMMTime() - lastMoveTime_).getMsec();

        if (elapsedMs < static_cast<double>(busyDelayMs_))
            return true;

        moveCommandIssued_ = false;
    }

    return false;
}

int BPC301Stage::SetPositionUm(double positionUm)
{
    if (!initialized_)
        return ERR_BPC301_NOT_INITIALIZED;

    if (Busy())
        return ERR_BUSY;

    const double physicalPositionUm =
        originPhysicalUm_ + DirectionSign() * positionUm;

    if (!PositionWithinHardwareLimits(physicalPositionUm))
        return ERR_BPC301_POSITION_OUT_OF_RANGE;

    double scaled =
        physicalPositionUm / maximumTravelUm_ *
        kFullScalePositionUnits;

    scaled = std::max(0.0,
        std::min(kFullScalePositionUnits, scaled));

    const short devicePosition =
        static_cast<short>(std::lround(scaled));

    const int ret = CheckSdkResult(
        ::PBC_SetPosition(
            connectedSerialNumber_.c_str(),
            channel_,
            devicePosition),
        "PBC_SetPosition");

    if (ret != DEVICE_OK)
        return ret;

    lastMoveTime_ = GetCurrentMMTime();
    moveCommandIssued_ = true;

    return DEVICE_OK;
}

int BPC301Stage::GetPositionUm(double& positionUm)
{
    if (!initialized_)
        return ERR_BPC301_NOT_INITIALIZED;

    double physicalPositionUm = 0.0;
    const int ret = ReadPhysicalPosition(physicalPositionUm);
    if (ret != DEVICE_OK)
        return ret;

    positionUm =
        DirectionSign() *
        (physicalPositionUm - originPhysicalUm_);

    return DEVICE_OK;
}

int BPC301Stage::SetPositionSteps(long steps)
{
    if (!initialized_)
        return ERR_BPC301_NOT_INITIALIZED;

    const double positionUm =
        static_cast<double>(steps) * stepSizeUm_;

    return SetPositionUm(positionUm);
}

int BPC301Stage::GetPositionSteps(long& steps)
{
    if (!initialized_)
        return ERR_BPC301_NOT_INITIALIZED;

    double positionUm = 0.0;
    const int ret = GetPositionUm(positionUm);
    if (ret != DEVICE_OK)
        return ret;

    if (!(stepSizeUm_ > 0.0))
        return ERR_BPC301_INVALID_MAX_TRAVEL;

    const double stepsDouble = positionUm / stepSizeUm_;

    if (stepsDouble <
        static_cast<double>(std::numeric_limits<long>::min()) ||
        stepsDouble >
        static_cast<double>(std::numeric_limits<long>::max()))
    {
        return ERR_BPC301_POSITION_OUT_OF_RANGE;
    }

    steps = static_cast<long>(std::lround(stepsDouble));
    return DEVICE_OK;
}

int BPC301Stage::SetOrigin()
{
    if (!initialized_)
        return ERR_BPC301_NOT_INITIALIZED;

    if (Busy())
        return ERR_BUSY;

    double physicalPositionUm = 0.0;
    const int ret = ReadPhysicalPosition(physicalPositionUm);
    if (ret != DEVICE_OK)
        return ret;

    originPhysicalUm_ = physicalPositionUm;
    UpdatePositionPropertyLimits();

    return DEVICE_OK;
}

int BPC301Stage::SetAdapterOriginUm(double newPositionUm)
{
    if (!initialized_)
        return ERR_BPC301_NOT_INITIALIZED;

    if (Busy())
        return ERR_BUSY;

    double physicalPositionUm = 0.0;
    const int ret = ReadPhysicalPosition(physicalPositionUm);
    if (ret != DEVICE_OK)
        return ret;

    originPhysicalUm_ =
        physicalPositionUm -
        DirectionSign() * newPositionUm;

    UpdatePositionPropertyLimits();
    return DEVICE_OK;
}

int BPC301Stage::GetLimits(double& lowerUm, double& upperUm)
{
    if (!(maximumTravelUm_ > 0.0))
    {
        lowerUm = 0.0;
        upperUm = 0.0;
        return DEVICE_OK;
    }

    if (!invertDirection_)
    {
        lowerUm = -originPhysicalUm_;
        upperUm = maximumTravelUm_ - originPhysicalUm_;
    }
    else
    {
        lowerUm = originPhysicalUm_ - maximumTravelUm_;
        upperUm = originPhysicalUm_;
    }

    return DEVICE_OK;
}

int BPC301Stage::Home()
{
    if (!initialized_)
        return ERR_BPC301_NOT_INITIALIZED;

    if (Busy())
        return ERR_BUSY;

    const int ret = CheckSdkResult(
        ::PBC_SetZero(
            connectedSerialNumber_.c_str(), channel_),
        "PBC_SetZero(Home)");

    if (ret != DEVICE_OK)
        return ret;

    pendingZero_ = true;
    lastMoveTime_ = GetCurrentMMTime();
    moveCommandIssued_ = true;

    return DEVICE_OK;
}

int BPC301Stage::Stop()
{
    if (!initialized_)
        return ERR_BPC301_NOT_INITIALIZED;

    // The BPC API does not expose a general stop command for normal
    // closed-loop SetPosition moves. SetPosition updates the piezo's
    // target directly, so there is no queued motor trajectory to cancel.
    return DEVICE_OK;
}

int BPC301Stage::OnPosition(
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

int BPC301Stage::OnStatusBits(
    MM::PropertyBase* property, MM::ActionType action)
{
    if (action == MM::BeforeGet)
    {
        unsigned long statusBits = 0;
        const int ret = ReadStatusBits(statusBits);
        if (ret != DEVICE_OK)
            return ret;

        property->Set(static_cast<long>(statusBits));
    }

    return DEVICE_OK;
}

int BPC301Stage::OpenController()
{
    connectedSerialNumber_.clear();

    const short buildResult = ::TLI_BuildDeviceList();
    if (buildResult != 0)
        return CheckSdkResult(buildResult, "TLI_BuildDeviceList");

    if (requestedSerialNumber_ != kAuto &&
        !requestedSerialNumber_.empty())
    {
        return TryOpenSerial(requestedSerialNumber_);
    }

    char listBuffer[4096] = { 0 };
    const short listResult =
        ::TLI_GetDeviceListByTypeExt(
            listBuffer,
            static_cast<unsigned long>(sizeof(listBuffer)),
            kBPCDeviceTypeId);

    if (listResult != 0)
        return CheckSdkResult(
            listResult, "TLI_GetDeviceListByTypeExt");

    const std::string listText(listBuffer);
    if (listText.empty())
        return ERR_BPC301_NO_CONTROLLER;

    LogMessage(
        std::string("BPC301 device list: ") + listText,
        false);

    const std::vector<std::string> serialNumbers =
        ParseSerialNumbers(listText);

    for (std::vector<std::string>::const_iterator it =
        serialNumbers.begin();
        it != serialNumbers.end();
        ++it)
    {
        if (TryOpenSerial(*it) == DEVICE_OK)
            return DEVICE_OK;
    }

    return ERR_BPC301_OPEN_FAILED;
}

int BPC301Stage::TryOpenSerial(
    const std::string& serialNumber)
{
    if (serialNumber.empty())
        return ERR_BPC301_OPEN_FAILED;

    const short openResult =
        ::PBC_Open(serialNumber.c_str());

    if (openResult != 0)
    {
        lastSdkError_ = openResult;

        std::ostringstream message;
        message << "BPC301 PBC_Open failed for serial '"
            << serialNumber
            << "' with SDK return code "
            << openResult;

        LogMessage(message.str(), false);
        return ERR_BPC301_OPEN_FAILED;
    }

    if (!::PBC_IsChannelValid(
        serialNumber.c_str(), channel_))
    {
        ::PBC_Close(serialNumber.c_str());
        return ERR_BPC301_INVALID_CHANNEL;
    }

    connectedSerialNumber_ = serialNumber;
    return DEVICE_OK;
}

int BPC301Stage::ReadPhysicalPosition(
    double& positionUm)
{
    if (connectedSerialNumber_.empty())
        return ERR_BPC301_NOT_INITIALIZED;

    if (!::PBC_CheckConnection(
        connectedSerialNumber_.c_str()))
    {
        LogMessage(
            "BPC301 PBC_CheckConnection returned false while reading position.",
            false);
        return ERR_BPC301_SDK_CALL_FAILED;
    }

    // With polling active this is the latest cached closed-loop position.
    const short devicePosition =
        ::PBC_GetPosition(
            connectedSerialNumber_.c_str(), channel_);

    positionUm =
        static_cast<double>(devicePosition) /
        kFullScalePositionUnits *
        maximumTravelUm_;

    return DEVICE_OK;
}

int BPC301Stage::ReadStatusBits(
    unsigned long& statusBits)
{
    if (connectedSerialNumber_.empty())
        return ERR_BPC301_NOT_INITIALIZED;

    if (!::PBC_CheckConnection(
        connectedSerialNumber_.c_str()))
    {
        LogMessage(
            "BPC301 PBC_CheckConnection returned false while reading status.",
            false);
        return ERR_BPC301_SDK_CALL_FAILED;
    }

    statusBits =
        static_cast<unsigned long>(
            ::PBC_GetStatusBits(
                connectedSerialNumber_.c_str(), channel_));

    lastStatusBits_ = statusBits;
    return DEVICE_OK;
}

int BPC301Stage::WaitForZeroComplete(long timeoutMs)
{
    const MM::MMTime start = GetCurrentMMTime();
    bool sawZeroing = false;

    while (true)
    {
        unsigned long statusBits = 0;
        const int ret = ReadStatusBits(statusBits);
        if (ret != DEVICE_OK)
            return ret;

        if ((statusBits & kStatusZeroing) != 0)
        {
            sawZeroing = true;
        }
        else if ((statusBits & kStatusZeroed) != 0 ||
            sawZeroing)
        {
            pendingZero_ = false;
            originPhysicalUm_ = 0.0;
            return DEVICE_OK;
        }

        if ((GetCurrentMMTime() - start).getMsec() >
            static_cast<double>(timeoutMs))
        {
            pendingZero_ = false;
            return ERR_BPC301_ZERO_TIMEOUT;
        }

        CDeviceUtils::SleepMs(50);
    }
}

int BPC301Stage::CheckSdkResult(
    short sdkResult, const char* operation)
{
    // The supplied Kinesis header documents zero as success and non-zero
    // values as FT-style error codes.
    if (sdkResult == 0)
        return DEVICE_OK;

    lastSdkError_ = sdkResult;

    std::ostringstream message;
    message << "BPC301 SDK call "
        << operation
        << " failed with return code "
        << sdkResult;

    LogMessage(message.str(), false);
    return ERR_BPC301_SDK_CALL_FAILED;
}

bool BPC301Stage::PositionWithinHardwareLimits(
    double physicalPositionUm) const
{
    return physicalPositionUm >= 0.0 &&
        physicalPositionUm <= maximumTravelUm_;
}

double BPC301Stage::DirectionSign() const
{
    return invertDirection_ ? -1.0 : 1.0;
}

void BPC301Stage::UpdatePositionPropertyLimits()
{
    if (!initialized_)
        return;

    double lowerUm = 0.0;
    double upperUm = 0.0;
    GetLimits(lowerUm, upperUm);

    SetPropertyLimits(
        MM::g_Keyword_Position,
        lowerUm,
        upperUm);
}
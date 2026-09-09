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

namespace
{
    const short kXChannel = 1;
    const short kYChannel = 2;
    const int kPollingIntervalMs = 250;
    const unsigned long kInitializeHomeTimeoutMs = 60000;
    const unsigned long kMoveTimeoutMs = 10000;
    const unsigned long kMoveWaitIntervalMs = 10;
    const double kCommandToleranceUm = 25.0;

    // Default serial number
    const char* const kBBD302SerialNumber = "103467624";

    const char* const kPropSerialNumber = "SerialNumber";
    const char* const kPropFlipX = "Flip X";
    const char* const kPropFlipY = "Flip Y";
    const char* const kYes = "Yes";
    const char* const kNo = "No";

    // BMC_GetStatusBits flags documented by the supplied Kinesis header.
    const DWORD kStatusMovingCW = 0x00000010;
    const DWORD kStatusMovingCCW = 0x00000020;
    const DWORD kStatusJoggingCW = 0x00000040;
    const DWORD kStatusJoggingCCW = 0x00000080;
    const DWORD kStatusHoming = 0x00000200;
    const DWORD kStatusHomed = 0x00000400;

    const WORD kMessageTypeGenericMotor = 2;
    const WORD kMessageIdMoved = 1;
    const WORD kMessageIdStopped = 2;

    std::string Trim(const std::string& value)
    {
        const std::string whitespace = " \t\r\n";
        const std::string::size_type first = value.find_first_not_of(whitespace);
        if (first == std::string::npos)
            return std::string();

        const std::string::size_type last = value.find_last_not_of(whitespace);
        return value.substr(first, last - first + 1);
    }

    std::vector<std::string> SplitDeviceList(const char* buffer)
    {
        std::vector<std::string> serials;
        if (buffer == 0)
            return serials;

        std::stringstream stream(buffer);
        std::string item;
        while (std::getline(stream, item, ','))
        {
            item = Trim(item);
            if (!item.empty())
                serials.push_back(item);
        }
        return serials;
    }

    void SleepMs(unsigned long milliseconds)
    {
#ifdef WIN32
        ::Sleep(milliseconds);
#else
        (void)milliseconds;
#endif
    }

    bool InLongRange(long long value)
    {
        return value >= static_cast<long long>((std::numeric_limits<long>::min)()) &&
            value <= static_cast<long long>((std::numeric_limits<long>::max)());
    }
}

BBD302Stage::BBD302Stage() :
    CXYStageBase<BBD302Stage>(),
    initialized_(false),
    opened_(false),
    pollingX_(false),
    pollingY_(false),
    flipX_(false),
    flipY_(false),
    serialNo_(kBBD302SerialNumber),
    xMinSteps_(0),
    xMaxSteps_(0),
    yMinSteps_(0),
    yMaxSteps_(0),
    xMinUm_(0.0),
    xMaxUm_(0.0),
    yMinUm_(0.0),
    yMaxUm_(0.0),
    stepSizeXUm_(1.0),
    stepSizeYUm_(1.0)
{
    InitializeDefaultErrorMessages();

    SetErrorText(ERR_BBD302_DEVICE_NOT_FOUND,
        "No suitable two-channel Thorlabs brushless controller was found.");
    SetErrorText(ERR_BBD302_OPEN_FAILED,
        "Failed to open the Thorlabs BBD302 controller.");
    SetErrorText(ERR_BBD302_CHANNEL_ENABLE_FAILED,
        "Failed to enable one or both BBD302 channels.");
    SetErrorText(ERR_BBD302_POLLING_FAILED,
        "Failed to start polling one or both BBD302 channels.");
    SetErrorText(ERR_BBD302_HOME_FAILED,
        "Failed to home the BBD302 XY stage.");
    SetErrorText(ERR_BBD302_MOVE_FAILED,
        "BBD302 failed to start the requested XY move.");
    SetErrorText(ERR_BBD302_POSITION_READ_FAILED,
        "Failed to read the BBD302 XY position.");
    SetErrorText(ERR_BBD302_STOP_FAILED,
        "Failed to stop BBD302 motion.");
    SetErrorText(ERR_BBD302_NOT_HOMED,
        "The BBD302 stage must be homed before moving.");
    SetErrorText(ERR_BBD302_INVALID_SERIAL,
        "The configured serial number is not a suitable two-channel brushless controller.");
    SetErrorText(ERR_BBD302_CONFIGURATION_FAILED,
        "Failed to obtain BBD302 stage configuration or unit conversion information.");
    SetErrorText(ERR_BBD302_MOVE_TIMEOUT,
        "Timed out waiting for the BBD302 move to complete.");

    CreateProperty(MM::g_Keyword_Name, g_BBD302StageDeviceName,
        MM::String, true);
    CreateProperty(MM::g_Keyword_Description,
        "Thorlabs BBD302 two-channel brushless-motor XY stage",
        MM::String, true);

    CPropertyAction* serialAction =
        new CPropertyAction(this, &BBD302Stage::OnSerialNumber);
    CreateProperty(kPropSerialNumber, kBBD302SerialNumber, MM::String, false,
        serialAction, true);

    CPropertyAction* flipXAction =
        new CPropertyAction(this, &BBD302Stage::OnFlipX);
    CreateProperty(kPropFlipX, kNo, MM::String, false, flipXAction, true);
    AddAllowedValue(kPropFlipX, kNo);
    AddAllowedValue(kPropFlipX, kYes);

    CPropertyAction* flipYAction =
        new CPropertyAction(this, &BBD302Stage::OnFlipY);
    CreateProperty(kPropFlipY, kNo, MM::String, false, flipYAction, true);
    AddAllowedValue(kPropFlipY, kNo);
    AddAllowedValue(kPropFlipY, kYes);
}

BBD302Stage::~BBD302Stage()
{
    Shutdown();
}

void BBD302Stage::GetName(char* name) const
{
    CDeviceUtils::CopyLimitedString(name, g_BBD302StageDeviceName);
}

int BBD302Stage::Initialize()
{
    if (initialized_)
        return DEVICE_OK;

    if (serialNo_.empty())
        return ERR_BBD302_INVALID_SERIAL;

    const short listResult = TLI_BuildDeviceList();
    if (listResult != 0)
    {
        LogKinesisError("TLI_BuildDeviceList", listResult);
        return ERR_BBD302_DEVICE_NOT_FOUND;
    }

    const short openResult = BMC_Open(serialNo_.c_str());
    if (openResult != 0)
    {
        LogKinesisError("BMC_Open", openResult);
        return ERR_BBD302_OPEN_FAILED;
    }
    opened_ = true;

    const short channelCount = BMC_GetNumChannels(serialNo_.c_str());
    if (channelCount != 2 ||
        !BMC_IsChannelValid(serialNo_.c_str(), kXChannel) ||
        !BMC_IsChannelValid(serialNo_.c_str(), kYChannel))
    {
        Shutdown();
        return ERR_BBD302_INVALID_SERIAL;
    }

    // This mirrors the Python reference: load each channel's stored stage
    // settings before enabling motion.  Some Kinesis installations can still
    // operate if this returns false, so treat it as a warning and validate the
    // actual unit/limit information below.
    if (!BMC_LoadSettings(serialNo_.c_str(), kXChannel))
        LogMessage("BBD302: BMC_LoadSettings returned false for X channel.", false);
    if (!BMC_LoadSettings(serialNo_.c_str(), kYChannel))
        LogMessage("BBD302: BMC_LoadSettings returned false for Y channel.", false);

    pollingX_ = BMC_StartPolling(serialNo_.c_str(), kXChannel,
        kPollingIntervalMs);
    if (!pollingX_)
    {
        Shutdown();
        return ERR_BBD302_POLLING_FAILED;
    }

    pollingY_ = BMC_StartPolling(serialNo_.c_str(), kYChannel,
        kPollingIntervalMs);
    if (!pollingY_)
    {
        Shutdown();
        return ERR_BBD302_POLLING_FAILED;
    }

    SleepMs(static_cast<unsigned long>(kPollingIntervalMs));

    short enableResult = BMC_EnableChannel(serialNo_.c_str(), kXChannel);
    if (enableResult != 0)
    {
        LogKinesisError("BMC_EnableChannel(X)", enableResult);
        Shutdown();
        return ERR_BBD302_CHANNEL_ENABLE_FAILED;
    }

    enableResult = BMC_EnableChannel(serialNo_.c_str(), kYChannel);
    if (enableResult != 0)
    {
        LogKinesisError("BMC_EnableChannel(Y)", enableResult);
        Shutdown();
        return ERR_BBD302_CHANNEL_ENABLE_FAILED;
    }

    SleepMs(250);

    int ret = ConfigureAxis(kXChannel, xMinSteps_, xMaxSteps_,
        xMinUm_, xMaxUm_, stepSizeXUm_);
    if (ret != DEVICE_OK)
    {
        Shutdown();
        return ret;
    }

    ret = ConfigureAxis(kYChannel, yMinSteps_, yMaxSteps_,
        yMinUm_, yMaxUm_, stepSizeYUm_);
    if (ret != DEVICE_OK)
    {
        Shutdown();
        return ret;
    }

    // Match the supplied Python device behavior: if either axis can home and
    // is not already homed, home both axes during initialization.
    const bool xNeedsHome = BMC_CanHome(serialNo_.c_str(), kXChannel) &&
        !AxisIsHomed(kXChannel);
    const bool yNeedsHome = BMC_CanHome(serialNo_.c_str(), kYChannel) &&
        !AxisIsHomed(kYChannel);
    if (xNeedsHome || yNeedsHome)
    {
        if (xNeedsHome)
        {
            BMC_ClearMessageQueue(serialNo_.c_str(), kXChannel);
            const short homeXResult = BMC_Home(serialNo_.c_str(), kXChannel);
            if (homeXResult != 0)
            {
                LogKinesisError("BMC_Home(X)", homeXResult);
                Shutdown();
                return ERR_BBD302_HOME_FAILED;
            }
        }

        if (yNeedsHome)
        {
            BMC_ClearMessageQueue(serialNo_.c_str(), kYChannel);
            const short homeYResult = BMC_Home(serialNo_.c_str(), kYChannel);
            if (homeYResult != 0)
            {
                LogKinesisError("BMC_Home(Y)", homeYResult);
                if (xNeedsHome)
                    BMC_StopImmediate(serialNo_.c_str(), kXChannel);
                Shutdown();
                return ERR_BBD302_HOME_FAILED;
            }
        }

        ret = WaitForHome(kInitializeHomeTimeoutMs);
        if (ret != DEVICE_OK)
        {
            Shutdown();
            return ret;
        }
    }

    // If serial was auto-detected, expose the resolved value while the
    // pre-initialization property is still writable.
    SetProperty(kPropSerialNumber, serialNo_.c_str());

    initialized_ = true;
    return UpdateStatus();
}

int BBD302Stage::Shutdown()
{
    if (opened_)
    {
        if (pollingX_)
        {
            BMC_StopPolling(serialNo_.c_str(), kXChannel);
            pollingX_ = false;
        }
        if (pollingY_)
        {
            BMC_StopPolling(serialNo_.c_str(), kYChannel);
            pollingY_ = false;
        }

        BMC_Close(serialNo_.c_str());
        opened_ = false;
    }

    initialized_ = false;
    return DEVICE_OK;
}

bool BBD302Stage::Busy()
{
    if (!opened_)
        return false;

    return AxisIsBusy(kXChannel) || AxisIsBusy(kYChannel);
}

int BBD302Stage::SetPositionSteps(long x, long y)
{
    std::lock_guard<std::mutex> moveLock(moveMutex_);

    if (!IsConnected())
        return DEVICE_NOT_CONNECTED;

    if (x < xMinSteps_ || x > xMaxSteps_ ||
        y < yMinSteps_ || y > yMaxSteps_)
        return DEVICE_INVALID_INPUT_PARAM;

    if ((!AxisIsHomed(kXChannel) &&
        !BMC_CanMoveWithoutHomingFirst(serialNo_.c_str(), kXChannel)) ||
        (!AxisIsHomed(kYChannel) &&
            !BMC_CanMoveWithoutHomingFirst(serialNo_.c_str(), kYChannel)))
        return ERR_BBD302_NOT_HOMED;

    const long physicalX = LogicalToPhysicalX(x);
    const long physicalY = LogicalToPhysicalY(y);

    if (physicalX < (std::numeric_limits<int>::min)() ||
        physicalX >(std::numeric_limits<int>::max)() ||
        physicalY < (std::numeric_limits<int>::min)() ||
        physicalY >(std::numeric_limits<int>::max)())
        return DEVICE_INVALID_INPUT_PARAM;

    const int currentX =
        BMC_GetPosition(serialNo_.c_str(), kXChannel);
    const int currentY =
        BMC_GetPosition(serialNo_.c_str(), kYChannel);

    // Ignore very small residual positioning errors so an unchanged
    // axis is not unnecessarily given another tiny correction move.
    const double xErrorUm =
        std::abs(
            (static_cast<double>(currentX) -
                static_cast<double>(physicalX)) *
            stepSizeXUm_);

    const double yErrorUm =
        std::abs(
            (static_cast<double>(currentY) -
                static_cast<double>(physicalY)) *
            stepSizeYUm_);

    const bool moveX = xErrorUm > kCommandToleranceUm;
    const bool moveY = yErrorUm > kCommandToleranceUm;

    // Clear the relevant completion-message queues before commanding motion.
    if (moveX)
        BMC_ClearMessageQueue(serialNo_.c_str(), kXChannel);

    if (moveY)
        BMC_ClearMessageQueue(serialNo_.c_str(), kYChannel);

    // Start X.
    if (moveX)
    {
        const short result =
            BMC_MoveToPosition(serialNo_.c_str(), kXChannel,
                static_cast<int>(physicalX));

        if (result != 0)
        {
            LogKinesisError("BMC_MoveToPosition(X)", result);
            return ERR_BBD302_MOVE_FAILED;
        }
    }

    // Start Y immediately. If X was also commanded, both axes
    // are now moving simultaneously.
    if (moveY)
    {
        const short result =
            BMC_MoveToPosition(serialNo_.c_str(), kYChannel,
                static_cast<int>(physicalY));

        if (result != 0)
        {
            LogKinesisError("BMC_MoveToPosition(Y)", result);

            if (moveX)
                BMC_StopImmediate(serialNo_.c_str(), kXChannel);

            return ERR_BBD302_MOVE_FAILED;
        }
    }

    // Both commands have already been issued, so these waits do not
    // make the physical X/Y movement sequential.
    if (moveX)
    {
        const int waitResult =
            WaitForMoveComplete(kXChannel, kMoveTimeoutMs);

        if (waitResult != DEVICE_OK)
        {
            if (moveY)
                BMC_StopImmediate(serialNo_.c_str(), kYChannel);

            return waitResult;
        }
    }

    if (moveY)
    {
        const int waitResult =
            WaitForMoveComplete(kYChannel, kMoveTimeoutMs);

        if (waitResult != DEVICE_OK)
            return waitResult;
    }

    return DEVICE_OK;
}

int BBD302Stage::GetPositionSteps(long& x, long& y)
{
    if (!IsConnected())
        return DEVICE_NOT_CONNECTED;

    // Polling continuously updates the position cache used by BMC_GetPosition.
    const int physicalX = BMC_GetPosition(serialNo_.c_str(), kXChannel);
    const int physicalY = BMC_GetPosition(serialNo_.c_str(), kYChannel);

    if (!IsConnected())
        return ERR_BBD302_POSITION_READ_FAILED;

    x = PhysicalToLogicalX(static_cast<long>(physicalX));
    y = PhysicalToLogicalY(static_cast<long>(physicalY));
    return DEVICE_OK;
}

int BBD302Stage::Home()
{
    std::lock_guard<std::mutex> moveLock(moveMutex_);

    if (!IsConnected())
        return DEVICE_NOT_CONNECTED;

    if (!BMC_CanHome(serialNo_.c_str(), kXChannel) ||
        !BMC_CanHome(serialNo_.c_str(), kYChannel))
        return ERR_BBD302_HOME_FAILED;

    BMC_ClearMessageQueue(serialNo_.c_str(), kXChannel);

    short result = BMC_Home(serialNo_.c_str(), kXChannel);
    if (result != 0)
    {
        LogKinesisError("BMC_Home(X)", result);
        return ERR_BBD302_HOME_FAILED;
    }

    int waitResult = WaitForAxisHome(kXChannel, kInitializeHomeTimeoutMs);
    if (waitResult != DEVICE_OK)
        return waitResult;

    BMC_ClearMessageQueue(serialNo_.c_str(), kYChannel);

    result = BMC_Home(serialNo_.c_str(), kYChannel);
    if (result != 0)
    {
        LogKinesisError("BMC_Home(Y)", result);
        return ERR_BBD302_HOME_FAILED;
    }

    waitResult = WaitForAxisHome(kYChannel, kInitializeHomeTimeoutMs);
    if (waitResult != DEVICE_OK)
        return waitResult;

    return DEVICE_OK;
}

int BBD302Stage::Stop()
{
    if (!IsConnected())
        return DEVICE_NOT_CONNECTED;

    const short xResult = BMC_StopImmediate(serialNo_.c_str(), kXChannel);
    const short yResult = BMC_StopImmediate(serialNo_.c_str(), kYChannel);

    if (xResult != 0)
        LogKinesisError("BMC_StopImmediate(X)", xResult);
    if (yResult != 0)
        LogKinesisError("BMC_StopImmediate(Y)", yResult);

    return (xResult == 0 && yResult == 0) ? DEVICE_OK : ERR_BBD302_STOP_FAILED;
}

int BBD302Stage::SetOrigin()
{
    // The BBD302 can alter its hardware position counters, but doing that would
    // also change the reference used by the flip transform and stage limits.
    // Let CXYStageBase's software-origin support handle coordinate offsets
    // instead of silently redefining the controller's hardware coordinates.
    return DEVICE_NOT_YET_IMPLEMENTED;
}

int BBD302Stage::GetLimitsUm(double& xMin, double& xMax,
    double& yMin, double& yMax)
{
    xMin = xMinUm_;
    xMax = xMaxUm_;
    yMin = yMinUm_;
    yMax = yMaxUm_;
    return DEVICE_OK;
}

int BBD302Stage::GetStepLimits(long& xMin, long& xMax,
    long& yMin, long& yMax)
{
    xMin = xMinSteps_;
    xMax = xMaxSteps_;
    yMin = yMinSteps_;
    yMax = yMaxSteps_;
    return DEVICE_OK;
}

double BBD302Stage::GetStepSizeXUm()
{
    return stepSizeXUm_;
}

double BBD302Stage::GetStepSizeYUm()
{
    return stepSizeYUm_;
}

int BBD302Stage::IsXYStageSequenceable(bool& isSequenceable) const
{
    isSequenceable = false;
    return DEVICE_OK;
}

int BBD302Stage::OnSerialNumber(MM::PropertyBase* pProp, MM::ActionType eAct)
{
    if (eAct == MM::BeforeGet)
    {
        pProp->Set(serialNo_.c_str());
    }
    else if (eAct == MM::AfterSet)
    {
        std::string value;
        pProp->Get(value);
        serialNo_ = Trim(value);
    }
    return DEVICE_OK;
}

int BBD302Stage::OnFlipX(MM::PropertyBase* pProp, MM::ActionType eAct)
{
    if (eAct == MM::BeforeGet)
    {
        pProp->Set(flipX_ ? kYes : kNo);
    }
    else if (eAct == MM::AfterSet)
    {
        std::string value;
        pProp->Get(value);
        flipX_ = (value == kYes);

        if (initialized_)
        {
            double x = 0.0;
            double y = 0.0;
            if (GetPositionUm(x, y) == DEVICE_OK)
                OnXYStagePositionChanged(x, y);
        }
    }
    return DEVICE_OK;
}

int BBD302Stage::OnFlipY(MM::PropertyBase* pProp, MM::ActionType eAct)
{
    if (eAct == MM::BeforeGet)
    {
        pProp->Set(flipY_ ? kYes : kNo);
    }
    else if (eAct == MM::AfterSet)
    {
        std::string value;
        pProp->Get(value);
        flipY_ = (value == kYes);

        if (initialized_)
        {
            double x = 0.0;
            double y = 0.0;
            if (GetPositionUm(x, y) == DEVICE_OK)
                OnXYStagePositionChanged(x, y);
        }
    }
    return DEVICE_OK;
}

int BBD302Stage::ConfigureAxis(short channel,
    long& minSteps, long& maxSteps,
    double& minUm, double& maxUm,
    double& stepSizeUm)
{
    MOT_StageAxisParameters axisParams;
    std::memset(&axisParams, 0, sizeof(axisParams));

    const short axisResult = BMC_GetStageAxisParamsBlock(serialNo_.c_str(),
        channel, &axisParams);
    if (axisResult != 0)
    {
        LogKinesisError("BMC_GetStageAxisParamsBlock", axisResult);
        return ERR_BBD302_CONFIGURATION_FAILED;
    }

    minSteps = static_cast<long>(axisParams.minPosition);
    maxSteps = static_cast<long>(axisParams.maxPosition);
    if (maxSteps <= minSteps)
        return ERR_BBD302_CONFIGURATION_FAILED;

    double minMm = 0.0;
    double maxMm = 0.0;
    const short travelResult = BMC_GetMotorTravelLimits(serialNo_.c_str(),
        channel, &minMm, &maxMm);

    if (travelResult == 0 && maxMm > minMm)
    {
        minUm = minMm * 1000.0;
        maxUm = maxMm * 1000.0;
    }
    else
    {
        // Fall back to Kinesis' unit conversion using the stage's device-unit
        // limits.  Unit type 0 is distance in the supplied Kinesis header.
        double minReal = 0.0;
        double maxReal = 0.0;
        const short minResult = BMC_GetRealValueFromDeviceUnit(serialNo_.c_str(),
            channel, static_cast<int>(minSteps), &minReal, 0);
        const short maxResult = BMC_GetRealValueFromDeviceUnit(serialNo_.c_str(),
            channel, static_cast<int>(maxSteps), &maxReal, 0);
        if (minResult != 0 || maxResult != 0 || maxReal <= minReal)
        {
            if (minResult != 0)
                LogKinesisError("BMC_GetRealValueFromDeviceUnit(min)", minResult);
            if (maxResult != 0)
                LogKinesisError("BMC_GetRealValueFromDeviceUnit(max)", maxResult);
            return ERR_BBD302_CONFIGURATION_FAILED;
        }

        minUm = minReal * 1000.0;
        maxUm = maxReal * 1000.0;
    }

    if (axisParams.countsPerUnit > 0)
    {
        // For a linear stage, Kinesis real-world distance units are millimeters.
        stepSizeUm = 1000.0 / static_cast<double>(axisParams.countsPerUnit);
    }
    else
    {
        double oneStepMm = 0.0;
        const short conversionResult = BMC_GetRealValueFromDeviceUnit(
            serialNo_.c_str(), channel, 1, &oneStepMm, 0);
        if (conversionResult != 0 || oneStepMm <= 0.0)
        {
            if (conversionResult != 0)
                LogKinesisError("BMC_GetRealValueFromDeviceUnit(step)",
                    conversionResult);
            return ERR_BBD302_CONFIGURATION_FAILED;
        }
        stepSizeUm = oneStepMm * 1000.0;
    }

    return DEVICE_OK;
}

int BBD302Stage::WaitForMoveComplete(short channel,
    unsigned long timeoutMs)
{
    unsigned long elapsedMs = 0;

    while (elapsedMs < timeoutMs)
    {
        if (!IsConnected())
            return DEVICE_NOT_CONNECTED;

        while (BMC_MessageQueueSize(serialNo_.c_str(), channel) > 0)
        {
            WORD messageType = 0;
            WORD messageId = 0;
            DWORD messageData = 0;

            if (!BMC_GetNextMessage(serialNo_.c_str(), channel,
                &messageType, &messageId,
                &messageData))
            {
                break;
            }

            if (messageType == kMessageTypeGenericMotor)
            {
                if (messageId == kMessageIdMoved)
                    return DEVICE_OK;

                if (messageId == kMessageIdStopped)
                    return ERR_BBD302_MOVE_FAILED;
            }
        }

        SleepMs(kMoveWaitIntervalMs);
        elapsedMs += kMoveWaitIntervalMs;
    }

    return ERR_BBD302_MOVE_TIMEOUT;
}

int BBD302Stage::WaitForHome(unsigned long timeoutMs)
{
    const unsigned long sleepIntervalMs = 100;
    unsigned long elapsedMs = 0;

    while (elapsedMs < timeoutMs)
    {
        if (!IsConnected())
            return DEVICE_NOT_CONNECTED;

        const bool xDone = !BMC_CanHome(serialNo_.c_str(), kXChannel) ||
            AxisIsHomed(kXChannel);
        const bool yDone = !BMC_CanHome(serialNo_.c_str(), kYChannel) ||
            AxisIsHomed(kYChannel);

        if (xDone && yDone)
            return DEVICE_OK;

        SleepMs(sleepIntervalMs);
        elapsedMs += sleepIntervalMs;
    }

    Stop();
    return ERR_BBD302_HOME_FAILED;
}

int BBD302Stage::WaitForAxisHome(short channel,
    unsigned long timeoutMs)
{
    const unsigned long sleepIntervalMs = 100;
    unsigned long elapsedMs = 0;

    while (elapsedMs < timeoutMs)
    {
        if (!IsConnected())
            return DEVICE_NOT_CONNECTED;

        if (AxisIsHomed(channel))
            return DEVICE_OK;

        SleepMs(sleepIntervalMs);
        elapsedMs += sleepIntervalMs;
    }

    BMC_StopImmediate(serialNo_.c_str(), channel);
    return ERR_BBD302_HOME_FAILED;
}

bool BBD302Stage::AxisIsHomed(short channel) const
{
    if (!opened_)
        return false;

    const DWORD status = BMC_GetStatusBits(serialNo_.c_str(), channel);
    return (status & kStatusHomed) != 0;
}

bool BBD302Stage::AxisIsBusy(short channel) const
{
    if (!opened_)
        return false;

    const DWORD status = BMC_GetStatusBits(serialNo_.c_str(), channel);
    const DWORD busyMask = kStatusMovingCW | kStatusMovingCCW |
        kStatusJoggingCW | kStatusJoggingCCW |
        kStatusHoming;
    return (status & busyMask) != 0;
}

bool BBD302Stage::IsConnected() const
{
    return opened_ && BMC_CheckConnection(serialNo_.c_str());
}

long BBD302Stage::LogicalToPhysicalX(long logicalX) const
{
    if (!flipX_)
        return logicalX;

    // Generalized form of MaxX - X.  For the normal BBD302 zero-based travel
    // range xMinSteps_ == 0, this is exactly MaxX - X.
    const long long mirrored = static_cast<long long>(xMinSteps_) +
        static_cast<long long>(xMaxSteps_) -
        static_cast<long long>(logicalX);
    return InLongRange(mirrored) ? static_cast<long>(mirrored) : logicalX;
}

long BBD302Stage::LogicalToPhysicalY(long logicalY) const
{
    if (!flipY_)
        return logicalY;

    const long long mirrored = static_cast<long long>(yMinSteps_) +
        static_cast<long long>(yMaxSteps_) -
        static_cast<long long>(logicalY);
    return InLongRange(mirrored) ? static_cast<long>(mirrored) : logicalY;
}

long BBD302Stage::PhysicalToLogicalX(long physicalX) const
{
    // Mirroring is its own inverse.
    return LogicalToPhysicalX(physicalX);
}

long BBD302Stage::PhysicalToLogicalY(long physicalY) const
{
    return LogicalToPhysicalY(physicalY);
}

void BBD302Stage::LogKinesisError(const char* operation, short code) const
{
    std::ostringstream message;
    message << "BBD302: " << operation << " failed with Kinesis error "
        << code << ".";
    LogMessage(message.str(), false);
}

int BBD302Stage::GetPropertyReadOnly(const char* name, bool& readOnly) const
{
    if (std::strcmp(name, MM::g_Keyword_Transpose_MirrorX) == 0 ||
        std::strcmp(name, MM::g_Keyword_Transpose_MirrorY) == 0)
    {
        readOnly = true;
        return DEVICE_OK;
    }

    return CXYStageBase<BBD302Stage>::GetPropertyReadOnly(name, readOnly);
}
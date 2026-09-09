///////////////////////////////////////////////////////////////////////////////
// FILE:          BPC301.h
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
// DEPENDENCIES
//
// Download Kinesis Software from Thorlabs: 
// https://www.thorlabs.com/software-pages/motion_control
// 
// C++ SDK Located in: 
//   C:\Program Files\Thorlabs\Kinesis
//
// Expected file locations:
//   Include:  3rdpartypublic\Thorlabs\BPC301\include
//   Lib:      3rdpartypublic\Thorlabs\BPC301\lib
//   Bin:      3rdpartypublic\Thorlabs\BPC301\bin
//
// Required Kinesis files:
//   - Thorlabs.MotionControl.Benchtop.Piezo.h
//   - Thorlabs.MotionControl.Benchtop.Piezo.lib
//
// Runtime DLLs:
//   - Thorlabs.MotionControl.DeviceManager.dll
//   - Thorlabs.MotionControl.Benchtop.Piezo.dll
///////////////////////////////////////////////////////////////////////////////

#ifndef _BPC301_H_
#define _BPC301_H_

#include <MMDevice.h>
#include <DeviceBase.h>

#include <string>

extern const char* g_BPC301StageDeviceName;

// Adapter-specific error codes.
#define ERR_BPC301_SDK_CALL_FAILED        11101
#define ERR_BPC301_NO_CONTROLLER          11102
#define ERR_BPC301_OPEN_FAILED            11103
#define ERR_BPC301_INVALID_CHANNEL        11104
#define ERR_BPC301_POLLING_FAILED         11105
#define ERR_BPC301_ENABLE_FAILED          11106
#define ERR_BPC301_CONTROL_MODE_FAILED    11107
#define ERR_BPC301_INVALID_MAX_TRAVEL     11108
#define ERR_BPC301_POSITION_OUT_OF_RANGE  11109
#define ERR_BPC301_NOT_INITIALIZED        11110
#define ERR_BPC301_ZERO_TIMEOUT           11111

class BPC301Stage : public CStageBase<BPC301Stage>
{
public:
    BPC301Stage();
    ~BPC301Stage();

    // MM::Device API
    int Initialize();
    int Shutdown();
    void GetName(char* name) const;
    bool Busy();

    // MM::Stage API
    int SetPositionUm(double positionUm);
    int GetPositionUm(double& positionUm);
    int SetPositionSteps(long steps);
    int GetPositionSteps(long& steps);
    int SetOrigin();
    int SetAdapterOriginUm(double newPositionUm);
    int GetLimits(double& lowerUm, double& upperUm);
    int Home();
    int Stop();

    int IsStageSequenceable(bool& isSequenceable) const
    {
        isSequenceable = false;
        return DEVICE_OK;
    }

    bool IsContinuousFocusDrive() const
    {
        return false;
    }

    // Property actions
    int OnPosition(MM::PropertyBase* property, MM::ActionType action);
    int OnStatusBits(MM::PropertyBase* property, MM::ActionType action);

private:
    int OpenController();
    int TryOpenSerial(const std::string& serialNumber);

    int ReadPhysicalPosition(double& positionUm);
    int ReadStatusBits(unsigned long& statusBits);
    int WaitForZeroComplete(long timeoutMs);
    int CheckSdkResult(short sdkResult, const char* operation);

    bool PositionWithinHardwareLimits(double physicalPositionUm) const;
    double DirectionSign() const;
    void UpdatePositionPropertyLimits();

    bool initialized_;
    bool polling_;
    bool pendingZero_;
    bool moveCommandIssued_;

    short channel_;
    int pollingIntervalMs_;
    long busyDelayMs_;

    bool invertDirection_;
    bool zeroOnInitialize_;

    std::string requestedSerialNumber_;
    std::string connectedSerialNumber_;

    double maximumTravelUm_;
    double originPhysicalUm_;
    double stepSizeUm_;

    unsigned long lastStatusBits_;
    short lastSdkError_;

    MM::MMTime lastMoveTime_;
};

#endif //_BPC301_H_

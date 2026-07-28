///////////////////////////////////////////////////////////////////////////////
// FILE:          MCM301.h
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

#ifndef _MCM301_H_
#define _MCM301_H_

#include <MMDevice.h>
#include <DeviceBase.h>

#include <string>

extern const char* g_MCM301StageDeviceName;

// Adapter-specific error codes.
#define ERR_MCM301_SDK_CALL_FAILED       11001
#define ERR_MCM301_NO_CONTROLLER         11002
#define ERR_MCM301_OPEN_FAILED           11003
#define ERR_MCM301_INVALID_SLOT          11004
#define ERR_MCM301_SLOT_NOT_READY        11005
#define ERR_MCM301_MOTOR_NOT_CONNECTED   11006
#define ERR_MCM301_INVALID_LIMITS        11007
#define ERR_MCM301_POSITION_OUT_OF_RANGE 11008
#define ERR_MCM301_ENCODER_OUT_OF_RANGE  11009
#define ERR_MCM301_NOT_INITIALIZED       11010
#define ERR_MCM301_HOME_TIMEOUT          11011

class MCM301Stage : public CStageBase<MCM301Stage>
{
public:
    MCM301Stage();
    ~MCM301Stage();

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
    int OnCurrentEncoder(MM::PropertyBase* property, MM::ActionType action);

private:
    int OpenController();
    int TryOpenIdentifier(const std::string& identifier);
    int ReadMotionStatus(int& encoder, unsigned int& statusBits);
    int ReadPhysicalPosition(int& encoder, double& positionNm);
    int CaptureCurrentAsOrigin();
    int FinalizeHomeOriginIfComplete();
    int WaitUntilStopped(long timeoutMs);
    int CheckSdkResult(int sdkResult, const char* operation);
    int ComputeStepSize();

    bool StatusIndicatesMotion(unsigned int statusBits) const;
    bool PositionWithinConfiguredLimits(double positionUm) const;
    double DirectionSign() const;

    bool initialized_;
    int handle_;
    char slot_;
    int openTimeoutMs_;
    long homeTimeoutMs_;

    std::string requestedIdentifier_;
    std::string connectedIdentifier_;
    std::string homeDirection_;

    bool invertDirection_;
    bool homeOnInitialize_;
    bool useCurrentPositionAsOrigin_;
    bool pendingHomeOrigin_;

    double minimumPositionUm_;
    double maximumPositionUm_;
    double originNm_;
    int originEncoder_;
    double stepSizeUm_;

    unsigned int lastStatusBits_;
    int lastSdkError_;
};

#endif //_MCM301_H_
///////////////////////////////////////////////////////////////////////////////
// FILE:          BBD302.h
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
// DEPENDENCIES
//
// Download Kinesis Software from Thorlabs: 
// https://www.thorlabs.com/software-pages/motion_control
// 
// C++ SDK Located in: 
//   C:\Program Files\Thorlabs\Kinesis
//
// Expected file locations:
//   Include:  3rdpartypublic\Thorlabs\BBD302\include
//   Lib:      3rdpartypublic\Thorlabs\BBD302\lib
//   Bin:      3rdpartypublic\Thorlabs\BBD302\bin
//
// Required Kinesis files:
//   - Thorlabs.MotionControl.Benchtop.BrushlessMotor.h
//   - Thorlabs.MotionControl.Benchtop.BrushlessMotor.lib
//
// Runtime DLLs:
//   - Thorlabs.MotionControl.DeviceManager.dll
//   - Thorlabs.MotionControl.Benchtop.BrushlessMotor.dll
///////////////////////////////////////////////////////////////////////////////

#ifndef _BBD302_H_
#define _BBD302_H_

#include <MMDevice.h>
#include <DeviceBase.h>

#include <string>
#include <mutex>

extern const char* g_BBD302StageDeviceName;

// Adapter-specific error codes
#define ERR_BBD302_DEVICE_NOT_FOUND      10001
#define ERR_BBD302_OPEN_FAILED           10002
#define ERR_BBD302_CHANNEL_ENABLE_FAILED 10003
#define ERR_BBD302_POLLING_FAILED        10004
#define ERR_BBD302_HOME_FAILED           10005
#define ERR_BBD302_MOVE_FAILED           10006
#define ERR_BBD302_POSITION_READ_FAILED  10007
#define ERR_BBD302_STOP_FAILED           10008
#define ERR_BBD302_NOT_HOMED             10009
#define ERR_BBD302_INVALID_SERIAL        10010
#define ERR_BBD302_CONFIGURATION_FAILED   10011
#define ERR_BBD302_MOVE_TIMEOUT 10012

class BBD302Stage : public CXYStageBase<BBD302Stage>
{
public:
	// Concstructor/Destructor
    BBD302Stage();
    ~BBD302Stage();

    // MM::Device API
    int Initialize();
    int Shutdown();
    void GetName(char* name) const;
    bool Busy();

    // Required MM::XYStage API.  CXYStageBase supplies the um/relative-motion
    // wrappers, so the hardware transform is implemented once at the step layer.
    int SetPositionSteps(long x, long y);
    int GetPositionSteps(long& x, long& y);
    int Home();
    int Stop();
    int SetOrigin();
    int GetLimitsUm(double& xMin, double& xMax,
        double& yMin, double& yMax);
    int GetStepLimits(long& xMin, long& xMax,
        long& yMin, long& yMax);
    double GetStepSizeXUm();
    double GetStepSizeYUm();
    int IsXYStageSequenceable(bool& isSequenceable) const;

    // Properties
    int OnSerialNumber(MM::PropertyBase* pProp, MM::ActionType eAct);
    int OnFlipX(MM::PropertyBase* pProp, MM::ActionType eAct);
    int OnFlipY(MM::PropertyBase* pProp, MM::ActionType eAct);

private:
    int ConfigureAxis(short channel,
        long& minSteps, long& maxSteps,
        double& minUm, double& maxUm,
        double& stepSizeUm);
    int WaitForHome(unsigned long timeoutMs);
    int WaitForAxisHome(short channel, unsigned long timeoutMs);
    int WaitForMoveComplete(short channel, unsigned long timeoutMs);
    bool AxisIsHomed(short channel) const;
    bool AxisIsBusy(short channel) const;
    bool IsConnected() const;

    long LogicalToPhysicalX(long logicalX) const;
    long LogicalToPhysicalY(long logicalY) const;
    long PhysicalToLogicalX(long physicalX) const;
    long PhysicalToLogicalY(long physicalY) const;

    void LogKinesisError(const char* operation, short code) const;

    int GetPropertyReadOnly(const char* name, bool& readOnly) const override;

    bool initialized_;
    bool opened_;
    bool pollingX_;
    bool pollingY_;
    bool flipX_;
    bool flipY_;

    std::string serialNo_;

    long xMinSteps_;
    long xMaxSteps_;
    long yMinSteps_;
    long yMaxSteps_;

    double xMinUm_;
    double xMaxUm_;
    double yMinUm_;
    double yMaxUm_;
    double stepSizeXUm_;
    double stepSizeYUm_;

    std::mutex moveMutex_;
};

#endif //_BBD302_H_

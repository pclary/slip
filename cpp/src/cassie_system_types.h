#ifndef CASSIE_CODEGEN_FUN_TYPES_H
#define CASSIE_CODEGEN_FUN_TYPES_H
#include <stdbool.h>
#ifndef typedef_battery_medulla_inputs_t
#define typedef_battery_medulla_inputs_t

typedef struct {
    double counter;
    double temperature[4];
    double voltage[12];
    double current;
} battery_medulla_inputs_t;

#endif

#ifndef typedef_battery_medulla_t
#define typedef_battery_medulla_t

typedef struct {
    battery_medulla_inputs_t inputs;
} battery_medulla_t;

#endif

#ifndef typedef_drive_inputs_t
#define typedef_drive_inputs_t

typedef struct {
    double statusWord;
    double temperature;
    double position;
    double torque;
} drive_inputs_t;

#endif

#ifndef typedef_drive_outputs_t
#define typedef_drive_outputs_t

typedef struct {
    double controlWord;
    double torque;
} drive_outputs_t;

#endif

#ifndef typedef_drive_t
#define typedef_drive_t

typedef struct {
    drive_inputs_t inputs;
    drive_outputs_t outputs;
} drive_t;

#endif

#ifndef typedef_leg_medulla_inputs_t
#define typedef_leg_medulla_inputs_t

typedef struct {
    double counter;
    bool reedSwitch;
    double ankleEncoder;
    double kneeEncoder;
    double footEncoder;
} leg_medulla_inputs_t;

#endif

#ifndef typedef_leg_medulla_t
#define typedef_leg_medulla_t

typedef struct {
    leg_medulla_inputs_t inputs;
} leg_medulla_t;

#endif

#ifndef typedef_pelvis_medulla_inputs_t
#define typedef_pelvis_medulla_inputs_t

typedef struct {
    double counter;
    bool bleederTriggered;
    bool leftReedSwitch;
    bool rightReedSwitch;
    unsigned short vnVPEStatus;
    double vnAccel[3];
    double vnGyro[3];
    double vnMag[3];
    double vnOrientation[4];
    double vnPressure;
    double vnTemperature;
    bool radioSignalGood;
    bool radioFrameLost;
    bool radioFailSafeActive;
    double radioChannel[16];
} pelvis_medulla_inputs_t;

#endif

#ifndef typedef_pelvis_medulla_outputs_t
#define typedef_pelvis_medulla_outputs_t

typedef struct {
    double radioChannel[14];
    bool sto;
} pelvis_medulla_outputs_t;

#endif

#ifndef typedef_pelvis_medulla_t
#define typedef_pelvis_medulla_t

typedef struct {
    pelvis_medulla_inputs_t inputs;
    pelvis_medulla_outputs_t outputs;
} pelvis_medulla_t;

#endif

#ifndef typedef_ethercat_data_t
#define typedef_ethercat_data_t

typedef struct {
    double status[6];
    pelvis_medulla_t pelvisMedulla;
    battery_medulla_t batteryMedulla;
    leg_medulla_t leftLegMedulla;
    leg_medulla_t rightLegMedulla;
    drive_t leftAbductionDrive;
    drive_t leftYawDrive;
    drive_t leftHipDrive;
    drive_t leftKneeDrive;
    drive_t leftFootDrive;
    drive_t rightAbductionDrive;
    drive_t rightYawDrive;
    drive_t rightHipDrive;
    drive_t rightKneeDrive;
    drive_t rightFootDrive;
} ethercat_data_t;

#endif

typedef struct cassie_system_t cassie_system_t;

#endif

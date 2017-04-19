#include "cassie_system.h"
#include <math.h>
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>

#ifndef typedef_CassieState
#define typedef_CassieState

typedef short CassieState;

#endif

#ifndef CassieState_constants
#define CassieState_constants

#define b_NULL                         ((CassieState)0)
#define ETHERCAT_INIT                  ((CassieState)1)
#define ETHERCAT_PREOP                 ((CassieState)2)
#define ETHERCAT_SAFEOP                ((CassieState)3)
#define ETHERCAT_OP                    ((CassieState)4)
#define JOINTS_CALIBRATED              ((CassieState)5)
#define TORQUE_LIMIT_REACHED           ((CassieState)6)
#define JOINT_LIMIT_REACHED            ((CassieState)7)
#define LOW_BATTERY_CHARGE             ((CassieState)200)
#define HIGH_DRIVE_TEMP                ((CassieState)201)
#define HIGH_MOTOR_TEMP                ((CassieState)202)
#define HIGH_BATTERY_TEMP              ((CassieState)203)
#define ETHERCAT_DC_ERROR              ((CassieState)400)
#define ETHERCAT_ERROR                 ((CassieState)410)
#define CRITICAL_BATTERY_CHARGE        ((CassieState)600)
#define CRITICAL_DRIVE_TEMP            ((CassieState)601)
#define CRITICAL_MOTOR_TEMP            ((CassieState)602)
#define CRITICAL_BATTERY_TEMP          ((CassieState)603)
#define ENCODER_FAILURE                ((CassieState)604)
#define SPRING_FAILURE                 ((CassieState)605)
#endif

#ifndef enum_EtherCATMasterState
#define enum_EtherCATMasterState

enum EtherCATMasterState
{
    INIT = 1,
    PREOP,
    SAFEOP = 4,
    OP = 8
};

#endif

#ifndef typedef_EtherCATMasterState
#define typedef_EtherCATMasterState

typedef enum EtherCATMasterState EtherCATMasterState;

#endif

#ifndef typedef_CassieEtherCAT
#define typedef_CassieEtherCAT

typedef struct {
    ethercat_data_t data;
} CassieEtherCAT;

#endif

#ifndef typedef_struct_T
#define typedef_struct_T

typedef struct {
    double radio[9];
    double torque[10];
} struct_T;

#endif

#ifndef typedef_CassieInputs
#define typedef_CassieInputs

typedef struct {
    struct_T data;
} CassieInputs;

#endif

#ifndef typedef_b_struct_T
#define typedef_b_struct_T

typedef struct {
    double vectorNavOrientation[4];
    double vectorNavAngularVelocity[3];
    double vectorNavLinearAcceleration[3];
    double vectorNavMagnetometer[3];
    double vectorNavPressure;
    double vectorNavTemperature;
    double motorPosition[10];
    double motorVelocity[10];
    double jointPosition[6];
    double jointVelocity[6];
    double radio[16];
    double stateOfCharge;
    double status;
} b_struct_T;

#endif

#ifndef typedef_CassieOutputs
#define typedef_CassieOutputs

typedef struct {
    b_struct_T data;
} CassieOutputs;

#endif

#ifndef typedef_ProcessInputsSimulation
#define typedef_ProcessInputsSimulation

typedef struct {
    int isInitialized;
    CassieEtherCAT etherCAT;
    CassieInputs inputs;
    CassieOutputs outputs;
    double softStart;
    double softStartDuration;
} ProcessInputsSimulation;

#endif

#ifndef typedef_TrackingLoopFilter
#define typedef_TrackingLoopFilter

typedef struct {
    double sampleTime;
    double proportionalGain;
    double integralGain;
    double signalValue[10];
    double signalRateOfChange[10];
    double integrator[10];
} TrackingLoopFilter;

#endif

#ifndef typedef_TrackingLoopFilter_1
#define typedef_TrackingLoopFilter_1

typedef struct {
    double sampleTime;
    double proportionalGain;
    double integralGain;
    double signalValue[6];
    double signalRateOfChange[6];
    double integrator[6];
} TrackingLoopFilter_1;

#endif

#ifndef typedef_ProcessOutputsSimulation
#define typedef_ProcessOutputsSimulation

typedef struct {
    int isInitialized;
    CassieEtherCAT etherCAT;
    CassieOutputs outputs;
    bool isCalibrated[5];
    double motorOffset[10];
    double jointOffset[6];
    TrackingLoopFilter motorFilter;
    TrackingLoopFilter_1 jointFilter;
    double counter;
} ProcessOutputsSimulation;

#endif

#ifndef typedef_InverseKinematicsFunction
#define typedef_InverseKinematicsFunction

typedef struct {
    double desiredState[12];
} InverseKinematicsFunction;

#endif

#ifndef typedef_NonlinearLeastSquaresProblem
#define typedef_NonlinearLeastSquaresProblem

typedef struct {
    InverseKinematicsFunction *residualFunction;
    double lowerBound[10];
    double upperBound[10];
} NonlinearLeastSquaresProblem;

#endif

#ifndef typedef_OptimizationStatus
#define typedef_OptimizationStatus

typedef signed char OptimizationStatus;

#endif

#ifndef OptimizationStatus_constants
#define OptimizationStatus_constants

#define INFEASIBLE                     ((OptimizationStatus)-3)
#define SINGULAR                       ((OptimizationStatus)-2)
#define ITERATION_LIMIT                ((OptimizationStatus)-1)
#define ITERATE                        ((OptimizationStatus)0)
#define F_TOLERANCE                    ((OptimizationStatus)1)
#define X_TOLERANCE                    ((OptimizationStatus)2)
#define G_TOLERANCE                    ((OptimizationStatus)3)
#define PHI_TOLERANCE                  ((OptimizationStatus)4)
#endif

#ifndef typedef_NonlinearLeastSquaresSolver
#define typedef_NonlinearLeastSquaresSolver

typedef struct {
    NonlinearLeastSquaresProblem *problem;
    double x[10];
    double levenbergMarquardtParameter;
    double iterationLimit;
    double xTolerance;
    double gTolerance;
    OptimizationStatus status;
    double iteration;
} NonlinearLeastSquaresSolver;

#endif

#ifndef typedef_PDController
#define typedef_PDController

typedef struct {
    double activeMotorIndex[10];
    double proportionalGain[10];
    double derivativeGain[10];
    double desiredMotorPosition[10];
    double desiredMotorVelocity[10];
} PDController;

#endif

#ifndef typedef_TestController
#define typedef_TestController

typedef struct {
    int isInitialized;
    CassieInputs inputs;
    CassieOutputs outputs;
    NonlinearLeastSquaresSolver ikSolver;
    NonlinearLeastSquaresProblem ikProblem;
    InverseKinematicsFunction ikFunction;
    PDController pdController;
    double controllerMode;
    double softStart;
} TestController;

#endif

#ifndef typedef_cassie_system_t
#define typedef_cassie_system_t

struct cassie_system_t {
    ProcessInputsSimulation *process_inputs;
    ProcessOutputsSimulation *process_outputs;
    TestController *controller;
};

#endif

static CassieEtherCAT *CassieEtherCAT_CassieEtherCAT(CassieEtherCAT *obj);
static CassieOutputs *CassieOutputs_CassieOutputs(CassieOutputs *obj);
static void ProcessInputs_stepImpl(ProcessInputsSimulation *obj, const
    ethercat_data_t *b_etherCAT, const double inputs_radio[9], const double
    inputs_torque[10], const b_struct_T *b_outputs);
static void ProcessOutputs_setStateOfCharge(ProcessOutputsSimulation *obj);
static void SystemCore_setup(ProcessOutputsSimulation *obj);
static void SystemCore_step(ProcessOutputsSimulation *obj, const ethercat_data_t
    *varargin_1, double c_varargout_1_vectorNavOrientat[4], double
    c_varargout_1_vectorNavAngularV[3], double c_varargout_1_vectorNavLinearAc[3],
    double c_varargout_1_vectorNavMagnetom[3], double
    *varargout_1_vectorNavPressure, double *c_varargout_1_vectorNavTemperat,
    double varargout_1_motorPosition[10], double varargout_1_motorVelocity[10],
    double varargout_1_jointPosition[6], double varargout_1_jointVelocity[6],
    double varargout_1_radio[16], double *varargout_1_stateOfCharge, double
    *varargout_1_status);
static void b_CassieEtherCAT_CassieEtherCAT(CassieEtherCAT **obj);
static void b_CassieOutputs_CassieOutputs(CassieOutputs **obj);
static void b_SystemCore_setup(TestController *obj);
static int b_bsearch(const double b_x[64], double xi);
static void b_sort(double b_x[5]);
static void c_InverseKinematicsFunction_eva(const InverseKinematicsFunction *obj,
    const double q[10], double f[12], double J[120]);
static void c_NonlinearLeastSquaresSolver_p(const InverseKinematicsFunction *r,
    double b_x[10], const double lb[10], const double ub[10], double r_x[12],
    double J_x[120]);
static void c_NonlinearLeastSquaresSolver_s(NonlinearLeastSquaresSolver *obj);


static void merge(int idx[5], double b_x[5], int offset, int np, int nq, int
                  iwork[5], double xwork[5]);
static void mldivide(const double A[100], double B[10]);
static double norm(const double b_x[10]);
static double rt_roundd(double u);
static void sort(double b_x[5]);
static CassieEtherCAT *CassieEtherCAT_CassieEtherCAT(CassieEtherCAT *obj)
{
    CassieEtherCAT *b_obj;
    b_obj = obj;
    b_CassieEtherCAT_CassieEtherCAT(&b_obj);
    return b_obj;
}

static CassieOutputs *CassieOutputs_CassieOutputs(CassieOutputs *obj)
{
    CassieOutputs *b_obj;
    b_obj = obj;
    b_CassieOutputs_CassieOutputs(&b_obj);
    return b_obj;
}

static void ProcessInputs_stepImpl(ProcessInputsSimulation *obj, const
    ethercat_data_t *b_etherCAT, const double inputs_radio[9], const double
    inputs_torque[10], const b_struct_T *b_outputs)
{
    CassieEtherCAT *b_obj;
    CassieInputs *c_obj;
    int i;
    CassieOutputs *d_obj;
    ProcessInputsSimulation *e_obj;
    double b_status[6];
    double messages[5];
    double b_messages;
    double b_stateOfCharge;
    double value[9];
    double q[10];
    static const signed char iv1[8] = { 0, 1, 2, 3, 5, 6, 7, 8 };

    double Dq[10];
    bool y;
    double u[10];
    int k;
    int j;
    bool exitg1;
    double r[20];
    static const double b[20] = { 0.1617993877991494, 0.29269908169872416,
        0.77266462599716479, 2.7623399732707004, 2.3434609527920611,
        0.29269908169872416, 0.29269908169872416, 0.77266462599716479,
        2.7623399732707004, 2.3434609527920611, 0.29269908169872416,
        0.29269908169872416, 1.2962634015954635, -0.74577182323790192,
        -0.62359877559829879, 0.1617993877991494, 0.29269908169872416,
        1.2962634015954635, -0.74577182323790192, -0.62359877559829879 };

    static const signed char a[200] = { -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1 };

    double b_x;
    double c_x[30];
    static const double dv6[10] = { -140.62750000000003, -140.62750000000003,
        -216.0, -216.0, -112.51799999999999, -140.62750000000003,
        -140.62750000000003, -216.0, -216.0, -112.51799999999999 };

    static const double dv7[10] = { 140.62750000000003, 140.62750000000003,
        216.0, 216.0, 112.51799999999999, 140.62750000000003, 140.62750000000003,
        216.0, 216.0, 112.51799999999999 };

    static const short Kp[10] = { 700, 700, 700, 700, 100, 700, 700, 700, 700,
        100 };

    static const signed char Kd[10] = { 10, 10, 20, 20, 20, 10, 10, 20, 20, 20 };

    double vwork[3];
    b_obj = &obj->etherCAT;
    b_obj->data = *b_etherCAT;
    c_obj = &obj->inputs;
    for (i = 0; i < 9; i++) {
        c_obj->data.radio[i] = inputs_radio[i];
    }

    for (i = 0; i < 10; i++) {
        c_obj->data.torque[i] = inputs_torque[i];
    }

    d_obj = &obj->outputs;
    d_obj->data = *b_outputs;
    b_obj = &obj->etherCAT;
    b_obj->data.pelvisMedulla.outputs.sto = false;
    b_obj = &obj->etherCAT;
    b_obj->data.leftAbductionDrive.outputs.torque = 0.0;
    b_obj->data.leftYawDrive.outputs.torque = 0.0;
    b_obj->data.leftHipDrive.outputs.torque = 0.0;
    b_obj->data.leftKneeDrive.outputs.torque = 0.0;
    b_obj->data.leftFootDrive.outputs.torque = 0.0;
    b_obj->data.rightAbductionDrive.outputs.torque = 0.0;
    b_obj->data.rightYawDrive.outputs.torque = 0.0;
    b_obj->data.rightHipDrive.outputs.torque = 0.0;
    b_obj->data.rightKneeDrive.outputs.torque = 0.0;
    b_obj->data.rightFootDrive.outputs.torque = 0.0;
    b_obj = &obj->etherCAT;
    for (i = 0; i < 14; i++) {
        b_obj->data.pelvisMedulla.outputs.radioChannel[i] = 0.0;
    }

    e_obj = obj;
    for (i = 0; i < 6; i++) {
        b_status[i] = e_obj->etherCAT.data.status[i];
    }

    if (b_status[1] == (double)INIT) {
        for (i = 0; i < 4; i++) {
            b_messages = e_obj->
                etherCAT.data.pelvisMedulla.outputs.radioChannel[i + 1];
            messages[i] = b_messages;
        }

        messages[4] = ETHERCAT_INIT;
        sort(messages);
        for (i = 0; i < 4; i++) {
            e_obj->etherCAT.data.pelvisMedulla.outputs.radioChannel[i + 1] =
                messages[i];
        }
    } else if (b_status[1] == (double)PREOP) {
        for (i = 0; i < 4; i++) {
            b_messages = e_obj->
                etherCAT.data.pelvisMedulla.outputs.radioChannel[i + 1];
            messages[i] = b_messages;
        }

        messages[4] = ETHERCAT_PREOP;
        sort(messages);
        for (i = 0; i < 4; i++) {
            e_obj->etherCAT.data.pelvisMedulla.outputs.radioChannel[i + 1] =
                messages[i];
        }
    } else if (b_status[1] == (double)SAFEOP) {
        for (i = 0; i < 4; i++) {
            b_messages = e_obj->
                etherCAT.data.pelvisMedulla.outputs.radioChannel[i + 1];
            messages[i] = b_messages;
        }

        messages[4] = ETHERCAT_SAFEOP;
        sort(messages);
        for (i = 0; i < 4; i++) {
            e_obj->etherCAT.data.pelvisMedulla.outputs.radioChannel[i + 1] =
                messages[i];
        }
    } else {
        if (b_status[1] == (double)OP) {
            for (i = 0; i < 4; i++) {
                b_messages =
                    e_obj->etherCAT.data.pelvisMedulla.outputs.radioChannel[i +
                    1];
                messages[i] = b_messages;
            }

            messages[4] = ETHERCAT_OP;
            sort(messages);
            for (i = 0; i < 4; i++) {
                e_obj->etherCAT.data.pelvisMedulla.outputs.radioChannel[i + 1] =
                    messages[i];
            }
        }
    }

    if (b_status[0] > 0.0) {
        for (i = 0; i < 4; i++) {
            b_messages = e_obj->
                etherCAT.data.pelvisMedulla.outputs.radioChannel[i + 1];
            messages[i] = b_messages;
        }

        messages[4] = ETHERCAT_ERROR;
        sort(messages);
        for (i = 0; i < 4; i++) {
            e_obj->etherCAT.data.pelvisMedulla.outputs.radioChannel[i + 1] =
                messages[i];
        }
    }

    if (b_status[2] > 0.0) {
        for (i = 0; i < 4; i++) {
            b_messages = e_obj->
                etherCAT.data.pelvisMedulla.outputs.radioChannel[i + 1];
            messages[i] = b_messages;
        }

        messages[4] = ETHERCAT_DC_ERROR;
        sort(messages);
        for (i = 0; i < 4; i++) {
            e_obj->etherCAT.data.pelvisMedulla.outputs.radioChannel[i + 1] =
                messages[i];
        }
    }

    d_obj = &obj->outputs;
    b_messages = d_obj->data.status;
    if (b_messages >= 1.0) {
        e_obj = obj;
        b_stateOfCharge = e_obj->outputs.data.stateOfCharge;
        e_obj->etherCAT.data.pelvisMedulla.outputs.radioChannel[0] = 100.0 *
            b_stateOfCharge;
        b_messages = e_obj->outputs.data.radio[8];
        switch ((int)b_messages) {
          case 0:
            b_messages = e_obj->outputs.data.radio[12];
            switch ((int)b_messages) {
              case 0:
                q[0] = e_obj->etherCAT.data.leftAbductionDrive.inputs.position;
                q[1] = e_obj->etherCAT.data.leftYawDrive.inputs.position;
                q[2] = e_obj->etherCAT.data.leftHipDrive.inputs.position;
                q[3] = e_obj->etherCAT.data.leftKneeDrive.inputs.position;
                q[4] = e_obj->etherCAT.data.leftFootDrive.inputs.position;
                q[5] = e_obj->etherCAT.data.rightAbductionDrive.inputs.position;
                q[6] = e_obj->etherCAT.data.rightYawDrive.inputs.position;
                q[7] = e_obj->etherCAT.data.rightHipDrive.inputs.position;
                q[8] = e_obj->etherCAT.data.rightKneeDrive.inputs.position;
                q[9] = e_obj->etherCAT.data.rightFootDrive.inputs.position;
                for (i = 0; i < 8; i++) {
                    e_obj->etherCAT.data.pelvisMedulla.outputs.radioChannel[5 +
                        i] = 1000.0 * q[iv1[i]];
                }

                e_obj->etherCAT.data.pelvisMedulla.outputs.radioChannel[13] =
                    0.0;
                break;

              case 1:
                for (i = 0; i < 10; i++) {
                    q[i] = e_obj->outputs.data.motorPosition[i];
                }

                for (i = 0; i < 8; i++) {
                    e_obj->etherCAT.data.pelvisMedulla.outputs.radioChannel[5 +
                        i] = 1000.0 * q[iv1[i]];
                }

                e_obj->etherCAT.data.pelvisMedulla.outputs.radioChannel[13] =
                    0.0;
                break;
            }
            break;

          case -1:
            b_messages = e_obj->outputs.data.radio[12];
            switch ((int)b_messages) {
              case 0:
                b_status[0] =
                    e_obj->etherCAT.data.leftLegMedulla.inputs.kneeEncoder;
                b_status[1] =
                    e_obj->etherCAT.data.leftLegMedulla.inputs.ankleEncoder;
                b_status[2] =
                    e_obj->etherCAT.data.leftLegMedulla.inputs.footEncoder;
                b_status[3] =
                    e_obj->etherCAT.data.rightLegMedulla.inputs.kneeEncoder;
                b_status[4] =
                    e_obj->etherCAT.data.rightLegMedulla.inputs.ankleEncoder;
                b_status[5] =
                    e_obj->etherCAT.data.rightLegMedulla.inputs.footEncoder;
                for (i = 0; i < 6; i++) {
                    e_obj->etherCAT.data.pelvisMedulla.outputs.radioChannel[i +
                        5] = 1000.0 * b_status[i];
                }

                for (i = 0; i < 3; i++) {
                    e_obj->etherCAT.data.pelvisMedulla.outputs.radioChannel[i +
                        11] = 0.0;
                }
                break;

              case 1:
                for (i = 0; i < 6; i++) {
                    b_status[i] = e_obj->outputs.data.jointPosition[i];
                }

                for (i = 0; i < 6; i++) {
                    e_obj->etherCAT.data.pelvisMedulla.outputs.radioChannel[i +
                        5] = 1000.0 * b_status[i];
                }

                for (i = 0; i < 3; i++) {
                    e_obj->etherCAT.data.pelvisMedulla.outputs.radioChannel[i +
                        11] = 0.0;
                }
                break;
            }
            break;

          default:
            for (i = 0; i < 9; i++) {
                value[i] = e_obj->inputs.data.radio[i];
            }

            for (i = 0; i < 9; i++) {
                e_obj->etherCAT.data.pelvisMedulla.outputs.radioChannel[i + 5] =
                    value[i];
            }
            break;
        }

        e_obj = obj;
        b_stateOfCharge = e_obj->outputs.data.stateOfCharge;
        if (b_stateOfCharge < 0.05) {
            for (i = 0; i < 4; i++) {
                b_messages =
                    e_obj->etherCAT.data.pelvisMedulla.outputs.radioChannel[i +
                    1];
                messages[i] = b_messages;
            }

            messages[4] = CRITICAL_BATTERY_CHARGE;
            sort(messages);
            for (i = 0; i < 4; i++) {
                e_obj->etherCAT.data.pelvisMedulla.outputs.radioChannel[i + 1] =
                    messages[i];
            }
        } else {
            if (b_stateOfCharge < 0.15) {
                for (i = 0; i < 4; i++) {
                    b_messages =
                        e_obj->
                        etherCAT.data.pelvisMedulla.outputs.radioChannel[i + 1];
                    messages[i] = b_messages;
                }

                messages[4] = LOW_BATTERY_CHARGE;
                sort(messages);
                for (i = 0; i < 4; i++) {
                    e_obj->etherCAT.data.pelvisMedulla.outputs.radioChannel[i +
                        1] = messages[i];
                }
            }
        }

        b_messages = e_obj->softStart;
        e_obj->softStart = fmin(b_messages, fmax(fmin(1.0 + (b_stateOfCharge -
            0.05) / 0.05, 1.0), 0.0));
        e_obj = obj;
        q[0] = e_obj->etherCAT.data.leftAbductionDrive.inputs.temperature;
        q[1] = e_obj->etherCAT.data.leftYawDrive.inputs.temperature;
        q[2] = e_obj->etherCAT.data.leftHipDrive.inputs.temperature;
        q[3] = e_obj->etherCAT.data.leftKneeDrive.inputs.temperature;
        q[4] = e_obj->etherCAT.data.leftFootDrive.inputs.temperature;
        q[5] = e_obj->etherCAT.data.rightAbductionDrive.inputs.temperature;
        q[6] = e_obj->etherCAT.data.rightYawDrive.inputs.temperature;
        q[7] = e_obj->etherCAT.data.rightHipDrive.inputs.temperature;
        q[8] = e_obj->etherCAT.data.rightKneeDrive.inputs.temperature;
        q[9] = e_obj->etherCAT.data.rightFootDrive.inputs.temperature;
        b_messages = q[0];
        for (i = 0; i < 9; i++) {
            if (q[i + 1] > b_messages) {
                b_messages = q[i + 1];
            }
        }

        if (b_messages > 80.0) {
            for (i = 0; i < 4; i++) {
                b_messages =
                    e_obj->etherCAT.data.pelvisMedulla.outputs.radioChannel[i +
                    1];
                messages[i] = b_messages;
            }

            messages[4] = CRITICAL_DRIVE_TEMP;
            sort(messages);
            for (i = 0; i < 4; i++) {
                e_obj->etherCAT.data.pelvisMedulla.outputs.radioChannel[i + 1] =
                    messages[i];
            }
        } else {
            if (b_messages > 70.0) {
                for (i = 0; i < 4; i++) {
                    b_messages =
                        e_obj->
                        etherCAT.data.pelvisMedulla.outputs.radioChannel[i + 1];
                    messages[i] = b_messages;
                }

                messages[4] = HIGH_DRIVE_TEMP;
                sort(messages);
                for (i = 0; i < 4; i++) {
                    e_obj->etherCAT.data.pelvisMedulla.outputs.radioChannel[i +
                        1] = messages[i];
                }
            }
        }

        d_obj = &obj->outputs;
        b_messages = d_obj->data.status;
        if (b_messages >= 2.0) {
            e_obj = obj;
            for (i = 0; i < 4; i++) {
                b_messages =
                    e_obj->etherCAT.data.pelvisMedulla.outputs.radioChannel[i +
                    1];
                messages[i] = b_messages;
            }

            messages[4] = JOINTS_CALIBRATED;
            sort(messages);
            for (i = 0; i < 4; i++) {
                e_obj->etherCAT.data.pelvisMedulla.outputs.radioChannel[i + 1] =
                    messages[i];
            }

            b_obj = &obj->etherCAT;
            b_obj->data.pelvisMedulla.outputs.sto = true;
            e_obj = obj;
            for (i = 0; i < 10; i++) {
                q[i] = e_obj->outputs.data.motorPosition[i];
                Dq[i] = e_obj->outputs.data.motorVelocity[i];
                u[i] = e_obj->inputs.data.torque[i];
            }

            for (i = 0; i < 20; i++) {
                b_messages = 0.0;
                j = 0;
                for (k = 0; k < 10; k++) {
                    b_messages += (double)a[j + i] * q[k];
                    j += 20;
                }

                r[i] = b_messages - b[i];
            }

            y = false;
            k = 0;
            exitg1 = false;
            while ((!exitg1) && (k < 20)) {
                if (r[k] > 0.0) {
                    y = true;
                    exitg1 = true;
                } else {
                    k++;
                }
            }

            if (y) {
                for (i = 0; i < 4; i++) {
                    b_messages =
                        e_obj->
                        etherCAT.data.pelvisMedulla.outputs.radioChannel[i + 1];
                    messages[i] = b_messages;
                }

                messages[4] = JOINT_LIMIT_REACHED;
                sort(messages);
                for (i = 0; i < 4; i++) {
                    e_obj->etherCAT.data.pelvisMedulla.outputs.radioChannel[i +
                        1] = messages[i];
                }
            }

            for (i = 0; i < 20; i++) {
                b_stateOfCharge = fmax(fmin(r[i] / 0.05, 1.0), 0.0);
                for (k = 0; k < 10; k++) {
                    b_x = u[k];
                    if (u[k] < 0.0) {
                        b_x = -1.0;
                    } else {
                        if (u[k] > 0.0) {
                            b_x = 1.0;
                        }
                    }

                    j = a[i + 20 * k];
                    if (a[i + 20 * k] < 0) {
                        j = -1;
                    } else {
                        if (a[i + 20 * k] > 0) {
                            j = 1;
                        }
                    }

                    u[k] *= 1.0 - (double)(b_x == j) * b_stateOfCharge;
                }
            }

            for (i = 0; i < 20; i++) {
                b_messages = fmin(0.05 - r[i], 0.0);
                b_stateOfCharge = fmax(fmin(r[i] / 0.05, 1.0), 0.0);
                for (k = 0; k < 10; k++) {
                    b_x = Dq[k];
                    if (Dq[k] < 0.0) {
                        b_x = -1.0;
                    } else {
                        if (Dq[k] > 0.0) {
                            b_x = 1.0;
                        }
                    }

                    j = a[i + 20 * k];
                    if (a[i + 20 * k] < 0) {
                        j = -1;
                    } else {
                        if (a[i + 20 * k] > 0) {
                            j = 1;
                        }
                    }

                    u[k] = (u[k] + (double)(Kp[k] * a[i + 20 * k]) * b_messages)
                        - (double)Kd[k] * Dq[k] * (double)(b_x == j) *
                        b_stateOfCharge;
                }
            }

            for (i = 0; i < 10; i++) {
                e_obj->inputs.data.torque[i] = u[i];
            }

            e_obj = obj;
            b_messages = e_obj->outputs.data.radio[8];
            if (b_messages != 1.0) {
                e_obj->softStart = 0.0;
            } else {
                b_messages = e_obj->softStartDuration;
                b_messages = e_obj->softStart + 0.0005 / b_messages;
                e_obj->softStart = fmax(fmin(b_messages, 1.0), 0.0);
            }

            for (i = 0; i < 10; i++) {
                b_messages = e_obj->inputs.data.torque[i];
                c_x[i] = dv6[i];
                c_x[i + 10] = b_messages;
                c_x[i + 20] = dv7[i];
            }

            b_messages = e_obj->softStart;
            for (j = 0; j < 10; j++) {
                for (k = 0; k < 3; k++) {
                    vwork[k] = c_x[j + k * 10];
                }

                if (vwork[0] < vwork[1]) {
                    if (vwork[1] < vwork[2]) {
                        i = 1;
                    } else if (vwork[0] < vwork[2]) {
                        i = 2;
                    } else {
                        i = 0;
                    }
                } else if (vwork[0] < vwork[2]) {
                    i = 0;
                } else if (vwork[1] < vwork[2]) {
                    i = 2;
                } else {
                    i = 1;
                }

                q[j] = b_messages * vwork[i];
            }

            e_obj->etherCAT.data.leftAbductionDrive.outputs.torque = q[0];
            e_obj->etherCAT.data.leftYawDrive.outputs.torque = q[1];
            e_obj->etherCAT.data.leftHipDrive.outputs.torque = q[2];
            e_obj->etherCAT.data.leftKneeDrive.outputs.torque = q[3];
            e_obj->etherCAT.data.leftFootDrive.outputs.torque = q[4];
            e_obj->etherCAT.data.rightAbductionDrive.outputs.torque = q[5];
            e_obj->etherCAT.data.rightYawDrive.outputs.torque = q[6];
            e_obj->etherCAT.data.rightHipDrive.outputs.torque = q[7];
            e_obj->etherCAT.data.rightKneeDrive.outputs.torque = q[8];
            e_obj->etherCAT.data.rightFootDrive.outputs.torque = q[9];
        }
    }
}

static void ProcessOutputs_setStateOfCharge(ProcessOutputsSimulation *obj)
{
    CassieEtherCAT *b_obj;
    int i;
    double mtmp;
    double U[12];
    double loadCurrent;
    CassieOutputs *c_obj;
    double b_stateOfCharge;
    double b_x[64];
    static const double varargin_1[64] = { 1.0, 0.984126984126984,
        0.968253968253968, 0.952380952380952, 0.936507936507937,
        0.920634920634921, 0.904761904761905, 0.888888888888889,
        0.873015873015873, 0.857142857142857, 0.841269841269841,
        0.825396825396825, 0.80952380952381, 0.793650793650794,
        0.777777777777778, 0.761904761904762, 0.746031746031746,
        0.73015873015873, 0.714285714285714, 0.698412698412698,
        0.682539682539683, 0.666666666666667, 0.650793650793651,
        0.634920634920635, 0.619047619047619, 0.603174603174603,
        0.587301587301587, 0.571428571428571, 0.555555555555556,
        0.53968253968254, 0.523809523809524, 0.507936507936508,
        0.492063492063492, 0.476190476190476, 0.46031746031746,
        0.444444444444444, 0.428571428571429, 0.412698412698413,
        0.396825396825397, 0.380952380952381, 0.365079365079365,
        0.349206349206349, 0.333333333333333, 0.317460317460317,
        0.301587301587302, 0.285714285714286, 0.26984126984127,
        0.253968253968254, 0.238095238095238, 0.222222222222222,
        0.206349206349206, 0.19047619047619, 0.174603174603175,
        0.158730158730159, 0.142857142857143, 0.126984126984127,
        0.111111111111111, 0.0952380952380952, 0.0793650793650794,
        0.0634920634920635, 0.0476190476190477, 0.0317460317460317,
        0.0158730158730159, 0.0 };

    double y[64];
    static const double varargin_2[64] = { 0.074657094174427, 0.0776288410537264,
        0.0775720452843985, 0.0779682335151647, 0.077915483200014,
        0.0783840409595333, 0.078100172313545, 0.0780324439020493,
        0.0776678681832252, 0.0777753750690216, 0.0753807431197581,
        0.074619318529329, 0.0740613565890792, 0.0727677044171342,
        0.0728257449122133, 0.0720551539281923, 0.0715571346289302,
        0.071837005038039, 0.0710802895132899, 0.071044049381788,
        0.0714685931753636, 0.070084389367934, 0.0700754533168423,
        0.069197058240914, 0.0697957993545229, 0.068888923911841,
        0.0684868114054925, 0.0669063834735484, 0.0675080554969434,
        0.0667027908381394, 0.0657369975810054, 0.0653284843414671,
        0.0646556124319945, 0.0651407737230404, 0.0645168409403845,
        0.0646372052858918, 0.064166611686287, 0.0650625762424001,
        0.0639169446605328, 0.0643772017581636, 0.0638966160369324,
        0.0634020965423058, 0.0636040364153852, 0.0632496745549067,
        0.0628277639761283, 0.0622817886070695, 0.0608264800973328,
        0.0615800527469564, 0.0614955876137078, 0.0598522020803924,
        0.0597276010525681, 0.0588427636201772, 0.0580197193630799,
        0.0560212744154546, 0.0543148640115735, 0.0521198044271855,
        0.0495810464638169, 0.0474406390693683, 0.0455218677805405,
        0.0440204017505073, 0.0412839801207766, 0.0357584837398558,
        0.0243947844887275, 0.0 };

    double tmp;
    double resistance;
    static const double b_varargin_1[64] = { 4.31308322669466, 4.27008483794419,
        4.25202435383552, 4.23935508975162, 4.22955509821477, 4.22359549673375,
        4.21450612857692, 4.20485997480167, 4.19423978145327, 4.18517906467606,
        4.1664440700573, 4.15048176300327, 4.13492396329799, 4.11310697767266,
        4.10024692994908, 4.07999714648193, 4.06282871049839, 4.05013462022116,
        4.03404317808651, 4.02140187519097, 4.00947969023684, 3.99085894985761,
        3.97798389496605, 3.96332344872145, 3.95008496399475, 3.93313848177829,
        3.91483521054906, 3.89556773076512, 3.88122715505987, 3.86229006989607,
        3.84300238227196, 3.82351366503398, 3.80698147690756, 3.79331819564584,
        3.77601094761374, 3.76475017340326, 3.74873366084273, 3.74275040648155,
        3.72397760716564, 3.71476913259574, 3.70166189588077, 3.6890729062753,
        3.67820932813621, 3.66365360668221, 3.65123551465579, 3.63651785361647,
        3.61540891503103, 3.60780299542904, 3.59269458283966, 3.57025574207086,
        3.55322429603934, 3.5308513030385, 3.5060724673725, 3.46884409351905,
        3.4314983695002, 3.38605513453656, 3.33265258711805, 3.27433797018951,
        3.21504240074429, 3.15468403609017, 3.08426578976195, 2.9826099705228,
        2.8209236897413, 2.50796606947643 };

    b_obj = &obj->etherCAT;
    for (i = 0; i < 12; i++) {
        U[i] = b_obj->data.batteryMedulla.inputs.voltage[i];
    }

    mtmp = U[0];
    for (i = 0; i < 11; i++) {
        if (U[i + 1] < mtmp) {
            mtmp = U[i + 1];
        }
    }

    b_obj = &obj->etherCAT;
    loadCurrent = b_obj->data.batteryMedulla.inputs.current;
    c_obj = &obj->outputs;
    b_stateOfCharge = c_obj->data.stateOfCharge;
    memcpy(&b_x[0], &varargin_1[0], sizeof(double) << 6);
    for (i = 0; i < 32; i++) {
        tmp = b_x[i];
        b_x[i] = b_x[63 - i];
        b_x[63 - i] = tmp;
    }

    memcpy(&y[0], &varargin_2[0], sizeof(double) << 6);
    for (i = 0; i < 32; i++) {
        tmp = y[i];
        y[i] = y[63 - i];
        y[63 - i] = tmp;
    }

    resistance = 0.0;
    if ((!(b_stateOfCharge > b_x[63])) && (!(b_stateOfCharge < b_x[0]))) {
        i = b_bsearch(b_x, b_stateOfCharge) - 1;
        resistance = (b_stateOfCharge - b_x[i]) / (b_x[i + 1] - b_x[i]);
        if (resistance == 0.0) {
            resistance = y[i];
        } else if (resistance == 1.0) {
            resistance = y[i + 1];
        } else if (y[i] == y[i + 1]) {
            resistance = y[i];
        } else {
            resistance = (1.0 - resistance) * y[i] + resistance * y[i + 1];
        }
    }

    resistance = resistance * (loadCurrent / 7.0) + mtmp;
    memcpy(&b_x[0], &b_varargin_1[0], sizeof(double) << 6);
    for (i = 0; i < 32; i++) {
        tmp = b_x[i];
        b_x[i] = b_x[63 - i];
        b_x[63 - i] = tmp;
    }

    memcpy(&y[0], &varargin_1[0], sizeof(double) << 6);
    for (i = 0; i < 32; i++) {
        tmp = y[i];
        y[i] = y[63 - i];
        y[63 - i] = tmp;
    }

    if (resistance > b_x[63]) {
        b_stateOfCharge = y[63] + (resistance - b_x[63]) / (b_x[63] - b_x[62]) *
            (y[63] - y[62]);
    } else if (resistance < b_x[0]) {
        b_stateOfCharge = y[0] + (resistance - b_x[0]) / (b_x[1] - b_x[0]) * (y
            [1] - y[0]);
    } else {
        i = b_bsearch(b_x, resistance) - 1;
        resistance = (resistance - b_x[i]) / (b_x[i + 1] - b_x[i]);
        if (resistance == 0.0) {
            b_stateOfCharge = y[i];
        } else if (resistance == 1.0) {
            b_stateOfCharge = y[i + 1];
        } else if (y[i] == y[i + 1]) {
            b_stateOfCharge = y[i];
        } else {
            b_stateOfCharge = (1.0 - resistance) * y[i] + resistance * y[i + 1];
        }
    }

    c_obj = &obj->outputs;
    c_obj->data.stateOfCharge = fmax(fmin(b_stateOfCharge, 1.0), 0.0);
}

static void SystemCore_setup(ProcessOutputsSimulation *obj)
{
    ProcessOutputsSimulation *b_obj;
    int i;
    obj->isInitialized = 1;
    b_obj = obj;
    CassieEtherCAT_CassieEtherCAT(&b_obj->etherCAT);
    CassieOutputs_CassieOutputs(&b_obj->outputs);
    for (i = 0; i < 5; i++) {
        b_obj->isCalibrated[i] = false;
    }

    b_obj->motorFilter.sampleTime = 0.001;
    b_obj->motorFilter.proportionalGain = 200.0;
    b_obj->motorFilter.integralGain = 1000.0;
    for (i = 0; i < 10; i++) {
        b_obj->motorFilter.signalValue[i] = 0.0;
    }

    for (i = 0; i < 10; i++) {
        b_obj->motorFilter.signalRateOfChange[i] = 0.0;
    }

    for (i = 0; i < 10; i++) {
        b_obj->motorFilter.integrator[i] = 0.0;
    }

    b_obj->motorFilter.sampleTime = 0.0005;
    b_obj->motorFilter.proportionalGain = 200.0;
    b_obj->motorFilter.integralGain = 1000.0;
    b_obj->jointFilter.sampleTime = 0.001;
    b_obj->jointFilter.proportionalGain = 200.0;
    b_obj->jointFilter.integralGain = 1000.0;
    for (i = 0; i < 6; i++) {
        b_obj->jointFilter.signalValue[i] = 0.0;
    }

    for (i = 0; i < 6; i++) {
        b_obj->jointFilter.signalRateOfChange[i] = 0.0;
    }

    for (i = 0; i < 6; i++) {
        b_obj->jointFilter.integrator[i] = 0.0;
    }

    b_obj->jointFilter.sampleTime = 0.0005;
    b_obj->jointFilter.proportionalGain = 200.0;
    b_obj->jointFilter.integralGain = 1000.0;
    for (i = 0; i < 10; i++) {
        b_obj->motorOffset[i] = 0.0;
    }

    for (i = 0; i < 6; i++) {
        b_obj->jointOffset[i] = 0.0;
    }

    for (i = 0; i < 5; i++) {
        b_obj->isCalibrated[i] = true;
    }

    b_obj->counter = 0.0;
}

static void SystemCore_step(ProcessOutputsSimulation *obj, const ethercat_data_t
    *varargin_1, double c_varargout_1_vectorNavOrientat[4], double
    c_varargout_1_vectorNavAngularV[3], double c_varargout_1_vectorNavLinearAc[3],
    double c_varargout_1_vectorNavMagnetom[3], double
    *varargout_1_vectorNavPressure, double *c_varargout_1_vectorNavTemperat,
    double varargout_1_motorPosition[10], double varargout_1_motorVelocity[10],
    double varargout_1_jointPosition[6], double varargout_1_jointVelocity[6],
    double varargout_1_radio[16], double *varargout_1_stateOfCharge, double
    *varargout_1_status)
{
    ProcessOutputsSimulation *b_obj;
    int i;
    double b_status[6];
    double b_motorPosition[10];
    double p;
    double value[16];
    double minval[16];
    double q[4];
    double w[3];
    double signalError[10];
    bool reedSwitch_idx_0;
    bool b_x[5];
    bool exitg1;
    bool reedSwitch_idx_1;
    bool reedSwitch_idx_2;
    double a;
    bool reedSwitch_idx_3;
    double b_signalError[6];
    static const double dv0[3] = { -0.3490658503988659, 0.0, 1.4835298641951802
    };

    static const double dv1[3] = { 0.24037, 0.2254, 0.04084 };

    static const double dv2[3] = { 0.0, 0.0, 0.26179938779914941 };

    static const double y[3] = { 0.25132741228718347, 0.25132741228718347,
        0.39269908169872414 };

    static const double dv3[3] = { 0.3490658503988659, 0.0, 1.4835298641951802 };

    static const double dv4[3] = { -0.24299, -0.21623, -0.1419 };

    static const double homePosition[6] = { 0.0, 1.5707963267948966,
        -2.4434609527920612, 0.0, 1.5707963267948966, -2.4434609527920612 };

    static const double measuredCalibrationPosition[6] = { -2.707, 1.3636, 2.926,
        3.106, 1.2631, -2.2757 };

    static const double calibrationPosition[6] = { 0.0, 2.9967303256742635,
        -2.4434609527920612, 0.0, 2.9967303256742635, -2.4434609527920612 };

    int i0;
    int i1;
    if (obj->isInitialized != 1) {
        b_obj = obj;
        SystemCore_setup(b_obj);
    }

    b_obj = obj;
    b_obj->etherCAT.data = *varargin_1;
    for (i = 0; i < 6; i++) {
        b_status[i] = b_obj->etherCAT.data.status[i];
    }

    if (b_status[1] == (double)OP) {
        b_obj->outputs.data.status = 1.0;
        for (i = 0; i < 16; i++) {
            p = b_obj->etherCAT.data.pelvisMedulla.inputs.radioChannel[i];
            p /= 819.0;
            minval[i] = fmin(p, 1.0);
        }

        for (i = 0; i < 16; i++) {
            value[i] = fmax(minval[i], -1.0);
        }

        for (i = 0; i < 16; i++) {
            b_obj->outputs.data.radio[i] = value[i];
        }

        for (i = 0; i < 4; i++) {
            q[i] = b_obj->etherCAT.data.pelvisMedulla.inputs.vnOrientation[i];
        }

        for (i = 0; i < 4; i++) {
            b_obj->outputs.data.vectorNavOrientation[i] = q[i];
        }

        for (i = 0; i < 3; i++) {
            w[i] = b_obj->etherCAT.data.pelvisMedulla.inputs.vnGyro[i];
        }

        for (i = 0; i < 3; i++) {
            b_obj->outputs.data.vectorNavAngularVelocity[i] = w[i];
        }

        for (i = 0; i < 3; i++) {
            w[i] = b_obj->etherCAT.data.pelvisMedulla.inputs.vnAccel[i];
        }

        for (i = 0; i < 3; i++) {
            b_obj->outputs.data.vectorNavLinearAcceleration[i] = w[i];
        }

        for (i = 0; i < 3; i++) {
            w[i] = b_obj->etherCAT.data.pelvisMedulla.inputs.vnMag[i];
        }

        for (i = 0; i < 3; i++) {
            b_obj->outputs.data.vectorNavMagnetometer[i] = w[i];
        }

        p = b_obj->etherCAT.data.pelvisMedulla.inputs.vnPressure;
        b_obj->outputs.data.vectorNavPressure = p;
        p = b_obj->etherCAT.data.pelvisMedulla.inputs.vnTemperature;
        b_obj->outputs.data.vectorNavTemperature = p;
        ProcessOutputs_setStateOfCharge(b_obj);
        for (i = 0; i < 5; i++) {
            b_x[i] = b_obj->isCalibrated[i];
        }

        reedSwitch_idx_0 = true;
        i = 0;
        exitg1 = false;
        while ((!exitg1) && (i < 5)) {
            if (!b_x[i]) {
                reedSwitch_idx_0 = false;
                exitg1 = true;
            } else {
                i++;
            }
        }

        if (reedSwitch_idx_0) {
            b_obj->outputs.data.status = 2.0;
            signalError[0] =
                b_obj->etherCAT.data.leftAbductionDrive.inputs.position;
            signalError[1] = b_obj->etherCAT.data.leftYawDrive.inputs.position;
            signalError[2] = b_obj->etherCAT.data.leftHipDrive.inputs.position;
            signalError[3] = b_obj->etherCAT.data.leftKneeDrive.inputs.position;
            signalError[4] = b_obj->etherCAT.data.leftFootDrive.inputs.position;
            signalError[5] =
                b_obj->etherCAT.data.rightAbductionDrive.inputs.position;
            signalError[6] = b_obj->etherCAT.data.rightYawDrive.inputs.position;
            signalError[7] = b_obj->etherCAT.data.rightHipDrive.inputs.position;
            signalError[8] = b_obj->etherCAT.data.rightKneeDrive.inputs.position;
            signalError[9] = b_obj->etherCAT.data.rightFootDrive.inputs.position;
            for (i = 0; i < 10; i++) {
                p = signalError[i] + b_obj->motorOffset[i];
                b_motorPosition[i] = b_obj->motorFilter.signalRateOfChange[i];
                signalError[i] = p;
            }

            p = b_obj->motorFilter.sampleTime;
            for (i = 0; i < 10; i++) {
                b_obj->motorFilter.signalValue[i] += b_motorPosition[i] * p;
            }

            for (i = 0; i < 10; i++) {
                p = signalError[i] - b_obj->motorFilter.signalValue[i];
                signalError[i] = p;
            }

            a = b_obj->motorFilter.integralGain;
            p = b_obj->motorFilter.sampleTime;
            for (i = 0; i < 10; i++) {
                b_obj->motorFilter.integrator[i] += a * signalError[i] * p;
            }

            a = b_obj->motorFilter.proportionalGain;
            for (i = 0; i < 10; i++) {
                b_motorPosition[i] = b_obj->motorFilter.integrator[i] + a *
                    signalError[i];
            }

            for (i = 0; i < 10; i++) {
                b_obj->motorFilter.signalRateOfChange[i] = b_motorPosition[i];
            }

            for (i = 0; i < 10; i++) {
                b_motorPosition[i] = b_obj->motorFilter.signalValue[i];
            }

            for (i = 0; i < 10; i++) {
                b_obj->outputs.data.motorPosition[i] = b_motorPosition[i];
            }

            for (i = 0; i < 10; i++) {
                b_motorPosition[i] = b_obj->motorFilter.signalRateOfChange[i];
            }

            for (i = 0; i < 10; i++) {
                b_obj->outputs.data.motorVelocity[i] = b_motorPosition[i];
            }

            b_signalError[0] =
                b_obj->etherCAT.data.leftLegMedulla.inputs.kneeEncoder;
            b_signalError[1] =
                b_obj->etherCAT.data.leftLegMedulla.inputs.ankleEncoder;
            b_signalError[2] =
                b_obj->etherCAT.data.leftLegMedulla.inputs.footEncoder;
            b_signalError[3] =
                b_obj->etherCAT.data.rightLegMedulla.inputs.kneeEncoder;
            b_signalError[4] =
                b_obj->etherCAT.data.rightLegMedulla.inputs.ankleEncoder;
            b_signalError[5] =
                b_obj->etherCAT.data.rightLegMedulla.inputs.footEncoder;
            for (i = 0; i < 6; i++) {
                p = b_signalError[i] + b_obj->jointOffset[i];
                b_status[i] = b_obj->jointFilter.signalRateOfChange[i];
                b_signalError[i] = p;
            }

            p = b_obj->jointFilter.sampleTime;
            for (i = 0; i < 6; i++) {
                b_obj->jointFilter.signalValue[i] += b_status[i] * p;
            }

            for (i = 0; i < 6; i++) {
                p = b_signalError[i] - b_obj->jointFilter.signalValue[i];
                b_signalError[i] = p;
            }

            a = b_obj->jointFilter.integralGain;
            p = b_obj->jointFilter.sampleTime;
            for (i = 0; i < 6; i++) {
                b_obj->jointFilter.integrator[i] += a * b_signalError[i] * p;
            }

            a = b_obj->jointFilter.proportionalGain;
            for (i = 0; i < 6; i++) {
                b_status[i] = b_obj->jointFilter.integrator[i] + a *
                    b_signalError[i];
            }

            for (i = 0; i < 6; i++) {
                b_obj->jointFilter.signalRateOfChange[i] = b_status[i];
            }

            for (i = 0; i < 6; i++) {
                b_status[i] = b_obj->jointFilter.signalValue[i];
            }

            for (i = 0; i < 6; i++) {
                b_obj->outputs.data.jointPosition[i] = b_status[i];
            }

            for (i = 0; i < 6; i++) {
                b_status[i] = b_obj->jointFilter.signalRateOfChange[i];
            }

            for (i = 0; i < 6; i++) {
                b_obj->outputs.data.jointVelocity[i] = b_status[i];
            }
        } else {
            reedSwitch_idx_0 =
                b_obj->etherCAT.data.pelvisMedulla.inputs.leftReedSwitch;
            reedSwitch_idx_1 =
                b_obj->etherCAT.data.leftLegMedulla.inputs.reedSwitch;
            reedSwitch_idx_2 =
                b_obj->etherCAT.data.pelvisMedulla.inputs.rightReedSwitch;
            reedSwitch_idx_3 =
                b_obj->etherCAT.data.rightLegMedulla.inputs.reedSwitch;
            b_motorPosition[0] =
                b_obj->etherCAT.data.leftAbductionDrive.inputs.position;
            b_motorPosition[1] =
                b_obj->etherCAT.data.leftYawDrive.inputs.position;
            b_motorPosition[2] =
                b_obj->etherCAT.data.leftHipDrive.inputs.position;
            b_motorPosition[3] =
                b_obj->etherCAT.data.leftKneeDrive.inputs.position;
            b_motorPosition[4] =
                b_obj->etherCAT.data.leftFootDrive.inputs.position;
            b_motorPosition[5] =
                b_obj->etherCAT.data.rightAbductionDrive.inputs.position;
            b_motorPosition[6] =
                b_obj->etherCAT.data.rightYawDrive.inputs.position;
            b_motorPosition[7] =
                b_obj->etherCAT.data.rightHipDrive.inputs.position;
            b_motorPosition[8] =
                b_obj->etherCAT.data.rightKneeDrive.inputs.position;
            b_motorPosition[9] =
                b_obj->etherCAT.data.rightFootDrive.inputs.position;
            b_status[0] = b_obj->etherCAT.data.leftLegMedulla.inputs.kneeEncoder;
            b_status[1] =
                b_obj->etherCAT.data.leftLegMedulla.inputs.ankleEncoder;
            b_status[2] = b_obj->etherCAT.data.leftLegMedulla.inputs.footEncoder;
            b_status[3] =
                b_obj->etherCAT.data.rightLegMedulla.inputs.kneeEncoder;
            b_status[4] =
                b_obj->etherCAT.data.rightLegMedulla.inputs.ankleEncoder;
            b_status[5] =
                b_obj->etherCAT.data.rightLegMedulla.inputs.footEncoder;
            if (reedSwitch_idx_0 && (!b_obj->isCalibrated[0])) {
                for (i = 0; i < 3; i++) {
                    w[i] = rt_roundd((dv0[i] - ((b_motorPosition[i] - dv1[i]) +
                                       dv2[i])) / y[i]);
                }

                for (i = 0; i < 3; i++) {
                    b_obj->motorOffset[i] = (w[i] * y[i] - dv1[i]) + dv2[i];
                }

                b_obj->isCalibrated[0] = true;
            }

            if (reedSwitch_idx_1 && (!b_obj->isCalibrated[1])) {
                b_obj->motorOffset[3] = (rt_roundd((-2.8405233576207713 -
                    ((b_motorPosition[3] - 0.06313) + -2.769837522915001)) /
                    0.39269908169872414) * 0.39269908169872414 - 0.06313) +
                    -2.769837522915001;
                b_obj->isCalibrated[1] = true;
            }

            if (reedSwitch_idx_2 && (!b_obj->isCalibrated[2])) {
                for (i = 0; i < 3; i++) {
                    w[i] = rt_roundd((dv3[i] - ((b_motorPosition[i + 5] - dv4[i])
                                       + dv2[i])) / y[i]);
                }

                for (i = 0; i < 3; i++) {
                    b_obj->motorOffset[i + 5] = (w[i] * y[i] - dv4[i]) + dv2[i];
                }

                b_obj->isCalibrated[2] = true;
            }

            if (reedSwitch_idx_3 && (!b_obj->isCalibrated[3])) {
                b_obj->motorOffset[8] = (rt_roundd((-2.8405233576207713 -
                    ((b_motorPosition[8] - 0.09433) + -2.769837522915001)) /
                    0.39269908169872414) * 0.39269908169872414 - 0.09433) +
                    -2.769837522915001;
                b_obj->isCalibrated[3] = true;
            }

            if (!b_obj->isCalibrated[4]) {
                for (i = 0; i < 6; i++) {
                    b_signalError[i] = rt_roundd((homePosition[i] - ((b_status[i]
                        - measuredCalibrationPosition[i]) +
                        calibrationPosition[i])) / 6.2831853071795862);
                }

                for (i = 0; i < 6; i++) {
                    b_obj->jointOffset[i] = (6.2831853071795862 *
                        b_signalError[i] - measuredCalibrationPosition[i]) +
                        calibrationPosition[i];
                }

                i = 0;
                i0 = 0;
                for (i1 = 0; i1 < 2; i1++) {
                    b_obj->motorOffset[i + 4] = (b_status[i0 + 2] +
                        b_obj->jointOffset[i0 + 2]) - b_motorPosition[i + 4];
                    i += 5;
                    i0 += 3;
                }

                b_obj->isCalibrated[4] = true;
            }

            b_motorPosition[0] =
                b_obj->etherCAT.data.leftAbductionDrive.inputs.position;
            b_motorPosition[1] =
                b_obj->etherCAT.data.leftYawDrive.inputs.position;
            b_motorPosition[2] =
                b_obj->etherCAT.data.leftHipDrive.inputs.position;
            b_motorPosition[3] =
                b_obj->etherCAT.data.leftKneeDrive.inputs.position;
            b_motorPosition[4] =
                b_obj->etherCAT.data.leftFootDrive.inputs.position;
            b_motorPosition[5] =
                b_obj->etherCAT.data.rightAbductionDrive.inputs.position;
            b_motorPosition[6] =
                b_obj->etherCAT.data.rightYawDrive.inputs.position;
            b_motorPosition[7] =
                b_obj->etherCAT.data.rightHipDrive.inputs.position;
            b_motorPosition[8] =
                b_obj->etherCAT.data.rightKneeDrive.inputs.position;
            b_motorPosition[9] =
                b_obj->etherCAT.data.rightFootDrive.inputs.position;
            for (i = 0; i < 10; i++) {
                p = b_motorPosition[i] + b_obj->motorOffset[i];
                b_motorPosition[i] = p;
            }

            for (i = 0; i < 10; i++) {
                b_obj->motorFilter.signalValue[i] = b_motorPosition[i];
            }

            b_status[0] = b_obj->etherCAT.data.leftLegMedulla.inputs.kneeEncoder;
            b_status[1] =
                b_obj->etherCAT.data.leftLegMedulla.inputs.ankleEncoder;
            b_status[2] = b_obj->etherCAT.data.leftLegMedulla.inputs.footEncoder;
            b_status[3] =
                b_obj->etherCAT.data.rightLegMedulla.inputs.kneeEncoder;
            b_status[4] =
                b_obj->etherCAT.data.rightLegMedulla.inputs.ankleEncoder;
            b_status[5] =
                b_obj->etherCAT.data.rightLegMedulla.inputs.footEncoder;
            for (i = 0; i < 6; i++) {
                p = b_status[i] + b_obj->jointOffset[i];
                b_status[i] = p;
            }

            for (i = 0; i < 6; i++) {
                b_obj->jointFilter.signalValue[i] = b_status[i];
            }
        }
    }

    if (b_obj->counter < 2.0) {
        b_motorPosition[0] =
            b_obj->etherCAT.data.leftAbductionDrive.inputs.position;
        b_motorPosition[1] = b_obj->etherCAT.data.leftYawDrive.inputs.position;
        b_motorPosition[2] = b_obj->etherCAT.data.leftHipDrive.inputs.position;
        b_motorPosition[3] = b_obj->etherCAT.data.leftKneeDrive.inputs.position;
        b_motorPosition[4] = b_obj->etherCAT.data.leftFootDrive.inputs.position;
        b_motorPosition[5] =
            b_obj->etherCAT.data.rightAbductionDrive.inputs.position;
        b_motorPosition[6] = b_obj->etherCAT.data.rightYawDrive.inputs.position;
        b_motorPosition[7] = b_obj->etherCAT.data.rightHipDrive.inputs.position;
        b_motorPosition[8] = b_obj->etherCAT.data.rightKneeDrive.inputs.position;
        b_motorPosition[9] = b_obj->etherCAT.data.rightFootDrive.inputs.position;
        for (i = 0; i < 10; i++) {
            b_obj->motorFilter.signalValue[i] = b_motorPosition[i];
        }

        b_status[0] = b_obj->etherCAT.data.leftLegMedulla.inputs.kneeEncoder;
        b_status[1] = b_obj->etherCAT.data.leftLegMedulla.inputs.ankleEncoder;
        b_status[2] = b_obj->etherCAT.data.leftLegMedulla.inputs.footEncoder;
        b_status[3] = b_obj->etherCAT.data.rightLegMedulla.inputs.kneeEncoder;
        b_status[4] = b_obj->etherCAT.data.rightLegMedulla.inputs.ankleEncoder;
        b_status[5] = b_obj->etherCAT.data.rightLegMedulla.inputs.footEncoder;
        for (i = 0; i < 6; i++) {
            b_obj->jointFilter.signalValue[i] = b_status[i];
        }

        signalError[0] = b_obj->etherCAT.data.leftAbductionDrive.inputs.position;
        signalError[1] = b_obj->etherCAT.data.leftYawDrive.inputs.position;
        signalError[2] = b_obj->etherCAT.data.leftHipDrive.inputs.position;
        signalError[3] = b_obj->etherCAT.data.leftKneeDrive.inputs.position;
        signalError[4] = b_obj->etherCAT.data.leftFootDrive.inputs.position;
        signalError[5] =
            b_obj->etherCAT.data.rightAbductionDrive.inputs.position;
        signalError[6] = b_obj->etherCAT.data.rightYawDrive.inputs.position;
        signalError[7] = b_obj->etherCAT.data.rightHipDrive.inputs.position;
        signalError[8] = b_obj->etherCAT.data.rightKneeDrive.inputs.position;
        signalError[9] = b_obj->etherCAT.data.rightFootDrive.inputs.position;
        for (i = 0; i < 10; i++) {
            b_motorPosition[i] = b_obj->motorFilter.signalRateOfChange[i];
        }

        p = b_obj->motorFilter.sampleTime;
        for (i = 0; i < 10; i++) {
            b_obj->motorFilter.signalValue[i] += b_motorPosition[i] * p;
        }

        for (i = 0; i < 10; i++) {
            p = signalError[i] - b_obj->motorFilter.signalValue[i];
            signalError[i] = p;
        }

        a = b_obj->motorFilter.integralGain;
        p = b_obj->motorFilter.sampleTime;
        for (i = 0; i < 10; i++) {
            b_obj->motorFilter.integrator[i] += a * signalError[i] * p;
        }

        a = b_obj->motorFilter.proportionalGain;
        for (i = 0; i < 10; i++) {
            b_motorPosition[i] = b_obj->motorFilter.integrator[i] + a *
                signalError[i];
        }

        for (i = 0; i < 10; i++) {
            b_obj->motorFilter.signalRateOfChange[i] = b_motorPosition[i];
        }

        b_signalError[0] =
            b_obj->etherCAT.data.leftLegMedulla.inputs.kneeEncoder;
        b_signalError[1] =
            b_obj->etherCAT.data.leftLegMedulla.inputs.ankleEncoder;
        b_signalError[2] =
            b_obj->etherCAT.data.leftLegMedulla.inputs.footEncoder;
        b_signalError[3] =
            b_obj->etherCAT.data.rightLegMedulla.inputs.kneeEncoder;
        b_signalError[4] =
            b_obj->etherCAT.data.rightLegMedulla.inputs.ankleEncoder;
        b_signalError[5] =
            b_obj->etherCAT.data.rightLegMedulla.inputs.footEncoder;
        for (i = 0; i < 6; i++) {
            b_status[i] = b_obj->jointFilter.signalRateOfChange[i];
        }

        p = b_obj->jointFilter.sampleTime;
        for (i = 0; i < 6; i++) {
            b_obj->jointFilter.signalValue[i] += b_status[i] * p;
        }

        for (i = 0; i < 6; i++) {
            p = b_signalError[i] - b_obj->jointFilter.signalValue[i];
            b_signalError[i] = p;
        }

        a = b_obj->jointFilter.integralGain;
        p = b_obj->jointFilter.sampleTime;
        for (i = 0; i < 6; i++) {
            b_obj->jointFilter.integrator[i] += a * b_signalError[i] * p;
        }

        a = b_obj->jointFilter.proportionalGain;
        for (i = 0; i < 6; i++) {
            b_status[i] = b_obj->jointFilter.integrator[i] + a * b_signalError[i];
        }

        for (i = 0; i < 6; i++) {
            b_obj->jointFilter.signalRateOfChange[i] = b_status[i];
        }

        for (i = 0; i < 6; i++) {
            b_status[i] = b_obj->jointFilter.signalValue[i];
        }

        for (i = 0; i < 6; i++) {
            b_obj->outputs.data.jointPosition[i] = b_status[i];
        }

        for (i = 0; i < 6; i++) {
            b_status[i] = b_obj->jointFilter.signalRateOfChange[i];
        }

        for (i = 0; i < 6; i++) {
            b_obj->outputs.data.jointVelocity[i] = b_status[i];
        }
    }

    b_obj->counter++;
    for (i = 0; i < 4; i++) {
        c_varargout_1_vectorNavOrientat[i] =
            b_obj->outputs.data.vectorNavOrientation[i];
    }

    for (i = 0; i < 3; i++) {
        c_varargout_1_vectorNavAngularV[i] =
            b_obj->outputs.data.vectorNavAngularVelocity[i];
        c_varargout_1_vectorNavLinearAc[i] =
            b_obj->outputs.data.vectorNavLinearAcceleration[i];
        c_varargout_1_vectorNavMagnetom[i] =
            b_obj->outputs.data.vectorNavMagnetometer[i];
    }

    *varargout_1_vectorNavPressure = b_obj->outputs.data.vectorNavPressure;
    *c_varargout_1_vectorNavTemperat = b_obj->outputs.data.vectorNavTemperature;
    for (i = 0; i < 10; i++) {
        varargout_1_motorPosition[i] = b_obj->outputs.data.motorPosition[i];
        varargout_1_motorVelocity[i] = b_obj->outputs.data.motorVelocity[i];
    }

    for (i = 0; i < 6; i++) {
        varargout_1_jointPosition[i] = b_obj->outputs.data.jointPosition[i];
        varargout_1_jointVelocity[i] = b_obj->outputs.data.jointVelocity[i];
    }

    for (i = 0; i < 16; i++) {
        varargout_1_radio[i] = b_obj->outputs.data.radio[i];
    }

    *varargout_1_stateOfCharge = b_obj->outputs.data.stateOfCharge;
    *varargout_1_status = b_obj->outputs.data.status;
}

static void b_CassieEtherCAT_CassieEtherCAT(CassieEtherCAT **obj)
{
    CassieEtherCAT *b_this;
    int i;
    signed char c_data_pelvisMedulla_inputs_vnO[4];
    static const signed char iv2[4] = { 1, 0, 0, 0 };

    b_this = *obj;
    *obj = b_this;
    for (i = 0; i < 4; i++) {
        c_data_pelvisMedulla_inputs_vnO[i] = iv2[i];
    }

    for (i = 0; i < 6; i++) {
        (*obj)->data.status[i] = 0.0;
    }

    (*obj)->data.pelvisMedulla.inputs.counter = 0.0;
    (*obj)->data.pelvisMedulla.inputs.bleederTriggered = false;
    (*obj)->data.pelvisMedulla.inputs.leftReedSwitch = false;
    (*obj)->data.pelvisMedulla.inputs.rightReedSwitch = false;
    (*obj)->data.pelvisMedulla.inputs.vnVPEStatus = 0;
    for (i = 0; i < 3; i++) {
        (*obj)->data.pelvisMedulla.inputs.vnAccel[i] = 0.0;
    }

    for (i = 0; i < 3; i++) {
        (*obj)->data.pelvisMedulla.inputs.vnGyro[i] = 0.0;
    }

    for (i = 0; i < 3; i++) {
        (*obj)->data.pelvisMedulla.inputs.vnMag[i] = 0.0;
    }

    for (i = 0; i < 4; i++) {
        (*obj)->data.pelvisMedulla.inputs.vnOrientation[i] =
            c_data_pelvisMedulla_inputs_vnO[i];
    }

    (*obj)->data.pelvisMedulla.inputs.vnPressure = 0.0;
    (*obj)->data.pelvisMedulla.inputs.vnTemperature = 0.0;
    (*obj)->data.pelvisMedulla.inputs.radioSignalGood = true;
    (*obj)->data.pelvisMedulla.inputs.radioFrameLost = false;
    (*obj)->data.pelvisMedulla.inputs.radioFailSafeActive = false;
    for (i = 0; i < 16; i++) {
        (*obj)->data.pelvisMedulla.inputs.radioChannel[i] = 0.0;
    }

    for (i = 0; i < 14; i++) {
        (*obj)->data.pelvisMedulla.outputs.radioChannel[i] = 0.0;
    }

    (*obj)->data.pelvisMedulla.outputs.sto = false;
    (*obj)->data.batteryMedulla.inputs.counter = 0.0;
    for (i = 0; i < 4; i++) {
        (*obj)->data.batteryMedulla.inputs.temperature[i] = 0.0;
    }

    for (i = 0; i < 12; i++) {
        (*obj)->data.batteryMedulla.inputs.voltage[i] = 0.0;
    }

    (*obj)->data.batteryMedulla.inputs.current = 0.0;
    (*obj)->data.leftLegMedulla.inputs.counter = 0.0;
    (*obj)->data.leftLegMedulla.inputs.reedSwitch = false;
    (*obj)->data.leftLegMedulla.inputs.ankleEncoder = 0.0;
    (*obj)->data.leftLegMedulla.inputs.kneeEncoder = 0.0;
    (*obj)->data.leftLegMedulla.inputs.footEncoder = 0.0;
    (*obj)->data.rightLegMedulla.inputs.counter = 0.0;
    (*obj)->data.rightLegMedulla.inputs.reedSwitch = false;
    (*obj)->data.rightLegMedulla.inputs.ankleEncoder = 0.0;
    (*obj)->data.rightLegMedulla.inputs.kneeEncoder = 0.0;
    (*obj)->data.rightLegMedulla.inputs.footEncoder = 0.0;
    (*obj)->data.leftAbductionDrive.inputs.statusWord = 0.0;
    (*obj)->data.leftAbductionDrive.inputs.temperature = 0.0;
    (*obj)->data.leftAbductionDrive.inputs.position = 0.0;
    (*obj)->data.leftAbductionDrive.inputs.torque = 0.0;
    (*obj)->data.leftAbductionDrive.outputs.controlWord = 0.0;
    (*obj)->data.leftAbductionDrive.outputs.torque = 0.0;
    (*obj)->data.leftYawDrive.inputs.statusWord = 0.0;
    (*obj)->data.leftYawDrive.inputs.temperature = 0.0;
    (*obj)->data.leftYawDrive.inputs.position = 0.0;
    (*obj)->data.leftYawDrive.inputs.torque = 0.0;
    (*obj)->data.leftYawDrive.outputs.controlWord = 0.0;
    (*obj)->data.leftYawDrive.outputs.torque = 0.0;
    (*obj)->data.leftHipDrive.inputs.statusWord = 0.0;
    (*obj)->data.leftHipDrive.inputs.temperature = 0.0;
    (*obj)->data.leftHipDrive.inputs.position = 0.0;
    (*obj)->data.leftHipDrive.inputs.torque = 0.0;
    (*obj)->data.leftHipDrive.outputs.controlWord = 0.0;
    (*obj)->data.leftHipDrive.outputs.torque = 0.0;
    (*obj)->data.leftKneeDrive.inputs.statusWord = 0.0;
    (*obj)->data.leftKneeDrive.inputs.temperature = 0.0;
    (*obj)->data.leftKneeDrive.inputs.position = 0.0;
    (*obj)->data.leftKneeDrive.inputs.torque = 0.0;
    (*obj)->data.leftKneeDrive.outputs.controlWord = 0.0;
    (*obj)->data.leftKneeDrive.outputs.torque = 0.0;
    (*obj)->data.leftFootDrive.inputs.statusWord = 0.0;
    (*obj)->data.leftFootDrive.inputs.temperature = 0.0;
    (*obj)->data.leftFootDrive.inputs.position = 0.0;
    (*obj)->data.leftFootDrive.inputs.torque = 0.0;
    (*obj)->data.leftFootDrive.outputs.controlWord = 0.0;
    (*obj)->data.leftFootDrive.outputs.torque = 0.0;
    (*obj)->data.rightAbductionDrive.inputs.statusWord = 0.0;
    (*obj)->data.rightAbductionDrive.inputs.temperature = 0.0;
    (*obj)->data.rightAbductionDrive.inputs.position = 0.0;
    (*obj)->data.rightAbductionDrive.inputs.torque = 0.0;
    (*obj)->data.rightAbductionDrive.outputs.controlWord = 0.0;
    (*obj)->data.rightAbductionDrive.outputs.torque = 0.0;
    (*obj)->data.rightYawDrive.inputs.statusWord = 0.0;
    (*obj)->data.rightYawDrive.inputs.temperature = 0.0;
    (*obj)->data.rightYawDrive.inputs.position = 0.0;
    (*obj)->data.rightYawDrive.inputs.torque = 0.0;
    (*obj)->data.rightYawDrive.outputs.controlWord = 0.0;
    (*obj)->data.rightYawDrive.outputs.torque = 0.0;
    (*obj)->data.rightHipDrive.inputs.statusWord = 0.0;
    (*obj)->data.rightHipDrive.inputs.temperature = 0.0;
    (*obj)->data.rightHipDrive.inputs.position = 0.0;
    (*obj)->data.rightHipDrive.inputs.torque = 0.0;
    (*obj)->data.rightHipDrive.outputs.controlWord = 0.0;
    (*obj)->data.rightHipDrive.outputs.torque = 0.0;
    (*obj)->data.rightKneeDrive.inputs.statusWord = 0.0;
    (*obj)->data.rightKneeDrive.inputs.temperature = 0.0;
    (*obj)->data.rightKneeDrive.inputs.position = 0.0;
    (*obj)->data.rightKneeDrive.inputs.torque = 0.0;
    (*obj)->data.rightKneeDrive.outputs.controlWord = 0.0;
    (*obj)->data.rightKneeDrive.outputs.torque = 0.0;
    (*obj)->data.rightFootDrive.inputs.statusWord = 0.0;
    (*obj)->data.rightFootDrive.inputs.temperature = 0.0;
    (*obj)->data.rightFootDrive.inputs.position = 0.0;
    (*obj)->data.rightFootDrive.inputs.torque = 0.0;
    (*obj)->data.rightFootDrive.outputs.controlWord = 0.0;
    (*obj)->data.rightFootDrive.outputs.torque = 0.0;
}

static void b_CassieOutputs_CassieOutputs(CassieOutputs **obj)
{
    CassieOutputs *b_this;
    int i;
    b_this = *obj;
    *obj = b_this;
    for (i = 0; i < 4; i++) {
        (*obj)->data.vectorNavOrientation[i] = 0.0;
    }

    for (i = 0; i < 3; i++) {
        (*obj)->data.vectorNavAngularVelocity[i] = 0.0;
    }

    for (i = 0; i < 3; i++) {
        (*obj)->data.vectorNavLinearAcceleration[i] = 0.0;
    }

    for (i = 0; i < 3; i++) {
        (*obj)->data.vectorNavMagnetometer[i] = 0.0;
    }

    (*obj)->data.vectorNavPressure = 0.0;
    (*obj)->data.vectorNavTemperature = 0.0;
    for (i = 0; i < 10; i++) {
        (*obj)->data.motorPosition[i] = 0.0;
    }

    for (i = 0; i < 10; i++) {
        (*obj)->data.motorVelocity[i] = 0.0;
    }

    for (i = 0; i < 6; i++) {
        (*obj)->data.jointPosition[i] = 0.0;
    }

    for (i = 0; i < 6; i++) {
        (*obj)->data.jointVelocity[i] = 0.0;
    }

    for (i = 0; i < 16; i++) {
        (*obj)->data.radio[i] = 0.0;
    }

    (*obj)->data.stateOfCharge = 1.0;
    (*obj)->data.status = 0.0;
}

static void b_SystemCore_setup(TestController *obj)
{
    TestController *b_obj;
    int i;
    signed char c_rightFootTransform_rotation_d[9];
    static const signed char iv0[9] = { 1, 0, 0, 0, 1, 0, 0, 0, 1 };

    signed char leftFootTransform_rotation_data[9];
    static const double lb[10] = { -0.26179938779914941, -0.39269908169872414,
        -0.87266462599716477, -2.8623399732707004, -2.4434609527920612,
        -0.39269908169872414, -0.39269908169872414, -0.87266462599716477,
        -2.8623399732707004, -2.4434609527920612 };

    static const double ub[10] = { 0.39269908169872414, 0.39269908169872414,
        1.3962634015954636, -0.64577182323790194, -0.52359877559829882,
        0.26179938779914941, 0.39269908169872414, 1.3962634015954636,
        -0.64577182323790194, -0.52359877559829882 };

    NonlinearLeastSquaresSolver *c_obj;
    static const short b_proportionalGain[10] = { 300, 300, 300, 300, 30, 300,
        300, 300, 300, 30 };

    static const signed char b_derivativeGain[10] = { 4, 4, 12, 12, 3, 4, 4, 12,
        12, 3 };

    obj->isInitialized = 1;
    b_obj = obj;
    for (i = 0; i < 9; i++) {
        b_obj->inputs.data.radio[i] = 0.0;
    }

    for (i = 0; i < 10; i++) {
        b_obj->inputs.data.torque[i] = 0.0;
    }

    CassieOutputs_CassieOutputs(&b_obj->outputs);
    for (i = 0; i < 9; i++) {
        c_rightFootTransform_rotation_d[i] = iv0[i];
    }

    for (i = 0; i < 9; i++) {
        leftFootTransform_rotation_data[i] = iv0[i];
    }

    for (i = 0; i < 3; i++) {
        b_obj->ikFunction.desiredState[i] = 0.0;
    }

    for (i = 0; i < 3; i++) {
        b_obj->ikFunction.desiredState[i + 3] =
            leftFootTransform_rotation_data[i];
    }

    for (i = 0; i < 3; i++) {
        b_obj->ikFunction.desiredState[i + 6] = 0.0;
    }

    for (i = 0; i < 3; i++) {
        b_obj->ikFunction.desiredState[i + 9] =
            c_rightFootTransform_rotation_d[i];
    }

    for (i = 0; i < 10; i++) {
        b_obj->ikProblem.lowerBound[i] = lb[i];
    }

    for (i = 0; i < 10; i++) {
        b_obj->ikProblem.upperBound[i] = ub[i];
    }

    b_obj->ikProblem.residualFunction = &b_obj->ikFunction;
    c_obj = &b_obj->ikSolver;
    b_obj->ikSolver.problem = &b_obj->ikProblem;
    for (i = 0; i < 10; i++) {
        c_obj->x[i] = 0.0;
    }

    c_obj->levenbergMarquardtParameter = 0.001;
    c_obj->iterationLimit = 1000.0;
    c_obj->xTolerance = 1.4901161193847656E-8;
    c_obj->gTolerance = 1.4901161193847656E-8;
    c_obj->status = ITERATE;
    b_obj->ikSolver.iterationLimit = 10.0;
    b_obj->ikSolver.gTolerance = 2.2204460492503131E-16;
    b_obj->ikSolver.xTolerance = 2.2204460492503131E-16;
    for (i = 0; i < 10; i++) {
        b_obj->pdController.activeMotorIndex[i] = 1.0 + (double)i;
    }

    for (i = 0; i < 10; i++) {
        b_obj->pdController.proportionalGain[i] = 0.0;
    }

    for (i = 0; i < 10; i++) {
        b_obj->pdController.derivativeGain[i] = 0.0;
    }

    for (i = 0; i < 10; i++) {
        b_obj->pdController.desiredMotorPosition[i] = 0.0;
    }

    for (i = 0; i < 10; i++) {
        b_obj->pdController.desiredMotorVelocity[i] = 0.0;
    }

    for (i = 0; i < 10; i++) {
        b_obj->pdController.proportionalGain[i] = b_proportionalGain[i];
    }

    for (i = 0; i < 10; i++) {
        b_obj->pdController.derivativeGain[i] = b_derivativeGain[i];
    }

    b_obj->controllerMode = 0.0;
    b_obj->softStart = 0.0;
}

static int b_bsearch(const double b_x[64], double xi)
{
    int n;
    int low_ip1;
    int high_i;
    int mid_i;
    n = 1;
    low_ip1 = 2;
    high_i = 64;
    while (high_i > low_ip1) {
        mid_i = (n + high_i) >> 1;
        if (xi >= b_x[mid_i - 1]) {
            n = mid_i;
            low_ip1 = mid_i + 1;
        } else {
            high_i = mid_i;
        }
    }

    return n;
}

static void b_sort(double b_x[5])
{
    int i;
    int idx[5];
    int ib;
    double x4[4];
    int k;
    signed char idx4[4];
    signed char perm[4];
    int iwork[5];
    double xwork[5];
    int i3;
    int i4;
    for (i = 0; i < 5; i++) {
        idx[i] = 0;
    }

    for (i = 0; i < 4; i++) {
        x4[i] = 0.0;
        idx4[i] = 0;
    }

    ib = 0;
    for (k = 0; k < 5; k++) {
        ib++;
        idx4[ib - 1] = (signed char)(k + 1);
        x4[ib - 1] = b_x[k];
        if (ib == 4) {
            if (x4[0] >= x4[1]) {
                i = 1;
                ib = 2;
            } else {
                i = 2;
                ib = 1;
            }

            if (x4[2] >= x4[3]) {
                i3 = 3;
                i4 = 4;
            } else {
                i3 = 4;
                i4 = 3;
            }

            if (x4[i - 1] >= x4[i3 - 1]) {
                if (x4[ib - 1] >= x4[i3 - 1]) {
                    perm[0] = (signed char)i;
                    perm[1] = (signed char)ib;
                    perm[2] = (signed char)i3;
                    perm[3] = (signed char)i4;
                } else if (x4[ib - 1] >= x4[i4 - 1]) {
                    perm[0] = (signed char)i;
                    perm[1] = (signed char)i3;
                    perm[2] = (signed char)ib;
                    perm[3] = (signed char)i4;
                } else {
                    perm[0] = (signed char)i;
                    perm[1] = (signed char)i3;
                    perm[2] = (signed char)i4;
                    perm[3] = (signed char)ib;
                }
            } else if (x4[i - 1] >= x4[i4 - 1]) {
                if (x4[ib - 1] >= x4[i4 - 1]) {
                    perm[0] = (signed char)i3;
                    perm[1] = (signed char)i;
                    perm[2] = (signed char)ib;
                    perm[3] = (signed char)i4;
                } else {
                    perm[0] = (signed char)i3;
                    perm[1] = (signed char)i;
                    perm[2] = (signed char)i4;
                    perm[3] = (signed char)ib;
                }
            } else {
                perm[0] = (signed char)i3;
                perm[1] = (signed char)i4;
                perm[2] = (signed char)i;
                perm[3] = (signed char)ib;
            }

            idx[k - 3] = idx4[perm[0] - 1];
            idx[k - 2] = idx4[perm[1] - 1];
            idx[k - 1] = idx4[perm[2] - 1];
            idx[k] = idx4[perm[3] - 1];
            b_x[k - 3] = x4[perm[0] - 1];
            b_x[k - 2] = x4[perm[1] - 1];
            b_x[k - 1] = x4[perm[2] - 1];
            b_x[k] = x4[perm[3] - 1];
            ib = 0;
        }
    }

    if (ib > 0) {
        for (i = 0; i < 4; i++) {
            perm[i] = 0;
        }

        switch (ib) {
          case 1:
            perm[0] = 1;
            break;

          case 2:
            if (x4[0] >= x4[1]) {
                perm[0] = 1;
                perm[1] = 2;
            } else {
                perm[0] = 2;
                perm[1] = 1;
            }
            break;

          default:
            if (x4[0] >= x4[1]) {
                if (x4[1] >= x4[2]) {
                    perm[0] = 1;
                    perm[1] = 2;
                    perm[2] = 3;
                } else if (x4[0] >= x4[2]) {
                    perm[0] = 1;
                    perm[1] = 3;
                    perm[2] = 2;
                } else {
                    perm[0] = 3;
                    perm[1] = 1;
                    perm[2] = 2;
                }
            } else if (x4[0] >= x4[2]) {
                perm[0] = 2;
                perm[1] = 1;
                perm[2] = 3;
            } else if (x4[1] >= x4[2]) {
                perm[0] = 2;
                perm[1] = 3;
                perm[2] = 1;
            } else {
                perm[0] = 3;
                perm[1] = 2;
                perm[2] = 1;
            }
            break;
        }

        for (k = 5; k - 4 <= ib; k++) {
            idx[k - ib] = idx4[perm[k - 5] - 1];
            b_x[k - ib] = x4[perm[k - 5] - 1];
        }
    }

    for (i = 0; i < 5; i++) {
        xwork[i] = 0.0;
        iwork[i] = 0;
    }

    merge(idx, b_x, 0, 4, 1, iwork, xwork);
}

static void c_InverseKinematicsFunction_eva(const InverseKinematicsFunction *obj,
    const double q[10], double f[12], double J[120])
{
    double t2;
    double t3;
    double t5;
    double t6;
    double t7;
    double t8;
    double t12;
    double t13;
    double t14;
    double t17;
    double t15;
    double t16;
    double t18;
    double t61;
    double t19;
    double t20;
    double t23;
    double t24;
    double t25;
    double t26;
    double t29;
    double t31;
    double t35;
    double t37;
    double t41;
    double t43;
    double t45;
    double t48;
    double t52;
    double t54;
    double t58;
    double t60;
    double t64;
    double t65;
    double t67;
    double t68;
    double t69;
    double t70;
    double t73;
    double t74;
    double t75;
    double t78;
    double t76;
    double t77;
    double t79;
    double t122;
    double t80;
    double t81;
    double t84;
    double t85;
    double t86;
    double t87;
    double t90;
    double t92;
    double t96;
    double t98;
    double t102;
    double t104;
    double t106;
    double t109;
    double t113;
    double t115;
    double t119;
    double t121;
    double t127;
    double t129;
    double t133;
    double t135;
    double t136;
    double t137;
    double t138;
    double t139;
    double t140;
    double t141;
    double t142;
    double t143;
    double t144;
    double t145;
    double t148;
    double t149;
    double t151;
    double t150;
    double t152;
    double t190;
    double t153;
    double t154;
    double t155;
    double t156;
    double t157;
    double t158;
    double t159;
    double t160;
    double t161;
    double t162;
    double t163;
    double t164;
    double t167;
    double t168;
    double t173;
    double t169;
    double t170;
    double t199;
    double t171;
    double t175;
    double t176;
    double t177;
    double t178;
    double t179;
    double t180;
    double t181;
    double t186;
    double t187;
    double t188;
    double t189;
    double t192;
    double t196;
    double t198;
    double t201;
    double t204;
    double t206;
    double t210;
    double t212;
    double t213;
    double t214;
    double t215;
    double t216;
    double t217;
    double t218;
    double t219;
    double t220;
    double t221;
    double t222;
    double t223;
    double t226;
    double t227;
    double t229;
    double t228;
    double t230;
    double t269;
    double t231;
    double t232;
    double t233;
    double t234;
    double t235;
    double t236;
    double t237;
    double t238;
    double t239;
    double t240;
    double t241;
    double t242;
    double t243;
    double t246;
    double t247;
    double t252;
    double t249;
    double t278;
    double t250;
    double t255;
    double t256;
    double t257;
    double t258;
    double t259;
    double t260;
    double t265;
    double t266;
    double t267;
    double t268;
    double t271;
    double t275;
    double t277;
    double t280;
    double dv5[120];
    int i6;
    int i7;
    t2 = cos(q[1]);
    t3 = sin(q[2]);
    t5 = cos(q[3] - 0.22689280275926282);
    t6 = cos(q[2]);
    t7 = cos(q[3]);
    t8 = sin(q[3]);
    t12 = t2 * t6 * t8 + t2 * t3 * t7;
    t13 = sin(q[3] - 0.22689280275926282);
    t14 = t2 * t3 * t8;
    t17 = t2 * t6 * t7;
    t15 = t14 - t17;
    t16 = cos(q[4]);
    t18 = t5 * t15;
    t61 = t12 * t13;
    t19 = t18 - t61;
    t20 = sin(q[4]);
    t23 = t5 * t12 + t13 * (t14 - t17);
    t24 = sin(q[0]);
    t25 = sin(q[1]);
    t26 = cos(q[0]);
    t29 = t6 * t24 + t3 * t25 * t26;
    t31 = t3 * t24 - t6 * t25 * t26;
    t35 = t7 * t31 + t8 * t29;
    t37 = t7 * t29 - t8 * t31;
    t41 = t5 * t37 + t13 * t35;
    t43 = t5 * t35 - t13 * t37;
    t45 = t6 * t26 - t3 * t24 * t25;
    t48 = t3 * t26 + t6 * t24 * t25;
    t52 = t7 * t48 + t8 * t45;
    t54 = t7 * t45 - t8 * t48;
    t58 = t5 * t54 + t13 * t52;
    t60 = t5 * t52 - t13 * t54;
    t64 = cos(q[6]);
    t65 = sin(q[7]);
    t67 = cos(q[8] - 0.22689280275926282);
    t68 = cos(q[7]);
    t69 = cos(q[8]);
    t70 = sin(q[8]);
    t73 = t64 * t68 * t70 + t64 * t65 * t69;
    t74 = sin(q[8] - 0.22689280275926282);
    t75 = t64 * t65 * t70;
    t78 = t64 * t68 * t69;
    t76 = t75 - t78;
    t77 = cos(q[9]);
    t79 = t67 * t76;
    t122 = t73 * t74;
    t80 = t79 - t122;
    t81 = sin(q[9]);
    t84 = t67 * t73 + t74 * (t75 - t78);
    t85 = sin(q[5]);
    t86 = sin(q[6]);
    t87 = cos(q[5]);
    t90 = t68 * t85 + t65 * t86 * t87;
    t92 = t65 * t85 - t68 * t86 * t87;
    t96 = t69 * t92 + t70 * t90;
    t98 = t69 * t90 - t70 * t92;
    t102 = t67 * t98 + t74 * t96;
    t104 = t67 * t96 - t74 * t98;
    t106 = t68 * t87 - t65 * t85 * t86;
    t109 = t65 * t87 + t68 * t85 * t86;
    t113 = t69 * t109 + t70 * t106;
    t115 = t69 * t106 - t70 * t109;
    t119 = t67 * t115 + t74 * t113;
    t121 = t67 * t113 - t74 * t115;
    t127 = t6 * t8 * t25 + t3 * t7 * t25;
    t129 = t3 * t8 * t25 - t6 * t7 * t25;
    t133 = t5 * t127 + t13 * t129;
    t135 = t5 * t129 - t13 * t127;
    t136 = t2 * t6 * t7 * 0.49544;
    t137 = t20 * (t18 - t61) * 0.05219;
    t138 = t13 * t54 * 0.04;
    t139 = t7 * t48 * 0.06741;
    t140 = t16 * t60 * 0.05219;
    t141 = t8 * t48 * 0.49544;
    t142 = t8 * t45 * 0.06741;
    t143 = t20 * t58 * 0.05219;
    t144 = t20 * t60 * 0.01762;
    t145 = t3 * t24 * t25 * 0.12;
    t148 = t2 * t6 * t8 * t26 + t2 * t3 * t7 * t26;
    t149 = t2 * t3 * t8 * t26;
    t151 = t2 * t6 * t7 * t26;
    t150 = t149 - t151;
    t152 = t5 * t150;
    t190 = t13 * t148;
    t153 = t152 - t190;
    t154 = t5 * t148;
    t155 = t8 * t31 * 0.06741;
    t156 = t20 * t43 * 0.05219;
    t157 = t24 * 0.09;
    t158 = t5 * t35 * 0.04;
    t159 = t5 * t37 * 0.408;
    t160 = t13 * t35 * 0.408;
    t161 = t6 * t24 * 0.12;
    t162 = t7 * t29 * 0.49544;
    t163 = t16 * t41 * 0.01762;
    t164 = t3 * t25 * t26 * 0.12;
    t167 = t2 * t6 * t8 * t24 + t2 * t3 * t7 * t24;
    t168 = t2 * t3 * t8 * t24;
    t173 = t2 * t6 * t7 * t24;
    t169 = t168 - t173;
    t170 = t5 * t169;
    t199 = t13 * t167;
    t171 = t170 - t199;
    t175 = t5 * t167 + t13 * t169;
    t176 = t7 * t48 * 0.49544;
    t177 = t7 * t45 * 0.06741;
    t178 = t8 * t45 * 0.49544;
    t179 = t16 * t58 * 0.05219;
    t180 = t16 * t60 * 0.01762;
    t181 = t20 * t58 * 0.01762;
    t186 = ((t16 * (t18 - t61) * 0.7660444431189779 + t20 * (t18 - t61) *
             0.64278760968653947) + t20 * t23 * 0.7660444431189779) - t16 * t23 *
        0.64278760968653947;
    t187 = t16 * t58 * 0.7660444431189779;
    t188 = t16 * t60 * 0.64278760968653947;
    t189 = t20 * t58 * 0.64278760968653947;
    t192 = t154 + t13 * (t149 - t151);
    t196 = ((t16 * t43 * 0.7660444431189779 + t20 * t41 * 0.7660444431189779) +
            t20 * t43 * 0.64278760968653947) - t16 * t41 * 0.64278760968653947;
    t198 = ((t20 * t43 * 0.7660444431189779 - t16 * t41 * 0.7660444431189779) -
            t16 * t43 * 0.64278760968653947) - t20 * t41 * 0.64278760968653947;
    t201 = ((t16 * t58 * 0.64278760968653947 - t16 * t60 * 0.7660444431189779) -
            t20 * t58 * 0.7660444431189779) - t20 * t60 * 0.64278760968653947;
    t204 = t68 * t70 * t86 + t65 * t69 * t86;
    t206 = t65 * t70 * t86 - t68 * t69 * t86;
    t210 = t67 * t204 + t74 * t206;
    t212 = t67 * t206 - t74 * t204;
    t213 = t64 * t68 * t69 * 0.49544;
    t214 = t81 * (t79 - t122) * 0.05219;
    t215 = t74 * t115 * 0.04;
    t216 = t64 * t85 * 0.0045;
    t217 = t69 * t109 * 0.06741;
    t218 = t77 * t121 * 0.05219;
    t219 = t70 * t109 * 0.49544;
    t220 = t70 * t106 * 0.06741;
    t221 = t81 * t119 * 0.05219;
    t222 = t81 * t121 * 0.01762;
    t223 = t65 * t85 * t86 * 0.12;
    t226 = t64 * t68 * t70 * t87 + t64 * t65 * t69 * t87;
    t227 = t64 * t65 * t70 * t87;
    t229 = t64 * t68 * t69 * t87;
    t228 = t227 - t229;
    t230 = t67 * t228;
    t269 = t74 * t226;
    t231 = t230 - t269;
    t232 = t67 * t226;
    t233 = t70 * t92 * 0.06741;
    t234 = t81 * t104 * 0.05219;
    t235 = t85 * 0.09;
    t236 = t67 * t96 * 0.04;
    t237 = t67 * t98 * 0.408;
    t238 = t64 * t87 * 0.0045;
    t239 = t74 * t96 * 0.408;
    t240 = t68 * t85 * 0.12;
    t241 = t69 * t90 * 0.49544;
    t242 = t77 * t102 * 0.01762;
    t243 = t65 * t86 * t87 * 0.12;
    t246 = t64 * t68 * t70 * t85 + t64 * t65 * t69 * t85;
    t247 = t64 * t65 * t70 * t85;
    t252 = t64 * t68 * t69 * t85;
    t169 = t247 - t252;
    t249 = t67 * t169;
    t278 = t74 * t246;
    t250 = t249 - t278;
    t169 = t67 * t246 + t74 * t169;
    t255 = t69 * t109 * 0.49544;
    t256 = t69 * t106 * 0.06741;
    t257 = t70 * t106 * 0.49544;
    t258 = t77 * t119 * 0.05219;
    t259 = t77 * t121 * 0.01762;
    t260 = t81 * t119 * 0.01762;
    t265 = ((t77 * (t79 - t122) * 0.7660444431189779 + t81 * (t79 - t122) *
             0.64278760968653947) + t81 * t84 * 0.7660444431189779) - t77 * t84 *
        0.64278760968653947;
    t266 = t77 * t119 * 0.7660444431189779;
    t267 = t77 * t121 * 0.64278760968653947;
    t268 = t81 * t119 * 0.64278760968653947;
    t271 = t232 + t74 * (t227 - t229);
    t275 = ((t77 * t104 * 0.7660444431189779 + t81 * t102 * 0.7660444431189779)
            + t81 * t104 * 0.64278760968653947) - t77 * t102 *
        0.64278760968653947;
    t277 = ((t81 * t104 * 0.7660444431189779 - t77 * t102 * 0.7660444431189779)
            - t77 * t104 * 0.64278760968653947) - t81 * t102 *
        0.64278760968653947;
    t280 = ((t77 * t119 * 0.64278760968653947 - t77 * t121 * 0.7660444431189779)
            - t81 * t119 * 0.7660444431189779) - t81 * t121 *
        0.64278760968653947;
    dv5[0] = 0.0;
    dv5[1] = ((((((((((((((t26 * 0.09 - t138) - t139) - t140) - t141) - t142) -
                      t143) - t144) - t145) + t2 * t24 * 0.0045) + t6 * t26 *
                  0.12) + t7 * t45 * 0.49544) + t5 * t52 * 0.04) + t5 * t54 *
               0.408) + t13 * t52 * 0.408) + t16 * t58 * 0.01762;
    dv5[2] = ((((((((((((((t157 + t158) + t159) + t160) + t161) + t162) + t163)
                     + t164) - t2 * t26 * 0.0045) - t8 * t29 * 0.06741) - t7 *
                  t31 * 0.06741) - t8 * t31 * 0.49544) - t13 * t37 * 0.04) - t16
               * t43 * 0.05219) - t20 * t41 * 0.05219) - t20 * t43 * 0.01762;
    dv5[3] = 0.0;
    dv5[4] = ((-t187 - t188) - t189) + t20 * t60 * 0.7660444431189779;
    dv5[5] = t198;
    dv5[6] = 0.0;
    dv5[7] = 0.0;
    dv5[8] = 0.0;
    dv5[9] = 0.0;
    dv5[10] = 0.0;
    dv5[11] = 0.0;
    dv5[12] = ((((((((((((t2 * 0.0045 - t3 * t25 * 0.12) - t5 * t127 * 0.408) -
                        t5 * t129 * 0.04) + t13 * t127 * 0.04) - t13 * t129 *
                      0.408) - t16 * t133 * 0.01762) + t16 * t135 * 0.05219) +
                   t20 * t133 * 0.05219) + t20 * t135 * 0.01762) - t3 * t7 * t25
                 * 0.49544) + t3 * t8 * t25 * 0.06741) - t6 * t7 * t25 * 0.06741)
        - t6 * t8 * t25 * 0.49544;
    dv5[13] = ((((((((((((t25 * t26 * 0.0045 + t5 * t148 * 0.408) - t13 * t148 *
                         0.04) - t16 * t153 * 0.05219) - t20 * t153 * 0.01762) +
                      t16 * t192 * 0.01762) + t5 * (t149 - t151) * 0.04) + t13 *
                    (t149 - t151) * 0.408) - t20 * (t154 + t13 * t150) * 0.05219)
                  + t2 * t3 * t26 * 0.12) + t2 * t3 * t7 * t26 * 0.49544) - t2 *
                t3 * t8 * t26 * 0.06741) + t2 * t6 * t7 * t26 * 0.06741) + t2 *
        t6 * t8 * t26 * 0.49544;
    dv5[14] = ((((((((((((t24 * t25 * 0.0045 + t5 * t167 * 0.408) - t13 * t167 *
                         0.04) - t16 * t171 * 0.05219) + t16 * t175 * 0.01762) -
                      t20 * t171 * 0.01762) - t20 * t175 * 0.05219) + t5 * (t168
                     - t173) * 0.04) + t13 * (t168 - t173) * 0.408) + t2 * t3 *
                  t24 * 0.12) + t2 * t3 * t7 * t24 * 0.49544) - t2 * t3 * t8 *
                t24 * 0.06741) + t2 * t6 * t7 * t24 * 0.06741) + t2 * t6 * t8 *
        t24 * 0.49544;
    dv5[15] = ((t16 * t133 * 0.7660444431189779 + t16 * t135 *
                0.64278760968653947) + t20 * t133 * 0.64278760968653947) - t20 *
        t135 * 0.7660444431189779;
    dv5[16] = ((t16 * t153 * -0.64278760968653947 - t16 * t192 *
                0.7660444431189779) - t20 * t192 * 0.64278760968653947) + t20 *
        (t152 - t190) * 0.7660444431189779;
    dv5[17] = ((t16 * t171 * -0.64278760968653947 - t16 * t175 *
                0.7660444431189779) - t20 * t175 * 0.64278760968653947) + t20 *
        (t170 - t199) * 0.7660444431189779;
    dv5[18] = 0.0;
    dv5[19] = 0.0;
    dv5[20] = 0.0;
    dv5[21] = 0.0;
    dv5[22] = 0.0;
    dv5[23] = 0.0;
    dv5[24] = (((((((((((t136 + t137) + t2 * t6 * 0.12) + t5 * t12 * 0.04) - t5 *
                      t15 * 0.408) + t12 * t13 * 0.408) - t16 * t19 * 0.01762) -
                   t16 * t23 * 0.05219) - t20 * t23 * 0.01762) + t13 * (t14 -
                  t17) * 0.04) - t2 * t3 * t7 * 0.06741) - t2 * t3 * t8 *
               0.49544) - t2 * t6 * t8 * 0.06741;
    dv5[25] = ((((((((((((t155 + t156) - t3 * t24 * 0.12) - t7 * t29 * 0.06741)
                       - t8 * t29 * 0.49544) - t7 * t31 * 0.49544) - t5 * t35 *
                     0.408) + t5 * t37 * 0.04) + t13 * t35 * 0.04) + t13 * t37 *
                  0.408) - t16 * t41 * 0.05219) - t16 * t43 * 0.01762) - t20 *
               t41 * 0.01762) + t6 * t25 * t26 * 0.12;
    dv5[26] = ((((((((((((t176 + t177) + t178) + t179) + t180) + t181) + t3 *
                     t26 * 0.12) - t8 * t48 * 0.06741) + t5 * t52 * 0.408) - t5 *
                  t54 * 0.04) - t13 * t52 * 0.04) - t13 * t54 * 0.408) - t20 *
               t60 * 0.05219) + t6 * t24 * t25 * 0.12;
    dv5[27] = t186;
    dv5[28] = t196;
    dv5[29] = t201;
    dv5[30] = 0.0;
    dv5[31] = 0.0;
    dv5[32] = 0.0;
    dv5[33] = 0.0;
    dv5[34] = 0.0;
    dv5[35] = 0.0;
    dv5[36] = ((t136 - t2 * t3 * t7 * 0.06741) - t2 * t3 * t8 * 0.49544) - t2 *
        t6 * t8 * 0.06741;
    dv5[37] = ((t155 - t7 * t29 * 0.06741) - t8 * t29 * 0.49544) - t7 * t31 *
        0.49544;
    dv5[38] = ((t176 + t177) + t178) - t8 * t48 * 0.06741;
    memset(&dv5[39], 0, 9U * sizeof(double));
    dv5[48] = ((t137 - t16 * t19 * 0.01762) - t16 * t23 * 0.05219) - t20 * t23 *
        0.01762;
    dv5[49] = ((t156 - t16 * t41 * 0.05219) - t16 * t43 * 0.01762) - t20 * t41 *
        0.01762;
    dv5[50] = ((t179 + t180) + t181) - t20 * t60 * 0.05219;
    dv5[51] = t186;
    dv5[52] = t196;
    dv5[53] = t201;
    memset(&dv5[54], 0, 13U * sizeof(double));
    dv5[67] = ((((((((((((((t87 * 0.09 - t215) - t216) - t217) - t218) - t219) -
                       t220) - t221) - t222) - t223) + t68 * t87 * 0.12) + t69 *
                  t106 * 0.49544) + t67 * t113 * 0.04) + t67 * t115 * 0.408) +
               t74 * t113 * 0.408) + t77 * t119 * 0.01762;
    dv5[68] = ((((((((((((((t235 + t236) + t237) + t238) + t239) + t240) + t241)
                      + t242) + t243) - t70 * t90 * 0.06741) - t69 * t92 *
                   0.06741) - t70 * t92 * 0.49544) - t74 * t98 * 0.04) - t77 *
                t104 * 0.05219) - t81 * t102 * 0.05219) - t81 * t104 * 0.01762;
    dv5[69] = 0.0;
    dv5[70] = ((-t266 - t267) - t268) + t81 * t121 * 0.7660444431189779;
    dv5[71] = t277;
    dv5[72] = 0.0;
    dv5[73] = 0.0;
    dv5[74] = 0.0;
    dv5[75] = 0.0;
    dv5[76] = 0.0;
    dv5[77] = 0.0;
    dv5[78] = ((((((((((((t64 * -0.0045 - t65 * t86 * 0.12) - t67 * t204 * 0.408)
                        - t67 * t206 * 0.04) + t74 * t204 * 0.04) - t74 * t206 *
                      0.408) - t77 * t210 * 0.01762) + t77 * t212 * 0.05219) +
                   t81 * t210 * 0.05219) + t81 * t212 * 0.01762) - t65 * t69 *
                 t86 * 0.49544) + t65 * t70 * t86 * 0.06741) - t68 * t69 * t86 *
               0.06741) - t68 * t70 * t86 * 0.49544;
    dv5[79] = ((((((((((((t86 * t87 * -0.0045 + t67 * t226 * 0.408) - t74 * t226
                         * 0.04) - t77 * t231 * 0.05219) - t81 * t231 * 0.01762)
                      + t77 * t271 * 0.01762) + t67 * (t227 - t229) * 0.04) +
                    t74 * (t227 - t229) * 0.408) - t81 * (t232 + t74 * t228) *
                   0.05219) + t64 * t65 * t87 * 0.12) + t64 * t65 * t69 * t87 *
                 0.49544) - t64 * t65 * t70 * t87 * 0.06741) + t64 * t68 * t69 *
               t87 * 0.06741) + t64 * t68 * t70 * t87 * 0.49544;
    dv5[80] = ((((((((((((t85 * t86 * -0.0045 + t67 * t246 * 0.408) - t74 * t246
                         * 0.04) - t77 * t250 * 0.05219) + t77 * t169 * 0.01762)
                      - t81 * t250 * 0.01762) - t81 * t169 * 0.05219) + t67 *
                    (t247 - t252) * 0.04) + t74 * (t247 - t252) * 0.408) + t64 *
                  t65 * t85 * 0.12) + t64 * t65 * t69 * t85 * 0.49544) - t64 *
                t65 * t70 * t85 * 0.06741) + t64 * t68 * t69 * t85 * 0.06741) +
        t64 * t68 * t70 * t85 * 0.49544;
    dv5[81] = ((t77 * t210 * 0.7660444431189779 + t77 * t212 *
                0.64278760968653947) + t81 * t210 * 0.64278760968653947) - t81 *
        t212 * 0.7660444431189779;
    dv5[82] = ((t77 * t231 * -0.64278760968653947 - t77 * t271 *
                0.7660444431189779) - t81 * t271 * 0.64278760968653947) + t81 *
        (t230 - t269) * 0.7660444431189779;
    dv5[83] = ((t77 * t250 * -0.64278760968653947 - t77 * t169 *
                0.7660444431189779) - t81 * t169 * 0.64278760968653947) + t81 *
        (t249 - t278) * 0.7660444431189779;
    dv5[84] = 0.0;
    dv5[85] = 0.0;
    dv5[86] = 0.0;
    dv5[87] = 0.0;
    dv5[88] = 0.0;
    dv5[89] = 0.0;
    dv5[90] = (((((((((((t213 + t214) + t64 * t68 * 0.12) + t67 * t73 * 0.04) -
                      t67 * t76 * 0.408) + t73 * t74 * 0.408) - t77 * t80 *
                    0.01762) - t77 * t84 * 0.05219) - t81 * t84 * 0.01762) + t74
                 * (t75 - t78) * 0.04) - t64 * t65 * t69 * 0.06741) - t64 * t65 *
               t70 * 0.49544) - t64 * t68 * t70 * 0.06741;
    dv5[91] = ((((((((((((t233 + t234) - t65 * t85 * 0.12) - t69 * t90 * 0.06741)
                       - t70 * t90 * 0.49544) - t69 * t92 * 0.49544) - t67 * t96
                     * 0.408) + t67 * t98 * 0.04) + t74 * t96 * 0.04) + t74 *
                  t98 * 0.408) - t77 * t102 * 0.05219) - t77 * t104 * 0.01762) -
               t81 * t102 * 0.01762) + t68 * t86 * t87 * 0.12;
    dv5[92] = ((((((((((((t255 + t256) + t257) + t258) + t259) + t260) + t65 *
                     t87 * 0.12) - t70 * t109 * 0.06741) + t67 * t113 * 0.408) -
                  t67 * t115 * 0.04) - t74 * t113 * 0.04) - t74 * t115 * 0.408)
               - t81 * t121 * 0.05219) + t68 * t85 * t86 * 0.12;
    dv5[93] = t265;
    dv5[94] = t275;
    dv5[95] = t280;
    dv5[96] = 0.0;
    dv5[97] = 0.0;
    dv5[98] = 0.0;
    dv5[99] = 0.0;
    dv5[100] = 0.0;
    dv5[101] = 0.0;
    dv5[102] = ((t213 - t64 * t65 * t69 * 0.06741) - t64 * t65 * t70 * 0.49544)
        - t64 * t68 * t70 * 0.06741;
    dv5[103] = ((t233 - t69 * t90 * 0.06741) - t70 * t90 * 0.49544) - t69 * t92 *
        0.49544;
    dv5[104] = ((t255 + t256) + t257) - t70 * t109 * 0.06741;
    memset(&dv5[105], 0, 9U * sizeof(double));
    dv5[114] = ((t214 - t77 * t80 * 0.01762) - t77 * t84 * 0.05219) - t81 * t84 *
        0.01762;
    dv5[115] = ((t234 - t77 * t102 * 0.05219) - t77 * t104 * 0.01762) - t81 *
        t102 * 0.01762;
    dv5[116] = ((t258 + t259) + t260) - t81 * t121 * 0.05219;
    dv5[117] = t265;
    dv5[118] = t275;
    dv5[119] = t280;
    i6 = 0;
    for (i7 = 0; i7 < 10; i7++) {
        memcpy(&J[i6], &dv5[i6], 12U * sizeof(double));
        i6 += 12;
    }

    f[0] = ((((((((((((((t25 * 0.0045 + t2 * t3 * 0.12) + t5 * t12 * 0.408) + t5
                       * t15 * 0.04) - t12 * t13 * 0.04) + t13 * t15 * 0.408) -
                    t16 * t19 * 0.05219) + t16 * t23 * 0.01762) - t19 * t20 *
                  0.01762) - t20 * t23 * 0.05219) + t2 * t3 * t7 * 0.49544) - t2
               * t3 * t8 * 0.06741) + t2 * t6 * t7 * 0.06741) + t2 * t6 * t8 *
             0.49544) - 0.049) - obj->desiredState[0];
    f[1] = ((((((((((((((((t157 + t158) + t159) + t160) + t161) + t162) + t163)
                     + t164) - t2 * t26 * 0.0045) - t8 * t29 * 0.06741) - t7 *
                  t31 * 0.06741) - t8 * t31 * 0.49544) - t13 * t37 * 0.04) - t16
               * t43 * 0.05219) - t20 * t41 * 0.05219) - t20 * t43 * 0.01762) +
            0.135) - obj->desiredState[1];
    f[2] = (((((((((((((((t26 * -0.09 + t138) + t139) + t140) + t141) + t142) +
                     t143) + t144) + t145) - t2 * t24 * 0.0045) - t6 * t26 *
                 0.12) - t7 * t45 * 0.49544) - t5 * t52 * 0.04) - t5 * t54 *
              0.408) - t13 * t52 * 0.408) - t16 * t58 * 0.01762) -
        obj->desiredState[2];
    f[3] = (((t16 * t19 * -0.64278760968653947 - t16 * t23 * 0.7660444431189779)
             - t20 * t23 * 0.64278760968653947) + t20 * (t18 - t61) *
            0.7660444431189779) - obj->desiredState[3];
    f[4] = t198 - obj->desiredState[4];
    f[5] = (((t187 + t188) + t189) - t20 * t60 * 0.7660444431189779) -
        obj->desiredState[5];
    f[6] = ((((((((((((((t86 * -0.0045 + t64 * t65 * 0.12) + t67 * t73 * 0.408)
                       + t67 * t76 * 0.04) - t73 * t74 * 0.04) + t74 * t76 *
                     0.408) - t77 * t80 * 0.05219) + t77 * t84 * 0.01762) - t80 *
                  t81 * 0.01762) - t81 * t84 * 0.05219) + t64 * t65 * t69 *
                0.49544) - t64 * t65 * t70 * 0.06741) + t64 * t68 * t69 *
              0.06741) + t64 * t68 * t70 * 0.49544) - 0.049) - obj->
        desiredState[6];
    f[7] = ((((((((((((((((t235 + t236) + t237) + t238) + t239) + t240) + t241)
                     + t242) + t243) - t70 * t90 * 0.06741) - t69 * t92 *
                  0.06741) - t70 * t92 * 0.49544) - t74 * t98 * 0.04) - t77 *
               t104 * 0.05219) - t81 * t102 * 0.05219) - t81 * t104 * 0.01762) -
            0.135) - obj->desiredState[7];
    f[8] = (((((((((((((((t87 * -0.09 + t215) + t216) + t217) + t218) + t219) +
                     t220) + t221) + t222) + t223) - t68 * t87 * 0.12) - t69 *
                t106 * 0.49544) - t67 * t113 * 0.04) - t67 * t115 * 0.408) - t74
             * t113 * 0.408) - t77 * t119 * 0.01762) - obj->desiredState[8];
    f[9] = (((t77 * t80 * -0.64278760968653947 - t77 * t84 * 0.7660444431189779)
             - t81 * t84 * 0.64278760968653947) + t81 * (t79 - t122) *
            0.7660444431189779) - obj->desiredState[9];
    f[10] = t277 - obj->desiredState[10];
    f[11] = (((t266 + t267) + t268) - t81 * t121 * 0.7660444431189779) -
        obj->desiredState[11];
}

static void c_NonlinearLeastSquaresSolver_p(const InverseKinematicsFunction *r,
    double b_x[10], const double lb[10], const double ub[10], double r_x[12],
    double J_x[120])
{
    int i;
    int b_r;
    for (i = 0; i < 10; i++) {
        if (b_x[i] < lb[i]) {
            b_x[i] = lb[i];
        } else {
            if (b_x[i] > ub[i]) {
                b_x[i] = ub[i];
            }
        }
    }

    c_InverseKinematicsFunction_eva(r, b_x, r_x, J_x);
    for (i = 0; i < 10; i++) {
        if (b_x[i] == lb[i]) {
            for (b_r = 0; b_r < 12; b_r++) {
                if (J_x[b_r + 12 * i] < 0.0) {
                    J_x[b_r + 12 * i] = 0.0;
                }
            }
        } else {
            if (b_x[i] == ub[i]) {
                for (b_r = 0; b_r < 12; b_r++) {
                    if (J_x[b_r + 12 * i] > 0.0) {
                        J_x[b_r + 12 * i] = 0.0;
                    }
                }
            }
        }
    }
}

static void c_NonlinearLeastSquaresSolver_s(NonlinearLeastSquaresSolver *obj)
{
    NonlinearLeastSquaresProblem *b_obj;
    InverseKinematicsFunction *r;
    int i;
    double lb[10];
    double ub[10];
    double g_x[10];
    double b_lb[10];
    double b_ub[10];
    double r_x[12];
    double J_x[120];
    double y;
    int i2;
    double f_x;
    double nu;
    int j;
    int i3;
    int i4;
    double H_x[100];
    int i5;
    double lambda;
    int exitg1;
    double b_H_x[100];
    static const signed char b[100] = { 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1 };

    double f_new;
    double rho;
    obj->status = ITERATE;
    obj->iteration = 0.0;
    b_obj = obj->problem;
    r = b_obj->residualFunction;
    for (i = 0; i < 10; i++) {
        lb[i] = b_obj->lowerBound[i];
        ub[i] = b_obj->upperBound[i];
    }

    for (i = 0; i < 10; i++) {
        g_x[i] = obj->x[i];
    }

    for (i = 0; i < 10; i++) {
        b_lb[i] = lb[i] + 1.0E-9;
        b_ub[i] = ub[i] - 1.0E-9;
    }

    c_NonlinearLeastSquaresSolver_p(r, g_x, b_lb, b_ub, r_x, J_x);
    for (i = 0; i < 10; i++) {
        obj->x[i] = g_x[i];
    }

    y = 0.0;
    for (i2 = 0; i2 < 12; i2++) {
        y += r_x[i2] * r_x[i2];
    }

    f_x = 0.5 * y;
    y = 0.0;
    for (i = 0; i < 10; i++) {
        g_x[i] = 0.0;
        for (i2 = 0; i2 < 12; i2++) {
            g_x[i] += J_x[i2 + 12 * i] * r_x[i2];
        }

        nu = fabs(g_x[i]);
        if (nu > y) {
            y = nu;
        }
    }

    if (y <= obj->gTolerance) {
        obj->status = G_TOLERANCE;
    } else {
        i2 = 0;
        for (i = 0; i < 10; i++) {
            j = 0;
            i3 = 0;
            for (i4 = 0; i4 < 10; i4++) {
                H_x[j + i] = 0.0;
                for (i5 = 0; i5 < 12; i5++) {
                    H_x[j + i] += J_x[i5 + i2] * J_x[i5 + i3];
                }

                j += 10;
                i3 += 12;
            }

            i2 += 12;
        }

        i = 0;
        for (j = 0; j < 10; j++) {
            b_lb[j] = H_x[i];
            i += 11;
        }

        y = b_lb[0];
        for (i = 0; i < 9; i++) {
            if (b_lb[i + 1] > y) {
                y = b_lb[i + 1];
            }
        }

        lambda = obj->levenbergMarquardtParameter * y;
        nu = 2.0;
        do {
            exitg1 = 0;
            if (obj->status == ITERATE) {
                obj->iteration++;
                if (obj->iteration > obj->iterationLimit) {
                    obj->status = ITERATION_LIMIT;
                }

                for (i = 0; i < 10; i++) {
                    b_lb[i] = -g_x[i];
                }

                for (i2 = 0; i2 < 100; i2++) {
                    b_H_x[i2] = H_x[i2] + lambda * (double)b[i2];
                }

                mldivide(b_H_x, b_lb);
                if (norm(b_lb) <= obj->xTolerance * (obj->xTolerance + norm
                        (obj->x))) {
                    obj->status = X_TOLERANCE;
                    exitg1 = 1;
                } else {
                    for (i = 0; i < 10; i++) {
                        b_ub[i] = obj->x[i] + b_lb[i];
                    }

                    c_NonlinearLeastSquaresSolver_p(r, b_ub, lb, ub, r_x, J_x);
                    y = 0.0;
                    for (i2 = 0; i2 < 12; i2++) {
                        y += r_x[i2] * r_x[i2];
                    }

                    f_new = 0.5 * y;
                    y = 0.0;
                    for (i2 = 0; i2 < 10; i2++) {
                        y += 0.5 * b_lb[i2] * (lambda * b_lb[i2] - g_x[i2]);
                    }

                    rho = (f_x - f_new) / y;
                    if (rho > 0.0) {
                        for (i = 0; i < 10; i++) {
                            obj->x[i] = b_ub[i];
                        }

                        f_x = f_new;
                        y = 0.0;
                        for (i = 0; i < 10; i++) {
                            g_x[i] = 0.0;
                            for (i2 = 0; i2 < 12; i2++) {
                                g_x[i] += J_x[i2 + 12 * i] * r_x[i2];
                            }

                            nu = fabs(g_x[i]);
                            if (nu > y) {
                                y = nu;
                            }
                        }

                        if (y <= obj->gTolerance) {
                            obj->status = G_TOLERANCE;
                            exitg1 = 1;
                        } else {
                            i2 = 0;
                            for (i = 0; i < 10; i++) {
                                j = 0;
                                i3 = 0;
                                for (i4 = 0; i4 < 10; i4++) {
                                    H_x[j + i] = 0.0;
                                    for (i5 = 0; i5 < 12; i5++) {
                                        H_x[j + i] += J_x[i5 + i2] * J_x[i5 + i3];
                                    }

                                    j += 10;
                                    i3 += 12;
                                }

                                i2 += 12;
                            }

                            lambda *= fmax(0.33333333333333331, 1.0 - pow(2.0 *
                                            rho - 1.0, 3.0));
                            nu = 2.0;
                        }
                    } else {
                        lambda *= nu;
                        nu *= 2.0;
                    }
                }
            } else {
                exitg1 = 1;
            }
        } while (exitg1 == 0);
    }
}

void cassie_system_init(const cassie_system_t *b_system)
{
    CassieOutputs co;
    ProcessInputsSimulation *obj;
    int i;
    CassieOutputs_CassieOutputs(&co);
    obj = b_system->process_inputs;
    obj->isInitialized = 1;
    CassieEtherCAT_CassieEtherCAT(&obj->etherCAT);
    for (i = 0; i < 9; i++) {
        obj->inputs.data.radio[i] = 0.0;
    }

    for (i = 0; i < 10; i++) {
        obj->inputs.data.torque[i] = 0.0;
    }

    CassieOutputs_CassieOutputs(&obj->outputs);
    obj->softStart = 0.0;
    obj->softStartDuration = 5.0;
    obj->softStart = 1.0;
    SystemCore_setup(b_system->process_outputs);
    b_SystemCore_setup(b_system->controller);
}

void cassie_system_step(const cassie_system_t *b_system, ethercat_data_t *
    ethercat)
{
    double outputs_vectorNavOrientation[4];
    double c_outputs_vectorNavAngularVeloc[3];
    double c_outputs_vectorNavLinearAccele[3];
    double outputs_vectorNavMagnetometer[3];
    double outputs_vectorNavPressure;
    double outputs_vectorNavTemperature;
    double outputs_motorPosition[10];
    double outputs_motorVelocity[10];
    double outputs_jointPosition[6];
    double outputs_jointVelocity[6];
    double outputs_radio[16];
    double outputs_stateOfCharge;
    double outputs_status;
    TestController *obj;
    int i;
    int i8;
    double b_radioChannel[16];
    double motorIndex;
    double q[10];
    static const double b_q[10] = { 0.065449846949787366, 0.0,
        0.26179938779914941, -1.7540558982543013, -1.4835298641951802,
        -0.065449846949787366, 0.0, 0.26179938779914941, -1.7540558982543013,
        -1.4835298641951802 };

    static const double A[10] = { 0.27724923474893681, 0.34269908169872415,
        1.0844640137963142, 1.0582840750163993, 0.90993108859688121,
        0.27724923474893681, 0.34269908169872415, 1.0844640137963142,
        1.0582840750163993, 0.90993108859688121 };

    double leftFootTransform_position_data[3];
    double obj_data[3];
    signed char leftFootTransform_rotation_data[9];
    static const signed char iv3[9] = { 1, 0, 0, 0, 1, 0, 0, 0, 1 };

    double inputs_radio[9];
    double dq[10];
    signed char c_rightFootTransform_rotation_d[9];
    int i9;
    ProcessInputsSimulation *b_obj;
    double u_desired[10];
    double kp[10];
    double q_desired[10];
    double kd[10];
    double dq_desired[10];
    double c_rightFootTransform_position_d[3];
    InverseKinematicsFunction *c_obj;
    b_struct_T expl_temp;
    SystemCore_step(b_system->process_outputs, ethercat,
                    outputs_vectorNavOrientation,
                    c_outputs_vectorNavAngularVeloc,
                    c_outputs_vectorNavLinearAccele,
                    outputs_vectorNavMagnetometer, &outputs_vectorNavPressure,
                    &outputs_vectorNavTemperature, outputs_motorPosition,
                    outputs_motorVelocity, outputs_jointPosition,
                    outputs_jointVelocity, outputs_radio, &outputs_stateOfCharge,
                    &outputs_status);
    obj = b_system->controller;
    if (obj->isInitialized != 1) {
        b_SystemCore_setup(obj);
    }

    for (i = 0; i < 10; i++) {
        obj->inputs.data.torque[i] = 0.0;
    }

    for (i = 0; i < 4; i++) {
        obj->outputs.data.vectorNavOrientation[i] =
            outputs_vectorNavOrientation[i];
    }

    for (i = 0; i < 3; i++) {
        obj->outputs.data.vectorNavAngularVelocity[i] =
            c_outputs_vectorNavAngularVeloc[i];
    }

    for (i = 0; i < 3; i++) {
        obj->outputs.data.vectorNavLinearAcceleration[i] =
            c_outputs_vectorNavLinearAccele[i];
    }

    for (i = 0; i < 3; i++) {
        obj->outputs.data.vectorNavMagnetometer[i] =
            outputs_vectorNavMagnetometer[i];
    }

    obj->outputs.data.vectorNavPressure = outputs_vectorNavPressure;
    obj->outputs.data.vectorNavTemperature = outputs_vectorNavTemperature;
    for (i = 0; i < 10; i++) {
        obj->outputs.data.motorPosition[i] = outputs_motorPosition[i];
    }

    for (i = 0; i < 10; i++) {
        obj->outputs.data.motorVelocity[i] = outputs_motorVelocity[i];
    }

    for (i = 0; i < 6; i++) {
        obj->outputs.data.jointPosition[i] = outputs_jointPosition[i];
    }

    for (i = 0; i < 6; i++) {
        obj->outputs.data.jointVelocity[i] = outputs_jointVelocity[i];
    }

    for (i = 0; i < 16; i++) {
        obj->outputs.data.radio[i] = outputs_radio[i];
    }

    obj->outputs.data.stateOfCharge = outputs_stateOfCharge;
    obj->outputs.data.status = outputs_status;
    for (i8 = 0; i8 < 16; i8++) {
        b_radioChannel[i8] = obj->outputs.data.radio[i8];
    }

    if (b_radioChannel[9] != obj->controllerMode) {
        obj->softStart = 0.0;
    } else {
        motorIndex = obj->softStart + 0.0001;
        obj->softStart = fmax(fmin(motorIndex, 1.0), 0.0);
    }

    obj->controllerMode = b_radioChannel[9];
    switch ((int)obj->controllerMode) {
      case 0:
        motorIndex = rt_roundd(3.8 * fabs(b_radioChannel[4]));
        obj->inputs.data.torque[(int)(motorIndex + 1.0) - 1] = 20.0 *
            b_radioChannel[0];
        obj->inputs.data.torque[(int)(motorIndex + 6.0) - 1] = 20.0 *
            b_radioChannel[2];
        break;

      case 1:
        motorIndex = rt_roundd(3.8 * fabs(b_radioChannel[4]));
        memcpy(&q[0], &b_q[0], 10U * sizeof(double));
        q[(int)(motorIndex + 1.0) - 1] = b_q[(int)(motorIndex + 1.0) - 1] + A
            [(int)(motorIndex + 1.0) - 1] * b_radioChannel[0];
        q[(int)(motorIndex + 6.0) - 1] += A[(int)(motorIndex + 6.0) - 1] *
            b_radioChannel[2];
        for (i = 0; i < 10; i++) {
            obj->pdController.desiredMotorPosition[i] = q[i];
        }

        for (i = 0; i < 10; i++) {
            obj->pdController.desiredMotorVelocity[i] = 0.0;
        }

        for (i8 = 0; i8 < 10; i8++) {
            q[i8] = obj->outputs.data.motorPosition[i8];
            dq[i8] = obj->outputs.data.motorVelocity[i8];
            kp[i8] = obj->pdController.proportionalGain[i8];
            kd[i8] = obj->pdController.derivativeGain[i8];
            q_desired[i8] = obj->pdController.desiredMotorPosition[i8];
            dq_desired[i8] = obj->pdController.desiredMotorVelocity[i8];
        }

        for (i8 = 0; i8 < 10; i8++) {
            u_desired[i8] = kp[i8] * (q_desired[i8] - q[(int)
                obj->pdController.activeMotorIndex[i8] - 1]) + kd[i8] *
                (dq_desired[i8] - dq[(int)obj->pdController.activeMotorIndex[i8]
                 - 1]);
        }

        memset(&q[0], 0, 10U * sizeof(double));
        for (i8 = 0; i8 < 10; i8++) {
            q[(int)obj->pdController.activeMotorIndex[i8] - 1] = u_desired[i8];
        }

        for (i = 0; i < 10; i++) {
            obj->inputs.data.torque[i] = q[i];
        }
        break;

      case -1:
        for (i = 0; i < 3; i++) {
            leftFootTransform_position_data[i] = 0.0;
        }

        for (i8 = 0; i8 < 9; i8++) {
            leftFootTransform_rotation_data[i8] = iv3[i8];
        }

        obj_data[0] = 0.5 * b_radioChannel[0];
        obj_data[1] = 0.135 - 0.135 * b_radioChannel[1];
        obj_data[2] = -0.75 + 0.35 * b_radioChannel[6];
        for (i = 0; i < 3; i++) {
            motorIndex = 0.0;
            i8 = 0;
            for (i9 = 0; i9 < 3; i9++) {
                motorIndex += (double)leftFootTransform_rotation_data[i8 + i] *
                    obj_data[i9];
                i8 += 3;
            }

            c_rightFootTransform_position_d[i] = 0.0;
            leftFootTransform_position_data[i] += motorIndex;
        }

        for (i8 = 0; i8 < 9; i8++) {
            c_rightFootTransform_rotation_d[i8] = iv3[i8];
        }

        obj_data[0] = 0.5 * b_radioChannel[2];
        obj_data[1] = -0.135 - 0.135 * b_radioChannel[3];
        obj_data[2] = -0.75 + 0.35 * b_radioChannel[7];
        for (i8 = 0; i8 < 3; i8++) {
            motorIndex = 0.0;
            i9 = 0;
            for (i = 0; i < 3; i++) {
                motorIndex += (double)c_rightFootTransform_rotation_d[i9 + i8] *
                    obj_data[i];
                i9 += 3;
            }

            c_rightFootTransform_position_d[i8] += motorIndex;
        }

        c_obj = obj->ikSolver.problem->residualFunction;
        for (i8 = 0; i8 < 3; i8++) {
            c_obj->desiredState[i8] = leftFootTransform_position_data[i8];
        }

        for (i8 = 0; i8 < 3; i8++) {
            c_obj->desiredState[i8 + 3] = leftFootTransform_rotation_data[i8];
        }

        for (i8 = 0; i8 < 3; i8++) {
            c_obj->desiredState[i8 + 6] = c_rightFootTransform_position_d[i8];
        }

        for (i8 = 0; i8 < 3; i8++) {
            c_obj->desiredState[i8 + 9] = c_rightFootTransform_rotation_d[i8];
        }

        c_NonlinearLeastSquaresSolver_s(&obj->ikSolver);
        for (i8 = 0; i8 < 10; i8++) {
            q[i8] = obj->ikSolver.x[i8];
        }

        for (i = 0; i < 10; i++) {
            obj->pdController.desiredMotorPosition[i] = q[i];
        }

        for (i = 0; i < 10; i++) {
            obj->pdController.desiredMotorVelocity[i] = 0.0;
        }

        for (i8 = 0; i8 < 10; i8++) {
            q[i8] = obj->outputs.data.motorPosition[i8];
            dq[i8] = obj->outputs.data.motorVelocity[i8];
            kp[i8] = obj->pdController.proportionalGain[i8];
            kd[i8] = obj->pdController.derivativeGain[i8];
            q_desired[i8] = obj->pdController.desiredMotorPosition[i8];
            dq_desired[i8] = obj->pdController.desiredMotorVelocity[i8];
        }

        for (i8 = 0; i8 < 10; i8++) {
            u_desired[i8] = kp[i8] * (q_desired[i8] - q[(int)
                obj->pdController.activeMotorIndex[i8] - 1]) + kd[i8] *
                (dq_desired[i8] - dq[(int)obj->pdController.activeMotorIndex[i8]
                 - 1]);
        }

        memset(&q[0], 0, 10U * sizeof(double));
        for (i8 = 0; i8 < 10; i8++) {
            q[(int)obj->pdController.activeMotorIndex[i8] - 1] = u_desired[i8];
        }

        for (i = 0; i < 10; i++) {
            obj->inputs.data.torque[i] = q[i];
        }
        break;

      default:
        for (i = 0; i < 10; i++) {
            obj->inputs.data.torque[i] = 0.0;
        }
        break;
    }

    for (i8 = 0; i8 < 10; i8++) {
        q[i8] = obj->inputs.data.torque[i8];
    }

    motorIndex = obj->softStart;
    for (i = 0; i < 10; i++) {
        obj->inputs.data.torque[i] = motorIndex * q[i];
    }

    for (i8 = 0; i8 < 9; i8++) {
        inputs_radio[i8] = obj->inputs.data.radio[i8];
    }

    for (i8 = 0; i8 < 10; i8++) {
        q[i8] = obj->inputs.data.torque[i8];
    }

    b_obj = b_system->process_inputs;
    if (b_obj->isInitialized != 1) {
        b_obj->isInitialized = 1;
        CassieEtherCAT_CassieEtherCAT(&b_obj->etherCAT);
        for (i = 0; i < 9; i++) {
            b_obj->inputs.data.radio[i] = 0.0;
        }

        for (i = 0; i < 10; i++) {
            b_obj->inputs.data.torque[i] = 0.0;
        }

        CassieOutputs_CassieOutputs(&b_obj->outputs);
        b_obj->softStart = 0.0;
        b_obj->softStartDuration = 5.0;
        b_obj->softStart = 1.0;
    }

    ethercat->status[1] = 8.0;
    ethercat->pelvisMedulla.inputs.radioChannel[8] = 819.0;
    for (i = 0; i < 12; i++) {
        ethercat->batteryMedulla.inputs.voltage[i] = 4.3;
    }

    ethercat->batteryMedulla.inputs.current = 0.0;
    expl_temp.status = outputs_status;
    expl_temp.stateOfCharge = outputs_stateOfCharge;
    memcpy(&expl_temp.radio[0], &outputs_radio[0], sizeof(double) << 4);
    for (i = 0; i < 6; i++) {
        expl_temp.jointVelocity[i] = outputs_jointVelocity[i];
        expl_temp.jointPosition[i] = outputs_jointPosition[i];
    }

    memcpy(&expl_temp.motorVelocity[0], &outputs_motorVelocity[0], 10U * sizeof
           (double));
    memcpy(&expl_temp.motorPosition[0], &outputs_motorPosition[0], 10U * sizeof
           (double));
    expl_temp.vectorNavTemperature = outputs_vectorNavTemperature;
    expl_temp.vectorNavPressure = outputs_vectorNavPressure;
    for (i = 0; i < 3; i++) {
        expl_temp.vectorNavMagnetometer[i] = outputs_vectorNavMagnetometer[i];
        expl_temp.vectorNavLinearAcceleration[i] =
            c_outputs_vectorNavLinearAccele[i];
        expl_temp.vectorNavAngularVelocity[i] =
            c_outputs_vectorNavAngularVeloc[i];
    }

    for (i = 0; i < 4; i++) {
        expl_temp.vectorNavOrientation[i] = outputs_vectorNavOrientation[i];
    }

    ProcessInputs_stepImpl(b_obj, ethercat, inputs_radio, q, &expl_temp);
    *ethercat = b_obj->etherCAT.data;
}

static void merge(int idx[5], double b_x[5], int offset, int np, int nq, int
                  iwork[5], double xwork[5])
{
    int n;
    int qend;
    int p;
    int iout;
    int exitg1;
    if ((np == 0) || (nq == 0)) {
    } else {
        n = np + nq;
        for (qend = 0; qend + 1 <= n; qend++) {
            iwork[qend] = idx[offset + qend];
            xwork[qend] = b_x[offset + qend];
        }

        p = 0;
        n = np;
        qend = np + nq;
        iout = offset - 1;
        do {
            exitg1 = 0;
            iout++;
            if (xwork[p] >= xwork[n]) {
                idx[iout] = iwork[p];
                b_x[iout] = xwork[p];
                if (p + 1 < np) {
                    p++;
                } else {
                    exitg1 = 1;
                }
            } else {
                idx[iout] = iwork[n];
                b_x[iout] = xwork[n];
                if (n + 1 < qend) {
                    n++;
                } else {
                    n = iout - p;
                    while (p + 1 <= np) {
                        idx[(n + p) + 1] = iwork[p];
                        b_x[(n + p) + 1] = xwork[p];
                        p++;
                    }

                    exitg1 = 1;
                }
            }
        } while (exitg1 == 0);
    }
}

static void mldivide(const double A[100], double B[10])
{
    double b_A[100];
    int i10;
    int j;
    signed char ipiv[10];
    int k;
    int c;
    int kAcol;
    int ix;
    double smax;
    int jy;
    double s;
    int ijA;
    memcpy(&b_A[0], &A[0], 100U * sizeof(double));
    for (i10 = 0; i10 < 10; i10++) {
        ipiv[i10] = (signed char)(1 + i10);
    }

    for (j = 0; j < 9; j++) {
        c = j * 11;
        kAcol = 0;
        ix = c;
        smax = fabs(b_A[c]);
        for (k = 2; k <= 10 - j; k++) {
            ix++;
            s = fabs(b_A[ix]);
            if (s > smax) {
                kAcol = k - 1;
                smax = s;
            }
        }

        if (b_A[c + kAcol] != 0.0) {
            if (kAcol != 0) {
                ipiv[j] = (signed char)((j + kAcol) + 1);
                ix = j;
                kAcol += j;
                for (k = 0; k < 10; k++) {
                    smax = b_A[ix];
                    b_A[ix] = b_A[kAcol];
                    b_A[kAcol] = smax;
                    ix += 10;
                    kAcol += 10;
                }
            }

            i10 = (c - j) + 10;
            for (jy = c + 1; jy + 1 <= i10; jy++) {
                b_A[jy] /= b_A[c];
            }
        }

        kAcol = c;
        jy = c + 10;
        for (k = 1; k <= 9 - j; k++) {
            smax = b_A[jy];
            if (b_A[jy] != 0.0) {
                ix = c + 1;
                i10 = (kAcol - j) + 20;
                for (ijA = 11 + kAcol; ijA + 1 <= i10; ijA++) {
                    b_A[ijA] += b_A[ix] * -smax;
                    ix++;
                }
            }

            jy += 10;
            kAcol += 10;
        }

        if (ipiv[j] != j + 1) {
            smax = B[j];
            B[j] = B[ipiv[j] - 1];
            B[ipiv[j] - 1] = smax;
        }
    }

    for (k = 0; k < 10; k++) {
        kAcol = 10 * k;
        if (B[k] != 0.0) {
            for (jy = k + 1; jy + 1 < 11; jy++) {
                B[jy] -= B[k] * b_A[jy + kAcol];
            }
        }
    }

    for (k = 9; k >= 0; k += -1) {
        kAcol = 10 * k;
        if (B[k] != 0.0) {
            B[k] /= b_A[k + kAcol];
            for (jy = 0; jy + 1 <= k; jy++) {
                B[jy] -= B[k] * b_A[jy + kAcol];
            }
        }
    }
}

static double norm(const double b_x[10])
{
    double y;
    double scale;
    int k;
    double absxk;
    double t;
    y = 0.0;
    scale = 2.2250738585072014E-308;
    for (k = 0; k < 10; k++) {
        absxk = fabs(b_x[k]);
        if (absxk > scale) {
            t = scale / absxk;
            y = 1.0 + y * t * t;
            scale = absxk;
        } else {
            t = absxk / scale;
            y += t * t;
        }
    }

    return scale * sqrt(y);
}

static double rt_roundd(double u)
{
    double y;
    if (fabs(u) < 4.503599627370496E+15) {
        if (u >= 0.5) {
            y = floor(u + 0.5);
        } else if (u > -0.5) {
            y = 0.0;
        } else {
            y = ceil(u - 0.5);
        }
    } else {
        y = u;
    }

    return y;
}

static void sort(double b_x[5])
{
    b_sort(b_x);
}

cassie_system_t* cassie_system_alloc(void)
{
    cassie_system_t *system = (cassie_system_t*) malloc(sizeof (cassie_system_t));
    system->process_inputs = (ProcessInputsSimulation*) malloc(sizeof (ProcessInputsSimulation));
    system->process_outputs = (ProcessOutputsSimulation*)malloc(sizeof (ProcessOutputsSimulation));
    system->controller = (TestController*)malloc(sizeof (TestController));
    return system;
}

void cassie_system_free(cassie_system_t *system)
{
    free(system->process_inputs);
    free(system->process_outputs);
    free(system->controller);
    free(system);
}

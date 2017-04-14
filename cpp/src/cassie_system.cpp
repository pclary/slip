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

#ifndef typedef_Quaternion_1
#define typedef_Quaternion_1

typedef struct {
    double data[4];
} Quaternion_1;

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

#ifndef typedef_Rotation3d
#define typedef_Rotation3d

typedef struct {
    double data[9];
} Rotation3d;

#endif

#ifndef typedef_Vector3d
#define typedef_Vector3d

typedef struct {
    double data[3];
} Vector3d;

#endif

#ifndef typedef_Transform3d
#define typedef_Transform3d

typedef struct {
    Vector3d position;
    Rotation3d rotation;
} Transform3d;

#endif

#ifndef typedef_WorldFrame
#define typedef_WorldFrame

typedef struct {
    Transform3d relativeTransform;
    double relativeSpatialVelocity[6];
} WorldFrame;

#endif

#ifndef typedef_Frame
#define typedef_Frame

typedef struct {
    WorldFrame *parentFrame;
    Transform3d relativeTransform;
    double relativeSpatialVelocity[6];
} Frame;

#endif

#ifndef typedef_Frame_1
#define typedef_Frame_1

typedef struct {
    Frame *parentFrame;
    Transform3d relativeTransform;
    double relativeSpatialVelocity[6];
} Frame_1;

#endif

#ifndef typedef_Frame_2
#define typedef_Frame_2

typedef struct {
    char name[6];
    Frame_1 *parentFrame;
    Transform3d relativeTransform;
    double relativeSpatialVelocity[6];
} Frame_2;

#endif

#ifndef typedef_Frame_3
#define typedef_Frame_3

typedef struct {
    char name[14];
    Frame_2 *parentFrame;
    Transform3d relativeTransform;
    double relativeSpatialVelocity[6];
} Frame_3;

#endif

#ifndef typedef_Frame_4
#define typedef_Frame_4

typedef struct {
    Frame_2 *parentFrame;
    Transform3d relativeTransform;
} Frame_4;

#endif

#ifndef typedef_LowPassFilter
#define typedef_LowPassFilter

typedef struct {
    double sampleTime;
    double smoothingFactor;
    Vector3d signalValue;
    Vector3d signalRateOfChange;
} LowPassFilter;

#endif

#ifndef typedef_ZUpFrame
#define typedef_ZUpFrame

typedef struct {
    char name[11];
    Frame_2 *parentFrame;
} ZUpFrame;

#endif

#ifndef typedef_CassieRobot
#define typedef_CassieRobot

typedef struct {
    CassieInputs inputs;
    CassieOutputs outputs;
    WorldFrame worldFrame;
    Frame northEastDownFrame;
    Frame_1 vectorNavFrame;
    Frame_2 pelvisFrame;
    ZUpFrame zUpPelvisFrame;
    Frame_3 centerOfMassFrame;
    Frame_4 leftFootFrame;
    Frame_4 rightFootFrame;
    Vector3d leftFootForce;
    Vector3d rightFootForce;
    Vector3d centerOfMassVelocity;
    LowPassFilter leftFootFilter;
    LowPassFilter rightFootFilter;
} CassieRobot;

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

#ifndef typedef_Frame_5
#define typedef_Frame_5

typedef struct {
    char name[14];
    WorldFrame *parentFrame;
    Transform3d relativeTransform;
    double relativeSpatialVelocity[6];
} Frame_5;

#endif

#ifndef typedef_Frame_6
#define typedef_Frame_6

typedef struct {
    char name[17];
    WorldFrame *parentFrame;
    Transform3d relativeTransform;
    double relativeSpatialVelocity[6];
} Frame_6;

#endif

#ifndef typedef_Frame_7
#define typedef_Frame_7

typedef struct {
    char name[18];
    WorldFrame *parentFrame;
    Transform3d relativeTransform;
    double relativeSpatialVelocity[6];
} Frame_7;

#endif

#ifndef typedef_Frame_8
#define typedef_Frame_8

typedef struct {
    WorldFrame *parentFrame;
    Transform3d relativeTransform;
    double relativeSpatialVelocity[6];
} Frame_8;

#endif

#ifndef typedef_SmoothRateLimiter
#define typedef_SmoothRateLimiter

typedef struct {
    double sampleTime;
    double rateLimit;
    double signalValue;
    double signalRateOfChange;
} SmoothRateLimiter;

#endif

#ifndef typedef_ZUpFrame_1
#define typedef_ZUpFrame_1

typedef struct {
    char name[17];
    Frame_8 *parentFrame;
} ZUpFrame_1;

#endif

#ifndef typedef_StandingController
#define typedef_StandingController

typedef struct {
    CassieRobot *robot;
    PDController *pdController;
    NonlinearLeastSquaresSolver *ikSolver;
    Frame_5 desiredPelvisFrame;
    Frame_6 desiredLeftFootFrame;
    Frame_7 desiredRightFootFrame;
    Frame_8 averageFootFrame;
    ZUpFrame_1 centerOfSupportFrame;
    SmoothRateLimiter legLengthFilter;
    SmoothRateLimiter pelvisYFilter;
    SmoothRateLimiter pelvisYawFilter;
    SmoothRateLimiter pelvisPitchFilter;
} StandingController;

#endif

#ifndef typedef_LowPassFilter_1
#define typedef_LowPassFilter_1

typedef struct {
    double sampleTime;
    double smoothingFactor;
    double signalValue;
    double signalRateOfChange;
} LowPassFilter_1;

#endif

#ifndef typedef_RobotSide
#define typedef_RobotSide

typedef signed char RobotSide;

#endif

#ifndef RobotSide_constants
#define RobotSide_constants

#define LEFT                           ((RobotSide)1)
#define RIGHT                          ((RobotSide)-1)
#endif

#ifndef typedef_SteppingController
#define typedef_SteppingController

typedef struct {
    CassieRobot *robot;
    PDController *pdController;
    NonlinearLeastSquaresSolver *ikSolver;
    Frame_5 desiredPelvisFrame;
    Frame_7 desiredLeftFootFrame;
    Frame_7 desiredRightFootFrame;
    RobotSide stanceLeg;
    double stanceTime;
    double stepInterval;
    SmoothRateLimiter legLengthFilter;
    LowPassFilter_1 xVelocityFilter;
    LowPassFilter_1 yVelocityFilter;
    LowPassFilter_1 yawVelocityFilter;
    double xIntegralError;
    double yIntegralError;
    Vector3d initialPelvisVelocity;
    Transform3d initialStancePelvisTransform;
    Transform3d initialSwingPelvisTransform;
    Transform3d initialStanceFootTransform;
    double c_initialStanceFootSpatialVeloc[6];
    Transform3d initialSwingFootTransform;
    double initialSwingFootSpatialVelocity[6];
    double stand;
} SteppingController;

#endif

#ifndef typedef_CassieController
#define typedef_CassieController

typedef struct {
    int isInitialized;
    CassieRobot robot;
    NonlinearLeastSquaresSolver ikSolver;
    NonlinearLeastSquaresProblem ikProblem;
    InverseKinematicsFunction ikFunction;
    PDController pdController;
    StandingController standingController;
    SteppingController steppingController;
    double controllerMode;
} CassieController;

#endif

#ifndef typedef_CassieEtherCAT
#define typedef_CassieEtherCAT

typedef struct {
    ethercat_data_t data;
} CassieEtherCAT;

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

#ifndef typedef_RadioSpoofer
#define typedef_RadioSpoofer

typedef struct {
    int isInitialized;
    CassieOutputs outputs;
    double timer;
} RadioSpoofer;

#endif

#ifndef typedef_cassie_system_t
#define typedef_cassie_system_t

struct cassie_system_t {
    ProcessInputsSimulation *process_inputs;
    ProcessOutputsSimulation *process_outputs;
    RadioSpoofer *radio_spoofer;
    CassieController *controller;
};

#endif

static void CassieController_setupImpl(CassieController *obj);
static void CassieController_stepImpl(CassieController *obj, const double
    outputs_vectorNavOrientation[4], const double
    c_outputs_vectorNavAngularVeloc[3], const double
    c_outputs_vectorNavLinearAccele[3], const double
    outputs_vectorNavMagnetometer[3], double outputs_vectorNavPressure, double
    outputs_vectorNavTemperature, const double outputs_motorPosition[10], const
    double outputs_motorVelocity[10], const double outputs_jointPosition[6],
    const double outputs_jointVelocity[6], const double outputs_radio[16],
    double outputs_stateOfCharge, double outputs_status, double inputs_radio[9],
    double inputs_torque[10]);
static CassieEtherCAT *CassieEtherCAT_CassieEtherCAT(CassieEtherCAT *obj);
static CassieOutputs *CassieOutputs_CassieOutputs(CassieOutputs *obj);
static CassieRobot *CassieRobot_CassieRobot(CassieRobot *obj);
static void CassieRobot_update(CassieRobot *obj);
static Frame_5 *Frame_Frame(Frame_5 *obj, WorldFrame *b_parentFrame);
static void Frame_getSpatialVelocity(Frame_3 *obj, ZUpFrame *relativeTo,
    ZUpFrame *inCoordinatesOf, double spatialVelocity[6]);
static void Frame_getTransform(Frame_2 *obj, ZUpFrame *withRespectTo, double
    transform_position_data[3], double transform_rotation_data[9]);
static void Frame_getWorldSpatialVelocity(Frame_2 *obj, double
    worldSpatialVelocity[6]);
static void Frame_getWorldTransform(ZUpFrame *obj, double
    worldTransform_position_data[3], Rotation3d *worldTransform_rotation);
static void PDController_evaluate(const PDController *obj, const double q[10],
    const double dq[10], double u[10]);
static void PDController_setDerivativeGain(PDController *obj, const double
    b_derivativeGain[10]);
static void ProcessInputs_stepImpl(ProcessInputsSimulation *obj, const
    ethercat_data_t *b_etherCAT, const double inputs_radio[9], const double
    inputs_torque[10], const b_struct_T *b_outputs);
static void ProcessOutputs_setStateOfCharge(ProcessOutputsSimulation *obj);
static void Quaternion_getRotation(const double obj_data[4], double
    rotation_data[9]);
static void Quaternion_normalize(Quaternion_1 *obj);
static void Rotation3d_getQuaternion(const double obj_data[9], double q_data[4]);
static void Rotation3d_removePitchAndRoll(Rotation3d *obj);
static void Rotation3d_rotZ(Rotation3d *obj, double rz);
static void SmoothRateLimiter_updateSignal(SmoothRateLimiter *obj, double value);
static void StandingController_eval(StandingController *obj, double tau[10]);
static void StandingController_setPelvisYaw(StandingController *obj, double
    pelvisYaw);
static void SteppingController_eval(SteppingController *obj, double tau[10]);
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
static void Transform3d_mldivide(const double a_position_data[3], const double
    a_rotation_data[9], const double b_position_data[3], const double
    b_rotation_data[9], double c_position_data[3], double c_rotation_data[9]);
static void ZUpFrame_getRelativeTransform(const ZUpFrame_1 *obj, double
    relativeTransform_position_data[3], double relativeTransform_rotation_data[9]);
static double __anon_fcn(double b_x, double x1, double x2);
static void b_CassieEtherCAT_CassieEtherCAT(CassieEtherCAT **obj);
static void b_CassieOutputs_CassieOutputs(CassieOutputs **obj);
static void b_CassieRobot_CassieRobot(CassieRobot **obj);
static Frame_7 *b_Frame_Frame(Frame_7 *obj, WorldFrame *b_parentFrame);
static void b_Frame_getSpatialVelocity(ZUpFrame_1 *obj, ZUpFrame *relativeTo,
    ZUpFrame *inCoordinatesOf, double spatialVelocity[6]);
static void b_Frame_getTransform(Frame_4 *obj, WorldFrame *withRespectTo,
    ZUpFrame *inCoordinatesOf, double transform_position_data[3], double
    transform_rotation_data[9]);
static void b_Frame_getWorldSpatialVelocity(ZUpFrame *obj, double
    worldSpatialVelocity[6]);
static void b_Frame_getWorldTransform(Frame_2 *obj, Transform3d *worldTransform);
static int b_bsearch(const double b_x[64], double xi);
static double b_clamp(double a);
static void b_cubicInterpolation(double t, const double ts[3], const double qs[3],
    const double dqs[3], double *q, double *dq, double *ddqt);
static int b_ixamax(int n, const double b_x[5], int ix0);
static void b_mldivide(const double A[30], const double B[6], double Y[5]);
static void b_sort(double b_x[5]);
static void b_xgeqp3(double A[30], double tau[5], int jpvt[5]);
static double b_xnrm2(int n, const double b_x[6], int ix0);
static void b_xswap(double b_x[30], int ix0, int iy0);
static void c_Frame_Frame(Frame_5 **obj, WorldFrame *b_parentFrame);
static void c_Frame_getSpatialVelocity(Frame_7 *obj, Frame_5 *relativeTo,
    Frame_5 *inCoordinatesOf, double spatialVelocity[6]);
static void c_Frame_getTransform(Frame_3 *obj, ZUpFrame *withRespectTo, double
    transform_position_data[3], double transform_rotation_data[9]);
static void c_Frame_getWorldTransform(Frame_4 *obj, double
    worldTransform_position_data[3], double worldTransform_rotation_data[9]);
static InverseKinematicsFunction *c_InverseKinematicsFunction_Inv
    (InverseKinematicsFunction *obj);
static void c_InverseKinematicsFunction_eva(const InverseKinematicsFunction *obj,
    const double q[10], double f[12], double J[120]);
static void c_NonlinearLeastSquaresSolver_p(const InverseKinematicsFunction *r,
    double b_x[10], const double lb[10], const double ub[10], double r_x[12],
    double J_x[120]);
static void c_NonlinearLeastSquaresSolver_s(NonlinearLeastSquaresSolver *obj);
static void c_PDController_setProportionalG(PDController *obj, const double
    b_proportionalGain[10]);
static void c_StandingController_setPelvisP(StandingController *obj, double
    pelvisPitch);
static void c_StandingController_updateDesi(StandingController *obj, double
    pelvisPitch);
static Frame_7 *c_SteppingController_getDesired(SteppingController *obj);
static Frame_4 *c_SteppingController_getStanceF(const SteppingController *obj);
static void c_SteppingController_updateDesi(SteppingController *obj);
static double c_clamp(double a);
static void c_mldivide(const double A[16], double B[4]);
static double c_xnrm2(const double b_x[30], int ix0);


static double clamp(double a, double limit_1, double limit_2);
static RobotSide convert_to_enum_RobotSide(signed char u);
static void cubicInterpolation(double t, const double ts[2], const double qs[2],
    const double dqs[2], double *q, double *dq, double *ddqt);
static void d_Frame_Frame(Frame_7 **obj, WorldFrame *b_parentFrame);
static void d_Frame_getSpatialVelocity(Frame_2 *obj, ZUpFrame *relativeTo,
    Frame_2 *inCoordinatesOf, double spatialVelocity[6]);
static void d_Frame_getTransform(ZUpFrame_1 *obj, ZUpFrame *withRespectTo,
    double transform_position_data[3], double transform_rotation_data[9]);
static void d_Frame_getWorldTransform(ZUpFrame_1 *obj, double
    worldTransform_position_data[3], double worldTransform_rotation_data[9]);
static void d_InverseKinematicsFunction_Inv(InverseKinematicsFunction **obj);
static void d_mldivide(const double A[100], double B[10]);
static double d_xnrm2(int n, const double b_x[30], int ix0);
static void e_Frame_getTransform(Frame_4 *obj, ZUpFrame *withRespectTo, double
    transform_position_data[3], double transform_rotation_data[9]);
static void f_Frame_getTransform(Frame_6 *obj, Frame_5 *withRespectTo, double
    transform_position_data[3], double transform_rotation_data[9]);
static void g_Frame_getTransform(Frame_7 *obj, Frame_5 *withRespectTo, double
    transform_position_data[3], double transform_rotation_data[9]);
static void generatedCenterOfMassJacobian(const double in1[10], double J[30]);
static void generatedCenterOfMassPosition(const double in1[10], double p[3]);
static void generatedGravityVector(const double in1[10], double G[10]);
static void generatedLeftFootJacobian(const double in1[5], const double in2[2],
    double J[42]);
static void generatedLeftFootTransform(const double in1[5], const double in2[2],
    double T[16]);
static void generatedRightFootJacobian(const double in1[5], const double in2[2],
    double J[42]);
static int ixamax(int n, const double b_x[3], int ix0);
static void merge(int idx[5], double b_x[5], int offset, int np, int nq, int
                  iwork[5], double xwork[5]);
static void mldivide(const double A[6], const double B[2], double Y[3]);
static double norm(const double b_x[10]);
static double rt_hypotd(double u0, double u1);
static double rt_roundd(double u);
static void sort(double b_x[5]);
static void sort3(int i1, double v1, int i2, double v2, int i3, double v3, int
                  *b_j1, int *j2);
static void xgeqp3(double A[6], double tau[2], int jpvt[3]);
static double xnrm2(const double b_x[6], int ix0);
static void xswap(double b_x[6], int ix0, int iy0);
static void CassieController_setupImpl(CassieController *obj)
{
    NonlinearLeastSquaresProblem *b_obj;
    InverseKinematicsFunction *r;
    int i;
    static const double lb[10] = { -0.26179938779914941, -0.39269908169872414,
        -0.87266462599716477, -2.8623399732707004, -2.4434609527920612,
        -0.39269908169872414, -0.39269908169872414, -0.87266462599716477,
        -2.8623399732707004, -2.4434609527920612 };

    static const double ub[10] = { 0.39269908169872414, 0.39269908169872414,
        1.3962634015954636, -0.64577182323790194, -0.52359877559829882,
        0.26179938779914941, 0.39269908169872414, 1.3962634015954636,
        -0.64577182323790194, -0.52359877559829882 };

    NonlinearLeastSquaresSolver *c_obj;
    PDController *d_obj;
    StandingController *e_obj;
    CassieRobot *b_robot;
    WorldFrame *b_parentFrame;
    static const char b_name[17] = { 'D', 'e', 's', 'i', 'r', 'e', 'd', ' ', 'L',
        'e', 'f', 't', ' ', 'F', 'o', 'o', 't' };

    signed char obj_rotation_data[9];
    int k;
    static const char c_name[17] = { 'C', 'e', 'n', 't', 'e', 'r', ' ', 'O', 'f',
        ' ', 'S', 'u', 'p', 'p', 'o', 'r', 't' };

    SteppingController *f_obj;
    static const char d_name[18] = { 'D', 'e', 's', 'i', 'r', 'e', 'd', ' ', 'L',
        'e', 'f', 't', ' ', 'F', 'o', 'o', 't', ' ' };

    CassieRobot_CassieRobot(&obj->robot);
    c_InverseKinematicsFunction_Inv(&obj->ikFunction);
    b_obj = &obj->ikProblem;
    r = &obj->ikFunction;
    for (i = 0; i < 10; i++) {
        b_obj->lowerBound[i] = lb[i];
    }

    for (i = 0; i < 10; i++) {
        b_obj->upperBound[i] = ub[i];
    }

    b_obj->residualFunction = r;
    c_obj = &obj->ikSolver;
    b_obj = &obj->ikProblem;
    c_obj->problem = b_obj;
    for (i = 0; i < 10; i++) {
        c_obj->x[i] = 0.0;
    }

    c_obj->levenbergMarquardtParameter = 0.001;
    c_obj->iterationLimit = 1000.0;
    c_obj->xTolerance = 1.4901161193847656E-8;
    c_obj->gTolerance = 1.4901161193847656E-8;
    c_obj->status = ITERATE;
    c_obj = &obj->ikSolver;
    c_obj->iterationLimit = 10.0;
    c_obj = &obj->ikSolver;
    c_obj->gTolerance = 2.2204460492503131E-16;
    c_obj = &obj->ikSolver;
    c_obj->xTolerance = 2.2204460492503131E-16;
    d_obj = &obj->pdController;
    for (i = 0; i < 10; i++) {
        d_obj->activeMotorIndex[i] = 1.0 + (double)i;
    }

    for (i = 0; i < 10; i++) {
        d_obj->proportionalGain[i] = 0.0;
    }

    for (i = 0; i < 10; i++) {
        d_obj->derivativeGain[i] = 0.0;
    }

    for (i = 0; i < 10; i++) {
        d_obj->desiredMotorPosition[i] = 0.0;
    }

    for (i = 0; i < 10; i++) {
        d_obj->desiredMotorVelocity[i] = 0.0;
    }

    e_obj = &obj->standingController;
    b_robot = &obj->robot;
    d_obj = &obj->pdController;
    c_obj = &obj->ikSolver;
    e_obj->robot = b_robot;
    e_obj->pdController = d_obj;
    e_obj->ikSolver = c_obj;
    Frame_Frame(&e_obj->desiredPelvisFrame, &e_obj->robot->worldFrame);
    b_parentFrame = &e_obj->robot->worldFrame;
    for (i = 0; i < 17; i++) {
        e_obj->desiredLeftFootFrame.name[i] = b_name[i];
    }

    e_obj->desiredLeftFootFrame.parentFrame = b_parentFrame;
    for (i = 0; i < 9; i++) {
        obj_rotation_data[i] = 0;
    }

    i = 0;
    for (k = 0; k < 3; k++) {
        obj_rotation_data[i] = 1;
        i += 4;
    }

    for (i = 0; i < 3; i++) {
        e_obj->desiredLeftFootFrame.relativeTransform.position.data[i] = 0.0;
    }

    for (i = 0; i < 9; i++) {
        e_obj->desiredLeftFootFrame.relativeTransform.rotation.data[i] =
            obj_rotation_data[i];
    }

    for (i = 0; i < 6; i++) {
        e_obj->desiredLeftFootFrame.relativeSpatialVelocity[i] = 0.0;
    }

    b_Frame_Frame(&e_obj->desiredRightFootFrame, &e_obj->robot->worldFrame);
    b_parentFrame = &e_obj->robot->worldFrame;
    e_obj->averageFootFrame.parentFrame = b_parentFrame;
    for (i = 0; i < 9; i++) {
        obj_rotation_data[i] = 0;
    }

    i = 0;
    for (k = 0; k < 3; k++) {
        obj_rotation_data[i] = 1;
        i += 4;
    }

    for (i = 0; i < 3; i++) {
        e_obj->averageFootFrame.relativeTransform.position.data[i] = 0.0;
    }

    for (i = 0; i < 9; i++) {
        e_obj->averageFootFrame.relativeTransform.rotation.data[i] =
            obj_rotation_data[i];
    }

    for (i = 0; i < 6; i++) {
        e_obj->averageFootFrame.relativeSpatialVelocity[i] = 0.0;
    }

    for (i = 0; i < 17; i++) {
        e_obj->centerOfSupportFrame.name[i] = c_name[i];
    }

    e_obj->centerOfSupportFrame.parentFrame = &e_obj->averageFootFrame;
    e_obj->legLengthFilter.sampleTime = 0.001;
    e_obj->legLengthFilter.rateLimit = 1.0;
    e_obj->legLengthFilter.signalValue = 1.0;
    e_obj->legLengthFilter.signalRateOfChange = 0.0;
    e_obj->legLengthFilter.sampleTime = 0.0005;
    e_obj->legLengthFilter.rateLimit = 0.4;
    e_obj->pelvisYFilter.sampleTime = 0.001;
    e_obj->pelvisYFilter.rateLimit = 1.0;
    e_obj->pelvisYFilter.signalValue = 0.0;
    e_obj->pelvisYFilter.signalRateOfChange = 0.0;
    e_obj->pelvisYFilter.sampleTime = 0.0005;
    e_obj->pelvisYFilter.rateLimit = 1.0;
    e_obj->pelvisYawFilter.sampleTime = 0.001;
    e_obj->pelvisYawFilter.rateLimit = 1.0;
    e_obj->pelvisYawFilter.signalValue = 0.0;
    e_obj->pelvisYawFilter.signalRateOfChange = 0.0;
    e_obj->pelvisYawFilter.sampleTime = 0.0005;
    e_obj->pelvisYawFilter.rateLimit = 1.0;
    e_obj->pelvisPitchFilter.sampleTime = 0.001;
    e_obj->pelvisPitchFilter.rateLimit = 1.0;
    e_obj->pelvisPitchFilter.signalValue = 0.0;
    e_obj->pelvisPitchFilter.signalRateOfChange = 0.0;
    e_obj->pelvisPitchFilter.sampleTime = 0.0005;
    e_obj->pelvisPitchFilter.rateLimit = 1.0;
    f_obj = &obj->steppingController;
    b_robot = &obj->robot;
    d_obj = &obj->pdController;
    c_obj = &obj->ikSolver;
    f_obj->robot = b_robot;
    f_obj->pdController = d_obj;
    f_obj->ikSolver = c_obj;
    Frame_Frame(&f_obj->desiredPelvisFrame, &f_obj->robot->worldFrame);
    b_parentFrame = &f_obj->robot->worldFrame;
    for (i = 0; i < 18; i++) {
        f_obj->desiredLeftFootFrame.name[i] = d_name[i];
    }

    f_obj->desiredLeftFootFrame.parentFrame = b_parentFrame;
    for (i = 0; i < 9; i++) {
        obj_rotation_data[i] = 0;
    }

    i = 0;
    for (k = 0; k < 3; k++) {
        obj_rotation_data[i] = 1;
        i += 4;
    }

    for (i = 0; i < 3; i++) {
        f_obj->desiredLeftFootFrame.relativeTransform.position.data[i] = 0.0;
    }

    for (i = 0; i < 9; i++) {
        f_obj->desiredLeftFootFrame.relativeTransform.rotation.data[i] =
            obj_rotation_data[i];
    }

    for (i = 0; i < 6; i++) {
        f_obj->desiredLeftFootFrame.relativeSpatialVelocity[i] = 0.0;
    }

    b_Frame_Frame(&f_obj->desiredRightFootFrame, &f_obj->robot->worldFrame);
    f_obj->stanceLeg = LEFT;
    f_obj->stanceTime = 0.0;
    f_obj->stepInterval = 0.38;
    f_obj->legLengthFilter.sampleTime = 0.001;
    f_obj->legLengthFilter.rateLimit = 1.0;
    f_obj->legLengthFilter.signalValue = 1.0;
    f_obj->legLengthFilter.signalRateOfChange = 0.0;
    f_obj->legLengthFilter.sampleTime = 0.0005;
    f_obj->legLengthFilter.rateLimit = 0.4;
    f_obj->xVelocityFilter.sampleTime = 0.001;
    f_obj->xVelocityFilter.smoothingFactor = 0.5;
    f_obj->xVelocityFilter.signalValue = 0.0;
    f_obj->xVelocityFilter.signalRateOfChange = 0.0;
    f_obj->xVelocityFilter.sampleTime = 0.0005;
    f_obj->xVelocityFilter.smoothingFactor = 0.0005;
    f_obj->yVelocityFilter.sampleTime = 0.001;
    f_obj->yVelocityFilter.smoothingFactor = 0.5;
    f_obj->yVelocityFilter.signalValue = 0.0;
    f_obj->yVelocityFilter.signalRateOfChange = 0.0;
    f_obj->yVelocityFilter.sampleTime = 0.0005;
    f_obj->yVelocityFilter.smoothingFactor = 0.002;
    f_obj->yawVelocityFilter.sampleTime = 0.001;
    f_obj->yawVelocityFilter.smoothingFactor = 0.5;
    f_obj->yawVelocityFilter.signalValue = 0.0;
    f_obj->yawVelocityFilter.signalRateOfChange = 0.0;
    f_obj->yawVelocityFilter.sampleTime = 0.0005;
    f_obj->yawVelocityFilter.smoothingFactor = 0.002;
    f_obj->xIntegralError = 0.0;
    f_obj->yIntegralError = 0.0;
    for (i = 0; i < 3; i++) {
        f_obj->initialPelvisVelocity.data[i] = 0.0;
    }

    for (i = 0; i < 9; i++) {
        obj_rotation_data[i] = 0;
    }

    i = 0;
    for (k = 0; k < 3; k++) {
        obj_rotation_data[i] = 1;
        i += 4;
    }

    for (i = 0; i < 3; i++) {
        f_obj->initialStancePelvisTransform.position.data[i] = 0.0;
    }

    for (i = 0; i < 9; i++) {
        f_obj->initialStancePelvisTransform.rotation.data[i] =
            obj_rotation_data[i];
    }

    for (i = 0; i < 9; i++) {
        obj_rotation_data[i] = 0;
    }

    i = 0;
    for (k = 0; k < 3; k++) {
        obj_rotation_data[i] = 1;
        i += 4;
    }

    for (i = 0; i < 3; i++) {
        f_obj->initialSwingPelvisTransform.position.data[i] = 0.0;
    }

    for (i = 0; i < 9; i++) {
        f_obj->initialSwingPelvisTransform.rotation.data[i] =
            obj_rotation_data[i];
    }

    for (i = 0; i < 9; i++) {
        obj_rotation_data[i] = 0;
    }

    i = 0;
    for (k = 0; k < 3; k++) {
        obj_rotation_data[i] = 1;
        i += 4;
    }

    for (i = 0; i < 3; i++) {
        f_obj->initialStanceFootTransform.position.data[i] = 0.0;
    }

    for (i = 0; i < 9; i++) {
        f_obj->initialStanceFootTransform.rotation.data[i] = obj_rotation_data[i];
    }

    for (i = 0; i < 6; i++) {
        f_obj->c_initialStanceFootSpatialVeloc[i] = 0.0;
    }

    for (i = 0; i < 9; i++) {
        obj_rotation_data[i] = 0;
    }

    i = 0;
    for (k = 0; k < 3; k++) {
        obj_rotation_data[i] = 1;
        i += 4;
    }

    for (i = 0; i < 3; i++) {
        f_obj->initialSwingFootTransform.position.data[i] = 0.0;
    }

    for (i = 0; i < 9; i++) {
        f_obj->initialSwingFootTransform.rotation.data[i] = obj_rotation_data[i];
    }

    for (i = 0; i < 6; i++) {
        f_obj->initialSwingFootSpatialVelocity[i] = 0.0;
    }

    f_obj->stand = 0.0;
    obj->controllerMode = 0.0;
}

static void CassieController_stepImpl(CassieController *obj, const double
    outputs_vectorNavOrientation[4], const double
    c_outputs_vectorNavAngularVeloc[3], const double
    c_outputs_vectorNavLinearAccele[3], const double
    outputs_vectorNavMagnetometer[3], double outputs_vectorNavPressure, double
    outputs_vectorNavTemperature, const double outputs_motorPosition[10], const
    double outputs_motorVelocity[10], const double outputs_jointPosition[6],
    const double outputs_jointVelocity[6], const double outputs_radio[16],
    double outputs_stateOfCharge, double outputs_status, double inputs_radio[9],
    double inputs_torque[10])
{
    CassieInputs *b_obj;
    int i;
    CassieOutputs *c_obj;
    PDController *d_obj;
    static const short b_proportionalGain[10] = { 300, 300, 300, 300, 30, 300,
        300, 300, 300, 30 };

    static const signed char b_derivativeGain[10] = { 4, 4, 12, 12, 3, 4, 4, 12,
        12, 3 };

    double b_radioChannel[16];
    double lastSignalValue;
    double y_offset;
    int desiredControllerMode;
    Vector3d v;
    SmoothRateLimiter *e_obj;
    Transform3d b_relativeTransform;
    Frame_3 *f_obj;
    ZUpFrame_1 *withRespectTo;
    ZUpFrame *inCoordinatesOf;
    Frame_2 *b_parentFrame;
    StandingController *g_obj;
    Transform3d r0;
    SteppingController *h_obj;
    Transform3d r1;
    double a_position_data[3];
    double a_rotation_data[9];
    Frame_6 *i_obj;
    double tau[10];
    int i2;
    double R_data[9];
    double p_data[3];
    int i3;
    double transform_position_data[3];
    Rotation3d expl_temp;
    Frame_7 *j_obj;
    double dq[10];
    int i4;
    int i5;
    double q_desired;
    double u_desired[10];
    double dq_desired;
    b_obj = &obj->robot.inputs;
    for (i = 0; i < 10; i++) {
        b_obj->data.torque[i] = 0.0;
    }

    c_obj = &obj->robot.outputs;
    for (i = 0; i < 4; i++) {
        c_obj->data.vectorNavOrientation[i] = outputs_vectorNavOrientation[i];
    }

    for (i = 0; i < 3; i++) {
        c_obj->data.vectorNavAngularVelocity[i] =
            c_outputs_vectorNavAngularVeloc[i];
    }

    for (i = 0; i < 3; i++) {
        c_obj->data.vectorNavLinearAcceleration[i] =
            c_outputs_vectorNavLinearAccele[i];
    }

    for (i = 0; i < 3; i++) {
        c_obj->data.vectorNavMagnetometer[i] = outputs_vectorNavMagnetometer[i];
    }

    c_obj->data.vectorNavPressure = outputs_vectorNavPressure;
    c_obj->data.vectorNavTemperature = outputs_vectorNavTemperature;
    for (i = 0; i < 10; i++) {
        c_obj->data.motorPosition[i] = outputs_motorPosition[i];
    }

    for (i = 0; i < 10; i++) {
        c_obj->data.motorVelocity[i] = outputs_motorVelocity[i];
    }

    for (i = 0; i < 6; i++) {
        c_obj->data.jointPosition[i] = outputs_jointPosition[i];
    }

    for (i = 0; i < 6; i++) {
        c_obj->data.jointVelocity[i] = outputs_jointVelocity[i];
    }

    for (i = 0; i < 16; i++) {
        c_obj->data.radio[i] = outputs_radio[i];
    }

    c_obj->data.stateOfCharge = outputs_stateOfCharge;
    c_obj->data.status = outputs_status;
    CassieRobot_update(&obj->robot);
    d_obj = &obj->pdController;
    for (i = 0; i < 10; i++) {
        d_obj->proportionalGain[i] = b_proportionalGain[i];
    }

    d_obj = &obj->pdController;
    for (i = 0; i < 10; i++) {
        d_obj->derivativeGain[i] = b_derivativeGain[i];
    }

    c_obj = &obj->robot.outputs;
    for (i = 0; i < 16; i++) {
        b_radioChannel[i] = c_obj->data.radio[i];
    }

    c_obj = &obj->robot.outputs;
    lastSignalValue = c_obj->data.status;
    if (lastSignalValue == 2.0) {
        y_offset = 0.0;
        switch ((int)b_radioChannel[9]) {
          case 0:
            desiredControllerMode = 2;
            break;

          case 1:
            desiredControllerMode = 3;
            break;

          default:
            desiredControllerMode = 0;
            break;
        }

        switch ((int)obj->controllerMode) {
          case 3:
            if (desiredControllerMode == 2) {
                v = obj->robot.centerOfMassVelocity;
                if ((fabs(v.data[0]) < 0.05) && (fabs(v.data[1]) < 0.1) &&
                        (obj->steppingController.stanceTime < 0.08) &&
                        (obj->steppingController.stanceTime > 0.06)) {
                    obj->steppingController.stand = 1.0;
                }

                if ((obj->steppingController.stanceTime == 0.0) &&
                        (obj->steppingController.stand == 1.0)) {
                    obj->steppingController.stand = 0.0;
                } else {
                    desiredControllerMode = 3;
                }
            }
            break;

          case 2:
            if (desiredControllerMode == 3) {
                y_offset = 0.2;
                e_obj = &obj->standingController.pelvisYFilter;
                lastSignalValue = e_obj->signalValue;
                if (lastSignalValue >= 0.08) {
                    obj->steppingController.stanceTime = 0.0;
                    obj->steppingController.stepInterval = 0.3;
                    obj->steppingController.stanceLeg = LEFT;
                    obj->steppingController.xIntegralError = 0.0;
                    obj->steppingController.yIntegralError = 0.0;
                    for (i = 0; i < 3; i++) {
                        v.data[i] = 0.0;
                    }

                    h_obj = &obj->steppingController;
                    h_obj->initialPelvisVelocity = v;
                    i_obj = &obj->standingController.desiredLeftFootFrame;
                    b_relativeTransform = i_obj->relativeTransform;
                    h_obj = &obj->steppingController;
                    if (h_obj->stanceLeg == LEFT) {
                        h_obj->initialStanceFootTransform = b_relativeTransform;
                    } else {
                        h_obj->initialSwingFootTransform = b_relativeTransform;
                    }

                    j_obj = &obj->standingController.desiredRightFootFrame;
                    b_relativeTransform = j_obj->relativeTransform;
                    h_obj = &obj->steppingController;
                    if (h_obj->stanceLeg == RIGHT) {
                        h_obj->initialStanceFootTransform = b_relativeTransform;
                    } else {
                        h_obj->initialSwingFootTransform = b_relativeTransform;
                    }

                    h_obj = &obj->steppingController;
                    if (h_obj->stanceLeg == LEFT) {
                        for (i = 0; i < 6; i++) {
                            h_obj->c_initialStanceFootSpatialVeloc[i] = 0.0;
                        }
                    } else {
                        for (i = 0; i < 6; i++) {
                            h_obj->initialSwingFootSpatialVelocity[i] = 0.0;
                        }
                    }

                    h_obj = &obj->steppingController;
                    if (h_obj->stanceLeg == RIGHT) {
                        for (i = 0; i < 6; i++) {
                            h_obj->c_initialStanceFootSpatialVeloc[i] = 0.0;
                        }
                    } else {
                        for (i = 0; i < 6; i++) {
                            h_obj->initialSwingFootSpatialVelocity[i] = 0.0;
                        }
                    }
                } else {
                    desiredControllerMode = 2;
                }
            }

            if ((desiredControllerMode == 2) && (b_radioChannel[10] == 1.0)) {
                f_obj = &obj->robot.centerOfMassFrame;
                withRespectTo = &obj->standingController.centerOfSupportFrame;
                inCoordinatesOf = &obj->robot.zUpPelvisFrame;
                b_parentFrame = f_obj->parentFrame;
                b_relativeTransform = f_obj->relativeTransform;
                b_Frame_getWorldTransform(b_parentFrame, &r0);
                for (i = 0; i < 3; i++) {
                    a_position_data[i] = r0.position.data[i];
                }

                b_Frame_getWorldTransform(b_parentFrame, &r1);
                memcpy(&a_rotation_data[0], &r1.rotation.data[0], 9U * sizeof
                       (double));
                for (i = 0; i < 3; i++) {
                    lastSignalValue = 0.0;
                    i2 = 0;
                    for (i3 = 0; i3 < 3; i3++) {
                        lastSignalValue += a_rotation_data[i2 + i] *
                            b_relativeTransform.position.data[i3];
                        R_data[i2 + i] = 0.0;
                        i4 = 0;
                        for (i5 = 0; i5 < 3; i5++) {
                            R_data[i2 + i] += a_rotation_data[i4 + i] *
                                b_relativeTransform.rotation.data[i5 + i2];
                            i4 += 3;
                        }

                        i2 += 3;
                    }

                    p_data[i] = a_position_data[i] + lastSignalValue;
                }

                for (i = 0; i < 3; i++) {
                    b_relativeTransform.position.data[i] = p_data[i];
                }

                memcpy(&b_relativeTransform.rotation.data[0], &R_data[0], 9U *
                       sizeof(double));
                d_Frame_getWorldTransform(withRespectTo, a_position_data,
                    a_rotation_data);
                Transform3d_mldivide(a_position_data, a_rotation_data,
                                     b_relativeTransform.position.data,
                                     b_relativeTransform.rotation.data,
                                     transform_position_data, expl_temp.data);
                Frame_getWorldTransform(inCoordinatesOf,
                                        b_relativeTransform.position.data,
                                        &b_relativeTransform.rotation);
                d_Frame_getWorldTransform(withRespectTo, a_position_data,
                    a_rotation_data);
                i = 0;
                for (i2 = 0; i2 < 3; i2++) {
                    lastSignalValue = 0.0;
                    i3 = 0;
                    for (i4 = 0; i4 < 3; i4++) {
                        R_data[i3 + i2] = 0.0;
                        for (i5 = 0; i5 < 3; i5++) {
                            R_data[i3 + i2] +=
                                b_relativeTransform.rotation.data[i5 + i] *
                                a_rotation_data[i5 + i3];
                        }

                        lastSignalValue += R_data[i3 + i2] *
                            transform_position_data[i4];
                        i3 += 3;
                    }

                    p_data[i2] = lastSignalValue;
                    i += 3;
                }

                if ((fabs(p_data[0]) > 0.08) || (fabs(p_data[1]) > 0.09)) {
                    e_obj = &obj->standingController.legLengthFilter;
                    lastSignalValue = e_obj->signalValue;
                    if (lastSignalValue >= 1.0) {
                        desiredControllerMode = 3;
                        obj->steppingController.stanceTime = 0.0;
                        obj->steppingController.stepInterval = 0.25;
                        obj->steppingController.stanceLeg = LEFT;
                        obj->steppingController.xIntegralError = 0.0;
                        obj->steppingController.yIntegralError = 0.0;
                        h_obj = &obj->steppingController;
                        v = obj->robot.centerOfMassVelocity;
                        h_obj->initialPelvisVelocity = v;
                        i_obj = &obj->standingController.desiredLeftFootFrame;
                        b_relativeTransform = i_obj->relativeTransform;
                        h_obj = &obj->steppingController;
                        if (h_obj->stanceLeg == LEFT) {
                            h_obj->initialStanceFootTransform =
                                b_relativeTransform;
                        } else {
                            h_obj->initialSwingFootTransform =
                                b_relativeTransform;
                        }

                        j_obj = &obj->standingController.desiredRightFootFrame;
                        b_relativeTransform = j_obj->relativeTransform;
                        h_obj = &obj->steppingController;
                        if (h_obj->stanceLeg == RIGHT) {
                            h_obj->initialStanceFootTransform =
                                b_relativeTransform;
                        } else {
                            h_obj->initialSwingFootTransform =
                                b_relativeTransform;
                        }

                        h_obj = &obj->steppingController;
                        if (h_obj->stanceLeg == LEFT) {
                            for (i = 0; i < 6; i++) {
                                h_obj->c_initialStanceFootSpatialVeloc[i] = 0.0;
                            }
                        } else {
                            for (i = 0; i < 6; i++) {
                                h_obj->initialSwingFootSpatialVelocity[i] = 0.0;
                            }
                        }

                        h_obj = &obj->steppingController;
                        if (h_obj->stanceLeg == RIGHT) {
                            for (i = 0; i < 6; i++) {
                                h_obj->c_initialStanceFootSpatialVeloc[i] = 0.0;
                            }
                        } else {
                            for (i = 0; i < 6; i++) {
                                h_obj->initialSwingFootSpatialVelocity[i] = 0.0;
                            }
                        }
                    }
                }
            }
            break;
        }

        if (b_radioChannel[15] == -1.0) {
            desiredControllerMode = 1;
        }

        b_Frame_getWorldTransform(&obj->robot.pelvisFrame, &b_relativeTransform);
        if ((fabs(-asin(b_relativeTransform.rotation.data[2])) >
                1.5707963267948966) || (fabs(atan2
                (b_relativeTransform.rotation.data[5],
                 b_relativeTransform.rotation.data[8])) > 1.5707963267948966)) {
            desiredControllerMode = 0;
        }

        obj->controllerMode = desiredControllerMode;
        g_obj = &obj->standingController;
        SmoothRateLimiter_updateSignal(&g_obj->legLengthFilter, fmax(fmin(0.75 +
            0.3 * b_radioChannel[6], 1.05), 0.45));
        h_obj = &obj->steppingController;
        SmoothRateLimiter_updateSignal(&h_obj->legLengthFilter, 0.75 + 0.3 *
            b_radioChannel[6]);
        switch ((int)obj->controllerMode) {
          case 2:
            g_obj = &obj->standingController;
            SmoothRateLimiter_updateSignal(&g_obj->pelvisYFilter, fmax(fmin
                (y_offset - 0.04 * b_radioChannel[1], 0.1), -0.1));
            StandingController_setPelvisYaw(&obj->standingController,
                -0.26179938779914941 * b_radioChannel[3]);
            c_StandingController_setPelvisP(&obj->standingController,
                -0.26179938779914941 * b_radioChannel[2]);
            StandingController_eval(&obj->standingController, tau);
            h_obj = &obj->steppingController;
            lastSignalValue = h_obj->xVelocityFilter.signalValue;
            h_obj->xVelocityFilter.signalValue +=
                h_obj->xVelocityFilter.smoothingFactor * (0.0 -
                h_obj->xVelocityFilter.signalValue);
            lastSignalValue = h_obj->xVelocityFilter.signalValue -
                lastSignalValue;
            y_offset = h_obj->xVelocityFilter.sampleTime;
            h_obj->xVelocityFilter.signalRateOfChange +=
                h_obj->xVelocityFilter.smoothingFactor * (lastSignalValue /
                y_offset - h_obj->xVelocityFilter.signalRateOfChange);
            h_obj = &obj->steppingController;
            lastSignalValue = h_obj->yVelocityFilter.signalValue;
            h_obj->yVelocityFilter.signalValue +=
                h_obj->yVelocityFilter.smoothingFactor * (0.0 -
                h_obj->yVelocityFilter.signalValue);
            lastSignalValue = h_obj->yVelocityFilter.signalValue -
                lastSignalValue;
            y_offset = h_obj->yVelocityFilter.sampleTime;
            h_obj->yVelocityFilter.signalRateOfChange +=
                h_obj->yVelocityFilter.smoothingFactor * (lastSignalValue /
                y_offset - h_obj->yVelocityFilter.signalRateOfChange);
            h_obj = &obj->steppingController;
            lastSignalValue = h_obj->yawVelocityFilter.signalValue;
            h_obj->yawVelocityFilter.signalValue +=
                h_obj->yawVelocityFilter.smoothingFactor * (0.0 -
                h_obj->yawVelocityFilter.signalValue);
            lastSignalValue = h_obj->yawVelocityFilter.signalValue -
                lastSignalValue;
            y_offset = h_obj->yawVelocityFilter.sampleTime;
            h_obj->yawVelocityFilter.signalRateOfChange +=
                h_obj->yawVelocityFilter.smoothingFactor * (lastSignalValue /
                y_offset - h_obj->yawVelocityFilter.signalRateOfChange);
            break;

          case 3:
            h_obj = &obj->steppingController;
            lastSignalValue = h_obj->xVelocityFilter.signalValue;
            h_obj->xVelocityFilter.signalValue +=
                h_obj->xVelocityFilter.smoothingFactor * (0.8 * b_radioChannel[0]
                - h_obj->xVelocityFilter.signalValue);
            lastSignalValue = h_obj->xVelocityFilter.signalValue -
                lastSignalValue;
            y_offset = h_obj->xVelocityFilter.sampleTime;
            h_obj->xVelocityFilter.signalRateOfChange +=
                h_obj->xVelocityFilter.smoothingFactor * (lastSignalValue /
                y_offset - h_obj->xVelocityFilter.signalRateOfChange);
            h_obj = &obj->steppingController;
            lastSignalValue = h_obj->yVelocityFilter.signalValue;
            h_obj->yVelocityFilter.signalValue +=
                h_obj->yVelocityFilter.smoothingFactor * (-0.2 * b_radioChannel
                [1] - h_obj->yVelocityFilter.signalValue);
            lastSignalValue = h_obj->yVelocityFilter.signalValue -
                lastSignalValue;
            y_offset = h_obj->yVelocityFilter.sampleTime;
            h_obj->yVelocityFilter.signalRateOfChange +=
                h_obj->yVelocityFilter.smoothingFactor * (lastSignalValue /
                y_offset - h_obj->yVelocityFilter.signalRateOfChange);
            h_obj = &obj->steppingController;
            lastSignalValue = h_obj->yawVelocityFilter.signalValue;
            h_obj->yawVelocityFilter.signalValue +=
                h_obj->yawVelocityFilter.smoothingFactor * (-0.5 *
                b_radioChannel[3] - h_obj->yawVelocityFilter.signalValue);
            lastSignalValue = h_obj->yawVelocityFilter.signalValue -
                lastSignalValue;
            y_offset = h_obj->yawVelocityFilter.sampleTime;
            h_obj->yawVelocityFilter.signalRateOfChange +=
                h_obj->yawVelocityFilter.smoothingFactor * (lastSignalValue /
                y_offset - h_obj->yawVelocityFilter.signalRateOfChange);
            SteppingController_eval(&obj->steppingController, tau);
            g_obj = &obj->standingController;
            SmoothRateLimiter_updateSignal(&g_obj->pelvisYFilter, 0.0);
            StandingController_setPelvisYaw(&obj->standingController, 0.0);
            c_StandingController_setPelvisP(&obj->standingController, 0.0);
            break;

          case 1:
            d_obj = &obj->pdController;
            for (i = 0; i < 10; i++) {
                d_obj->desiredMotorVelocity[i] = 0.0;
            }

            c_obj = &obj->robot.outputs;
            for (i = 0; i < 10; i++) {
                tau[i] = c_obj->data.motorPosition[i];
            }

            c_obj = &obj->robot.outputs;
            for (i = 0; i < 10; i++) {
                dq[i] = c_obj->data.motorVelocity[i];
            }

            d_obj = &obj->pdController;
            for (i = 0; i < 10; i++) {
                lastSignalValue = d_obj->proportionalGain[i];
                y_offset = d_obj->derivativeGain[i];
                q_desired = d_obj->desiredMotorPosition[i];
                dq_desired = d_obj->desiredMotorVelocity[i];
                u_desired[i] = lastSignalValue * (q_desired - tau[(int)
                    d_obj->activeMotorIndex[i] - 1]) + y_offset * (dq_desired -
                    dq[(int)d_obj->activeMotorIndex[i] - 1]);
            }

            memset(&tau[0], 0, 10U * sizeof(double));
            for (i = 0; i < 10; i++) {
                tau[(int)d_obj->activeMotorIndex[i] - 1] = u_desired[i];
            }
            break;

          default:
            c_obj = &obj->robot.outputs;
            for (i = 0; i < 10; i++) {
                tau[i] = c_obj->data.motorPosition[i];
            }

            d_obj = &obj->pdController;
            for (i = 0; i < 10; i++) {
                d_obj->desiredMotorPosition[i] = tau[i];
            }

            d_obj = &obj->pdController;
            for (i = 0; i < 10; i++) {
                d_obj->desiredMotorVelocity[i] = 0.0;
            }

            c_obj = &obj->robot.outputs;
            for (i = 0; i < 10; i++) {
                tau[i] = c_obj->data.motorPosition[i];
            }

            c_obj = &obj->robot.outputs;
            for (i = 0; i < 10; i++) {
                dq[i] = c_obj->data.motorVelocity[i];
            }

            d_obj = &obj->pdController;
            for (i = 0; i < 10; i++) {
                lastSignalValue = d_obj->proportionalGain[i];
                y_offset = d_obj->derivativeGain[i];
                q_desired = d_obj->desiredMotorPosition[i];
                dq_desired = d_obj->desiredMotorVelocity[i];
                u_desired[i] = lastSignalValue * (q_desired - tau[(int)
                    d_obj->activeMotorIndex[i] - 1]) + y_offset * (dq_desired -
                    dq[(int)d_obj->activeMotorIndex[i] - 1]);
            }

            memset(&tau[0], 0, 10U * sizeof(double));
            for (i = 0; i < 10; i++) {
                tau[(int)d_obj->activeMotorIndex[i] - 1] = u_desired[i];
            }
            break;
        }

        b_obj = &obj->robot.inputs;
        for (i = 0; i < 10; i++) {
            b_obj->data.torque[i] = tau[i];
        }
    }

    b_obj = &obj->robot.inputs;
    for (i = 0; i < 9; i++) {
        inputs_radio[i] = b_obj->data.radio[i];
    }

    for (i = 0; i < 10; i++) {
        inputs_torque[i] = b_obj->data.torque[i];
    }
}

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

static CassieRobot *CassieRobot_CassieRobot(CassieRobot *obj)
{
    CassieRobot *b_obj;
    b_obj = obj;
    b_CassieRobot_CassieRobot(&b_obj);
    return b_obj;
}

static void CassieRobot_update(CassieRobot *obj)
{
    CassieOutputs *b_obj;
    double t7;
    int i;
    double vectorNavQuaternion_data[4];
    double obj_rotation_data[9];
    Frame_1 *c_obj;
    double w[3];
    double q[10];
    double obj_data[3];
    double b_q[10];
    int k;
    Frame_3 *d_obj;
    double dv5[30];
    int i6;
    double c_q[5];
    double d_q[2];
    double T_l[16];
    double e_q[5];
    double f_q[2];
    double t2;
    double t3;
    double t4;
    double t5;
    double t6;
    double t8;
    double t11;
    double t12;
    double t14;
    double t15;
    double t17;
    double t21;
    double t22;
    double t24;
    double t28;
    double t29;
    double t31;
    double t33;
    double t34;
    double t35;
    double t37;
    double t40;
    double t44;
    double t45;
    double t48;
    double t46;
    double t47;
    double t50;
    double t52;
    double t54;
    double t56;
    double t58;
    double t62;
    double t64;
    double t68;
    double t69;
    double t70;
    double t73;
    double t74;
    double t72;
    double t75;
    double t76;
    double t77;
    double t83;
    double t80;
    double t82;
    double t86;
    double b_t15[16];
    double T_r[16];
    Frame_4 *e_obj;
    int i7;
    double g_q[2];
    double dv6[42];
    double dv7[6];
    double dv8[2];
    Vector3d rateOfChange;
    double b_obj_data[3];
    LowPassFilter *f_obj;
    Vector3d a;
    double lastSignalValue_data[3];
    b_obj = &obj->outputs;
    t7 = b_obj->data.status;
    if (t7 == 2.0) {
        b_obj = &obj->outputs;
        for (i = 0; i < 4; i++) {
            t7 = b_obj->data.vectorNavOrientation[i];
            vectorNavQuaternion_data[i] = t7;
        }

        Quaternion_getRotation(vectorNavQuaternion_data, obj_rotation_data);
        c_obj = &obj->vectorNavFrame;
        for (i = 0; i < 3; i++) {
            c_obj->relativeTransform.position.data[i] = 0.0;
        }

        for (i = 0; i < 9; i++) {
            c_obj->relativeTransform.rotation.data[i] = obj_rotation_data[i];
        }

        b_obj = &obj->outputs;
        for (i = 0; i < 3; i++) {
            w[i] = b_obj->data.vectorNavAngularVelocity[i];
        }

        c_obj = &obj->vectorNavFrame;
        for (i = 0; i < 3; i++) {
            c_obj->relativeSpatialVelocity[i + 3] = w[i];
        }

        b_obj = &obj->outputs;
        for (i = 0; i < 10; i++) {
            q[i] = b_obj->data.motorPosition[i];
        }

        b_obj = &obj->outputs;
        for (i = 0; i < 10; i++) {
            b_q[i] = b_obj->data.motorPosition[i];
        }

        generatedCenterOfMassPosition(q, obj_data);
        for (i = 0; i < 3; i++) {
            w[i] = obj_data[i];
        }

        memset(&obj_rotation_data[0], 0, 9U * sizeof(double));
        i = 0;
        for (k = 0; k < 3; k++) {
            obj_rotation_data[i] = 1.0;
            i += 4;
        }

        d_obj = &obj->centerOfMassFrame;
        for (i = 0; i < 3; i++) {
            d_obj->relativeTransform.position.data[i] = w[i];
        }

        for (i = 0; i < 9; i++) {
            d_obj->relativeTransform.rotation.data[i] = obj_rotation_data[i];
        }

        b_obj = &obj->outputs;
        for (i = 0; i < 10; i++) {
            q[i] = b_obj->data.motorVelocity[i];
        }

        d_obj = &obj->centerOfMassFrame;
        generatedCenterOfMassJacobian(b_q, dv5);
        for (i = 0; i < 3; i++) {
            w[i] = 0.0;
            k = 0;
            for (i6 = 0; i6 < 10; i6++) {
                w[i] += dv5[k + i] * q[i6];
                k += 3;
            }
        }

        for (i = 0; i < 3; i++) {
            d_obj->relativeSpatialVelocity[i] = w[i];
        }

        d_obj->relativeSpatialVelocity[3] = 0.0;
        d_obj->relativeSpatialVelocity[4] = 0.0;
        d_obj->relativeSpatialVelocity[5] = 0.0;
        b_obj = &obj->outputs;
        for (i = 0; i < 5; i++) {
            c_q[i] = b_obj->data.motorPosition[i];
        }

        b_obj = &obj->outputs;
        d_q[0] = b_obj->data.jointPosition[0];
        d_q[1] = ((b_obj->data.jointPosition[1] + b_obj->data.motorPosition[3])
                  + b_obj->data.jointPosition[0]) - 0.22689280275926285;
        generatedLeftFootTransform(c_q, d_q, T_l);
        b_obj = &obj->outputs;
        for (i = 0; i < 5; i++) {
            c_q[i] = b_obj->data.motorPosition[i];
        }

        b_obj = &obj->outputs;
        d_q[0] = b_obj->data.jointPosition[0];
        d_q[1] = ((b_obj->data.jointPosition[1] + b_obj->data.motorPosition[3])
                  + b_obj->data.jointPosition[0]) - 0.22689280275926285;
        b_obj = &obj->outputs;
        for (i = 0; i < 5; i++) {
            e_q[i] = b_obj->data.motorPosition[i + 5];
        }

        b_obj = &obj->outputs;
        f_q[0] = b_obj->data.jointPosition[3];
        f_q[1] = ((b_obj->data.jointPosition[4] + b_obj->data.motorPosition[8])
                  + b_obj->data.jointPosition[3]) - 0.22689280275926285;
        t2 = cos(e_q[1]);
        t3 = cos(e_q[2]);
        t4 = sin(e_q[3]);
        t5 = cos(e_q[3]);
        t6 = sin(e_q[2]);
        t7 = ((e_q[3] + f_q[0]) - f_q[1]) - 0.22689280275926282;
        t8 = cos(f_q[0]);
        t11 = t2 * t3 * t4 + t2 * t5 * t6;
        t12 = sin(f_q[0]);
        t14 = t2 * t3 * t5 - t2 * t4 * t6;
        t15 = cos(e_q[4]);
        t17 = cos(t7);
        t21 = t8 * t11 + t12 * t14;
        t22 = sin(t7);
        t24 = t8 * t14 - t11 * t12;
        t28 = t17 * t24 + t21 * t22;
        t29 = sin(e_q[4]);
        t31 = t17 * t21 - t22 * t24;
        t33 = sin(e_q[1]);
        t34 = sin(e_q[0]);
        t35 = cos(e_q[0]);
        t37 = t6 * t34 - t3 * t33 * t35;
        t40 = t3 * t34 + t6 * t33 * t35;
        t44 = t5 * t37 + t4 * t40;
        t45 = t4 * t37;
        t48 = t5 * t40;
        t46 = t45 - t48;
        t47 = t12 * t44;
        t50 = t8 * t44 - t12 * t46;
        t52 = t17 * t50;
        t54 = t47 + t8 * (t45 - t48);
        t56 = t17 * t54 - t22 * t50;
        t58 = t52 + t22 * t54;
        t62 = t6 * t35 + t3 * t33 * t34;
        t64 = t3 * t35 - t6 * t33 * t34;
        t68 = t5 * t62 + t4 * t64;
        t69 = t4 * t62;
        t70 = t12 * t68;
        t73 = t5 * t64;
        t74 = t69 - t73;
        t72 = t70 + t8 * t74;
        t75 = t8 * t68;
        t76 = t75 - t12 * t74;
        t77 = t22 * t72;
        t83 = t22 * t76;
        t80 = t17 * t72 - t83;
        t7 = t17 * t76;
        t82 = t77 + t7;
        t72 = t70 + t8 * (t69 - t73);
        t70 = t17 * t72;
        t86 = t83 - t70;
        t7 += t22 * t72;
        b_t15[0] = ((t15 * t28 * 0.64278760968653947 - t15 * t31 *
                     0.7660444431189779) - t28 * t29 * 0.7660444431189779) - t29
            * t31 * 0.64278760968653947;
        b_t15[1] = ((t15 * t56 * 0.7660444431189779 + t29 * t56 *
                     0.64278760968653947) + t29 * t58 * 0.7660444431189779) -
            t15 * (t52 + t22 * (t47 + t8 * t46)) * 0.64278760968653947;
        b_t15[2] = ((t15 * (t77 + t17 * (t75 - t12 * (t69 - t5 * t64))) *
                     0.64278760968653947 - t15 * t80 * 0.7660444431189779) - t29
                    * t80 * 0.64278760968653947) - t29 * t82 *
            0.7660444431189779;
        b_t15[3] = 0.0;
        b_t15[4] = (((-t33 - t15 * t28 * 4.6906693763513648E-17) - t15 * t31 *
                     3.9359389436709938E-17) - t28 * t29 *
                    3.9359389436709938E-17) + t29 * t31 * 4.6906693763513648E-17;
        b_t15[5] = (((t2 * t35 + t15 * t56 * 3.9359389436709938E-17) + t15 * t58
                     * 4.6906693763513648E-17) - t29 * t56 *
                    4.6906693763513648E-17) + t29 * t58 * 3.9359389436709938E-17;
        b_t15[6] = (((t2 * t34 - t15 * t82 * 4.6906693763513648E-17) + t15 * t86
                     * 3.9359389436709938E-17) - t29 * t86 *
                    4.6906693763513648E-17) - t29 * t7 * 3.9359389436709938E-17;
        b_t15[7] = 0.0;
        b_t15[8] = (((t33 * 6.123233995736766E-17 - t15 * t28 *
                      0.7660444431189779) - t15 * t31 * 0.64278760968653947) -
                    t28 * t29 * 0.64278760968653947) + t29 * t31 *
            0.7660444431189779;
        b_t15[9] = (((t2 * t35 * -6.123233995736766E-17 + t15 * t56 *
                      0.64278760968653947) + t15 * t58 * 0.7660444431189779) -
                    t29 * t56 * 0.7660444431189779) + t29 * t58 *
            0.64278760968653947;
        b_t15[10] = (((t2 * t34 * -6.123233995736766E-17 - t15 * t7 *
                       0.7660444431189779) - t29 * t86 * 0.7660444431189779) -
                     t29 * t7 * 0.64278760968653947) + t15 * (t83 - t70) *
            0.64278760968653947;
        b_t15[11] = 0.0;
        b_t15[12] = (((((((((((((((((t33 * -0.0045 + t2 * t6 * 0.12) + t8 * t11 *
            0.43476) + t8 * t14 * 0.02) - t11 * t12 * 0.02) + t12 * t14 *
            0.43476) + t17 * t21 * 0.408) - t17 * t24 * 0.04) + t15 * t28 *
                              0.05219) - t21 * t22 * 0.04) + t15 * t31 * 0.01762)
                           - t22 * t24 * 0.408) + t28 * t29 * 0.01762) - t29 *
                         t31 * 0.05219) + t2 * t3 * t4 * 0.06068) + t2 * t3 * t5
                       * 0.04741) - t2 * t4 * t6 * 0.04741) + t2 * t5 * t6 *
                     0.06068) - 0.049;
        b_t15[13] = (((((((((((((((((((t34 * 0.09 + t2 * t35 * 0.0045) + t3 *
            t34 * 0.12) - t4 * t37 * 0.06068) - t5 * t37 * 0.04741) - t4 * t40 *
            0.04741) + t5 * t40 * 0.06068) - t8 * t44 * 0.02) - t8 * t46 *
                                0.43476) - t12 * t44 * 0.43476) + t17 * t50 *
                              0.04) - t15 * t56 * 0.01762) - t17 * t54 * 0.408)
                           + t22 * t50 * 0.408) - t15 * t58 * 0.05219) + t22 *
                         t54 * 0.04) + t29 * t56 * 0.05219) - t29 * t58 *
                       0.01762) + t12 * (t45 - t48) * 0.02) + t6 * t33 * t35 *
                     0.12) - 0.135;
        b_t15[14] = ((((((((((((((((((t35 * -0.09 + t2 * t34 * 0.0045) - t3 *
            t35 * 0.12) + t4 * t62 * 0.06068) + t5 * t62 * 0.04741) + t4 * t64 *
            0.04741) - t5 * t64 * 0.06068) + t8 * t68 * 0.02) + t12 * t68 *
                               0.43476) - t12 * t74 * 0.02) - t17 * t76 * 0.04)
                            - t22 * t76 * 0.408) - t15 * t86 * 0.01762) + t17 *
                          t72 * 0.408) + t15 * t7 * 0.05219) - t22 * t72 * 0.04)
                       + t29 * t7 * 0.01762) + t8 * (t69 - t73) * 0.43476) + t29
                     * (t83 - t70) * 0.05219) + t6 * t33 * t34 * 0.12;
        b_t15[15] = 1.0;
        i = 0;
        for (k = 0; k < 4; k++) {
            for (i6 = 0; i6 < 4; i6++) {
                T_r[i6 + i] = b_t15[i6 + i];
            }

            i += 4;
        }

        b_obj = &obj->outputs;
        for (i = 0; i < 5; i++) {
            e_q[i] = b_obj->data.motorPosition[i + 5];
        }

        b_obj = &obj->outputs;
        f_q[0] = b_obj->data.jointPosition[3];
        f_q[1] = ((b_obj->data.jointPosition[4] + b_obj->data.motorPosition[8])
                  + b_obj->data.jointPosition[3]) - 0.22689280275926285;
        i = 0;
        k = 0;
        for (i6 = 0; i6 < 3; i6++) {
            w[i6] = T_l[i6 + 12];
            for (i7 = 0; i7 < 3; i7++) {
                obj_rotation_data[i7 + i] = T_l[i7 + k];
            }

            i += 3;
            k += 4;
        }

        e_obj = &obj->leftFootFrame;
        for (i = 0; i < 3; i++) {
            e_obj->relativeTransform.position.data[i] = w[i];
        }

        for (i = 0; i < 9; i++) {
            e_obj->relativeTransform.rotation.data[i] = obj_rotation_data[i];
        }

        i = 0;
        k = 0;
        for (i6 = 0; i6 < 3; i6++) {
            w[i6] = T_r[i6 + 12];
            for (i7 = 0; i7 < 3; i7++) {
                obj_rotation_data[i7 + i] = T_r[i7 + k];
            }

            i += 3;
            k += 4;
        }

        e_obj = &obj->rightFootFrame;
        for (i = 0; i < 3; i++) {
            e_obj->relativeTransform.position.data[i] = w[i];
        }

        for (i = 0; i < 9; i++) {
            e_obj->relativeTransform.rotation.data[i] = obj_rotation_data[i];
        }

        Frame_getTransform(&obj->pelvisFrame, &obj->zUpPelvisFrame, w,
                           obj_rotation_data);
        b_obj = &obj->outputs;
        g_q[0] = b_obj->data.jointPosition[0];
        g_q[1] = ((b_obj->data.jointPosition[1] + b_obj->data.motorPosition[3])
                  + b_obj->data.jointPosition[0]) - 0.22689280275926285;
        generatedLeftFootJacobian(c_q, d_q, dv6);
        i = 0;
        for (k = 0; k < 3; k++) {
            i6 = 0;
            for (i7 = 0; i7 < 2; i7++) {
                dv7[i7 + i] = -dv6[(i6 + k) + 30];
                i6 += 6;
            }

            i += 2;
        }

        for (i = 0; i < 2; i++) {
            dv8[i] = (1610.0 + -520.0 * (double)i) * g_q[i];
        }

        mldivide(dv7, dv8, w);
        for (i = 0; i < 3; i++) {
            rateOfChange.data[i] = 0.0;
            k = 0;
            for (i6 = 0; i6 < 3; i6++) {
                rateOfChange.data[i] += obj_rotation_data[k + i] * w[i6];
                k += 3;
            }
        }

        obj->leftFootForce = rateOfChange;
        b_obj = &obj->outputs;
        d_q[0] = b_obj->data.jointPosition[3];
        d_q[1] = ((b_obj->data.jointPosition[4] + b_obj->data.motorPosition[8])
                  + b_obj->data.jointPosition[3]) - 0.22689280275926285;
        generatedRightFootJacobian(e_q, f_q, dv6);
        i = 0;
        for (k = 0; k < 3; k++) {
            i6 = 0;
            for (i7 = 0; i7 < 2; i7++) {
                dv7[i7 + i] = -dv6[(i6 + k) + 30];
                i6 += 6;
            }

            i += 2;
        }

        for (i = 0; i < 2; i++) {
            dv8[i] = (1610.0 + -520.0 * (double)i) * d_q[i];
        }

        mldivide(dv7, dv8, w);
        for (i = 0; i < 3; i++) {
            rateOfChange.data[i] = 0.0;
            k = 0;
            for (i6 = 0; i6 < 3; i6++) {
                rateOfChange.data[i] += obj_rotation_data[k + i] * w[i6];
                k += 3;
            }
        }

        obj->rightFootForce = rateOfChange;
        for (i = 0; i < 3; i++) {
            obj_data[i] = obj->leftFootForce.data[i];
        }

        for (i = 0; i < 3; i++) {
            b_obj_data[i] = obj->rightFootForce.data[i];
        }

        b_Frame_getTransform(&obj->leftFootFrame, &obj->worldFrame,
                             &obj->zUpPelvisFrame, w, obj_rotation_data);
        f_obj = &obj->leftFootFilter;
        t7 = f_obj->smoothingFactor;
        a = f_obj->signalValue;
        for (i = 0; i < 3; i++) {
            lastSignalValue_data[i] = f_obj->signalValue.data[i];
            t70 = f_obj->signalValue.data[i];
            a.data[i] += t7 * (w[i] - t70);
        }

        f_obj->signalValue = a;
        rateOfChange = f_obj->signalValue;
        t72 = f_obj->sampleTime;
        t7 = f_obj->smoothingFactor;
        a = f_obj->signalRateOfChange;
        for (i = 0; i < 3; i++) {
            rateOfChange.data[i] -= lastSignalValue_data[i];
            rateOfChange.data[i] /= t72;
            t70 = f_obj->signalRateOfChange.data[i];
            rateOfChange.data[i] -= t70;
            a.data[i] += t7 * rateOfChange.data[i];
        }

        f_obj->signalRateOfChange = a;
        b_Frame_getTransform(&obj->rightFootFrame, &obj->worldFrame,
                             &obj->zUpPelvisFrame, w, obj_rotation_data);
        f_obj = &obj->rightFootFilter;
        t7 = f_obj->smoothingFactor;
        a = f_obj->signalValue;
        for (i = 0; i < 3; i++) {
            lastSignalValue_data[i] = f_obj->signalValue.data[i];
            t70 = f_obj->signalValue.data[i];
            a.data[i] += t7 * (w[i] - t70);
        }

        f_obj->signalValue = a;
        rateOfChange = f_obj->signalValue;
        t72 = f_obj->sampleTime;
        t7 = f_obj->smoothingFactor;
        a = f_obj->signalRateOfChange;
        for (i = 0; i < 3; i++) {
            rateOfChange.data[i] -= lastSignalValue_data[i];
            rateOfChange.data[i] /= t72;
            t70 = f_obj->signalRateOfChange.data[i];
            rateOfChange.data[i] -= t70;
            a.data[i] += t7 * rateOfChange.data[i];
        }

        f_obj->signalRateOfChange = a;
        f_obj = &obj->leftFootFilter;
        rateOfChange = f_obj->signalRateOfChange;
        for (i = 0; i < 3; i++) {
            rateOfChange.data[i] = -rateOfChange.data[i];
        }

        for (i = 0; i < 3; i++) {
            w[i] = obj->centerOfMassVelocity.data[i];
        }

        for (i = 0; i < 3; i++) {
            rateOfChange.data[i] -= w[i];
        }

        t7 = 0.0025 * fmax(fmin((obj_data[2] - -32.68692) / -130.74768, 1.0),
                           0.0);
        a = obj->centerOfMassVelocity;
        for (i = 0; i < 3; i++) {
            a.data[i] += t7 * rateOfChange.data[i];
        }

        f_obj = &obj->rightFootFilter;
        rateOfChange = f_obj->signalRateOfChange;
        for (i = 0; i < 3; i++) {
            rateOfChange.data[i] = -rateOfChange.data[i];
        }

        for (i = 0; i < 3; i++) {
            w[i] = obj->centerOfMassVelocity.data[i];
        }

        t7 = 0.0025 * fmax(fmin((b_obj_data[2] - -32.68692) / -130.74768, 1.0),
                           0.0);
        for (i = 0; i < 3; i++) {
            rateOfChange.data[i] -= w[i];
            a.data[i] += t7 * rateOfChange.data[i];
        }

        obj->centerOfMassVelocity = a;
    }
}

static Frame_5 *Frame_Frame(Frame_5 *obj, WorldFrame *b_parentFrame)
{
    Frame_5 *b_obj;
    b_obj = obj;
    c_Frame_Frame(&b_obj, b_parentFrame);
    return b_obj;
}

static void Frame_getSpatialVelocity(Frame_3 *obj, ZUpFrame *relativeTo,
    ZUpFrame *inCoordinatesOf, double spatialVelocity[6])
{
    Frame_3 *b_obj;
    int i;
    int i59;
    double relativeTransform_position_data[3];
    double relativeTransform_rotation_data[9];
    int i60;
    Frame_2 *b_parentFrame;
    int i61;
    double b[6];
    int i62;
    double R_T[9];
    ZUpFrame *c_obj;
    double b_relativeSpatialVelocity[6];
    double b_relativeTransform_position_da[3];
    Transform3d r13;
    Transform3d r14;
    double a_position_data[3];
    double a_rotation_data[9];
    double d6;
    double R_data[9];
    double p_data[3];
    Rotation3d a_rotation;
    double transform_position_data[3];
    double transform_rotation_data[9];
    bool b_bool;
    char a[14];
    char b_b[14];
    int exitg1;
    double c_b[6];
    Transform3d unusedExpr;
    Transform3d r15;
    Transform3d b_unusedExpr;
    Transform3d r16;
    double b_R_data[9];
    int i63;
    int i64;
    double c_data[9];
    double b_R_T[36];
    double b_transform_rotation_data[36];
    double c_transform_rotation_data[6];
    b_obj = obj;
    for (i = 0; i < 3; i++) {
        relativeTransform_position_data[i] =
            b_obj->relativeTransform.position.data[i];
    }

    for (i59 = 0; i59 < 9; i59++) {
        relativeTransform_rotation_data[i59] =
            b_obj->relativeTransform.rotation.data[i59];
    }

    i59 = 0;
    for (i60 = 0; i60 < 3; i60++) {
        i61 = 0;
        for (i62 = 0; i62 < 3; i62++) {
            R_T[i62 + i59] = relativeTransform_rotation_data[i61 + i60];
            i61 += 3;
        }

        i59 += 3;
    }

    b_parentFrame = b_obj->parentFrame;
    Frame_getWorldSpatialVelocity(b_parentFrame, b);
    for (i = 0; i < 6; i++) {
        b_relativeSpatialVelocity[i] = b_obj->relativeSpatialVelocity[i];
    }

    c_obj = relativeTo;
    b_obj = obj;
    b_parentFrame = b_obj->parentFrame;
    for (i = 0; i < 3; i++) {
        b_relativeTransform_position_da[i] =
            b_obj->relativeTransform.position.data[i];
    }

    for (i59 = 0; i59 < 9; i59++) {
        relativeTransform_rotation_data[i59] =
            b_obj->relativeTransform.rotation.data[i59];
    }

    b_Frame_getWorldTransform(b_parentFrame, &r13);
    for (i = 0; i < 3; i++) {
        a_position_data[i] = r13.position.data[i];
    }

    b_Frame_getWorldTransform(b_parentFrame, &r14);
    memcpy(&a_rotation_data[0], &r14.rotation.data[0], 9U * sizeof(double));
    for (i59 = 0; i59 < 3; i59++) {
        d6 = 0.0;
        i60 = 0;
        for (i61 = 0; i61 < 3; i61++) {
            d6 += a_rotation_data[i60 + i59] *
                b_relativeTransform_position_da[i61];
            R_data[i60 + i59] = 0.0;
            i62 = 0;
            for (i = 0; i < 3; i++) {
                R_data[i60 + i59] += a_rotation_data[i62 + i59] *
                    relativeTransform_rotation_data[i + i60];
                i62 += 3;
            }

            i60 += 3;
        }

        p_data[i59] = a_position_data[i59] + d6;
    }

    for (i = 0; i < 3; i++) {
        b_relativeTransform_position_da[i] = p_data[i];
    }

    memcpy(&relativeTransform_rotation_data[0], &R_data[0], 9U * sizeof(double));
    Frame_getWorldTransform(c_obj, a_position_data, &a_rotation);
    memcpy(&a_rotation_data[0], &a_rotation.data[0], 9U * sizeof(double));
    Transform3d_mldivide(b_relativeTransform_position_da,
                         relativeTransform_rotation_data, a_position_data,
                         a_rotation_data, transform_position_data,
                         transform_rotation_data);
    for (i59 = 0; i59 < 14; i59++) {
        a[i59] = b_obj->name[i59];
        b_b[i59] = b_obj->name[i59];
    }

    b_bool = false;
    i = 0;
    do {
        exitg1 = 0;
        if (i + 1 < 15) {
            if (a[i] != b_b[i]) {
                exitg1 = 1;
            } else {
                i++;
            }
        } else {
            b_bool = true;
            exitg1 = 1;
        }
    } while (exitg1 == 0);

    if (!b_bool) {
        b_parentFrame = b_obj->parentFrame;
        for (i59 = 0; i59 < 9; i59++) {
            relativeTransform_rotation_data[i59] =
                b_obj->relativeTransform.rotation.data[i59];
        }

        b_Frame_getWorldTransform(b_parentFrame, &unusedExpr);
        b_Frame_getWorldTransform(b_parentFrame, &r15);
        memcpy(&a_rotation_data[0], &r15.rotation.data[0], 9U * sizeof(double));
        for (i59 = 0; i59 < 3; i59++) {
            i60 = 0;
            for (i61 = 0; i61 < 3; i61++) {
                R_data[i60 + i59] = 0.0;
                i62 = 0;
                for (i = 0; i < 3; i++) {
                    R_data[i60 + i59] += a_rotation_data[i62 + i59] *
                        relativeTransform_rotation_data[i + i60];
                    i62 += 3;
                }

                i60 += 3;
            }
        }

        b_parentFrame = b_obj->parentFrame;
        for (i59 = 0; i59 < 9; i59++) {
            relativeTransform_rotation_data[i59] =
                b_obj->relativeTransform.rotation.data[i59];
        }

        b_Frame_getWorldTransform(b_parentFrame, &b_unusedExpr);
        b_Frame_getWorldTransform(b_parentFrame, &r16);
        memcpy(&a_rotation_data[0], &r16.rotation.data[0], 9U * sizeof(double));
        for (i59 = 0; i59 < 3; i59++) {
            i60 = 0;
            for (i61 = 0; i61 < 3; i61++) {
                b_R_data[i60 + i59] = 0.0;
                i62 = 0;
                for (i = 0; i < 3; i++) {
                    b_R_data[i60 + i59] += a_rotation_data[i62 + i59] *
                        relativeTransform_rotation_data[i + i60];
                    i62 += 3;
                }

                i60 += 3;
            }
        }

        i59 = 0;
        for (i60 = 0; i60 < 3; i60++) {
            d6 = 0.0;
            i61 = 0;
            for (i62 = 0; i62 < 3; i62++) {
                c_data[i61 + i60] = 0.0;
                for (i = 0; i < 3; i++) {
                    c_data[i61 + i60] += R_data[i + i59] * b_R_data[i + i61];
                }

                d6 += c_data[i61 + i60] * transform_position_data[i62];
                i61 += 3;
            }

            p_data[i60] = d6;
            i59 += 3;
        }

        for (i = 0; i < 3; i++) {
            i59 = 0;
            for (i60 = 0; i60 < 3; i60++) {
                R_data[i59 + i] = 0.0;
                i61 = 0;
                for (i62 = 0; i62 < 3; i62++) {
                    R_data[i59 + i] += c_data[i61 + i] *
                        transform_rotation_data[i62 + i59];
                    i61 += 3;
                }

                i59 += 3;
            }

            transform_position_data[i] = p_data[i];
        }

        memcpy(&transform_rotation_data[0], &R_data[0], 9U * sizeof(double));
    }

    b_Frame_getWorldSpatialVelocity(relativeTo, c_b);
    i59 = 0;
    for (i60 = 0; i60 < 3; i60++) {
        for (i61 = 0; i61 < 3; i61++) {
            relativeTransform_rotation_data[i61 + i59] = -R_T[i61 + i59];
        }

        i59 += 3;
    }

    a_rotation_data[0] = 0.0;
    a_rotation_data[3] = -relativeTransform_position_data[2];
    a_rotation_data[6] = relativeTransform_position_data[1];
    a_rotation_data[1] = relativeTransform_position_data[2];
    a_rotation_data[4] = 0.0;
    a_rotation_data[7] = -relativeTransform_position_data[0];
    a_rotation_data[2] = -relativeTransform_position_data[1];
    a_rotation_data[5] = relativeTransform_position_data[0];
    a_rotation_data[8] = 0.0;
    i59 = 0;
    i60 = 0;
    for (i61 = 0; i61 < 3; i61++) {
        i62 = 0;
        for (i = 0; i < 3; i++) {
            R_data[i62 + i61] = 0.0;
            i63 = 0;
            for (i64 = 0; i64 < 3; i64++) {
                R_data[i62 + i61] += relativeTransform_rotation_data[i63 + i61] *
                    a_rotation_data[i64 + i62];
                i63 += 3;
            }

            b_R_T[i + i59] = R_T[i + i60];
            i62 += 3;
        }

        i59 += 6;
        i60 += 3;
    }

    a_rotation_data[0] = 0.0;
    a_rotation_data[3] = -transform_position_data[2];
    a_rotation_data[6] = transform_position_data[1];
    a_rotation_data[1] = transform_position_data[2];
    a_rotation_data[4] = 0.0;
    a_rotation_data[7] = -transform_position_data[0];
    a_rotation_data[2] = -transform_position_data[1];
    a_rotation_data[5] = transform_position_data[0];
    a_rotation_data[8] = 0.0;
    i59 = 0;
    i60 = 0;
    for (i61 = 0; i61 < 3; i61++) {
        i62 = 0;
        for (i = 0; i < 3; i++) {
            b_R_T[(i + i59) + 18] = R_data[i + i60];
            b_R_T[(i + i59) + 3] = 0.0;
            b_R_T[(i + i59) + 21] = R_T[i + i60];
            relativeTransform_rotation_data[i62 + i61] = 0.0;
            i63 = 0;
            for (i64 = 0; i64 < 3; i64++) {
                relativeTransform_rotation_data[i62 + i61] +=
                    a_rotation_data[i63 + i61] * transform_rotation_data[i64 +
                    i62];
                i63 += 3;
            }

            b_transform_rotation_data[i + i59] = transform_rotation_data[i + i60];
            i62 += 3;
        }

        i59 += 6;
        i60 += 3;
    }

    i59 = 0;
    i60 = 0;
    for (i61 = 0; i61 < 3; i61++) {
        for (i62 = 0; i62 < 3; i62++) {
            b_transform_rotation_data[(i62 + i59) + 18] =
                relativeTransform_rotation_data[i62 + i60];
            b_transform_rotation_data[(i62 + i59) + 3] = 0.0;
            b_transform_rotation_data[(i62 + i59) + 21] =
                transform_rotation_data[i62 + i60];
        }

        i59 += 6;
        i60 += 3;
    }

    for (i59 = 0; i59 < 6; i59++) {
        d6 = 0.0;
        for (i60 = 0; i60 < 6; i60++) {
            d6 += b_R_T[i59 + 6 * i60] * b[i60];
        }

        c_transform_rotation_data[i59] = 0.0;
        for (i60 = 0; i60 < 6; i60++) {
            c_transform_rotation_data[i59] += b_transform_rotation_data[i59 + 6 *
                i60] * c_b[i60];
        }

        spatialVelocity[i59] = (d6 + b_relativeSpatialVelocity[i59]) -
            c_transform_rotation_data[i59];
    }

    c_Frame_getTransform(obj, inCoordinatesOf, b_relativeTransform_position_da,
                         relativeTransform_rotation_data);
    for (i59 = 0; i59 < 3; i59++) {
        relativeTransform_position_data[i59] = 0.0;
        b_relativeTransform_position_da[i59] = 0.0;
        i60 = 0;
        for (i61 = 0; i61 < 3; i61++) {
            relativeTransform_position_data[i59] +=
                relativeTransform_rotation_data[i60 + i59] * spatialVelocity[i61];
            b_relativeTransform_position_da[i59] +=
                relativeTransform_rotation_data[i60 + i59] * spatialVelocity[i61
                + 3];
            i60 += 3;
        }
    }

    for (i59 = 0; i59 < 3; i59++) {
        spatialVelocity[i59] = relativeTransform_position_data[i59];
        spatialVelocity[i59 + 3] = b_relativeTransform_position_da[i59];
    }
}

static void Frame_getTransform(Frame_2 *obj, ZUpFrame *withRespectTo, double
    transform_position_data[3], double transform_rotation_data[9])
{
    ZUpFrame *inCoordinatesOf;
    double obj_position_data[3];
    Rotation3d obj_rotation;
    double obj_rotation_data[9];
    Transform3d r2;
    int i;
    Transform3d r3;
    double b_obj_position_data[3];
    double b_obj_rotation_data[9];
    ZUpFrame *a;
    bool b_bool;
    char b_a[11];
    char b[11];
    int exitg1;
    double d0;
    int i20;
    double R_data[9];
    double c_data[9];
    int i21;
    inCoordinatesOf = withRespectTo;
    Frame_getWorldTransform(withRespectTo, obj_position_data, &obj_rotation);
    memcpy(&obj_rotation_data[0], &obj_rotation.data[0], 9U * sizeof(double));
    b_Frame_getWorldTransform(obj, &r2);
    for (i = 0; i < 3; i++) {
        b_obj_position_data[i] = r2.position.data[i];
    }

    b_Frame_getWorldTransform(obj, &r3);
    memcpy(&b_obj_rotation_data[0], &r3.rotation.data[0], 9U * sizeof(double));
    Transform3d_mldivide(obj_position_data, obj_rotation_data,
                         b_obj_position_data, b_obj_rotation_data,
                         transform_position_data, transform_rotation_data);
    a = withRespectTo;
    for (i = 0; i < 11; i++) {
        b_a[i] = a->name[i];
        b[i] = inCoordinatesOf->name[i];
    }

    b_bool = false;
    i = 0;
    do {
        exitg1 = 0;
        if (i + 1 < 12) {
            if (b_a[i] != b[i]) {
                exitg1 = 1;
            } else {
                i++;
            }
        } else {
            b_bool = true;
            exitg1 = 1;
        }
    } while (exitg1 == 0);

    if (!b_bool) {
        Frame_getWorldTransform(inCoordinatesOf, obj_position_data,
                                &obj_rotation);
        memcpy(&obj_rotation_data[0], &obj_rotation.data[0], 9U * sizeof(double));
        Frame_getWorldTransform(withRespectTo, b_obj_position_data,
                                &obj_rotation);
        memcpy(&b_obj_rotation_data[0], &obj_rotation.data[0], 9U * sizeof
               (double));
        for (i = 0; i < 3; i++) {
            d0 = 0.0;
            for (i20 = 0; i20 < 3; i20++) {
                c_data[i + 3 * i20] = 0.0;
                for (i21 = 0; i21 < 3; i21++) {
                    c_data[i + 3 * i20] += obj_rotation_data[i21 + 3 * i] *
                        b_obj_rotation_data[i21 + 3 * i20];
                }

                d0 += c_data[i + 3 * i20] * transform_position_data[i20];
            }

            obj_position_data[i] = d0;
            for (i20 = 0; i20 < 3; i20++) {
                R_data[i + 3 * i20] = 0.0;
                for (i21 = 0; i21 < 3; i21++) {
                    R_data[i + 3 * i20] += c_data[i + 3 * i21] *
                        transform_rotation_data[i21 + 3 * i20];
                }
            }
        }

        for (i = 0; i < 3; i++) {
            transform_position_data[i] = obj_position_data[i];
        }

        memcpy(&transform_rotation_data[0], &R_data[0], 9U * sizeof(double));
    }
}

static void Frame_getWorldSpatialVelocity(Frame_2 *obj, double
    worldSpatialVelocity[6])
{
    Frame_2 *b_obj;
    int i;
    int i65;
    double relativeTransform_position_data[3];
    double relativeTransform_rotation_data[9];
    int i66;
    Frame_1 *b_parentFrame;
    int b_i;
    double R_T[9];
    double b_relativeTransform_position_da[3];
    Frame *c_parentFrame;
    double c_relativeTransform_position_da[3];
    double b_R_T[9];
    WorldFrame *d_parentFrame;
    double c_R_T[9];
    double b_relativeSpatialVelocity[6];
    double c_relativeSpatialVelocity[6];
    double d_relativeSpatialVelocity[6];
    double dv14[9];
    double d_R_T[9];
    int i67;
    int i68;
    double e_R_T[9];
    double f_R_T[9];
    int i69;
    double d7;
    double g_R_T[36];
    double h_R_T[36];
    double i_R_T[6];
    b_obj = obj;
    for (i = 0; i < 3; i++) {
        relativeTransform_position_data[i] =
            b_obj->relativeTransform.position.data[i];
    }

    for (i65 = 0; i65 < 9; i65++) {
        relativeTransform_rotation_data[i65] =
            b_obj->relativeTransform.rotation.data[i65];
    }

    i65 = 0;
    for (i66 = 0; i66 < 3; i66++) {
        i = 0;
        for (b_i = 0; b_i < 3; b_i++) {
            R_T[b_i + i65] = relativeTransform_rotation_data[i + i66];
            i += 3;
        }

        i65 += 3;
    }

    b_obj = obj;
    b_parentFrame = b_obj->parentFrame;
    for (i = 0; i < 3; i++) {
        b_relativeTransform_position_da[i] =
            b_parentFrame->relativeTransform.position.data[i];
    }

    for (i65 = 0; i65 < 9; i65++) {
        relativeTransform_rotation_data[i65] =
            b_parentFrame->relativeTransform.rotation.data[i65];
    }

    c_parentFrame = b_parentFrame->parentFrame;
    i = 0;
    for (b_i = 0; b_i < 3; b_i++) {
        i65 = 0;
        for (i66 = 0; i66 < 3; i66++) {
            b_R_T[i66 + i] = relativeTransform_rotation_data[i65 + b_i];
            i65 += 3;
        }

        c_relativeTransform_position_da[b_i] =
            c_parentFrame->relativeTransform.position.data[b_i];
        i += 3;
    }

    for (i65 = 0; i65 < 9; i65++) {
        relativeTransform_rotation_data[i65] =
            c_parentFrame->relativeTransform.rotation.data[i65];
    }

    i65 = 0;
    for (i66 = 0; i66 < 3; i66++) {
        i = 0;
        for (b_i = 0; b_i < 3; b_i++) {
            c_R_T[b_i + i65] = relativeTransform_rotation_data[i + i66];
            i += 3;
        }

        i65 += 3;
    }

    d_parentFrame = c_parentFrame->parentFrame;
    for (i = 0; i < 6; i++) {
        worldSpatialVelocity[i] = d_parentFrame->relativeSpatialVelocity[i];
        b_relativeSpatialVelocity[i] = c_parentFrame->relativeSpatialVelocity[i];
        c_relativeSpatialVelocity[i] = b_parentFrame->relativeSpatialVelocity[i];
    }

    b_obj = obj;
    for (i = 0; i < 6; i++) {
        d_relativeSpatialVelocity[i] = b_obj->relativeSpatialVelocity[i];
    }

    i65 = 0;
    for (i66 = 0; i66 < 3; i66++) {
        for (i = 0; i < 3; i++) {
            relativeTransform_rotation_data[i + i65] = -R_T[i + i65];
        }

        i65 += 3;
    }

    dv14[0] = 0.0;
    dv14[3] = -relativeTransform_position_data[2];
    dv14[6] = relativeTransform_position_data[1];
    dv14[1] = relativeTransform_position_data[2];
    dv14[4] = 0.0;
    dv14[7] = -relativeTransform_position_data[0];
    dv14[2] = -relativeTransform_position_data[1];
    dv14[5] = relativeTransform_position_data[0];
    dv14[8] = 0.0;
    i65 = 0;
    for (i66 = 0; i66 < 3; i66++) {
        i = 0;
        for (b_i = 0; b_i < 3; b_i++) {
            d_R_T[i + i66] = 0.0;
            i67 = 0;
            for (i68 = 0; i68 < 3; i68++) {
                d_R_T[i + i66] += relativeTransform_rotation_data[i67 + i66] *
                    dv14[i68 + i];
                i67 += 3;
            }

            e_R_T[b_i + i65] = -b_R_T[b_i + i65];
            i += 3;
        }

        i65 += 3;
    }

    dv14[0] = 0.0;
    dv14[3] = -b_relativeTransform_position_da[2];
    dv14[6] = b_relativeTransform_position_da[1];
    dv14[1] = b_relativeTransform_position_da[2];
    dv14[4] = 0.0;
    dv14[7] = -b_relativeTransform_position_da[0];
    dv14[2] = -b_relativeTransform_position_da[1];
    dv14[5] = b_relativeTransform_position_da[0];
    dv14[8] = 0.0;
    i65 = 0;
    for (i66 = 0; i66 < 3; i66++) {
        i = 0;
        for (b_i = 0; b_i < 3; b_i++) {
            relativeTransform_rotation_data[i + i66] = 0.0;
            i67 = 0;
            for (i68 = 0; i68 < 3; i68++) {
                relativeTransform_rotation_data[i + i66] += e_R_T[i67 + i66] *
                    dv14[i68 + i];
                i67 += 3;
            }

            f_R_T[b_i + i65] = -c_R_T[b_i + i65];
            i += 3;
        }

        i65 += 3;
    }

    dv14[0] = 0.0;
    dv14[3] = -c_relativeTransform_position_da[2];
    dv14[6] = c_relativeTransform_position_da[1];
    dv14[1] = c_relativeTransform_position_da[2];
    dv14[4] = 0.0;
    dv14[7] = -c_relativeTransform_position_da[0];
    dv14[2] = -c_relativeTransform_position_da[1];
    dv14[5] = c_relativeTransform_position_da[0];
    dv14[8] = 0.0;
    i65 = 0;
    i66 = 0;
    for (i = 0; i < 3; i++) {
        b_i = 0;
        for (i67 = 0; i67 < 3; i67++) {
            e_R_T[b_i + i] = 0.0;
            i68 = 0;
            for (i69 = 0; i69 < 3; i69++) {
                e_R_T[b_i + i] += f_R_T[i68 + i] * dv14[i69 + b_i];
                i68 += 3;
            }

            g_R_T[i67 + i65] = c_R_T[i67 + i66];
            b_i += 3;
        }

        i65 += 6;
        i66 += 3;
    }

    i65 = 0;
    i66 = 0;
    for (i = 0; i < 3; i++) {
        for (b_i = 0; b_i < 3; b_i++) {
            g_R_T[(b_i + i65) + 18] = e_R_T[b_i + i66];
            g_R_T[(b_i + i65) + 3] = 0.0;
            g_R_T[(b_i + i65) + 21] = c_R_T[b_i + i66];
            h_R_T[b_i + i65] = b_R_T[b_i + i66];
            h_R_T[(b_i + i65) + 18] = relativeTransform_rotation_data[b_i + i66];
            h_R_T[(b_i + i65) + 3] = 0.0;
            h_R_T[(b_i + i65) + 21] = b_R_T[b_i + i66];
        }

        i65 += 6;
        i66 += 3;
    }

    for (i65 = 0; i65 < 6; i65++) {
        d7 = 0.0;
        i66 = 0;
        for (i = 0; i < 6; i++) {
            d7 += g_R_T[i66 + i65] * worldSpatialVelocity[i];
            i66 += 6;
        }

        i_R_T[i65] = d7 + b_relativeSpatialVelocity[i65];
    }

    i65 = 0;
    i66 = 0;
    for (i = 0; i < 3; i++) {
        for (b_i = 0; b_i < 3; b_i++) {
            g_R_T[b_i + i65] = R_T[b_i + i66];
            g_R_T[(b_i + i65) + 18] = d_R_T[b_i + i66];
            g_R_T[(b_i + i65) + 3] = 0.0;
            g_R_T[(b_i + i65) + 21] = R_T[b_i + i66];
        }

        i65 += 6;
        i66 += 3;
    }

    for (i65 = 0; i65 < 6; i65++) {
        d7 = 0.0;
        i66 = 0;
        for (i = 0; i < 6; i++) {
            d7 += h_R_T[i66 + i65] * i_R_T[i];
            i66 += 6;
        }

        b_relativeSpatialVelocity[i65] = d7 + c_relativeSpatialVelocity[i65];
    }

    for (i65 = 0; i65 < 6; i65++) {
        d7 = 0.0;
        i66 = 0;
        for (i = 0; i < 6; i++) {
            d7 += g_R_T[i66 + i65] * b_relativeSpatialVelocity[i];
            i66 += 6;
        }

        worldSpatialVelocity[i65] = d7 + d_relativeSpatialVelocity[i65];
    }
}

static void Frame_getWorldTransform(ZUpFrame *obj, double
    worldTransform_position_data[3], Rotation3d *worldTransform_rotation)
{
    ZUpFrame *b_obj;
    Frame_2 *b_parentFrame;
    Transform3d unusedExpr;
    Transform3d r4;
    Rotation3d obj_rotation;
    int i22;
    int i23;
    Transform3d r5;
    int i24;
    int i;
    int i25;
    Transform3d r6;
    double obj_position_data[3];
    double c_data[9];
    b_obj = obj;
    b_parentFrame = b_obj->parentFrame;
    b_obj = obj;
    b_Frame_getWorldTransform(b_obj->parentFrame, &unusedExpr);
    b_Frame_getWorldTransform(b_obj->parentFrame, &r4);
    obj_rotation = r4.rotation;
    *worldTransform_rotation = obj_rotation;
    Rotation3d_removePitchAndRoll(worldTransform_rotation);
    i22 = 0;
    for (i23 = 0; i23 < 3; i23++) {
        i24 = 0;
        for (i25 = 0; i25 < 3; i25++) {
            c_data[i24 + i23] = 0.0;
            for (i = 0; i < 3; i++) {
                c_data[i24 + i23] += obj_rotation.data[i + i22] *
                    worldTransform_rotation->data[i + i24];
            }

            i24 += 3;
        }

        i22 += 3;
    }

    b_Frame_getWorldTransform(b_parentFrame, &r5);
    for (i = 0; i < 3; i++) {
        obj_position_data[i] = r5.position.data[i];
    }

    b_Frame_getWorldTransform(b_parentFrame, &r6);
    obj_rotation = r6.rotation;
    for (i = 0; i < 3; i++) {
        i22 = 0;
        for (i23 = 0; i23 < 3; i23++) {
            worldTransform_rotation->data[i22 + i] = 0.0;
            i24 = 0;
            for (i25 = 0; i25 < 3; i25++) {
                worldTransform_rotation->data[i22 + i] += obj_rotation.data[i24
                    + i] * c_data[i25 + i22];
                i24 += 3;
            }

            i22 += 3;
        }

        worldTransform_position_data[i] = obj_position_data[i];
    }
}

static void PDController_evaluate(const PDController *obj, const double q[10],
    const double dq[10], double u[10])
{
    int i;
    double kp[10];
    double kd[10];
    double q_desired[10];
    double dq_desired[10];
    double u_desired[10];
    for (i = 0; i < 10; i++) {
        kp[i] = obj->proportionalGain[i];
    }

    for (i = 0; i < 10; i++) {
        kd[i] = obj->derivativeGain[i];
    }

    for (i = 0; i < 10; i++) {
        q_desired[i] = obj->desiredMotorPosition[i];
    }

    for (i = 0; i < 10; i++) {
        dq_desired[i] = obj->desiredMotorVelocity[i];
    }

    for (i = 0; i < 10; i++) {
        u_desired[i] = kp[i] * (q_desired[i] - q[(int)obj->activeMotorIndex[i] -
                                1]) + kd[i] * (dq_desired[i] - dq[(int)
            obj->activeMotorIndex[i] - 1]);
    }

    memset(&u[0], 0, 10U * sizeof(double));
    for (i = 0; i < 10; i++) {
        u[(int)obj->activeMotorIndex[i] - 1] = u_desired[i];
    }
}

static void PDController_setDerivativeGain(PDController *obj, const double
    b_derivativeGain[10])
{
    int i;
    for (i = 0; i < 10; i++) {
        obj->derivativeGain[i] = b_derivativeGain[i];
    }
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
    static const signed char iv6[8] = { 0, 1, 2, 3, 5, 6, 7, 8 };

    double Dq[10];
    bool y;
    double u[10];
    int k;
    int j2;
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
    int j;
    double c_x[30];
    static const double dv28[10] = { -140.62750000000003, -140.62750000000003,
        -216.0, -216.0, -112.51799999999999, -140.62750000000003,
        -140.62750000000003, -216.0, -216.0, -112.51799999999999 };

    static const double dv29[10] = { 140.62750000000003, 140.62750000000003,
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
                        i] = 1000.0 * q[iv6[i]];
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
                        i] = 1000.0 * q[iv6[i]];
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
                j2 = 0;
                for (k = 0; k < 10; k++) {
                    b_messages += (double)a[j2 + i] * q[k];
                    j2 += 20;
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

                    j2 = a[i + 20 * k];
                    if (a[i + 20 * k] < 0) {
                        j2 = -1;
                    } else {
                        if (a[i + 20 * k] > 0) {
                            j2 = 1;
                        }
                    }

                    u[k] *= 1.0 - (double)(b_x == j2) * b_stateOfCharge;
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

                    j2 = a[i + 20 * k];
                    if (a[i + 20 * k] < 0) {
                        j2 = -1;
                    } else {
                        if (a[i + 20 * k] > 0) {
                            j2 = 1;
                        }
                    }

                    u[k] = (u[k] + (double)(Kp[k] * a[i + 20 * k]) * b_messages)
                        - (double)Kd[k] * Dq[k] * (double)(b_x == j2) *
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
                c_x[i] = dv28[i];
                c_x[i + 10] = b_messages;
                c_x[i + 20] = dv29[i];
            }

            b_messages = e_obj->softStart;
            for (j = 0; j < 10; j++) {
                for (k = 0; k < 3; k++) {
                    vwork[k] = c_x[j + k * 10];
                }

                sort3(1, vwork[0], 2, vwork[1], 3, vwork[2], &i, &j2);
                q[j] = b_messages * vwork[j2 - 1];
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

static void Quaternion_getRotation(const double obj_data[4], double
    rotation_data[9])
{
    rotation_data[0] = ((obj_data[0] * obj_data[0] + obj_data[1] * obj_data[1])
                        - obj_data[2] * obj_data[2]) - obj_data[3] * obj_data[3];
    rotation_data[3] = 2.0 * (obj_data[1] * obj_data[2] - obj_data[0] *
        obj_data[3]);
    rotation_data[6] = 2.0 * (obj_data[0] * obj_data[2] + obj_data[1] *
        obj_data[3]);
    rotation_data[1] = 2.0 * (obj_data[1] * obj_data[2] + obj_data[0] *
        obj_data[3]);
    rotation_data[4] = ((obj_data[0] * obj_data[0] - obj_data[1] * obj_data[1])
                        + obj_data[2] * obj_data[2]) - obj_data[3] * obj_data[3];
    rotation_data[7] = 2.0 * (obj_data[2] * obj_data[3] - obj_data[0] *
        obj_data[1]);
    rotation_data[2] = 2.0 * (obj_data[1] * obj_data[3] - obj_data[0] *
        obj_data[2]);
    rotation_data[5] = 2.0 * (obj_data[0] * obj_data[1] + obj_data[2] *
        obj_data[3]);
    rotation_data[8] = ((obj_data[0] * obj_data[0] - obj_data[1] * obj_data[1])
                        - obj_data[2] * obj_data[2]) + obj_data[3] * obj_data[3];
}

static void Quaternion_normalize(Quaternion_1 *obj)
{
    int k;
    double n;
    double y[4];
    for (k = 0; k < 4; k++) {
        y[k] = obj->data[k] * obj->data[k];
    }

    n = y[0];
    for (k = 0; k < 3; k++) {
        n += y[k + 1];
    }

    n = sqrt(n);
    for (k = 0; k < 4; k++) {
        obj->data[k] /= n;
    }
}

static void Rotation3d_getQuaternion(const double obj_data[9], double q_data[4])
{
    double trace;
    double qw;
    double qx;
    double qy;
    trace = (obj_data[0] + obj_data[4]) + obj_data[8];
    if (trace > 0.0) {
        trace = 2.0 * sqrt(trace + 1.0);
        qw = 0.25 * trace;
        qx = (obj_data[5] - obj_data[7]) / trace;
        qy = (obj_data[6] - obj_data[2]) / trace;
        trace = (obj_data[1] - obj_data[3]) / trace;
    } else if ((obj_data[0] > obj_data[4]) && (obj_data[0] > obj_data[8])) {
        trace = 2.0 * sqrt(((1.0 + obj_data[0]) - obj_data[4]) - obj_data[8]);
        qw = (obj_data[5] - obj_data[7]) / trace;
        qx = 0.25 * trace;
        qy = (obj_data[3] + obj_data[1]) / trace;
        trace = (obj_data[6] + obj_data[2]) / trace;
    } else if (obj_data[4] > obj_data[8]) {
        trace = 2.0 * sqrt(((1.0 + obj_data[4]) - obj_data[0]) - obj_data[8]);
        qw = (obj_data[6] - obj_data[2]) / trace;
        qx = (obj_data[3] + obj_data[1]) / trace;
        qy = 0.25 * trace;
        trace = (obj_data[7] + obj_data[5]) / trace;
    } else {
        trace = 2.0 * sqrt(((1.0 + obj_data[8]) - obj_data[0]) - obj_data[4]);
        qw = (obj_data[1] - obj_data[3]) / trace;
        qx = (obj_data[6] + obj_data[2]) / trace;
        qy = (obj_data[7] + obj_data[5]) / trace;
        trace *= 0.25;
    }

    q_data[0] = qw;
    q_data[1] = qx;
    q_data[2] = qy;
    q_data[3] = trace;
}

static void Rotation3d_removePitchAndRoll(Rotation3d *obj)
{
    double n;
    double R11;
    int k;
    int b_k;
    static const signed char iv11[3] = { 0, 0, 1 };

    n = sqrt(obj->data[0] * obj->data[0] + obj->data[1] * obj->data[1]);
    if (n == 0.0) {
        memset(&obj->data[0], 0, 9U * sizeof(double));
        k = 0;
        for (b_k = 0; b_k < 3; b_k++) {
            obj->data[k] = 1.0;
            k += 4;
        }
    } else {
        R11 = obj->data[0] / n;
        n = obj->data[1] / n;
        obj->data[0] = R11;
        obj->data[3] = -n;
        obj->data[6] = 0.0;
        obj->data[1] = n;
        obj->data[4] = R11;
        obj->data[7] = 0.0;
        k = 0;
        for (b_k = 0; b_k < 3; b_k++) {
            obj->data[k + 2] = iv11[b_k];
            k += 3;
        }
    }
}

static void Rotation3d_rotZ(Rotation3d *obj, double rz)
{
    double c;
    double s;
    double b_c[9];
    int i126;
    int i127;
    static const signed char iv12[3] = { 0, 0, 1 };

    int i128;
    double b_obj[9];
    int i129;
    int i130;
    c = cos(rz);
    s = sin(rz);
    b_c[0] = c;
    b_c[3] = -s;
    b_c[6] = 0.0;
    b_c[1] = s;
    b_c[4] = c;
    b_c[7] = 0.0;
    i126 = 0;
    for (i127 = 0; i127 < 3; i127++) {
        b_c[i126 + 2] = iv12[i127];
        i126 += 3;
    }

    for (i126 = 0; i126 < 3; i126++) {
        i127 = 0;
        for (i128 = 0; i128 < 3; i128++) {
            b_obj[i127 + i126] = 0.0;
            i129 = 0;
            for (i130 = 0; i130 < 3; i130++) {
                b_obj[i127 + i126] += obj->data[i129 + i126] * b_c[i130 + i127];
                i129 += 3;
            }

            i127 += 3;
        }
    }

    i126 = 0;
    for (i127 = 0; i127 < 3; i127++) {
        for (i128 = 0; i128 < 3; i128++) {
            obj->data[i128 + i126] = b_obj[i128 + i126];
        }

        i126 += 3;
    }
}

static void SmoothRateLimiter_updateSignal(SmoothRateLimiter *obj, double value)
{
    double A;
    double b_x;
    double B;
    double t;
    double dqs_idx_0;
    double c_x[3];
    int unusedU4;
    int j2;
    double c[4];
    double dv12[16];
    A = obj->rateLimit;
    b_x = value - obj->signalValue;
    B = obj->rateLimit;
    t = obj->sampleTime;
    b_x = fmax(A / 2.0, sqrt(fabs(b_x))) / B;
    B = obj->signalValue;
    dqs_idx_0 = obj->signalRateOfChange;
    c_x[0] = 0.0;
    c_x[1] = t;
    c_x[2] = b_x;
    sort3(1, 0.0, 2, t, 3, b_x, &unusedU4, &j2);
    A = 0.0;
    if ((c_x[j2 - 1] >= 0.0) && (c_x[j2 - 1] <= b_x)) {
        c[0] = B;
        c[1] = dqs_idx_0;
        c[2] = value;
        c[3] = 0.0;
        dv12[0] = 1.0;
        dv12[4] = 0.0;
        dv12[8] = 0.0;
        dv12[12] = pow(0.0, 3.0);
        dv12[1] = 0.0;
        dv12[5] = 1.0;
        dv12[9] = 0.0;
        dv12[13] = 0.0;
        dv12[2] = 1.0;
        dv12[6] = b_x;
        dv12[10] = b_x * b_x;
        dv12[14] = pow(b_x, 3.0);
        dv12[3] = 0.0;
        dv12[7] = 1.0;
        dv12[11] = 2.0 * b_x;
        dv12[15] = 3.0 * (b_x * b_x);
        c_mldivide(dv12, c);
        B = ((c[0] + c[1] * c_x[j2 - 1]) + c[2] * (c_x[j2 - 1] * c_x[j2 - 1])) +
            c[3] * pow(c_x[j2 - 1], 3.0);
        A = (c[1] + 2.0 * c[2] * c_x[j2 - 1]) + 3.0 * c[3] * (c_x[j2 - 1] *
            c_x[j2 - 1]);
    }

    obj->signalValue = B;
    obj->signalRateOfChange = A;
}

static void StandingController_eval(StandingController *obj, double tau[10])
{
    CassieOutputs *b_obj;
    int i;
    SmoothRateLimiter *c_obj;
    double b_radioChannel[16];
    double legLength;
    double yPosition;
    double yOffset;
    double leftFootTransform_position_data[3];
    double leftFootTransform_rotation_data[9];
    Frame_4 *d_obj;
    Frame_2 *b_parentFrame;
    int i48;
    double relativeTransform_position_data[3];
    Transform3d a;
    double R_T[9];
    Quaternion_1 q_avg;
    double scale;
    double R_data[9];
    double b_data[4];
    int i49;
    double p_data[3];
    double centerOfMassVelocity_data[3];
    int i50;
    int i51;
    Frame_8 *e_obj;
    double c_data[3];
    double v_com[6];
    double v_cos[6];
    double centerOfMassPosition_data[3];
    double leftFootNormalForce;
    double rightFootNormalForce;
    double u_desired[10];
    static const short Kp_sit[5] = { 500, 500, 500, 700, 2 };

    static const short b[5] = { -100, -100, -100, -300, 0 };

    PDController *f_obj;
    static const double b_derivativeGain[10] = { 4.0, 4.0, 10.0, 10.0, 0.2, 4.0,
        4.0, 10.0, 10.0, 0.2 };

    double a_data[3];
    Transform3d g_obj;
    int k;
    Frame_6 *h_obj;
    Frame_7 *i_obj;
    double f;
    InverseKinematicsFunction *j_obj;
    NonlinearLeastSquaresSolver *k_obj;
    double q[10];
    double dv13[2];
    double J_l[42];
    double J_r[42];
    Frame_5 *relativeTo;
    Frame_5 *inCoordinatesOf;
    WorldFrame *c_parentFrame;
    double b_R_T[9];
    double transform_rotation_data[9];
    bool b_bool;
    char b_a[17];
    char b_b[17];
    int exitg1;
    double worldSpatialVelocity[6];
    double desiredLeftFootSpatialVelocity[6];
    double b_R_data[9];
    double c_R_T[9];
    int i52;
    double d_R_T[36];
    double e_R_T[36];
    double f_R_T[6];
    double b_transform_rotation_data[36];
    double b_J_l[5];
    double g_R_T[6];
    double b_J_r[5];
    double dq[10];
    b_obj = &obj->robot->outputs;
    for (i = 0; i < 16; i++) {
        b_radioChannel[i] = b_obj->data.radio[i];
    }

    c_obj = &obj->legLengthFilter;
    legLength = c_obj->signalValue;
    c_obj = &obj->pelvisYFilter;
    yPosition = c_obj->signalValue;
    yOffset = 0.04 * b_radioChannel[4];
    c_Frame_getWorldTransform(&obj->robot->leftFootFrame,
        leftFootTransform_position_data, leftFootTransform_rotation_data);
    d_obj = &obj->robot->rightFootFrame;
    b_parentFrame = d_obj->parentFrame;
    for (i = 0; i < 3; i++) {
        relativeTransform_position_data[i] =
            d_obj->relativeTransform.position.data[i];
    }

    for (i48 = 0; i48 < 9; i48++) {
        R_T[i48] = d_obj->relativeTransform.rotation.data[i48];
    }

    b_Frame_getWorldTransform(b_parentFrame, &a);
    for (i = 0; i < 3; i++) {
        scale = 0.0;
        i48 = 0;
        for (i49 = 0; i49 < 3; i49++) {
            scale += a.rotation.data[i48 + i] *
                relativeTransform_position_data[i49];
            R_data[i48 + i] = 0.0;
            i50 = 0;
            for (i51 = 0; i51 < 3; i51++) {
                R_data[i48 + i] += a.rotation.data[i50 + i] * R_T[i51 + i48];
                i50 += 3;
            }

            i48 += 3;
        }

        scale = leftFootTransform_position_data[i] + (a.position.data[i] + scale);
        p_data[i] = 0.5 * scale;
        centerOfMassVelocity_data[i] = scale;
    }

    Rotation3d_getQuaternion(leftFootTransform_rotation_data, q_avg.data);
    Rotation3d_getQuaternion(R_data, b_data);
    for (i48 = 0; i48 < 4; i48++) {
        q_avg.data[i48] += b_data[i48];
        q_avg.data[i48] *= 0.5;
    }

    Quaternion_normalize(&q_avg);
    Quaternion_getRotation(q_avg.data, R_data);
    for (i = 0; i < 3; i++) {
        a.position.data[i] = p_data[i];
    }

    memcpy(&a.rotation.data[0], &R_data[0], 9U * sizeof(double));
    e_obj = &obj->averageFootFrame;
    e_obj->relativeTransform = a;
    c_Frame_getTransform(&obj->robot->centerOfMassFrame, &obj->
                         robot->zUpPelvisFrame, a.position.data, a.rotation.data);
    c_data[0] = a.position.data[0];
    c_data[1] = a.position.data[1];
    c_data[2] = 0.0;
    d_Frame_getTransform(&obj->centerOfSupportFrame, &obj->robot->zUpPelvisFrame,
                         a.position.data, a.rotation.data);
    Frame_getSpatialVelocity(&obj->robot->centerOfMassFrame, &obj->
        robot->zUpPelvisFrame, &obj->robot->zUpPelvisFrame, v_com);
    b_Frame_getSpatialVelocity(&obj->centerOfSupportFrame, &obj->
        robot->zUpPelvisFrame, &obj->robot->zUpPelvisFrame, v_cos);
    for (i = 0; i < 3; i++) {
        centerOfMassPosition_data[i] = c_data[i] - a.position.data[i];
    }

    for (i48 = 0; i48 < 3; i48++) {
        centerOfMassVelocity_data[i48] = obj->robot->
            centerOfMassVelocity.data[i48];
    }

    for (i48 = 0; i48 < 3; i48++) {
        centerOfMassVelocity_data[i48] += v_com[i48] - v_cos[i48];
    }

    for (i48 = 0; i48 < 3; i48++) {
        relativeTransform_position_data[i48] = obj->robot->
            leftFootForce.data[i48];
    }

    leftFootNormalForce = fmax(0.0, -relativeTransform_position_data[2] -
        16.343460000000004);
    for (i48 = 0; i48 < 3; i48++) {
        relativeTransform_position_data[i48] = obj->robot->
            rightFootForce.data[i48];
    }

    rightFootNormalForce = fmax(0.0, -relativeTransform_position_data[2] -
        16.343460000000004);
    e_Frame_getTransform(&obj->robot->leftFootFrame, &obj->robot->zUpPelvisFrame,
                         a.position.data, a.rotation.data);
    e_Frame_getTransform(&obj->robot->rightFootFrame, &obj->
                         robot->zUpPelvisFrame, leftFootTransform_position_data,
                         leftFootTransform_rotation_data);
    c_StandingController_updateDesi(obj, 0.0);
    scale = fmax(fmin((legLength - 0.45) / 0.6, 1.0), 0.0);
    for (i = 0; i < 5; i++) {
        u_desired[i] = (double)Kp_sit[i] + scale * (double)b[i];
        u_desired[i + 5] = (double)Kp_sit[i] + scale * (double)b[i];
    }

    c_PDController_setProportionalG(obj->pdController, u_desired);
    f_obj = obj->pdController;
    for (i = 0; i < 10; i++) {
        f_obj->derivativeGain[i] = b_derivativeGain[i];
    }

    scale = fmax(fmin(0.4 * (centerOfMassPosition_data[1] - yPosition / 2.0) +
                      0.15 * centerOfMassVelocity_data[1], 0.1), -0.1);
    a_data[0] = c_data[0];
    a_data[1] = c_data[1] + ((yOffset + 0.16) + (1.0 - fmin(leftFootNormalForce /
        163.43460000000002, 1.0)) * fmax(-centerOfMassPosition_data[1], 0.0));
    a_data[2] = -scale + 0.9 * fmax(fmin(a.position.data[2] -
        leftFootTransform_position_data[2], 0.4), 0.0);
    for (i = 0; i < 3; i++) {
        g_obj.position.data[i] = a_data[i];
    }

    memset(&g_obj.rotation.data[0], 0, 9U * sizeof(double));
    i = 0;
    for (k = 0; k < 3; k++) {
        g_obj.rotation.data[i] = 1.0;
        i += 4;
    }

    h_obj = &obj->desiredLeftFootFrame;
    h_obj->relativeTransform = g_obj;
    p_data[0] = c_data[0];
    p_data[1] = c_data[1] + ((yOffset - 0.16) + (1.0 - fmin(rightFootNormalForce
        / 163.43460000000002, 1.0)) * fmin(-centerOfMassPosition_data[1], 0.0));
    p_data[2] = scale + 0.9 * fmax(fmin(leftFootTransform_position_data[2] -
        a.position.data[2], 0.4), 0.0);
    for (i = 0; i < 3; i++) {
        a.position.data[i] = p_data[i];
    }

    memset(&a.rotation.data[0], 0, 9U * sizeof(double));
    i = 0;
    for (k = 0; k < 3; k++) {
        a.rotation.data[i] = 1.0;
        i += 4;
    }

    i_obj = &obj->desiredRightFootFrame;
    i_obj->relativeTransform = a;
    f = 325.0 * centerOfMassPosition_data[0] + 100.0 *
        centerOfMassVelocity_data[0];
    j_obj = obj->ikSolver->problem->residualFunction;
    f_Frame_getTransform(&obj->desiredLeftFootFrame, &obj->desiredPelvisFrame,
                         leftFootTransform_position_data,
                         leftFootTransform_rotation_data);
    g_Frame_getTransform(&obj->desiredRightFootFrame, &obj->desiredPelvisFrame,
                         a.position.data, a.rotation.data);
    for (i48 = 0; i48 < 3; i48++) {
        j_obj->desiredState[i48] = leftFootTransform_position_data[i48];
    }

    for (i48 = 0; i48 < 3; i48++) {
        j_obj->desiredState[i48 + 3] = leftFootTransform_rotation_data[i48];
    }

    for (i48 = 0; i48 < 3; i48++) {
        j_obj->desiredState[i48 + 6] = a.position.data[i48];
    }

    for (i48 = 0; i48 < 3; i48++) {
        j_obj->desiredState[i48 + 9] = a.rotation.data[i48];
    }

    c_NonlinearLeastSquaresSolver_s(obj->ikSolver);
    k_obj = obj->ikSolver;
    for (i = 0; i < 10; i++) {
        q[i] = k_obj->x[i];
    }

    for (i = 0; i < 2; i++) {
        dv13[i] = 0.0;
    }

    generatedLeftFootJacobian(*(double (*)[5])&q[0], dv13, J_l);
    for (i = 0; i < 2; i++) {
        dv13[i] = 0.0;
    }

    generatedRightFootJacobian(*(double (*)[5])&q[5], dv13, J_r);
    f_obj = obj->pdController;
    for (i = 0; i < 10; i++) {
        f_obj->desiredMotorPosition[i] = q[i];
    }

    h_obj = &obj->desiredLeftFootFrame;
    relativeTo = &obj->desiredPelvisFrame;
    inCoordinatesOf = &obj->desiredPelvisFrame;
    for (i = 0; i < 3; i++) {
        relativeTransform_position_data[i] =
            h_obj->relativeTransform.position.data[i];
    }

    for (i48 = 0; i48 < 9; i48++) {
        R_T[i48] = h_obj->relativeTransform.rotation.data[i48];
    }

    i48 = 0;
    for (i49 = 0; i49 < 3; i49++) {
        i50 = 0;
        for (i51 = 0; i51 < 3; i51++) {
            b_R_T[i51 + i48] = R_T[i50 + i49];
            i50 += 3;
        }

        i48 += 3;
    }

    c_parentFrame = h_obj->parentFrame;
    for (i = 0; i < 6; i++) {
        v_com[i] = c_parentFrame->relativeSpatialVelocity[i];
        v_cos[i] = h_obj->relativeSpatialVelocity[i];
    }

    c_parentFrame = h_obj->parentFrame;
    a = c_parentFrame->relativeTransform;
    g_obj = h_obj->relativeTransform;
    for (i48 = 0; i48 < 3; i48++) {
        scale = 0.0;
        i49 = 0;
        for (i50 = 0; i50 < 3; i50++) {
            scale += a.rotation.data[i49 + i48] * g_obj.position.data[i50];
            R_data[i49 + i48] = 0.0;
            i51 = 0;
            for (k = 0; k < 3; k++) {
                R_data[i49 + i48] += a.rotation.data[i51 + i48] *
                    g_obj.rotation.data[k + i49];
                i51 += 3;
            }

            i49 += 3;
        }

        p_data[i48] = a.position.data[i48] + scale;
    }

    for (i = 0; i < 3; i++) {
        a.position.data[i] = p_data[i];
    }

    memcpy(&a.rotation.data[0], &R_data[0], 9U * sizeof(double));
    c_parentFrame = relativeTo->parentFrame;
    for (i = 0; i < 3; i++) {
        leftFootTransform_position_data[i] =
            c_parentFrame->relativeTransform.position.data[i];
    }

    for (i48 = 0; i48 < 9; i48++) {
        leftFootTransform_rotation_data[i48] =
            c_parentFrame->relativeTransform.rotation.data[i48];
    }

    g_obj = relativeTo->relativeTransform;
    for (i = 0; i < 3; i++) {
        scale = 0.0;
        i48 = 0;
        for (i49 = 0; i49 < 3; i49++) {
            scale += leftFootTransform_rotation_data[i48 + i] *
                g_obj.position.data[i49];
            R_data[i48 + i] = 0.0;
            i50 = 0;
            for (i51 = 0; i51 < 3; i51++) {
                R_data[i48 + i] += leftFootTransform_rotation_data[i50 + i] *
                    g_obj.rotation.data[i51 + i48];
                i50 += 3;
            }

            i48 += 3;
        }

        leftFootTransform_position_data[i] += scale;
    }

    memcpy(&leftFootTransform_rotation_data[0], &R_data[0], 9U * sizeof(double));
    Transform3d_mldivide(a.position.data, a.rotation.data,
                         leftFootTransform_position_data,
                         leftFootTransform_rotation_data,
                         centerOfMassPosition_data, transform_rotation_data);
    for (i48 = 0; i48 < 17; i48++) {
        b_a[i48] = h_obj->name[i48];
        b_b[i48] = h_obj->name[i48];
    }

    b_bool = false;
    i = 0;
    do {
        exitg1 = 0;
        if (i + 1 < 18) {
            if (b_a[i] != b_b[i]) {
                exitg1 = 1;
            } else {
                i++;
            }
        } else {
            b_bool = true;
            exitg1 = 1;
        }
    } while (exitg1 == 0);

    if (!b_bool) {
        c_parentFrame = h_obj->parentFrame;
        a = c_parentFrame->relativeTransform;
        g_obj = h_obj->relativeTransform;
        for (i48 = 0; i48 < 3; i48++) {
            i49 = 0;
            for (i50 = 0; i50 < 3; i50++) {
                R_data[i49 + i48] = 0.0;
                i51 = 0;
                for (k = 0; k < 3; k++) {
                    R_data[i49 + i48] += a.rotation.data[i51 + i48] *
                        g_obj.rotation.data[k + i49];
                    i51 += 3;
                }

                i49 += 3;
            }
        }

        c_parentFrame = h_obj->parentFrame;
        a = c_parentFrame->relativeTransform;
        g_obj = h_obj->relativeTransform;
        for (i48 = 0; i48 < 3; i48++) {
            i49 = 0;
            for (i50 = 0; i50 < 3; i50++) {
                b_R_data[i49 + i48] = 0.0;
                i51 = 0;
                for (k = 0; k < 3; k++) {
                    b_R_data[i49 + i48] += a.rotation.data[i51 + i48] *
                        g_obj.rotation.data[k + i49];
                    i51 += 3;
                }

                i49 += 3;
            }
        }

        i48 = 0;
        for (i49 = 0; i49 < 3; i49++) {
            scale = 0.0;
            i50 = 0;
            for (i51 = 0; i51 < 3; i51++) {
                c_R_T[i50 + i49] = 0.0;
                for (k = 0; k < 3; k++) {
                    c_R_T[i50 + i49] += R_data[k + i48] * b_R_data[k + i50];
                }

                scale += c_R_T[i50 + i49] * centerOfMassPosition_data[i51];
                i50 += 3;
            }

            p_data[i49] = scale;
            i48 += 3;
        }

        for (i = 0; i < 3; i++) {
            i48 = 0;
            for (i49 = 0; i49 < 3; i49++) {
                R_data[i48 + i] = 0.0;
                i50 = 0;
                for (i51 = 0; i51 < 3; i51++) {
                    R_data[i48 + i] += c_R_T[i50 + i] *
                        transform_rotation_data[i51 + i48];
                    i50 += 3;
                }

                i48 += 3;
            }

            centerOfMassPosition_data[i] = p_data[i];
        }

        memcpy(&transform_rotation_data[0], &R_data[0], 9U * sizeof(double));
    }

    g_obj = relativeTo->relativeTransform;
    i48 = 0;
    for (i49 = 0; i49 < 3; i49++) {
        i50 = 0;
        for (i51 = 0; i51 < 3; i51++) {
            R_T[i51 + i48] = g_obj.rotation.data[i50 + i49];
            i50 += 3;
        }

        i48 += 3;
    }

    c_parentFrame = relativeTo->parentFrame;
    for (i = 0; i < 6; i++) {
        worldSpatialVelocity[i] = c_parentFrame->relativeSpatialVelocity[i];
        desiredLeftFootSpatialVelocity[i] = relativeTo->
            relativeSpatialVelocity[i];
    }

    i48 = 0;
    for (i49 = 0; i49 < 3; i49++) {
        for (i50 = 0; i50 < 3; i50++) {
            leftFootTransform_rotation_data[i50 + i48] = -b_R_T[i50 + i48];
        }

        i48 += 3;
    }

    R_data[0] = 0.0;
    R_data[3] = -relativeTransform_position_data[2];
    R_data[6] = relativeTransform_position_data[1];
    R_data[1] = relativeTransform_position_data[2];
    R_data[4] = 0.0;
    R_data[7] = -relativeTransform_position_data[0];
    R_data[2] = -relativeTransform_position_data[1];
    R_data[5] = relativeTransform_position_data[0];
    R_data[8] = 0.0;
    i48 = 0;
    i49 = 0;
    for (i50 = 0; i50 < 3; i50++) {
        i51 = 0;
        for (k = 0; k < 3; k++) {
            c_R_T[i51 + i50] = 0.0;
            i = 0;
            for (i52 = 0; i52 < 3; i52++) {
                c_R_T[i51 + i50] += leftFootTransform_rotation_data[i + i50] *
                    R_data[i52 + i51];
                i += 3;
            }

            d_R_T[k + i48] = b_R_T[k + i49];
            i51 += 3;
        }

        i48 += 6;
        i49 += 3;
    }

    R_data[0] = 0.0;
    R_data[3] = -centerOfMassPosition_data[2];
    R_data[6] = centerOfMassPosition_data[1];
    R_data[1] = centerOfMassPosition_data[2];
    R_data[4] = 0.0;
    R_data[7] = -centerOfMassPosition_data[0];
    R_data[2] = -centerOfMassPosition_data[1];
    R_data[5] = centerOfMassPosition_data[0];
    R_data[8] = 0.0;
    i48 = 0;
    i49 = 0;
    for (i50 = 0; i50 < 3; i50++) {
        i51 = 0;
        for (k = 0; k < 3; k++) {
            d_R_T[(k + i48) + 18] = c_R_T[k + i49];
            d_R_T[(k + i48) + 3] = 0.0;
            d_R_T[(k + i48) + 21] = b_R_T[k + i49];
            b_R_data[i51 + i50] = 0.0;
            i = 0;
            for (i52 = 0; i52 < 3; i52++) {
                b_R_data[i51 + i50] += R_data[i + i50] *
                    transform_rotation_data[i52 + i51];
                i += 3;
            }

            leftFootTransform_rotation_data[k + i49] = -R_T[k + i49];
            i51 += 3;
        }

        i48 += 6;
        i49 += 3;
    }

    R_data[0] = 0.0;
    R_data[3] = -g_obj.position.data[2];
    R_data[6] = g_obj.position.data[1];
    R_data[1] = g_obj.position.data[2];
    R_data[4] = 0.0;
    R_data[7] = -g_obj.position.data[0];
    R_data[2] = -g_obj.position.data[1];
    R_data[5] = g_obj.position.data[0];
    R_data[8] = 0.0;
    i48 = 0;
    i49 = 0;
    for (i50 = 0; i50 < 3; i50++) {
        i51 = 0;
        for (k = 0; k < 3; k++) {
            b_R_T[i51 + i50] = 0.0;
            i = 0;
            for (i52 = 0; i52 < 3; i52++) {
                b_R_T[i51 + i50] += leftFootTransform_rotation_data[i + i50] *
                    R_data[i52 + i51];
                i += 3;
            }

            e_R_T[k + i48] = R_T[k + i49];
            i51 += 3;
        }

        i48 += 6;
        i49 += 3;
    }

    i48 = 0;
    i49 = 0;
    for (i50 = 0; i50 < 3; i50++) {
        for (i51 = 0; i51 < 3; i51++) {
            e_R_T[(i51 + i48) + 18] = b_R_T[i51 + i49];
            e_R_T[(i51 + i48) + 3] = 0.0;
            e_R_T[(i51 + i48) + 21] = R_T[i51 + i49];
            b_transform_rotation_data[i51 + i48] = transform_rotation_data[i51 +
                i49];
            b_transform_rotation_data[(i51 + i48) + 18] = b_R_data[i51 + i49];
            b_transform_rotation_data[(i51 + i48) + 3] = 0.0;
            b_transform_rotation_data[(i51 + i48) + 21] =
                transform_rotation_data[i51 + i49];
        }

        i48 += 6;
        i49 += 3;
    }

    for (i48 = 0; i48 < 6; i48++) {
        scale = 0.0;
        for (i49 = 0; i49 < 6; i49++) {
            scale += e_R_T[i48 + 6 * i49] * worldSpatialVelocity[i49];
        }

        f_R_T[i48] = scale + desiredLeftFootSpatialVelocity[i48];
        scale = 0.0;
        for (i49 = 0; i49 < 6; i49++) {
            scale += d_R_T[i48 + 6 * i49] * v_com[i49];
        }

        g_R_T[i48] = scale + v_cos[i48];
    }

    for (i48 = 0; i48 < 6; i48++) {
        worldSpatialVelocity[i48] = 0.0;
        i49 = 0;
        for (i50 = 0; i50 < 6; i50++) {
            worldSpatialVelocity[i48] += b_transform_rotation_data[i49 + i48] *
                f_R_T[i50];
            i49 += 6;
        }

        desiredLeftFootSpatialVelocity[i48] = g_R_T[i48] -
            worldSpatialVelocity[i48];
    }

    f_Frame_getTransform(h_obj, inCoordinatesOf, a.position.data,
                         a.rotation.data);
    c_Frame_getSpatialVelocity(&obj->desiredRightFootFrame,
        &obj->desiredPelvisFrame, &obj->desiredPelvisFrame, v_com);
    for (i48 = 0; i48 < 3; i48++) {
        centerOfMassPosition_data[i48] = 0.0;
        relativeTransform_position_data[i48] = 0.0;
        i49 = 0;
        for (i50 = 0; i50 < 3; i50++) {
            centerOfMassPosition_data[i48] += a.rotation.data[i49 + i48] *
                desiredLeftFootSpatialVelocity[i50];
            relativeTransform_position_data[i48] += a.rotation.data[i49 + i48] *
                desiredLeftFootSpatialVelocity[i50 + 3];
            i49 += 3;
        }

        v_cos[i48] = centerOfMassPosition_data[i48];
        v_cos[i48 + 3] = relativeTransform_position_data[i48];
    }

    b_mldivide(*(double (*)[30])&J_l[0], v_cos, b_J_l);
    b_mldivide(*(double (*)[30])&J_r[0], v_com, b_J_r);
    for (i = 0; i < 5; i++) {
        dq[i] = b_J_l[i];
        dq[i + 5] = b_J_r[i];
    }

    f_obj = obj->pdController;
    for (i = 0; i < 10; i++) {
        f_obj->desiredMotorVelocity[i] = dq[i];
    }

    b_obj = &obj->robot->outputs;
    for (i = 0; i < 10; i++) {
        q[i] = b_obj->data.motorPosition[i];
    }

    b_obj = &obj->robot->outputs;
    for (i = 0; i < 10; i++) {
        dq[i] = b_obj->data.motorVelocity[i];
    }

    f_obj = obj->pdController;
    for (i = 0; i < 10; i++) {
        scale = f_obj->proportionalGain[i];
        legLength = f_obj->derivativeGain[i];
        yPosition = f_obj->desiredMotorPosition[i];
        yOffset = f_obj->desiredMotorVelocity[i];
        u_desired[i] = scale * (yPosition - q[(int)f_obj->activeMotorIndex[i] -
                                1]) + legLength * (yOffset - dq[(int)
            f_obj->activeMotorIndex[i] - 1]);
    }

    memset(&q[0], 0, 10U * sizeof(double));
    for (i48 = 0; i48 < 10; i48++) {
        q[(int)f_obj->activeMotorIndex[i48] - 1] = u_desired[i48];
    }

    v_cos[0] = 0.0;
    v_cos[1] = 0.0;
    v_cos[2] = 0.0;
    v_cos[3] = 0.0;
    v_cos[4] = fmax(fmin(f, leftFootNormalForce * 0.063), -leftFootNormalForce *
                    0.063);
    v_cos[5] = 0.0;
    worldSpatialVelocity[0] = 0.0;
    worldSpatialVelocity[1] = 0.0;
    worldSpatialVelocity[2] = 0.0;
    worldSpatialVelocity[3] = 0.0;
    worldSpatialVelocity[4] = fmax(fmin(f, rightFootNormalForce * 0.063),
        -rightFootNormalForce * 0.063);
    worldSpatialVelocity[5] = 0.0;
    i48 = 0;
    for (i49 = 0; i49 < 5; i49++) {
        b_J_l[i49] = 0.0;
        b_J_r[i49] = 0.0;
        for (i50 = 0; i50 < 6; i50++) {
            b_J_l[i49] += J_l[i50 + i48] * v_cos[i50];
            b_J_r[i49] += J_r[i50 + i48] * worldSpatialVelocity[i50];
        }

        u_desired[i49] = b_J_l[i49];
        u_desired[i49 + 5] = b_J_r[i49];
        i48 += 6;
    }

    for (i48 = 0; i48 < 10; i48++) {
        tau[i48] = u_desired[i48] + q[i48];
    }
}

static void StandingController_setPelvisYaw(StandingController *obj, double
    pelvisYaw)
{
    SmoothRateLimiter *b_obj;
    double legLength;
    b_obj = &obj->legLengthFilter;
    legLength = b_obj->signalValue;
    legLength = fmax(fmin((legLength - 0.45) / 0.6, 1.0), 0.0);
    SmoothRateLimiter_updateSignal(&obj->pelvisYawFilter, fmax(fmin(pelvisYaw,
        0.26179938779914941 * legLength), -0.26179938779914941 * legLength));
}

static void SteppingController_eval(SteppingController *obj, double tau[10])
{
    CassieOutputs *b_obj;
    int i;
    SmoothRateLimiter *c_obj;
    double b_radioChannel[16];
    double legLength;
    LowPassFilter_1 *d_obj;
    double xVelocity;
    double yVelocity;
    double yawVelocity;
    Transform3d desiredRightFootTransform;
    Vector3d b_centerOfMassVelocity;
    Vector3d e_obj;
    double membershipLeft;
    double membershipRight;
    bool isWalking;
    double y;
    double q[10];
    static const short a[5] = { 600, 300, 600, 600, 2 };

    static const short b_a[5] = { 300, 300, 400, 400, 10 };

    static const double c_a[5] = { 4.0, 4.0, 10.0, 10.0, 0.2 };

    static const signed char d_a[5] = { 4, 4, 10, 10, 1 };

    double f;
    Transform3d desiredSwingPelvisTransform;
    Transform3d T;
    double b_data[3];
    double c_desiredLeftFootTransform_posi[3];
    double R_T[9];
    double c_data[3];
    double dv18[2];
    double b_desiredSwingPelvisTransform[2];
    double d_data[2];
    double xStance;
    double dxStance;
    double yStance;
    double dyStance;
    double dzStance;
    static const signed char iv4[9] = { 1, 0, 0, 0, 1, 0, 0, 0, 1 };

    Frame_7 *f_obj;
    double R_data[9];
    double c_desiredSwingPelvisTransform[3];
    double dv19[3];
    SteppingController *g_obj;
    double c[9];
    int i109;
    static const signed char iv5[3] = { 0, 0, 1 };

    int i110;
    double obj_data[9];
    int i111;
    int i112;
    Frame_5 *h_obj;
    WorldFrame *b_parentFrame;
    Quaternion_1 qInitialStance;
    double qDesiredStance_data[4];
    Quaternion_1 qInitialSwing;
    double qDesiredSwing_data[4];
    double dqInitialStance[6];
    double sv[6];
    double b_relativeSpatialVelocity[6];
    double dqDesiredSwing[6];
    double dqInitialSwing[6];
    double b_R_T[9];
    int i113;
    int i114;
    double c_R_T[36];
    double c_desiredStanceFootPelvisTransf[9];
    double dqStance[6];
    InverseKinematicsFunction *i_obj;
    NonlinearLeastSquaresSolver *j_obj;
    PDController *k_obj;
    double dv20[42];
    double dv21[5];
    double dv22[5];
    double dq[10];
    double g[10];
    double dv23[10];
    b_obj = &obj->robot->outputs;
    for (i = 0; i < 16; i++) {
        b_radioChannel[i] = b_obj->data.radio[i];
    }

    c_obj = &obj->legLengthFilter;
    legLength = c_obj->signalValue;
    d_obj = &obj->xVelocityFilter;
    xVelocity = d_obj->signalValue;
    d_obj = &obj->yVelocityFilter;
    yVelocity = d_obj->signalValue;
    d_obj = &obj->yawVelocityFilter;
    yawVelocity = d_obj->signalValue;
    c_Frame_getTransform(&obj->robot->centerOfMassFrame, &obj->
                         robot->zUpPelvisFrame,
                         desiredRightFootTransform.position.data,
                         desiredRightFootTransform.rotation.data);
    b_centerOfMassVelocity = obj->robot->centerOfMassVelocity;
    e_obj = obj->robot->leftFootForce;
    membershipLeft = __anon_fcn(e_obj.data[2], -32.686920000000008,
        -326.86920000000003);
    e_obj = obj->robot->rightFootForce;
    membershipRight = __anon_fcn(e_obj.data[2], -32.686920000000008,
        -326.86920000000003);
    obj->xIntegralError = b_clamp(obj->xIntegralError + 0.0005 *
        (b_centerOfMassVelocity.data[0] - xVelocity));
    obj->yIntegralError = b_clamp(obj->yIntegralError + 0.0005 *
        (b_centerOfMassVelocity.data[1] - yVelocity));
    isWalking = (fabs(xVelocity) >= 0.1);
    y = 0.06 * (double)!isWalking;
    for (i = 0; i < 5; i++) {
        q[i] = (double)a[i] * membershipLeft + (double)b_a[i] * (1.0 -
            membershipLeft);
        q[i + 5] = (double)a[i] * membershipRight + (double)b_a[i] * (1.0 -
            membershipRight);
    }

    c_PDController_setProportionalG(obj->pdController, q);
    for (i = 0; i < 5; i++) {
        q[i] = c_a[i] * membershipLeft + (double)d_a[i] * (1.0 - membershipLeft);
        q[i + 5] = c_a[i] * membershipRight + (double)d_a[i] * (1.0 -
            membershipRight);
    }

    PDController_setDerivativeGain(obj->pdController, q);
    tau[0] = 44.127342000000006 * membershipLeft;
    tau[1] = 0.0;
    tau[2] = 0.0;
    tau[3] = 0.0;
    tau[4] = 0.0;
    tau[5] = -44.127342000000006 * membershipRight;
    tau[6] = 0.0;
    tau[7] = 0.0;
    tau[8] = 0.0;
    tau[9] = 0.0;
    e_obj = obj->robot->leftFootForce;
    membershipLeft = fmax(0.0, -e_obj.data[2] - 16.343460000000004);
    e_obj = obj->robot->rightFootForce;
    membershipRight = fmax(0.0, -e_obj.data[2] - 16.343460000000004);
    if (!isWalking) {
        f = 50.0 * (b_centerOfMassVelocity.data[0] - xVelocity);
        if (obj->stanceLeg == LEFT) {
            tau[4] = clamp(-f, -membershipLeft * 0.018, membershipLeft * 0.018);
        } else {
            tau[9] = clamp(-f, -membershipRight * 0.018, membershipRight * 0.018);
        }
    }

    desiredSwingPelvisTransform = obj->initialStanceFootTransform;
    T = obj->initialStanceFootTransform;
    for (i = 0; i < 3; i++) {
        b_data[i] = obj->c_initialStanceFootSpatialVeloc[i];
    }

    for (i = 0; i < 3; i++) {
        c_data[i] = obj->c_initialStanceFootSpatialVeloc[i + 3];
    }

    e_Frame_getTransform(c_SteppingController_getStanceF(obj), &obj->
                         robot->zUpPelvisFrame, c_desiredLeftFootTransform_posi,
                         R_T);
    dv18[0] = 0.0;
    dv18[1] = 0.08 + y;
    b_desiredSwingPelvisTransform[0] =
        desiredSwingPelvisTransform.position.data[0];
    b_desiredSwingPelvisTransform[1] = c_desiredLeftFootTransform_posi[0] +
        (-legLength - c_desiredLeftFootTransform_posi[2]) *
        (c_desiredLeftFootTransform_posi[0] / c_desiredLeftFootTransform_posi[2]);
    d_data[0] = b_data[0];
    d_data[1] = -xVelocity;
    cubicInterpolation(obj->stanceTime, dv18, b_desiredSwingPelvisTransform,
                       d_data, &xStance, &dxStance, &membershipLeft);
    dv18[0] = 0.0;
    dv18[1] = 0.08 + y;
    b_desiredSwingPelvisTransform[0] =
        desiredSwingPelvisTransform.position.data[1];
    b_desiredSwingPelvisTransform[1] = c_desiredLeftFootTransform_posi[1] +
        (-legLength - c_desiredLeftFootTransform_posi[2]) *
        (c_desiredLeftFootTransform_posi[1] / c_desiredLeftFootTransform_posi[2]);
    d_data[0] = b_data[1];
    d_data[1] = -yVelocity;
    cubicInterpolation(obj->stanceTime, dv18, b_desiredSwingPelvisTransform,
                       d_data, &yStance, &dyStance, &membershipLeft);
    dv18[0] = 0.0;
    dv18[1] = 0.08 + y;
    b_desiredSwingPelvisTransform[0] =
        desiredSwingPelvisTransform.position.data[2];
    b_desiredSwingPelvisTransform[1] = 0.0;
    d_data[0] = b_data[2];
    d_data[1] = 0.0;
    cubicInterpolation(obj->stanceTime, dv18, b_desiredSwingPelvisTransform,
                       d_data, &y, &dzStance, &membershipLeft);
    membershipRight = -yawVelocity * obj->stepInterval;
    dv18[0] = 0.0;
    dv18[1] = obj->stepInterval;
    b_desiredSwingPelvisTransform[0] = atan2(T.rotation.data[1],
        T.rotation.data[0]);
    b_desiredSwingPelvisTransform[1] = membershipRight / 2.0;
    d_data[0] = c_data[2];
    d_data[1] = -yawVelocity;
    cubicInterpolation(obj->stanceTime, dv18, b_desiredSwingPelvisTransform,
                       d_data, &membershipLeft, &membershipRight, &f);
    e_obj.data[0] = xStance;
    e_obj.data[1] = yStance;
    e_obj.data[2] = y;
    desiredSwingPelvisTransform.position = e_obj;
    for (i = 0; i < 9; i++) {
        desiredSwingPelvisTransform.rotation.data[i] = iv4[i];
        R_data[i] = iv4[i];
    }

    Rotation3d_rotZ(&desiredSwingPelvisTransform.rotation, membershipLeft);
    f_obj = c_SteppingController_getDesired(obj);
    f_obj->relativeTransform = desiredSwingPelvisTransform;
    f_obj = c_SteppingController_getDesired(obj);
    f_obj->relativeSpatialVelocity[0] = dxStance;
    f_obj->relativeSpatialVelocity[1] = dyStance;
    f_obj->relativeSpatialVelocity[2] = dzStance;
    f_obj->relativeSpatialVelocity[3] = 0.0;
    f_obj->relativeSpatialVelocity[4] = 0.0;
    f_obj->relativeSpatialVelocity[5] = membershipRight;
    c_SteppingController_getDesired(obj);
    if (isWalking) {
        membershipLeft = 0.05;
        membershipRight = 0.4;
    } else {
        membershipLeft = 0.12;
        membershipRight = 0.3;
    }

    desiredSwingPelvisTransform = obj->initialSwingFootTransform;
    T = obj->initialSwingFootTransform;
    for (i = 0; i < 3; i++) {
        b_data[i] = obj->initialSwingFootSpatialVelocity[i];
    }

    for (i = 0; i < 3; i++) {
        c_data[i] = obj->initialSwingFootSpatialVelocity[i + 3];
    }

    e_obj = obj->initialPelvisVelocity;
    f = (((desiredRightFootTransform.position.data[0] + 0.18 *
           b_centerOfMassVelocity.data[0]) + 0.06 *
          (b_centerOfMassVelocity.data[0] - xVelocity)) + 0.01 *
         obj->xIntegralError) + 0.2 * (b_centerOfMassVelocity.data[0] -
        e_obj.data[0]);
    e_obj = obj->initialPelvisVelocity;
    membershipRight = ((((desiredRightFootTransform.position.data[1] + fmax(0.0,
                           0.12 - 0.04 * fabs(xVelocity)) * -(double)
                          obj->stanceLeg) + 0.16 * b_centerOfMassVelocity.data[1])
                        + membershipLeft * (b_centerOfMassVelocity.data[1] -
                         yVelocity)) + 0.01 * obj->yIntegralError) +
        membershipRight * (b_centerOfMassVelocity.data[1] - e_obj.data[1]);
    if (obj->stand == 1.0) {
        membershipRight = desiredRightFootTransform.position.data[1] + 0.18 *
            -(double)obj->stanceLeg;
    }

    f = c_clamp(f);
    membershipRight = clamp(membershipRight, c_desiredLeftFootTransform_posi[1]
                            - 0.07 * (double)obj->stanceLeg, -0.4 * (double)
                            obj->stanceLeg);
    membershipLeft = obj->stepInterval;
    for (i = 0; i < 2; i++) {
        dv18[i] = (double)i * membershipLeft;
    }

    b_desiredSwingPelvisTransform[0] =
        desiredSwingPelvisTransform.position.data[0];
    b_desiredSwingPelvisTransform[1] = f;
    d_data[0] = b_data[0];
    d_data[1] = -xVelocity;
    cubicInterpolation(obj->stanceTime, dv18, b_desiredSwingPelvisTransform,
                       d_data, &xStance, &dxStance, &membershipLeft);
    membershipLeft = obj->stepInterval;
    for (i = 0; i < 2; i++) {
        dv18[i] = (double)i * membershipLeft;
    }

    b_desiredSwingPelvisTransform[0] =
        desiredSwingPelvisTransform.position.data[1];
    b_desiredSwingPelvisTransform[1] = membershipRight;
    d_data[0] = b_data[1];
    d_data[1] = -yVelocity;
    cubicInterpolation(obj->stanceTime, dv18, b_desiredSwingPelvisTransform,
                       d_data, &yStance, &dyStance, &membershipLeft);
    membershipLeft = obj->stepInterval;
    for (i = 0; i < 3; i++) {
        dv19[i] = 0.5 * (double)i * membershipLeft;
    }

    c_desiredSwingPelvisTransform[0] =
        desiredSwingPelvisTransform.position.data[2];
    c_desiredSwingPelvisTransform[1] = 0.17;
    c_desiredSwingPelvisTransform[2] = legLength +
        c_desiredLeftFootTransform_posi[2];
    b_data[0] = b_data[2];
    b_data[1] = 0.0;
    b_data[2] = 0.0;
    b_cubicInterpolation(obj->stanceTime, dv19, c_desiredSwingPelvisTransform,
                         b_data, &y, &dzStance, &membershipLeft);
    membershipRight = yawVelocity * obj->stepInterval;
    dv18[0] = 0.0;
    dv18[1] = obj->stepInterval;
    b_desiredSwingPelvisTransform[0] = atan2(T.rotation.data[1],
        T.rotation.data[0]);
    b_desiredSwingPelvisTransform[1] = membershipRight / 2.0;
    d_data[0] = c_data[2];
    d_data[1] = -yawVelocity;
    cubicInterpolation(obj->stanceTime, dv18, b_desiredSwingPelvisTransform,
                       d_data, &membershipRight, &f, &membershipLeft);
    for (i = 0; i < 9; i++) {
        R_T[i] = iv4[i];
    }

    g_obj = obj;
    if (g_obj->stanceLeg == LEFT) {
        f_obj = &g_obj->desiredRightFootFrame;
    } else {
        f_obj = &g_obj->desiredLeftFootFrame;
    }

    e_obj.data[0] = xStance;
    e_obj.data[1] = yStance;
    e_obj.data[2] = y;
    membershipLeft = cos(membershipRight);
    membershipRight = sin(membershipRight);
    c[0] = membershipLeft;
    c[3] = -membershipRight;
    c[6] = 0.0;
    c[1] = membershipRight;
    c[4] = membershipLeft;
    c[7] = 0.0;
    i = 0;
    for (i109 = 0; i109 < 3; i109++) {
        c[i + 2] = iv5[i109];
        i += 3;
    }

    for (i = 0; i < 3; i++) {
        i109 = 0;
        for (i110 = 0; i110 < 3; i110++) {
            obj_data[i109 + i] = 0.0;
            i111 = 0;
            for (i112 = 0; i112 < 3; i112++) {
                obj_data[i109 + i] += R_T[i111 + i] * c[i112 + i109];
                i111 += 3;
            }

            i109 += 3;
        }
    }

    i = 0;
    for (i109 = 0; i109 < 3; i109++) {
        for (i110 = 0; i110 < 3; i110++) {
            R_T[i110 + i] = obj_data[i110 + i];
        }

        i += 3;
    }

    desiredRightFootTransform.position = e_obj;
    memcpy(&desiredRightFootTransform.rotation.data[0], &R_T[0], 9U * sizeof
           (double));
    f_obj->relativeTransform = desiredRightFootTransform;
    g_obj = obj;
    if (g_obj->stanceLeg == LEFT) {
        f_obj = &g_obj->desiredRightFootFrame;
    } else {
        f_obj = &g_obj->desiredLeftFootFrame;
    }

    f_obj->relativeSpatialVelocity[0] = dxStance;
    f_obj->relativeSpatialVelocity[1] = dyStance;
    f_obj->relativeSpatialVelocity[2] = dzStance;
    f_obj->relativeSpatialVelocity[3] = 0.0;
    f_obj->relativeSpatialVelocity[4] = 0.0;
    f_obj->relativeSpatialVelocity[5] = f;
    c_SteppingController_updateDesi(obj);
    h_obj = &obj->desiredPelvisFrame;
    b_parentFrame = h_obj->parentFrame;
    desiredSwingPelvisTransform = b_parentFrame->relativeTransform;
    T = h_obj->relativeTransform;
    for (i = 0; i < 3; i++) {
        membershipLeft = 0.0;
        i109 = 0;
        for (i110 = 0; i110 < 3; i110++) {
            membershipLeft += desiredSwingPelvisTransform.rotation.data[i109 + i]
                * T.position.data[i110];
            R_data[i109 + i] = 0.0;
            i111 = 0;
            for (i112 = 0; i112 < 3; i112++) {
                R_data[i109 + i] +=
                    desiredSwingPelvisTransform.rotation.data[i111 + i] *
                    T.rotation.data[i112 + i109];
                i111 += 3;
            }

            i109 += 3;
        }

        b_data[i] = desiredSwingPelvisTransform.position.data[i] +
            membershipLeft;
    }

    Frame_getTransform(&obj->robot->pelvisFrame, &obj->robot->zUpPelvisFrame,
                       desiredSwingPelvisTransform.position.data,
                       desiredSwingPelvisTransform.rotation.data);
    desiredRightFootTransform = obj->initialStancePelvisTransform;
    Rotation3d_getQuaternion(desiredRightFootTransform.rotation.data,
        qInitialStance.data);
    Rotation3d_getQuaternion(R_data, qDesiredStance_data);
    desiredRightFootTransform = obj->initialSwingPelvisTransform;
    Rotation3d_getQuaternion(desiredRightFootTransform.rotation.data,
        qInitialSwing.data);
    Rotation3d_getQuaternion(desiredSwingPelvisTransform.rotation.data,
        qDesiredSwing_data);
    for (i = 0; i < 6; i++) {
        dqInitialStance[i] = obj->c_initialStanceFootSpatialVeloc[i];
    }

    h_obj = &obj->desiredPelvisFrame;
    T = h_obj->relativeTransform;
    i = 0;
    for (i109 = 0; i109 < 3; i109++) {
        i110 = 0;
        for (i111 = 0; i111 < 3; i111++) {
            R_T[i111 + i] = T.rotation.data[i110 + i109];
            i110 += 3;
        }

        i += 3;
    }

    b_parentFrame = h_obj->parentFrame;
    for (i = 0; i < 6; i++) {
        sv[i] = b_parentFrame->relativeSpatialVelocity[i];
        b_relativeSpatialVelocity[i] = h_obj->relativeSpatialVelocity[i];
    }

    for (i = 0; i < 6; i++) {
        dqInitialSwing[i] = obj->initialSwingFootSpatialVelocity[i];
    }

    d_Frame_getSpatialVelocity(&obj->robot->pelvisFrame, &obj->
        robot->zUpPelvisFrame, &obj->robot->pelvisFrame, dqDesiredSwing);
    membershipRight = 2.0 * obj->stanceTime;
    membershipLeft = obj->stepInterval;
    membershipRight = fmax(fmin(membershipRight / membershipLeft, 1.0), 0.0);
    for (i = 0; i < 4; i++) {
        qInitialStance.data[i] += membershipRight * (qDesiredStance_data[i] -
            qInitialStance.data[i]);
    }

    i = 0;
    for (i109 = 0; i109 < 3; i109++) {
        for (i110 = 0; i110 < 3; i110++) {
            c[i110 + i] = -R_T[i110 + i];
        }

        i += 3;
    }

    obj_data[0] = 0.0;
    obj_data[3] = -T.position.data[2];
    obj_data[6] = T.position.data[1];
    obj_data[1] = T.position.data[2];
    obj_data[4] = 0.0;
    obj_data[7] = -T.position.data[0];
    obj_data[2] = -T.position.data[1];
    obj_data[5] = T.position.data[0];
    obj_data[8] = 0.0;
    i = 0;
    i109 = 0;
    for (i110 = 0; i110 < 3; i110++) {
        i111 = 0;
        for (i112 = 0; i112 < 3; i112++) {
            b_R_T[i111 + i110] = 0.0;
            i113 = 0;
            for (i114 = 0; i114 < 3; i114++) {
                b_R_T[i111 + i110] += c[i113 + i110] * obj_data[i114 + i111];
                i113 += 3;
            }

            c_R_T[i112 + i] = R_T[i112 + i109];
            i111 += 3;
        }

        i += 6;
        i109 += 3;
    }

    i = 0;
    i109 = 0;
    for (i110 = 0; i110 < 3; i110++) {
        for (i111 = 0; i111 < 3; i111++) {
            c_R_T[(i111 + i) + 18] = b_R_T[i111 + i109];
            c_R_T[(i111 + i) + 3] = 0.0;
            c_R_T[(i111 + i) + 21] = R_T[i111 + i109];
        }

        i += 6;
        i109 += 3;
    }

    for (i = 0; i < 6; i++) {
        membershipLeft = 0.0;
        i109 = 0;
        for (i110 = 0; i110 < 6; i110++) {
            membershipLeft += c_R_T[i109 + i] * sv[i110];
            i109 += 6;
        }

        dqStance[i] = dqInitialStance[i] + membershipRight * ((membershipLeft +
            b_relativeSpatialVelocity[i]) - dqInitialStance[i]);
    }

    Quaternion_normalize(&qInitialStance);
    Quaternion_getRotation(qInitialStance.data, R_data);
    for (i = 0; i < 3; i++) {
        c_data[i] = b_data[i];
    }

    memcpy(&c_desiredStanceFootPelvisTransf[0], &R_data[0], 9U * sizeof(double));
    membershipRight = 2.0 * obj->stanceTime;
    membershipLeft = obj->stepInterval;
    membershipRight = fmax(fmin(membershipRight / membershipLeft, 1.0), 0.0);
    for (i = 0; i < 4; i++) {
        qInitialSwing.data[i] += membershipRight * (qDesiredSwing_data[i] -
            qInitialSwing.data[i]);
    }

    for (i = 0; i < 6; i++) {
        dqInitialSwing[i] += membershipRight * (dqDesiredSwing[i] -
            dqInitialSwing[i]);
    }

    Quaternion_normalize(&qInitialSwing);
    Quaternion_getRotation(qInitialSwing.data, R_data);
    for (i = 0; i < 3; i++) {
        desiredSwingPelvisTransform.position.data[i] = b_data[i];
    }

    memcpy(&desiredSwingPelvisTransform.rotation.data[0], &R_data[0], 9U *
           sizeof(double));
    h_obj = &obj->desiredPelvisFrame;
    T = h_obj->relativeTransform;
    h_obj = &obj->desiredPelvisFrame;
    for (i = 0; i < 6; i++) {
        sv[i] = h_obj->relativeSpatialVelocity[i];
    }

    if (obj->stanceLeg == LEFT) {
        h_obj = &obj->desiredPelvisFrame;
        for (i = 0; i < 3; i++) {
            h_obj->relativeTransform.position.data[i] = c_data[i];
        }

        for (i = 0; i < 9; i++) {
            h_obj->relativeTransform.rotation.data[i] =
                c_desiredStanceFootPelvisTransf[i];
        }

        h_obj = &obj->desiredPelvisFrame;
        for (i = 0; i < 6; i++) {
            h_obj->relativeSpatialVelocity[i] = dqStance[i];
        }

        g_Frame_getTransform(&obj->desiredLeftFootFrame,
                             &obj->desiredPelvisFrame,
                             c_desiredLeftFootTransform_posi, R_T);
        h_obj = &obj->desiredPelvisFrame;
        h_obj->relativeTransform = desiredSwingPelvisTransform;
        h_obj = &obj->desiredPelvisFrame;
        for (i = 0; i < 6; i++) {
            h_obj->relativeSpatialVelocity[i] = dqInitialSwing[i];
        }

        g_Frame_getTransform(&obj->desiredRightFootFrame,
                             &obj->desiredPelvisFrame,
                             desiredRightFootTransform.position.data,
                             desiredRightFootTransform.rotation.data);
    } else {
        h_obj = &obj->desiredPelvisFrame;
        h_obj->relativeTransform = desiredSwingPelvisTransform;
        h_obj = &obj->desiredPelvisFrame;
        for (i = 0; i < 6; i++) {
            h_obj->relativeSpatialVelocity[i] = dqInitialSwing[i];
        }

        g_Frame_getTransform(&obj->desiredLeftFootFrame,
                             &obj->desiredPelvisFrame,
                             c_desiredLeftFootTransform_posi, R_T);
        h_obj = &obj->desiredPelvisFrame;
        for (i = 0; i < 3; i++) {
            h_obj->relativeTransform.position.data[i] = c_data[i];
        }

        for (i = 0; i < 9; i++) {
            h_obj->relativeTransform.rotation.data[i] =
                c_desiredStanceFootPelvisTransf[i];
        }

        h_obj = &obj->desiredPelvisFrame;
        for (i = 0; i < 6; i++) {
            h_obj->relativeSpatialVelocity[i] = dqStance[i];
        }

        g_Frame_getTransform(&obj->desiredRightFootFrame,
                             &obj->desiredPelvisFrame,
                             desiredRightFootTransform.position.data,
                             desiredRightFootTransform.rotation.data);
    }

    i_obj = obj->ikSolver->problem->residualFunction;
    for (i = 0; i < 3; i++) {
        i_obj->desiredState[i] = c_desiredLeftFootTransform_posi[i];
    }

    for (i = 0; i < 3; i++) {
        i_obj->desiredState[i + 3] = R_T[i];
    }

    for (i = 0; i < 3; i++) {
        i_obj->desiredState[i + 6] = desiredRightFootTransform.position.data[i];
    }

    for (i = 0; i < 3; i++) {
        i_obj->desiredState[i + 9] = desiredRightFootTransform.rotation.data[i];
    }

    h_obj = &obj->desiredPelvisFrame;
    h_obj->relativeTransform = T;
    h_obj = &obj->desiredPelvisFrame;
    for (i = 0; i < 6; i++) {
        h_obj->relativeSpatialVelocity[i] = sv[i];
    }

    c_NonlinearLeastSquaresSolver_s(obj->ikSolver);
    j_obj = obj->ikSolver;
    for (i = 0; i < 10; i++) {
        q[i] = j_obj->x[i];
    }

    k_obj = obj->pdController;
    for (i = 0; i < 10; i++) {
        k_obj->desiredMotorPosition[i] = q[i];
    }

    c_Frame_getSpatialVelocity(&obj->desiredLeftFootFrame,
        &obj->desiredPelvisFrame, &obj->desiredPelvisFrame, sv);
    c_Frame_getSpatialVelocity(&obj->desiredRightFootFrame,
        &obj->desiredPelvisFrame, &obj->desiredPelvisFrame, dqInitialStance);
    for (i = 0; i < 2; i++) {
        dv18[i] = 0.0;
    }

    generatedLeftFootJacobian(*(double (*)[5])&q[0], dv18, dv20);
    b_mldivide(*(double (*)[30])&dv20[0], sv, dv21);
    for (i = 0; i < 2; i++) {
        dv18[i] = 0.0;
    }

    generatedRightFootJacobian(*(double (*)[5])&q[5], dv18, dv20);
    b_mldivide(*(double (*)[30])&dv20[0], dqInitialStance, dv22);
    for (i = 0; i < 5; i++) {
        dq[i] = dv21[i];
        dq[i + 5] = dv22[i];
    }

    k_obj = obj->pdController;
    for (i = 0; i < 10; i++) {
        k_obj->desiredMotorVelocity[i] = dq[i];
    }

    generatedGravityVector(q, g);
    b_obj = &obj->robot->outputs;
    for (i = 0; i < 10; i++) {
        q[i] = b_obj->data.motorPosition[i];
    }

    b_obj = &obj->robot->outputs;
    for (i = 0; i < 10; i++) {
        dq[i] = b_obj->data.motorVelocity[i];
    }

    PDController_evaluate(obj->pdController, q, dq, dv23);
    for (i = 0; i < 10; i++) {
        tau[i] = (tau[i] + g[i]) + dv23[i];
    }

    obj->stanceTime += 0.0005;
    if (b_radioChannel[11] == 1.0) {
        obj->xIntegralError = 0.0;
        obj->yIntegralError = 0.0;
    }

    if (obj->stanceTime > obj->stepInterval) {
        obj->stepInterval = 0.38 + 0.03 * b_radioChannel[5];
        obj->stanceTime = 0.0;
        obj->initialPelvisVelocity = b_centerOfMassVelocity;
        obj->initialStancePelvisTransform = desiredSwingPelvisTransform;
        g_obj = obj;
        if (g_obj->stanceLeg == LEFT) {
            f_obj = &g_obj->desiredRightFootFrame;
        } else {
            f_obj = &g_obj->desiredLeftFootFrame;
        }

        b_parentFrame = f_obj->parentFrame;
        desiredSwingPelvisTransform = b_parentFrame->relativeTransform;
        T = f_obj->relativeTransform;
        for (i = 0; i < 3; i++) {
            membershipLeft = 0.0;
            i109 = 0;
            for (i110 = 0; i110 < 3; i110++) {
                membershipLeft += desiredSwingPelvisTransform.rotation.data[i109
                    + i] * T.position.data[i110];
                R_data[i109 + i] = 0.0;
                i111 = 0;
                for (i112 = 0; i112 < 3; i112++) {
                    R_data[i109 + i] +=
                        desiredSwingPelvisTransform.rotation.data[i111 + i] *
                        T.rotation.data[i112 + i109];
                    i111 += 3;
                }

                i109 += 3;
            }

            b_data[i] = desiredSwingPelvisTransform.position.data[i] +
                membershipLeft;
        }

        for (i = 0; i < 3; i++) {
            desiredSwingPelvisTransform.position.data[i] = b_data[i];
        }

        memcpy(&desiredSwingPelvisTransform.rotation.data[0], &R_data[0], 9U *
               sizeof(double));
        obj->initialStanceFootTransform = desiredSwingPelvisTransform;
        g_obj = obj;
        if (g_obj->stanceLeg == LEFT) {
            f_obj = &g_obj->desiredRightFootFrame;
        } else {
            f_obj = &g_obj->desiredLeftFootFrame;
        }

        T = f_obj->relativeTransform;
        i = 0;
        for (i109 = 0; i109 < 3; i109++) {
            i110 = 0;
            for (i111 = 0; i111 < 3; i111++) {
                R_T[i111 + i] = T.rotation.data[i110 + i109];
                i110 += 3;
            }

            i += 3;
        }

        b_parentFrame = f_obj->parentFrame;
        for (i = 0; i < 6; i++) {
            sv[i] = b_parentFrame->relativeSpatialVelocity[i];
            b_relativeSpatialVelocity[i] = f_obj->relativeSpatialVelocity[i];
        }

        i = 0;
        for (i109 = 0; i109 < 3; i109++) {
            for (i110 = 0; i110 < 3; i110++) {
                c[i110 + i] = -R_T[i110 + i];
            }

            i += 3;
        }

        obj_data[0] = 0.0;
        obj_data[3] = -T.position.data[2];
        obj_data[6] = T.position.data[1];
        obj_data[1] = T.position.data[2];
        obj_data[4] = 0.0;
        obj_data[7] = -T.position.data[0];
        obj_data[2] = -T.position.data[1];
        obj_data[5] = T.position.data[0];
        obj_data[8] = 0.0;
        i = 0;
        i109 = 0;
        for (i110 = 0; i110 < 3; i110++) {
            i111 = 0;
            for (i112 = 0; i112 < 3; i112++) {
                b_R_T[i111 + i110] = 0.0;
                i113 = 0;
                for (i114 = 0; i114 < 3; i114++) {
                    b_R_T[i111 + i110] += c[i113 + i110] * obj_data[i114 + i111];
                    i113 += 3;
                }

                c_R_T[i112 + i] = R_T[i112 + i109];
                i111 += 3;
            }

            i += 6;
            i109 += 3;
        }

        i = 0;
        i109 = 0;
        for (i110 = 0; i110 < 3; i110++) {
            for (i111 = 0; i111 < 3; i111++) {
                c_R_T[(i111 + i) + 18] = b_R_T[i111 + i109];
                c_R_T[(i111 + i) + 3] = 0.0;
                c_R_T[(i111 + i) + 21] = R_T[i111 + i109];
            }

            i += 6;
            i109 += 3;
        }

        for (i = 0; i < 6; i++) {
            membershipLeft = 0.0;
            i109 = 0;
            for (i110 = 0; i110 < 6; i110++) {
                membershipLeft += c_R_T[i109 + i] * sv[i110];
                i109 += 6;
            }

            obj->c_initialStanceFootSpatialVeloc[i] = membershipLeft +
                b_relativeSpatialVelocity[i];
        }

        for (i = 0; i < 3; i++) {
            obj->initialSwingPelvisTransform.position.data[i] = c_data[i];
        }

        for (i = 0; i < 9; i++) {
            obj->initialSwingPelvisTransform.rotation.data[i] =
                c_desiredStanceFootPelvisTransf[i];
        }

        g_obj = obj;
        if (g_obj->stanceLeg == LEFT) {
            f_obj = &g_obj->desiredLeftFootFrame;
        } else {
            f_obj = &g_obj->desiredRightFootFrame;
        }

        b_parentFrame = f_obj->parentFrame;
        desiredSwingPelvisTransform = b_parentFrame->relativeTransform;
        T = f_obj->relativeTransform;
        for (i = 0; i < 3; i++) {
            membershipLeft = 0.0;
            i109 = 0;
            for (i110 = 0; i110 < 3; i110++) {
                membershipLeft += desiredSwingPelvisTransform.rotation.data[i109
                    + i] * T.position.data[i110];
                R_data[i109 + i] = 0.0;
                i111 = 0;
                for (i112 = 0; i112 < 3; i112++) {
                    R_data[i109 + i] +=
                        desiredSwingPelvisTransform.rotation.data[i111 + i] *
                        T.rotation.data[i112 + i109];
                    i111 += 3;
                }

                i109 += 3;
            }

            b_data[i] = desiredSwingPelvisTransform.position.data[i] +
                membershipLeft;
        }

        for (i = 0; i < 3; i++) {
            desiredSwingPelvisTransform.position.data[i] = b_data[i];
        }

        memcpy(&desiredSwingPelvisTransform.rotation.data[0], &R_data[0], 9U *
               sizeof(double));
        obj->initialSwingFootTransform = desiredSwingPelvisTransform;
        g_obj = obj;
        if (g_obj->stanceLeg == LEFT) {
            f_obj = &g_obj->desiredLeftFootFrame;
        } else {
            f_obj = &g_obj->desiredRightFootFrame;
        }

        T = f_obj->relativeTransform;
        i = 0;
        for (i109 = 0; i109 < 3; i109++) {
            i110 = 0;
            for (i111 = 0; i111 < 3; i111++) {
                R_T[i111 + i] = T.rotation.data[i110 + i109];
                i110 += 3;
            }

            i += 3;
        }

        b_parentFrame = f_obj->parentFrame;
        for (i = 0; i < 6; i++) {
            sv[i] = b_parentFrame->relativeSpatialVelocity[i];
            b_relativeSpatialVelocity[i] = f_obj->relativeSpatialVelocity[i];
        }

        i = 0;
        for (i109 = 0; i109 < 3; i109++) {
            for (i110 = 0; i110 < 3; i110++) {
                c[i110 + i] = -R_T[i110 + i];
            }

            i += 3;
        }

        obj_data[0] = 0.0;
        obj_data[3] = -T.position.data[2];
        obj_data[6] = T.position.data[1];
        obj_data[1] = T.position.data[2];
        obj_data[4] = 0.0;
        obj_data[7] = -T.position.data[0];
        obj_data[2] = -T.position.data[1];
        obj_data[5] = T.position.data[0];
        obj_data[8] = 0.0;
        i = 0;
        i109 = 0;
        for (i110 = 0; i110 < 3; i110++) {
            i111 = 0;
            for (i112 = 0; i112 < 3; i112++) {
                b_R_T[i111 + i110] = 0.0;
                i113 = 0;
                for (i114 = 0; i114 < 3; i114++) {
                    b_R_T[i111 + i110] += c[i113 + i110] * obj_data[i114 + i111];
                    i113 += 3;
                }

                c_R_T[i112 + i] = R_T[i112 + i109];
                i111 += 3;
            }

            i += 6;
            i109 += 3;
        }

        i = 0;
        i109 = 0;
        for (i110 = 0; i110 < 3; i110++) {
            for (i111 = 0; i111 < 3; i111++) {
                c_R_T[(i111 + i) + 18] = b_R_T[i111 + i109];
                c_R_T[(i111 + i) + 3] = 0.0;
                c_R_T[(i111 + i) + 21] = R_T[i111 + i109];
            }

            i += 6;
            i109 += 3;
        }

        for (i = 0; i < 6; i++) {
            membershipLeft = 0.0;
            i109 = 0;
            for (i110 = 0; i110 < 6; i110++) {
                membershipLeft += c_R_T[i109 + i] * sv[i110];
                i109 += 6;
            }

            obj->initialSwingFootSpatialVelocity[i] = membershipLeft +
                b_relativeSpatialVelocity[i];
        }

        obj->stanceLeg = convert_to_enum_RobotSide((signed char)-(double)
            obj->stanceLeg);
    }
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

static void Transform3d_mldivide(const double a_position_data[3], const double
    a_rotation_data[9], const double b_position_data[3], const double
    b_rotation_data[9], double c_position_data[3], double c_rotation_data[9])
{
    int i;
    int b_i;
    double R_data[9];
    double b_a_rotation_data[3];
    double c_a_rotation_data[3];
    int i30;
    int i31;
    double p_data[3];
    int i32;
    i = 0;
    for (b_i = 0; b_i < 3; b_i++) {
        b_a_rotation_data[b_i] = 0.0;
        c_a_rotation_data[b_i] = 0.0;
        i30 = 0;
        for (i31 = 0; i31 < 3; i31++) {
            R_data[i30 + b_i] = 0.0;
            for (i32 = 0; i32 < 3; i32++) {
                R_data[i30 + b_i] += a_rotation_data[i32 + i] *
                    b_rotation_data[i32 + i30];
            }

            b_a_rotation_data[b_i] += a_rotation_data[i31 + i] *
                b_position_data[i31];
            c_a_rotation_data[b_i] += a_rotation_data[i31 + i] *
                a_position_data[i31];
            i30 += 3;
        }

        p_data[b_i] = b_a_rotation_data[b_i] - c_a_rotation_data[b_i];
        c_position_data[b_i] = p_data[b_i];
        i += 3;
    }

    memcpy(&c_rotation_data[0], &R_data[0], 9U * sizeof(double));
}

static void ZUpFrame_getRelativeTransform(const ZUpFrame_1 *obj, double
    relativeTransform_position_data[3], double relativeTransform_rotation_data[9])
{
    Frame_8 *b_obj;
    WorldFrame *b_parentFrame;
    int i45;
    double worldTransform_rotation_data[9];
    Rotation3d worldZUpRotation;
    double R_data[9];
    int i46;
    int i47;
    int i;
    int b_i;
    double c_data[9];
    b_obj = obj->parentFrame;
    b_parentFrame = b_obj->parentFrame;
    for (i45 = 0; i45 < 9; i45++) {
        worldTransform_rotation_data[i45] =
            b_parentFrame->relativeTransform.rotation.data[i45];
        relativeTransform_rotation_data[i45] =
            b_obj->relativeTransform.rotation.data[i45];
    }

    for (i45 = 0; i45 < 3; i45++) {
        i46 = 0;
        for (i47 = 0; i47 < 3; i47++) {
            R_data[i46 + i45] = 0.0;
            i = 0;
            for (b_i = 0; b_i < 3; b_i++) {
                R_data[i46 + i45] += worldTransform_rotation_data[i + i45] *
                    relativeTransform_rotation_data[b_i + i46];
                i += 3;
            }

            i46 += 3;
        }
    }

    memcpy(&worldZUpRotation.data[0], &R_data[0], 9U * sizeof(double));
    Rotation3d_removePitchAndRoll(&worldZUpRotation);
    i = 0;
    for (b_i = 0; b_i < 3; b_i++) {
        i45 = 0;
        for (i46 = 0; i46 < 3; i46++) {
            c_data[i45 + b_i] = 0.0;
            for (i47 = 0; i47 < 3; i47++) {
                c_data[i45 + b_i] += R_data[i47 + i] * worldZUpRotation.data[i47
                    + i45];
            }

            i45 += 3;
        }

        relativeTransform_position_data[b_i] = 0.0;
        i += 3;
    }

    memcpy(&relativeTransform_rotation_data[0], &c_data[0], 9U * sizeof(double));
}

static double __anon_fcn(double b_x, double x1, double x2)
{
    return fmax(fmin((b_x - x1) / (x2 - x1), 1.0), 0.0);
}

static void b_CassieEtherCAT_CassieEtherCAT(CassieEtherCAT **obj)
{
    CassieEtherCAT *b_this;
    int i;
    signed char c_data_pelvisMedulla_inputs_vnO[4];
    static const signed char iv7[4] = { 1, 0, 0, 0 };

    b_this = *obj;
    *obj = b_this;
    for (i = 0; i < 4; i++) {
        c_data_pelvisMedulla_inputs_vnO[i] = iv7[i];
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

static void b_CassieRobot_CassieRobot(CassieRobot **obj)
{
    CassieRobot *b_this;
    CassieInputs *b_obj;
    int i;
    WorldFrame *c_obj;
    signed char obj_rotation_data[9];
    int k;
    Frame *d_obj;
    static const signed char iv8[9] = { 1, 0, 0, 0, -1, 0, 0, 0, -1 };

    Frame_1 *e_obj;
    Frame_2 *f_obj;
    static const char b_name[6] = { 'P', 'e', 'l', 'v', 'i', 's' };

    double relativeTransform_position_data[3];
    static const double dv30[3] = { -0.10756, 0.0, 0.002 };

    static const signed char iv9[9] = { 0, 0, -1, 0, -1, 0, -1, 0, 0 };

    ZUpFrame *g_obj;
    static const char c_name[11] = { 'Z', '-', 'U', 'p', ' ', 'P', 'e', 'l', 'v',
        'i', 's' };

    Frame_3 *h_obj;
    static const char d_name[14] = { 'C', 'e', 'n', 't', 'e', 'r', ' ', 'O', 'f',
        ' ', 'M', 'a', 's', 's' };

    Frame_4 *i_obj;
    LowPassFilter *j_obj;
    b_this = *obj;
    *obj = b_this;
    b_obj = &(*obj)->inputs;
    for (i = 0; i < 9; i++) {
        b_obj->data.radio[i] = 0.0;
    }

    for (i = 0; i < 10; i++) {
        b_obj->data.torque[i] = 0.0;
    }

    CassieOutputs_CassieOutputs(&(*obj)->outputs);
    c_obj = &(*obj)->worldFrame;
    for (i = 0; i < 9; i++) {
        obj_rotation_data[i] = 0;
    }

    i = 0;
    for (k = 0; k < 3; k++) {
        obj_rotation_data[i] = 1;
        i += 4;
    }

    for (i = 0; i < 3; i++) {
        c_obj->relativeTransform.position.data[i] = 0.0;
    }

    for (i = 0; i < 9; i++) {
        c_obj->relativeTransform.rotation.data[i] = obj_rotation_data[i];
    }

    for (i = 0; i < 6; i++) {
        c_obj->relativeSpatialVelocity[i] = 0.0;
    }

    d_obj = &(*obj)->northEastDownFrame;
    c_obj = &(*obj)->worldFrame;
    d_obj->parentFrame = c_obj;
    for (i = 0; i < 9; i++) {
        obj_rotation_data[i] = 0;
    }

    i = 0;
    for (k = 0; k < 3; k++) {
        obj_rotation_data[i] = 1;
        i += 4;
    }

    for (i = 0; i < 3; i++) {
        d_obj->relativeTransform.position.data[i] = 0.0;
    }

    for (i = 0; i < 9; i++) {
        d_obj->relativeTransform.rotation.data[i] = obj_rotation_data[i];
    }

    for (i = 0; i < 6; i++) {
        d_obj->relativeSpatialVelocity[i] = 0.0;
    }

    d_obj = &(*obj)->northEastDownFrame;
    for (i = 0; i < 9; i++) {
        obj_rotation_data[i] = iv8[i];
    }

    for (i = 0; i < 3; i++) {
        d_obj->relativeTransform.position.data[i] = 0.0;
    }

    for (i = 0; i < 9; i++) {
        d_obj->relativeTransform.rotation.data[i] = obj_rotation_data[i];
    }

    e_obj = &(*obj)->vectorNavFrame;
    d_obj = &(*obj)->northEastDownFrame;
    e_obj->parentFrame = d_obj;
    for (i = 0; i < 9; i++) {
        obj_rotation_data[i] = 0;
    }

    i = 0;
    for (k = 0; k < 3; k++) {
        obj_rotation_data[i] = 1;
        i += 4;
    }

    for (i = 0; i < 3; i++) {
        e_obj->relativeTransform.position.data[i] = 0.0;
    }

    for (i = 0; i < 9; i++) {
        e_obj->relativeTransform.rotation.data[i] = obj_rotation_data[i];
    }

    for (i = 0; i < 6; i++) {
        e_obj->relativeSpatialVelocity[i] = 0.0;
    }

    f_obj = &(*obj)->pelvisFrame;
    e_obj = &(*obj)->vectorNavFrame;
    for (i = 0; i < 6; i++) {
        f_obj->name[i] = b_name[i];
    }

    f_obj->parentFrame = e_obj;
    for (i = 0; i < 9; i++) {
        obj_rotation_data[i] = 0;
    }

    i = 0;
    for (k = 0; k < 3; k++) {
        obj_rotation_data[i] = 1;
        i += 4;
    }

    for (i = 0; i < 3; i++) {
        f_obj->relativeTransform.position.data[i] = 0.0;
    }

    for (i = 0; i < 9; i++) {
        f_obj->relativeTransform.rotation.data[i] = obj_rotation_data[i];
    }

    for (i = 0; i < 6; i++) {
        f_obj->relativeSpatialVelocity[i] = 0.0;
    }

    f_obj = &(*obj)->pelvisFrame;
    for (i = 0; i < 3; i++) {
        relativeTransform_position_data[i] = dv30[i];
    }

    for (i = 0; i < 9; i++) {
        obj_rotation_data[i] = iv9[i];
    }

    for (i = 0; i < 3; i++) {
        f_obj->relativeTransform.position.data[i] =
            relativeTransform_position_data[i];
    }

    for (i = 0; i < 9; i++) {
        f_obj->relativeTransform.rotation.data[i] = obj_rotation_data[i];
    }

    g_obj = &(*obj)->zUpPelvisFrame;
    f_obj = &(*obj)->pelvisFrame;
    for (i = 0; i < 11; i++) {
        g_obj->name[i] = c_name[i];
    }

    g_obj->parentFrame = f_obj;
    h_obj = &(*obj)->centerOfMassFrame;
    f_obj = &(*obj)->pelvisFrame;
    for (i = 0; i < 14; i++) {
        h_obj->name[i] = d_name[i];
    }

    h_obj->parentFrame = f_obj;
    for (i = 0; i < 9; i++) {
        obj_rotation_data[i] = 0;
    }

    i = 0;
    for (k = 0; k < 3; k++) {
        obj_rotation_data[i] = 1;
        i += 4;
    }

    for (i = 0; i < 3; i++) {
        h_obj->relativeTransform.position.data[i] = 0.0;
    }

    for (i = 0; i < 9; i++) {
        h_obj->relativeTransform.rotation.data[i] = obj_rotation_data[i];
    }

    for (i = 0; i < 6; i++) {
        h_obj->relativeSpatialVelocity[i] = 0.0;
    }

    i_obj = &(*obj)->leftFootFrame;
    f_obj = &(*obj)->pelvisFrame;
    i_obj->parentFrame = f_obj;
    for (i = 0; i < 9; i++) {
        obj_rotation_data[i] = 0;
    }

    i = 0;
    for (k = 0; k < 3; k++) {
        obj_rotation_data[i] = 1;
        i += 4;
    }

    for (i = 0; i < 3; i++) {
        i_obj->relativeTransform.position.data[i] = 0.0;
    }

    for (i = 0; i < 9; i++) {
        i_obj->relativeTransform.rotation.data[i] = obj_rotation_data[i];
    }

    i_obj = &(*obj)->rightFootFrame;
    f_obj = &(*obj)->pelvisFrame;
    i_obj->parentFrame = f_obj;
    for (i = 0; i < 9; i++) {
        obj_rotation_data[i] = 0;
    }

    i = 0;
    for (k = 0; k < 3; k++) {
        obj_rotation_data[i] = 1;
        i += 4;
    }

    for (i = 0; i < 3; i++) {
        i_obj->relativeTransform.position.data[i] = 0.0;
    }

    for (i = 0; i < 9; i++) {
        i_obj->relativeTransform.rotation.data[i] = obj_rotation_data[i];
    }

    for (i = 0; i < 3; i++) {
        (*obj)->leftFootForce.data[i] = 0.0;
    }

    for (i = 0; i < 3; i++) {
        (*obj)->rightFootForce.data[i] = 0.0;
    }

    for (i = 0; i < 3; i++) {
        (*obj)->centerOfMassVelocity.data[i] = 0.0;
    }

    j_obj = &(*obj)->leftFootFilter;
    j_obj->sampleTime = 0.001;
    j_obj->smoothingFactor = 0.5;
    for (i = 0; i < 3; i++) {
        j_obj->signalValue.data[i] = 0.0;
    }

    for (i = 0; i < 3; i++) {
        j_obj->signalRateOfChange.data[i] = 0.0;
    }

    j_obj = &(*obj)->leftFootFilter;
    j_obj->sampleTime = 0.0005;
    j_obj = &(*obj)->leftFootFilter;
    j_obj->smoothingFactor = 1.0;
    j_obj = &(*obj)->rightFootFilter;
    j_obj->sampleTime = 0.001;
    j_obj->smoothingFactor = 0.5;
    for (i = 0; i < 3; i++) {
        j_obj->signalValue.data[i] = 0.0;
    }

    for (i = 0; i < 3; i++) {
        j_obj->signalRateOfChange.data[i] = 0.0;
    }

    j_obj = &(*obj)->rightFootFilter;
    j_obj->sampleTime = 0.0005;
    j_obj = &(*obj)->rightFootFilter;
    j_obj->smoothingFactor = 1.0;
}

static Frame_7 *b_Frame_Frame(Frame_7 *obj, WorldFrame *b_parentFrame)
{
    Frame_7 *b_obj;
    b_obj = obj;
    d_Frame_Frame(&b_obj, b_parentFrame);
    return b_obj;
}

static void b_Frame_getSpatialVelocity(ZUpFrame_1 *obj, ZUpFrame *relativeTo,
    ZUpFrame *inCoordinatesOf, double spatialVelocity[6])
{
    ZUpFrame_1 *b_obj;
    double obj_position_data[3];
    double obj_rotation_data[9];
    Frame_8 *b_parentFrame;
    int i;
    int b_i;
    int i77;
    int i78;
    double relativeTransform_rotation_data[9];
    double relativeTransform_position_data[3];
    double R_T[9];
    WorldFrame *c_parentFrame;
    double b_obj_position_data[3];
    double worldSpatialVelocity[6];
    double b_R_T[9];
    double b_relativeSpatialVelocity[6];
    double b_relativeTransform_position_da[3];
    double c_R_T[9];
    double b_worldSpatialVelocity[6];
    double d_R_T[9];
    double c_relativeSpatialVelocity[6];
    double dv16[9];
    double e_R_T[9];
    int i79;
    int i80;
    int i81;
    double d8;
    double f_R_T[36];
    ZUpFrame *c_obj;
    double s[6];
    double g_R_T[36];
    ZUpFrame_1 *b_inCoordinatesOf;
    double h_R_T[6];
    Rotation3d obj_rotation;
    double transform_position_data[3];
    double transform_rotation_data[9];
    bool b_bool;
    char a[17];
    char b[17];
    int exitg1;
    double R_data[9];
    double c_data[9];
    b_obj = obj;
    ZUpFrame_getRelativeTransform(b_obj, obj_position_data, obj_rotation_data);
    b_parentFrame = b_obj->parentFrame;
    i = 0;
    for (b_i = 0; b_i < 3; b_i++) {
        i77 = 0;
        for (i78 = 0; i78 < 3; i78++) {
            R_T[i78 + i] = obj_rotation_data[i77 + b_i];
            i77 += 3;
        }

        relativeTransform_position_data[b_i] =
            b_parentFrame->relativeTransform.position.data[b_i];
        i += 3;
    }

    for (i77 = 0; i77 < 9; i77++) {
        relativeTransform_rotation_data[i77] =
            b_parentFrame->relativeTransform.rotation.data[i77];
    }

    i77 = 0;
    for (i78 = 0; i78 < 3; i78++) {
        i = 0;
        for (b_i = 0; b_i < 3; b_i++) {
            b_R_T[b_i + i77] = relativeTransform_rotation_data[i + i78];
            i += 3;
        }

        i77 += 3;
    }

    c_parentFrame = b_parentFrame->parentFrame;
    for (i = 0; i < 6; i++) {
        worldSpatialVelocity[i] = c_parentFrame->relativeSpatialVelocity[i];
        b_relativeSpatialVelocity[i] = b_parentFrame->relativeSpatialVelocity[i];
    }

    ZUpFrame_getRelativeTransform(b_obj, b_obj_position_data, obj_rotation_data);
    b_parentFrame = b_obj->parentFrame;
    i = 0;
    for (b_i = 0; b_i < 3; b_i++) {
        i77 = 0;
        for (i78 = 0; i78 < 3; i78++) {
            c_R_T[i78 + i] = obj_rotation_data[i77 + b_i];
            i77 += 3;
        }

        b_relativeTransform_position_da[b_i] =
            b_parentFrame->relativeTransform.position.data[b_i];
        i += 3;
    }

    for (i77 = 0; i77 < 9; i77++) {
        relativeTransform_rotation_data[i77] =
            b_parentFrame->relativeTransform.rotation.data[i77];
    }

    i77 = 0;
    for (i78 = 0; i78 < 3; i78++) {
        i = 0;
        for (b_i = 0; b_i < 3; b_i++) {
            d_R_T[b_i + i77] = relativeTransform_rotation_data[i + i78];
            i += 3;
        }

        i77 += 3;
    }

    c_parentFrame = b_parentFrame->parentFrame;
    for (i = 0; i < 6; i++) {
        b_worldSpatialVelocity[i] = c_parentFrame->relativeSpatialVelocity[i];
        c_relativeSpatialVelocity[i] = b_parentFrame->relativeSpatialVelocity[i];
    }

    i77 = 0;
    for (i78 = 0; i78 < 3; i78++) {
        for (i = 0; i < 3; i++) {
            e_R_T[i + i77] = -c_R_T[i + i77];
        }

        i77 += 3;
    }

    dv16[0] = 0.0;
    dv16[3] = -b_obj_position_data[2];
    dv16[6] = b_obj_position_data[1];
    dv16[1] = b_obj_position_data[2];
    dv16[4] = 0.0;
    dv16[7] = -b_obj_position_data[0];
    dv16[2] = -b_obj_position_data[1];
    dv16[5] = b_obj_position_data[0];
    dv16[8] = 0.0;
    i77 = 0;
    for (i78 = 0; i78 < 3; i78++) {
        i = 0;
        for (b_i = 0; b_i < 3; b_i++) {
            obj_rotation_data[i + i78] = 0.0;
            i79 = 0;
            for (i80 = 0; i80 < 3; i80++) {
                obj_rotation_data[i + i78] += e_R_T[i79 + i78] * dv16[i80 + i];
                i79 += 3;
            }

            relativeTransform_rotation_data[b_i + i77] = -d_R_T[b_i + i77];
            i += 3;
        }

        i77 += 3;
    }

    dv16[0] = 0.0;
    dv16[3] = -b_relativeTransform_position_da[2];
    dv16[6] = b_relativeTransform_position_da[1];
    dv16[1] = b_relativeTransform_position_da[2];
    dv16[4] = 0.0;
    dv16[7] = -b_relativeTransform_position_da[0];
    dv16[2] = -b_relativeTransform_position_da[1];
    dv16[5] = b_relativeTransform_position_da[0];
    dv16[8] = 0.0;
    i77 = 0;
    i78 = 0;
    for (i = 0; i < 3; i++) {
        b_i = 0;
        for (i79 = 0; i79 < 3; i79++) {
            e_R_T[b_i + i] = 0.0;
            i80 = 0;
            for (i81 = 0; i81 < 3; i81++) {
                e_R_T[b_i + i] += relativeTransform_rotation_data[i80 + i] *
                    dv16[i81 + b_i];
                i80 += 3;
            }

            f_R_T[i79 + i77] = d_R_T[i79 + i78];
            b_i += 3;
        }

        i77 += 6;
        i78 += 3;
    }

    i77 = 0;
    i78 = 0;
    for (i = 0; i < 3; i++) {
        for (b_i = 0; b_i < 3; b_i++) {
            f_R_T[(b_i + i77) + 18] = e_R_T[b_i + i78];
            f_R_T[(b_i + i77) + 3] = 0.0;
            f_R_T[(b_i + i77) + 21] = d_R_T[b_i + i78];
            g_R_T[b_i + i77] = c_R_T[b_i + i78];
            g_R_T[(b_i + i77) + 18] = obj_rotation_data[b_i + i78];
            g_R_T[(b_i + i77) + 3] = 0.0;
            g_R_T[(b_i + i77) + 21] = c_R_T[b_i + i78];
        }

        i77 += 6;
        i78 += 3;
    }

    for (i77 = 0; i77 < 6; i77++) {
        d8 = 0.0;
        i78 = 0;
        for (i = 0; i < 6; i++) {
            d8 += f_R_T[i78 + i77] * b_worldSpatialVelocity[i];
            i78 += 6;
        }

        h_R_T[i77] = d8 + c_relativeSpatialVelocity[i77];
    }

    for (i77 = 0; i77 < 6; i77++) {
        s[i77] = 0.0;
        i78 = 0;
        for (i = 0; i < 6; i++) {
            s[i77] += g_R_T[i78 + i77] * h_R_T[i];
            i78 += 6;
        }
    }

    c_obj = relativeTo;
    b_obj = obj;
    b_inCoordinatesOf = b_obj;
    d_Frame_getWorldTransform(b_obj, b_relativeTransform_position_da,
        relativeTransform_rotation_data);
    Frame_getWorldTransform(c_obj, b_obj_position_data, &obj_rotation);
    memcpy(&obj_rotation_data[0], &obj_rotation.data[0], 9U * sizeof(double));
    Transform3d_mldivide(b_relativeTransform_position_da,
                         relativeTransform_rotation_data, b_obj_position_data,
                         obj_rotation_data, transform_position_data,
                         transform_rotation_data);
    for (i77 = 0; i77 < 17; i77++) {
        a[i77] = b_obj->name[i77];
        b[i77] = b_inCoordinatesOf->name[i77];
    }

    b_bool = false;
    i = 0;
    do {
        exitg1 = 0;
        if (i + 1 < 18) {
            if (a[i] != b[i]) {
                exitg1 = 1;
            } else {
                i++;
            }
        } else {
            b_bool = true;
            exitg1 = 1;
        }
    } while (exitg1 == 0);

    if (!b_bool) {
        d_Frame_getWorldTransform(b_inCoordinatesOf, b_obj_position_data,
            obj_rotation_data);
        d_Frame_getWorldTransform(b_obj, b_relativeTransform_position_da,
            relativeTransform_rotation_data);
        for (i77 = 0; i77 < 3; i77++) {
            d8 = 0.0;
            for (i78 = 0; i78 < 3; i78++) {
                c_data[i77 + 3 * i78] = 0.0;
                for (i = 0; i < 3; i++) {
                    c_data[i77 + 3 * i78] += obj_rotation_data[i + 3 * i77] *
                        relativeTransform_rotation_data[i + 3 * i78];
                }

                d8 += c_data[i77 + 3 * i78] * transform_position_data[i78];
            }

            b_relativeTransform_position_da[i77] = d8;
            for (i78 = 0; i78 < 3; i78++) {
                R_data[i77 + 3 * i78] = 0.0;
                for (i = 0; i < 3; i++) {
                    R_data[i77 + 3 * i78] += c_data[i77 + 3 * i] *
                        transform_rotation_data[i + 3 * i78];
                }
            }
        }

        for (i = 0; i < 3; i++) {
            transform_position_data[i] = b_relativeTransform_position_da[i];
        }

        memcpy(&transform_rotation_data[0], &R_data[0], 9U * sizeof(double));
    }

    b_Frame_getWorldSpatialVelocity(relativeTo, b_worldSpatialVelocity);
    i77 = 0;
    for (i78 = 0; i78 < 3; i78++) {
        for (i = 0; i < 3; i++) {
            c_R_T[i + i77] = -R_T[i + i77];
        }

        i77 += 3;
    }

    dv16[0] = 0.0;
    dv16[3] = -obj_position_data[2];
    dv16[6] = obj_position_data[1];
    dv16[1] = obj_position_data[2];
    dv16[4] = 0.0;
    dv16[7] = -obj_position_data[0];
    dv16[2] = -obj_position_data[1];
    dv16[5] = obj_position_data[0];
    dv16[8] = 0.0;
    i77 = 0;
    for (i78 = 0; i78 < 3; i78++) {
        i = 0;
        for (b_i = 0; b_i < 3; b_i++) {
            d_R_T[i + i78] = 0.0;
            i79 = 0;
            for (i80 = 0; i80 < 3; i80++) {
                d_R_T[i + i78] += c_R_T[i79 + i78] * dv16[i80 + i];
                i79 += 3;
            }

            e_R_T[b_i + i77] = -b_R_T[b_i + i77];
            i += 3;
        }

        i77 += 3;
    }

    dv16[0] = 0.0;
    dv16[3] = -relativeTransform_position_data[2];
    dv16[6] = relativeTransform_position_data[1];
    dv16[1] = relativeTransform_position_data[2];
    dv16[4] = 0.0;
    dv16[7] = -relativeTransform_position_data[0];
    dv16[2] = -relativeTransform_position_data[1];
    dv16[5] = relativeTransform_position_data[0];
    dv16[8] = 0.0;
    i77 = 0;
    i78 = 0;
    for (i = 0; i < 3; i++) {
        b_i = 0;
        for (i79 = 0; i79 < 3; i79++) {
            c_R_T[b_i + i] = 0.0;
            i80 = 0;
            for (i81 = 0; i81 < 3; i81++) {
                c_R_T[b_i + i] += e_R_T[i80 + i] * dv16[i81 + b_i];
                i80 += 3;
            }

            f_R_T[i79 + i77] = b_R_T[i79 + i78];
            b_i += 3;
        }

        i77 += 6;
        i78 += 3;
    }

    i77 = 0;
    i78 = 0;
    for (i = 0; i < 3; i++) {
        for (b_i = 0; b_i < 3; b_i++) {
            f_R_T[(b_i + i77) + 18] = c_R_T[b_i + i78];
            f_R_T[(b_i + i77) + 3] = 0.0;
            f_R_T[(b_i + i77) + 21] = b_R_T[b_i + i78];
            g_R_T[b_i + i77] = R_T[b_i + i78];
            g_R_T[(b_i + i77) + 18] = d_R_T[b_i + i78];
            g_R_T[(b_i + i77) + 3] = 0.0;
            g_R_T[(b_i + i77) + 21] = R_T[b_i + i78];
        }

        i77 += 6;
        i78 += 3;
    }

    for (i77 = 0; i77 < 6; i77++) {
        d8 = 0.0;
        i78 = 0;
        for (i = 0; i < 6; i++) {
            d8 += f_R_T[i78 + i77] * worldSpatialVelocity[i];
            i78 += 6;
        }

        h_R_T[i77] = d8 + b_relativeSpatialVelocity[i77];
    }

    for (i77 = 0; i77 < 6; i77++) {
        b_relativeSpatialVelocity[i77] = 0.0;
        i78 = 0;
        for (i = 0; i < 6; i++) {
            b_relativeSpatialVelocity[i77] += g_R_T[i78 + i77] * h_R_T[i];
            i78 += 6;
        }
    }

    worldSpatialVelocity[0] = 0.0;
    worldSpatialVelocity[1] = 0.0;
    worldSpatialVelocity[2] = 0.0;
    worldSpatialVelocity[3] = -s[3];
    worldSpatialVelocity[4] = -s[4];
    worldSpatialVelocity[5] = 0.0;
    dv16[0] = 0.0;
    dv16[3] = -transform_position_data[2];
    dv16[6] = transform_position_data[1];
    dv16[1] = transform_position_data[2];
    dv16[4] = 0.0;
    dv16[7] = -transform_position_data[0];
    dv16[2] = -transform_position_data[1];
    dv16[5] = transform_position_data[0];
    dv16[8] = 0.0;
    i77 = 0;
    i78 = 0;
    for (i = 0; i < 3; i++) {
        b_i = 0;
        for (i79 = 0; i79 < 3; i79++) {
            obj_rotation_data[b_i + i] = 0.0;
            i80 = 0;
            for (i81 = 0; i81 < 3; i81++) {
                obj_rotation_data[b_i + i] += dv16[i80 + i] *
                    transform_rotation_data[i81 + b_i];
                i80 += 3;
            }

            f_R_T[i79 + i77] = transform_rotation_data[i79 + i78];
            b_i += 3;
        }

        i77 += 6;
        i78 += 3;
    }

    i77 = 0;
    i78 = 0;
    for (i = 0; i < 3; i++) {
        for (b_i = 0; b_i < 3; b_i++) {
            f_R_T[(b_i + i77) + 18] = obj_rotation_data[b_i + i78];
            f_R_T[(b_i + i77) + 3] = 0.0;
            f_R_T[(b_i + i77) + 21] = transform_rotation_data[b_i + i78];
        }

        i77 += 6;
        i78 += 3;
    }

    for (i77 = 0; i77 < 6; i77++) {
        d8 = 0.0;
        i78 = 0;
        for (i = 0; i < 6; i++) {
            d8 += f_R_T[i78 + i77] * b_worldSpatialVelocity[i];
            i78 += 6;
        }

        spatialVelocity[i77] = (b_relativeSpatialVelocity[i77] +
                                worldSpatialVelocity[i77]) - d8;
    }

    d_Frame_getTransform(obj, inCoordinatesOf, b_relativeTransform_position_da,
                         relativeTransform_rotation_data);
    for (i77 = 0; i77 < 3; i77++) {
        b_relativeTransform_position_da[i77] = 0.0;
        b_obj_position_data[i77] = 0.0;
        i78 = 0;
        for (i = 0; i < 3; i++) {
            b_relativeTransform_position_da[i77] +=
                relativeTransform_rotation_data[i78 + i77] * spatialVelocity[i];
            b_obj_position_data[i77] += relativeTransform_rotation_data[i78 +
                i77] * spatialVelocity[i + 3];
            i78 += 3;
        }
    }

    for (i77 = 0; i77 < 3; i77++) {
        spatialVelocity[i77] = b_relativeTransform_position_da[i77];
        spatialVelocity[i77 + 3] = b_obj_position_data[i77];
    }
}

static void b_Frame_getTransform(Frame_4 *obj, WorldFrame *withRespectTo,
    ZUpFrame *inCoordinatesOf, double transform_position_data[3], double
    transform_rotation_data[9])
{
    Frame_4 *b_obj;
    Frame_2 *b_parentFrame;
    int i;
    int i33;
    double relativeTransform_position_data[3];
    Transform3d r7;
    double relativeTransform_rotation_data[9];
    Transform3d r8;
    double a_position_data[3];
    double a_rotation_data[9];
    Rotation3d relativeTransform_rotation;
    double p_data;
    int i34;
    WorldFrame *c_obj;
    double b_p_data[3];
    double R_data[9];
    int i35;
    double b_R_data[9];
    int i36;
    double c_data[9];
    b_obj = obj;
    b_parentFrame = b_obj->parentFrame;
    for (i = 0; i < 3; i++) {
        relativeTransform_position_data[i] =
            b_obj->relativeTransform.position.data[i];
    }

    for (i33 = 0; i33 < 9; i33++) {
        relativeTransform_rotation_data[i33] =
            b_obj->relativeTransform.rotation.data[i33];
    }

    b_Frame_getWorldTransform(b_parentFrame, &r7);
    for (i = 0; i < 3; i++) {
        a_position_data[i] = r7.position.data[i];
    }

    b_Frame_getWorldTransform(b_parentFrame, &r8);
    memcpy(&a_rotation_data[0], &r8.rotation.data[0], 9U * sizeof(double));
    for (i33 = 0; i33 < 3; i33++) {
        p_data = 0.0;
        i34 = 0;
        for (i = 0; i < 3; i++) {
            p_data += a_rotation_data[i34 + i33] *
                relativeTransform_position_data[i];
            R_data[i34 + i33] = 0.0;
            i35 = 0;
            for (i36 = 0; i36 < 3; i36++) {
                R_data[i34 + i33] += a_rotation_data[i35 + i33] *
                    relativeTransform_rotation_data[i36 + i34];
                i35 += 3;
            }

            i34 += 3;
        }

        b_p_data[i33] = a_position_data[i33] + p_data;
    }

    Frame_getWorldTransform(inCoordinatesOf, relativeTransform_position_data,
                            &relativeTransform_rotation);
    memcpy(&relativeTransform_rotation_data[0],
           &relativeTransform_rotation.data[0], 9U * sizeof(double));
    c_obj = withRespectTo;
    for (i33 = 0; i33 < 9; i33++) {
        a_rotation_data[i33] = c_obj->relativeTransform.rotation.data[i33];
    }

    for (i = 0; i < 3; i++) {
        p_data = 0.0;
        for (i33 = 0; i33 < 3; i33++) {
            c_data[i + 3 * i33] = 0.0;
            for (i34 = 0; i34 < 3; i34++) {
                c_data[i + 3 * i33] += relativeTransform_rotation_data[i34 + 3 *
                    i] * a_rotation_data[i34 + 3 * i33];
            }

            p_data += c_data[i + 3 * i33] * b_p_data[i33];
        }

        for (i33 = 0; i33 < 3; i33++) {
            b_R_data[i + 3 * i33] = 0.0;
            for (i34 = 0; i34 < 3; i34++) {
                b_R_data[i + 3 * i33] += c_data[i + 3 * i34] * R_data[i34 + 3 *
                    i33];
            }
        }

        transform_position_data[i] = p_data;
    }

    memcpy(&transform_rotation_data[0], &b_R_data[0], 9U * sizeof(double));
}

static void b_Frame_getWorldSpatialVelocity(ZUpFrame *obj, double
    worldSpatialVelocity[6])
{
    ZUpFrame *b_obj;
    Transform3d r17;
    Rotation3d obj_rotation;
    Rotation3d worldZUpRotation;
    int i70;
    int i71;
    int i72;
    Frame_2 *b_parentFrame;
    int i73;
    double b[6];
    double R_T[9];
    Transform3d r18;
    int i74;
    double b_R_T[9];
    double b_b[6];
    signed char iv0[9];
    double c_R_T[9];
    double d_R_T[9];
    int i75;
    int i76;
    double s[6];
    double e_R_T[36];
    double dv15[6];
    b_obj = obj;
    b_Frame_getWorldTransform(b_obj->parentFrame, &r17);
    obj_rotation = r17.rotation;
    worldZUpRotation = obj_rotation;
    Rotation3d_removePitchAndRoll(&worldZUpRotation);
    i70 = 0;
    for (i71 = 0; i71 < 3; i71++) {
        i72 = 0;
        for (i73 = 0; i73 < 3; i73++) {
            R_T[i72 + i71] = 0.0;
            for (i74 = 0; i74 < 3; i74++) {
                R_T[i72 + i71] += obj_rotation.data[i74 + i70] *
                    worldZUpRotation.data[i74 + i72];
            }

            b_R_T[i73 + i70] = R_T[i72 + i71];
            i72 += 3;
        }

        i70 += 3;
    }

    b_obj = obj;
    b_parentFrame = b_obj->parentFrame;
    Frame_getWorldSpatialVelocity(b_parentFrame, b);
    b_obj = obj;
    b_Frame_getWorldTransform(b_obj->parentFrame, &r18);
    obj_rotation = r18.rotation;
    worldZUpRotation = obj_rotation;
    Rotation3d_removePitchAndRoll(&worldZUpRotation);
    i70 = 0;
    for (i71 = 0; i71 < 3; i71++) {
        i72 = 0;
        for (i73 = 0; i73 < 3; i73++) {
            R_T[i72 + i71] = 0.0;
            for (i74 = 0; i74 < 3; i74++) {
                R_T[i72 + i71] += obj_rotation.data[i74 + i70] *
                    worldZUpRotation.data[i74 + i72];
            }

            c_R_T[i73 + i70] = R_T[i72 + i71];
            i72 += 3;
        }

        i70 += 3;
    }

    b_parentFrame = b_obj->parentFrame;
    Frame_getWorldSpatialVelocity(b_parentFrame, b_b);
    i70 = 0;
    for (i71 = 0; i71 < 3; i71++) {
        for (i72 = 0; i72 < 3; i72++) {
            R_T[i72 + i70] = -c_R_T[i72 + i70];
        }

        i70 += 3;
    }

    iv0[0] = 0;
    iv0[3] = 0;
    iv0[6] = 0;
    iv0[1] = 0;
    iv0[4] = 0;
    iv0[7] = 0;
    iv0[2] = 0;
    iv0[5] = 0;
    iv0[8] = 0;
    i70 = 0;
    i71 = 0;
    for (i72 = 0; i72 < 3; i72++) {
        i73 = 0;
        for (i74 = 0; i74 < 3; i74++) {
            d_R_T[i73 + i72] = 0.0;
            i75 = 0;
            for (i76 = 0; i76 < 3; i76++) {
                d_R_T[i73 + i72] += R_T[i75 + i72] * (double)iv0[i76 + i73];
                i75 += 3;
            }

            e_R_T[i74 + i70] = c_R_T[i74 + i71];
            i73 += 3;
        }

        i70 += 6;
        i71 += 3;
    }

    i70 = 0;
    i71 = 0;
    for (i72 = 0; i72 < 3; i72++) {
        for (i73 = 0; i73 < 3; i73++) {
            e_R_T[(i73 + i70) + 18] = d_R_T[i73 + i71];
            e_R_T[(i73 + i70) + 3] = 0.0;
            e_R_T[(i73 + i70) + 21] = c_R_T[i73 + i71];
        }

        i70 += 6;
        i71 += 3;
    }

    for (i70 = 0; i70 < 6; i70++) {
        s[i70] = 0.0;
        i71 = 0;
        for (i72 = 0; i72 < 6; i72++) {
            s[i70] += e_R_T[i71 + i70] * b_b[i72];
            i71 += 6;
        }
    }

    i70 = 0;
    for (i71 = 0; i71 < 3; i71++) {
        for (i72 = 0; i72 < 3; i72++) {
            c_R_T[i72 + i70] = -b_R_T[i72 + i70];
        }

        i70 += 3;
    }

    iv0[0] = 0;
    iv0[3] = 0;
    iv0[6] = 0;
    iv0[1] = 0;
    iv0[4] = 0;
    iv0[7] = 0;
    iv0[2] = 0;
    iv0[5] = 0;
    iv0[8] = 0;
    i70 = 0;
    i71 = 0;
    for (i72 = 0; i72 < 3; i72++) {
        i73 = 0;
        for (i74 = 0; i74 < 3; i74++) {
            R_T[i73 + i72] = 0.0;
            i75 = 0;
            for (i76 = 0; i76 < 3; i76++) {
                R_T[i73 + i72] += c_R_T[i75 + i72] * (double)iv0[i76 + i73];
                i75 += 3;
            }

            e_R_T[i74 + i70] = b_R_T[i74 + i71];
            i73 += 3;
        }

        i70 += 6;
        i71 += 3;
    }

    i70 = 0;
    i71 = 0;
    for (i72 = 0; i72 < 3; i72++) {
        for (i73 = 0; i73 < 3; i73++) {
            e_R_T[(i73 + i70) + 18] = R_T[i73 + i71];
            e_R_T[(i73 + i70) + 3] = 0.0;
            e_R_T[(i73 + i70) + 21] = b_R_T[i73 + i71];
        }

        i70 += 6;
        i71 += 3;
    }

    dv15[0] = 0.0;
    dv15[1] = 0.0;
    dv15[2] = 0.0;
    dv15[3] = -s[3];
    dv15[4] = -s[4];
    dv15[5] = 0.0;
    for (i70 = 0; i70 < 6; i70++) {
        b_b[i70] = 0.0;
        i71 = 0;
        for (i72 = 0; i72 < 6; i72++) {
            b_b[i70] += e_R_T[i71 + i70] * b[i72];
            i71 += 6;
        }

        worldSpatialVelocity[i70] = b_b[i70] + dv15[i70];
    }
}

static void b_Frame_getWorldTransform(Frame_2 *obj, Transform3d *worldTransform)
{
    Frame_2 *b_obj;
    Frame_1 *b_parentFrame;
    Frame *c_parentFrame;
    WorldFrame *d_parentFrame;
    int i;
    int i26;
    double relativeTransform_position_data[3];
    double relativeTransform_rotation_data[9];
    double d1;
    int i27;
    int i28;
    double p_data[3];
    double R_data[9];
    int i29;
    double b_p_data[3];
    double b_R_data[9];
    b_obj = obj;
    b_parentFrame = b_obj->parentFrame;
    c_parentFrame = b_parentFrame->parentFrame;
    d_parentFrame = c_parentFrame->parentFrame;
    *worldTransform = d_parentFrame->relativeTransform;
    for (i = 0; i < 3; i++) {
        relativeTransform_position_data[i] =
            c_parentFrame->relativeTransform.position.data[i];
    }

    for (i26 = 0; i26 < 9; i26++) {
        relativeTransform_rotation_data[i26] =
            c_parentFrame->relativeTransform.rotation.data[i26];
    }

    for (i26 = 0; i26 < 3; i26++) {
        d1 = 0.0;
        i27 = 0;
        for (i28 = 0; i28 < 3; i28++) {
            d1 += worldTransform->rotation.data[i27 + i26] *
                relativeTransform_position_data[i28];
            R_data[i27 + i26] = 0.0;
            i29 = 0;
            for (i = 0; i < 3; i++) {
                R_data[i27 + i26] += worldTransform->rotation.data[i29 + i26] *
                    relativeTransform_rotation_data[i + i27];
                i29 += 3;
            }

            i27 += 3;
        }

        p_data[i26] = worldTransform->position.data[i26] + d1;
    }

    for (i = 0; i < 3; i++) {
        relativeTransform_position_data[i] =
            b_parentFrame->relativeTransform.position.data[i];
    }

    for (i26 = 0; i26 < 9; i26++) {
        relativeTransform_rotation_data[i26] =
            b_parentFrame->relativeTransform.rotation.data[i26];
    }

    for (i26 = 0; i26 < 3; i26++) {
        d1 = 0.0;
        i27 = 0;
        for (i28 = 0; i28 < 3; i28++) {
            d1 += R_data[i27 + i26] * relativeTransform_position_data[i28];
            b_R_data[i27 + i26] = 0.0;
            i29 = 0;
            for (i = 0; i < 3; i++) {
                b_R_data[i27 + i26] += R_data[i29 + i26] *
                    relativeTransform_rotation_data[i + i27];
                i29 += 3;
            }

            i27 += 3;
        }

        b_p_data[i26] = p_data[i26] + d1;
    }

    b_obj = obj;
    for (i = 0; i < 3; i++) {
        relativeTransform_position_data[i] =
            b_obj->relativeTransform.position.data[i];
    }

    for (i26 = 0; i26 < 9; i26++) {
        relativeTransform_rotation_data[i26] =
            b_obj->relativeTransform.rotation.data[i26];
    }

    for (i = 0; i < 3; i++) {
        d1 = 0.0;
        i26 = 0;
        for (i27 = 0; i27 < 3; i27++) {
            d1 += b_R_data[i26 + i] * relativeTransform_position_data[i27];
            R_data[i26 + i] = 0.0;
            i28 = 0;
            for (i29 = 0; i29 < 3; i29++) {
                R_data[i26 + i] += b_R_data[i28 + i] *
                    relativeTransform_rotation_data[i29 + i26];
                i28 += 3;
            }

            i26 += 3;
        }

        worldTransform->position.data[i] = b_p_data[i] + d1;
    }

    memcpy(&worldTransform->rotation.data[0], &R_data[0], 9U * sizeof(double));
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

static double b_clamp(double a)
{
    return fmax(fmin(a, 1.0), -1.0);
}

static void b_cubicInterpolation(double t, const double ts[3], const double qs[3],
    const double dqs[3], double *q, double *dq, double *ddqt)
{
    double b_x[3];
    int unusedU4;
    int j2;
    double a;
    double c[4];
    double dv25[16];
    b_x[0] = ts[0];
    b_x[1] = t;
    b_x[2] = ts[2];
    sort3(1, ts[0], 2, t, 3, ts[2], &unusedU4, &j2);
    a = b_x[j2 - 1];
    *q = qs[0];
    *dq = 0.0;
    *ddqt = 0.0;
    for (unusedU4 = 0; unusedU4 < 2; unusedU4++) {
        if ((a >= ts[unusedU4]) && (a <= ts[unusedU4 + 1])) {
            c[0] = qs[unusedU4];
            c[1] = dqs[unusedU4];
            c[2] = qs[unusedU4 + 1];
            c[3] = dqs[unusedU4 + 1];
            dv25[0] = 1.0;
            dv25[4] = ts[unusedU4];
            dv25[8] = ts[unusedU4] * ts[unusedU4];
            dv25[12] = pow(ts[unusedU4], 3.0);
            dv25[1] = 0.0;
            dv25[5] = 1.0;
            dv25[9] = 2.0 * ts[unusedU4];
            dv25[13] = 3.0 * (ts[unusedU4] * ts[unusedU4]);
            dv25[2] = 1.0;
            dv25[6] = ts[unusedU4 + 1];
            dv25[10] = ts[unusedU4 + 1] * ts[unusedU4 + 1];
            dv25[14] = pow(ts[unusedU4 + 1], 3.0);
            dv25[3] = 0.0;
            dv25[7] = 1.0;
            dv25[11] = 2.0 * ts[unusedU4 + 1];
            dv25[15] = 3.0 * (ts[unusedU4 + 1] * ts[unusedU4 + 1]);
            c_mldivide(dv25, c);
            *q = ((c[0] + c[1] * a) + c[2] * (a * a)) + c[3] * pow(a, 3.0);
            *dq = (c[1] + 2.0 * c[2] * a) + 3.0 * c[3] * (a * a);
            *ddqt = 2.0 * c[2] + 6.0 * c[3] * a;
        }
    }
}

static int b_ixamax(int n, const double b_x[5], int ix0)
{
    int idxmax;
    int ix;
    double smax;
    int k;
    double s;
    if (n < 1) {
        idxmax = 0;
    } else {
        idxmax = 1;
        if (n > 1) {
            ix = ix0 - 1;
            smax = fabs(b_x[ix0 - 1]);
            for (k = 2; k <= n; k++) {
                ix++;
                s = fabs(b_x[ix]);
                if (s > smax) {
                    idxmax = k;
                    smax = s;
                }
            }
        }
    }

    return idxmax;
}

static void b_mldivide(const double A[30], const double B[6], double Y[5])
{
    double b_A[30];
    double tau[5];
    int jpvt[5];
    int rankR;
    double tol;
    int i;
    int j;
    double b_B[6];
    memcpy(&b_A[0], &A[0], 30U * sizeof(double));
    b_xgeqp3(b_A, tau, jpvt);
    rankR = 0;
    tol = 6.0 * fabs(b_A[0]) * 2.2204460492503131E-16;
    while ((rankR < 5) && (fabs(b_A[rankR + 6 * rankR]) >= tol)) {
        rankR++;
    }

    for (i = 0; i < 6; i++) {
        b_B[i] = B[i];
    }

    for (j = 0; j < 5; j++) {
        Y[j] = 0.0;
        if (tau[j] != 0.0) {
            tol = b_B[j];
            for (i = j + 1; i + 1 < 7; i++) {
                tol += b_A[i + 6 * j] * b_B[i];
            }

            tol *= tau[j];
            if (tol != 0.0) {
                b_B[j] -= tol;
                for (i = j + 1; i + 1 < 7; i++) {
                    b_B[i] -= b_A[i + 6 * j] * tol;
                }
            }
        }
    }

    for (i = 0; i + 1 <= rankR; i++) {
        Y[jpvt[i] - 1] = b_B[i];
    }

    for (j = rankR - 1; j + 1 > 0; j--) {
        Y[jpvt[j] - 1] /= b_A[j + 6 * j];
        for (i = 0; i + 1 <= j; i++) {
            Y[jpvt[i] - 1] -= Y[jpvt[j] - 1] * b_A[i + 6 * j];
        }
    }
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

static void b_xgeqp3(double A[30], double tau[5], int jpvt[5])
{
    int k;
    int j;
    int i;
    double work[5];
    int i_i;
    double xnorm;
    int pvt;
    double vn1[5];
    double vn2[5];
    double t;
    double absxk;
    double beta1;
    int i132;
    int i_ip1;
    int lastv;
    int lastc;
    bool exitg2;
    int exitg1;
    int ix;
    int ijA;
    k = 1;
    for (j = 0; j < 5; j++) {
        jpvt[j] = 1 + j;
        work[j] = 0.0;
        xnorm = c_xnrm2(A, k);
        vn2[j] = xnorm;
        k += 6;
        vn1[j] = xnorm;
    }

    for (i = 0; i < 5; i++) {
        i_i = i + i * 6;
        pvt = (i + b_ixamax(5 - i, vn1, i + 1)) - 1;
        if (pvt + 1 != i + 1) {
            b_xswap(A, 1 + 6 * pvt, 1 + 6 * i);
            k = jpvt[pvt];
            jpvt[pvt] = jpvt[i];
            jpvt[i] = k;
            vn1[pvt] = vn1[i];
            vn2[pvt] = vn2[i];
        }

        t = A[i_i];
        absxk = 0.0;
        xnorm = d_xnrm2(5 - i, A, i_i + 2);
        if (xnorm != 0.0) {
            beta1 = rt_hypotd(A[i_i], xnorm);
            if (A[i_i] >= 0.0) {
                beta1 = -beta1;
            }

            if (fabs(beta1) < 1.0020841800044864E-292) {
                pvt = 0;
                i132 = (i_i - i) + 6;
                do {
                    pvt++;
                    for (k = i_i + 1; k + 1 <= i132; k++) {
                        A[k] *= 9.9792015476736E+291;
                    }

                    beta1 *= 9.9792015476736E+291;
                    t *= 9.9792015476736E+291;
                } while (!(fabs(beta1) >= 1.0020841800044864E-292));

                beta1 = rt_hypotd(t, d_xnrm2(5 - i, A, i_i + 2));
                if (t >= 0.0) {
                    beta1 = -beta1;
                }

                absxk = (beta1 - t) / beta1;
                t = 1.0 / (t - beta1);
                i132 = (i_i - i) + 6;
                for (k = i_i + 1; k + 1 <= i132; k++) {
                    A[k] *= t;
                }

                for (k = 1; k <= pvt; k++) {
                    beta1 *= 1.0020841800044864E-292;
                }

                t = beta1;
            } else {
                absxk = (beta1 - A[i_i]) / beta1;
                t = 1.0 / (A[i_i] - beta1);
                i132 = (i_i - i) + 6;
                for (k = i_i + 1; k + 1 <= i132; k++) {
                    A[k] *= t;
                }

                t = beta1;
            }
        }

        tau[i] = absxk;
        A[i_i] = t;
        if (i + 1 < 5) {
            t = A[i_i];
            A[i_i] = 1.0;
            i_ip1 = (i + (i + 1) * 6) + 1;
            if (tau[i] != 0.0) {
                lastv = 6 - i;
                pvt = (i_i - i) + 5;
                while ((lastv > 0) && (A[pvt] == 0.0)) {
                    lastv--;
                    pvt--;
                }

                lastc = 4 - i;
                exitg2 = false;
                while ((!exitg2) && (lastc > 0)) {
                    pvt = i_ip1 + (lastc - 1) * 6;
                    j = pvt;
                    do {
                        exitg1 = 0;
                        if (j <= (pvt + lastv) - 1) {
                            if (A[j - 1] != 0.0) {
                                exitg1 = 1;
                            } else {
                                j++;
                            }
                        } else {
                            lastc--;
                            exitg1 = 2;
                        }
                    } while (exitg1 == 0);

                    if (exitg1 == 1) {
                        exitg2 = true;
                    }
                }
            } else {
                lastv = 0;
                lastc = 0;
            }

            if (lastv > 0) {
                if (lastc != 0) {
                    for (pvt = 1; pvt <= lastc; pvt++) {
                        work[pvt - 1] = 0.0;
                    }

                    pvt = 0;
                    i132 = i_ip1 + 6 * (lastc - 1);
                    for (k = i_ip1; k <= i132; k += 6) {
                        ix = i_i;
                        xnorm = 0.0;
                        ijA = (k + lastv) - 1;
                        for (j = k; j <= ijA; j++) {
                            xnorm += A[j - 1] * A[ix];
                            ix++;
                        }

                        work[pvt] += xnorm;
                        pvt++;
                    }
                }

                if (!(-tau[i] == 0.0)) {
                    pvt = i_ip1 - 1;
                    k = 0;
                    for (j = 1; j <= lastc; j++) {
                        if (work[k] != 0.0) {
                            xnorm = work[k] * -tau[i];
                            ix = i_i;
                            i132 = lastv + pvt;
                            for (ijA = pvt; ijA + 1 <= i132; ijA++) {
                                A[ijA] += A[ix] * xnorm;
                                ix++;
                            }
                        }

                        k++;
                        pvt += 6;
                    }
                }
            }

            A[i_i] = t;
        }

        for (j = i + 1; j + 1 < 6; j++) {
            pvt = (i + 6 * j) + 1;
            if (vn1[j] != 0.0) {
                xnorm = fabs(A[i + 6 * j]) / vn1[j];
                xnorm = 1.0 - xnorm * xnorm;
                if (xnorm < 0.0) {
                    xnorm = 0.0;
                }

                beta1 = vn1[j] / vn2[j];
                beta1 = xnorm * (beta1 * beta1);
                if (beta1 <= 1.4901161193847656E-8) {
                    xnorm = 0.0;
                    if (5 - i == 1) {
                        xnorm = fabs(A[pvt]);
                    } else {
                        beta1 = 2.2250738585072014E-308;
                        k = (pvt - i) + 5;
                        while (pvt + 1 <= k) {
                            absxk = fabs(A[pvt]);
                            if (absxk > beta1) {
                                t = beta1 / absxk;
                                xnorm = 1.0 + xnorm * t * t;
                                beta1 = absxk;
                            } else {
                                t = absxk / beta1;
                                xnorm += t * t;
                            }

                            pvt++;
                        }

                        xnorm = beta1 * sqrt(xnorm);
                    }

                    vn1[j] = xnorm;
                    vn2[j] = vn1[j];
                } else {
                    vn1[j] *= sqrt(xnorm);
                }
            }
        }
    }
}

static double b_xnrm2(int n, const double b_x[6], int ix0)
{
    double y;
    double scale;
    int kend;
    int k;
    double absxk;
    double t;
    y = 0.0;
    if (!(n < 1)) {
        if (n == 1) {
            y = fabs(b_x[ix0 - 1]);
        } else {
            scale = 2.2250738585072014E-308;
            kend = (ix0 + n) - 1;
            for (k = ix0; k <= kend; k++) {
                absxk = fabs(b_x[k - 1]);
                if (absxk > scale) {
                    t = scale / absxk;
                    y = 1.0 + y * t * t;
                    scale = absxk;
                } else {
                    t = absxk / scale;
                    y += t * t;
                }
            }

            y = scale * sqrt(y);
        }
    }

    return y;
}

static void b_xswap(double b_x[30], int ix0, int iy0)
{
    int ix;
    int iy;
    int k;
    double temp;
    ix = ix0 - 1;
    iy = iy0 - 1;
    for (k = 0; k < 6; k++) {
        temp = b_x[ix];
        b_x[ix] = b_x[iy];
        b_x[iy] = temp;
        ix++;
        iy++;
    }
}

static void c_Frame_Frame(Frame_5 **obj, WorldFrame *b_parentFrame)
{
    Frame_5 *b_this;
    int i;
    static const char b_name[14] = { 'D', 'e', 's', 'i', 'r', 'e', 'd', ' ', 'P',
        'e', 'l', 'v', 'i', 's' };

    WorldFrame *c_parentFrame;
    signed char obj_rotation_data[9];
    int k;
    b_this = *obj;
    *obj = b_this;
    b_this = *obj;
    for (i = 0; i < 14; i++) {
        b_this->name[i] = b_name[i];
    }

    b_this = *obj;
    c_parentFrame = b_parentFrame;
    b_this->parentFrame = c_parentFrame;
    for (i = 0; i < 9; i++) {
        obj_rotation_data[i] = 0;
    }

    i = 0;
    for (k = 0; k < 3; k++) {
        obj_rotation_data[i] = 1;
        i += 4;
    }

    for (i = 0; i < 3; i++) {
        (*obj)->relativeTransform.position.data[i] = 0.0;
    }

    for (i = 0; i < 9; i++) {
        (*obj)->relativeTransform.rotation.data[i] = obj_rotation_data[i];
    }

    for (i = 0; i < 6; i++) {
        (*obj)->relativeSpatialVelocity[i] = 0.0;
    }
}

static void c_Frame_getSpatialVelocity(Frame_7 *obj, Frame_5 *relativeTo,
    Frame_5 *inCoordinatesOf, double spatialVelocity[6])
{
    Frame_7 *b_obj;
    int i;
    int i103;
    double relativeTransform_position_data[3];
    double relativeTransform_rotation_data[9];
    int i104;
    WorldFrame *b_parentFrame;
    int i105;
    int i106;
    Frame_5 *c_obj;
    double worldSpatialVelocity[6];
    double R_T[9];
    double b_relativeSpatialVelocity[6];
    double worldTransform_position_data[3];
    double worldTransform_rotation_data[9];
    double b_relativeTransform_position_da[3];
    double R_data[9];
    double d12;
    double b_worldTransform_position_data[3];
    double b_worldTransform_rotation_data[9];
    double transform_position_data[3];
    double transform_rotation_data[9];
    bool b_bool;
    char a[18];
    char b[18];
    int exitg1;
    double b_worldSpatialVelocity[6];
    double b_R_T[9];
    double c_relativeSpatialVelocity[6];
    double c_data[9];
    int i107;
    int i108;
    double c_R_T[36];
    double d_R_T[36];
    double e_R_T[6];
    double b_transform_rotation_data[36];
    double f_R_T[6];
    b_obj = obj;
    for (i = 0; i < 3; i++) {
        relativeTransform_position_data[i] =
            b_obj->relativeTransform.position.data[i];
    }

    for (i103 = 0; i103 < 9; i103++) {
        relativeTransform_rotation_data[i103] =
            b_obj->relativeTransform.rotation.data[i103];
    }

    i103 = 0;
    for (i104 = 0; i104 < 3; i104++) {
        i105 = 0;
        for (i106 = 0; i106 < 3; i106++) {
            R_T[i106 + i103] = relativeTransform_rotation_data[i105 + i104];
            i105 += 3;
        }

        i103 += 3;
    }

    b_parentFrame = b_obj->parentFrame;
    for (i = 0; i < 6; i++) {
        worldSpatialVelocity[i] = b_parentFrame->relativeSpatialVelocity[i];
        b_relativeSpatialVelocity[i] = b_obj->relativeSpatialVelocity[i];
    }

    c_obj = relativeTo;
    b_obj = obj;
    b_parentFrame = b_obj->parentFrame;
    for (i = 0; i < 3; i++) {
        worldTransform_position_data[i] =
            b_parentFrame->relativeTransform.position.data[i];
    }

    for (i103 = 0; i103 < 9; i103++) {
        worldTransform_rotation_data[i103] =
            b_parentFrame->relativeTransform.rotation.data[i103];
    }

    for (i = 0; i < 3; i++) {
        b_relativeTransform_position_da[i] =
            b_obj->relativeTransform.position.data[i];
    }

    for (i103 = 0; i103 < 9; i103++) {
        relativeTransform_rotation_data[i103] =
            b_obj->relativeTransform.rotation.data[i103];
    }

    for (i = 0; i < 3; i++) {
        d12 = 0.0;
        i103 = 0;
        for (i104 = 0; i104 < 3; i104++) {
            d12 += worldTransform_rotation_data[i103 + i] *
                b_relativeTransform_position_da[i104];
            R_data[i103 + i] = 0.0;
            i105 = 0;
            for (i106 = 0; i106 < 3; i106++) {
                R_data[i103 + i] += worldTransform_rotation_data[i105 + i] *
                    relativeTransform_rotation_data[i106 + i103];
                i105 += 3;
            }

            i103 += 3;
        }

        worldTransform_position_data[i] += d12;
    }

    memcpy(&worldTransform_rotation_data[0], &R_data[0], 9U * sizeof(double));
    b_parentFrame = c_obj->parentFrame;
    for (i = 0; i < 3; i++) {
        b_worldTransform_position_data[i] =
            b_parentFrame->relativeTransform.position.data[i];
    }

    for (i103 = 0; i103 < 9; i103++) {
        b_worldTransform_rotation_data[i103] =
            b_parentFrame->relativeTransform.rotation.data[i103];
    }

    for (i = 0; i < 3; i++) {
        b_relativeTransform_position_da[i] =
            c_obj->relativeTransform.position.data[i];
    }

    for (i103 = 0; i103 < 9; i103++) {
        relativeTransform_rotation_data[i103] =
            c_obj->relativeTransform.rotation.data[i103];
    }

    for (i = 0; i < 3; i++) {
        d12 = 0.0;
        i103 = 0;
        for (i104 = 0; i104 < 3; i104++) {
            d12 += b_worldTransform_rotation_data[i103 + i] *
                b_relativeTransform_position_da[i104];
            R_data[i103 + i] = 0.0;
            i105 = 0;
            for (i106 = 0; i106 < 3; i106++) {
                R_data[i103 + i] += b_worldTransform_rotation_data[i105 + i] *
                    relativeTransform_rotation_data[i106 + i103];
                i105 += 3;
            }

            i103 += 3;
        }

        b_worldTransform_position_data[i] += d12;
    }

    memcpy(&b_worldTransform_rotation_data[0], &R_data[0], 9U * sizeof(double));
    Transform3d_mldivide(worldTransform_position_data,
                         worldTransform_rotation_data,
                         b_worldTransform_position_data,
                         b_worldTransform_rotation_data, transform_position_data,
                         transform_rotation_data);
    for (i103 = 0; i103 < 18; i103++) {
        a[i103] = b_obj->name[i103];
        b[i103] = b_obj->name[i103];
    }

    b_bool = false;
    i = 0;
    do {
        exitg1 = 0;
        if (i + 1 < 19) {
            if (a[i] != b[i]) {
                exitg1 = 1;
            } else {
                i++;
            }
        } else {
            b_bool = true;
            exitg1 = 1;
        }
    } while (exitg1 == 0);

    if (!b_bool) {
        b_parentFrame = b_obj->parentFrame;
        for (i103 = 0; i103 < 9; i103++) {
            worldTransform_rotation_data[i103] =
                b_parentFrame->relativeTransform.rotation.data[i103];
            relativeTransform_rotation_data[i103] =
                b_obj->relativeTransform.rotation.data[i103];
        }

        for (i103 = 0; i103 < 3; i103++) {
            i104 = 0;
            for (i105 = 0; i105 < 3; i105++) {
                R_data[i104 + i103] = 0.0;
                i106 = 0;
                for (i = 0; i < 3; i++) {
                    R_data[i104 + i103] += worldTransform_rotation_data[i106 +
                        i103] * relativeTransform_rotation_data[i + i104];
                    i106 += 3;
                }

                i104 += 3;
            }
        }

        b_parentFrame = b_obj->parentFrame;
        for (i103 = 0; i103 < 9; i103++) {
            worldTransform_rotation_data[i103] =
                b_parentFrame->relativeTransform.rotation.data[i103];
            relativeTransform_rotation_data[i103] =
                b_obj->relativeTransform.rotation.data[i103];
        }

        for (i103 = 0; i103 < 3; i103++) {
            i104 = 0;
            for (i105 = 0; i105 < 3; i105++) {
                b_R_T[i104 + i103] = 0.0;
                i106 = 0;
                for (i = 0; i < 3; i++) {
                    b_R_T[i104 + i103] += worldTransform_rotation_data[i106 +
                        i103] * relativeTransform_rotation_data[i + i104];
                    i106 += 3;
                }

                i104 += 3;
            }
        }

        i103 = 0;
        for (i104 = 0; i104 < 3; i104++) {
            d12 = 0.0;
            i105 = 0;
            for (i106 = 0; i106 < 3; i106++) {
                c_data[i105 + i104] = 0.0;
                for (i = 0; i < 3; i++) {
                    c_data[i105 + i104] += R_data[i + i103] * b_R_T[i + i105];
                }

                d12 += c_data[i105 + i104] * transform_position_data[i106];
                i105 += 3;
            }

            b_relativeTransform_position_da[i104] = d12;
            i103 += 3;
        }

        for (i = 0; i < 3; i++) {
            i103 = 0;
            for (i104 = 0; i104 < 3; i104++) {
                R_data[i103 + i] = 0.0;
                i105 = 0;
                for (i106 = 0; i106 < 3; i106++) {
                    R_data[i103 + i] += c_data[i105 + i] *
                        transform_rotation_data[i106 + i103];
                    i105 += 3;
                }

                i103 += 3;
            }

            transform_position_data[i] = b_relativeTransform_position_da[i];
        }

        memcpy(&transform_rotation_data[0], &R_data[0], 9U * sizeof(double));
    }

    c_obj = relativeTo;
    for (i = 0; i < 3; i++) {
        b_relativeTransform_position_da[i] =
            c_obj->relativeTransform.position.data[i];
    }

    for (i103 = 0; i103 < 9; i103++) {
        relativeTransform_rotation_data[i103] =
            c_obj->relativeTransform.rotation.data[i103];
    }

    i103 = 0;
    for (i104 = 0; i104 < 3; i104++) {
        i105 = 0;
        for (i106 = 0; i106 < 3; i106++) {
            b_R_T[i106 + i103] = relativeTransform_rotation_data[i105 + i104];
            i105 += 3;
        }

        i103 += 3;
    }

    b_parentFrame = c_obj->parentFrame;
    for (i = 0; i < 6; i++) {
        b_worldSpatialVelocity[i] = b_parentFrame->relativeSpatialVelocity[i];
        c_relativeSpatialVelocity[i] = c_obj->relativeSpatialVelocity[i];
    }

    i103 = 0;
    for (i104 = 0; i104 < 3; i104++) {
        for (i105 = 0; i105 < 3; i105++) {
            worldTransform_rotation_data[i105 + i103] = -R_T[i105 + i103];
        }

        i103 += 3;
    }

    R_data[0] = 0.0;
    R_data[3] = -relativeTransform_position_data[2];
    R_data[6] = relativeTransform_position_data[1];
    R_data[1] = relativeTransform_position_data[2];
    R_data[4] = 0.0;
    R_data[7] = -relativeTransform_position_data[0];
    R_data[2] = -relativeTransform_position_data[1];
    R_data[5] = relativeTransform_position_data[0];
    R_data[8] = 0.0;
    i103 = 0;
    i104 = 0;
    for (i105 = 0; i105 < 3; i105++) {
        i106 = 0;
        for (i = 0; i < 3; i++) {
            b_worldTransform_rotation_data[i106 + i105] = 0.0;
            i107 = 0;
            for (i108 = 0; i108 < 3; i108++) {
                b_worldTransform_rotation_data[i106 + i105] +=
                    worldTransform_rotation_data[i107 + i105] * R_data[i108 +
                    i106];
                i107 += 3;
            }

            c_R_T[i + i103] = R_T[i + i104];
            i106 += 3;
        }

        i103 += 6;
        i104 += 3;
    }

    R_data[0] = 0.0;
    R_data[3] = -transform_position_data[2];
    R_data[6] = transform_position_data[1];
    R_data[1] = transform_position_data[2];
    R_data[4] = 0.0;
    R_data[7] = -transform_position_data[0];
    R_data[2] = -transform_position_data[1];
    R_data[5] = transform_position_data[0];
    R_data[8] = 0.0;
    i103 = 0;
    i104 = 0;
    for (i105 = 0; i105 < 3; i105++) {
        i106 = 0;
        for (i = 0; i < 3; i++) {
            c_R_T[(i + i103) + 18] = b_worldTransform_rotation_data[i + i104];
            c_R_T[(i + i103) + 3] = 0.0;
            c_R_T[(i + i103) + 21] = R_T[i + i104];
            c_data[i106 + i105] = 0.0;
            i107 = 0;
            for (i108 = 0; i108 < 3; i108++) {
                c_data[i106 + i105] += R_data[i107 + i105] *
                    transform_rotation_data[i108 + i106];
                i107 += 3;
            }

            worldTransform_rotation_data[i + i104] = -b_R_T[i + i104];
            i106 += 3;
        }

        i103 += 6;
        i104 += 3;
    }

    R_data[0] = 0.0;
    R_data[3] = -b_relativeTransform_position_da[2];
    R_data[6] = b_relativeTransform_position_da[1];
    R_data[1] = b_relativeTransform_position_da[2];
    R_data[4] = 0.0;
    R_data[7] = -b_relativeTransform_position_da[0];
    R_data[2] = -b_relativeTransform_position_da[1];
    R_data[5] = b_relativeTransform_position_da[0];
    R_data[8] = 0.0;
    i103 = 0;
    i104 = 0;
    for (i105 = 0; i105 < 3; i105++) {
        i106 = 0;
        for (i = 0; i < 3; i++) {
            R_T[i106 + i105] = 0.0;
            i107 = 0;
            for (i108 = 0; i108 < 3; i108++) {
                R_T[i106 + i105] += worldTransform_rotation_data[i107 + i105] *
                    R_data[i108 + i106];
                i107 += 3;
            }

            d_R_T[i + i103] = b_R_T[i + i104];
            i106 += 3;
        }

        i103 += 6;
        i104 += 3;
    }

    i103 = 0;
    i104 = 0;
    for (i105 = 0; i105 < 3; i105++) {
        for (i106 = 0; i106 < 3; i106++) {
            d_R_T[(i106 + i103) + 18] = R_T[i106 + i104];
            d_R_T[(i106 + i103) + 3] = 0.0;
            d_R_T[(i106 + i103) + 21] = b_R_T[i106 + i104];
            b_transform_rotation_data[i106 + i103] =
                transform_rotation_data[i106 + i104];
            b_transform_rotation_data[(i106 + i103) + 18] = c_data[i106 + i104];
            b_transform_rotation_data[(i106 + i103) + 3] = 0.0;
            b_transform_rotation_data[(i106 + i103) + 21] =
                transform_rotation_data[i106 + i104];
        }

        i103 += 6;
        i104 += 3;
    }

    for (i103 = 0; i103 < 6; i103++) {
        d12 = 0.0;
        for (i104 = 0; i104 < 6; i104++) {
            d12 += d_R_T[i103 + 6 * i104] * b_worldSpatialVelocity[i104];
        }

        e_R_T[i103] = d12 + c_relativeSpatialVelocity[i103];
        d12 = 0.0;
        for (i104 = 0; i104 < 6; i104++) {
            d12 += c_R_T[i103 + 6 * i104] * worldSpatialVelocity[i104];
        }

        f_R_T[i103] = d12 + b_relativeSpatialVelocity[i103];
    }

    for (i103 = 0; i103 < 6; i103++) {
        worldSpatialVelocity[i103] = 0.0;
        i104 = 0;
        for (i105 = 0; i105 < 6; i105++) {
            worldSpatialVelocity[i103] += b_transform_rotation_data[i104 + i103]
                * e_R_T[i105];
            i104 += 6;
        }

        spatialVelocity[i103] = f_R_T[i103] - worldSpatialVelocity[i103];
    }

    g_Frame_getTransform(obj, inCoordinatesOf, relativeTransform_position_data,
                         relativeTransform_rotation_data);
    for (i103 = 0; i103 < 3; i103++) {
        b_relativeTransform_position_da[i103] = 0.0;
        worldTransform_position_data[i103] = 0.0;
        i104 = 0;
        for (i105 = 0; i105 < 3; i105++) {
            b_relativeTransform_position_da[i103] +=
                relativeTransform_rotation_data[i104 + i103] *
                spatialVelocity[i105];
            worldTransform_position_data[i103] +=
                relativeTransform_rotation_data[i104 + i103] *
                spatialVelocity[i105 + 3];
            i104 += 3;
        }
    }

    for (i103 = 0; i103 < 3; i103++) {
        spatialVelocity[i103] = b_relativeTransform_position_da[i103];
        spatialVelocity[i103 + 3] = worldTransform_position_data[i103];
    }
}

static void c_Frame_getTransform(Frame_3 *obj, ZUpFrame *withRespectTo, double
    transform_position_data[3], double transform_rotation_data[9])
{
    ZUpFrame *inCoordinatesOf;
    Frame_3 *b_obj;
    Frame_2 *b_parentFrame;
    int i;
    double relativeTransform_position_data[3];
    Transform3d r11;
    double relativeTransform_rotation_data[9];
    Transform3d r12;
    double a_position_data[3];
    double a_rotation_data[9];
    double d4;
    int i53;
    double R_data[9];
    double p_data[3];
    int i54;
    Rotation3d a_rotation;
    ZUpFrame *a;
    int i55;
    int i56;
    bool b_bool;
    char b_a[11];
    char b[11];
    int exitg1;
    double c_data[9];
    inCoordinatesOf = withRespectTo;
    b_obj = obj;
    b_parentFrame = b_obj->parentFrame;
    for (i = 0; i < 3; i++) {
        relativeTransform_position_data[i] =
            b_obj->relativeTransform.position.data[i];
    }

    for (i = 0; i < 9; i++) {
        relativeTransform_rotation_data[i] =
            b_obj->relativeTransform.rotation.data[i];
    }

    b_Frame_getWorldTransform(b_parentFrame, &r11);
    for (i = 0; i < 3; i++) {
        a_position_data[i] = r11.position.data[i];
    }

    b_Frame_getWorldTransform(b_parentFrame, &r12);
    memcpy(&a_rotation_data[0], &r12.rotation.data[0], 9U * sizeof(double));
    for (i = 0; i < 3; i++) {
        d4 = 0.0;
        i53 = 0;
        for (i54 = 0; i54 < 3; i54++) {
            d4 += a_rotation_data[i53 + i] * relativeTransform_position_data[i54];
            R_data[i53 + i] = 0.0;
            i55 = 0;
            for (i56 = 0; i56 < 3; i56++) {
                R_data[i53 + i] += a_rotation_data[i55 + i] *
                    relativeTransform_rotation_data[i56 + i53];
                i55 += 3;
            }

            i53 += 3;
        }

        p_data[i] = a_position_data[i] + d4;
    }

    for (i = 0; i < 3; i++) {
        relativeTransform_position_data[i] = p_data[i];
    }

    memcpy(&relativeTransform_rotation_data[0], &R_data[0], 9U * sizeof(double));
    Frame_getWorldTransform(withRespectTo, a_position_data, &a_rotation);
    memcpy(&a_rotation_data[0], &a_rotation.data[0], 9U * sizeof(double));
    Transform3d_mldivide(a_position_data, a_rotation_data,
                         relativeTransform_position_data,
                         relativeTransform_rotation_data,
                         transform_position_data, transform_rotation_data);
    a = withRespectTo;
    for (i = 0; i < 11; i++) {
        b_a[i] = a->name[i];
        b[i] = inCoordinatesOf->name[i];
    }

    b_bool = false;
    i = 0;
    do {
        exitg1 = 0;
        if (i + 1 < 12) {
            if (b_a[i] != b[i]) {
                exitg1 = 1;
            } else {
                i++;
            }
        } else {
            b_bool = true;
            exitg1 = 1;
        }
    } while (exitg1 == 0);

    if (!b_bool) {
        Frame_getWorldTransform(inCoordinatesOf, relativeTransform_position_data,
                                &a_rotation);
        memcpy(&relativeTransform_rotation_data[0], &a_rotation.data[0], 9U *
               sizeof(double));
        Frame_getWorldTransform(withRespectTo, a_position_data, &a_rotation);
        memcpy(&a_rotation_data[0], &a_rotation.data[0], 9U * sizeof(double));
        for (i = 0; i < 3; i++) {
            d4 = 0.0;
            for (i53 = 0; i53 < 3; i53++) {
                c_data[i + 3 * i53] = 0.0;
                for (i54 = 0; i54 < 3; i54++) {
                    c_data[i + 3 * i53] += relativeTransform_rotation_data[i54 +
                        3 * i] * a_rotation_data[i54 + 3 * i53];
                }

                d4 += c_data[i + 3 * i53] * transform_position_data[i53];
            }

            p_data[i] = d4;
            for (i53 = 0; i53 < 3; i53++) {
                R_data[i + 3 * i53] = 0.0;
                for (i54 = 0; i54 < 3; i54++) {
                    R_data[i + 3 * i53] += c_data[i + 3 * i54] *
                        transform_rotation_data[i54 + 3 * i53];
                }
            }
        }

        for (i = 0; i < 3; i++) {
            transform_position_data[i] = p_data[i];
        }

        memcpy(&transform_rotation_data[0], &R_data[0], 9U * sizeof(double));
    }
}

static void c_Frame_getWorldTransform(Frame_4 *obj, double
    worldTransform_position_data[3], double worldTransform_rotation_data[9])
{
    Frame_4 *b_obj;
    Frame_2 *b_parentFrame;
    int i;
    int i37;
    double relativeTransform_position_data[3];
    Transform3d r9;
    double relativeTransform_rotation_data[9];
    Transform3d r10;
    double a_position_data[3];
    double a_rotation_data[9];
    double R_data[9];
    double d2;
    int i38;
    int i39;
    int i40;
    b_obj = obj;
    b_parentFrame = b_obj->parentFrame;
    b_obj = obj;
    for (i = 0; i < 3; i++) {
        relativeTransform_position_data[i] =
            b_obj->relativeTransform.position.data[i];
    }

    for (i37 = 0; i37 < 9; i37++) {
        relativeTransform_rotation_data[i37] =
            b_obj->relativeTransform.rotation.data[i37];
    }

    b_Frame_getWorldTransform(b_parentFrame, &r9);
    for (i = 0; i < 3; i++) {
        a_position_data[i] = r9.position.data[i];
    }

    b_Frame_getWorldTransform(b_parentFrame, &r10);
    memcpy(&a_rotation_data[0], &r10.rotation.data[0], 9U * sizeof(double));
    for (i = 0; i < 3; i++) {
        d2 = 0.0;
        i37 = 0;
        for (i38 = 0; i38 < 3; i38++) {
            d2 += a_rotation_data[i37 + i] * relativeTransform_position_data[i38];
            R_data[i37 + i] = 0.0;
            i39 = 0;
            for (i40 = 0; i40 < 3; i40++) {
                R_data[i37 + i] += a_rotation_data[i39 + i] *
                    relativeTransform_rotation_data[i40 + i37];
                i39 += 3;
            }

            i37 += 3;
        }

        worldTransform_position_data[i] = a_position_data[i] + d2;
    }

    memcpy(&worldTransform_rotation_data[0], &R_data[0], 9U * sizeof(double));
}

static InverseKinematicsFunction *c_InverseKinematicsFunction_Inv
    (InverseKinematicsFunction *obj)
{
    InverseKinematicsFunction *b_obj;
    b_obj = obj;
    d_InverseKinematicsFunction_Inv(&b_obj);
    return b_obj;
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
    double dv17[120];
    int i101;
    int i102;
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
    dv17[0] = 0.0;
    dv17[1] = ((((((((((((((t26 * 0.09 - t138) - t139) - t140) - t141) - t142) -
                       t143) - t144) - t145) + t2 * t24 * 0.0045) + t6 * t26 *
                   0.12) + t7 * t45 * 0.49544) + t5 * t52 * 0.04) + t5 * t54 *
                0.408) + t13 * t52 * 0.408) + t16 * t58 * 0.01762;
    dv17[2] = ((((((((((((((t157 + t158) + t159) + t160) + t161) + t162) + t163)
                      + t164) - t2 * t26 * 0.0045) - t8 * t29 * 0.06741) - t7 *
                   t31 * 0.06741) - t8 * t31 * 0.49544) - t13 * t37 * 0.04) -
                t16 * t43 * 0.05219) - t20 * t41 * 0.05219) - t20 * t43 *
        0.01762;
    dv17[3] = 0.0;
    dv17[4] = ((-t187 - t188) - t189) + t20 * t60 * 0.7660444431189779;
    dv17[5] = t198;
    dv17[6] = 0.0;
    dv17[7] = 0.0;
    dv17[8] = 0.0;
    dv17[9] = 0.0;
    dv17[10] = 0.0;
    dv17[11] = 0.0;
    dv17[12] = ((((((((((((t2 * 0.0045 - t3 * t25 * 0.12) - t5 * t127 * 0.408) -
                         t5 * t129 * 0.04) + t13 * t127 * 0.04) - t13 * t129 *
                       0.408) - t16 * t133 * 0.01762) + t16 * t135 * 0.05219) +
                    t20 * t133 * 0.05219) + t20 * t135 * 0.01762) - t3 * t7 *
                  t25 * 0.49544) + t3 * t8 * t25 * 0.06741) - t6 * t7 * t25 *
                0.06741) - t6 * t8 * t25 * 0.49544;
    dv17[13] = ((((((((((((t25 * t26 * 0.0045 + t5 * t148 * 0.408) - t13 * t148 *
                          0.04) - t16 * t153 * 0.05219) - t20 * t153 * 0.01762)
                       + t16 * t192 * 0.01762) + t5 * (t149 - t151) * 0.04) +
                     t13 * (t149 - t151) * 0.408) - t20 * (t154 + t13 * t150) *
                    0.05219) + t2 * t3 * t26 * 0.12) + t2 * t3 * t7 * t26 *
                  0.49544) - t2 * t3 * t8 * t26 * 0.06741) + t2 * t6 * t7 * t26 *
                0.06741) + t2 * t6 * t8 * t26 * 0.49544;
    dv17[14] = ((((((((((((t24 * t25 * 0.0045 + t5 * t167 * 0.408) - t13 * t167 *
                          0.04) - t16 * t171 * 0.05219) + t16 * t175 * 0.01762)
                       - t20 * t171 * 0.01762) - t20 * t175 * 0.05219) + t5 *
                     (t168 - t173) * 0.04) + t13 * (t168 - t173) * 0.408) + t2 *
                   t3 * t24 * 0.12) + t2 * t3 * t7 * t24 * 0.49544) - t2 * t3 *
                 t8 * t24 * 0.06741) + t2 * t6 * t7 * t24 * 0.06741) + t2 * t6 *
        t8 * t24 * 0.49544;
    dv17[15] = ((t16 * t133 * 0.7660444431189779 + t16 * t135 *
                 0.64278760968653947) + t20 * t133 * 0.64278760968653947) - t20 *
        t135 * 0.7660444431189779;
    dv17[16] = ((t16 * t153 * -0.64278760968653947 - t16 * t192 *
                 0.7660444431189779) - t20 * t192 * 0.64278760968653947) + t20 *
        (t152 - t190) * 0.7660444431189779;
    dv17[17] = ((t16 * t171 * -0.64278760968653947 - t16 * t175 *
                 0.7660444431189779) - t20 * t175 * 0.64278760968653947) + t20 *
        (t170 - t199) * 0.7660444431189779;
    dv17[18] = 0.0;
    dv17[19] = 0.0;
    dv17[20] = 0.0;
    dv17[21] = 0.0;
    dv17[22] = 0.0;
    dv17[23] = 0.0;
    dv17[24] = (((((((((((t136 + t137) + t2 * t6 * 0.12) + t5 * t12 * 0.04) - t5
                       * t15 * 0.408) + t12 * t13 * 0.408) - t16 * t19 * 0.01762)
                    - t16 * t23 * 0.05219) - t20 * t23 * 0.01762) + t13 * (t14 -
                   t17) * 0.04) - t2 * t3 * t7 * 0.06741) - t2 * t3 * t8 *
                0.49544) - t2 * t6 * t8 * 0.06741;
    dv17[25] = ((((((((((((t155 + t156) - t3 * t24 * 0.12) - t7 * t29 * 0.06741)
                        - t8 * t29 * 0.49544) - t7 * t31 * 0.49544) - t5 * t35 *
                      0.408) + t5 * t37 * 0.04) + t13 * t35 * 0.04) + t13 * t37 *
                   0.408) - t16 * t41 * 0.05219) - t16 * t43 * 0.01762) - t20 *
                t41 * 0.01762) + t6 * t25 * t26 * 0.12;
    dv17[26] = ((((((((((((t176 + t177) + t178) + t179) + t180) + t181) + t3 *
                      t26 * 0.12) - t8 * t48 * 0.06741) + t5 * t52 * 0.408) - t5
                   * t54 * 0.04) - t13 * t52 * 0.04) - t13 * t54 * 0.408) - t20 *
                t60 * 0.05219) + t6 * t24 * t25 * 0.12;
    dv17[27] = t186;
    dv17[28] = t196;
    dv17[29] = t201;
    dv17[30] = 0.0;
    dv17[31] = 0.0;
    dv17[32] = 0.0;
    dv17[33] = 0.0;
    dv17[34] = 0.0;
    dv17[35] = 0.0;
    dv17[36] = ((t136 - t2 * t3 * t7 * 0.06741) - t2 * t3 * t8 * 0.49544) - t2 *
        t6 * t8 * 0.06741;
    dv17[37] = ((t155 - t7 * t29 * 0.06741) - t8 * t29 * 0.49544) - t7 * t31 *
        0.49544;
    dv17[38] = ((t176 + t177) + t178) - t8 * t48 * 0.06741;
    memset(&dv17[39], 0, 9U * sizeof(double));
    dv17[48] = ((t137 - t16 * t19 * 0.01762) - t16 * t23 * 0.05219) - t20 * t23 *
        0.01762;
    dv17[49] = ((t156 - t16 * t41 * 0.05219) - t16 * t43 * 0.01762) - t20 * t41 *
        0.01762;
    dv17[50] = ((t179 + t180) + t181) - t20 * t60 * 0.05219;
    dv17[51] = t186;
    dv17[52] = t196;
    dv17[53] = t201;
    memset(&dv17[54], 0, 13U * sizeof(double));
    dv17[67] = ((((((((((((((t87 * 0.09 - t215) - t216) - t217) - t218) - t219)
                        - t220) - t221) - t222) - t223) + t68 * t87 * 0.12) +
                   t69 * t106 * 0.49544) + t67 * t113 * 0.04) + t67 * t115 *
                 0.408) + t74 * t113 * 0.408) + t77 * t119 * 0.01762;
    dv17[68] = ((((((((((((((t235 + t236) + t237) + t238) + t239) + t240) + t241)
                       + t242) + t243) - t70 * t90 * 0.06741) - t69 * t92 *
                    0.06741) - t70 * t92 * 0.49544) - t74 * t98 * 0.04) - t77 *
                 t104 * 0.05219) - t81 * t102 * 0.05219) - t81 * t104 * 0.01762;
    dv17[69] = 0.0;
    dv17[70] = ((-t266 - t267) - t268) + t81 * t121 * 0.7660444431189779;
    dv17[71] = t277;
    dv17[72] = 0.0;
    dv17[73] = 0.0;
    dv17[74] = 0.0;
    dv17[75] = 0.0;
    dv17[76] = 0.0;
    dv17[77] = 0.0;
    dv17[78] = ((((((((((((t64 * -0.0045 - t65 * t86 * 0.12) - t67 * t204 *
                          0.408) - t67 * t206 * 0.04) + t74 * t204 * 0.04) - t74
                       * t206 * 0.408) - t77 * t210 * 0.01762) + t77 * t212 *
                     0.05219) + t81 * t210 * 0.05219) + t81 * t212 * 0.01762) -
                  t65 * t69 * t86 * 0.49544) + t65 * t70 * t86 * 0.06741) - t68 *
                t69 * t86 * 0.06741) - t68 * t70 * t86 * 0.49544;
    dv17[79] = ((((((((((((t86 * t87 * -0.0045 + t67 * t226 * 0.408) - t74 *
                          t226 * 0.04) - t77 * t231 * 0.05219) - t81 * t231 *
                        0.01762) + t77 * t271 * 0.01762) + t67 * (t227 - t229) *
                      0.04) + t74 * (t227 - t229) * 0.408) - t81 * (t232 + t74 *
                     t228) * 0.05219) + t64 * t65 * t87 * 0.12) + t64 * t65 *
                  t69 * t87 * 0.49544) - t64 * t65 * t70 * t87 * 0.06741) + t64 *
                t68 * t69 * t87 * 0.06741) + t64 * t68 * t70 * t87 * 0.49544;
    dv17[80] = ((((((((((((t85 * t86 * -0.0045 + t67 * t246 * 0.408) - t74 *
                          t246 * 0.04) - t77 * t250 * 0.05219) + t77 * t169 *
                        0.01762) - t81 * t250 * 0.01762) - t81 * t169 * 0.05219)
                     + t67 * (t247 - t252) * 0.04) + t74 * (t247 - t252) * 0.408)
                   + t64 * t65 * t85 * 0.12) + t64 * t65 * t69 * t85 * 0.49544)
                 - t64 * t65 * t70 * t85 * 0.06741) + t64 * t68 * t69 * t85 *
                0.06741) + t64 * t68 * t70 * t85 * 0.49544;
    dv17[81] = ((t77 * t210 * 0.7660444431189779 + t77 * t212 *
                 0.64278760968653947) + t81 * t210 * 0.64278760968653947) - t81 *
        t212 * 0.7660444431189779;
    dv17[82] = ((t77 * t231 * -0.64278760968653947 - t77 * t271 *
                 0.7660444431189779) - t81 * t271 * 0.64278760968653947) + t81 *
        (t230 - t269) * 0.7660444431189779;
    dv17[83] = ((t77 * t250 * -0.64278760968653947 - t77 * t169 *
                 0.7660444431189779) - t81 * t169 * 0.64278760968653947) + t81 *
        (t249 - t278) * 0.7660444431189779;
    dv17[84] = 0.0;
    dv17[85] = 0.0;
    dv17[86] = 0.0;
    dv17[87] = 0.0;
    dv17[88] = 0.0;
    dv17[89] = 0.0;
    dv17[90] = (((((((((((t213 + t214) + t64 * t68 * 0.12) + t67 * t73 * 0.04) -
                       t67 * t76 * 0.408) + t73 * t74 * 0.408) - t77 * t80 *
                     0.01762) - t77 * t84 * 0.05219) - t81 * t84 * 0.01762) +
                  t74 * (t75 - t78) * 0.04) - t64 * t65 * t69 * 0.06741) - t64 *
                t65 * t70 * 0.49544) - t64 * t68 * t70 * 0.06741;
    dv17[91] = ((((((((((((t233 + t234) - t65 * t85 * 0.12) - t69 * t90 *
                         0.06741) - t70 * t90 * 0.49544) - t69 * t92 * 0.49544)
                      - t67 * t96 * 0.408) + t67 * t98 * 0.04) + t74 * t96 *
                    0.04) + t74 * t98 * 0.408) - t77 * t102 * 0.05219) - t77 *
                 t104 * 0.01762) - t81 * t102 * 0.01762) + t68 * t86 * t87 *
        0.12;
    dv17[92] = ((((((((((((t255 + t256) + t257) + t258) + t259) + t260) + t65 *
                      t87 * 0.12) - t70 * t109 * 0.06741) + t67 * t113 * 0.408)
                   - t67 * t115 * 0.04) - t74 * t113 * 0.04) - t74 * t115 *
                 0.408) - t81 * t121 * 0.05219) + t68 * t85 * t86 * 0.12;
    dv17[93] = t265;
    dv17[94] = t275;
    dv17[95] = t280;
    dv17[96] = 0.0;
    dv17[97] = 0.0;
    dv17[98] = 0.0;
    dv17[99] = 0.0;
    dv17[100] = 0.0;
    dv17[101] = 0.0;
    dv17[102] = ((t213 - t64 * t65 * t69 * 0.06741) - t64 * t65 * t70 * 0.49544)
        - t64 * t68 * t70 * 0.06741;
    dv17[103] = ((t233 - t69 * t90 * 0.06741) - t70 * t90 * 0.49544) - t69 * t92
        * 0.49544;
    dv17[104] = ((t255 + t256) + t257) - t70 * t109 * 0.06741;
    memset(&dv17[105], 0, 9U * sizeof(double));
    dv17[114] = ((t214 - t77 * t80 * 0.01762) - t77 * t84 * 0.05219) - t81 * t84
        * 0.01762;
    dv17[115] = ((t234 - t77 * t102 * 0.05219) - t77 * t104 * 0.01762) - t81 *
        t102 * 0.01762;
    dv17[116] = ((t258 + t259) + t260) - t81 * t121 * 0.05219;
    dv17[117] = t265;
    dv17[118] = t275;
    dv17[119] = t280;
    i101 = 0;
    for (i102 = 0; i102 < 10; i102++) {
        memcpy(&J[i101], &dv17[i101], 12U * sizeof(double));
        i101 += 12;
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
    int i97;
    double f_x;
    double nu;
    int j;
    int i98;
    int i99;
    double H_x[100];
    int i100;
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
    for (i97 = 0; i97 < 12; i97++) {
        y += r_x[i97] * r_x[i97];
    }

    f_x = 0.5 * y;
    y = 0.0;
    for (i = 0; i < 10; i++) {
        g_x[i] = 0.0;
        for (i97 = 0; i97 < 12; i97++) {
            g_x[i] += J_x[i97 + 12 * i] * r_x[i97];
        }

        nu = fabs(g_x[i]);
        if (nu > y) {
            y = nu;
        }
    }

    if (y <= obj->gTolerance) {
        obj->status = G_TOLERANCE;
    } else {
        i97 = 0;
        for (i = 0; i < 10; i++) {
            j = 0;
            i98 = 0;
            for (i99 = 0; i99 < 10; i99++) {
                H_x[j + i] = 0.0;
                for (i100 = 0; i100 < 12; i100++) {
                    H_x[j + i] += J_x[i100 + i97] * J_x[i100 + i98];
                }

                j += 10;
                i98 += 12;
            }

            i97 += 12;
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

                for (i97 = 0; i97 < 100; i97++) {
                    b_H_x[i97] = H_x[i97] + lambda * (double)b[i97];
                }

                d_mldivide(b_H_x, b_lb);
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
                    for (i97 = 0; i97 < 12; i97++) {
                        y += r_x[i97] * r_x[i97];
                    }

                    f_new = 0.5 * y;
                    y = 0.0;
                    for (i97 = 0; i97 < 10; i97++) {
                        y += 0.5 * b_lb[i97] * (lambda * b_lb[i97] - g_x[i97]);
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
                            for (i97 = 0; i97 < 12; i97++) {
                                g_x[i] += J_x[i97 + 12 * i] * r_x[i97];
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
                            i97 = 0;
                            for (i = 0; i < 10; i++) {
                                j = 0;
                                i98 = 0;
                                for (i99 = 0; i99 < 10; i99++) {
                                    H_x[j + i] = 0.0;
                                    for (i100 = 0; i100 < 12; i100++) {
                                        H_x[j + i] += J_x[i100 + i97] * J_x[i100
                                            + i98];
                                    }

                                    j += 10;
                                    i98 += 12;
                                }

                                i97 += 12;
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

static void c_PDController_setProportionalG(PDController *obj, const double
    b_proportionalGain[10])
{
    int i;
    for (i = 0; i < 10; i++) {
        obj->proportionalGain[i] = b_proportionalGain[i];
    }
}

static void c_StandingController_setPelvisP(StandingController *obj, double
    pelvisPitch)
{
    SmoothRateLimiter *b_obj;
    double legLength;
    b_obj = &obj->legLengthFilter;
    legLength = b_obj->signalValue;
    SmoothRateLimiter_updateSignal(&obj->pelvisPitchFilter, fmax(fmin
        (pelvisPitch, 0.26179938779914941 * fmax(fmin((legLength - 0.45) / 0.6,
        1.0), 0.0)), -0.26179938779914941));
}

static void c_StandingController_updateDesi(StandingController *obj, double
    pelvisPitch)
{
    SmoothRateLimiter *b_obj;
    double legLength;
    double pelvisY;
    double pelvisYaw;
    double value;
    int i86;
    double obj_data[3];
    double b_obj_data[9];
    static const signed char iv1[9] = { 1, 0, 0, 0, 1, 0, 0, 0, 1 };

    double c[9];
    int i;
    static const signed char iv2[3] = { 0, 0, 1 };

    int b_i;
    double c_obj_data[9];
    int i87;
    int i88;
    static const signed char iv3[3] = { 0, 1, 0 };

    Frame_5 *c_obj;
    double c_desiredPelvisTransform_positi[3];
    b_obj = &obj->legLengthFilter;
    legLength = b_obj->signalValue;
    b_obj = &obj->pelvisYFilter;
    pelvisY = b_obj->signalValue;
    b_obj = &obj->pelvisYawFilter;
    pelvisYaw = b_obj->signalValue;
    b_obj = &obj->pelvisPitchFilter;
    value = b_obj->signalValue;
    pelvisPitch = fmax(fmin(pelvisPitch + value, 0.26179938779914941),
                       -0.26179938779914941);
    for (i86 = 0; i86 < 9; i86++) {
        b_obj_data[i86] = iv1[i86];
    }

    obj_data[0] = 0.0;
    obj_data[1] = pelvisY;
    obj_data[2] = legLength;
    pelvisY = cos(pelvisYaw);
    legLength = sin(pelvisYaw);
    c[0] = pelvisY;
    c[3] = -legLength;
    c[6] = 0.0;
    c[1] = legLength;
    c[4] = pelvisY;
    c[7] = 0.0;
    i86 = 0;
    for (i = 0; i < 3; i++) {
        c[i86 + 2] = iv2[i];
        i86 += 3;
    }

    for (i86 = 0; i86 < 3; i86++) {
        i = 0;
        for (b_i = 0; b_i < 3; b_i++) {
            c_obj_data[i + i86] = 0.0;
            i87 = 0;
            for (i88 = 0; i88 < 3; i88++) {
                c_obj_data[i + i86] += b_obj_data[i87 + i86] * c[i88 + i];
                i87 += 3;
            }

            i += 3;
        }
    }

    pelvisY = cos(pelvisPitch);
    legLength = sin(pelvisPitch);
    c[0] = pelvisY;
    c[3] = 0.0;
    c[6] = legLength;
    i86 = 0;
    for (i = 0; i < 3; i++) {
        for (b_i = 0; b_i < 3; b_i++) {
            b_obj_data[b_i + i86] = c_obj_data[b_i + i86];
        }

        c[i86 + 1] = iv3[i];
        i86 += 3;
    }

    c[2] = -legLength;
    c[5] = 0.0;
    c[8] = pelvisY;
    for (i86 = 0; i86 < 3; i86++) {
        i = 0;
        for (b_i = 0; b_i < 3; b_i++) {
            c_obj_data[i + i86] = 0.0;
            i87 = 0;
            for (i88 = 0; i88 < 3; i88++) {
                c_obj_data[i + i86] += b_obj_data[i87 + i86] * c[i88 + i];
                i87 += 3;
            }

            i += 3;
        }
    }

    i = 0;
    for (b_i = 0; b_i < 3; b_i++) {
        for (i86 = 0; i86 < 3; i86++) {
            b_obj_data[i86 + i] = c_obj_data[i86 + i];
        }

        c_desiredPelvisTransform_positi[b_i] = obj_data[b_i];
        i += 3;
    }

    c_obj = &obj->desiredPelvisFrame;
    for (i = 0; i < 3; i++) {
        c_obj->relativeTransform.position.data[i] =
            c_desiredPelvisTransform_positi[i];
    }

    for (i86 = 0; i86 < 9; i86++) {
        c_obj->relativeTransform.rotation.data[i86] = b_obj_data[i86];
    }
}

static Frame_7 *c_SteppingController_getDesired(SteppingController *obj)
{
    Frame_7 *desiredStanceFootFrame;
    if (obj->stanceLeg == LEFT) {
        desiredStanceFootFrame = &obj->desiredLeftFootFrame;
    } else {
        desiredStanceFootFrame = &obj->desiredRightFootFrame;
    }

    return desiredStanceFootFrame;
}

static Frame_4 *c_SteppingController_getStanceF(const SteppingController *obj)
{
    Frame_4 *stanceFootFrame;
    if (obj->stanceLeg == LEFT) {
        stanceFootFrame = &obj->robot->leftFootFrame;
    } else {
        stanceFootFrame = &obj->robot->rightFootFrame;
    }

    return stanceFootFrame;
}

static void c_SteppingController_updateDesi(SteppingController *obj)
{
    SmoothRateLimiter *b_obj;
    double legLength;
    double obj_data[3];
    int i;
    double c_desiredPelvisTransform_positi[3];
    signed char c_desiredPelvisTransform_rotati[9];
    int k;
    Frame_5 *c_obj;
    double relativeTransform_position_data[3];
    double relativeTransform_rotation_data[9];
    int i115;
    double d_desiredPelvisTransform_rotati[9];
    double W[9];
    int i116;
    int i117;
    b_obj = &obj->legLengthFilter;
    legLength = b_obj->signalValue;
    obj_data[0] = 0.0;
    obj_data[1] = 0.0;
    obj_data[2] = legLength;
    for (i = 0; i < 3; i++) {
        c_desiredPelvisTransform_positi[i] = obj_data[i];
    }

    for (i = 0; i < 9; i++) {
        c_desiredPelvisTransform_rotati[i] = 0;
    }

    i = 0;
    for (k = 0; k < 3; k++) {
        c_desiredPelvisTransform_rotati[i] = 1;
        i += 4;
    }

    c_obj = &obj->desiredPelvisFrame;
    for (i = 0; i < 3; i++) {
        relativeTransform_position_data[i] =
            c_obj->relativeTransform.position.data[i];
    }

    c_obj = &obj->desiredPelvisFrame;
    for (i = 0; i < 9; i++) {
        relativeTransform_rotation_data[i] =
            c_obj->relativeTransform.rotation.data[i];
    }

    i = 0;
    for (k = 0; k < 3; k++) {
        for (i115 = 0; i115 < 3; i115++) {
            d_desiredPelvisTransform_rotati[i115 + i] = ((double)
                c_desiredPelvisTransform_rotati[i115 + i] -
                relativeTransform_rotation_data[i115 + i]) / 0.0005;
        }

        i += 3;
    }

    for (i = 0; i < 3; i++) {
        k = 0;
        for (i115 = 0; i115 < 3; i115++) {
            W[k + i] = 0.0;
            i116 = 0;
            for (i117 = 0; i117 < 3; i117++) {
                W[k + i] += d_desiredPelvisTransform_rotati[i116 + i] * (double)
                    c_desiredPelvisTransform_rotati[i116 + i115];
                i116 += 3;
            }

            k += 3;
        }
    }

    c_obj = &obj->desiredPelvisFrame;
    for (i = 0; i < 3; i++) {
        c_obj->relativeTransform.position.data[i] =
            c_desiredPelvisTransform_positi[i];
    }

    for (i = 0; i < 9; i++) {
        c_obj->relativeTransform.rotation.data[i] =
            c_desiredPelvisTransform_rotati[i];
    }

    c_obj = &obj->desiredPelvisFrame;
    for (i = 0; i < 3; i++) {
        c_obj->relativeSpatialVelocity[i] = (obj_data[i] -
            relativeTransform_position_data[i]) / 0.0005;
    }

    c_obj = &obj->desiredPelvisFrame;
    c_obj->relativeSpatialVelocity[3] = W[1];
    c_obj->relativeSpatialVelocity[4] = -W[2];
    c_obj->relativeSpatialVelocity[5] = W[5];
}

static double c_clamp(double a)
{
    return fmax(fmin(a, 0.4), -0.4);
}

static void c_mldivide(const double A[16], double B[4])
{
    double b_A[16];
    int i125;
    int j;
    signed char ipiv[4];
    int k;
    int c;
    int kAcol;
    int ix;
    double smax;
    int jy;
    double s;
    int ijA;
    memcpy(&b_A[0], &A[0], sizeof(double) << 4);
    for (i125 = 0; i125 < 4; i125++) {
        ipiv[i125] = (signed char)(1 + i125);
    }

    for (j = 0; j < 3; j++) {
        c = j * 5;
        kAcol = 0;
        ix = c;
        smax = fabs(b_A[c]);
        for (k = 2; k <= 4 - j; k++) {
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
                for (k = 0; k < 4; k++) {
                    smax = b_A[ix];
                    b_A[ix] = b_A[kAcol];
                    b_A[kAcol] = smax;
                    ix += 4;
                    kAcol += 4;
                }
            }

            i125 = (c - j) + 4;
            for (jy = c + 1; jy + 1 <= i125; jy++) {
                b_A[jy] /= b_A[c];
            }
        }

        kAcol = c;
        jy = c + 4;
        for (k = 1; k <= 3 - j; k++) {
            smax = b_A[jy];
            if (b_A[jy] != 0.0) {
                ix = c + 1;
                i125 = (kAcol - j) + 8;
                for (ijA = 5 + kAcol; ijA + 1 <= i125; ijA++) {
                    b_A[ijA] += b_A[ix] * -smax;
                    ix++;
                }
            }

            jy += 4;
            kAcol += 4;
        }

        if (ipiv[j] != j + 1) {
            smax = B[j];
            B[j] = B[ipiv[j] - 1];
            B[ipiv[j] - 1] = smax;
        }
    }

    for (k = 0; k < 4; k++) {
        kAcol = k << 2;
        if (B[k] != 0.0) {
            for (jy = k + 1; jy + 1 < 5; jy++) {
                B[jy] -= B[k] * b_A[jy + kAcol];
            }
        }
    }

    for (k = 3; k >= 0; k += -1) {
        kAcol = k << 2;
        if (B[k] != 0.0) {
            B[k] /= b_A[k + kAcol];
            for (jy = 0; jy + 1 <= k; jy++) {
                B[jy] -= B[k] * b_A[jy + kAcol];
            }
        }
    }
}

static double c_xnrm2(const double b_x[30], int ix0)
{
    double y;
    double scale;
    int k;
    double absxk;
    double t;
    y = 0.0;
    scale = 2.2250738585072014E-308;
    for (k = ix0; k <= ix0 + 5; k++) {
        absxk = fabs(b_x[k - 1]);
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

void cassie_system_init(const cassie_system_t *b_system)
{
    CassieOutputs co;
    ProcessInputsSimulation *obj;
    int i;
    RadioSpoofer *b_obj;
    CassieController *c_obj;
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
    b_obj = b_system->radio_spoofer;
    b_obj->isInitialized = 1;
    CassieOutputs_CassieOutputs(&b_obj->outputs);
    b_obj->timer = 0.0;
    c_obj = b_system->controller;
    c_obj->isInitialized = 1;
    CassieController_setupImpl(c_obj);
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
    RadioSpoofer *obj;
    int i;
    bool value;
    double A;
    double c_spoofed_outputs_vectorNavOrie[4];
    double c_spoofed_outputs_vectorNavAngu[3];
    double c_spoofed_outputs_vectorNavTemp;
    double c_spoofed_outputs_vectorNavLine[3];
    double c_spoofed_outputs_vectorNavMagn[3];
    double spoofed_outputs_motorPosition[10];
    double spoofed_outputs_motorVelocity[10];
    double spoofed_outputs_jointPosition[6];
    double spoofed_outputs_jointVelocity[6];
    double spoofed_outputs_stateOfCharge;
    double spoofed_outputs_radio[16];
    double spoofed_outputs_status;
    CassieController *b_obj;
    double inputs_radio[9];
    double inputs_torque[10];
    ProcessInputsSimulation *c_obj;
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
    obj = b_system->radio_spoofer;
    if (obj->isInitialized != 1) {
        obj->isInitialized = 1;
        CassieOutputs_CassieOutputs(&obj->outputs);
        obj->timer = 0.0;
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
    obj->timer += 0.0005;
    obj->outputs.data.radio[8] = 1.0;
    obj->outputs.data.radio[6] = 1.0;
    value = (obj->timer > 2.0);
    obj->outputs.data.radio[9] = value;
    A = obj->timer - 5.0;
    obj->outputs.data.radio[0] = fmax(fmin(A / 2.0, 1.0), 0.0);
    for (i = 0; i < 4; i++) {
        c_spoofed_outputs_vectorNavOrie[i] =
            obj->outputs.data.vectorNavOrientation[i];
    }

    for (i = 0; i < 3; i++) {
        c_spoofed_outputs_vectorNavAngu[i] =
            obj->outputs.data.vectorNavAngularVelocity[i];
        c_spoofed_outputs_vectorNavLine[i] =
            obj->outputs.data.vectorNavLinearAcceleration[i];
        c_spoofed_outputs_vectorNavMagn[i] =
            obj->outputs.data.vectorNavMagnetometer[i];
    }

    A = obj->outputs.data.vectorNavPressure;
    c_spoofed_outputs_vectorNavTemp = obj->outputs.data.vectorNavTemperature;
    for (i = 0; i < 10; i++) {
        spoofed_outputs_motorPosition[i] = obj->outputs.data.motorPosition[i];
        spoofed_outputs_motorVelocity[i] = obj->outputs.data.motorVelocity[i];
    }

    for (i = 0; i < 6; i++) {
        spoofed_outputs_jointPosition[i] = obj->outputs.data.jointPosition[i];
        spoofed_outputs_jointVelocity[i] = obj->outputs.data.jointVelocity[i];
    }

    for (i = 0; i < 16; i++) {
        spoofed_outputs_radio[i] = obj->outputs.data.radio[i];
    }

    spoofed_outputs_stateOfCharge = obj->outputs.data.stateOfCharge;
    spoofed_outputs_status = obj->outputs.data.status;
    b_obj = b_system->controller;
    if (b_obj->isInitialized != 1) {
        b_obj->isInitialized = 1;
        CassieController_setupImpl(b_obj);
    }

    CassieController_stepImpl(b_obj, c_spoofed_outputs_vectorNavOrie,
        c_spoofed_outputs_vectorNavAngu, c_spoofed_outputs_vectorNavLine,
        c_spoofed_outputs_vectorNavMagn, A, c_spoofed_outputs_vectorNavTemp,
        spoofed_outputs_motorPosition, spoofed_outputs_motorVelocity,
        spoofed_outputs_jointPosition, spoofed_outputs_jointVelocity,
        spoofed_outputs_radio, spoofed_outputs_stateOfCharge,
        spoofed_outputs_status, inputs_radio, inputs_torque);
    c_obj = b_system->process_inputs;
    if (c_obj->isInitialized != 1) {
        c_obj->isInitialized = 1;
        CassieEtherCAT_CassieEtherCAT(&c_obj->etherCAT);
        for (i = 0; i < 9; i++) {
            c_obj->inputs.data.radio[i] = 0.0;
        }

        for (i = 0; i < 10; i++) {
            c_obj->inputs.data.torque[i] = 0.0;
        }

        CassieOutputs_CassieOutputs(&c_obj->outputs);
        c_obj->softStart = 0.0;
        c_obj->softStartDuration = 5.0;
        c_obj->softStart = 1.0;
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

    ProcessInputs_stepImpl(c_obj, ethercat, inputs_radio, inputs_torque,
                           &expl_temp);
    *ethercat = c_obj->etherCAT.data;
}

static double clamp(double a, double limit_1, double limit_2)
{
    return fmax(fmin(a, fmax(limit_1, limit_2)), fmin(limit_1, limit_2));
}

static RobotSide convert_to_enum_RobotSide(signed char u)
{
    RobotSide y;
    switch (u) {
      case 1:
        y = LEFT;
        break;

      case -1:
        y = RIGHT;
        break;

      default:
        y = LEFT;
        break;
    }

    return y;
}

static void cubicInterpolation(double t, const double ts[2], const double qs[2],
    const double dqs[2], double *q, double *dq, double *ddqt)
{
    double b_x[3];
    int unusedU4;
    int j2;
    double c[4];
    double dv24[16];
    b_x[0] = ts[0];
    b_x[1] = t;
    b_x[2] = ts[1];
    sort3(1, ts[0], 2, t, 3, ts[1], &unusedU4, &j2);
    *q = qs[0];
    *dq = 0.0;
    *ddqt = 0.0;
    if ((b_x[j2 - 1] >= ts[0]) && (b_x[j2 - 1] <= ts[1])) {
        c[0] = qs[0];
        c[1] = dqs[0];
        c[2] = qs[1];
        c[3] = dqs[1];
        dv24[0] = 1.0;
        dv24[4] = ts[0];
        dv24[8] = ts[0] * ts[0];
        dv24[12] = pow(ts[0], 3.0);
        dv24[1] = 0.0;
        dv24[5] = 1.0;
        dv24[9] = 2.0 * ts[0];
        dv24[13] = 3.0 * (ts[0] * ts[0]);
        dv24[2] = 1.0;
        dv24[6] = ts[1];
        dv24[10] = ts[1] * ts[1];
        dv24[14] = pow(ts[1], 3.0);
        dv24[3] = 0.0;
        dv24[7] = 1.0;
        dv24[11] = 2.0 * ts[1];
        dv24[15] = 3.0 * (ts[1] * ts[1]);
        c_mldivide(dv24, c);
        *q = ((c[0] + c[1] * b_x[j2 - 1]) + c[2] * (b_x[j2 - 1] * b_x[j2 - 1]))
            + c[3] * pow(b_x[j2 - 1], 3.0);
        *dq = (c[1] + 2.0 * c[2] * b_x[j2 - 1]) + 3.0 * c[3] * (b_x[j2 - 1] *
            b_x[j2 - 1]);
        *ddqt = 2.0 * c[2] + 6.0 * c[3] * b_x[j2 - 1];
    }
}

static void d_Frame_Frame(Frame_7 **obj, WorldFrame *b_parentFrame)
{
    Frame_7 *b_this;
    int i;
    static const char b_name[18] = { 'D', 'e', 's', 'i', 'r', 'e', 'd', ' ', 'R',
        'i', 'g', 'h', 't', ' ', 'F', 'o', 'o', 't' };

    WorldFrame *c_parentFrame;
    signed char obj_rotation_data[9];
    int k;
    b_this = *obj;
    *obj = b_this;
    b_this = *obj;
    for (i = 0; i < 18; i++) {
        b_this->name[i] = b_name[i];
    }

    b_this = *obj;
    c_parentFrame = b_parentFrame;
    b_this->parentFrame = c_parentFrame;
    for (i = 0; i < 9; i++) {
        obj_rotation_data[i] = 0;
    }

    i = 0;
    for (k = 0; k < 3; k++) {
        obj_rotation_data[i] = 1;
        i += 4;
    }

    for (i = 0; i < 3; i++) {
        (*obj)->relativeTransform.position.data[i] = 0.0;
    }

    for (i = 0; i < 9; i++) {
        (*obj)->relativeTransform.rotation.data[i] = obj_rotation_data[i];
    }

    for (i = 0; i < 6; i++) {
        (*obj)->relativeSpatialVelocity[i] = 0.0;
    }
}

static void d_Frame_getSpatialVelocity(Frame_2 *obj, ZUpFrame *relativeTo,
    Frame_2 *inCoordinatesOf, double spatialVelocity[6])
{
    ZUpFrame *b_obj;
    Frame_2 *withRespectTo;
    Frame_2 *b_inCoordinatesOf;
    Frame_2 *b;
    int i118;
    bool tf;
    char a[6];
    int i;
    char b_b[6];
    int exitg1;
    Transform3d r21;
    Transform3d unusedExpr;
    Transform3d r22;
    Rotation3d obj_rotation;
    Transform3d r23;
    double obj_position_data[3];
    Rotation3d worldZUpRotation;
    double b_obj_position_data[3];
    double obj_rotation_data[9];
    int b_i;
    double transform_position_data[3];
    double transform_rotation_data[9];
    double c_data[9];
    int i119;
    int i120;
    Transform3d b_unusedExpr;
    double c_b[6];
    Transform3d r24;
    double dv26[6];
    Transform3d c_unusedExpr;
    Transform3d r25;
    double d13;
    double dv27[9];
    int i121;
    int i122;
    Frame_2 *b_a;
    double b_transform_rotation_data[36];
    Transform3d r26;
    Transform3d r27;
    Transform3d r28;
    Transform3d r29;
    Transform3d d_unusedExpr;
    Transform3d r30;
    Transform3d e_unusedExpr;
    Transform3d r31;
    b_obj = relativeTo;
    withRespectTo = obj;
    b_inCoordinatesOf = withRespectTo;
    b = b_obj->parentFrame;
    for (i118 = 0; i118 < 6; i118++) {
        a[i118] = withRespectTo->name[i118];
        b_b[i118] = b->name[i118];
    }

    tf = false;
    i = 0;
    do {
        exitg1 = 0;
        if (i + 1 < 7) {
            if (a[i] != b_b[i]) {
                exitg1 = 1;
            } else {
                i++;
            }
        } else {
            tf = true;
            exitg1 = 1;
        }
    } while (exitg1 == 0);

    if (tf) {
        b_Frame_getWorldTransform(b_obj->parentFrame, &unusedExpr);
        b_Frame_getWorldTransform(b_obj->parentFrame, &r22);
        obj_rotation = r22.rotation;
        worldZUpRotation = obj_rotation;
        Rotation3d_removePitchAndRoll(&worldZUpRotation);
        i = 0;
        for (b_i = 0; b_i < 3; b_i++) {
            i118 = 0;
            for (i119 = 0; i119 < 3; i119++) {
                c_data[i118 + b_i] = 0.0;
                for (i120 = 0; i120 < 3; i120++) {
                    c_data[i118 + b_i] += obj_rotation.data[i120 + i] *
                        worldZUpRotation.data[i120 + i118];
                }

                i118 += 3;
            }

            transform_position_data[b_i] = 0.0;
            i += 3;
        }

        memcpy(&transform_rotation_data[0], &c_data[0], 9U * sizeof(double));
    } else {
        b_Frame_getWorldTransform(withRespectTo, &r21);
        for (i = 0; i < 3; i++) {
            obj_position_data[i] = r21.position.data[i];
        }

        b_Frame_getWorldTransform(withRespectTo, &r23);
        obj_rotation = r23.rotation;
        Frame_getWorldTransform(b_obj, b_obj_position_data, &worldZUpRotation);
        memcpy(&obj_rotation_data[0], &worldZUpRotation.data[0], 9U * sizeof
               (double));
        Transform3d_mldivide(obj_position_data, obj_rotation.data,
                             b_obj_position_data, obj_rotation_data,
                             transform_position_data, transform_rotation_data);
    }

    for (i118 = 0; i118 < 6; i118++) {
        a[i118] = withRespectTo->name[i118];
        b_b[i118] = b_inCoordinatesOf->name[i118];
    }

    tf = false;
    i = 0;
    do {
        exitg1 = 0;
        if (i + 1 < 7) {
            if (a[i] != b_b[i]) {
                exitg1 = 1;
            } else {
                i++;
            }
        } else {
            tf = true;
            exitg1 = 1;
        }
    } while (exitg1 == 0);

    if (!tf) {
        b_Frame_getWorldTransform(b_inCoordinatesOf, &b_unusedExpr);
        b_Frame_getWorldTransform(b_inCoordinatesOf, &r24);
        obj_rotation = r24.rotation;
        b_Frame_getWorldTransform(withRespectTo, &c_unusedExpr);
        b_Frame_getWorldTransform(withRespectTo, &r25);
        memcpy(&obj_rotation_data[0], &r25.rotation.data[0], 9U * sizeof(double));
        for (i118 = 0; i118 < 3; i118++) {
            d13 = 0.0;
            for (i119 = 0; i119 < 3; i119++) {
                c_data[i118 + 3 * i119] = 0.0;
                for (i120 = 0; i120 < 3; i120++) {
                    c_data[i118 + 3 * i119] += obj_rotation.data[i120 + 3 * i118]
                        * obj_rotation_data[i120 + 3 * i119];
                }

                d13 += c_data[i118 + 3 * i119] * transform_position_data[i119];
            }

            obj_position_data[i118] = d13;
            for (i119 = 0; i119 < 3; i119++) {
                worldZUpRotation.data[i118 + 3 * i119] = 0.0;
                for (i120 = 0; i120 < 3; i120++) {
                    worldZUpRotation.data[i118 + 3 * i119] += c_data[i118 + 3 *
                        i120] * transform_rotation_data[i120 + 3 * i119];
                }
            }
        }

        for (i = 0; i < 3; i++) {
            transform_position_data[i] = obj_position_data[i];
        }

        memcpy(&transform_rotation_data[0], &worldZUpRotation.data[0], 9U *
               sizeof(double));
    }

    b_Frame_getWorldSpatialVelocity(relativeTo, c_b);
    Frame_getWorldSpatialVelocity(obj, dv26);
    obj_rotation_data[0] = 0.0;
    obj_rotation_data[3] = -transform_position_data[2];
    obj_rotation_data[6] = transform_position_data[1];
    obj_rotation_data[1] = transform_position_data[2];
    obj_rotation_data[4] = 0.0;
    obj_rotation_data[7] = -transform_position_data[0];
    obj_rotation_data[2] = -transform_position_data[1];
    obj_rotation_data[5] = transform_position_data[0];
    obj_rotation_data[8] = 0.0;
    i118 = 0;
    i119 = 0;
    for (i120 = 0; i120 < 3; i120++) {
        i = 0;
        for (b_i = 0; b_i < 3; b_i++) {
            dv27[i + i120] = 0.0;
            i121 = 0;
            for (i122 = 0; i122 < 3; i122++) {
                dv27[i + i120] += obj_rotation_data[i121 + i120] *
                    transform_rotation_data[i122 + i];
                i121 += 3;
            }

            b_transform_rotation_data[b_i + i118] = transform_rotation_data[b_i
                + i119];
            i += 3;
        }

        i118 += 6;
        i119 += 3;
    }

    i118 = 0;
    i119 = 0;
    for (i120 = 0; i120 < 3; i120++) {
        for (i = 0; i < 3; i++) {
            b_transform_rotation_data[(i + i118) + 18] = dv27[i + i119];
            b_transform_rotation_data[(i + i118) + 3] = 0.0;
            b_transform_rotation_data[(i + i118) + 21] =
                transform_rotation_data[i + i119];
        }

        i118 += 6;
        i119 += 3;
    }

    for (i118 = 0; i118 < 6; i118++) {
        d13 = 0.0;
        i119 = 0;
        for (i120 = 0; i120 < 6; i120++) {
            d13 += b_transform_rotation_data[i119 + i118] * c_b[i120];
            i119 += 6;
        }

        spatialVelocity[i118] = dv26[i118] - d13;
    }

    b_a = obj;
    b = inCoordinatesOf;
    for (i118 = 0; i118 < 6; i118++) {
        a[i118] = b_a->name[i118];
        b_b[i118] = b->name[i118];
    }

    tf = false;
    i = 0;
    do {
        exitg1 = 0;
        if (i + 1 < 7) {
            if (a[i] != b_b[i]) {
                exitg1 = 1;
            } else {
                i++;
            }
        } else {
            tf = true;
            exitg1 = 1;
        }
    } while (exitg1 == 0);

    if (!tf) {
        b_a = obj;
        withRespectTo = inCoordinatesOf;
        b_inCoordinatesOf = withRespectTo;
        for (i118 = 0; i118 < 6; i118++) {
            a[i118] = withRespectTo->name[i118];
            b_b[i118] = b_a->name[i118];
        }

        tf = false;
        i = 0;
        do {
            exitg1 = 0;
            if (i + 1 < 7) {
                if (a[i] != b_b[i]) {
                    exitg1 = 1;
                } else {
                    i++;
                }
            } else {
                tf = true;
                exitg1 = 1;
            }
        } while (exitg1 == 0);

        if (tf) {
            memset(&transform_rotation_data[0], 0, 9U * sizeof(double));
            i = 0;
            for (b_i = 0; b_i < 3; b_i++) {
                transform_rotation_data[i] = 1.0;
                i += 4;
            }
        } else {
            b_Frame_getWorldTransform(withRespectTo, &r26);
            for (i = 0; i < 3; i++) {
                obj_position_data[i] = r26.position.data[i];
            }

            b_Frame_getWorldTransform(withRespectTo, &r27);
            obj_rotation = r27.rotation;
            b_Frame_getWorldTransform(b_a, &r28);
            for (i = 0; i < 3; i++) {
                b_obj_position_data[i] = r28.position.data[i];
            }

            b_Frame_getWorldTransform(b_a, &r29);
            memcpy(&obj_rotation_data[0], &r29.rotation.data[0], 9U * sizeof
                   (double));
            Transform3d_mldivide(obj_position_data, obj_rotation.data,
                                 b_obj_position_data, obj_rotation_data,
                                 transform_position_data,
                                 transform_rotation_data);
        }

        for (i118 = 0; i118 < 6; i118++) {
            a[i118] = withRespectTo->name[i118];
            b_b[i118] = b_inCoordinatesOf->name[i118];
        }

        tf = false;
        i = 0;
        do {
            exitg1 = 0;
            if (i + 1 < 7) {
                if (a[i] != b_b[i]) {
                    exitg1 = 1;
                } else {
                    i++;
                }
            } else {
                tf = true;
                exitg1 = 1;
            }
        } while (exitg1 == 0);

        if (!tf) {
            b_Frame_getWorldTransform(b_inCoordinatesOf, &d_unusedExpr);
            b_Frame_getWorldTransform(b_inCoordinatesOf, &r30);
            obj_rotation = r30.rotation;
            b_Frame_getWorldTransform(withRespectTo, &e_unusedExpr);
            b_Frame_getWorldTransform(withRespectTo, &r31);
            memcpy(&obj_rotation_data[0], &r31.rotation.data[0], 9U * sizeof
                   (double));
            for (i118 = 0; i118 < 3; i118++) {
                for (i119 = 0; i119 < 3; i119++) {
                    c_data[i118 + 3 * i119] = 0.0;
                    for (i120 = 0; i120 < 3; i120++) {
                        c_data[i118 + 3 * i119] += obj_rotation.data[i120 + 3 *
                            i118] * obj_rotation_data[i120 + 3 * i119];
                    }
                }

                for (i119 = 0; i119 < 3; i119++) {
                    worldZUpRotation.data[i118 + 3 * i119] = 0.0;
                    for (i120 = 0; i120 < 3; i120++) {
                        worldZUpRotation.data[i118 + 3 * i119] += c_data[i118 +
                            3 * i120] * transform_rotation_data[i120 + 3 * i119];
                    }
                }
            }

            memcpy(&transform_rotation_data[0], &worldZUpRotation.data[0], 9U *
                   sizeof(double));
        }

        for (i118 = 0; i118 < 3; i118++) {
            obj_position_data[i118] = 0.0;
            b_obj_position_data[i118] = 0.0;
            i119 = 0;
            for (i120 = 0; i120 < 3; i120++) {
                obj_position_data[i118] += transform_rotation_data[i119 + i118] *
                    spatialVelocity[i120];
                b_obj_position_data[i118] += transform_rotation_data[i119 + i118]
                    * spatialVelocity[i120 + 3];
                i119 += 3;
            }
        }

        for (i118 = 0; i118 < 3; i118++) {
            spatialVelocity[i118] = obj_position_data[i118];
            spatialVelocity[i118 + 3] = b_obj_position_data[i118];
        }
    }
}

static void d_Frame_getTransform(ZUpFrame_1 *obj, ZUpFrame *withRespectTo,
    double transform_position_data[3], double transform_rotation_data[9])
{
    ZUpFrame *inCoordinatesOf;
    double obj_position_data[3];
    Rotation3d obj_rotation;
    double obj_rotation_data[9];
    double b_obj_position_data[3];
    double b_obj_rotation_data[9];
    ZUpFrame *a;
    int i;
    bool b_bool;
    char b_a[11];
    char b[11];
    int exitg1;
    double d5;
    int i57;
    double R_data[9];
    double c_data[9];
    int i58;
    inCoordinatesOf = withRespectTo;
    Frame_getWorldTransform(withRespectTo, obj_position_data, &obj_rotation);
    memcpy(&obj_rotation_data[0], &obj_rotation.data[0], 9U * sizeof(double));
    d_Frame_getWorldTransform(obj, b_obj_position_data, b_obj_rotation_data);
    Transform3d_mldivide(obj_position_data, obj_rotation_data,
                         b_obj_position_data, b_obj_rotation_data,
                         transform_position_data, transform_rotation_data);
    a = withRespectTo;
    for (i = 0; i < 11; i++) {
        b_a[i] = a->name[i];
        b[i] = inCoordinatesOf->name[i];
    }

    b_bool = false;
    i = 0;
    do {
        exitg1 = 0;
        if (i + 1 < 12) {
            if (b_a[i] != b[i]) {
                exitg1 = 1;
            } else {
                i++;
            }
        } else {
            b_bool = true;
            exitg1 = 1;
        }
    } while (exitg1 == 0);

    if (!b_bool) {
        Frame_getWorldTransform(inCoordinatesOf, obj_position_data,
                                &obj_rotation);
        memcpy(&obj_rotation_data[0], &obj_rotation.data[0], 9U * sizeof(double));
        Frame_getWorldTransform(withRespectTo, b_obj_position_data,
                                &obj_rotation);
        memcpy(&b_obj_rotation_data[0], &obj_rotation.data[0], 9U * sizeof
               (double));
        for (i = 0; i < 3; i++) {
            d5 = 0.0;
            for (i57 = 0; i57 < 3; i57++) {
                c_data[i + 3 * i57] = 0.0;
                for (i58 = 0; i58 < 3; i58++) {
                    c_data[i + 3 * i57] += obj_rotation_data[i58 + 3 * i] *
                        b_obj_rotation_data[i58 + 3 * i57];
                }

                d5 += c_data[i + 3 * i57] * transform_position_data[i57];
            }

            obj_position_data[i] = d5;
            for (i57 = 0; i57 < 3; i57++) {
                R_data[i + 3 * i57] = 0.0;
                for (i58 = 0; i58 < 3; i58++) {
                    R_data[i + 3 * i57] += c_data[i + 3 * i58] *
                        transform_rotation_data[i58 + 3 * i57];
                }
            }
        }

        for (i = 0; i < 3; i++) {
            transform_position_data[i] = obj_position_data[i];
        }

        memcpy(&transform_rotation_data[0], &R_data[0], 9U * sizeof(double));
    }
}

static void d_Frame_getWorldTransform(ZUpFrame_1 *obj, double
    worldTransform_position_data[3], double worldTransform_rotation_data[9])
{
    ZUpFrame_1 *b_obj;
    Frame_8 *b_parentFrame;
    WorldFrame *c_parentFrame;
    int i;
    int i41;
    double relativeTransform_position_data[3];
    double relativeTransform_rotation_data[9];
    double d3;
    int i42;
    int i43;
    double R_data[9];
    double p_data[3];
    double b_R_data[9];
    int i44;
    b_obj = obj;
    b_parentFrame = b_obj->parentFrame;
    c_parentFrame = b_parentFrame->parentFrame;
    for (i = 0; i < 3; i++) {
        worldTransform_position_data[i] =
            c_parentFrame->relativeTransform.position.data[i];
    }

    for (i41 = 0; i41 < 9; i41++) {
        worldTransform_rotation_data[i41] =
            c_parentFrame->relativeTransform.rotation.data[i41];
    }

    for (i = 0; i < 3; i++) {
        relativeTransform_position_data[i] =
            b_parentFrame->relativeTransform.position.data[i];
    }

    for (i41 = 0; i41 < 9; i41++) {
        relativeTransform_rotation_data[i41] =
            b_parentFrame->relativeTransform.rotation.data[i41];
    }

    for (i41 = 0; i41 < 3; i41++) {
        d3 = 0.0;
        i42 = 0;
        for (i43 = 0; i43 < 3; i43++) {
            d3 += worldTransform_rotation_data[i42 + i41] *
                relativeTransform_position_data[i43];
            b_R_data[i42 + i41] = 0.0;
            i44 = 0;
            for (i = 0; i < 3; i++) {
                b_R_data[i42 + i41] += worldTransform_rotation_data[i44 + i41] *
                    relativeTransform_rotation_data[i + i42];
                i44 += 3;
            }

            i42 += 3;
        }

        p_data[i41] = worldTransform_position_data[i41] + d3;
    }

    ZUpFrame_getRelativeTransform(obj, relativeTransform_position_data,
        relativeTransform_rotation_data);
    for (i = 0; i < 3; i++) {
        d3 = 0.0;
        i41 = 0;
        for (i42 = 0; i42 < 3; i42++) {
            d3 += b_R_data[i41 + i] * relativeTransform_position_data[i42];
            R_data[i41 + i] = 0.0;
            i43 = 0;
            for (i44 = 0; i44 < 3; i44++) {
                R_data[i41 + i] += b_R_data[i43 + i] *
                    relativeTransform_rotation_data[i44 + i41];
                i43 += 3;
            }

            i41 += 3;
        }

        worldTransform_position_data[i] = p_data[i] + d3;
    }

    memcpy(&worldTransform_rotation_data[0], &R_data[0], 9U * sizeof(double));
}

static void d_InverseKinematicsFunction_Inv(InverseKinematicsFunction **obj)
{
    InverseKinematicsFunction *b_obj;
    int i123;
    signed char c_rightFootTransform_rotation_d[9];
    static const signed char iv10[9] = { 1, 0, 0, 0, 1, 0, 0, 0, 1 };

    signed char leftFootTransform_rotation_data[9];
    b_obj = *obj;
    *obj = b_obj;
    b_obj = *obj;
    *obj = b_obj;
    b_obj = *obj;
    for (i123 = 0; i123 < 9; i123++) {
        c_rightFootTransform_rotation_d[i123] = iv10[i123];
    }

    for (i123 = 0; i123 < 9; i123++) {
        leftFootTransform_rotation_data[i123] = iv10[i123];
    }

    for (i123 = 0; i123 < 3; i123++) {
        b_obj->desiredState[i123] = 0.0;
    }

    for (i123 = 0; i123 < 3; i123++) {
        b_obj->desiredState[i123 + 3] = leftFootTransform_rotation_data[i123];
    }

    for (i123 = 0; i123 < 3; i123++) {
        b_obj->desiredState[i123 + 6] = 0.0;
    }

    for (i123 = 0; i123 < 3; i123++) {
        b_obj->desiredState[i123 + 9] = c_rightFootTransform_rotation_d[i123];
    }
}

static void d_mldivide(const double A[100], double B[10])
{
    double b_A[100];
    int i131;
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
    for (i131 = 0; i131 < 10; i131++) {
        ipiv[i131] = (signed char)(1 + i131);
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

            i131 = (c - j) + 10;
            for (jy = c + 1; jy + 1 <= i131; jy++) {
                b_A[jy] /= b_A[c];
            }
        }

        kAcol = c;
        jy = c + 10;
        for (k = 1; k <= 9 - j; k++) {
            smax = b_A[jy];
            if (b_A[jy] != 0.0) {
                ix = c + 1;
                i131 = (kAcol - j) + 20;
                for (ijA = 11 + kAcol; ijA + 1 <= i131; ijA++) {
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

static double d_xnrm2(int n, const double b_x[30], int ix0)
{
    double y;
    double scale;
    int kend;
    int k;
    double absxk;
    double t;
    y = 0.0;
    if (!(n < 1)) {
        if (n == 1) {
            y = fabs(b_x[ix0 - 1]);
        } else {
            scale = 2.2250738585072014E-308;
            kend = (ix0 + n) - 1;
            for (k = ix0; k <= kend; k++) {
                absxk = fabs(b_x[k - 1]);
                if (absxk > scale) {
                    t = scale / absxk;
                    y = 1.0 + y * t * t;
                    scale = absxk;
                } else {
                    t = absxk / scale;
                    y += t * t;
                }
            }

            y = scale * sqrt(y);
        }
    }

    return y;
}

static void e_Frame_getTransform(Frame_4 *obj, ZUpFrame *withRespectTo, double
    transform_position_data[3], double transform_rotation_data[9])
{
    ZUpFrame *inCoordinatesOf;
    Frame_4 *b_obj;
    Frame_2 *b_parentFrame;
    int i;
    double relativeTransform_position_data[3];
    Transform3d r19;
    double relativeTransform_rotation_data[9];
    Transform3d r20;
    double a_position_data[3];
    double a_rotation_data[9];
    double d9;
    int i82;
    double R_data[9];
    double p_data[3];
    int i83;
    Rotation3d a_rotation;
    ZUpFrame *a;
    int i84;
    int i85;
    bool b_bool;
    char b_a[11];
    char b[11];
    int exitg1;
    double c_data[9];
    inCoordinatesOf = withRespectTo;
    b_obj = obj;
    b_parentFrame = b_obj->parentFrame;
    for (i = 0; i < 3; i++) {
        relativeTransform_position_data[i] =
            b_obj->relativeTransform.position.data[i];
    }

    for (i = 0; i < 9; i++) {
        relativeTransform_rotation_data[i] =
            b_obj->relativeTransform.rotation.data[i];
    }

    b_Frame_getWorldTransform(b_parentFrame, &r19);
    for (i = 0; i < 3; i++) {
        a_position_data[i] = r19.position.data[i];
    }

    b_Frame_getWorldTransform(b_parentFrame, &r20);
    memcpy(&a_rotation_data[0], &r20.rotation.data[0], 9U * sizeof(double));
    for (i = 0; i < 3; i++) {
        d9 = 0.0;
        i82 = 0;
        for (i83 = 0; i83 < 3; i83++) {
            d9 += a_rotation_data[i82 + i] * relativeTransform_position_data[i83];
            R_data[i82 + i] = 0.0;
            i84 = 0;
            for (i85 = 0; i85 < 3; i85++) {
                R_data[i82 + i] += a_rotation_data[i84 + i] *
                    relativeTransform_rotation_data[i85 + i82];
                i84 += 3;
            }

            i82 += 3;
        }

        p_data[i] = a_position_data[i] + d9;
    }

    for (i = 0; i < 3; i++) {
        relativeTransform_position_data[i] = p_data[i];
    }

    memcpy(&relativeTransform_rotation_data[0], &R_data[0], 9U * sizeof(double));
    Frame_getWorldTransform(withRespectTo, a_position_data, &a_rotation);
    memcpy(&a_rotation_data[0], &a_rotation.data[0], 9U * sizeof(double));
    Transform3d_mldivide(a_position_data, a_rotation_data,
                         relativeTransform_position_data,
                         relativeTransform_rotation_data,
                         transform_position_data, transform_rotation_data);
    a = withRespectTo;
    for (i = 0; i < 11; i++) {
        b_a[i] = a->name[i];
        b[i] = inCoordinatesOf->name[i];
    }

    b_bool = false;
    i = 0;
    do {
        exitg1 = 0;
        if (i + 1 < 12) {
            if (b_a[i] != b[i]) {
                exitg1 = 1;
            } else {
                i++;
            }
        } else {
            b_bool = true;
            exitg1 = 1;
        }
    } while (exitg1 == 0);

    if (!b_bool) {
        Frame_getWorldTransform(inCoordinatesOf, relativeTransform_position_data,
                                &a_rotation);
        memcpy(&relativeTransform_rotation_data[0], &a_rotation.data[0], 9U *
               sizeof(double));
        Frame_getWorldTransform(withRespectTo, a_position_data, &a_rotation);
        memcpy(&a_rotation_data[0], &a_rotation.data[0], 9U * sizeof(double));
        for (i = 0; i < 3; i++) {
            d9 = 0.0;
            for (i82 = 0; i82 < 3; i82++) {
                c_data[i + 3 * i82] = 0.0;
                for (i83 = 0; i83 < 3; i83++) {
                    c_data[i + 3 * i82] += relativeTransform_rotation_data[i83 +
                        3 * i] * a_rotation_data[i83 + 3 * i82];
                }

                d9 += c_data[i + 3 * i82] * transform_position_data[i82];
            }

            p_data[i] = d9;
            for (i82 = 0; i82 < 3; i82++) {
                R_data[i + 3 * i82] = 0.0;
                for (i83 = 0; i83 < 3; i83++) {
                    R_data[i + 3 * i82] += c_data[i + 3 * i83] *
                        transform_rotation_data[i83 + 3 * i82];
                }
            }
        }

        for (i = 0; i < 3; i++) {
            transform_position_data[i] = p_data[i];
        }

        memcpy(&transform_rotation_data[0], &R_data[0], 9U * sizeof(double));
    }
}

static void f_Frame_getTransform(Frame_6 *obj, Frame_5 *withRespectTo, double
    transform_position_data[3], double transform_rotation_data[9])
{
    Frame_5 *inCoordinatesOf;
    Frame_5 *b_obj;
    WorldFrame *b_parentFrame;
    int i;
    int i89;
    double worldTransform_position_data[3];
    double worldTransform_rotation_data[9];
    double relativeTransform_position_data[3];
    double relativeTransform_rotation_data[9];
    double R_data[9];
    double d10;
    Frame_6 *c_obj;
    int i90;
    double b_worldTransform_position_data[3];
    int i91;
    double b_worldTransform_rotation_data[9];
    int i92;
    bool b_bool;
    char a[14];
    char b[14];
    int exitg1;
    double b_R_data[9];
    double c_data[9];
    inCoordinatesOf = withRespectTo;
    b_obj = withRespectTo;
    b_parentFrame = b_obj->parentFrame;
    for (i = 0; i < 3; i++) {
        worldTransform_position_data[i] =
            b_parentFrame->relativeTransform.position.data[i];
    }

    for (i89 = 0; i89 < 9; i89++) {
        worldTransform_rotation_data[i89] =
            b_parentFrame->relativeTransform.rotation.data[i89];
    }

    for (i = 0; i < 3; i++) {
        relativeTransform_position_data[i] =
            b_obj->relativeTransform.position.data[i];
    }

    for (i89 = 0; i89 < 9; i89++) {
        relativeTransform_rotation_data[i89] =
            b_obj->relativeTransform.rotation.data[i89];
    }

    for (i = 0; i < 3; i++) {
        d10 = 0.0;
        i89 = 0;
        for (i90 = 0; i90 < 3; i90++) {
            d10 += worldTransform_rotation_data[i89 + i] *
                relativeTransform_position_data[i90];
            R_data[i89 + i] = 0.0;
            i91 = 0;
            for (i92 = 0; i92 < 3; i92++) {
                R_data[i89 + i] += worldTransform_rotation_data[i91 + i] *
                    relativeTransform_rotation_data[i92 + i89];
                i91 += 3;
            }

            i89 += 3;
        }

        worldTransform_position_data[i] += d10;
    }

    memcpy(&worldTransform_rotation_data[0], &R_data[0], 9U * sizeof(double));
    c_obj = obj;
    b_parentFrame = c_obj->parentFrame;
    for (i = 0; i < 3; i++) {
        b_worldTransform_position_data[i] =
            b_parentFrame->relativeTransform.position.data[i];
    }

    for (i89 = 0; i89 < 9; i89++) {
        b_worldTransform_rotation_data[i89] =
            b_parentFrame->relativeTransform.rotation.data[i89];
    }

    for (i = 0; i < 3; i++) {
        relativeTransform_position_data[i] =
            c_obj->relativeTransform.position.data[i];
    }

    for (i89 = 0; i89 < 9; i89++) {
        relativeTransform_rotation_data[i89] =
            c_obj->relativeTransform.rotation.data[i89];
    }

    for (i = 0; i < 3; i++) {
        d10 = 0.0;
        i89 = 0;
        for (i90 = 0; i90 < 3; i90++) {
            d10 += b_worldTransform_rotation_data[i89 + i] *
                relativeTransform_position_data[i90];
            R_data[i89 + i] = 0.0;
            i91 = 0;
            for (i92 = 0; i92 < 3; i92++) {
                R_data[i89 + i] += b_worldTransform_rotation_data[i91 + i] *
                    relativeTransform_rotation_data[i92 + i89];
                i91 += 3;
            }

            i89 += 3;
        }

        b_worldTransform_position_data[i] += d10;
    }

    memcpy(&b_worldTransform_rotation_data[0], &R_data[0], 9U * sizeof(double));
    Transform3d_mldivide(worldTransform_position_data,
                         worldTransform_rotation_data,
                         b_worldTransform_position_data,
                         b_worldTransform_rotation_data, transform_position_data,
                         transform_rotation_data);
    b_obj = withRespectTo;
    for (i89 = 0; i89 < 14; i89++) {
        a[i89] = b_obj->name[i89];
        b[i89] = inCoordinatesOf->name[i89];
    }

    b_bool = false;
    i = 0;
    do {
        exitg1 = 0;
        if (i + 1 < 15) {
            if (a[i] != b[i]) {
                exitg1 = 1;
            } else {
                i++;
            }
        } else {
            b_bool = true;
            exitg1 = 1;
        }
    } while (exitg1 == 0);

    if (!b_bool) {
        b_parentFrame = inCoordinatesOf->parentFrame;
        for (i89 = 0; i89 < 9; i89++) {
            worldTransform_rotation_data[i89] =
                b_parentFrame->relativeTransform.rotation.data[i89];
            relativeTransform_rotation_data[i89] =
                inCoordinatesOf->relativeTransform.rotation.data[i89];
        }

        for (i89 = 0; i89 < 3; i89++) {
            i90 = 0;
            for (i91 = 0; i91 < 3; i91++) {
                R_data[i90 + i89] = 0.0;
                i92 = 0;
                for (i = 0; i < 3; i++) {
                    R_data[i90 + i89] += worldTransform_rotation_data[i92 + i89]
                        * relativeTransform_rotation_data[i + i90];
                    i92 += 3;
                }

                i90 += 3;
            }
        }

        b_obj = withRespectTo;
        b_parentFrame = b_obj->parentFrame;
        for (i89 = 0; i89 < 9; i89++) {
            worldTransform_rotation_data[i89] =
                b_parentFrame->relativeTransform.rotation.data[i89];
            relativeTransform_rotation_data[i89] =
                b_obj->relativeTransform.rotation.data[i89];
        }

        for (i89 = 0; i89 < 3; i89++) {
            i90 = 0;
            for (i91 = 0; i91 < 3; i91++) {
                b_R_data[i90 + i89] = 0.0;
                i92 = 0;
                for (i = 0; i < 3; i++) {
                    b_R_data[i90 + i89] += worldTransform_rotation_data[i92 +
                        i89] * relativeTransform_rotation_data[i + i90];
                    i92 += 3;
                }

                i90 += 3;
            }
        }

        i89 = 0;
        for (i90 = 0; i90 < 3; i90++) {
            d10 = 0.0;
            i91 = 0;
            for (i92 = 0; i92 < 3; i92++) {
                c_data[i91 + i90] = 0.0;
                for (i = 0; i < 3; i++) {
                    c_data[i91 + i90] += R_data[i + i89] * b_R_data[i + i91];
                }

                d10 += c_data[i91 + i90] * transform_position_data[i92];
                i91 += 3;
            }

            relativeTransform_position_data[i90] = d10;
            i89 += 3;
        }

        for (i = 0; i < 3; i++) {
            i89 = 0;
            for (i90 = 0; i90 < 3; i90++) {
                R_data[i89 + i] = 0.0;
                i91 = 0;
                for (i92 = 0; i92 < 3; i92++) {
                    R_data[i89 + i] += c_data[i91 + i] *
                        transform_rotation_data[i92 + i89];
                    i91 += 3;
                }

                i89 += 3;
            }

            transform_position_data[i] = relativeTransform_position_data[i];
        }

        memcpy(&transform_rotation_data[0], &R_data[0], 9U * sizeof(double));
    }
}

static void g_Frame_getTransform(Frame_7 *obj, Frame_5 *withRespectTo, double
    transform_position_data[3], double transform_rotation_data[9])
{
    Frame_5 *inCoordinatesOf;
    Frame_5 *b_obj;
    WorldFrame *b_parentFrame;
    int i;
    int i93;
    double worldTransform_position_data[3];
    double worldTransform_rotation_data[9];
    double relativeTransform_position_data[3];
    double relativeTransform_rotation_data[9];
    double R_data[9];
    double d11;
    Frame_7 *c_obj;
    int i94;
    double b_worldTransform_position_data[3];
    int i95;
    double b_worldTransform_rotation_data[9];
    int i96;
    bool b_bool;
    char a[14];
    char b[14];
    int exitg1;
    double b_R_data[9];
    double c_data[9];
    inCoordinatesOf = withRespectTo;
    b_obj = withRespectTo;
    b_parentFrame = b_obj->parentFrame;
    for (i = 0; i < 3; i++) {
        worldTransform_position_data[i] =
            b_parentFrame->relativeTransform.position.data[i];
    }

    for (i93 = 0; i93 < 9; i93++) {
        worldTransform_rotation_data[i93] =
            b_parentFrame->relativeTransform.rotation.data[i93];
    }

    for (i = 0; i < 3; i++) {
        relativeTransform_position_data[i] =
            b_obj->relativeTransform.position.data[i];
    }

    for (i93 = 0; i93 < 9; i93++) {
        relativeTransform_rotation_data[i93] =
            b_obj->relativeTransform.rotation.data[i93];
    }

    for (i = 0; i < 3; i++) {
        d11 = 0.0;
        i93 = 0;
        for (i94 = 0; i94 < 3; i94++) {
            d11 += worldTransform_rotation_data[i93 + i] *
                relativeTransform_position_data[i94];
            R_data[i93 + i] = 0.0;
            i95 = 0;
            for (i96 = 0; i96 < 3; i96++) {
                R_data[i93 + i] += worldTransform_rotation_data[i95 + i] *
                    relativeTransform_rotation_data[i96 + i93];
                i95 += 3;
            }

            i93 += 3;
        }

        worldTransform_position_data[i] += d11;
    }

    memcpy(&worldTransform_rotation_data[0], &R_data[0], 9U * sizeof(double));
    c_obj = obj;
    b_parentFrame = c_obj->parentFrame;
    for (i = 0; i < 3; i++) {
        b_worldTransform_position_data[i] =
            b_parentFrame->relativeTransform.position.data[i];
    }

    for (i93 = 0; i93 < 9; i93++) {
        b_worldTransform_rotation_data[i93] =
            b_parentFrame->relativeTransform.rotation.data[i93];
    }

    for (i = 0; i < 3; i++) {
        relativeTransform_position_data[i] =
            c_obj->relativeTransform.position.data[i];
    }

    for (i93 = 0; i93 < 9; i93++) {
        relativeTransform_rotation_data[i93] =
            c_obj->relativeTransform.rotation.data[i93];
    }

    for (i = 0; i < 3; i++) {
        d11 = 0.0;
        i93 = 0;
        for (i94 = 0; i94 < 3; i94++) {
            d11 += b_worldTransform_rotation_data[i93 + i] *
                relativeTransform_position_data[i94];
            R_data[i93 + i] = 0.0;
            i95 = 0;
            for (i96 = 0; i96 < 3; i96++) {
                R_data[i93 + i] += b_worldTransform_rotation_data[i95 + i] *
                    relativeTransform_rotation_data[i96 + i93];
                i95 += 3;
            }

            i93 += 3;
        }

        b_worldTransform_position_data[i] += d11;
    }

    memcpy(&b_worldTransform_rotation_data[0], &R_data[0], 9U * sizeof(double));
    Transform3d_mldivide(worldTransform_position_data,
                         worldTransform_rotation_data,
                         b_worldTransform_position_data,
                         b_worldTransform_rotation_data, transform_position_data,
                         transform_rotation_data);
    b_obj = withRespectTo;
    for (i93 = 0; i93 < 14; i93++) {
        a[i93] = b_obj->name[i93];
        b[i93] = inCoordinatesOf->name[i93];
    }

    b_bool = false;
    i = 0;
    do {
        exitg1 = 0;
        if (i + 1 < 15) {
            if (a[i] != b[i]) {
                exitg1 = 1;
            } else {
                i++;
            }
        } else {
            b_bool = true;
            exitg1 = 1;
        }
    } while (exitg1 == 0);

    if (!b_bool) {
        b_parentFrame = inCoordinatesOf->parentFrame;
        for (i93 = 0; i93 < 9; i93++) {
            worldTransform_rotation_data[i93] =
                b_parentFrame->relativeTransform.rotation.data[i93];
            relativeTransform_rotation_data[i93] =
                inCoordinatesOf->relativeTransform.rotation.data[i93];
        }

        for (i93 = 0; i93 < 3; i93++) {
            i94 = 0;
            for (i95 = 0; i95 < 3; i95++) {
                R_data[i94 + i93] = 0.0;
                i96 = 0;
                for (i = 0; i < 3; i++) {
                    R_data[i94 + i93] += worldTransform_rotation_data[i96 + i93]
                        * relativeTransform_rotation_data[i + i94];
                    i96 += 3;
                }

                i94 += 3;
            }
        }

        b_obj = withRespectTo;
        b_parentFrame = b_obj->parentFrame;
        for (i93 = 0; i93 < 9; i93++) {
            worldTransform_rotation_data[i93] =
                b_parentFrame->relativeTransform.rotation.data[i93];
            relativeTransform_rotation_data[i93] =
                b_obj->relativeTransform.rotation.data[i93];
        }

        for (i93 = 0; i93 < 3; i93++) {
            i94 = 0;
            for (i95 = 0; i95 < 3; i95++) {
                b_R_data[i94 + i93] = 0.0;
                i96 = 0;
                for (i = 0; i < 3; i++) {
                    b_R_data[i94 + i93] += worldTransform_rotation_data[i96 +
                        i93] * relativeTransform_rotation_data[i + i94];
                    i96 += 3;
                }

                i94 += 3;
            }
        }

        i93 = 0;
        for (i94 = 0; i94 < 3; i94++) {
            d11 = 0.0;
            i95 = 0;
            for (i96 = 0; i96 < 3; i96++) {
                c_data[i95 + i94] = 0.0;
                for (i = 0; i < 3; i++) {
                    c_data[i95 + i94] += R_data[i + i93] * b_R_data[i + i95];
                }

                d11 += c_data[i95 + i94] * transform_position_data[i96];
                i95 += 3;
            }

            relativeTransform_position_data[i94] = d11;
            i93 += 3;
        }

        for (i = 0; i < 3; i++) {
            i93 = 0;
            for (i94 = 0; i94 < 3; i94++) {
                R_data[i93 + i] = 0.0;
                i95 = 0;
                for (i96 = 0; i96 < 3; i96++) {
                    R_data[i93 + i] += c_data[i95 + i] *
                        transform_rotation_data[i96 + i93];
                    i95 += 3;
                }

                i93 += 3;
            }

            transform_position_data[i] = relativeTransform_position_data[i];
        }

        memcpy(&transform_rotation_data[0], &R_data[0], 9U * sizeof(double));
    }
}

static void generatedCenterOfMassJacobian(const double in1[10], double J[30])
{
    double t2;
    double t3;
    double t4;
    double t6;
    double t7;
    double t8;
    double t12;
    double t13;
    double t15;
    double t16;
    double t19;
    double t20;
    double t23;
    double t24;
    double t27;
    double t29;
    double t33;
    double t35;
    double t36;
    double t37;
    double t39;
    double t40;
    double t41;
    double t43;
    double t44;
    double t45;
    double t48;
    double t49;
    double t51;
    double t52;
    double t55;
    double t56;
    double t59;
    double t60;
    double t63;
    double t65;
    double t69;
    double t71;
    double t72;
    double t73;
    double t75;
    double t76;
    double t78;
    double t81;
    double t85;
    double t87;
    double t91;
    double t93;
    double t96;
    double t98;
    double t102;
    double t104;
    double t107;
    double t109;
    double t113;
    double t115;
    double t119;
    double t121;
    double t122;
    double t124;
    double t125;
    double t126;
    double t128;
    double t131;
    double t135;
    double t137;
    double t141;
    double t143;
    double t146;
    double t148;
    double t152;
    double t154;
    double t157;
    double t159;
    double t163;
    double t165;
    double t169;
    double t171;
    double t172;
    double t174;
    double t177;
    double t179;
    double t183;
    double t185;
    double t187;
    double t188;
    double t189;
    double t190;
    double t191;
    double t192;
    double t195;
    double t197;
    double t201;
    double t203;
    double t205;
    double t206;
    double t207;
    double t208;
    double t209;
    double t210;
    double dv9[30];
    int i8;
    int i9;
    int i10;
    t2 = sin(in1[1]);
    t3 = cos(in1[2]);
    t4 = sin(in1[2]);
    t6 = cos(in1[3] - 0.22689280275926282);
    t7 = sin(in1[3]);
    t8 = cos(in1[3]);
    t12 = t2 * t3 * t7 + t2 * t4 * t8;
    t13 = sin(in1[3] - 0.22689280275926282);
    t15 = t2 * t3 * t8 - t2 * t4 * t7;
    t16 = cos(in1[4]);
    t19 = t6 * t12 - t13 * t15;
    t20 = sin(in1[4]);
    t23 = t6 * t15 + t12 * t13;
    t24 = cos(in1[1]);
    t27 = t3 * t7 * t24 + t4 * t8 * t24;
    t29 = t3 * t8 * t24 - t4 * t7 * t24;
    t33 = t6 * t29 + t13 * t27;
    t35 = t6 * t27 - t13 * t29;
    t36 = t3 * t8 * t24 * 0.01944171628216226;
    t37 = t16 * t33 * 2.2269015091641261E-5;
    t39 = sin(in1[6]);
    t40 = cos(in1[7]);
    t41 = sin(in1[7]);
    t43 = cos(in1[8] - 0.22689280275926282);
    t44 = sin(in1[8]);
    t45 = cos(in1[8]);
    t48 = t39 * t40 * t44 + t39 * t41 * t45;
    t49 = sin(in1[8] - 0.22689280275926282);
    t51 = t39 * t40 * t45 - t39 * t41 * t44;
    t52 = cos(in1[9]);
    t55 = t43 * t48 - t49 * t51;
    t56 = sin(in1[9]);
    t59 = t43 * t51 + t48 * t49;
    t60 = cos(in1[6]);
    t63 = t40 * t44 * t60 + t41 * t45 * t60;
    t65 = t40 * t45 * t60 - t41 * t44 * t60;
    t69 = t43 * t65 + t49 * t63;
    t71 = t43 * t63 - t49 * t65;
    t72 = t40 * t45 * t60 * 0.01944171628216226;
    t73 = t52 * t69 * 2.2269015091641261E-5;
    t75 = cos(in1[0]);
    t76 = sin(in1[0]);
    t78 = t3 * t75 - t2 * t4 * t76;
    t81 = t4 * t75 + t2 * t3 * t76;
    t85 = t8 * t81 + t7 * t78;
    t87 = t8 * t78 - t7 * t81;
    t91 = t6 * t87 + t13 * t85;
    t93 = t6 * t85 - t13 * t87;
    t96 = t3 * t7 * t24 * t75 + t4 * t8 * t24 * t75;
    t98 = t3 * t8 * t24 * t75 - t4 * t7 * t24 * t75;
    t102 = t6 * t98 + t13 * t96;
    t104 = t6 * t96 - t13 * t98;
    t107 = t3 * t76 + t2 * t4 * t75;
    t109 = t4 * t76 - t2 * t3 * t75;
    t113 = t8 * t109 + t7 * t107;
    t115 = t8 * t107 - t7 * t109;
    t119 = t6 * t115 + t13 * t113;
    t121 = t6 * t113 - t13 * t115;
    t122 = t7 * t109 * 0.00361124807120545;
    t124 = t20 * t121 * 0.0001291039102781228;
    t125 = cos(in1[5]);
    t126 = sin(in1[5]);
    t128 = t40 * t125 - t39 * t41 * t126;
    t131 = t41 * t125 + t39 * t40 * t126;
    t135 = t45 * t131 + t44 * t128;
    t137 = t45 * t128 - t44 * t131;
    t141 = t43 * t137 + t49 * t135;
    t143 = t43 * t135 - t49 * t137;
    t146 = t40 * t44 * t60 * t125 + t41 * t45 * t60 * t125;
    t148 = t40 * t45 * t60 * t125 - t41 * t44 * t60 * t125;
    t152 = t43 * t148 + t49 * t146;
    t154 = t43 * t146 - t49 * t148;
    t157 = t40 * t126 + t39 * t41 * t125;
    t159 = t41 * t126 - t39 * t40 * t125;
    t163 = t45 * t159 + t44 * t157;
    t165 = t45 * t157 - t44 * t159;
    t169 = t43 * t165 + t49 * t163;
    t171 = t43 * t163 - t49 * t165;
    t172 = t44 * t159 * 0.00361124807120545;
    t174 = t56 * t171 * 0.0001291039102781228;
    t177 = t3 * t7 * t24 * t76 + t4 * t8 * t24 * t76;
    t179 = t3 * t8 * t24 * t76 - t4 * t7 * t24 * t76;
    t183 = t6 * t179 + t13 * t177;
    t185 = t6 * t177 - t13 * t179;
    t187 = t8 * t81 * 0.01944171628216226;
    t188 = t8 * t78 * 0.00361124807120545;
    t189 = t7 * t78 * 0.01944171628216226;
    t190 = t16 * t91 * 0.0001291039102781228;
    t191 = t16 * t93 * 2.2269015091641261E-5;
    t192 = t20 * t91 * 2.2269015091641261E-5;
    t195 = t40 * t44 * t60 * t126 + t41 * t45 * t60 * t126;
    t197 = t40 * t45 * t60 * t126 - t41 * t44 * t60 * t126;
    t201 = t43 * t197 + t49 * t195;
    t203 = t43 * t195 - t49 * t197;
    t205 = t45 * t131 * 0.01944171628216226;
    t206 = t45 * t128 * 0.00361124807120545;
    t207 = t44 * t128 * 0.01944171628216226;
    t208 = t52 * t141 * 0.0001291039102781228;
    t209 = t52 * t143 * 2.2269015091641261E-5;
    t210 = t56 * t141 * 2.2269015091641261E-5;
    dv9[0] = 0.0;
    dv9[1] = (((((((((((((((((t75 * 0.02221404964372185 - t76 *
        5.7079773688106083E-6) + t3 * t75 * 0.0188241315720146) - t4 * t75 *
                            8.6560535922622416E-6) - t7 * t78 *
                           0.00361124807120545) + t8 * t78 * 0.01944171628216226)
                         - t7 * t81 * 0.01944171628216226) - t8 * t81 *
                        0.00361124807120545) + t6 * t85 * 0.0009379135147341086)
                      + t6 * t87 * 0.0046259117082533587) + t13 * t85 *
                     0.0046259117082533587) - t13 * t87 * 0.0009379135147341086)
                   - t24 * t76 * 0.0059599891705242559) + t16 * t91 *
                  2.2269015091641261E-5) - t16 * t93 * 0.0001291039102781228) -
                t20 * t91 * 0.0001291039102781228) - t20 * t93 *
               2.2269015091641261E-5) - t2 * t3 * t76 * 8.6560535922622416E-6) -
        t2 * t4 * t76 * 0.0188241315720146;
    dv9[2] = (((((((((((((((((t75 * 5.7079773688106083E-6 + t76 *
        0.02221404964372185) + t3 * t76 * 0.0188241315720146) - t4 * t76 *
                            8.6560535922622416E-6) + t24 * t75 *
                           0.0059599891705242559) - t7 * t107 *
                          0.00361124807120545) + t8 * t107 * 0.01944171628216226)
                        - t7 * t109 * 0.01944171628216226) - t8 * t109 *
                       0.00361124807120545) + t6 * t113 * 0.0009379135147341086)
                     + t6 * t115 * 0.0046259117082533587) + t13 * t113 *
                    0.0046259117082533587) - t13 * t115 * 0.0009379135147341086)
                  + t16 * t119 * 2.2269015091641261E-5) - t16 * t121 *
                 0.0001291039102781228) - t20 * t119 * 0.0001291039102781228) -
               t20 * t121 * 2.2269015091641261E-5) + t2 * t3 * t75 *
              8.6560535922622416E-6) + t2 * t4 * t75 * 0.0188241315720146;
    dv9[3] = (((((((((((((t24 * -0.0059599891705242559 - t2 * t3 *
                          8.6560535922622416E-6) - t2 * t4 * 0.0188241315720146)
                        - t6 * t12 * 0.0046259117082533587) + t6 * t15 *
                       0.0009379135147341086) + t12 * t13 *
                      0.0009379135147341086) + t13 * t15 * 0.0046259117082533587)
                    - t16 * t19 * 2.2269015091641261E-5) - t16 * t23 *
                   0.0001291039102781228) + t19 * t20 * 0.0001291039102781228) -
                 t20 * t23 * 2.2269015091641261E-5) - t2 * t3 * t7 *
                0.01944171628216226) - t2 * t3 * t8 * 0.00361124807120545) + t2 *
              t4 * t7 * 0.00361124807120545) - t2 * t4 * t8 *
        0.01944171628216226;
    dv9[4] = (((((((((((((t2 * t75 * -0.0059599891705242559 + t6 * t96 *
                          0.0046259117082533587) - t6 * t98 *
                         0.0009379135147341086) - t13 * t96 *
                        0.0009379135147341086) - t13 * t98 *
                       0.0046259117082533587) + t16 * t102 *
                      0.0001291039102781228) + t16 * t104 *
                     2.2269015091641261E-5) + t20 * t102 * 2.2269015091641261E-5)
                   - t20 * t104 * 0.0001291039102781228) + t3 * t24 * t75 *
                  8.6560535922622416E-6) + t4 * t24 * t75 * 0.0188241315720146)
                + t3 * t7 * t24 * t75 * 0.01944171628216226) + t3 * t8 * t24 *
               t75 * 0.00361124807120545) - t4 * t7 * t24 * t75 *
              0.00361124807120545) + t4 * t8 * t24 * t75 * 0.01944171628216226;
    dv9[5] = (((((((((((((t2 * t76 * -0.0059599891705242559 + t6 * t177 *
                          0.0046259117082533587) - t6 * t179 *
                         0.0009379135147341086) - t13 * t177 *
                        0.0009379135147341086) - t13 * t179 *
                       0.0046259117082533587) + t16 * t183 *
                      0.0001291039102781228) + t16 * t185 *
                     2.2269015091641261E-5) + t20 * t183 * 2.2269015091641261E-5)
                   - t20 * t185 * 0.0001291039102781228) + t3 * t24 * t76 *
                  8.6560535922622416E-6) + t4 * t24 * t76 * 0.0188241315720146)
                + t3 * t7 * t24 * t76 * 0.01944171628216226) + t3 * t8 * t24 *
               t76 * 0.00361124807120545) - t4 * t7 * t24 * t76 *
              0.00361124807120545) + t4 * t8 * t24 * t76 * 0.01944171628216226;
    dv9[6] = ((((((((((((t36 + t37) + t3 * t24 * 0.0188241315720146) - t4 * t24 *
                       8.6560535922622416E-6) + t6 * t27 * 0.0009379135147341086)
                     + t6 * t29 * 0.0046259117082533587) + t13 * t27 *
                    0.0046259117082533587) - t13 * t29 * 0.0009379135147341086)
                  - t16 * t35 * 0.0001291039102781228) - t20 * t33 *
                 0.0001291039102781228) - t20 * t35 * 2.2269015091641261E-5) -
               t3 * t7 * t24 * 0.00361124807120545) - t4 * t7 * t24 *
              0.01944171628216226) - t4 * t8 * t24 * 0.00361124807120545;
    dv9[7] = ((((((((((((((t122 + t124) - t3 * t76 * 8.6560535922622416E-6) - t4
                         * t76 * 0.0188241315720146) - t7 * t107 *
                        0.01944171628216226) - t8 * t107 * 0.00361124807120545)
                      - t8 * t109 * 0.01944171628216226) - t6 * t113 *
                     0.0046259117082533587) + t6 * t115 * 0.0009379135147341086)
                   + t13 * t113 * 0.0009379135147341086) + t13 * t115 *
                  0.0046259117082533587) - t16 * t119 * 0.0001291039102781228) -
                t16 * t121 * 2.2269015091641261E-5) - t20 * t119 *
               2.2269015091641261E-5) + t2 * t3 * t75 * 0.0188241315720146) - t2
        * t4 * t75 * 8.6560535922622416E-6;
    dv9[8] = ((((((((((((((t187 + t188) + t189) + t190) + t191) + t192) + t3 *
                      t75 * 8.6560535922622416E-6) + t4 * t75 *
                     0.0188241315720146) - t7 * t81 * 0.00361124807120545) + t6 *
                   t85 * 0.0046259117082533587) - t6 * t87 *
                  0.0009379135147341086) - t13 * t85 * 0.0009379135147341086) -
                t13 * t87 * 0.0046259117082533587) - t20 * t93 *
               0.0001291039102781228) + t2 * t3 * t76 * 0.0188241315720146) - t2
        * t4 * t76 * 8.6560535922622416E-6;
    dv9[9] = ((t36 - t3 * t7 * t24 * 0.00361124807120545) - t4 * t7 * t24 *
              0.01944171628216226) - t4 * t8 * t24 * 0.00361124807120545;
    dv9[10] = ((t122 - t7 * t107 * 0.01944171628216226) - t8 * t107 *
               0.00361124807120545) - t8 * t109 * 0.01944171628216226;
    dv9[11] = ((t187 + t188) + t189) - t7 * t81 * 0.00361124807120545;
    dv9[12] = ((t37 - t16 * t35 * 0.0001291039102781228) - t20 * t33 *
               0.0001291039102781228) - t20 * t35 * 2.2269015091641261E-5;
    dv9[13] = ((t124 - t16 * t119 * 0.0001291039102781228) - t16 * t121 *
               2.2269015091641261E-5) - t20 * t119 * 2.2269015091641261E-5;
    dv9[14] = ((t190 + t191) + t192) - t20 * t93 * 0.0001291039102781228;
    dv9[15] = 0.0;
    dv9[16] = (((((((((((((((((t125 * 0.02221404964372185 + t126 *
        5.7079773688106083E-6) + t40 * t125 * 0.0188241315720146) - t41 * t125 *
        8.6560535922622416E-6) - t44 * t128 * 0.00361124807120545) + t45 * t128 *
                           0.01944171628216226) - t44 * t131 *
                          0.01944171628216226) - t45 * t131 *
                         0.00361124807120545) + t43 * t135 *
                        0.0009379135147341086) + t43 * t137 *
                       0.0046259117082533587) + t49 * t135 *
                      0.0046259117082533587) - t49 * t137 *
                     0.0009379135147341086) + t60 * t126 * 0.0059603564255516668)
                   + t52 * t141 * 2.2269015091641261E-5) - t52 * t143 *
                  0.0001291039102781228) - t56 * t141 * 0.0001291039102781228) -
                t56 * t143 * 2.2269015091641261E-5) - t39 * t40 * t126 *
               8.6560535922622416E-6) - t39 * t41 * t126 * 0.0188241315720146;
    dv9[17] = (((((((((((((((((t125 * -5.7079773688106083E-6 + t126 *
        0.02221404964372185) + t40 * t126 * 0.0188241315720146) - t41 * t126 *
        8.6560535922622416E-6) - t60 * t125 * 0.0059603564255516668) - t44 *
                           t157 * 0.00361124807120545) + t45 * t157 *
                          0.01944171628216226) - t44 * t159 *
                         0.01944171628216226) - t45 * t159 * 0.00361124807120545)
                       + t43 * t163 * 0.0009379135147341086) + t43 * t165 *
                      0.0046259117082533587) + t49 * t163 *
                     0.0046259117082533587) - t49 * t165 * 0.0009379135147341086)
                   + t52 * t169 * 2.2269015091641261E-5) - t52 * t171 *
                  0.0001291039102781228) - t56 * t169 * 0.0001291039102781228) -
                t56 * t171 * 2.2269015091641261E-5) + t39 * t40 * t125 *
               8.6560535922622416E-6) + t39 * t41 * t125 * 0.0188241315720146;
    dv9[18] = (((((((((((((t60 * 0.0059603564255516668 - t39 * t40 *
                           8.6560535922622416E-6) - t39 * t41 *
                          0.0188241315720146) - t43 * t48 *
                         0.0046259117082533587) + t43 * t51 *
                        0.0009379135147341086) + t48 * t49 *
                       0.0009379135147341086) + t49 * t51 *
                      0.0046259117082533587) - t52 * t55 * 2.2269015091641261E-5)
                    - t52 * t59 * 0.0001291039102781228) + t55 * t56 *
                   0.0001291039102781228) - t56 * t59 * 2.2269015091641261E-5) -
                 t39 * t40 * t44 * 0.01944171628216226) - t39 * t40 * t45 *
                0.00361124807120545) + t39 * t41 * t44 * 0.00361124807120545) -
        t39 * t41 * t45 * 0.01944171628216226;
    dv9[19] = (((((((((((((t39 * t125 * 0.0059603564255516668 + t43 * t146 *
                           0.0046259117082533587) - t43 * t148 *
                          0.0009379135147341086) - t49 * t146 *
                         0.0009379135147341086) - t49 * t148 *
                        0.0046259117082533587) + t52 * t152 *
                       0.0001291039102781228) + t52 * t154 *
                      2.2269015091641261E-5) + t56 * t152 *
                     2.2269015091641261E-5) - t56 * t154 * 0.0001291039102781228)
                   + t40 * t60 * t125 * 8.6560535922622416E-6) + t41 * t60 *
                  t125 * 0.0188241315720146) + t40 * t44 * t60 * t125 *
                 0.01944171628216226) + t40 * t45 * t60 * t125 *
                0.00361124807120545) - t41 * t44 * t60 * t125 *
               0.00361124807120545) + t41 * t45 * t60 * t125 *
        0.01944171628216226;
    dv9[20] = (((((((((((((t39 * t126 * 0.0059603564255516668 + t43 * t195 *
                           0.0046259117082533587) - t43 * t197 *
                          0.0009379135147341086) - t49 * t195 *
                         0.0009379135147341086) - t49 * t197 *
                        0.0046259117082533587) + t52 * t201 *
                       0.0001291039102781228) + t52 * t203 *
                      2.2269015091641261E-5) + t56 * t201 *
                     2.2269015091641261E-5) - t56 * t203 * 0.0001291039102781228)
                   + t40 * t60 * t126 * 8.6560535922622416E-6) + t41 * t60 *
                  t126 * 0.0188241315720146) + t40 * t44 * t60 * t126 *
                 0.01944171628216226) + t40 * t45 * t60 * t126 *
                0.00361124807120545) - t41 * t44 * t60 * t126 *
               0.00361124807120545) + t41 * t45 * t60 * t126 *
        0.01944171628216226;
    dv9[21] = ((((((((((((t72 + t73) + t40 * t60 * 0.0188241315720146) - t41 *
                        t60 * 8.6560535922622416E-6) + t43 * t63 *
                       0.0009379135147341086) + t43 * t65 *
                      0.0046259117082533587) + t49 * t63 * 0.0046259117082533587)
                    - t49 * t65 * 0.0009379135147341086) - t52 * t71 *
                   0.0001291039102781228) - t56 * t69 * 0.0001291039102781228) -
                 t56 * t71 * 2.2269015091641261E-5) - t40 * t44 * t60 *
                0.00361124807120545) - t41 * t44 * t60 * 0.01944171628216226) -
        t41 * t45 * t60 * 0.00361124807120545;
    dv9[22] = ((((((((((((((t172 + t174) - t40 * t126 * 8.6560535922622416E-6) -
                          t41 * t126 * 0.0188241315720146) - t44 * t157 *
                         0.01944171628216226) - t45 * t157 * 0.00361124807120545)
                       - t45 * t159 * 0.01944171628216226) - t43 * t163 *
                      0.0046259117082533587) + t43 * t165 *
                     0.0009379135147341086) + t49 * t163 * 0.0009379135147341086)
                   + t49 * t165 * 0.0046259117082533587) - t52 * t169 *
                  0.0001291039102781228) - t52 * t171 * 2.2269015091641261E-5) -
                t56 * t169 * 2.2269015091641261E-5) + t39 * t40 * t125 *
               0.0188241315720146) - t39 * t41 * t125 * 8.6560535922622416E-6;
    dv9[23] = ((((((((((((((t205 + t206) + t207) + t208) + t209) + t210) + t40 *
                       t125 * 8.6560535922622416E-6) + t41 * t125 *
                      0.0188241315720146) - t44 * t131 * 0.00361124807120545) +
                    t43 * t135 * 0.0046259117082533587) - t43 * t137 *
                   0.0009379135147341086) - t49 * t135 * 0.0009379135147341086)
                 - t49 * t137 * 0.0046259117082533587) - t56 * t143 *
                0.0001291039102781228) + t39 * t40 * t126 * 0.0188241315720146)
        - t39 * t41 * t126 * 8.6560535922622416E-6;
    dv9[24] = ((t72 - t40 * t44 * t60 * 0.00361124807120545) - t41 * t44 * t60 *
               0.01944171628216226) - t41 * t45 * t60 * 0.00361124807120545;
    dv9[25] = ((t172 - t44 * t157 * 0.01944171628216226) - t45 * t157 *
               0.00361124807120545) - t45 * t159 * 0.01944171628216226;
    dv9[26] = ((t205 + t206) + t207) - t44 * t131 * 0.00361124807120545;
    dv9[27] = ((t73 - t52 * t71 * 0.0001291039102781228) - t56 * t69 *
               0.0001291039102781228) - t56 * t71 * 2.2269015091641261E-5;
    dv9[28] = ((t174 - t52 * t169 * 0.0001291039102781228) - t52 * t171 *
               2.2269015091641261E-5) - t56 * t169 * 2.2269015091641261E-5;
    dv9[29] = ((t208 + t209) + t210) - t56 * t143 * 0.0001291039102781228;
    i8 = 0;
    for (i9 = 0; i9 < 10; i9++) {
        for (i10 = 0; i10 < 3; i10++) {
            J[i10 + i8] = dv9[i10 + i8];
        }

        i8 += 3;
    }
}

static void generatedCenterOfMassPosition(const double in1[10], double p[3])
{
    double t2;
    double t3;
    double t4;
    double t5;
    double t7;
    double t8;
    double t9;
    double t11;
    double t12;
    double t14;
    double t15;
    double t16;
    double t19;
    double t20;
    double t22;
    double t25;
    double t26;
    double t28;
    double t30;
    double t32;
    double t35;
    double t36;
    double t38;
    double t41;
    double t42;
    double t44;
    double t45;
    double t46;
    double t47;
    double t50;
    double t52;
    double t53;
    double t54;
    double t55;
    double t58;
    double t60;
    double t64;
    double t66;
    double t70;
    double t72;
    double t77;
    double t79;
    double t82;
    double t84;
    double t86;
    double t89;
    double t91;
    double t94;
    double t98;
    double t100;
    double t104;
    double t106;
    double t111;
    double t113;
    double t116;
    double t118;
    t2 = cos(in1[1]);
    t3 = cos(in1[6]);
    t4 = cos(in1[2]);
    t5 = sin(in1[2]);
    t7 = cos(in1[3] - 0.22689280275926282);
    t8 = cos(in1[3]);
    t9 = sin(in1[3]);
    t11 = cos(in1[7]);
    t12 = sin(in1[7]);
    t14 = cos(in1[8] - 0.22689280275926282);
    t15 = cos(in1[8]);
    t16 = sin(in1[8]);
    t19 = t2 * t4 * t9 + t2 * t5 * t8;
    t20 = sin(in1[3] - 0.22689280275926282);
    t22 = t2 * t4 * t8 - t2 * t5 * t9;
    t25 = t3 * t11 * t16 + t3 * t12 * t15;
    t26 = sin(in1[8] - 0.22689280275926282);
    t28 = t3 * t11 * t15 - t3 * t12 * t16;
    t30 = cos(in1[4]);
    t32 = cos(in1[9]);
    t35 = t7 * t22 + t19 * t20;
    t36 = sin(in1[4]);
    t38 = t7 * t19 - t20 * t22;
    t41 = t14 * t28 + t25 * t26;
    t42 = sin(in1[9]);
    t44 = t14 * t25 - t26 * t28;
    t45 = sin(in1[0]);
    t46 = cos(in1[0]);
    t47 = sin(in1[1]);
    t50 = t4 * t45 + t5 * t46 * t47;
    t52 = t5 * t45 - t4 * t46 * t47;
    t53 = sin(in1[5]);
    t54 = cos(in1[5]);
    t55 = sin(in1[6]);
    t58 = t11 * t53 + t12 * t54 * t55;
    t60 = t12 * t53 - t11 * t54 * t55;
    t64 = t8 * t52 + t9 * t50;
    t66 = t8 * t50 - t9 * t52;
    t70 = t15 * t60 + t16 * t58;
    t72 = t15 * t58 - t16 * t60;
    t77 = t7 * t66 + t20 * t64;
    t79 = t7 * t64 - t20 * t66;
    t82 = t14 * t72 + t26 * t70;
    t84 = t14 * t70 - t26 * t72;
    t86 = t4 * t46 - t5 * t45 * t47;
    t89 = t5 * t46 + t4 * t45 * t47;
    t91 = t11 * t54 - t12 * t53 * t55;
    t94 = t12 * t54 + t11 * t53 * t55;
    t98 = t8 * t89 + t9 * t86;
    t100 = t8 * t86 - t9 * t89;
    t104 = t15 * t94 + t16 * t91;
    t106 = t15 * t91 - t16 * t94;
    t111 = t7 * t100 + t20 * t98;
    t113 = t7 * t98 - t20 * t100;
    t116 = t14 * t106 + t26 * t104;
    t118 = t14 * t104 - t26 * t106;
    p[0] = (((((((((((((((((((((((((((((t47 * -0.0059599891705242559 + t55 *
        0.0059603564255516668) + t2 * t4 * 8.6560535922622416E-6) + t2 * t5 *
        0.0188241315720146) + t3 * t11 * 8.6560535922622416E-6) + t3 * t12 *
        0.0188241315720146) + t7 * t19 * 0.0046259117082533587) - t7 * t22 *
        0.0009379135147341086) + t14 * t25 * 0.0046259117082533587) - t19 * t20 *
        0.0009379135147341086) - t14 * t28 * 0.0009379135147341086) - t20 * t22 *
        0.0046259117082533587) - t25 * t26 * 0.0009379135147341086) - t26 * t28 *
                            0.0046259117082533587) + t30 * t35 *
                           0.0001291039102781228) + t30 * t38 *
                          2.2269015091641261E-5) + t35 * t36 *
                         2.2269015091641261E-5) + t32 * t41 *
                        0.0001291039102781228) - t36 * t38 *
                       0.0001291039102781228) + t32 * t44 *
                      2.2269015091641261E-5) + t41 * t42 * 2.2269015091641261E-5)
                    - t42 * t44 * 0.0001291039102781228) + t2 * t4 * t8 *
                   0.00361124807120545) + t2 * t4 * t9 * 0.01944171628216226) +
                 t2 * t5 * t8 * 0.01944171628216226) - t2 * t5 * t9 *
                0.00361124807120545) + t3 * t11 * t15 * 0.00361124807120545) +
              t3 * t11 * t16 * 0.01944171628216226) + t3 * t12 * t15 *
             0.01944171628216226) - t3 * t12 * t16 * 0.00361124807120545) -
        0.013778775105691671;
    p[1] = (((((((((((((((((((((((((((((((((((((t45 * 0.02221404964372185 + t46 *
        5.7079773688106083E-6) + t53 * 0.02221404964372185) - t54 *
        5.7079773688106083E-6) + t2 * t46 * 0.0059599891705242559) + t4 * t45 *
        0.0188241315720146) - t5 * t45 * 8.6560535922622416E-6) - t3 * t54 *
        0.0059603564255516668) + t8 * t50 * 0.01944171628216226) - t9 * t50 *
        0.00361124807120545) - t8 * t52 * 0.00361124807120545) - t9 * t52 *
        0.01944171628216226) + t11 * t53 * 0.0188241315720146) - t12 * t53 *
        8.6560535922622416E-6) + t7 * t64 * 0.0009379135147341086) + t7 * t66 *
        0.0046259117082533587) + t15 * t58 * 0.01944171628216226) - t16 * t58 *
        0.00361124807120545) - t15 * t60 * 0.00361124807120545) - t16 * t60 *
        0.01944171628216226) + t14 * t70 * 0.0009379135147341086) + t20 * t64 *
                            0.0046259117082533587) + t14 * t72 *
                           0.0046259117082533587) - t20 * t66 *
                          0.0009379135147341086) + t26 * t70 *
                         0.0046259117082533587) - t26 * t72 *
                        0.0009379135147341086) + t30 * t77 *
                       2.2269015091641261E-5) - t30 * t79 *
                      0.0001291039102781228) - t36 * t77 * 0.0001291039102781228)
                    + t32 * t82 * 2.2269015091641261E-5) - t36 * t79 *
                   2.2269015091641261E-5) - t32 * t84 * 0.0001291039102781228) -
                 t42 * t82 * 0.0001291039102781228) - t42 * t84 *
                2.2269015091641261E-5) + t4 * t46 * t47 * 8.6560535922622416E-6)
              + t5 * t46 * t47 * 0.0188241315720146) + t11 * t54 * t55 *
             8.6560535922622416E-6) + t12 * t54 * t55 * 0.0188241315720146) +
        0.0001120952667695357;
    p[2] = (((((((((((((((((((((((((((((((((((((t45 * 5.7079773688106083E-6 -
        t46 * 0.02221404964372185) - t53 * 5.7079773688106083E-6) - t54 *
        0.02221404964372185) + t2 * t45 * 0.0059599891705242559) - t4 * t46 *
        0.0188241315720146) + t5 * t46 * 8.6560535922622416E-6) - t3 * t53 *
        0.0059603564255516668) - t11 * t54 * 0.0188241315720146) + t12 * t54 *
        8.6560535922622416E-6) - t8 * t86 * 0.01944171628216226) + t9 * t86 *
        0.00361124807120545) + t8 * t89 * 0.00361124807120545) + t9 * t89 *
        0.01944171628216226) - t7 * t98 * 0.0009379135147341086) - t15 * t91 *
        0.01944171628216226) - t7 * t100 * 0.0046259117082533587) + t16 * t91 *
        0.00361124807120545) + t15 * t94 * 0.00361124807120545) + t16 * t94 *
        0.01944171628216226) - t14 * t104 * 0.0009379135147341086) - t20 * t98 *
                            0.0046259117082533587) - t14 * t106 *
                           0.0046259117082533587) + t20 * t100 *
                          0.0009379135147341086) - t26 * t104 *
                         0.0046259117082533587) + t26 * t106 *
                        0.0009379135147341086) - t30 * t111 *
                       2.2269015091641261E-5) + t30 * t113 *
                      0.0001291039102781228) + t36 * t111 *
                     0.0001291039102781228) - t32 * t116 * 2.2269015091641261E-5)
                   + t36 * t113 * 2.2269015091641261E-5) + t32 * t118 *
                  0.0001291039102781228) + t42 * t116 * 0.0001291039102781228) +
                t42 * t118 * 2.2269015091641261E-5) + t4 * t45 * t47 *
               8.6560535922622416E-6) + t5 * t45 * t47 * 0.0188241315720146) +
             t11 * t53 * t55 * 8.6560535922622416E-6) + t12 * t53 * t55 *
            0.0188241315720146) + 0.00920412291596101;
}

static void generatedGravityVector(const double in1[10], double G[10])
{
    double t2;
    double t3;
    double t4;
    double t5;
    double t6;
    double t8;
    double t9;
    double t12;
    double t13;
    double t15;
    double t20;
    double t21;
    double t23;
    double t24;
    double t27;
    double t29;
    double t31;
    double t34;
    double t38;
    double t40;
    double t41;
    double t42;
    double t43;
    double t44;
    double t45;
    double t46;
    double t47;
    double t48;
    double t50;
    double t51;
    double t54;
    double t55;
    double t57;
    double t61;
    double t62;
    double t64;
    double t65;
    double t68;
    double t70;
    double t75;
    double t79;
    double t81;
    double t82;
    double t83;
    t2 = sin(in1[0]);
    t3 = cos(in1[2]);
    t4 = cos(in1[0]);
    t5 = sin(in1[1]);
    t6 = sin(in1[2]);
    t8 = cos(in1[3] - 0.22689280275926282);
    t9 = cos(in1[3]);
    t12 = t2 * t3 + t4 * t5 * t6;
    t13 = sin(in1[3]);
    t15 = t2 * t6 - t3 * t4 * t5;
    t20 = t9 * t15 + t12 * t13;
    t21 = sin(in1[3] - 0.22689280275926282);
    t23 = t9 * t12 - t13 * t15;
    t24 = cos(in1[1]);
    t27 = t2 * t3 * t13 * t24 + t2 * t6 * t9 * t24;
    t29 = t2 * t3 * t9 * t24 - t2 * t6 * t13 * t24;
    t31 = t3 * t4 - t2 * t5 * t6;
    t34 = t4 * t6 + t2 * t3 * t5;
    t38 = t9 * t34 + t13 * t31;
    t40 = t9 * t31 - t13 * t34;
    t41 = t9 * t34 * 4.87226748312;
    t42 = t9 * t31 * 0.88455012048;
    t43 = t13 * t31 * 4.87226748312;
    t44 = sin(in1[5]);
    t45 = cos(in1[7]);
    t46 = cos(in1[5]);
    t47 = sin(in1[6]);
    t48 = sin(in1[7]);
    t50 = cos(in1[8] - 0.22689280275926282);
    t51 = cos(in1[8]);
    t54 = t44 * t45 + t46 * t47 * t48;
    t55 = sin(in1[8]);
    t57 = t44 * t48 - t45 * t46 * t47;
    t61 = t51 * t57 + t54 * t55;
    t62 = sin(in1[8] - 0.22689280275926282);
    t64 = t51 * t54 - t55 * t57;
    t65 = cos(in1[6]);
    t68 = t44 * t45 * t55 * t65 + t44 * t48 * t51 * t65;
    t70 = t44 * t45 * t51 * t65 - t44 * t48 * t55 * t65;
    t31 = t45 * t46 - t44 * t47 * t48;
    t75 = t46 * t48 + t44 * t45 * t47;
    t79 = t51 * t75 + t55 * t31;
    t81 = t51 * t31 - t55 * t75;
    t82 = t51 * t75 * 4.87226748312;
    t83 = t51 * t31 * 0.88455012048;
    t31 = t55 * t31 * 4.87226748312;
    G[0] = ((((((((((t2 * 7.90866504 + t2 * t3 * 2.66824152) + t9 * t12 *
                    4.87226748312) - t9 * t15 * 0.88455012048) - t12 * t13 *
                  0.88455012048) - t4 * t24 * 0.100059057) + t8 * t20 *
                0.05878152) - t13 * t15 * 4.87226748312) + t8 * t23 *
              0.599571504) + t20 * t21 * 0.599571504) - t21 * t23 * 0.05878152)
        + t4 * t5 * t6 * 2.66824152;
    G[1] = ((((((((t2 * t5 * 0.100059057 + t8 * t27 * 0.599571504) - t8 * t29 *
                  0.05878152) - t21 * t27 * 0.05878152) - t21 * t29 *
                0.599571504) + t2 * t6 * t24 * 2.66824152) + t2 * t3 * t9 * t24 *
              0.88455012048) + t2 * t6 * t9 * t24 * 4.87226748312) + t2 * t3 *
            t13 * t24 * 4.87226748312) - t2 * t6 * t13 * t24 * 0.88455012048;
    G[2] = ((((((((t41 + t42) + t43) + t4 * t6 * 2.66824152) + t8 * t38 *
                0.599571504) - t13 * t34 * 0.88455012048) - t8 * t40 *
              0.05878152) - t21 * t38 * 0.05878152) - t21 * t40 * 0.599571504) +
        t2 * t3 * t5 * 2.66824152;
    G[3] = ((t41 + t42) + t43) - t13 * t34 * 0.88455012048;
    G[4] = 0.0;
    G[5] = ((((((((((t44 * 7.90866504 + t44 * t45 * 2.66824152) + t51 * t54 *
                    4.87226748312) - t51 * t57 * 0.88455012048) - t54 * t55 *
                  0.88455012048) + t46 * t65 * 0.100059057) + t50 * t61 *
                0.05878152) - t55 * t57 * 4.87226748312) + t50 * t64 *
              0.599571504) + t61 * t62 * 0.599571504) - t62 * t64 * 0.05878152)
        + t46 * t47 * t48 * 2.66824152;
    G[6] = ((((((((t44 * t47 * -0.100059057 + t50 * t68 * 0.599571504) - t50 *
                  t70 * 0.05878152) - t62 * t68 * 0.05878152) - t62 * t70 *
                0.599571504) + t44 * t48 * t65 * 2.66824152) + t44 * t45 * t51 *
              t65 * 0.88455012048) + t44 * t48 * t51 * t65 * 4.87226748312) +
            t44 * t45 * t55 * t65 * 4.87226748312) - t44 * t48 * t55 * t65 *
        0.88455012048;
    G[7] = ((((((((t82 + t83) + t31) + t46 * t48 * 2.66824152) + t50 * t79 *
                0.599571504) - t55 * t75 * 0.88455012048) - t50 * t81 *
              0.05878152) - t62 * t79 * 0.05878152) - t62 * t81 * 0.599571504) +
        t44 * t45 * t47 * 2.66824152;
    G[8] = ((t82 + t83) + t31) - t55 * t75 * 0.88455012048;
    G[9] = 0.0;
}

static void generatedLeftFootJacobian(const double in1[5], const double in2[2],
    double J[42])
{
    double t2;
    double t3;
    double t4;
    double t5;
    double t6;
    double t7;
    double t8;
    double t9;
    double t12;
    double t13;
    double t14;
    double t17;
    double t15;
    double t19;
    double t20;
    double t22;
    double t23;
    double t24;
    double t27;
    double t28;
    double t29;
    double t30;
    double t31;
    double t32;
    double t35;
    double t37;
    double t43;
    double t47;
    double t49;
    double t50;
    double t51;
    double t52;
    double t54;
    double t55;
    double t56;
    double t57;
    double t58;
    double t60;
    double t63;
    double t66;
    double t69;
    double t71;
    double t77;
    double t72;
    double t75;
    double t78;
    double t81;
    double t84;
    double t86;
    double t90;
    double t92;
    double t96;
    double t98;
    double t101;
    double t103;
    double t106;
    double t109;
    double t111;
    double t117;
    double t112;
    double t115;
    double t118;
    double t119;
    double t120;
    double t121;
    double t122;
    double t123;
    double t125;
    double t126;
    double t127;
    double t128;
    double t131;
    double t133;
    double t136;
    double t139;
    double t143;
    double t145;
    double t147;
    double t148;
    double t149;
    double t150;
    double t151;
    double t152;
    double t153;
    double t154;
    double t155;
    double t156;
    double t157;
    double t164;
    double t165;
    double t166;
    double t167;
    double t158;
    double t171;
    double t172;
    double t159;
    double t175;
    double t176;
    double t177;
    double t178;
    double t160;
    double t179;
    double t180;
    double t181;
    double t182;
    double t161;
    double t162;
    double t193;
    double t280;
    double t205;
    double t215;
    double t219;
    double t223;
    double t227;
    double t232;
    double t236;
    double t244;
    double dv10[42];
    int i14;
    int i15;
    int i16;
    t2 = sin(in1[1]);
    t3 = cos(in1[2]);
    t4 = sin(in1[3]);
    t5 = cos(in1[3]);
    t6 = sin(in1[2]);
    t7 = ((in1[3] + in2[0]) - in2[1]) - 0.22689280275926282;
    t8 = cos(t7);
    t9 = cos(in2[0]);
    t12 = t2 * t3 * t4 + t2 * t5 * t6;
    t13 = sin(in2[0]);
    t14 = t2 * t3 * t5;
    t17 = t2 * t4 * t6;
    t15 = t14 - t17;
    t19 = sin(t7);
    t20 = t9 * t12;
    t22 = t20 + t13 * (t14 - t17);
    t23 = t9 * t15 - t12 * t13;
    t24 = cos(in1[4]);
    t27 = t8 * t22 - t19 * t23;
    t28 = sin(in1[4]);
    t29 = t8 * t23;
    t30 = t19 * t22;
    t31 = t29 + t30;
    t32 = cos(in1[1]);
    t35 = t3 * t4 * t32 + t5 * t6 * t32;
    t37 = t3 * t5 * t32 - t4 * t6 * t32;
    t7 = t9 * t35 + t13 * t37;
    t43 = t9 * t37 - t13 * t35;
    t47 = t8 * t43 + t19 * t7;
    t49 = t8 * t7 - t19 * t43;
    t50 = t9 * t37 * 0.43476;
    t51 = t3 * t5 * t32 * 0.06068;
    t52 = t24 * t47 * 0.01762;
    t54 = t8 * t7 * 0.04;
    t55 = t8 * t43 * 0.408;
    t56 = t19 * t7 * 0.408;
    t57 = cos(in1[0]);
    t58 = sin(in1[0]);
    t60 = t3 * t57 - t2 * t6 * t58;
    t63 = t6 * t57 + t2 * t3 * t58;
    t66 = t5 * t60 - t4 * t63;
    t69 = t5 * t63 + t4 * t60;
    t71 = t9 * t66;
    t77 = t13 * t69;
    t72 = t71 - t77;
    t75 = t13 * t66 + t9 * t69;
    t78 = t8 * t75 - t19 * t72;
    t7 = t8 * t72;
    t14 = t19 * t75;
    t81 = t7 + t14;
    t84 = t3 * t4 * t32 * t57 + t5 * t6 * t32 * t57;
    t86 = t3 * t5 * t32 * t57 - t4 * t6 * t32 * t57;
    t90 = t9 * t84 + t13 * t86;
    t92 = t9 * t86 - t13 * t84;
    t96 = t8 * t92 + t19 * t90;
    t98 = t8 * t90 - t19 * t92;
    t101 = t3 * t58 + t2 * t6 * t57;
    t103 = t6 * t58 - t2 * t3 * t57;
    t106 = t5 * t101 - t4 * t103;
    t109 = t5 * t103 + t4 * t101;
    t111 = t9 * t106;
    t117 = t13 * t109;
    t112 = t111 - t117;
    t115 = t13 * t106 + t9 * t109;
    t118 = t8 * t115 - t19 * t112;
    t119 = t8 * t112;
    t120 = t19 * t115;
    t121 = t119 + t120;
    t122 = t13 * t109 * 0.02;
    t123 = t4 * t103 * 0.04741;
    t125 = t28 * t118 * 0.05219;
    t126 = t8 * (t111 - t117) * 0.04;
    t127 = t19 * t115 * 0.04;
    t128 = t19 * (t111 - t117) * 0.408;
    t131 = t3 * t4 * t32 * t58 + t5 * t6 * t32 * t58;
    t133 = t3 * t5 * t32 * t58 - t4 * t6 * t32 * t58;
    t136 = t9 * t133 - t13 * t131;
    t139 = t9 * t131 + t13 * t133;
    t143 = t8 * t136 + t19 * t139;
    t145 = t8 * t139 - t19 * t136;
    t147 = t13 * t66 * 0.43476;
    t148 = t5 * t63 * 0.06068;
    t149 = t5 * t60 * 0.04741;
    t150 = t4 * t60 * 0.06068;
    t151 = t9 * t69 * 0.43476;
    t152 = t9 * t66 * 0.02;
    t153 = t24 * t78 * 0.01762;
    t154 = t24 * (t7 + t14) * 0.05219;
    t155 = t28 * (t7 + t14) * 0.01762;
    t156 = t8 * t75 * 0.408;
    t157 = (((-(t32 * t58) + t24 * t78 * 4.6906693763513648E-17) - t24 * t81 *
             3.9359389436709938E-17) + t28 * t78 * 3.9359389436709938E-17) + t28
        * (t7 + t14) * 4.6906693763513648E-17;
    t164 = t24 * t118 * 0.7660444431189779;
    t165 = t24 * t121 * 0.64278760968653947;
    t166 = t28 * t118 * 0.64278760968653947;
    t167 = t28 * (t119 + t120) * 0.7660444431189779;
    t158 = (((-(t32 * t57 * 6.123233995736766E-17) + t164) - t165) + t166) +
        t167;
    t17 = t24 * t78 * 0.7660444431189779;
    t171 = t24 * t81 * 0.64278760968653947;
    t172 = t28 * t78 * 0.64278760968653947;
    t7 = t28 * (t7 + t14) * 0.7660444431189779;
    t159 = (((t32 * t58 * 6.123233995736766E-17 + t17) - t171) + t172) + t7;
    t175 = t24 * t118 * 0.64278760968653947;
    t176 = t24 * t121 * 0.7660444431189779;
    t177 = t28 * t118 * 0.7660444431189779;
    t178 = t28 * t121 * 0.64278760968653947;
    t160 = ((t175 + t176) - t177) + t178;
    t179 = t24 * t78 * 0.64278760968653947;
    t180 = t24 * t81 * 0.7660444431189779;
    t181 = t28 * t78 * 0.7660444431189779;
    t182 = t28 * t81 * 0.64278760968653947;
    t161 = ((t179 + t180) - t181) + t182;
    t162 = (((t32 * t57 + t24 * t118 * 4.6906693763513648E-17) - t24 * t121 *
             3.9359389436709938E-17) + t28 * t118 * 3.9359389436709938E-17) +
        t28 * (t119 + t120) * 4.6906693763513648E-17;
    t193 = ((t17 - t171) + t172) + t7;
    t167 += (t164 - t165) + t166;
    t172 = ((t24 * t78 * 3.9359389436709938E-17 + t24 * t81 *
             4.6906693763513648E-17) + t28 * t81 * 3.9359389436709938E-17) - t28
        * t78 * 4.6906693763513648E-17;
    t280 = ((t24 * t118 * 3.9359389436709938E-17 + t24 * t121 *
             4.6906693763513648E-17) + t28 * t121 * 3.9359389436709938E-17) -
        t28 * t118 * 4.6906693763513648E-17;
    t205 = ((((t193 * (((t175 + t176) - t177) + t178) * 0.5 + t157 * t280 * 0.5)
              + t159 * (((t175 + t176) - t177) + t178) * 0.5) - t161 * t167 *
             0.5) - t162 * t172 * 0.5) - t158 * t161 * 0.5;
    t7 = t24 * t47 * 0.7660444431189779;
    t14 = t24 * t49 * 0.64278760968653947;
    t17 = t28 * t47 * 0.64278760968653947;
    t171 = t28 * t49 * 0.7660444431189779;
    t215 = (((t7 + t14) + t17) - t2 * 6.123233995736766E-17) - t171;
    t219 = (((t24 * t143 * 0.7660444431189779 + t24 * t145 * 0.64278760968653947)
             + t28 * t143 * 0.64278760968653947) - t2 * t58 *
            6.123233995736766E-17) - t28 * t145 * 0.7660444431189779;
    t223 = ((t24 * t145 * 0.7660444431189779 + t28 * t143 * 0.7660444431189779)
            + t28 * t145 * 0.64278760968653947) - t24 * t143 *
        0.64278760968653947;
    t227 = ((t24 * t49 * 0.7660444431189779 + t28 * t47 * 0.7660444431189779) +
            t28 * t49 * 0.64278760968653947) - t24 * t47 * 0.64278760968653947;
    t232 = (((t2 * t58 + t24 * t143 * 4.6906693763513648E-17) + t24 * t145 *
             3.9359389436709938E-17) + t28 * t143 * 3.9359389436709938E-17) -
        t28 * t145 * 4.6906693763513648E-17;
    t236 = (((t2 + t24 * t47 * 4.6906693763513648E-17) + t24 * t49 *
             3.9359389436709938E-17) + t28 * t47 * 3.9359389436709938E-17) - t28
        * t49 * 4.6906693763513648E-17;
    t244 = ((t24 * t49 * 4.6906693763513648E-17 + t28 * t47 *
             4.6906693763513648E-17) + t28 * t49 * 3.9359389436709938E-17) - t24
        * t47 * 3.9359389436709938E-17;
    t7 = ((t7 + t14) + t17) - t171;
    t172 = ((((-(t157 * t244 * 0.5) - t159 * t227 * 0.5) - t161 * t215 * 0.5) -
             t172 * t236 * 0.5) - t161 * t7 * 0.5) - t193 * t227 * 0.5;
    t164 = (((t32 * 6.123233995736766E-17 + t24 * t27 * 0.64278760968653947) +
             t24 * t31 * 0.7660444431189779) + t28 * t31 * 0.64278760968653947)
        - t27 * t28 * 0.7660444431189779;
    t165 = (((t28 * t96 * 0.64278760968653947 + t24 * t96 * 0.7660444431189779)
             + t24 * t98 * 0.64278760968653947) - t2 * t57 *
            6.123233995736766E-17) - t28 * t98 * 0.7660444431189779;
    t166 = (((-t32 + t24 * t27 * 3.9359389436709938E-17) + t24 * t31 *
             4.6906693763513648E-17) + t28 * t31 * 3.9359389436709938E-17) - t27
        * t28 * 4.6906693763513648E-17;
    t14 = ((t24 * t27 * 0.7660444431189779 + t27 * t28 * 0.64278760968653947) +
           t28 * (t29 + t30) * 0.7660444431189779) - t24 * t31 *
        0.64278760968653947;
    t17 = ((t28 * t96 * 0.7660444431189779 + t28 * t98 * 0.64278760968653947) +
           t24 * t98 * 0.7660444431189779) - t24 * t96 * 0.64278760968653947;
    t171 = (((t28 * t96 * 3.9359389436709938E-17 + t2 * t57) + t24 * t96 *
             4.6906693763513648E-17) + t24 * t98 * 3.9359389436709938E-17) - t28
        * t98 * 4.6906693763513648E-17;
    t7 = ((((-(t158 * t227 * 0.5) - t160 * t215 * 0.5) - t236 * t280 * 0.5) -
           t160 * t7 * 0.5) - t167 * t227 * 0.5) - t162 * t244 * 0.5;
    dv10[0] = 0.0;
    dv10[1] = ((((((((((((((((((t57 * 0.09 + t3 * t57 * 0.12) - t4 * t60 *
        0.04741) + t5 * t60 * 0.06068) - t4 * t63 * 0.06068) - t5 * t63 *
                            0.04741) + t9 * t66 * 0.43476) - t9 * t69 * 0.02) -
                         t13 * t66 * 0.02) - t13 * t69 * 0.43476) + t8 * t75 *
                       0.04) + t32 * t58 * 0.0045) - t19 * t72 * 0.04) + t19 *
                    t75 * 0.408) - t24 * t78 * 0.05219) + t24 * t81 * 0.01762) -
                 t28 * t78 * 0.01762) - t28 * t81 * 0.05219) + t8 * (t71 - t77) *
               0.408) - t2 * t6 * t58 * 0.12;
    dv10[2] = ((((((((((((((((((t58 * 0.09 + t24 * (t119 + t120) * 0.01762) + t3
        * t58 * 0.12) - t32 * t57 * 0.0045) - t4 * t101 * 0.04741) + t5 * t101 *
                            0.06068) - t4 * t103 * 0.06068) - t5 * t103 *
                          0.04741) + t9 * t106 * 0.43476) - t9 * t109 * 0.02) -
                       t13 * t106 * 0.02) - t13 * t109 * 0.43476) + t8 * t115 *
                     0.04) - t19 * t112 * 0.04) + t19 * t115 * 0.408) - t24 *
                  t118 * 0.05219) - t28 * t118 * 0.01762) - t28 * t121 * 0.05219)
               + t8 * (t111 - t117) * 0.408) + t2 * t6 * t57 * 0.12;
    dv10[3] = ((((t157 * t157 * 0.5 + t158 * t158 * 0.5) + t159 * t159 * 0.5) +
                t160 * t160 * 0.5) + t161 * t161 * 0.5) + t162 * t162 * 0.5;
    dv10[4] = (t158 * t215 * 0.5 - t160 * t227 * 0.5) + t162 * t236 * 0.5;
    dv10[5] = (t159 * t215 * -0.5 - t157 * t236 * 0.5) + t227 * (((t179 + t180)
        - t181) + t182) * 0.5;
    dv10[6] = ((((((((((((((((t32 * 0.0045 - t2 * t6 * 0.12) - t9 * t12 *
        0.43476) - t9 * t15 * 0.02) + t12 * t13 * 0.02) - t13 * t15 * 0.43476) +
                         t8 * t23 * 0.04) + t19 * t22 * 0.04) + t19 * t23 *
                       0.408) - t24 * t27 * 0.01762) - t24 * t31 * 0.05219) +
                    t27 * t28 * 0.05219) - t28 * t31 * 0.01762) - t8 * (t20 +
                   t13 * t15) * 0.408) - t2 * t3 * t4 * 0.06068) - t2 * t3 * t5 *
                0.04741) + t2 * t4 * t6 * 0.04741) - t2 * t5 * t6 * 0.06068;
    dv10[7] = ((((((((((((((((t2 * t57 * 0.0045 + t9 * t84 * 0.43476) + t9 * t86
        * 0.02) - t13 * t84 * 0.02) + t8 * t90 * 0.408) + t13 * t86 * 0.43476) -
                         t8 * t92 * 0.04) - t19 * t90 * 0.04) - t19 * t92 *
                       0.408) + t24 * t96 * 0.05219) + t24 * t98 * 0.01762) +
                    t28 * t96 * 0.01762) - t28 * t98 * 0.05219) + t6 * t32 * t57
                  * 0.12) + t3 * t4 * t32 * t57 * 0.06068) + t3 * t5 * t32 * t57
                * 0.04741) - t4 * t6 * t32 * t57 * 0.04741) + t5 * t6 * t32 *
        t57 * 0.06068;
    dv10[8] = ((((((((((((((((t2 * t58 * 0.0045 + t9 * t131 * 0.43476) + t9 *
        t133 * 0.02) - t8 * t136 * 0.04) - t13 * t131 * 0.02) + t13 * t133 *
                          0.43476) + t8 * t139 * 0.408) - t19 * t136 * 0.408) -
                       t19 * t139 * 0.04) + t24 * t143 * 0.05219) + t24 * t145 *
                     0.01762) + t28 * t143 * 0.01762) - t28 * t145 * 0.05219) +
                  t6 * t32 * t58 * 0.12) + t3 * t4 * t32 * t58 * 0.06068) + t3 *
                t5 * t32 * t58 * 0.04741) - t4 * t6 * t32 * t58 * 0.04741) + t5 *
        t6 * t32 * t58 * 0.06068;
    dv10[9] = ((((t158 * t219 * -0.5 - t162 * t232 * 0.5) - t159 * t165 * 0.5) -
                t157 * t171 * 0.5) + t223 * (((t175 + t176) - t177) + t178) *
               0.5) + t17 * (((t179 + t180) - t181) + t182) * 0.5;
    dv10[10] = ((((t159 * t164 * -0.5 - t157 * t166 * 0.5) - t215 * t219 * 0.5)
                 - t223 * t227 * 0.5) - t232 * t236 * 0.5) + t14 * (((t179 +
        t180) - t181) + t182) * 0.5;
    dv10[11] = ((((t158 * t164 * -0.5 - t162 * t166 * 0.5) + t215 * t165 * 0.5)
                 + t227 * t17 * 0.5) + t236 * t171 * 0.5) + t14 * (((t175 + t176)
        - t177) + t178) * 0.5;
    dv10[12] = (((((((((((((((t50 + t51) + t52) + t54) + t55) + t56) + t3 * t32 *
                         0.12) - t9 * t35 * 0.02) - t13 * t35 * 0.43476) - t13 *
                      t37 * 0.02) - t19 * t43 * 0.04) - t24 * t49 * 0.05219) -
                   t28 * t47 * 0.05219) - t28 * t49 * 0.01762) - t3 * t4 * t32 *
                 0.04741) - t4 * t6 * t32 * 0.06068) - t5 * t6 * t32 * 0.04741;
    dv10[13] = ((((((((((((((((t122 + t123) + t125) + t126) + t127) + t128) - t6
                          * t58 * 0.12) - t4 * t101 * 0.06068) - t5 * t101 *
                        0.04741) - t5 * t103 * 0.06068) - t9 * t106 * 0.02) - t9
                     * t109 * 0.43476) - t13 * t106 * 0.43476) - t8 * t115 *
                   0.408) - t24 * t118 * 0.01762) - t24 * t121 * 0.05219) - t28 *
                t121 * 0.01762) + t2 * t3 * t57 * 0.12;
    dv10[14] = ((((((((((((((((t147 + t148) + t149) + t150) + t151) + t152) +
                          t153) + t154) + t155) + t156) + t6 * t57 * 0.12) - t4 *
                     t63 * 0.04741) - t8 * t72 * 0.04) - t13 * t69 * 0.02) - t19
                  * t72 * 0.408) - t19 * t75 * 0.04) - t28 * t78 * 0.05219) + t2
        * t3 * t58 * 0.12;
    dv10[15] = t205;
    dv10[16] = t172;
    dv10[17] = t7;
    dv10[18] = ((((((t50 + t51) - t9 * t35 * 0.02) - t13 * t35 * 0.43476) - t13 *
                  t37 * 0.02) - t3 * t4 * t32 * 0.04741) - t4 * t6 * t32 *
                0.06068) - t5 * t6 * t32 * 0.04741;
    dv10[19] = ((((((t122 + t123) - t4 * t101 * 0.06068) - t5 * t101 * 0.04741)
                  - t5 * t103 * 0.06068) - t9 * t106 * 0.02) - t9 * t109 *
                0.43476) - t13 * t106 * 0.43476;
    dv10[20] = ((((((t147 + t148) + t149) + t150) + t151) + t152) - t4 * t63 *
                0.04741) - t13 * t69 * 0.02;
    dv10[21] = 0.0;
    dv10[22] = 0.0;
    dv10[23] = 0.0;
    dv10[24] = ((t52 - t24 * t49 * 0.05219) - t28 * t47 * 0.05219) - t28 * t49 *
        0.01762;
    dv10[25] = ((t125 - t24 * t118 * 0.01762) - t24 * t121 * 0.05219) - t28 *
        t121 * 0.01762;
    dv10[26] = ((t153 + t154) + t155) - t28 * t78 * 0.05219;
    dv10[27] = t205;
    dv10[28] = t172;
    dv10[29] = t7;
    dv10[30] = ((t50 - t9 * t35 * 0.02) - t13 * t35 * 0.43476) - t13 * t37 *
        0.02;
    dv10[31] = ((t122 - t9 * t106 * 0.02) - t9 * t109 * 0.43476) - t13 * t106 *
        0.43476;
    dv10[32] = ((t147 + t151) + t152) - t13 * t69 * 0.02;
    dv10[33] = 0.0;
    dv10[34] = 0.0;
    dv10[35] = 0.0;
    dv10[36] = ((((((t52 + t54) + t55) + t56) - t19 * t43 * 0.04) - t24 * t49 *
                 0.05219) - t28 * t47 * 0.05219) - t28 * t49 * 0.01762;
    dv10[37] = ((((((t125 + t126) + t127) + t128) - t8 * t115 * 0.408) - t24 *
                 t118 * 0.01762) - t24 * t121 * 0.05219) - t28 * t121 * 0.01762;
    dv10[38] = ((((((t153 + t154) + t155) + t156) - t8 * t72 * 0.04) - t19 * t72
                 * 0.408) - t19 * t75 * 0.04) - t28 * t78 * 0.05219;
    dv10[39] = t205;
    dv10[40] = t172;
    dv10[41] = t7;
    i14 = 0;
    for (i15 = 0; i15 < 7; i15++) {
        for (i16 = 0; i16 < 6; i16++) {
            J[i16 + i14] = dv10[i16 + i14];
        }

        i14 += 6;
    }
}

static void generatedLeftFootTransform(const double in1[5], const double in2[2],
    double T[16])
{
    double t2;
    double t3;
    double t4;
    double t5;
    double t6;
    double t7;
    double t8;
    double t11;
    double t12;
    double t14;
    double t15;
    double t17;
    double t21;
    double t22;
    double t24;
    double t28;
    double t29;
    double t31;
    double t33;
    double t34;
    double t35;
    double t37;
    double t40;
    double t44;
    double t45;
    double t48;
    double t46;
    double t47;
    double t50;
    double t52;
    double t54;
    double t56;
    double t58;
    double t62;
    double t64;
    double t68;
    double t69;
    double t70;
    double t73;
    double t74;
    double t75;
    double t76;
    double t77;
    double t83;
    double t80;
    double t82;
    double t89;
    double t86;
    double b_t15[16];
    int i11;
    int i12;
    int i13;
    t2 = cos(in1[1]);
    t3 = cos(in1[2]);
    t4 = sin(in1[3]);
    t5 = cos(in1[3]);
    t6 = sin(in1[2]);
    t7 = ((in1[3] + in2[0]) - in2[1]) - 0.22689280275926282;
    t8 = cos(in2[0]);
    t11 = t2 * t3 * t4 + t2 * t5 * t6;
    t12 = sin(in2[0]);
    t14 = t2 * t3 * t5 - t2 * t4 * t6;
    t15 = cos(in1[4]);
    t17 = cos(t7);
    t21 = t8 * t11 + t12 * t14;
    t22 = sin(t7);
    t24 = t8 * t14 - t11 * t12;
    t28 = t17 * t24 + t21 * t22;
    t29 = sin(in1[4]);
    t31 = t17 * t21 - t22 * t24;
    t33 = sin(in1[1]);
    t34 = sin(in1[0]);
    t35 = cos(in1[0]);
    t37 = t6 * t34 - t3 * t33 * t35;
    t40 = t3 * t34 + t6 * t33 * t35;
    t44 = t5 * t37 + t4 * t40;
    t45 = t4 * t37;
    t48 = t5 * t40;
    t46 = t45 - t48;
    t47 = t12 * t44;
    t50 = t8 * t44 - t12 * t46;
    t52 = t17 * t50;
    t54 = t47 + t8 * (t45 - t48);
    t56 = t17 * t54 - t22 * t50;
    t58 = t52 + t22 * t54;
    t62 = t6 * t35 + t3 * t33 * t34;
    t64 = t3 * t35 - t6 * t33 * t34;
    t68 = t5 * t62 + t4 * t64;
    t69 = t4 * t62;
    t70 = t12 * t68;
    t73 = t5 * t64;
    t74 = t69 - t73;
    t7 = t70 + t8 * t74;
    t75 = t8 * t68;
    t76 = t75 - t12 * t74;
    t77 = t22 * t7;
    t83 = t22 * t76;
    t80 = t17 * t7 - t83;
    t7 = t17 * t76;
    t82 = t77 + t7;
    t70 += t8 * (t69 - t73);
    t89 = t17 * t70;
    t86 = t83 - t89;
    t7 += t22 * t70;
    b_t15[0] = ((t15 * t28 * 0.64278760968653947 - t15 * t31 *
                 0.7660444431189779) - t28 * t29 * 0.7660444431189779) - t29 *
        t31 * 0.64278760968653947;
    b_t15[1] = ((t15 * t56 * 0.7660444431189779 + t29 * t56 *
                 0.64278760968653947) + t29 * t58 * 0.7660444431189779) - t15 *
        (t52 + t22 * (t47 + t8 * t46)) * 0.64278760968653947;
    b_t15[2] = ((t15 * (t77 + t17 * (t75 - t12 * (t69 - t5 * t64))) *
                 0.64278760968653947 - t15 * t80 * 0.7660444431189779) - t29 *
                t80 * 0.64278760968653947) - t29 * t82 * 0.7660444431189779;
    b_t15[3] = 0.0;
    b_t15[4] = (((-t33 - t15 * t28 * 4.6906693763513648E-17) - t15 * t31 *
                 3.9359389436709938E-17) - t28 * t29 * 3.9359389436709938E-17) +
        t29 * t31 * 4.6906693763513648E-17;
    b_t15[5] = (((t2 * t35 + t15 * t56 * 3.9359389436709938E-17) + t15 * t58 *
                 4.6906693763513648E-17) - t29 * t56 * 4.6906693763513648E-17) +
        t29 * t58 * 3.9359389436709938E-17;
    b_t15[6] = (((t2 * t34 - t15 * t82 * 4.6906693763513648E-17) + t15 * t86 *
                 3.9359389436709938E-17) - t29 * t86 * 4.6906693763513648E-17) -
        t29 * t7 * 3.9359389436709938E-17;
    b_t15[7] = 0.0;
    b_t15[8] = (((t33 * 6.123233995736766E-17 - t15 * t28 * 0.7660444431189779)
                 - t15 * t31 * 0.64278760968653947) - t28 * t29 *
                0.64278760968653947) + t29 * t31 * 0.7660444431189779;
    b_t15[9] = (((t2 * t35 * -6.123233995736766E-17 + t15 * t56 *
                  0.64278760968653947) + t15 * t58 * 0.7660444431189779) - t29 *
                t56 * 0.7660444431189779) + t29 * t58 * 0.64278760968653947;
    b_t15[10] = (((t2 * t34 * -6.123233995736766E-17 - t15 * t7 *
                   0.7660444431189779) - t29 * t86 * 0.7660444431189779) - t29 *
                 t7 * 0.64278760968653947) + t15 * (t83 - t89) *
        0.64278760968653947;
    b_t15[11] = 0.0;
    b_t15[12] = (((((((((((((((((t33 * 0.0045 + t2 * t6 * 0.12) + t8 * t11 *
        0.43476) + t8 * t14 * 0.02) - t11 * t12 * 0.02) + t12 * t14 * 0.43476) +
                            t17 * t21 * 0.408) - t17 * t24 * 0.04) + t15 * t28 *
                          0.05219) - t21 * t22 * 0.04) + t15 * t31 * 0.01762) -
                       t22 * t24 * 0.408) + t28 * t29 * 0.01762) - t29 * t31 *
                     0.05219) + t2 * t3 * t4 * 0.06068) + t2 * t3 * t5 * 0.04741)
                  - t2 * t4 * t6 * 0.04741) + t2 * t5 * t6 * 0.06068) - 0.049;
    b_t15[13] = (((((((((((((((((((t34 * 0.09 - t2 * t35 * 0.0045) + t3 * t34 *
        0.12) - t4 * t37 * 0.06068) - t5 * t37 * 0.04741) - t4 * t40 * 0.04741)
        + t5 * t40 * 0.06068) - t8 * t44 * 0.02) - t8 * t46 * 0.43476) - t12 *
                           t44 * 0.43476) + t17 * t50 * 0.04) - t15 * t56 *
                         0.01762) - t17 * t54 * 0.408) + t22 * t50 * 0.408) -
                      t15 * t58 * 0.05219) + t22 * t54 * 0.04) + t29 * t56 *
                    0.05219) - t29 * t58 * 0.01762) + t12 * (t45 - t48) * 0.02)
                 + t6 * t33 * t35 * 0.12) + 0.135;
    b_t15[14] = ((((((((((((((((((t35 * -0.09 - t2 * t34 * 0.0045) - t3 * t35 *
        0.12) + t4 * t62 * 0.06068) + t5 * t62 * 0.04741) + t4 * t64 * 0.04741)
        - t5 * t64 * 0.06068) + t8 * t68 * 0.02) + t12 * t68 * 0.43476) - t12 *
                          t74 * 0.02) - t17 * t76 * 0.04) - t22 * t76 * 0.408) -
                       t15 * t86 * 0.01762) + t17 * t70 * 0.408) + t15 * t7 *
                     0.05219) - t22 * t70 * 0.04) + t29 * t7 * 0.01762) + t8 *
                  (t69 - t73) * 0.43476) + t29 * (t83 - t89) * 0.05219) + t6 *
        t33 * t34 * 0.12;
    b_t15[15] = 1.0;
    i11 = 0;
    for (i12 = 0; i12 < 4; i12++) {
        for (i13 = 0; i13 < 4; i13++) {
            T[i13 + i11] = b_t15[i13 + i11];
        }

        i11 += 4;
    }
}

static void generatedRightFootJacobian(const double in1[5], const double in2[2],
    double J[42])
{
    double t2;
    double t3;
    double t4;
    double t5;
    double t6;
    double t7;
    double t8;
    double t9;
    double t12;
    double t13;
    double t14;
    double t17;
    double t15;
    double t19;
    double t20;
    double t22;
    double t23;
    double t24;
    double t27;
    double t28;
    double t29;
    double t30;
    double t31;
    double t32;
    double t35;
    double t37;
    double t43;
    double t47;
    double t49;
    double t50;
    double t51;
    double t52;
    double t54;
    double t55;
    double t56;
    double t57;
    double t58;
    double t60;
    double t63;
    double t66;
    double t69;
    double t71;
    double t77;
    double t72;
    double t75;
    double t78;
    double t81;
    double t84;
    double t86;
    double t90;
    double t92;
    double t96;
    double t98;
    double t101;
    double t103;
    double t106;
    double t109;
    double t111;
    double t117;
    double t112;
    double t115;
    double t118;
    double t119;
    double t120;
    double t121;
    double t122;
    double t123;
    double t125;
    double t126;
    double t127;
    double t128;
    double t131;
    double t133;
    double t136;
    double t139;
    double t143;
    double t145;
    double t147;
    double t148;
    double t149;
    double t150;
    double t151;
    double t152;
    double t153;
    double t154;
    double t155;
    double t156;
    double t157;
    double t164;
    double t165;
    double t166;
    double t167;
    double t158;
    double t171;
    double t172;
    double t159;
    double t175;
    double t176;
    double t177;
    double t178;
    double t160;
    double t179;
    double t180;
    double t181;
    double t182;
    double t161;
    double t162;
    double t193;
    double t280;
    double t205;
    double t215;
    double t219;
    double t223;
    double t227;
    double t232;
    double t236;
    double t244;
    double dv11[42];
    int i17;
    int i18;
    int i19;
    t2 = sin(in1[1]);
    t3 = cos(in1[2]);
    t4 = sin(in1[3]);
    t5 = cos(in1[3]);
    t6 = sin(in1[2]);
    t7 = ((in1[3] + in2[0]) - in2[1]) - 0.22689280275926282;
    t8 = cos(t7);
    t9 = cos(in2[0]);
    t12 = t2 * t3 * t4 + t2 * t5 * t6;
    t13 = sin(in2[0]);
    t14 = t2 * t3 * t5;
    t17 = t2 * t4 * t6;
    t15 = t14 - t17;
    t19 = sin(t7);
    t20 = t9 * t12;
    t22 = t20 + t13 * (t14 - t17);
    t23 = t9 * t15 - t12 * t13;
    t24 = cos(in1[4]);
    t27 = t8 * t22 - t19 * t23;
    t28 = sin(in1[4]);
    t29 = t8 * t23;
    t30 = t19 * t22;
    t31 = t29 + t30;
    t32 = cos(in1[1]);
    t35 = t3 * t4 * t32 + t5 * t6 * t32;
    t37 = t3 * t5 * t32 - t4 * t6 * t32;
    t7 = t9 * t35 + t13 * t37;
    t43 = t9 * t37 - t13 * t35;
    t47 = t8 * t43 + t19 * t7;
    t49 = t8 * t7 - t19 * t43;
    t50 = t9 * t37 * 0.43476;
    t51 = t3 * t5 * t32 * 0.06068;
    t52 = t24 * t47 * 0.01762;
    t54 = t8 * t7 * 0.04;
    t55 = t8 * t43 * 0.408;
    t56 = t19 * t7 * 0.408;
    t57 = cos(in1[0]);
    t58 = sin(in1[0]);
    t60 = t3 * t57 - t2 * t6 * t58;
    t63 = t6 * t57 + t2 * t3 * t58;
    t66 = t5 * t60 - t4 * t63;
    t69 = t5 * t63 + t4 * t60;
    t71 = t9 * t66;
    t77 = t13 * t69;
    t72 = t71 - t77;
    t75 = t13 * t66 + t9 * t69;
    t78 = t8 * t75 - t19 * t72;
    t7 = t8 * t72;
    t14 = t19 * t75;
    t81 = t7 + t14;
    t84 = t3 * t4 * t32 * t57 + t5 * t6 * t32 * t57;
    t86 = t3 * t5 * t32 * t57 - t4 * t6 * t32 * t57;
    t90 = t9 * t84 + t13 * t86;
    t92 = t9 * t86 - t13 * t84;
    t96 = t8 * t92 + t19 * t90;
    t98 = t8 * t90 - t19 * t92;
    t101 = t3 * t58 + t2 * t6 * t57;
    t103 = t6 * t58 - t2 * t3 * t57;
    t106 = t5 * t101 - t4 * t103;
    t109 = t5 * t103 + t4 * t101;
    t111 = t9 * t106;
    t117 = t13 * t109;
    t112 = t111 - t117;
    t115 = t13 * t106 + t9 * t109;
    t118 = t8 * t115 - t19 * t112;
    t119 = t8 * t112;
    t120 = t19 * t115;
    t121 = t119 + t120;
    t122 = t13 * t109 * 0.02;
    t123 = t4 * t103 * 0.04741;
    t125 = t28 * t118 * 0.05219;
    t126 = t8 * (t111 - t117) * 0.04;
    t127 = t19 * t115 * 0.04;
    t128 = t19 * (t111 - t117) * 0.408;
    t131 = t3 * t4 * t32 * t58 + t5 * t6 * t32 * t58;
    t133 = t3 * t5 * t32 * t58 - t4 * t6 * t32 * t58;
    t136 = t9 * t133 - t13 * t131;
    t139 = t9 * t131 + t13 * t133;
    t143 = t8 * t136 + t19 * t139;
    t145 = t8 * t139 - t19 * t136;
    t147 = t13 * t66 * 0.43476;
    t148 = t5 * t63 * 0.06068;
    t149 = t5 * t60 * 0.04741;
    t150 = t4 * t60 * 0.06068;
    t151 = t9 * t69 * 0.43476;
    t152 = t9 * t66 * 0.02;
    t153 = t24 * t78 * 0.01762;
    t154 = t24 * (t7 + t14) * 0.05219;
    t155 = t28 * (t7 + t14) * 0.01762;
    t156 = t8 * t75 * 0.408;
    t157 = (((-(t32 * t58) + t24 * t78 * 4.6906693763513648E-17) - t24 * t81 *
             3.9359389436709938E-17) + t28 * t78 * 3.9359389436709938E-17) + t28
        * (t7 + t14) * 4.6906693763513648E-17;
    t164 = t24 * t118 * 0.7660444431189779;
    t165 = t24 * t121 * 0.64278760968653947;
    t166 = t28 * t118 * 0.64278760968653947;
    t167 = t28 * (t119 + t120) * 0.7660444431189779;
    t158 = (((-(t32 * t57 * 6.123233995736766E-17) + t164) - t165) + t166) +
        t167;
    t17 = t24 * t78 * 0.7660444431189779;
    t171 = t24 * t81 * 0.64278760968653947;
    t172 = t28 * t78 * 0.64278760968653947;
    t7 = t28 * (t7 + t14) * 0.7660444431189779;
    t159 = (((t32 * t58 * 6.123233995736766E-17 + t17) - t171) + t172) + t7;
    t175 = t24 * t118 * 0.64278760968653947;
    t176 = t24 * t121 * 0.7660444431189779;
    t177 = t28 * t118 * 0.7660444431189779;
    t178 = t28 * t121 * 0.64278760968653947;
    t160 = ((t175 + t176) - t177) + t178;
    t179 = t24 * t78 * 0.64278760968653947;
    t180 = t24 * t81 * 0.7660444431189779;
    t181 = t28 * t78 * 0.7660444431189779;
    t182 = t28 * t81 * 0.64278760968653947;
    t161 = ((t179 + t180) - t181) + t182;
    t162 = (((t32 * t57 + t24 * t118 * 4.6906693763513648E-17) - t24 * t121 *
             3.9359389436709938E-17) + t28 * t118 * 3.9359389436709938E-17) +
        t28 * (t119 + t120) * 4.6906693763513648E-17;
    t193 = ((t17 - t171) + t172) + t7;
    t167 += (t164 - t165) + t166;
    t172 = ((t24 * t78 * 3.9359389436709938E-17 + t24 * t81 *
             4.6906693763513648E-17) + t28 * t81 * 3.9359389436709938E-17) - t28
        * t78 * 4.6906693763513648E-17;
    t280 = ((t24 * t118 * 3.9359389436709938E-17 + t24 * t121 *
             4.6906693763513648E-17) + t28 * t121 * 3.9359389436709938E-17) -
        t28 * t118 * 4.6906693763513648E-17;
    t205 = ((((t193 * (((t175 + t176) - t177) + t178) * 0.5 + t157 * t280 * 0.5)
              + t159 * (((t175 + t176) - t177) + t178) * 0.5) - t161 * t167 *
             0.5) - t162 * t172 * 0.5) - t158 * t161 * 0.5;
    t7 = t24 * t47 * 0.7660444431189779;
    t14 = t24 * t49 * 0.64278760968653947;
    t17 = t28 * t47 * 0.64278760968653947;
    t171 = t28 * t49 * 0.7660444431189779;
    t215 = (((t7 + t14) + t17) - t2 * 6.123233995736766E-17) - t171;
    t219 = (((t24 * t143 * 0.7660444431189779 + t24 * t145 * 0.64278760968653947)
             + t28 * t143 * 0.64278760968653947) - t2 * t58 *
            6.123233995736766E-17) - t28 * t145 * 0.7660444431189779;
    t223 = ((t24 * t145 * 0.7660444431189779 + t28 * t143 * 0.7660444431189779)
            + t28 * t145 * 0.64278760968653947) - t24 * t143 *
        0.64278760968653947;
    t227 = ((t24 * t49 * 0.7660444431189779 + t28 * t47 * 0.7660444431189779) +
            t28 * t49 * 0.64278760968653947) - t24 * t47 * 0.64278760968653947;
    t232 = (((t2 * t58 + t24 * t143 * 4.6906693763513648E-17) + t24 * t145 *
             3.9359389436709938E-17) + t28 * t143 * 3.9359389436709938E-17) -
        t28 * t145 * 4.6906693763513648E-17;
    t236 = (((t2 + t24 * t47 * 4.6906693763513648E-17) + t24 * t49 *
             3.9359389436709938E-17) + t28 * t47 * 3.9359389436709938E-17) - t28
        * t49 * 4.6906693763513648E-17;
    t244 = ((t24 * t49 * 4.6906693763513648E-17 + t28 * t47 *
             4.6906693763513648E-17) + t28 * t49 * 3.9359389436709938E-17) - t24
        * t47 * 3.9359389436709938E-17;
    t7 = ((t7 + t14) + t17) - t171;
    t172 = ((((-(t157 * t244 * 0.5) - t159 * t227 * 0.5) - t161 * t215 * 0.5) -
             t172 * t236 * 0.5) - t161 * t7 * 0.5) - t193 * t227 * 0.5;
    t164 = (((t32 * 6.123233995736766E-17 + t24 * t27 * 0.64278760968653947) +
             t24 * t31 * 0.7660444431189779) + t28 * t31 * 0.64278760968653947)
        - t27 * t28 * 0.7660444431189779;
    t165 = (((t28 * t96 * 0.64278760968653947 + t24 * t96 * 0.7660444431189779)
             + t24 * t98 * 0.64278760968653947) - t2 * t57 *
            6.123233995736766E-17) - t28 * t98 * 0.7660444431189779;
    t166 = (((-t32 + t24 * t27 * 3.9359389436709938E-17) + t24 * t31 *
             4.6906693763513648E-17) + t28 * t31 * 3.9359389436709938E-17) - t27
        * t28 * 4.6906693763513648E-17;
    t14 = ((t24 * t27 * 0.7660444431189779 + t27 * t28 * 0.64278760968653947) +
           t28 * (t29 + t30) * 0.7660444431189779) - t24 * t31 *
        0.64278760968653947;
    t17 = ((t28 * t96 * 0.7660444431189779 + t28 * t98 * 0.64278760968653947) +
           t24 * t98 * 0.7660444431189779) - t24 * t96 * 0.64278760968653947;
    t171 = (((t28 * t96 * 3.9359389436709938E-17 + t2 * t57) + t24 * t96 *
             4.6906693763513648E-17) + t24 * t98 * 3.9359389436709938E-17) - t28
        * t98 * 4.6906693763513648E-17;
    t7 = ((((-(t158 * t227 * 0.5) - t160 * t215 * 0.5) - t236 * t280 * 0.5) -
           t160 * t7 * 0.5) - t167 * t227 * 0.5) - t162 * t244 * 0.5;
    dv11[0] = 0.0;
    dv11[1] = ((((((((((((((((((t57 * 0.09 + t3 * t57 * 0.12) - t4 * t60 *
        0.04741) + t5 * t60 * 0.06068) - t4 * t63 * 0.06068) - t5 * t63 *
                            0.04741) + t9 * t66 * 0.43476) - t9 * t69 * 0.02) -
                         t13 * t66 * 0.02) - t13 * t69 * 0.43476) + t8 * t75 *
                       0.04) - t32 * t58 * 0.0045) - t19 * t72 * 0.04) + t19 *
                    t75 * 0.408) - t24 * t78 * 0.05219) + t24 * t81 * 0.01762) -
                 t28 * t78 * 0.01762) - t28 * t81 * 0.05219) + t8 * (t71 - t77) *
               0.408) - t2 * t6 * t58 * 0.12;
    dv11[2] = ((((((((((((((((((t58 * 0.09 + t24 * (t119 + t120) * 0.01762) + t3
        * t58 * 0.12) + t32 * t57 * 0.0045) - t4 * t101 * 0.04741) + t5 * t101 *
                            0.06068) - t4 * t103 * 0.06068) - t5 * t103 *
                          0.04741) + t9 * t106 * 0.43476) - t9 * t109 * 0.02) -
                       t13 * t106 * 0.02) - t13 * t109 * 0.43476) + t8 * t115 *
                     0.04) - t19 * t112 * 0.04) + t19 * t115 * 0.408) - t24 *
                  t118 * 0.05219) - t28 * t118 * 0.01762) - t28 * t121 * 0.05219)
               + t8 * (t111 - t117) * 0.408) + t2 * t6 * t57 * 0.12;
    dv11[3] = ((((t157 * t157 * 0.5 + t158 * t158 * 0.5) + t159 * t159 * 0.5) +
                t160 * t160 * 0.5) + t161 * t161 * 0.5) + t162 * t162 * 0.5;
    dv11[4] = (t158 * t215 * 0.5 - t160 * t227 * 0.5) + t162 * t236 * 0.5;
    dv11[5] = (t159 * t215 * -0.5 - t157 * t236 * 0.5) + t227 * (((t179 + t180)
        - t181) + t182) * 0.5;
    dv11[6] = ((((((((((((((((t32 * -0.0045 - t2 * t6 * 0.12) - t9 * t12 *
        0.43476) - t9 * t15 * 0.02) + t12 * t13 * 0.02) - t13 * t15 * 0.43476) +
                         t8 * t23 * 0.04) + t19 * t22 * 0.04) + t19 * t23 *
                       0.408) - t24 * t27 * 0.01762) - t24 * t31 * 0.05219) +
                    t27 * t28 * 0.05219) - t28 * t31 * 0.01762) - t8 * (t20 +
                   t13 * t15) * 0.408) - t2 * t3 * t4 * 0.06068) - t2 * t3 * t5 *
                0.04741) + t2 * t4 * t6 * 0.04741) - t2 * t5 * t6 * 0.06068;
    dv11[7] = ((((((((((((((((t2 * t57 * -0.0045 + t9 * t84 * 0.43476) + t9 *
        t86 * 0.02) - t13 * t84 * 0.02) + t8 * t90 * 0.408) + t13 * t86 *
                          0.43476) - t8 * t92 * 0.04) - t19 * t90 * 0.04) - t19 *
                       t92 * 0.408) + t24 * t96 * 0.05219) + t24 * t98 * 0.01762)
                    + t28 * t96 * 0.01762) - t28 * t98 * 0.05219) + t6 * t32 *
                  t57 * 0.12) + t3 * t4 * t32 * t57 * 0.06068) + t3 * t5 * t32 *
                t57 * 0.04741) - t4 * t6 * t32 * t57 * 0.04741) + t5 * t6 * t32 *
        t57 * 0.06068;
    dv11[8] = ((((((((((((((((t2 * t58 * -0.0045 + t9 * t131 * 0.43476) + t9 *
        t133 * 0.02) - t8 * t136 * 0.04) - t13 * t131 * 0.02) + t13 * t133 *
                          0.43476) + t8 * t139 * 0.408) - t19 * t136 * 0.408) -
                       t19 * t139 * 0.04) + t24 * t143 * 0.05219) + t24 * t145 *
                     0.01762) + t28 * t143 * 0.01762) - t28 * t145 * 0.05219) +
                  t6 * t32 * t58 * 0.12) + t3 * t4 * t32 * t58 * 0.06068) + t3 *
                t5 * t32 * t58 * 0.04741) - t4 * t6 * t32 * t58 * 0.04741) + t5 *
        t6 * t32 * t58 * 0.06068;
    dv11[9] = ((((t158 * t219 * -0.5 - t162 * t232 * 0.5) - t159 * t165 * 0.5) -
                t157 * t171 * 0.5) + t223 * (((t175 + t176) - t177) + t178) *
               0.5) + t17 * (((t179 + t180) - t181) + t182) * 0.5;
    dv11[10] = ((((t159 * t164 * -0.5 - t157 * t166 * 0.5) - t215 * t219 * 0.5)
                 - t223 * t227 * 0.5) - t232 * t236 * 0.5) + t14 * (((t179 +
        t180) - t181) + t182) * 0.5;
    dv11[11] = ((((t158 * t164 * -0.5 - t162 * t166 * 0.5) + t215 * t165 * 0.5)
                 + t227 * t17 * 0.5) + t236 * t171 * 0.5) + t14 * (((t175 + t176)
        - t177) + t178) * 0.5;
    dv11[12] = (((((((((((((((t50 + t51) + t52) + t54) + t55) + t56) + t3 * t32 *
                         0.12) - t9 * t35 * 0.02) - t13 * t35 * 0.43476) - t13 *
                      t37 * 0.02) - t19 * t43 * 0.04) - t24 * t49 * 0.05219) -
                   t28 * t47 * 0.05219) - t28 * t49 * 0.01762) - t3 * t4 * t32 *
                 0.04741) - t4 * t6 * t32 * 0.06068) - t5 * t6 * t32 * 0.04741;
    dv11[13] = ((((((((((((((((t122 + t123) + t125) + t126) + t127) + t128) - t6
                          * t58 * 0.12) - t4 * t101 * 0.06068) - t5 * t101 *
                        0.04741) - t5 * t103 * 0.06068) - t9 * t106 * 0.02) - t9
                     * t109 * 0.43476) - t13 * t106 * 0.43476) - t8 * t115 *
                   0.408) - t24 * t118 * 0.01762) - t24 * t121 * 0.05219) - t28 *
                t121 * 0.01762) + t2 * t3 * t57 * 0.12;
    dv11[14] = ((((((((((((((((t147 + t148) + t149) + t150) + t151) + t152) +
                          t153) + t154) + t155) + t156) + t6 * t57 * 0.12) - t4 *
                     t63 * 0.04741) - t8 * t72 * 0.04) - t13 * t69 * 0.02) - t19
                  * t72 * 0.408) - t19 * t75 * 0.04) - t28 * t78 * 0.05219) + t2
        * t3 * t58 * 0.12;
    dv11[15] = t205;
    dv11[16] = t172;
    dv11[17] = t7;
    dv11[18] = ((((((t50 + t51) - t9 * t35 * 0.02) - t13 * t35 * 0.43476) - t13 *
                  t37 * 0.02) - t3 * t4 * t32 * 0.04741) - t4 * t6 * t32 *
                0.06068) - t5 * t6 * t32 * 0.04741;
    dv11[19] = ((((((t122 + t123) - t4 * t101 * 0.06068) - t5 * t101 * 0.04741)
                  - t5 * t103 * 0.06068) - t9 * t106 * 0.02) - t9 * t109 *
                0.43476) - t13 * t106 * 0.43476;
    dv11[20] = ((((((t147 + t148) + t149) + t150) + t151) + t152) - t4 * t63 *
                0.04741) - t13 * t69 * 0.02;
    dv11[21] = 0.0;
    dv11[22] = 0.0;
    dv11[23] = 0.0;
    dv11[24] = ((t52 - t24 * t49 * 0.05219) - t28 * t47 * 0.05219) - t28 * t49 *
        0.01762;
    dv11[25] = ((t125 - t24 * t118 * 0.01762) - t24 * t121 * 0.05219) - t28 *
        t121 * 0.01762;
    dv11[26] = ((t153 + t154) + t155) - t28 * t78 * 0.05219;
    dv11[27] = t205;
    dv11[28] = t172;
    dv11[29] = t7;
    dv11[30] = ((t50 - t9 * t35 * 0.02) - t13 * t35 * 0.43476) - t13 * t37 *
        0.02;
    dv11[31] = ((t122 - t9 * t106 * 0.02) - t9 * t109 * 0.43476) - t13 * t106 *
        0.43476;
    dv11[32] = ((t147 + t151) + t152) - t13 * t69 * 0.02;
    dv11[33] = 0.0;
    dv11[34] = 0.0;
    dv11[35] = 0.0;
    dv11[36] = ((((((t52 + t54) + t55) + t56) - t19 * t43 * 0.04) - t24 * t49 *
                 0.05219) - t28 * t47 * 0.05219) - t28 * t49 * 0.01762;
    dv11[37] = ((((((t125 + t126) + t127) + t128) - t8 * t115 * 0.408) - t24 *
                 t118 * 0.01762) - t24 * t121 * 0.05219) - t28 * t121 * 0.01762;
    dv11[38] = ((((((t153 + t154) + t155) + t156) - t8 * t72 * 0.04) - t19 * t72
                 * 0.408) - t19 * t75 * 0.04) - t28 * t78 * 0.05219;
    dv11[39] = t205;
    dv11[40] = t172;
    dv11[41] = t7;
    i17 = 0;
    for (i18 = 0; i18 < 7; i18++) {
        for (i19 = 0; i19 < 6; i19++) {
            J[i19 + i17] = dv11[i19 + i17];
        }

        i17 += 6;
    }
}

static int ixamax(int n, const double b_x[3], int ix0)
{
    int idxmax;
    int ix;
    double smax;
    int k;
    double s;
    if (n < 1) {
        idxmax = 0;
    } else {
        idxmax = 1;
        if (n > 1) {
            ix = ix0 - 1;
            smax = fabs(b_x[ix0 - 1]);
            for (k = 2; k <= n; k++) {
                ix++;
                s = fabs(b_x[ix]);
                if (s > smax) {
                    idxmax = k;
                    smax = s;
                }
            }
        }
    }

    return idxmax;
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

static void mldivide(const double A[6], const double B[2], double Y[3])
{
    int rankR;
    double b_A[6];
    double tau[2];
    int jpvt[3];
    double tol;
    int i;
    double b_B[2];
    int j;
    for (rankR = 0; rankR < 6; rankR++) {
        b_A[rankR] = A[rankR];
    }

    xgeqp3(b_A, tau, jpvt);
    rankR = 0;
    tol = 3.0 * fabs(b_A[0]) * 2.2204460492503131E-16;
    while ((rankR < 2) && (fabs(b_A[rankR + (rankR << 1)]) >= tol)) {
        rankR++;
    }

    for (i = 0; i < 2; i++) {
        b_B[i] = B[i];
    }

    for (i = 0; i < 3; i++) {
        Y[i] = 0.0;
    }

    for (j = 0; j < 2; j++) {
        if (tau[j] != 0.0) {
            tol = b_B[j];
            i = j + 2;
            while (i < 3) {
                tol += b_A[1 + (j << 1)] * b_B[1];
                i = 3;
            }

            tol *= tau[j];
            if (tol != 0.0) {
                b_B[j] -= tol;
                i = j + 2;
                while (i < 3) {
                    b_B[1] -= b_A[1 + (j << 1)] * tol;
                    i = 3;
                }
            }
        }
    }

    for (i = 0; i + 1 <= rankR; i++) {
        Y[jpvt[i] - 1] = b_B[i];
    }

    for (j = rankR - 1; j + 1 > 0; j--) {
        Y[jpvt[j] - 1] /= b_A[j + (j << 1)];
        i = 1;
        while (i <= j) {
            Y[jpvt[0] - 1] -= Y[jpvt[j] - 1] * b_A[j << 1];
            i = 2;
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

static double rt_hypotd(double u0, double u1)
{
    double y;
    double a;
    double b;
    a = fabs(u0);
    b = fabs(u1);
    if (a < b) {
        a /= b;
        y = b * sqrt(a * a + 1.0);
    } else if (a > b) {
        b /= a;
        y = a * sqrt(b * b + 1.0);
    } else {
        y = a * 1.4142135623730951;
    }

    return y;
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

static void sort3(int i1, double v1, int i2, double v2, int i3, double v3, int
                  *b_j1, int *j2)
{
    if (v1 < v2) {
        if (v2 < v3) {
            *b_j1 = i1;
            *j2 = i2;
        } else if (v1 < v3) {
            *b_j1 = i1;
            *j2 = i3;
        } else {
            *b_j1 = i3;
            *j2 = i1;
        }
    } else if (v1 < v3) {
        *b_j1 = i2;
        *j2 = i1;
    } else if (v2 < v3) {
        *b_j1 = i2;
        *j2 = i3;
    } else {
        *b_j1 = i3;
        *j2 = i2;
    }
}

static void xgeqp3(double A[6], double tau[2], int jpvt[3])
{
    int k;
    int j;
    int i;
    double work[3];
    int i_i;
    double xnorm;
    int pvt;
    double vn1[3];
    double vn2[3];
    double atmp;
    double d14;
    int i_ip1;
    double beta1;
    int lastv;
    int lastc;
    bool exitg2;
    int i124;
    int exitg1;
    int ix;
    int ijA;
    k = 1;
    for (j = 0; j < 3; j++) {
        jpvt[j] = 1 + j;
        work[j] = 0.0;
        xnorm = xnrm2(A, k);
        vn2[j] = xnorm;
        k += 2;
        vn1[j] = xnorm;
    }

    for (i = 0; i < 2; i++) {
        i_i = i + (i << 1);
        pvt = (i + ixamax(3 - i, vn1, i + 1)) - 1;
        if (pvt + 1 != i + 1) {
            xswap(A, 1 + (pvt << 1), 1 + (i << 1));
            k = jpvt[pvt];
            jpvt[pvt] = jpvt[i];
            jpvt[i] = k;
            vn1[pvt] = vn1[i];
            vn2[pvt] = vn2[i];
        }

        if (i + 1 < 2) {
            atmp = A[i_i];
            d14 = 0.0;
            xnorm = b_xnrm2(1, A, i_i + 2);
            if (xnorm != 0.0) {
                beta1 = rt_hypotd(A[i_i], xnorm);
                if (A[i_i] >= 0.0) {
                    beta1 = -beta1;
                }

                if (fabs(beta1) < 1.0020841800044864E-292) {
                    pvt = 0;
                    do {
                        pvt++;
                        for (k = i_i + 1; k + 1 <= i_i + 2; k++) {
                            A[k] *= 9.9792015476736E+291;
                        }

                        beta1 *= 9.9792015476736E+291;
                        atmp *= 9.9792015476736E+291;
                    } while (!(fabs(beta1) >= 1.0020841800044864E-292));

                    beta1 = rt_hypotd(atmp, b_xnrm2(1, A, i_i + 2));
                    if (atmp >= 0.0) {
                        beta1 = -beta1;
                    }

                    d14 = (beta1 - atmp) / beta1;
                    atmp = 1.0 / (atmp - beta1);
                    for (k = i_i + 1; k + 1 <= i_i + 2; k++) {
                        A[k] *= atmp;
                    }

                    for (k = 1; k <= pvt; k++) {
                        beta1 *= 1.0020841800044864E-292;
                    }

                    atmp = beta1;
                } else {
                    d14 = (beta1 - A[i_i]) / beta1;
                    atmp = 1.0 / (A[i_i] - beta1);
                    for (k = i_i + 1; k + 1 <= i_i + 2; k++) {
                        A[k] *= atmp;
                    }

                    atmp = beta1;
                }
            }

            tau[0] = d14;
            A[i_i] = atmp;
        } else {
            tau[1] = 0.0;
        }

        atmp = A[i_i];
        A[i_i] = 1.0;
        i_ip1 = (i + ((i + 1) << 1)) + 1;
        if (tau[i] != 0.0) {
            lastv = 2 - i;
            pvt = (i_i - i) + 1;
            while ((lastv > 0) && (A[pvt] == 0.0)) {
                lastv--;
                pvt--;
            }

            lastc = 2 - i;
            exitg2 = false;
            while ((!exitg2) && (lastc > 0)) {
                pvt = i_ip1 + ((lastc - 1) << 1);
                j = pvt;
                do {
                    exitg1 = 0;
                    if (j <= (pvt + lastv) - 1) {
                        if (A[j - 1] != 0.0) {
                            exitg1 = 1;
                        } else {
                            j++;
                        }
                    } else {
                        lastc--;
                        exitg1 = 2;
                    }
                } while (exitg1 == 0);

                if (exitg1 == 1) {
                    exitg2 = true;
                }
            }
        } else {
            lastv = 0;
            lastc = 0;
        }

        if (lastv > 0) {
            if (lastc != 0) {
                for (pvt = 1; pvt <= lastc; pvt++) {
                    work[pvt - 1] = 0.0;
                }

                pvt = 0;
                i124 = i_ip1 + ((lastc - 1) << 1);
                for (k = i_ip1; k <= i124; k += 2) {
                    ix = i_i;
                    xnorm = 0.0;
                    ijA = (k + lastv) - 1;
                    for (j = k; j <= ijA; j++) {
                        xnorm += A[j - 1] * A[ix];
                        ix++;
                    }

                    work[pvt] += xnorm;
                    pvt++;
                }
            }

            if (!(-tau[i] == 0.0)) {
                pvt = i_ip1 - 1;
                k = 0;
                for (j = 1; j <= lastc; j++) {
                    if (work[k] != 0.0) {
                        xnorm = work[k] * -tau[i];
                        ix = i_i;
                        i124 = lastv + pvt;
                        for (ijA = pvt; ijA + 1 <= i124; ijA++) {
                            A[ijA] += A[ix] * xnorm;
                            ix++;
                        }
                    }

                    k++;
                    pvt += 2;
                }
            }
        }

        A[i_i] = atmp;
        for (j = i + 1; j + 1 < 4; j++) {
            if (vn1[j] != 0.0) {
                xnorm = fabs(A[i + (j << 1)]) / vn1[j];
                xnorm = 1.0 - xnorm * xnorm;
                if (xnorm < 0.0) {
                    xnorm = 0.0;
                }

                beta1 = vn1[j] / vn2[j];
                beta1 = xnorm * (beta1 * beta1);
                if (beta1 <= 1.4901161193847656E-8) {
                    if (i + 1 < 2) {
                        vn1[j] = fabs(A[(i + (j << 1)) + 1]);
                        vn2[j] = vn1[j];
                    } else {
                        vn1[j] = 0.0;
                        vn2[j] = 0.0;
                    }
                } else {
                    vn1[j] *= sqrt(xnorm);
                }
            }
        }
    }
}

static double xnrm2(const double b_x[6], int ix0)
{
    double y;
    double scale;
    int k;
    double absxk;
    double t;
    y = 0.0;
    scale = 2.2250738585072014E-308;
    for (k = ix0; k <= ix0 + 1; k++) {
        absxk = fabs(b_x[k - 1]);
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

static void xswap(double b_x[6], int ix0, int iy0)
{
    int ix;
    int iy;
    int k;
    double temp;
    ix = ix0 - 1;
    iy = iy0 - 1;
    for (k = 0; k < 2; k++) {
        temp = b_x[ix];
        b_x[ix] = b_x[iy];
        b_x[iy] = temp;
        ix++;
        iy++;
    }
}

cassie_system_t* cassie_system_alloc(void)
{
    cassie_system_t *system = (cassie_system_t*) malloc(sizeof (cassie_system_t));
    system->process_inputs = (ProcessInputsSimulation*) malloc(sizeof (ProcessInputsSimulation));
    system->process_outputs = (ProcessOutputsSimulation*)malloc(sizeof (ProcessOutputsSimulation));
    system->radio_spoofer = (RadioSpoofer*)malloc(sizeof (RadioSpoofer));
    system->controller = (CassieController*)malloc(sizeof (CassieController));
    return system;
}

void cassie_system_free(cassie_system_t *system)
{
    free(system->process_inputs);
    free(system->process_outputs);
    free(system->radio_spoofer);
    free(system->controller);
    free(system);
}

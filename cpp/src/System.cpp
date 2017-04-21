#include "System.hpp"
#include "cassie_system.h"
#include <cstddef>
#include <cmath>
#include "mujoco.h"


static double encoder(const mjModel* m, const mjtNum* sensordata, size_t i)
{
    size_t bits = m->sensor_user[m->nuser_sensor * i];
    int encoder_value = sensordata[i] / (2 * M_PI) * (1 << bits);
    double ratio = 1;
    if (m->sensor_objtype[i] == mjOBJ_ACTUATOR)
        ratio = m->actuator_gear[6 * m->sensor_objid[i]];
    return encoder_value * (2 * M_PI) / (1 << bits) / ratio;
}


static void motor(const mjModel* m, mjData* d, size_t i, double u)
{
    double ratio = m->actuator_gear[6 * i];
    double tmax = m->actuator_ctrlrange[2 * i + 1];
    double w = d->actuator_velocity[i];
    double wmax = m->actuator_user[m->nuser_actuator * i] * 2 * M_PI / 60;

    // Calculate torque limit based on motor speed
    double tlim = 2 * tmax * (1 - std::fabs(w) / wmax);
    tlim = std::fmax(std::fmin(tlim, tmax), 0);

    // Compute motor-side torque
    d->ctrl[i] = std::copysign(std::fmin(std::fabs(u / ratio), tlim), u);
}


System::System()
{
    ethercat = {};
    system = cassie_system_alloc();
    cassie_system_init(system);
}


System::~System()
{
    cassie_system_free(system);
}


void System::step(const mjModel* m, mjData* d)
{
    size_t i = 0;

    // Encoders
    ethercat.leftAbductionDrive.inputs.position =
        encoder(m, d->sensordata, i++);
    ethercat.leftYawDrive.inputs.position =
        encoder(m, d->sensordata, i++);
    ethercat.leftHipDrive.inputs.position =
        encoder(m, d->sensordata, i++);
    ethercat.leftKneeDrive.inputs.position =
        encoder(m, d->sensordata, i++);
    ethercat.leftLegMedulla.inputs.kneeEncoder =
        encoder(m, d->sensordata, i++);
    ethercat.leftLegMedulla.inputs.ankleEncoder =
        encoder(m, d->sensordata, i++);
    ethercat.leftFootDrive.inputs.position =
        encoder(m, d->sensordata, i++);
    ethercat.leftLegMedulla.inputs.footEncoder =
        encoder(m, d->sensordata, i++);

    ethercat.rightAbductionDrive.inputs.position =
        -encoder(m, d->sensordata, i++);
    ethercat.rightYawDrive.inputs.position =
        -encoder(m, d->sensordata, i++);
    ethercat.rightHipDrive.inputs.position =
        encoder(m, d->sensordata, i++);
    ethercat.rightKneeDrive.inputs.position =
        encoder(m, d->sensordata, i++);
    ethercat.rightLegMedulla.inputs.kneeEncoder =
        encoder(m, d->sensordata, i++);
    ethercat.rightLegMedulla.inputs.ankleEncoder =
        encoder(m, d->sensordata, i++);
    ethercat.rightFootDrive.inputs.position =
        encoder(m, d->sensordata, i++);
    ethercat.rightLegMedulla.inputs.footEncoder =
        encoder(m, d->sensordata, i++);

    // IMU
    mju_copy(ethercat.pelvisMedulla.inputs.vnOrientation,
             &d->sensordata[i], 4);
    i += 4;
    mju_copy(ethercat.pelvisMedulla.inputs.vnGyro, &d->sensordata[i], 3);
    i += 3;
    mju_copy(ethercat.pelvisMedulla.inputs.vnAccel, &d->sensordata[i], 3);
    i += 3;
    mju_copy(ethercat.pelvisMedulla.inputs.vnMag, &d->sensordata[i], 3);
    i += 3;

    // Run control system
    cassie_system_step(system, &ethercat);

    // Motors
    i = 0;
    motor(m, d, i++, ethercat.leftAbductionDrive.outputs.torque);
    motor(m, d, i++, ethercat.leftYawDrive.outputs.torque);
    motor(m, d, i++, ethercat.leftHipDrive.outputs.torque);
    motor(m, d, i++, ethercat.leftKneeDrive.outputs.torque);
    motor(m, d, i++, ethercat.leftFootDrive.outputs.torque);

    motor(m, d, i++, -ethercat.rightAbductionDrive.outputs.torque);
    motor(m, d, i++, -ethercat.rightYawDrive.outputs.torque);
    motor(m, d, i++, ethercat.rightHipDrive.outputs.torque);
    motor(m, d, i++, ethercat.rightKneeDrive.outputs.torque);
    motor(m, d, i++, ethercat.rightFootDrive.outputs.torque);
}

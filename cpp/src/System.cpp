#include "System.hpp"
#include "cassie_system.h"
#include <cstddef>
#include <cmath>
#include "mujoco.h"
#include <iostream>


static double encoder(const mjModel* m, const mjtNum* sensordata, size_t i)
{
    size_t bits = m->sensor_user[m->nuser_sensor * i];
    int encoder_value = sensordata[i] / (2 * M_PI) * (1 << bits);
    double ratio = 1;
    if (m->sensor_objtype[i] == 17)
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


static void quatmul(const mjtNum* q, const mjtNum* p, mjtNum* dst)
{
    dst[0] = p[0]*q[0] - p[1]*q[1] - p[2]*q[2] - p[3]*q[3];
    dst[1] = p[0]*q[1] + p[1]*q[0] - p[2]*q[3] + p[3]*q[2];
    dst[2] = p[0]*q[2] + p[2]*q[0] + p[1]*q[3] - p[3]*q[1];
    dst[3] = p[0]*q[3] - p[1]*q[2] + p[2]*q[1] + p[3]*q[0];
}


static void quatrot(const mjtNum* q, const mjtNum* v, mjtNum* dst)
{
    dst[0] = v[0]*q[0]*q[0] + 2*v[2]*q[0]*q[2] - 2*v[1]*q[0]*q[3] +
        v[0]*q[1]*q[1] + 2*v[1]*q[1]*q[2] + 2*v[2]*q[1]*q[3] -
        v[0]*q[2]*q[2] - v[0]*q[3]*q[3];
    dst[1] = v[1]*q[0]*q[0] - 2*v[2]*q[0]*q[1] + 2*v[0]*q[0]*q[3] -
        v[1]*q[1]*q[1] + 2*v[0]*q[1]*q[2] + v[1]*q[2]*q[2] +
        2*v[2]*q[2]*q[3] - v[1]*q[3]*q[3];
    dst[2] = v[2]*q[0]*q[0] + 2*v[1]*q[0]*q[1] - 2*v[0]*q[0]*q[2] -
        v[2]*q[1]*q[1] + 2*v[0]*q[1]*q[3] - v[2]*q[2]*q[2] +
        2*v[1]*q[2]*q[3] + v[2]*q[3]*q[3];
}


System::System()
{
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
        encoder(m, d->sensordata, i++);
    ethercat.rightYawDrive.inputs.position =
        encoder(m, d->sensordata, i++);
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
    double rot[] = {-sqrt(2) / 2, 0, sqrt(2) / 2, 0};
    quatmul(rot, &d->sensordata[i],
            ethercat.pelvisMedulla.inputs.vnOrientation);
    i += 4;
    quatrot(rot, &d->sensordata[i], ethercat.pelvisMedulla.inputs.vnGyro);
    i += 3;
    quatrot(rot, &d->sensordata[i], ethercat.pelvisMedulla.inputs.vnAccel);
    i += 3;
    quatrot(rot, &d->sensordata[i], ethercat.pelvisMedulla.inputs.vnMag);
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

    motor(m, d, i++, ethercat.rightAbductionDrive.outputs.torque);
    motor(m, d, i++, ethercat.rightYawDrive.outputs.torque);
    motor(m, d, i++, ethercat.rightHipDrive.outputs.torque);
    motor(m, d, i++, ethercat.rightKneeDrive.outputs.torque);
    motor(m, d, i++, ethercat.rightFootDrive.outputs.torque);

    for (int j = 0; j < m->nu; ++j)
        std::cout << j << " " << d->ctrl[j] << std::endl;
}

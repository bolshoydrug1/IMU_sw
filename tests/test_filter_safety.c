/* Compiled with the actual filter helpers by run_safety_tests.py. */
#include <assert.h>
#include <float.h>
#include <stdio.h>

int main(void)
{
    float q[4] = {1, 0, 0, 0};
    for (int i = 0; i < 10000; ++i)
        assert(MadgwickUpdateIMU(q, 0, 0, 0, 0, 0, 1, .002f));
    assert(fabsf(q[0] - 1) < 1e-6f); /* zero gradient must stay finite */
    assert(!MadgwickUpdateIMU(q, NAN, 0, 0, 0, 0, 1, .002f));
    assert(!MadgwickUpdateIMU(q, 0, 0, 0, 0, 0, 1, 0));
    assert(!MadgwickUpdateIMU(q, 0, 0, 0, 0, 0, 1, .2f));
    assert(MadgwickUpdateIMU(q, 0, 0, 1, 0, 0, 0, .002f));
    float bad[4] = {0};
    assert(!NormalizeQuaternion(bad));
    bad[0] = INFINITY;
    assert(!NormalizeQuaternion(bad));
    bad[0] = FLT_MAX;
    assert(!NormalizeQuaternion(bad));

    float a[3] = {0, .5f, .8660254f};
    SeedQuaternion(q, a, .4f);
    float roll, pitch, yaw;
    QuaternionToEuler(q, &roll, &pitch, &yaw);
    assert(fabsf(roll - 30) < .001f && fabsf(pitch) < .001f);
    assert(fabsf(yaw - .4f*180/(float)M_PI) < .001f);

    IMU_Sample_t sample = {0};
    sample.valid = true;
    sample.accel_g[2] = 1;
    sample.gyro_dps[2] = 90;
    sample.timestamp_ms = 100;
    assert(UpdateSensor(0, &sample, 100, 0));
    float saved[4];
    memcpy(saved, s_q[0], sizeof(saved));
    assert(UpdateSensor(0, &sample, 102, 0));
    assert(memcmp(saved, s_q[0], sizeof(saved)) == 0); /* duplicate */
    sample.timestamp_ms = 102;
    assert(UpdateSensor(0, &sample, 102, 0));
    QuaternionToEuler(s_q[0], &roll, &pitch, &yaw);
    assert(fabsf(yaw - .18f) < .001f); /* dt from sample, not consumer */
    assert(!UpdateSensor(0, &sample, 113, 0)); /* stale */
    sample.timestamp_ms = 1000;
    assert(UpdateSensor(0, &sample, 1000, .7f)); /* reconnect, no huge dt */
    QuaternionToEuler(s_q[0], &roll, &pitch, &yaw);
    assert(fabsf(yaw - .7f*180/(float)M_PI) < .001f);
    s_q[0][0] = NAN;
    assert(UpdateSensor(0, &sample, 1000, 0)); /* corrupt duplicate recovers */
    assert(isfinite(s_q[0][0]));
    sample.valid = false;
    assert(!UpdateSensor(0, &sample, 1000, 0));
    sample.valid = true;
    sample.gyro_dps[0] = NAN;
    assert(!UpdateSensor(0, &sample, 1000, 0));
    assert(!UpdateSensor(1, NULL, 1000, 0)); /* missing independent sensor */
    sample.gyro_dps[0] = 0;
    sample.timestamp_ms = UINT32_MAX - 1;
    assert(UpdateSensor(2, &sample, UINT32_MAX - 1, 0));
    sample.timestamp_ms = 0;
    assert(UpdateSensor(2, &sample, 0, 0)); /* tick wrap */
    assert(s_lastUpdateMs[2] == 0);
    puts("Filter safety: all assertions passed");
    return 0;
}

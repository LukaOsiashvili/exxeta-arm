// Joint motion generator: smooth, speed- and acceleration-limited moves.
//
// Role: hobby servos jump to whatever pulse they are given. To move gently the
// firmware never sends the target directly; every 10 ms it moves an internal
// "position" a little towards the target and sends THAT to the servo.
//
// Rule each step (a classic velocity profile with a braking-distance check):
//   wanted speed = min(max speed, speed from which we can still brake to a stop in time)
//   speed changes by at most accel * dt
//   position += speed * dt
//
// Pure C++ (no Arduino): also compiled into the host unit tests. The simulator
// (dora/exxeta_arm/servo_model.py) implements exactly the same rule, so the
// simulated arm moves like the real one.
#pragma once
#include <math.h>
#include <stdint.h>

struct JointMotion {
  float pos = 0, vel = 0, target = 0;   // degrees, degrees/s
  float vmax = 60, amax = 180;          // limits for this joint (from the configuration)
  float speedCap = 0;                   // per-command speed cap, 0 = use vmax
  float minDeg = -90, maxDeg = 90;
  bool clamped = false;                 // last target was outside the joint limits

  void reset(float p) { pos = target = p; vel = 0; }

  void setTarget(float t, float cap = 0) {
    clamped = (t < minDeg || t > maxDeg);
    target = t < minDeg ? minDeg : (t > maxDeg ? maxDeg : t);
    speedCap = (cap > 0 && cap < vmax) ? cap : 0;
  }

  // Controlled stop: aim at the point where the joint comes to rest when braking now.
  void stop() {
    float d = vel * vel / (2 * amax);
    target = pos + (vel >= 0 ? d : -d);
  }

  bool moving() const { return fabsf(target - pos) > 0.02f || fabsf(vel) > 0.5f; }

  void step(float dt) {
    float err = target - pos;
    float vlim = speedCap > 0 ? speedCap : vmax;
    // Exact braking speed for steps of dt (plain sqrt(2*a*err) brakes a little late and overshoots).
    float adt = amax * dt;
    float want = adt * (sqrtf(0.25f + 2 * fabsf(err) / (adt * dt)) - 0.5f);
    if (want > vlim) want = vlim;
    if (err < 0) want = -want;
    float dv = want - vel, dvmax = amax * dt;
    if (dv > dvmax) dv = dvmax;
    if (dv < -dvmax) dv = -dvmax;
    vel += dv;
    float step = vel * dt;
    if (fabsf(err) <= fabsf(step) && fabsf(vel) <= amax * dt * 2) {  // arrived: snap, no creeping
      pos = target;
      vel = 0;
    } else {
      pos += step;
    }
  }
};

// Servo calibration: pulse width for a joint angle (per servo, stored in flash).
struct ServoCal {
  uint16_t minUs = 500, maxUs = 2500, zeroUs = 1500;
  float usPerDeg = 11.11f;   // sign = direction

  uint16_t pulse(float deg) const {
    float us = zeroUs + usPerDeg * deg;
    if (us < minUs) us = minUs;
    if (us > maxUs) us = maxUs;
    return (uint16_t)(us + 0.5f);
  }
};

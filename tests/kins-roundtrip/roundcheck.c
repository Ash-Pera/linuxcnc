/* Check that a kinematics module's forward and inverse are inverse maps
 * over a stretch of its working range, the way motion calls them.
 *
 * Loaded after the module under test, so kinematicsForward() and
 * kinematicsInverse() resolve to it.  A failed check fails the load, and
 * a failed load fails the test.
 *
 * The other kinematics tests hold a module at a point or two chosen by
 * hand.  This one draws a few thousand points at random, from a fixed
 * seed so a failure repeats, out of a box the test names per module, and
 * requires the round trip at every one of them.  A bug that lives in one
 * quadrant, past a half turn, or where a sine changes sign is found
 * without anyone having guessed where to look.
 *
 * Two flows, as in kins-params.
 *
 * With frompose=0 the box is in joint space, for the serial machines.
 * Take the pose of the joints from the forward, then hand the inverse
 * that pose, the flags the forward set, and a seed a little away from the
 * joints, as motion does on the next cycle.  The inverse has to
 * return the joints that made the pose: not another branch of the arm,
 * and not a joint a whole turn away, since motion would take either as a
 * step and fault on it.
 *
 * With frompose=1 the box is in pose space, for the parallel machines,
 * whose forward iterates from a seed.  Take the joints of the pose from
 * the inverse, then hand the forward those joints and a seed a little
 * away from the pose.  The forward has to return the pose, and the
 * inverse of that has to return the joints.
 *
 * The box should keep clear of the module's singularities, where the
 * joints that reach a pose are not determined and no inverse can be held
 * to any one of them; test.sh says, per module, what it keeps clear of.
 *
 * Module parameters
 *   joints    joint count the module was loaded for
 *   types     switchkins types to check, from 0; -1 for all it has
 *   frompose  0 or 1, as above
 *   lo, hi    the box, one pair per joint (frompose=0) or per pose
 *             coordinate x y z a b c u v w (frompose=1), in units of
 *             1/denom; a coordinate with lo == hi is held there
 *   denom     divisor for lo and hi, for boxes smaller than a unit
 *   n         how many points to draw, per kinematics type
 *   rseed     seed for the draw
 *   tolexp    a round trip may miss by 10 to the minus this
 *   stepexp   the seed is 10 to the minus this away, per coordinate; the
 *             default 3 is one servo cycle at 1 unit or degree a second
 *             and 1 kHz
 *   tool      x y z of a tool offset motion would hand the module
 *
 * Author: LinuxCNC
 * License: GPL Version 2
 * System: Linux
 *
 * Copyright (c) 2026 All rights reserved.
 */

#include <rtapi.h>
#include <rtapi_app.h>
#include <rtapi_errno.h>
#include <rtapi_math.h>
#include <rtapi_string.h>
#include <hal.h>
#include <emcpos.h>
#include <emcmotcfg.h>
#include <kinematics.h>

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("kinematics round trip checker");

static int joints = 3;
RTAPI_MP_INT(joints, "joint count the module under test was loaded for");

static int types = -1;
RTAPI_MP_INT(types, "how many switchkins types to check, from 0; -1 for all the module has");

static int frompose = 0;
RTAPI_MP_INT(frompose, "1 to draw poses and take the joints from the inverse");

static int lo[EMCMOT_MAX_JOINTS];
RTAPI_MP_ARRAY_INT(lo, EMCMOT_MAX_JOINTS, "low corner of the box, in units of 1/denom");

static int hi[EMCMOT_MAX_JOINTS];
RTAPI_MP_ARRAY_INT(hi, EMCMOT_MAX_JOINTS, "high corner of the box, in units of 1/denom");

static int denom = 1;
RTAPI_MP_INT(denom, "divisor for lo and hi");

static int n = 2000;
RTAPI_MP_INT(n, "points to draw per kinematics type");

static int rseed = 1;
RTAPI_MP_INT(rseed, "seed for the draw");

static int tolexp = 6;
RTAPI_MP_INT(tolexp, "a round trip may miss by 10 to the minus this");

static int stepexp = 3;
RTAPI_MP_INT(stepexp, "the seed is 10 to the minus this from the answer");

static int tool[3] = { 0, 0, 0 };
RTAPI_MP_ARRAY_INT(tool, 3, "tool offset x y z, as motion hands it over");

/* switchkins.h is not an exported header, and a module rejects a type
   it does not have, so the loop only needs an upper bound */
#define MAX_TYPES 9
/* failures to print in full before only counting them */
#define MAX_REPORTS 10

static int comp_id = -1;
static int failures;
static double tolerance, step;
static unsigned long long state;

/* xorshift64*: the same draw on every platform, which rand() is not */
static double draw(void)
{
    state ^= state >> 12;
    state ^= state << 25;
    state ^= state >> 27;
    return (double)((state * 2685821657736338717ULL) >> 11) / 9007199254740992.0;
}

static double pose_coord(const EmcPose *p, int a)
{
    switch (a) {
    case 0: return p->tran.x;
    case 1: return p->tran.y;
    case 2: return p->tran.z;
    case 3: return p->a;
    case 4: return p->b;
    case 5: return p->c;
    case 6: return p->u;
    case 7: return p->v;
    default: return p->w;
    }
}

static void pose_set(EmcPose *p, int a, double v)
{
    switch (a) {
    case 0: p->tran.x = v; break;
    case 1: p->tran.y = v; break;
    case 2: p->tran.z = v; break;
    case 3: p->a = v; break;
    case 4: p->b = v; break;
    case 5: p->c = v; break;
    case 6: p->u = v; break;
    case 7: p->v = v; break;
    default: p->w = v; break;
    }
}

static int fmt(char *buf, int size, const double *v, int count)
{
    int i, k = 0;
    for (i = 0; i < count && k < size - 16; i++) {
        k += rtapi_snprintf(buf + k, size - k, "%s%.10g", i ? "," : "", v[i]);
    }
    return k;
}

/* the larger miss, and a NaN over any number */
static double larger(double w, double e)
{
    if (isnan(w)) { return w; }
    return (isnan(e) || e > w) ? e : w;
}

static void report(int t, const char *what, const double *at, int at_n,
                   const double *got, int got_n)
{
    char a[200], g[200];
    double miss = 0;
    int i;
    failures++;
    if (failures > MAX_REPORTS) { return; }
    fmt(a, sizeof(a), at, at_n);
    fmt(g, sizeof(g), got, got_n);
    for (i = 0; i < at_n && i < got_n; i++) {
        miss = larger(miss, fabs(got[i] - at[i]));
    }
    rtapi_print_msg(RTAPI_MSG_ERR, "roundcheck: FAIL type %d %s at [%s] got [%s], a miss of %.3g\n",
                    t, what, a, g, miss);
}

/* a coordinate of the box, drawn */
static double coord(int i)
{
    return (lo[i] + (hi[i] - lo[i]) * draw()) / denom;
}

/* the seed: each coordinate moved by step one way or the other */
static double nudge(void)
{
    return draw() < 0.5 ? -step : step;
}

static double worst;

static void from_joints(int t)
{
    double q[EMCMOT_MAX_JOINTS], r[EMCMOT_MAX_JOINTS];
    EmcPose p;
    KINEMATICS_FORWARD_FLAGS ff = 0;
    KINEMATICS_INVERSE_FLAGS inf = 0;
    int i, bad = 0;

    memset(q, 0, sizeof(q));
    for (i = 0; i < joints; i++) { q[i] = coord(i); }
    ZERO_EMC_POSE(p);
    if (kinematicsForward(q, &p, &ff, &inf)) {
        report(t, "forward fails", q, joints, q, 0);
        return;
    }
    for (i = 0; i < EMCMOT_MAX_JOINTS; i++) { r[i] = q[i] + (i < joints ? nudge() : 0); }
    if (kinematicsInverse(&p, r, &inf, &ff)) {
        report(t, "inverse fails on the forward's pose", q, joints, q, 0);
        return;
    }
    for (i = 0; i < joints; i++) {
        double e = fabs(r[i] - q[i]);
        if (!(e <= tolerance)) { bad = 1; }
        worst = larger(worst, e);
    }
    if (bad) { report(t, "inverse(forward(joints))", q, joints, r, joints); }
}

static void from_pose(int t)
{
    double q[EMCMOT_MAX_JOINTS], r[EMCMOT_MAX_JOINTS], want[EMCMOT_MAX_AXIS], got[EMCMOT_MAX_AXIS];
    EmcPose p, f;
    KINEMATICS_FORWARD_FLAGS ff = 0;
    KINEMATICS_INVERSE_FLAGS inf = 0;
    int i, bad = 0;

    ZERO_EMC_POSE(p);
    for (i = 0; i < EMCMOT_MAX_AXIS; i++) {
        want[i] = coord(i);
        pose_set(&p, i, want[i]);
    }
    memset(q, 0, sizeof(q));
    if (kinematicsInverse(&p, q, &inf, &ff)) {
        report(t, "inverse fails", want, EMCMOT_MAX_AXIS, want, 0);
        return;
    }
    f = p;
    for (i = 0; i < EMCMOT_MAX_AXIS; i++) {
        if (lo[i] != hi[i]) { pose_set(&f, i, want[i] + nudge()); }
    }
    if (kinematicsForward(q, &f, &ff, &inf)) {
        report(t, "forward fails on the inverse's joints", want, EMCMOT_MAX_AXIS, q, joints);
        return;
    }
    for (i = 0; i < EMCMOT_MAX_AXIS; i++) {
        double e;
        got[i] = pose_coord(&f, i);
        e = fabs(got[i] - want[i]);
        if (!(e <= tolerance)) { bad = 1; }
        worst = larger(worst, e);
    }
    if (bad) {
        report(t, "forward(inverse(pose))", want, EMCMOT_MAX_AXIS, got, EMCMOT_MAX_AXIS);
        return;
    }
    /* and back: the inverse of what the forward found, seeded a cycle off */
    for (i = 0; i < EMCMOT_MAX_JOINTS; i++) { r[i] = q[i] + (i < joints ? nudge() : 0); }
    if (kinematicsInverse(&f, r, &inf, &ff)) {
        report(t, "inverse fails on the forward's pose", want, EMCMOT_MAX_AXIS, q, joints);
        return;
    }
    for (i = 0; i < joints; i++) {
        double e = fabs(r[i] - q[i]);
        if (!(e <= tolerance)) { bad = 1; }
        worst = larger(worst, e);
    }
    if (bad) { report(t, "inverse(forward(inverse(pose)))", q, joints, r, joints); }
}

int rtapi_app_main(void)
{
    int t, k, checked = 0, total = 0;

    comp_id = hal_init("roundcheck");
    if (comp_id < 0) { return comp_id; }

    if (joints < 1 || joints > EMCMOT_MAX_JOINTS || denom < 1 || n < 1) {
        rtapi_print_msg(RTAPI_MSG_ERR, "roundcheck: bad joints, denom or n\n");
        hal_exit(comp_id);
        return -EINVAL;
    }
    tolerance = pow(10.0, -tolexp);
    step = pow(10.0, -stepexp);

    /* a tool offset, for the modules that turn it with the head; both
       directions see it, as both do in motion */
    if (tool[0] || tool[1] || tool[2]) {
        EmcPose t = { { tool[0], tool[1], tool[2] }, 0, 0, 0, 0, 0, 0 };
        if (kinematicsSetTool(&t)) {
            rtapi_print_msg(RTAPI_MSG_ERR, "roundcheck: the module takes no tool offset\n");
            hal_exit(comp_id);
            return -EINVAL;
        }
    }

    for (t = 0; t < MAX_TYPES && (types < 0 || t < types); t++) {
        /* the module starts in type 0 and is not switched to it: a switch
           restarts an iterating forward from a stored pose that is still
           zero, which for a hexapod is the singular pose it cannot leave */
        if (kinematicsSwitchable() && t > 0 && kinematicsSwitch(t)) { break; }
        checked++;
        /* every type sees the same points */
        state = 0x9E3779B97F4A7C15ULL ^ (unsigned long long)(unsigned)rseed;
        worst = 0;
        for (k = 0; k < n; k++) {
            if (frompose) { from_pose(t); } else { from_joints(t); }
            total++;
        }
        rtapi_print("roundcheck: type %d worst miss %.3g over %d points\n", t, worst, n);
        if (!kinematicsSwitchable()) { break; }
    }

    if (failures) {
        rtapi_print_msg(RTAPI_MSG_ERR, "roundcheck: %d of %d point(s) failed\n",
                        failures, total);
        hal_exit(comp_id);
        return -ERANGE;
    }
    rtapi_print("roundcheck: round trip holds for %d kinematics type(s), %d point(s)\n",
                checked, total);
    hal_ready(comp_id);
    return 0;
}

void rtapi_app_exit(void) { hal_exit(comp_id); }

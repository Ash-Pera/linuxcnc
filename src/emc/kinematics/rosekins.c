/*
  Copyright 2016 Dewey Garrett <dgarrett@panix.com>

  This program is free software; you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation; either version 2 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program; if not, write to the Free Software
  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
*/

#include <rtapi.h>
#include <rtapi_math.h>
#include <rtapi_string.h>
#include <rtapi_app.h>
#include <hal.h>
#include <kins_module.h>
#include <kins_rt.h>

MODULE_LICENSE("GPL");

#ifndef hypot
#define hypot(a,b) (sqrt((a)*(a)+(b)*(b)))
#endif

// the inverse reports joint 2's whole turns and its angles
static const kins_param_desc rose_params[] = {
    { "revolutions",      KINS_PARAM_REAL, KINS_OUT, 0, {.r=0.0} },
    { "theta_degrees",    KINS_PARAM_REAL, KINS_OUT, 0, {.r=0.0} },
    { "bigtheta_degrees", KINS_PARAM_REAL, KINS_OUT, 0, {.r=0.0} },
};
enum { O_REVOLUTIONS, O_THETA, O_BIGTHETA };

/* angle, in degrees, moved by whole turns to within a half turn of near */
static double nearestTurn(double angle, double near)
{
    return angle - 360.0*round((angle - near)/360.0);
}

static int rose_forward(const kins_params *p, kins_scratch *s,
                        const double *joints,
                        EmcPose * pos,
                        const KINEMATICS_FORWARD_FLAGS * fflags,
                        KINEMATICS_INVERSE_FLAGS * iflags)
{
    (void)p;
    (void)s;
    (void)fflags;
    (void)iflags;
    double radius,z,theta;

    radius = joints[0];
    z      = joints[1];
    theta  = TO_RAD * joints[2];

    pos->tran.x = radius * cos(theta);
    pos->tran.y = radius * sin(theta);
    pos->tran.z = z;
    pos->a = 0;
    pos->b = 0;
    pos->c = 0;
    pos->u = 0;
    pos->v = 0;
    pos->w = 0;

    return 0;
}

static int rose_inverse(const kins_params *p, kins_scratch *s,
                        const EmcPose * pos,
                        double *joints,
                        const KINEMATICS_INVERSE_FLAGS * iflags,
                        KINEMATICS_FORWARD_FLAGS * fflags)
{
    (void)p;
    (void)iflags;
    (void)fflags;
    double     x = pos->tran.x;
    double     y = pos->tran.y;
    double     z = pos->tran.z;
    double     theta = TO_DEG * atan2(y,x);
    // joint 2 is the angle within a half turn of the joint handed in
    double     bigtheta = nearestTurn(theta, joints[2]);

    s->out[O_REVOLUTIONS] = round((bigtheta - theta)/360.0);
    s->out[O_THETA]       = theta;
    s->out[O_BIGTHETA]    = bigtheta;

    joints[0] = hypot(x,y);
    joints[1] = z;
    joints[2] = bigtheta;
    joints[3] = 0;
    joints[4] = 0;
    joints[5] = 0;
    joints[6] = 0;
    joints[7] = 0;
    joints[8] = 0;

    return 0;
}

static int rose_jacobian(const kins_params *p, const double *joints,
                         const EmcPose *pos,
                         double jac[EMCMOT_MAX_JOINTS][EMCMOT_MAX_AXIS],
                         const KINEMATICS_INVERSE_FLAGS *iflags)
{
    double x = pos->tran.x, y = pos->tran.y;
    double r2 = x*x + y*y;
    double r = sqrt(r2);
    (void)p;
    (void)joints;
    (void)iflags;
    // on the axis the angle is undefined and its rate unbounded
    if (r2 <= 0) { return -1; }
    memset(jac, 0, EMCMOT_MAX_JOINTS * EMCMOT_MAX_AXIS * sizeof(jac[0][0]));
    jac[0][0] = x/r;  jac[0][1] = y/r;
    jac[1][2] = 1;
    jac[2][0] = -y/r2 * TO_DEG;
    jac[2][1] =  x/r2 * TO_DEG;
    return 0;
}

static const kins_ops rose_ops = {
    .forward  = rose_forward,
    .inverse  = rose_inverse,
    .jacobian = rose_jacobian,
};

// joints 0..2 are radius, z and the unwrapped angle; the entry points
// come from kins_single.c
const kins_module_info kins_module = {
    .name                 = "rosekins",
    .halprefix            = "rosekins",
    .params               = rose_params,
    .nparams              = sizeof(rose_params)/sizeof(rose_params[0]),
    .required_coordinates = "XYZ",
    .max_joints           = 3,
    .allow_duplicates     = 0,
    .ntypes               = 1,
    .ops                  = { &rose_ops },
};

static int comp_id;

void rtapi_app_exit(void) { hal_exit(comp_id); }

int rtapi_app_main(void) {
    comp_id = hal_init("rosekins");
    if(comp_id < 0) return comp_id;

    if (kinsSingleInit(comp_id, &kins_module, "XYZ", KINEMATICS_BOTH)) {
        hal_exit(comp_id);
        return -1;
    }

    hal_ready(comp_id);
    return 0;
}

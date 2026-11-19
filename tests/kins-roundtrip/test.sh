#!/bin/bash
set -e

halcompile --install roundcheck.c >/dev/null

# One hal file per module: they all define the same entry points, so
# only one can be loaded at a time.  Each entry names the box roundcheck
# draws from, in joint space for the serial machines and in pose space
# (frompose=1) for the parallel ones and the gantry, whose forward is not
# one to one.  Rotaries go past a whole turn wherever the module claims
# to follow the joint it is handed; the comments say where a box is held
# back and why.
# ONLY=<module> in the environment runs the entries for that module alone
run() {
    local hal
    case "$1" in "${ONLY:-}"*) ;; *) return 0 ;; esac
    hal=$(mktemp --suffix=.hal)
    { printf 'loadrt %s\n' "$1"
      printf '%s\n' "$2"
      printf 'loadrt roundcheck %s\n' "$3"
    } > "$hal"
    echo "=== $1"
    halrun -f "$hal" || status=1
    rm -f "$hal"
}

# every module runs, so one failure does not hide the next
status=0

ROT9_LO=-100,-100,-100,-400,-400,-400,-10,-10,-10
ROT9_HI=100,100,100,400,400,400,10,10,10

# identity, and the gantry from the pose side
run "trivkins coordinates=XYZ" "" "joints=3 lo=-100,-100,-100 hi=100,100,100"
run "trivkins coordinates=XYZY kinstype=BOTH" "" "joints=4 frompose=1 lo=-100,-100,-100 hi=100,100,100"
run "trivkins coordinates=XYZABCUVW" "" "joints=9 lo=$ROT9_LO hi=$ROT9_HI"
run "userkins" "" "joints=3 lo=-100,-100,-100 hi=100,100,100"
run "millturn" "" "joints=4 lo=-100,-100,-100,-400 hi=100,100,100,400"

# linear maps and one rotation
run "corexykins" "" "joints=9 lo=$ROT9_LO hi=$ROT9_HI"
run "rotatekins" "" "joints=9 lo=$ROT9_LO hi=$ROT9_HI"
run "matrixkins" \
    "setp matrixkins.C_xy 0.02
setp matrixkins.C_xz -0.01
setp matrixkins.C_yx 0.03
setp matrixkins.C_yz 0.015
setp matrixkins.C_zx -0.02
setp matrixkins.C_zy 0.01
setp matrixkins.C_zz 1.001" \
    "joints=9 lo=$ROT9_LO hi=$ROT9_HI"

# tables and heads, offsets set so no term drops out; the tilts to 120,
# the turns past a whole one.  maxkins has no A.
run "maxkins" \
    "setp maxkins.pivot-length 100" \
    "joints=9 lo=-100,-100,-100,0,-120,-400,-10,-10,-10 hi=100,100,100,0,120,400,10,10,10"
run "5axiskins coordinates=XYZBCW" "" "joints=6 lo=-100,-100,-100,-120,-400,-10 hi=100,100,100,120,400,10"
run "5axiskins coordinates=XYZBCW sparm=identityfirst" "" "joints=6 lo=-100,-100,-100,-120,-400,-10 hi=100,100,100,120,400,10"
run "5axiskins coordinates=XYZBCW" "" "joints=6 tool=4,3,12 lo=-100,-100,-100,-120,-400,-10 hi=100,100,100,120,400,10"

run "xyzac-trt-kins coordinates=XYZAC" \
    "setp xyzac-trt-kins.y-offset 3
setp xyzac-trt-kins.z-offset 11
setp xyzac-trt-kins.tool-offset 7
setp xyzac-trt-kins.x-rot-point 1
setp xyzac-trt-kins.y-rot-point 2
setp xyzac-trt-kins.z-rot-point 5" \
    "joints=5 lo=-100,-100,-100,-120,-400 hi=100,100,100,120,400"
run "xyzac-trt-kins coordinates=XYZAC" \
    "setp xyzac-trt-kins.conventional-directions 1
setp xyzac-trt-kins.y-offset 3
setp xyzac-trt-kins.z-offset 11
setp xyzac-trt-kins.x-rot-point 1
setp xyzac-trt-kins.y-rot-point 2
setp xyzac-trt-kins.z-rot-point 5" \
    "joints=5 tool=4,3,7 lo=-100,-100,-100,-120,-400 hi=100,100,100,120,400"
run "xyzbc-trt-kins coordinates=XYZBC" \
    "setp xyzbc-trt-kins.x-offset 3
setp xyzbc-trt-kins.z-offset 11
setp xyzbc-trt-kins.tool-offset 7
setp xyzbc-trt-kins.x-rot-point 1
setp xyzbc-trt-kins.y-rot-point 2
setp xyzbc-trt-kins.z-rot-point 5" \
    "joints=5 lo=-100,-100,-100,-120,-400 hi=100,100,100,120,400"
run "xyzbc-trt-kins coordinates=XYZBC" \
    "setp xyzbc-trt-kins.conventional-directions 1
setp xyzbc-trt-kins.x-offset 3
setp xyzbc-trt-kins.z-offset 11
setp xyzbc-trt-kins.x-rot-point 1
setp xyzbc-trt-kins.y-rot-point 2
setp xyzbc-trt-kins.z-rot-point 5" \
    "joints=5 tool=4,3,7 lo=-100,-100,-100,-120,-400 hi=100,100,100,120,400"
run "xyzab_tdr_kins" \
    "setp xyzab_tdr_kins.x-offset 3
setp xyzab_tdr_kins.z-offset 11
setp xyzab_tdr_kins.tool-offset-z 7
setp xyzab_tdr_kins.x-rot-point 1
setp xyzab_tdr_kins.y-rot-point 2
setp xyzab_tdr_kins.z-rot-point 5" \
    "joints=5 lo=-100,-100,-100,-120,-120 hi=100,100,100,120,120"

# nutating heads: the rotary angles come from the joints handed in, so
# all three go past a whole turn
run "xyzacb_trsrn" \
    "setp xyzacb_trsrn_kins.nut-angle 45
setp xyzacb_trsrn_kins.y-pivot 100
setp xyzacb_trsrn_kins.z-pivot 200
setp xyzacb_trsrn_kins.x-offset 5
setp xyzacb_trsrn_kins.y-offset 7
setp xyzacb_trsrn_kins.y-rot-axis 300
setp xyzacb_trsrn_kins.z-rot-axis 400
setp xyzacb_trsrn_kins.tool-offset-z 50
setp xyzacb_trsrn_kins.pre-rot 0.3
setp xyzacb_trsrn_kins.primary-angle 20
setp xyzacb_trsrn_kins.secondary-angle 35" \
    "joints=6 lo=-100,-100,-100,-400,-400,-400 hi=100,100,100,400,400,400"
run "xyzbca_trsrn" \
    "setp xyzbca_trsrn_kins.nut-angle 45
setp xyzbca_trsrn_kins.x-pivot 100
setp xyzbca_trsrn_kins.z-pivot 200
setp xyzbca_trsrn_kins.x-offset 5
setp xyzbca_trsrn_kins.y-offset 7
setp xyzbca_trsrn_kins.x-rot-axis 300
setp xyzbca_trsrn_kins.z-rot-axis 400
setp xyzbca_trsrn_kins.tool-offset-z 50
setp xyzbca_trsrn_kins.pre-rot 0.3
setp xyzbca_trsrn_kins.primary-angle 20
setp xyzbca_trsrn_kins.secondary-angle 35" \
    "joints=6 lo=-100,-100,-100,-400,-400,-400 hi=100,100,100,400,400,400"
run "xyzacb_trsrn" \
    "setp xyzacb_trsrn_kins.nut-angle 45
setp xyzacb_trsrn_kins.y-pivot 100
setp xyzacb_trsrn_kins.z-pivot 200
setp xyzacb_trsrn_kins.x-offset 5
setp xyzacb_trsrn_kins.y-offset 7
setp xyzacb_trsrn_kins.y-rot-axis 300
setp xyzacb_trsrn_kins.z-rot-axis 400" \
    "joints=6 tool=4,3,50 lo=-100,-100,-100,-400,-400,-400 hi=100,100,100,400,400,400"

# arms.  The base and the wrist roll go past a whole turn; the wrist
# pitch keeps 10 degrees off 0 and 180, where the wrist is straight or
# folded and joints 4 and 6 are not determined, and takes each sign in
# its own entry, since the forward sets the wrist flip from it
run "pumakins" "setp pumakins.D6 50" \
    "joints=6 lo=-400,-100,-100,-400,10,-400 hi=400,100,100,400,170,400"
run "pumakins" "setp pumakins.D6 50" \
    "joints=6 rseed=2 lo=-400,-100,-100,-400,-170,-400 hi=400,100,100,400,-10,400"
run "three21kins" "" \
    "joints=6 lo=-400,-100,-100,-400,10,-400 hi=400,100,100,400,170,400"
run "three21kins" "" \
    "joints=6 rseed=2 lo=-400,-100,-100,-400,-170,-400 hi=400,100,100,400,-10,400"
# genserkins' inverse stops when every joint steps less than 1e-7
# radians, about 6e-6 degrees, not when the error is that small, so it
# comes back within about 1e-5 degrees, and 2e-5 leaves room for that
run "genserkins" "" \
    "joints=9 tolmant=2 tolexp=5 lo=-400,-100,-100,-400,10,-400 hi=400,100,100,400,170,400"
run "genserkins" "setp genserkins.unrotate-3 1" \
    "joints=9 tolmant=2 tolexp=5 lo=-400,-100,-100,-400,10,-400 hi=400,100,100,400,170,400"
# the sim arm's own limits on joint 0, and the elbow both ways
run "scarakins" "" "joints=6 lo=-360,-170,-100,-400,0,0 hi=360,170,100,400,0,0"
# scorbot-kins solves one arm: the wrist in front of the base and the
# elbow up, which with links of one length means joint 1 above joint 2.
# The box keeps to that arm.
run "scorbot-kins" "" "joints=5 lo=-150,40,-85,-90,-400 hi=150,90,30,90,400"

# rosekins is left out: its inverse counts the turns of joint 2 from the
# quadrants it saw on earlier calls, not from the joint it is handed, so
# it holds only along a path walked in small steps, not at points drawn
# at random

# parallel machines, from the pose side, the forward seeded a servo
# cycle away the way motion seeds it
run "tripodkins" \
    "setp tripodkins.Bx 2
setp tripodkins.Cx 1
setp tripodkins.Cy 2" \
    "joints=3 frompose=1 denom=10 lo=5,5,10 hi=15,15,30"
run "lineardeltakins" "" "joints=9 frompose=1 lo=-100,-100,-300 hi=100,100,-100"
run "rotarydeltakins" "" "joints=9 frompose=1 lo=-5,-5,-15 hi=5,5,-8"
run "genhexkins" "setp genhexkins.screw-lead 0" \
    "joints=6 frompose=1 lo=-5,-5,10,-10,-10,-15 hi=5,5,30,10,10,15"
# with a screw lead the strut lengths fold near y = 2 in this box, where
# two nearby poses have the same lengths; held below it
run "genhexkins" "setp genhexkins.screw-lead 5" \
    "joints=6 frompose=1 lo=-5,-5,10,-10,-10,-15 hi=5,1,30,10,10,15"
# the round trip holds in that box with or without the screw correction's
# rates in the forward's step; what they change is how fast it gets there.
# With them it takes as many steps as with no lead, 3 or 4; without
# them it took 12 to 14
case genhexkins in "${ONLY:-}"*)
    hal=$(mktemp --suffix=.hal)
    printf '%s\n' 'loadrt genhexkins' 'setp genhexkins.screw-lead 5' \
        'loadrt roundcheck joints=6 frompose=1 types=1 lo=-5,-5,10,-10,-10,-15 hi=5,1,30,10,10,15' \
        'getp genhexkins.max-iterations' > "$hal"
    iters=$(halrun -f "$hal" 2>/dev/null | grep -E '^[0-9]+$' | tail -1)
    rm -f "$hal"
    echo "genhexkins screw-lead 5: the forward took at most ${iters:-no} steps"
    # types=1: switching to the other types resets the count; and a count
    # of 0 means it was not read, not that the forward took no steps
    [ -n "$iters" ] && [ "$iters" -ge 1 ] && [ "$iters" -le 6 ] || status=1
    ;;
esac
# past a = 5 this geometry nears a singularity, where the forward can
# land on another pose with the same strut lengths; held below it
run "pentakins" "" "joints=5 frompose=1 lo=-20,-20,-10,-15,-15 hi=20,20,10,5,15"

exit $status

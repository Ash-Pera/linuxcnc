#!/bin/bash
# Round-trip a kinematics module the way motion calls it.  cases holds
# hal lines: each case loads the module, may set its pins, and ends with
# a check line, the roundcheck arguments for it.  The box is in joint
# space for the serial machines and in pose space (frompose=1) for the
# parallel ones and the gantry, whose forward is not one to one.
# Rotaries go past a whole turn wherever the module claims to follow
# the joint it is handed.  One directory per module, so runtests -j runs
# them side by side.
#
# rosekins has no directory: its inverse counts the turns of joint 2
# from the quadrants it saw on earlier calls, not from the joint it is
# handed, so it holds only along a path walked in small steps, not at
# points drawn at random.
set -e

[ -r cases ] || { echo "Error: file 'cases' does not exist"; exit 1; }

halcompile --install ../roundcheck.c >/dev/null

# every case runs, so one failure does not hide the next
status=0
hal=
trap 'rm -f case.hal' EXIT
while read -r line; do
    case "$line" in
    ''|'#'*) ;;
    check|check[[:space:]]*)
        [ -n "$hal" ] || { echo "Error: a check line with no loadrt before it"; exit 1; }
        printf '%s\nloadrt roundcheck %s\n' "$hal" "${line#check}" > case.hal
        echo "=== ${hal%%$'\n'*}"
        halrun -f case.hal < /dev/null || status=1
        hal= ;;
    *) hal="${hal:+$hal$'\n'}$line" ;;
    esac
done < cases
[ -z "$hal" ] || { echo "Error: the last case has no check line"; exit 1; }
exit $status

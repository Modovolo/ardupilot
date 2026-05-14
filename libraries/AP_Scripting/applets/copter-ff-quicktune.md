# Copter FF QuickTune

This applet is a Copter-only tuning workflow that runs a feed-forward first tune and can optionally run a conservative P/D refinement pass.

The script is designed for LOITER, ALT_HOLD, POSHOLD, or GUIDED.

## Requirements

- Copter firmware with Lua scripting enabled (`SCR_ENABLE=1`)
- A switch assigned to a scripting RC option (`RCx_OPTION=300` by default)
- Stable hover conditions and low wind

## Switch Behavior

The switch selected by `FFQT_RC_FUNC` controls workflow:

- Low position: stop tuning and revert any unsaved changes
- Mid or high position: run tuning

The script saves gains only when all enabled axes complete.

## Parameters

## FFQT_ENABLE

Enable script operation.

## FFQT_AXES

Bitmask for axes to tune:

- bit 0: roll
- bit 1: pitch
- bit 2: yaw

Default is roll and pitch (`3`).

## FFQT_RC_FUNC

Scripting RC function number used for tune control. Default `300`.

## FFQT_EXCITE

Excitation mode:

- `0`: pilot-assisted only
- `1`: hybrid (pilot-assisted plus optional scripted pulses in Guided)

## FFQT_RATE_MAX

Maximum command rate used for command normalization and pulse limiting.

## FFQT_CMD_MIN

Minimum command rate required before a response window is accepted.

## FFQT_WINDOW

Averaging window in seconds for response-ratio estimation.

## FFQT_RESP_TGT

Target measured-to-commanded response ratio.

## FFQT_RESP_TOL

Tolerance around response target for success counting.

## FFQT_FF_MIN / FFQT_FF_MAX

Bounding limits for FF updates in the FF stage.

## FFQT_ANG_MAX

Attitude error abort threshold in degrees. Set `0` to disable.

## FFQT_PILOT_PAUSE

Pause time after pilot stick movement.

## FFQT_MIN_SAMP

Minimum scheduler samples needed before processing one response window.

## FFQT_SUCC_WIN

Consecutive in-tolerance windows required to complete an axis.

## FFQT_OPTIONS

Bitmask options:

- bit 0: enable optional P/D refinement after FF stage
- bit 1: allow scripted pulses in Guided when `FFQT_EXCITE=1`

## FFQT_PD_DBL / FFQT_PD_OSC

Optional P/D refinement controls:

- doubling time for conservative gain increase
- slew-rate threshold used as an oscillation indicator

## FFQT_PULSE_RT

Scripted pulse rate in degrees/second for hybrid mode.

## FFQT_VERBOSE

Enable frequent progress messages.

## Operation

1. Arm and take off into a stable hover.
2. Move tune switch to mid/high.
3. For pilot-assisted operation, command one axis at a time with moderate stick input.
4. Wait for status text that axis FF is complete.
5. Keep switch in tune position until all selected axes complete.
6. On completion, gains are saved automatically.

If any safety condition fails, the script reverts unsaved changes and aborts.

## Notes

- This applet does not call Copter AUTOTUNE mode.
- This applet is not intended for QuadPlane.
- Scripted pulses are only attempted in Guided mode.

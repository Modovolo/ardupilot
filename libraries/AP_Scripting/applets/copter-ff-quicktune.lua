--[[
 Copter FF QuickTune

 A Copter-only tuning applet focused on feed-forward (FF) tuning first,
 with an optional conservative P/D refinement stage.

 Switch behavior (using FFQT_RC_FUNC):
  - low: stop/abort and revert unsaved gains
  - mid/high: run tuning workflow

 This script auto-saves only after all selected axes complete.
--]]

---@diagnostic disable: param-type-mismatch
---@diagnostic disable: need-check-nil
---@diagnostic disable: missing-parameter

local MAV_SEVERITY_EMERGENCY = 0
local MAV_SEVERITY_CRITICAL = 2
local MAV_SEVERITY_NOTICE = 5
local MAV_SEVERITY_INFO = 6

local MODE_ALT_HOLD = 2
local MODE_GUIDED = 4
local MODE_LOITER = 5
local MODE_POSHOLD = 16

local UPDATE_RATE_HZ = 40
local RAD_TO_DEG = 57.29577951308232
local EPSILON = 1.0e-6

local OPTIONS_ENABLE_PD_REFINE = (1 << 0)
local OPTIONS_SCRIPTED_PULSES = (1 << 1)

local PARAM_TABLE_KEY = 117
local PARAM_TABLE_PREFIX = "FFQT_"

function constrain(v, lo, hi)
    if v < lo then
        return lo
    end
    if v > hi then
        return hi
    end
    return v
end

function get_time()
    return millis():tofloat() * 0.001
end

function bind_param(name)
    local p = Parameter()
    assert(p:init(name), string.format("could not find %s", name))
    return p
end

function bind_add_param(name, idx, default_value)
    assert(param:add_param(PARAM_TABLE_KEY, idx, name, default_value), string.format("could not add param %s", name))
    return bind_param(PARAM_TABLE_PREFIX .. name)
end

assert(param:add_table(PARAM_TABLE_KEY, PARAM_TABLE_PREFIX, 20), "could not add param table")

--[[
  // @Param: FFQT_ENABLE
  // @DisplayName: Copter FF QuickTune enable
  // @Description: Enable Copter FF QuickTune script
  // @Values: 0:Disabled,1:Enabled
  // @User: Standard
--]]
local FFQT_ENABLE = bind_add_param("ENABLE", 1, 0)

--[[
  // @Param: FFQT_AXES
  // @DisplayName: Copter FF QuickTune axes
  // @Description: Axes to tune
  // @Bitmask: 0:Roll,1:Pitch,2:Yaw
  // @User: Standard
--]]
local FFQT_AXES = bind_add_param("AXES", 2, 3)

--[[
  // @Param: FFQT_RC_FUNC
  // @DisplayName: Copter FF QuickTune RC function
  // @Description: RCx_OPTION scripting function for tune control
  // @User: Standard
--]]
local FFQT_RC_FUNC = bind_add_param("RC_FUNC", 3, 300)

--[[
  // @Param: FFQT_EXCITE
  // @DisplayName: Copter FF QuickTune excitation mode
  // @Description: 0 is pilot-assisted only, 1 allows scripted pulses when enabled in options
  // @Values: 0:PilotOnly,1:Hybrid
  // @User: Standard
--]]
local FFQT_EXCITE = bind_add_param("EXCITE", 4, 1)

--[[
  // @Param: FFQT_RATE_MAX
  // @DisplayName: Copter FF QuickTune max command rate
  // @Description: Maximum axis command rate used by this script for response estimation and scripted pulses
  // @Range: 20 200
  // @Units: deg/s
  // @User: Standard
--]]
local FFQT_RATE_MAX = bind_add_param("RATE_MAX", 5, 60)

--[[
  // @Param: FFQT_CMD_MIN
  // @DisplayName: Copter FF QuickTune min command rate
  // @Description: Minimum command magnitude required for response sampling
  // @Range: 5 100
  // @Units: deg/s
  // @User: Standard
--]]
local FFQT_CMD_MIN = bind_add_param("CMD_MIN", 6, 15)

--[[
  // @Param: FFQT_WINDOW
  // @DisplayName: Copter FF QuickTune sample window
  // @Description: Response averaging window per sample
  // @Range: 0.3 4
  // @Units: s
  // @User: Standard
--]]
local FFQT_WINDOW = bind_add_param("WINDOW", 7, 1.2)

--[[
  // @Param: FFQT_RESP_TGT
  // @DisplayName: Copter FF QuickTune response target
  // @Description: Target measured-to-commanded rate ratio for FF convergence
  // @Range: 0.5 1.2
  // @User: Standard
--]]
local FFQT_RESP_TGT = bind_add_param("RESP_TGT", 8, 0.95)

--[[
  // @Param: FFQT_RESP_TOL
  // @DisplayName: Copter FF QuickTune response tolerance
  // @Description: Response ratio tolerance around target for success counting
  // @Range: 0.005 0.2
  // @User: Standard
--]]
local FFQT_RESP_TOL = bind_add_param("RESP_TOL", 9, 0.025)

--[[
  // @Param: FFQT_FF_MIN
  // @DisplayName: Copter FF QuickTune minimum FF
  // @Description: Minimum FF value allowed during FF stage
  // @Range: 0.0 0.5
  // @Units: s/rad
  // @User: Standard
--]]
local FFQT_FF_MIN = bind_add_param("FF_MIN", 10, 0.02)

--[[
  // @Param: FFQT_FF_MAX
  // @DisplayName: Copter FF QuickTune maximum FF
  // @Description: Maximum FF value allowed during FF stage
  // @Range: 0.0 0.5
  // @Units: s/rad
  // @User: Standard
--]]
local FFQT_FF_MAX = bind_add_param("FF_MAX", 11, 0.50)

--[[
  // @Param: FFQT_ANG_MAX
  // @DisplayName: Copter FF QuickTune max attitude error
  // @Description: Abort threshold for attitude error during tuning. Zero disables this check
  // @Range: 0 45
  // @Units: deg
  // @User: Standard
--]]
local FFQT_ANG_MAX = bind_add_param("ANG_MAX", 12, 10)

--[[
  // @Param: FFQT_PILOT_PAUSE
  // @DisplayName: Copter FF QuickTune pilot pause
  // @Description: Pause tuning this long after pilot stick movement
  // @Range: 0 10
  // @Units: s
  // @User: Standard
--]]
local FFQT_PILOT_PAUSE = bind_add_param("PILOT_PAUSE", 13, 3)

--[[
  // @Param: FFQT_MIN_SAMP
  // @DisplayName: Copter FF QuickTune min samples
  // @Description: Minimum number of loop samples required before processing a response window
  // @Range: 5 200
  // @User: Standard
--]]
local FFQT_MIN_SAMP = bind_add_param("MIN_SAMP", 14, 20)

--[[
  // @Param: FFQT_SUCC_WIN
  // @DisplayName: Copter FF QuickTune success windows
  // @Description: Number of consecutive in-tolerance windows needed per axis
  // @Range: 1 20
  // @User: Standard
--]]
local FFQT_SUCC_WIN = bind_add_param("SUCC_WIN", 15, 3)

--[[
  // @Param: FFQT_OPTIONS
  // @DisplayName: Copter FF QuickTune options
  // @Description: Additional options. Bit 0 enables optional conservative P/D refinement after FF stage. Bit 1 enables scripted pulse excitation in Guided mode when FFQT_EXCITE is 1
  // @Bitmask: 0:EnablePDRefine,1:ScriptedPulses
  // @User: Standard
--]]
local FFQT_OPTIONS = bind_add_param("OPTIONS", 16, 0)

--[[
  // @Param: FFQT_PD_DBL
  // @DisplayName: Copter FF QuickTune P/D doubling time
  // @Description: Time in seconds for gain doubling in optional P/D refinement stage
  // @Range: 5 40
  // @Units: s
  // @User: Standard
--]]
local FFQT_PD_DBL = bind_add_param("PD_DBL", 17, 14)

--[[
  // @Param: FFQT_PD_OSC
  // @DisplayName: Copter FF QuickTune P/D oscillation threshold
  // @Description: Slew-rate threshold for optional P/D refinement stage
  // @Range: 1 10
  // @User: Standard
--]]
local FFQT_PD_OSC = bind_add_param("PD_OSC", 18, 5)

--[[
  // @Param: FFQT_PULSE_RT
  // @DisplayName: Copter FF QuickTune pulse rate
  // @Description: Scripted pulse command rate for hybrid excitation in Guided mode
  // @Range: 5 200
  // @Units: deg/s
  // @User: Standard
--]]
local FFQT_PULSE_RT = bind_add_param("PULSE_RT", 19, 40)

--[[
  // @Param: FFQT_VERBOSE
  // @DisplayName: Copter FF QuickTune verbose messaging
  // @Description: Enable additional status messages
  // @Values: 0:Disabled,1:Enabled
  // @User: Standard
--]]
local FFQT_VERBOSE = bind_add_param("VERBOSE", 20, 0)

if not param:get("ATC_RAT_RLL_FF") then
    gcs:send_text(MAV_SEVERITY_EMERGENCY, "FFQT: not a Copter rate-controller target")
    return
end

if param:get("Q_A_RAT_RLL_FF") then
    gcs:send_text(MAV_SEVERITY_EMERGENCY, "FFQT: Copter only, not QuadPlane")
    return
end

local RCMAP_ROLL = bind_param("RCMAP_ROLL")
local RCMAP_PITCH = bind_param("RCMAP_PITCH")
local RCMAP_YAW = bind_param("RCMAP_YAW")

local RCIN_ROLL = rc:get_channel(RCMAP_ROLL:get())
local RCIN_PITCH = rc:get_channel(RCMAP_PITCH:get())
local RCIN_YAW = rc:get_channel(RCMAP_YAW:get())

local axis_names = { "RLL", "PIT", "YAW" }
local axis_masks = { RLL = 1, PIT = 2, YAW = 4 }
local axis_to_index = { RLL = 1, PIT = 2, YAW = 3 }

local gains = {}
local gain_saved = {}
local gain_changed = {}

for _, axis in ipairs(axis_names) do
    for _, suffix in ipairs({ "FF", "P", "I", "D" }) do
        local pname = axis .. "_" .. suffix
        gains[pname] = bind_param("ATC_RAT_" .. pname)
        gain_changed[pname] = false
    end
end

local STATE_IDLE = 0
local STATE_FF = 1
local STATE_PD = 2
local STATE_COMPLETE = 3
local STATE_ABORTED = 4

local state = STATE_IDLE
local need_restore = false
local aborted = false
local save_done = false

local last_warning = get_time()
local last_info = get_time()
local last_pilot_input = get_time()

local axis_done = {}
local axis_success_windows = {}
local pd_stage = "D"

local sample_axis = nil
local sample_start = 0
local sample_cmd_sum = 0
local sample_rsp_sum = 0
local sample_count = 0

local pulse_axis = nil
local pulse_sign = 1
local pulse_next_switch = 0

function verbose_enabled()
    return FFQT_VERBOSE:get() > 0
end

function send_status(severity, text)
    gcs:send_text(severity, text)
end

function mode_allowed(mode)
    return mode == MODE_ALT_HOLD or mode == MODE_LOITER or mode == MODE_POSHOLD or mode == MODE_GUIDED
end

function axis_enabled(axis)
    return (FFQT_AXES:get() & axis_masks[axis]) ~= 0
end

function reset_axis_state()
    for _, axis in ipairs(axis_names) do
        axis_done[axis] = false
        axis_success_windows[axis] = 0
    end
    pd_stage = "D"
end

function reset_sample()
    sample_axis = nil
    sample_start = 0
    sample_cmd_sum = 0
    sample_rsp_sum = 0
    sample_count = 0
end

function get_current_axis()
    for _, axis in ipairs(axis_names) do
        if axis_enabled(axis) and not axis_done[axis] then
            return axis
        end
    end
    return nil
end

function get_all_gains()
    for pname in pairs(gains) do
        gain_saved[pname] = gains[pname]:get()
    end
end

function restore_all_gains()
    for pname in pairs(gains) do
        if gain_changed[pname] then
            gains[pname]:set(gain_saved[pname])
            gain_changed[pname] = false
        end
    end
end

function save_all_gains()
    for pname in pairs(gains) do
        if gain_changed[pname] then
            local current = gains[pname]:get()
            gains[pname]:set_and_save(current)
            gain_saved[pname] = current
            gain_changed[pname] = false
        end
    end
end

function adjust_gain(pname, value)
    gains[pname]:set(value)
    gain_changed[pname] = true
end

function have_pilot_input()
    return math.abs(RCIN_ROLL:norm_input_dz()) > 0 or
           math.abs(RCIN_PITCH:norm_input_dz()) > 0 or
           math.abs(RCIN_YAW:norm_input_dz()) > 0
end

function dominant_pilot_axis_command()
    local roll_cmd = RCIN_ROLL:norm_input_dz() * FFQT_RATE_MAX:get()
    local pitch_cmd = RCIN_PITCH:norm_input_dz() * FFQT_RATE_MAX:get()
    local yaw_cmd = RCIN_YAW:norm_input_dz() * FFQT_RATE_MAX:get()

    local axis = "RLL"
    local cmd = roll_cmd
    if math.abs(pitch_cmd) > math.abs(cmd) then
        axis = "PIT"
        cmd = pitch_cmd
    end
    if math.abs(yaw_cmd) > math.abs(cmd) then
        axis = "YAW"
        cmd = yaw_cmd
    end

    if math.abs(cmd) < FFQT_CMD_MIN:get() then
        return nil, 0
    end
    return axis, cmd
end

function get_axis_rate_dps(axis)
    local gyro = ahrs:get_gyro()
    if gyro == nil then
        return nil
    end

    if axis == "RLL" then
        return gyro:x() * RAD_TO_DEG
    end
    if axis == "PIT" then
        return gyro:y() * RAD_TO_DEG
    end
    return gyro:z() * RAD_TO_DEG
end

function get_axis_slew_rate(axis)
    local roll_srate, pitch_srate, yaw_srate = AC_AttitudeControl:get_rpy_srate()
    if axis == "RLL" then
        return roll_srate
    end
    if axis == "PIT" then
        return pitch_srate
    end
    return yaw_srate
end

function scripted_pulse_enabled()
    return FFQT_EXCITE:get() > 0 and (FFQT_OPTIONS:get() & OPTIONS_SCRIPTED_PULSES) ~= 0
end

function get_scripted_axis_command(axis)
    if not scripted_pulse_enabled() then
        return 0, false
    end
    if vehicle:get_mode() ~= MODE_GUIDED then
        return 0, false
    end
    if have_pilot_input() then
        return 0, false
    end

    local now = get_time()
    if pulse_axis ~= axis then
        pulse_axis = axis
        pulse_sign = 1
        pulse_next_switch = now + 0.8
    elseif now >= pulse_next_switch then
        pulse_sign = -pulse_sign
        pulse_next_switch = now + 0.8
    end

    local cmd_rate = constrain(FFQT_PULSE_RT:get(), FFQT_CMD_MIN:get(), FFQT_RATE_MAX:get()) * pulse_sign

    local roll_rate = 0
    local pitch_rate = 0
    local yaw_rate = 0
    if axis == "RLL" then
        roll_rate = cmd_rate
    elseif axis == "PIT" then
        pitch_rate = cmd_rate
    else
        yaw_rate = cmd_rate
    end

    local throttle = vehicle:get_control_output(3)
    if throttle == nil then
        throttle = 0.5
    end
    throttle = constrain(throttle, 0.2, 0.8)

    local ok = vehicle:set_target_rate_and_throttle(roll_rate, pitch_rate, yaw_rate, throttle)
    if not ok then
        return 0, false
    end
    return cmd_rate, true
end

function response_scale(target_ratio, measured_ratio, tolerance)
    if measured_ratio <= 0.001 then
        return 1.25
    end

    local ratio_error = target_ratio - measured_ratio
    local abs_error = math.abs(ratio_error)
    if abs_error > 0.2 then
        return constrain(target_ratio / measured_ratio, 0.75, 1.25)
    end

    if measured_ratio < target_ratio - 0.1 then
        return 1.05
    end
    if measured_ratio > target_ratio + 0.1 then
        return 0.95
    end

    if measured_ratio < target_ratio - tolerance then
        return 1.02
    end
    if measured_ratio > target_ratio + tolerance then
        return 0.98
    end

    return 1.0
end

function update_response_window(axis, cmd_rate_abs)
    if cmd_rate_abs < FFQT_CMD_MIN:get() then
        return nil, nil, nil
    end

    local rsp_rate = get_axis_rate_dps(axis)
    if rsp_rate == nil then
        return nil, nil, nil
    end

    local now = get_time()
    if sample_axis ~= axis then
        sample_axis = axis
        sample_start = now
        sample_cmd_sum = 0
        sample_rsp_sum = 0
        sample_count = 0
    end

    sample_cmd_sum = sample_cmd_sum + cmd_rate_abs
    sample_rsp_sum = sample_rsp_sum + math.abs(rsp_rate)
    sample_count = sample_count + 1

    local min_samples = math.max(5, FFQT_MIN_SAMP:get())
    if now - sample_start < FFQT_WINDOW:get() or sample_count < min_samples then
        return nil, nil, nil
    end

    local avg_cmd = sample_cmd_sum / sample_count
    local avg_rsp = sample_rsp_sum / sample_count
    local ratio = avg_rsp / math.max(avg_cmd, EPSILON)

    reset_sample()
    return ratio, avg_cmd, avg_rsp
end

function apply_ff_update(axis, ratio, avg_cmd, avg_rsp)
    local pname = axis .. "_FF"
    local ff = gains[pname]:get()
    local ff_min = math.max(0.0, FFQT_FF_MIN:get())
    local ff_max = constrain(FFQT_FF_MAX:get(), ff_min, 0.5)
    local tgt = constrain(FFQT_RESP_TGT:get(), 0.5, 1.2)
    local tol = constrain(FFQT_RESP_TOL:get(), 0.005, 0.2)

    local scale = response_scale(tgt, ratio, tol)
    local new_ff = constrain(ff * scale, ff_min, ff_max)

    if math.abs(scale - 1.0) <= 0.0001 then
        axis_success_windows[axis] = axis_success_windows[axis] + 1
    else
        axis_success_windows[axis] = 0
        if math.abs(new_ff - ff) > EPSILON then
            adjust_gain(pname, new_ff)
        end
    end

    logger:write("FFQT", "Axis,Cmd,Rsp,Ratio,FF", "fffff",
                 axis_to_index[axis], avg_cmd, avg_rsp, ratio, gains[pname]:get())

    if verbose_enabled() then
        send_status(MAV_SEVERITY_INFO,
                    string.format("FFQT %s ff=%.4f ratio=%.3f cmd=%.1f rsp=%.1f win=%u",
                                  axis, gains[pname]:get(), ratio, avg_cmd, avg_rsp, axis_success_windows[axis]))
    end

    if axis_success_windows[axis] >= FFQT_SUCC_WIN:get() then
        axis_done[axis] = true
        axis_success_windows[axis] = 0
        reset_sample()
        send_status(MAV_SEVERITY_NOTICE,
                    string.format("FFQT: %s FF done (%.4f)", axis, gains[pname]:get()))
    end
end

function update_ff_stage()
    local axis = get_current_axis()
    if axis == nil then
        if (FFQT_OPTIONS:get() & OPTIONS_ENABLE_PD_REFINE) ~= 0 then
            reset_axis_state()
            state = STATE_PD
            send_status(MAV_SEVERITY_NOTICE, "FFQT: FF complete, starting optional P/D refine")
        else
            state = STATE_COMPLETE
        end
        return
    end

    local cmd_axis, cmd_rate = dominant_pilot_axis_command()
    local command_used = false

    if cmd_axis == axis then
        command_used = true
    elseif FFQT_EXCITE:get() > 0 and (get_time() - last_pilot_input) >= FFQT_PILOT_PAUSE:get() then
        cmd_rate, command_used = get_scripted_axis_command(axis)
    end

    if not command_used then
        if verbose_enabled() and get_time() - last_info > 2.0 then
            last_info = get_time()
            send_status(MAV_SEVERITY_INFO, string.format("FFQT: command %s to continue", axis))
        end
        return
    end

    local ratio, avg_cmd, avg_rsp = update_response_window(axis, math.abs(cmd_rate))
    if ratio ~= nil then
        apply_ff_update(axis, ratio, avg_cmd, avg_rsp)
    end
end

function pd_gain_mul()
    return math.exp(math.log(2.0) / (UPDATE_RATE_HZ * constrain(FFQT_PD_DBL:get(), 5, 40)))
end

function update_pd_stage()
    local axis = get_current_axis()
    if axis == nil then
        state = STATE_COMPLETE
        return
    end

    local cmd_axis, cmd_rate = dominant_pilot_axis_command()
    local command_used = false

    if cmd_axis == axis then
        command_used = true
    elseif FFQT_EXCITE:get() > 0 and (get_time() - last_pilot_input) >= FFQT_PILOT_PAUSE:get() then
        cmd_rate, command_used = get_scripted_axis_command(axis)
    end

    if not command_used then
        return
    end

    if math.abs(cmd_rate) < FFQT_CMD_MIN:get() then
        return
    end

    local pname = axis .. "_" .. pd_stage
    local current_gain = gains[pname]:get()
    local srate = get_axis_slew_rate(axis)

    if srate >= FFQT_PD_OSC:get() then
        local new_gain = math.max(current_gain * 0.6, 0.0001)
        adjust_gain(pname, new_gain)
        logger:write("FFQT", "Axis,Stage,SRate,Gain", "ffff",
                     axis_to_index[axis], (pd_stage == "D") and 1 or 2, srate, new_gain)

        if pd_stage == "D" then
            pd_stage = "P"
            send_status(MAV_SEVERITY_INFO, string.format("FFQT: %s D done", axis))
        else
            pd_stage = "D"
            axis_done[axis] = true
            send_status(MAV_SEVERITY_NOTICE, string.format("FFQT: %s P/D done", axis))
        end
        return
    end

    local new_gain = current_gain * pd_gain_mul()
    if new_gain <= 0.0001 then
        new_gain = 0.001
    end
    adjust_gain(pname, new_gain)

    if verbose_enabled() and get_time() - last_info > 3.0 then
        last_info = get_time()
        send_status(MAV_SEVERITY_INFO,
                    string.format("FFQT %s_%s gain %.4f sr=%.2f", axis, pd_stage, new_gain, srate))
    end
end

function clear_runtime_state()
    reset_axis_state()
    reset_sample()
    pulse_axis = nil
    pulse_sign = 1
    pulse_next_switch = 0
end

function begin_tune()
    if need_restore then
        return
    end
    clear_runtime_state()
    get_all_gains()
    need_restore = true
    save_done = false
    state = STATE_FF
    send_status(MAV_SEVERITY_NOTICE, "FFQT: starting FF tune")
end

function abort_tune(reason, severity)
    clear_runtime_state()
    if need_restore then
        restore_all_gains()
        need_restore = false
    end
    save_done = false
    aborted = true
    state = STATE_ABORTED
    send_status(severity, "FFQT: " .. reason)
end

function complete_and_save()
    if save_done then
        return
    end
    if need_restore then
        save_all_gains()
        need_restore = false
    end
    save_done = true
    send_status(MAV_SEVERITY_NOTICE, "FFQT: tune complete, gains saved")
end

function in_tune_position(sw_pos)
    return sw_pos == 1 or sw_pos == 2
end

function enforce_safety()
    if not arming:is_armed() or not vehicle:get_likely_flying() then
        if need_restore then
            abort_tune("reverted, must be flying", MAV_SEVERITY_EMERGENCY)
        else
            clear_runtime_state()
            state = STATE_IDLE
        end
        return false
    end

    local mode = vehicle:get_mode()
    if not mode_allowed(mode) then
        abort_tune("mode not supported for tuning", MAV_SEVERITY_CRITICAL)
        return false
    end

    if vehicle:has_ekf_failsafed() then
        abort_tune("EKF failsafe", MAV_SEVERITY_CRITICAL)
        return false
    end

    if FFQT_ANG_MAX:get() > 0 then
        local att_error = AC_AttitudeControl:get_att_error_angle_deg()
        if att_error > FFQT_ANG_MAX:get() then
            abort_tune(string.format("attitude error %.1fdeg", att_error), MAV_SEVERITY_CRITICAL)
            return false
        end
    end

    return true
end

function update()
    if FFQT_ENABLE:get() < 1 then
        return
    end

    local sw_pos = rc:get_aux_cached(FFQT_RC_FUNC:get())
    if sw_pos == nil then
        if get_time() - last_warning > 5.0 then
            last_warning = get_time()
            send_status(MAV_SEVERITY_INFO, "FFQT: waiting for RC function switch")
        end
        return
    end

    if aborted then
        if sw_pos == 0 then
            aborted = false
            state = STATE_IDLE
            send_status(MAV_SEVERITY_NOTICE, "FFQT: reset")
        end
        return
    end

    if sw_pos == 0 then
        if need_restore then
            abort_tune("reverted by switch", MAV_SEVERITY_NOTICE)
            return
        end
        if state == STATE_COMPLETE then
            clear_runtime_state()
            save_done = false
            state = STATE_IDLE
            send_status(MAV_SEVERITY_NOTICE, "FFQT: ready")
        end
        return
    end

    if have_pilot_input() then
        last_pilot_input = get_time()
    end

    if not in_tune_position(sw_pos) then
        return
    end

    if not enforce_safety() then
        return
    end

    if state == STATE_IDLE then
        begin_tune()
        return
    end

    if state == STATE_COMPLETE then
        complete_and_save()
        return
    end

    local scripted_allowed = FFQT_EXCITE:get() > 0 and scripted_pulse_enabled() and vehicle:get_mode() == MODE_GUIDED
    if not scripted_allowed and (get_time() - last_pilot_input) < FFQT_PILOT_PAUSE:get() then
        return
    end

    if state == STATE_FF then
        update_ff_stage()
    elseif state == STATE_PD then
        update_pd_stage()
    end

    if state == STATE_COMPLETE then
        complete_and_save()
    end
end

function protected_wrapper()
    local ok, err = pcall(update)
    if not ok then
        send_status(MAV_SEVERITY_EMERGENCY, "FFQT internal error: " .. err)
        return protected_wrapper, 1000
    end
    return protected_wrapper, 1000 / UPDATE_RATE_HZ
end

send_status(MAV_SEVERITY_NOTICE, "FFQT: Copter FF QuickTune loaded")
return protected_wrapper()

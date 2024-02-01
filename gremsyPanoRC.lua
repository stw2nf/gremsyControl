-- Script triggers Pano on Gremsy gimbal
-- SBUS control

local CAM_CHAN = 6 -- Servo 7        |  Camera Trigger  | Pull HIGH to Trigger |
local GEO_START_CHAN = 7 -- Servo 8  | GeoTagging START | Pull HIGH to Trigger |
local AXIS_SEL_CHAN = 8 -- Servo 9   |    Axis Toggle   |  Pull HIGH to Toggle |
local GIMB_CHAN = 9 -- Servo 10     |     Yaw/Pitch    |    PWM Rate Control  | Long press sends both axes home
local GEO_STOP_CHAN = 11 -- Servo 12 |  GeoTagging STOP | Pull HIGH to Trigger |

local PARAM_TABLE_KEY = 73
assert(param:add_table(PARAM_TABLE_KEY, "PANO_", 3), 'could not add param table')
assert(param:add_param(PARAM_TABLE_KEY, 1, 'SPEED', 90), 'could not add PANO_SPEED') -- Speed of gimbal (deg/s)
assert(param:add_param(PARAM_TABLE_KEY, 2, 'YSTEP', 15), 'could not add PANO_YSTEP') -- Yaw step (deg)
assert(param:add_param(PARAM_TABLE_KEY, 3, 'PSTEP', 30), 'could not add PANO_PSTEP') -- Pitch step (deg)

local pano_speed = param:get("PANO_SPEED") -- Speed of gimbal (deg/s)
local yaw_step_size = param:get("PANO_YSTEP") -- Yaw step size (deg)
local pitch_step_size = param:get("PANO_PSTEP") -- Pitch step size (deg)
local yaw_time_step = math.floor((yaw_step_size/pano_speed)*1000) -- Time to write yaw command to move yaw step (ms)
local pitch_time_step = math.floor((pitch_step_size/pano_speed)*1000) -- Time to write yaw command to move pitch step (ms)
local pitch_time -- Actual time used to write a pitch command
local pitch_cmd -- Actual command to send for pitch
local pitch_state = 0 -- 0 = Neutral, 1 = Up, -1 = Down
local des_pitch_state = 1 -- 0 = Neutral, 1 = Up, -1 = Down
local cur_yaw_step = 0 -- Current yaw step of a pano
local tot_yaw_step = math.floor(360/yaw_step_size) -- Total yaw steps of pano based on step size

local YAW_AXIS = "yaw"
local PITCH_AXIS = "pitch"
local cur_axis = PITCH_AXIS

local RC_PANO_TRIG_CHAN = rc:find_channel_for_option(300) -- Manual Pano Trigger on RC
local rc_pano = 0
local rc_pano_prev = 0
local axis_sel_out = 0
local axis_sel_timeout = 1
local prevAxisSel = 0

local toggleStart = 0
local trigCamStart = 0
local geoStart = 0
local geoStop = 0
local homeStart = 0
local yawStart = 0
local pitchStart = 0
local rcParam = 0

local GPIO = -1
local RCPassThru = 1

function updateRCParams() -- Update parameters for RC Control
    if rcParam == 0 then
        param:set('SERVO7_FUNCTION',RCPassThru)
        param:set('SERVO8_FUNCTION',RCPassThru)
        param:set('SERVO9_FUNCTION',RCPassThru)
        param:set('SERVO10_FUNCTION',RCPassThru)
        param:set('SERVO12_FUNCTION',RCPassThru)
        gcs:send_text(3, 'Updated RC Params')
        rcParam = 1
    end
end

function updateScriptParams() -- Update parameters for Scripted Control
    pano_speed = param:get("PANO_SPEED")
    yaw_step_size = param:get("PANO_YSTEP")
    pitch_step_size = param:get("PANO_PSTEP")
    yaw_time_step = math.floor((yaw_step_size/pano_speed)*1000)
    pitch_time_step = math.floor((pitch_step_size/pano_speed)*1000)
    tot_yaw_step = math.floor(360/yaw_step_size)
    param:set('SERVO7_FUNCTION',GPIO)
    param:set('SERVO8_FUNCTION',GPIO)
    param:set('SERVO9_FUNCTION',GPIO)
    param:set('SERVO10_FUNCTION',GPIO)
    param:set('SERVO12_FUNCTION',GPIO)
    rcParam = 0
    gcs:send_text(3, 'Updated Scripted Params')
end

function stopGeoTag() -- Triggers Servo 12, Geotag Stop
    SRV_Channels:set_output_pwm_chan_timeout(GEO_STOP_CHAN, 1900, 500)
    gcs:send_text(3, 'Stopping GeoTag')
end

function startGeoTag() -- Triggers Servo 8, Geotag Start
    SRV_Channels:set_output_pwm_chan_timeout(GEO_START_CHAN, 1900, 500)
    gcs:send_text(3, 'Starting GeoTag')
end

function triggerCamera() -- Triggers Servo 7, Camera
    if trigCamStart == 0 then
        gcs:send_text(3, 'Triggering Camera')
        SRV_Channels:set_output_pwm_chan_timeout(CAM_CHAN, 1900, 500)
        trigCamStart = 1
        return triggerCamera, 500
    else
        SRV_Channels:set_output_pwm_chan_timeout(CAM_CHAN, 1100, 500)
        trigCamStart = 0
    end
    return runPano, 1000
end

function toggleAxis() -- Toggle Servo 9, Axis Select
    if toggleStart == 0 then
        gcs:send_text(3, 'Toggling Axis')
        SRV_Channels:set_output_pwm_chan_timeout(AXIS_SEL_CHAN, 1900, 500)
        toggleStart = 1
        return toggleAxis, 500
    else
        SRV_Channels:set_output_pwm_chan_timeout(AXIS_SEL_CHAN, 1100, 500)
        toggleStart = 0
    end
    if cur_axis == YAW_AXIS then -- Keep track of which axis we are controlling
        cur_axis = PITCH_AXIS
        gcs:send_text(3, 'Switched to '..tostring(PITCH_AXIS))
        return movePitch, 1
    else
        cur_axis = YAW_AXIS
        gcs:send_text(3, 'Switched to '..tostring(YAW_AXIS))
        return moveYaw, 1
    end
end

function writeAxis(command, time) -- Servo 10, Write command to current gimbal axis
    gcs:send_text(3, 'Writing PWM: '..tostring(command)..' Time: '..tostring(time))
    SRV_Channels:set_output_pwm_chan_timeout(GIMB_CHAN, command, time)
end

function moveYaw() -- Move yaw axis
    if cur_axis ~= YAW_AXIS then -- If we aren't controlling yaw, make it so
        return toggleAxis,1
    end
    if yawStart == 0 then
        writeAxis(1900, yaw_time_step)
        yawStart = 1
        gcs:send_text(3, 'Moving Yaw, Step: '..tostring(cur_yaw_step))
        return moveYaw, yaw_time_step
    else
        writeAxis(1500, yaw_time_step)
        yawStart = 0
    end

    cur_yaw_step = cur_yaw_step + 1 -- Iterate yaw step
    return movePitch, 2*yaw_time_step
end

function movePitch() -- Move pitch axis
    if cur_axis ~= PITCH_AXIS then -- If we aren't controlling pitch, make it so
        return toggleAxis,1
    end
    if des_pitch_state-pitch_state < 0 then -- This logic decides move up/down
        pitch_cmd = 1100
    else
        pitch_cmd = 1900
    end
    pitch_time = math.abs(des_pitch_state-pitch_state)*pitch_time_step -- At home we only move for 1 step, after that we move 2 steps. This handles the time difference
    if pitchStart == 0 then
        writeAxis(pitch_cmd, pitch_time)
        pitchStart = 1
        gcs:send_text(3, 'Moving Pitch, State: '..tostring(pitch_state))
        return movePitch, pitch_time
    else
        writeAxis(1500, pitch_time)
        pitchStart = 0
    end
    pitch_state = des_pitch_state -- Update currrent pitch
    if des_pitch_state == 1 then -- Flip desired pitch for next step
        des_pitch_state = -1
    else
        des_pitch_state = 1
    end
    return triggerCamera, 3000
end

function writeHome() -- Send both axes home
    if homeStart == 0 then
        gcs:send_text(3, 'Resetting to Home')
        SRV_Channels:set_output_pwm_chan_timeout(AXIS_SEL_CHAN, 1900, 2000)
        homeStart = 1
        return writeHome, 2000
    else
        SRV_Channels:set_output_pwm_chan_timeout(AXIS_SEL_CHAN, 1100, 500)
        homeStart = 0
        pitch_state = 0
        cur_yaw_step = 0
        cur_axis = PITCH_AXIS
    end
    return update, 1000
end

function runPano() -- Run Pano
    gcs:send_text(3, 'Yaw Step: '..tostring(cur_yaw_step)..' of '..tostring(tot_yaw_step))
    if cur_yaw_step == 0 then
        gcs:send_text(3, 'Beginning Pano')
        cur_yaw_step = cur_yaw_step + 1 -- Iterate yaw step
        return movePitch, 1
    end
    if cur_yaw_step >= tot_yaw_step then
        return writeHome, 1
    else
        return moveYaw, 1000
    end
end

function update() -- Monitor RC Commands for Pano/Ortho Triggers
    rc_pano_prev = rc_pano
    rc_pano = RC_PANO_TRIG_CHAN:get_aux_switch_pos()
    if rc_pano~=rc_pano_prev and rc_pano == 2 then
        updateScriptParams()
        return writeHome, 1
    end
    if rc_pano == 2 then
        return runPano, 1
    else
        updateRCParams()
        return update, 500
    end
end

return update, 5000
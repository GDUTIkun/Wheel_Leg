# STM32 State Capture Analysis Notes

## 2026-06-27 Static Test Round 1

### Scope

- Data files:
  - [joints_static.summary.txt](/home/tan/wheel%20_leg_ws/logs/stm32_state_capture/joints_static.summary.txt)
  - [leg_static.summary.txt](/home/tan/wheel%20_leg_ws/logs/stm32_state_capture/leg_static.summary.txt)
  - [imu_static.summary.txt](/home/tan/wheel%20_leg_ws/logs/stm32_state_capture/imu_static.summary.txt)
- Sampling rate was stable at about `200 Hz`.
- UART / ROS capture chain looked healthy for this round:
  - `frames_ok` matched sample count
  - `crc_errors = 0`
  - `length_errors = 0`
  - `sync_losses = 0`

### User Context

- IMU angular velocity is intentionally filtered inside the sensor and includes a deadband.
- Therefore, `gyro_x / gyro_y / gyro_z == 0` while static is expected for this project.
- IMU filtering conclusions should not be based on this static-zero gyro behavior alone.

### Static Noise Observations

- Joint position noise is small and usable:
  - hip and knee position variation is present but very small
  - static spread is on the order of about `1e-4 ~ 4e-4 rad`
- Joint velocity noise is visible and larger than position noise:
  - left/right hip velocity std is about `0.0067 ~ 0.0073 rad/s`
  - left/right knee velocity std is about `0.010 ~ 0.013 rad/s`
  - knee velocity is noisier than hip velocity
- Leg geometry states are already very clean:
  - `phi_std ~= 0.002 deg`
  - `length_std ~= 0.005 ~ 0.006 mm`
- Differentiated leg quantities are noisier, which is expected:
  - `length_rate_raw_std ~= 1.1 ~ 1.35 mm/s`
  - LPF replay reduces `length_rate` noise substantially

### Current Conclusions

- `joint_pos`: no extra low-pass needed for now
- `joint_vel`: likely needs low-pass, especially knee velocity
- `phi`: no extra low-pass needed for now
- `length`: no extra low-pass needed for now
- `phi_rate`: light filtering is reasonable
- `length_rate`: filtering is recommended

### Recommended First-Pass Filter Targets

- `hip_vel`: start around `20 Hz`
- `knee_vel`: start around `15 Hz`
- `phi_rate`: start around `20 Hz`
- `length_rate`: start around `10 Hz`
- `joint_pos`: start with no filter
- `phi`: start with no filter
- `length`: start with no filter

### Notes For Next Test

- Next priority is dynamic data, not another static-only decision.
- Recommended sequence:
  1. joint dynamic
  2. leg dynamic
  3. IMU dynamic
- For dynamic tests, begin with small and slow motions first so that noise and delay can be separated.

## 2026-06-27 Dynamic Test Round 1

### Scope

- Data files:
  - [joints_dynamic.summary.txt](/home/tan/wheel%20_leg_ws/logs/stm32_state_capture/joints_dynamic.summary.txt)
  - [leg_dynamic.summary.txt](/home/tan/wheel%20_leg_ws/logs/stm32_state_capture/leg_dynamic.summary.txt)
- IMU conclusions were intentionally skipped for this round.
- Capture quality remained healthy:
  - sample rate stayed near `200 Hz`
  - `crc_errors = 0`
  - `length_errors = 0`
  - `sync_losses = 0`

### Dynamic Observations

- Joint motion is large enough now to separate real motion from static noise.
- Joint velocity carries real motion content and should not be filtered too heavily.
- Knee velocity is still the noisiest joint-velocity channel, but dynamic content is now much larger than the static noise floor.
- Leg geometry channels are smooth enough:
  - `phi` and `length` track motion cleanly
  - no sign that their base signals need heavy low-pass filtering
- Rate-like leg channels still show visible amplification of noise:
  - `phi_rate_raw` contains useful motion but should only get light filtering
  - `length_rate_raw` is clearly noisier and benefits from filtering

### Practical Conclusions

- `joint_pos`:
  - keep unfiltered
- `joint_vel`:
  - filter lightly only
  - avoid strong low-pass or the motion response will become sluggish
- `phi`:
  - keep unfiltered
- `length`:
  - keep unfiltered
- `phi_rate`:
  - light low-pass is appropriate
- `length_rate`:
  - low-pass is recommended

### Updated Filter Starting Points

- `hip_vel`: `20 ~ 25 Hz`
- `knee_vel`: `15 ~ 20 Hz`
- `wheel_vel`: wait for a wheel-focused dynamic test before finalizing
- `phi_rate`: `15 ~ 20 Hz`
- `length_rate`: `8 ~ 12 Hz`
- `joint_pos`: no filter
- `phi`: no filter
- `length`: no filter

### What This Round Changed

- Static data suggested where noise exists.
- Dynamic data confirmed that:
  - `joint_vel` should be filtered only lightly
  - `phi_rate` can tolerate light filtering
  - `length_rate` is the clearest candidate for stronger filtering
  - `phi` and `length` themselves do not currently justify extra low-pass filtering

### Next Useful Test

- If the next goal is parameter finalization, the most useful follow-up is:
  1. repeat one small/slow dynamic run
  2. repeat one slightly faster dynamic run
- That will let us judge not only noise suppression, but also whether the chosen cutoff introduces unacceptable delay.

## 2026-06-28 LPF Validation Pass

### Scope

- Data files:
  - [joints_static_lpf.summary.txt](/mnt/d/STM32%20depository/stm32item/wheel_leg_ws/logs/stm32_state_capture/joints_static_lpf.summary.txt)
  - [joints_dynamic_lpf.summary.txt](/mnt/d/STM32%20depository/stm32item/wheel_leg_ws/logs/stm32_state_capture/joints_dynamic_lpf.summary.txt)
  - [joints_static_lpf.csv](/mnt/d/STM32%20depository/stm32item/wheel_leg_ws/logs/stm32_state_capture/joints_static_lpf.csv)
  - [joints_dynamic_lpf.csv](/mnt/d/STM32%20depository/stm32item/wheel_leg_ws/logs/stm32_state_capture/joints_dynamic_lpf.csv)
- Sampling rate remained near `200 Hz`.
- Capture quality remained healthy:
  - `crc_errors = 0`
  - `length_errors = 0`
  - `sync_losses = 0`

### Main Findings

- `joint_vel` low-pass is active and effective.
  - In static data, joint velocity std drops to about `60% ~ 66%` of raw for hip and knee.
  - In dynamic data, low-frequency motion below about `3 Hz` is almost fully preserved.
  - High-frequency power above about `20 Hz` is strongly attenuated, while motion-band content is mostly kept.
- `phi_rate` filtering is clearly effective and still looks acceptable.
  - Dynamic std drops to about `68% ~ 72%` of raw.
  - Motion below about `3 Hz` is preserved, while `10 Hz+` content is heavily reduced.
- `length_rate` filtering is the strongest and is doing the most useful cleanup.
  - In static data, std drops to about `23% ~ 24%` of raw.
  - In dynamic data, low-frequency content is preserved, but `10 Hz+` content is reduced to about `10%` of raw power.

### Delay / Over-Filtering Check

- `joint_vel`:
  - estimated delay is about `5 ms` for hip and knee, about `10 ms` for wheel
  - dynamic 99% span stays at about `96% ~ 100%` of raw
  - conclusion: not obviously over-filtered
- `phi_rate`:
  - no obvious whole-sample lag from correlation check
  - dynamic 99% span is about `62%` of raw
  - conclusion: filtering is noticeable but still reasonable
- `length_rate`:
  - no obvious whole-sample lag from correlation check
  - dynamic 99% span is about `62%` of raw
  - conclusion: strong filtering, but still not clearly excessive for this signal class

### Suspected Weak / Strong Areas

- `joint_vel` is not under-filtered in the frequency domain:
  - dynamic `20 ~ 50 Hz` power is already reduced to about `19% ~ 26%` for hip/knee
  - wheel velocity is filtered even more strongly in high frequency
- `joint_vel` is also not obviously over-filtered:
  - dynamic main motion amplitude is nearly unchanged
  - delay remains within about `1 ~ 2` samples at `200 Hz`
- `phi_rate` is close to the upper edge of what should be tolerated:
  - static noise drops well
  - dynamic motion peaks are visibly rounded
  - if this feeds a sensitive feedback loop, consider slightly relaxing cutoff
- `length_rate` is the most aggressive filter:
  - this is justified by the raw noise level
  - if faster leg-length transients matter later, this channel is the first one worth re-checking

### Practical Conclusion

- Current LPF looks effective overall.
- No channel shows clear evidence of "filtered too much" in the present logs.
- No channel shows clear evidence of "filtering not enough" either.
- The only channels worth close follow-up are:
  - `phi_rate`: maybe slightly strong
  - `length_rate`: intentionally strong, acceptable for now

### Next Best Validation

- To finalize cutoffs with more confidence, the most useful next log is a deliberately faster but still controlled motion test.
- Focus on whether:
  - `phi_rate_lpf` peak timing starts to lag real motion
  - `length_rate_lpf` misses narrow transients
  - `joint_vel` remains smooth without making the actuator feel soft

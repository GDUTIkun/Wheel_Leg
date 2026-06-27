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

## 2026-06-28 Dynamic LPF Tuning Update

### Scope

- Data files:
  - [joints_dynamic_lpf_fast.summary.txt](/home/tan/wheel%20_leg_ws/logs/stm32_state_capture/joints_dynamic_lpf_fast.summary.txt)
  - [joints_dynamic_lpf_slow.summary.txt](/home/tan/wheel%20_leg_ws/logs/stm32_state_capture/joints_dynamic_lpf_slow.summary.txt)
- `slow` and `fast` use the same filter parameters.
- The difference is excitation speed only:
  - `slow`: manually moved motors slowly
  - `fast`: manually moved motors quickly

### What The New Data Showed

- Current `hip_vel` and `knee_vel` filtering is not obviously over-filtered:
  - typical lag stayed around one sample at `200 Hz`, about `5 ms`
  - motion peaks were still mostly preserved
- Current `fast` run suggests hip/knee filtering is still slightly light:
  - dynamic standard deviation changed very little
  - visible motion is preserved, but spike suppression during fast movement is limited
- Current `wheel_vel` filtering is stronger than hip/knee and already starts to compress peaks in `slow`:
  - wheel velocity should stay separately tuned
  - it should not be increased together with hip/knee
- `phi_rate` filtering looks acceptable as a light filter:
  - it reduces rate noise without obvious large delay
- `length_rate` is the heaviest filter in the current stack:
  - noise reduction is clear
  - but fast transients are being suppressed more aggressively than the other rate channels

### Updated Parameter Decision

- Keep the same filter structure.
- Adjust only default alpha values in `wheel_leg_stm32_bridge`.
- New defaults chosen as a compromise between the `slow` and `fast` runs:
  - `hip_velocity_low_pass_alpha`: `0.57 -> 0.60`
  - `knee_velocity_low_pass_alpha`: `0.62 -> 0.65`
  - `wheel_velocity_low_pass_alpha`: `0.73 -> 0.68`
  - `phi_rate_low_pass_alpha`: `0.57 -> 0.60`
  - `length_rate_low_pass_alpha`: `0.73 -> 0.70`
  - `body_velocity_low_pass_alpha`: keep `0.73`

### Rationale

- `hip_vel`:
  - slightly stronger than before to better suppress fast-run spikes
  - still intended to remain responsive
- `knee_vel`:
  - keep slightly stronger than hip because knee velocity remains noisier
- `wheel_vel`:
  - back off a little from the previous setting because slow-run peak compression was already visible
- `phi_rate`:
  - move to a slightly steadier setting while keeping it in the light-filter category
- `length_rate`:
  - reduce filter strength slightly because the previous setting was the closest to over-filtering in fast transitions
- `body_velocity`:
  - keep unchanged for now because it directly affects wheel-related closed-loop behavior and no stronger evidence suggests retuning it yet

### Next Check

- Re-run the same `slow` and `fast` manual-motion tests with the new defaults.
- Focus on:
  - whether hip/knee spikes are reduced in `fast`
  - whether wheel velocity peaks stay less compressed in `slow`
  - whether `length_rate` recovers more useful transient content without becoming visibly noisy

## 2026-06-28 Dynamic LPF Tuning Follow-Up

### Scope

- Data files:
  - [joints_dynamic_lpf_fast.summary.txt](/home/tan/wheel%20_leg_ws/logs/stm32_state_capture/joints_dynamic_lpf_fast.summary.txt)
  - [joints_dynamic_lpf_slow.summary.txt](/home/tan/wheel%20_leg_ws/logs/stm32_state_capture/joints_dynamic_lpf_slow.summary.txt)
- These runs were taken after the first round of default LPF updates:
  - `hip_velocity_low_pass_alpha = 0.60`
  - `knee_velocity_low_pass_alpha = 0.65`
  - `wheel_velocity_low_pass_alpha = 0.68`
  - `phi_rate_low_pass_alpha = 0.60`
  - `length_rate_low_pass_alpha = 0.70`

### What Improved

- `hip_vel` and `knee_vel` stayed responsive:
  - slow-run lag remained about `5 ms`
  - fast-run lag stayed in the light-filter range, about `10 ms`
  - no clear sign of over-filtering
- `wheel_vel` improved relative to the previous setting:
  - slow-run peak compression became noticeably milder
  - wheel velocity now looks closer to the intended compromise between noise suppression and responsiveness
- `phi_rate` remained acceptable as a light filter:
  - useful smoothing is present
  - no strong evidence of excessive delay

### What Still Looks Conservative

- `length_rate` is still the strongest filter in the current group:
  - dynamic standard deviation is still reduced heavily
  - fast-transition peaks are still compressed much more than the other rate channels
- This means:
  - for observation-only use, the current setting is still usable
  - for judging fast transient leg extension/retraction behavior, it is still somewhat too conservative

### Updated Decision

- Keep these unchanged:
  - `hip_velocity_low_pass_alpha = 0.60`
  - `knee_velocity_low_pass_alpha = 0.65`
  - `wheel_velocity_low_pass_alpha = 0.68`
  - `phi_rate_low_pass_alpha = 0.60`
  - `body_velocity_low_pass_alpha = 0.73`
- Adjust only:
  - `length_rate_low_pass_alpha: 0.70 -> 0.67`

### Rationale

- The latest retest suggests the first parameter update already fixed the more obvious wheel-velocity issue.
- The main remaining candidate for improvement is `length_rate`.
- Changing only one parameter in the next round makes the next comparison easier to interpret.
- `0.67` is intended as a small step, not a large jump:
  - recover a bit more transient content
  - avoid immediately exposing too much differentiated noise

### Next Check

- Re-run the same manual `slow` and `fast` tests with only `length_rate_low_pass_alpha = 0.67` changed.
- Focus on:
  - whether `length_rate_lpf` retains more transient structure in `fast`
  - whether `length_rate_lpf` remains visually cleaner than raw in `slow`
  - whether the leg-length-rate channel becomes more useful without becoming obviously spiky

## 2026-06-28 Dynamic LPF Tuning Follow-Up 2

### Scope

- Data files:
  - [joints_dynamic_lpf_fast.summary.txt](/home/tan/wheel%20_leg_ws/logs/stm32_state_capture/joints_dynamic_lpf_fast.summary.txt)
  - [joints_dynamic_lpf_slow.summary.txt](/home/tan/wheel%20_leg_ws/logs/stm32_state_capture/joints_dynamic_lpf_slow.summary.txt)
- This round changed only:
  - `length_rate_low_pass_alpha: 0.70 -> 0.67`

### What Changed

- `hip_vel`, `knee_vel`, `wheel_vel`, `phi_rate`, and `body_velocity` remained in the same practical range as the previous round.
- `length_rate` did move in the intended direction, but only slightly:
  - filtered standard deviation increased a little relative to raw
  - some transient content was recovered
  - however, the effect was still modest

### Practical Reading

- The `0.67` setting appears safe:
  - it did not create an obvious noise explosion
  - it did not introduce a new visible problem in the other channels
- But `0.67` still looks conservative for `length_rate`:
  - peak retention is still much lower than the other rate-like channels
  - transition detail is still more suppressed than desired

### Updated Decision

- Keep unchanged:
  - `hip_velocity_low_pass_alpha = 0.60`
  - `knee_velocity_low_pass_alpha = 0.65`
  - `wheel_velocity_low_pass_alpha = 0.68`
  - `phi_rate_low_pass_alpha = 0.60`
  - `body_velocity_low_pass_alpha = 0.73`
- Adjust only:
  - `length_rate_low_pass_alpha: 0.67 -> 0.62`

### Rationale

- The previous step was likely too small to create a clearly readable improvement.
- The main unresolved issue is still `length_rate` transient suppression.
- A larger single-step change is more useful now than another tiny nudge, because:
  - it should produce a more visible difference in the next `slow/fast` comparison
  - it still keeps the test isolated to one parameter

### Next Check

- Re-run the same `slow` and `fast` manual-motion tests with only `length_rate_low_pass_alpha = 0.62`.
- Focus on:
  - whether `length_rate_lpf` becomes meaningfully closer to the raw transition shape
  - whether the channel stays interpretable in `slow`
  - whether the noise increase is acceptable for the intended downstream use

## 2026-06-28 Dynamic LPF Tuning Follow-Up 3

### Scope

- Data files:
  - [joints_dynamic_lpf_fast.summary.txt](/home/tan/wheel%20_leg_ws/logs/stm32_state_capture/joints_dynamic_lpf_fast.summary.txt)
  - [joints_dynamic_lpf_slow.summary.txt](/home/tan/wheel%20_leg_ws/logs/stm32_state_capture/joints_dynamic_lpf_slow.summary.txt)
- This round changed only:
  - `length_rate_low_pass_alpha: 0.67 -> 0.62`

### Result

- `hip_vel`, `knee_vel`, `wheel_vel`, `phi_rate`, and `body_velocity` remained in a stable and acceptable range.
- `length_rate` became more open than the `0.67` setting:
  - more transient structure is visible
  - dynamic attenuation is reduced compared with the previous round
- The tradeoff is expected:
  - low-motion noise increased somewhat
  - but it did not become obviously unusable in this manual-motion test

### Practical Conclusion

- `length_rate_low_pass_alpha = 0.62` is a reasonable stopping point for now.
- It is a better compromise than `0.67` if the goal is to observe leg-length-rate transitions more clearly.
- It does not currently justify another immediate reduction, because:
  - additional transient recovery will likely come with disproportionately more noise
  - the next more useful validation step is no longer manual motion, but real closed-loop or close-to-real actions

### Current Recommended Default Set

- `hip_velocity_low_pass_alpha = 0.60`
- `knee_velocity_low_pass_alpha = 0.65`
- `wheel_velocity_low_pass_alpha = 0.68`
- `phi_rate_low_pass_alpha = 0.60`
- `length_rate_low_pass_alpha = 0.62`
- `body_velocity_low_pass_alpha = 0.73`

## How To Validate On Real Actions

### Goal

- The next objective is not only to compare raw-vs-filtered smoothness.
- The real question is whether these signals remain useful when the robot is doing meaningful tasks:
  - standing balance
  - small forward/backward wheel motion
  - squat-like leg length change
  - small disturbance recovery

### Recommended Test Order

1. Closed-loop stand with no intentional command input for a short hold.
2. Closed-loop stand with small forward/backward velocity commands.
3. Closed-loop stand with small leg-length reference changes if available.
4. A slightly more dynamic but still safe maneuver, such as gentle start-stop or mild push recovery.

### What To Record

- Keep recording the same filtered and raw channels when possible.
- The most useful channels to compare together are:
  - `left/right_hip_vel_raw` vs `left/right_hip_vel`
  - `left/right_knee_vel_raw` vs `left/right_knee_vel`
  - `left/right_wheel_vel_raw` vs `left/right_wheel_vel`
  - `left/right_phi_rate_raw` vs `left/right_phi_rate_lpf`
  - `left/right_length_rate_raw` vs `left/right_length_rate_lpf`
  - `body_velocity_raw` vs `body_velocity`
- During closed-loop tests, also pay attention to related behavior signals:
  - `pitch`
  - `pitch_rate`
  - wheel torque / wheel effort
  - hip torque / hip effort
  - commanded velocity or leg-length reference if available

### What To Look For

- For `hip_vel`, `knee_vel`, and `wheel_vel`:
  - filtered signals should stay close to raw during real motion
  - they should remove small chatter without visibly delaying actual motion reversals
- For `phi_rate`:
  - filtered output should stay smooth enough for control use
  - it should still track body-leg swing timing clearly
- For `length_rate`:
  - filtered output should show clear extend/retract phases during real actions
  - it should not collapse small real transitions into near-flat output
  - it should also not inject obvious high-frequency spikes into downstream control
- For `body_velocity`:
  - filtered output should remain smooth
  - it should not visibly lag starts, stops, or reversals too much

### Signs The Filters Are Still Too Heavy

- Real motion reversals show up in raw first, but filtered signals react late enough to be visually obvious.
- `length_rate_lpf` still looks too flat during clear leg extension or retraction.
- Wheel or joint filtered velocity misses the shape of small but real command-driven motion.

### Signs The Filters Are Too Light

- Wheel torque or hip torque chatters even during otherwise steady motion.
- `length_rate_lpf` becomes dominated by sharp spikes instead of extend/retract structure.
- `phi_rate_lpf` or wheel/joint filtered velocity starts to look nearly as ragged as raw while offering little practical cleanup.

### Decision Rule For The Next Step

- If the robot behaves well in closed-loop and the filtered signals remain interpretable, keep this parameter set and stop tuning here.
- If the robot is stable but `length_rate_lpf` still hides too much real transition structure, then only `length_rate` should be revisited.
- If control effort becomes visibly noisy after moving into real closed-loop operation, revisit the specific channel most associated with that behavior rather than globally increasing all filters.

## 2026-06-28 Final Manual-Validation Conclusion

### Scope

- Additional focused checks were done with:
  - [length_raw_vs_lpf.csv](/home/t/wheel_leg_ws/logs/stm32_state_capture/length_raw_vs_lpf.csv)
  - [length_raw_vs_lpf.png](/home/t/wheel_leg_ws/logs/stm32_state_capture/length_raw_vs_lpf.png)
  - [phi_raw_vs_lpf.csv](/home/t/wheel_leg_ws/logs/stm32_state_capture/phi_raw_vs_lpf.csv)
  - [phi_raw_vs_lpf.png](/home/t/wheel_leg_ws/logs/stm32_state_capture/phi_raw_vs_lpf.png)
  - [length_integral_replay.png](/home/t/wheel_leg_ws/logs/stm32_state_capture/length_integral_replay.png)

### Important Clarification

- `phi` and `leg_length` currently do **not** have separate filtered outputs in the STM32 bridge.
- Therefore:
  - `/robot_state_raw.left_phi == /robot_state.left_phi`
  - `/robot_state_raw.left_leg_length == /robot_state.left_leg_length`
  - the same applies to the right leg
- So `phi_raw` vs `phi` and `length_raw` vs `length` are useful as publication-consistency checks, but not as evidence of actual LPF on those base geometry signals.

### What The Integral Replay Showed

- The most useful proxy for judging `length`-related filtering was:
  - integrate `length_rate_raw`
  - integrate `length_rate_lpf`
  - compare both replayed trajectories against measured `length`
- Result:
  - replay from `length_rate_lpf` stayed close to measured `length`
  - replay from `length_rate_raw` drifted much more
- This indicates:
  - the current `length_rate` LPF is doing useful cleanup
  - it is not obviously over-smoothing the underlying leg-length motion in this manual test

### Final Recommended Filter Set

- `hip_velocity_low_pass_alpha = 0.60`
- `knee_velocity_low_pass_alpha = 0.65`
- `wheel_velocity_low_pass_alpha = 0.68`
- `phi_rate_low_pass_alpha = 0.60`
- `length_rate_low_pass_alpha = 0.62`
- `body_velocity_low_pass_alpha = 0.73`

### Control Use Decision

- This parameter set is acceptable for the next control stage.
- In particular:
  - `hip/knee/wheel` velocity filtering is light enough to preserve motion response
  - `phi_rate` remains usable without obvious excessive lag
  - `length_rate` now appears clean enough for control use while still tracking meaningful motion structure
- Recommendation:
  - keep this set as the current default in code
  - move the next validation effort from manual-motion tuning to controller-side behavior checks

# GRF Analysis - All Forces at Minimum

## Observed Behavior
```
GRF at k = 0:
0.000 0.000 0.000 0.000  (fx for all 4 feet)
0.000 0.000 0.000 0.000  (fy for all 4 feet)
23.000 23.000 23.000 23.000  (fz for all 4 feet - exactly min_vertical_grf)
```

## Analysis

### Issue Identified
All vertical forces are exactly at the **minimum constraint** (23.0N), and all horizontal forces are zero. This suggests:

1. **MPC is minimizing control effort** - With R = 1e-5 (very small), control cost is low, so MPC minimizes forces to the constraint
2. **Missing upper bound** - Previously no `fz_max` constraint, so forces could theoretically go to infinity, but MPC chose minimum
3. **Possible zero state error** - If robot is at reference, MPC might not need forces

### Fixes Applied

1. ✅ **Added `fz_max` parameter** (500N) - matches Python
2. ✅ **Added upper bound constraint** - `fz <= fz_max` for stance legs
3. ✅ **Fixed R matrix** - Changed from Identity to 1e-5 * Identity (matches Python)

### Expected Behavior After Fix

With the upper bound constraint:
- Forces should vary between `min_vertical_grf` (23N) and `fz_max` (500N)
- Forces should respond to state errors
- If robot is balanced, forces should be around `mass * g / 4 = 15.7 * 9.81 / 4 ≈ 38.5N` per leg

### If Forces Still at Minimum

If forces remain at minimum after the fix, check:

1. **State error** - Look at `x0 - X_ref` output
   - If all zeros → robot is at reference (expected behavior)
   - If non-zero → MPC should generate forces

2. **Reference trajectory** - Check if reference matches current state
   - Height reference vs actual height
   - Velocity reference vs actual velocity

3. **Contact states** - Verify all legs are in contact (contact_state = 1)

4. **Q matrix weights** - Ensure state weights are non-zero for states you want to control

### Diagnostic Commands

Check the state error output:
```
x0 - X_ref: [roll_err, pitch_err, yaw_err, x_err, y_err, z_err, wx_err, wy_err, wz_err, vx_err, vy_err, vz_err, g_err]
```

If errors are small/zero, forces at minimum might be correct behavior.

### Next Steps

1. **Test with the fix** - Rebuild and run
2. **Check state error** - Verify if errors are non-zero
3. **Adjust `min_vertical_grf`** - If too high, lower it (e.g., 10N instead of 23N)
4. **Check reference trajectory** - Ensure it's reasonable



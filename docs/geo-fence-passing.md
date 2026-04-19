# Geo-Fenced Passing Mode

## Problem

Pass shots failed ~50% of the time because the 20-degree velocity-compensation
divergence gate blocked feeding. Pass time-of-flight (1.2-1.8s) is 3-4x longer
than hub shots (~0.4s), which causes the iterative velcomp solver to shift the
aim point 20-40 degrees — well past the 20-degree gate designed for hub precision.

## Solution

When `Shots/GeoFencePass/Enabled = 1.0` (default), PASS and LONG_PASS shots use:

1. **Single-pass open-loop lead correction** instead of 3-pass iterative velcomp.
   Compute shot at static target, get TOF, shift aim by `robot_velocity * TOF`.
   No iteration, no convergence issues.

2. **Predicted-landing-point geo-fence** instead of divergence gate.
   Uses vacuum ballistics to predict where the ball lands on the field, then
   checks that point is inside the alliance zone (shrunk by a safety margin).

HUB shots are completely unaffected.

## Tunables (NetworkTables)

| Key | Default | Description |
|-----|---------|-------------|
| `Shots/GeoFencePass/Enabled` | `1.0` | `1.0` = geo-fence mode, `0.0` = legacy divergence gate |
| `Shots/GeoFencePass/MarginM` | `0.5` | Safety margin (meters) shrunk inward from zone edges |

## Safe Zone Definition

The geo-fence is a rectangle defined in `FieldConstants.PassSafeZone`:

- **Blue alliance**: X from `0 + margin` to `allianceZone - margin`, Y from `0 + margin` to `fieldWidth - margin`
- **Red alliance**: X bounds mirrored across field center

The ball must land inside this rectangle to be approved.

## Logged Outputs

| Signal | Type | Description |
|--------|------|-------------|
| `SmartLaunch/Pass/PredictedLandingX` | double | Predicted ball landing X (field coords) |
| `SmartLaunch/Pass/PredictedLandingY` | double | Predicted ball landing Y (field coords) |
| `SmartLaunch/Pass/GeoFencePass` | boolean | True if landing is inside safe zone |
| `SmartLaunch/Pass/FenceReason` | string | "ok" or reason for rejection (e.g., "x=1.2 > maxX=0.8 (past zone)") |

The `SmartLaunch/Blocking` signal shows `geofence(reason)` instead of `velcomp(deg)` when geo-fence mode is active and a pass is blocked.

## Rollback

Set `Shots/GeoFencePass/Enabled` to `0.0` via NetworkTables. Takes effect
immediately (next loop cycle). No redeploy needed.

# Shoot-on-the-Move: Lesson Plan & Implementation Guide

## Who This Is For

Students who already understand our robot code structure and want to deeply
understand how our shooting system works, why it makes the choices it does,
and how to build an alternative approach that could improve accuracy.

**Prerequisites**:
- **Java**: Comfortable with interfaces, inheritance, and reading unfamiliar
  classes. You don't need to be an expert — if you've taken AP CS A or
  equivalent, you're ready.
- **Math**: Trigonometry (sin, cos, tan, atan2) and basic kinematics
  (d = vt, d = v₀t + ½at²). If you've taken or are in AP Physics 1 or
  AP Calculus, you have everything you need. If not, the relevant equations
  are explained as they come up — you can learn them here.
- **WPILib**: Understanding of subsystems, commands, `Pose2d`, and
  `ChassisSpeeds`. You should know what field-relative coordinates mean.
- **Our codebase**: You've read through at least one subsystem end-to-end
  and can navigate the project in your IDE.

**Time estimate**: 5 sessions of ~2 hours each. Sessions 1-2 are reading
and understanding. Sessions 3-4 are building and testing. Session 5 is an
advanced improvement that can be done independently.

### How to Use This Guide

**If you have limited time** (e.g., build season crunch): Do Sessions 1, 2,
and 5. You'll deeply understand how the system works and can implement the
velocity prediction improvement without building the full strategy system.

**If you have a full week**: Do all 5 sessions in order. Each builds on the
last.

**General tips**:
- Do the exercises. Seriously. Reading code is not the same as understanding
  it. The hand calculations in Session 1 will save you hours of debugging
  later.
- Work with a partner if possible. The design discussions (3.3a, 3.3d, 5.4a)
  are much better as conversations.
- Keep AdvantageScope open while reading. Seeing the real logged values
  alongside the code makes everything click faster.

---

## Session 1: Understanding What We Have (The Parametric Approach)

### 1.1 The Big Picture — What Problem Are We Solving?

When our robot is stationary, shooting is straightforward: measure the distance
to the hub, calculate the right RPM and hood angle, fire. But in a real match
the robot is moving. By the time the ball reaches the hub, the robot (and
therefore the launch point) has shifted. If we aim at the hub's current
position, we'll miss.

**The core question**: Where should we aim RIGHT NOW so the ball — launched
from where we WILL BE — lands in the hub?

Open this file and read it top to bottom before continuing:

```
src/main/java/frc/robot/subsystems/shooting/ShotCalculator.java
```

### 1.2 Tracing the Data Flow

Here is how a shot gets calculated every robot cycle (20ms):

```
ShootingCoordinator.periodic()          [ShootingCoordinator.java:138]
  │
  ├─ updateShotCalculation()            [ShootingCoordinator.java:164]
  │    │
  │    ├─ TurretAimingHelper.getAimTarget()   → decides SHOOT / PASS / HOLDFIRE
  │    │
  │    ├─ if SHOOT → calculateShotToHub()     [ShootingCoordinator.java:230]
  │    │      │
  │    │      └─ ShotCalculator.calculateHubShot()     [ShotCalculator.java:309]
  │    │           │
  │    │           ├─ TrajectoryOptimizer.calculateOptimalShot()  → RPM + hood angle
  │    │           ├─ calculateTimeOfFlight()                     → TOF estimate
  │    │           ├─ predictTargetPos() × 3 iterations           → velocity compensation
  │    │           └─ calculateOutsideTurretAngle()               → turret pointing
  │    │
  │    └─ if PASS → calculatePassToTarget()   [ShootingCoordinator.java:298]
  │
  ├─ runAutoShoot()                      → fires if all readiness checks pass
  │
  └─ visualizer.update()                 → 3D trajectory display
```

**Exercise 1.2a**: Open `ShootingCoordinator.java` and trace this flow yourself.
Write down the line numbers for each step. Confirm they match the diagram above.

### 1.3 The Physics Model — Two-Point Trajectory Solving

Our system is **parametric**: it solves physics equations in real-time rather
than looking things up in a table. Here's the math.

#### The Projectile Equation

A ball launched at angle `θ` with velocity `v` follows a parabolic path:

```
y(x) = x·tan(θ) - g·x² / (2·v²·cos²(θ))
```

We simplify by defining `K = g / (2·v²·cos²(θ))`:

```
y(x) = x·tan(θ) - K·x²
```

This is the equation of a parabola. Given any `θ` and `v`, we can trace
the entire ball trajectory.

#### How TrajectoryOptimizer Uses Two Points

Open `TrajectoryOptimizer.java` and find `calculateTrajectoryThroughTwoPoints`
at line 235.

Our hub has specific geometry:
- **Hub lip height**: 1.83m (the rim the ball must clear)
- **Hub center height**: 1.43m (where we want the ball to land)
- **Hub entry radius**: 0.530m (distance from lip to center)

The optimizer defines two constraint points:

```
Point A (Hub Edge):  x₁ = D - R,   y₁ = hubCenter + R·tan(descentAngle) - turretHeight
Point B (Hub Center): x₂ = D,       y₂ = hubCenter - turretHeight

Where:
  D = horizontal distance from turret to hub center
  R = hub entry radius (0.530m)
  descentAngle = 60° (tunable — the angle the ball enters the hub)
```

With two points on a parabola, there is exactly ONE `θ` and `v` that
fits. The math (lines 249-297):

```
tan(θ) = (y₁·x₂² - y₂·x₁²) / (x₁·x₂·(x₂ - x₁))

K = (x₁·tan(θ) - y₁) / x₁²

v² = g / (2·K·cos²(θ))

RPM = (v / (2π·r·efficiency)) × 60
```

#### Worked Example: D = 5.0m

Let's walk through the math once at D = 5.0m so you see how the pieces fit.
You'll do it yourself at a different distance in the exercise below.

```
Given:
  D = 5.0m (distance from turret to hub center)
  R = 0.530m (hub entry radius)
  hubCenterHeight = 1.43m
  turretHeight ≈ 0.60m (from robot geometry)
  descentAngle = 60°

Step 1: Find Point A (hub edge) and Point B (hub center)

  x₁ = D - R = 5.0 - 0.530 = 4.47m
  y₁ = (hubCenterHeight + R·tan(60°)) - turretHeight
     = (1.43 + 0.530 × 1.732) - 0.60
     = (1.43 + 0.918) - 0.60
     = 1.748m

  x₂ = D = 5.0m
  y₂ = hubCenterHeight - turretHeight = 1.43 - 0.60 = 0.83m

Step 2: Solve for tan(θ)

  tan(θ) = (y₁·x₂² - y₂·x₁²) / (x₁·x₂·(x₂ - x₁))
         = (1.748 × 25.0 - 0.83 × 19.98) / (4.47 × 5.0 × 0.53)
         = (43.70 - 16.58) / (11.85)
         = 27.12 / 11.85
         = 2.288

  θ = atan(2.288) = 66.4°

Step 3: Solve for K, then v

  K = (x₁·tan(θ) - y₁) / x₁²
    = (4.47 × 2.288 - 1.748) / 19.98
    = (10.23 - 1.748) / 19.98
    = 0.4244

  v² = g / (2·K·cos²(θ))
     = 9.81 / (2 × 0.4244 × cos²(66.4°))
     = 9.81 / (2 × 0.4244 × 0.1597)
     = 9.81 / 0.1356
     = 72.3

  v = 8.51 m/s

Step 4: Convert to RPM

  RPM = (v / (2π × WHEEL_RADIUS × efficiency)) × 60
      = (8.51 / (2π × 0.0508 × 0.4)) × 60
      = (8.51 / 0.1277) × 60
      = 66.64 × 60
      = 3998 RPM
```

Don't worry if your numbers don't match exactly — the turret height and
descent angle are tunables that may have changed. The point is understanding
the PROCESS: two points define a parabola, and from that parabola you
extract the launch angle and velocity.

**Exercise 1.3a**: Now do it yourself at D = 4.0m. Using the same constants:
1. What are Point A and Point B? (x₁, y₁, x₂, y₂)
2. What is tan(θ)?
3. What launch angle θ does this give?
4. What exit velocity v is needed?
5. What RPM does that require?

Check your answers against what the robot logs in AdvantageScope at that
distance. They should be close but may not match exactly (the code uses
slightly different constants and includes additional corrections).

### 1.4 Velocity Compensation — The Iterative Prediction Loop

This is the core of shoot-on-the-move. Open `ShotCalculator.java` line 309,
the `calculateHubShot` method.

When the robot is moving faster than 0.1 m/s, the code runs a 3-iteration
refinement loop (lines 343-356):

```java
for (int i = 0; i < 3; i++) {
    aimTarget = predictTargetPos(hubTarget, fieldSpeeds, tof);
    // ... recalculate shot to new aimTarget ...
    // ... update tof based on new distance ...
}
```

**What `predictTargetPos` does** (line 185):

```java
predictedX = target.getX() - fieldSpeeds.vxMetersPerSecond * timeOfFlight;
predictedY = target.getY() - fieldSpeeds.vyMetersPerSecond * timeOfFlight;
```

This is saying: "If the robot moves `v * t` meters during the ball's flight,
then we need to aim `v * t` meters *behind* where the hub actually is."

Wait — why subtract? Because from the ball's perspective, the ball inherits
the robot's velocity. If the robot is moving toward the hub at 2 m/s, the ball
already has 2 m/s of approach velocity baked in. So we compensate by aiming
as if the hub were closer (offset by the robot's velocity contribution during
flight time).

#### Why 3 Iterations?

The problem is circular:
1. To know where to aim, we need the time of flight (TOF)
2. To know the TOF, we need the distance to the aim point
3. But the aim point depends on the TOF

So we iterate:
- **Iteration 0**: Calculate TOF to the real hub position → initial guess
- **Iteration 1**: Offset hub by `v × TOF₀`, recalculate TOF₁
- **Iteration 2**: Offset hub by `v × TOF₁`, recalculate TOF₂
- **Iteration 3**: Offset hub by `v × TOF₂`, recalculate TOF₃

Each iteration refines the answer. By iteration 3, the solution has converged
for typical FRC speeds (< 5 m/s).

**Exercise 1.4a**: If the robot is at (3, 4) moving at vx=2.0, vy=0.0 m/s
and the hub is at (5, 4, 1.43):

The TOF formula from our code (line 157) is:
```
TOF = horizontalDistance / (v_exit × cos(θ))
```
This is just "distance = speed × time" rearranged, using the horizontal
component of the exit velocity.

Using v_exit = 8 m/s and θ = 55°:
1. What is the initial horizontal distance? What is the initial TOF?
   (Hint: distance from (3,4) to (5,4) = 2.0m. TOF = 2.0 / (8 × cos(55°)) = ?)
2. Using that TOF, where does `predictTargetPos` put the aim target?
   What is the new distance to THAT point?
3. Repeat: use the new distance to get a new TOF, then a new aim target.
4. How much did the aim point change between iterations 2 and 3?
   (If it changed less than 1cm, the iteration has converged.)

### 1.5 What This Approach Gets Right and Where It Falls Short

**Strengths of our parametric approach:**
- No empirical data collection needed — works immediately from physics
- Smooth across all distances — no interpolation artifacts
- All parameters are tunable in real-time via LoggedTunableNumber
- Clean, readable code

**Weaknesses:**
1. **No air drag**: Our TOF calculation (line 157) is `distance / (v·cos(θ))`.
   This assumes the ball travels in a vacuum. Real balls experience drag, so
   the actual TOF is LONGER than we calculate. This means our velocity lead
   is systematically too small at long range.

2. **Instantaneous velocity only**: We use the robot's current velocity to
   predict where to aim. But if the driver is decelerating, the robot will
   be slower during the ball's flight than it is right now, and we'll
   over-lead the shot.

3. **Launch efficiency is a fudge factor**: `launchEfficiency = 0.4` absorbs
   many real-world effects (ball compression, slip, spin) into one number.
   If the real efficiency varies with RPM (it probably does), our velocity
   calculations are off by different amounts at different speeds.

**Exercise 1.5a**: Discussion question — which of these three weaknesses do
you think matters most for our robot at typical shooting distances (3-6m)?
Why?

### Session 1 Key Takeaways

If you can explain these to a teammate, you're ready for Session 2:

1. The shooting system uses a **two-point trajectory solver** to find the
   exact launch angle and velocity that sends the ball through the hub lip
   and into the center.
2. Velocity compensation works by **iteratively offsetting the aim point**
   to account for robot motion during ball flight.
3. The iteration converges because each refinement makes a smaller change
   — typically < 1cm after 3 iterations.
4. The main weaknesses are: no air drag in TOF, instantaneous velocity
   assumption, and a single fudge-factor efficiency constant.

---

## Session 2: The Alternative — LUT Recursion (Nonparametric Approach)

### 2.1 What Is a Lookup Table (LUT)?

Instead of solving equations, we measure the real robot's behavior and store
it in a table. For each distance, we record:
- The RPM that actually scores
- The hood angle that actually scores
- The actual time-of-flight (measured with a stopwatch or high-speed camera)

Then at runtime, we interpolate between table entries to get parameters for
any distance.

WPILib provides `InterpolatingDoubleTreeMap` for exactly this purpose.

### 2.2 Why Teams Use LUTs

From the Chief Delphi thread (link in references), Oblarg (lead programmer
of 6328 Mechanical Advantage) recommends LUT recursion as the first approach
teams should try. His reasoning:

> "Any team that has not yet decided on an algorithm but wants to try this
> problem, start with this approach first. It is simple."

The key insight: **a LUT-measured TOF inherently includes air drag, spin
effects, and every other real-world factor** because you measured the actual
ball, not a theoretical one. Our parametric TOF calculation
`distance / (v·cos(θ))` ignores all of that.

### 2.3 The LUT Recursion Algorithm

This is the algorithm for shoot-on-the-move using a LUT. It is structurally
identical to our current iterative loop, but replaces physics calculations
with table lookups.

```
Algorithm: LUT_RECURSION_SOTM(robotPose, robotVelocity, hubTarget)

  Input:
    tofTable[distance] → time of flight (seconds)
    rpmTable[distance] → launcher RPM
    hoodTable[distance] → hood angle (degrees)

  1. staticDistance = distance(robotPose, hubTarget)
  2. tof = tofTable.get(staticDistance)         // initial TOF guess

  3. for i = 1 to N:                           // N = 3 to 5 iterations
       aimTarget = hubTarget - robotVelocity * tof
       aimDistance = distance(robotPose, aimTarget)
       tof = tofTable.get(aimDistance)           // refined TOF

  4. finalDistance = distance(robotPose, aimTarget)
  5. rpm = rpmTable.get(finalDistance)
  6. hoodAngle = hoodTable.get(finalDistance)
  7. turretAngle = angleTo(robotPose, aimTarget)

  Return: (rpm, hoodAngle, turretAngle, aimTarget)
```

Compare this to our current approach in `ShotCalculator.calculateHubShot()`.
The structure is almost identical! The difference is:
- **Ours**: Calls `TrajectoryOptimizer.calculateOptimalShot()` + `calculateTimeOfFlight()`
- **LUT**: Calls `tofTable.get()` + `rpmTable.get()` + `hoodTable.get()`

### 2.4 Designing the Data Structures

Here is what the LUT tables look like in WPILib Java:

```java
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

// Key: distance in meters, Value: parameter
InterpolatingDoubleTreeMap tofTable = new InterpolatingDoubleTreeMap();
InterpolatingDoubleTreeMap rpmTable = new InterpolatingDoubleTreeMap();
InterpolatingDoubleTreeMap hoodAngleTable = new InterpolatingDoubleTreeMap();

// Populate from empirical measurements
tofTable.put(2.0, 0.45);   // at 2m, ball takes 0.45s to reach hub
tofTable.put(3.0, 0.62);   // at 3m, ball takes 0.62s
tofTable.put(4.0, 0.81);
tofTable.put(5.0, 1.05);
tofTable.put(6.0, 1.32);

rpmTable.put(2.0, 2100.0);
rpmTable.put(3.0, 2450.0);
// ... etc

hoodAngleTable.put(2.0, 35.0);
hoodAngleTable.put(3.0, 32.0);
// ... etc
```

`InterpolatingDoubleTreeMap` does linear interpolation between entries, so
if you query `tofTable.get(2.5)` it returns `(0.45 + 0.62) / 2 = 0.535`.

**Exercise 2.4a**: Why do we need at least 5-6 distance entries? What happens
if we only have 2 entries? What happens if we have 20? Discuss tradeoffs.

### 2.5 How to Collect the Data

This is the hardest part of the LUT approach and the reason many teams skip
it. You need to physically put the robot at known distances and measure:

**Step 1: Set up distance markers**
- Use tape on the floor at 2m, 3m, 4m, 5m, 6m from hub center
- Verify with a laser distance meter

**Step 2: For each distance, find the RPM + hood angle that scores**
- Start with our parametric model's output as an initial guess
- Fine-tune by hand until 5/5 shots score consistently
- Record: (distance, finalRPM, finalHoodAngle)

**Step 3: Measure time of flight**
Option A (stopwatch): Have someone watch high-speed video (240fps phone camera)
and count frames from launch to hub entry. TOF = frames / 240.

Option B (code-based): Log the timestamp when the ball breaks a beam sensor
at launch and use AdvantageScope to measure when it enters the hub
(visible on field camera).

Option C (calculated from measured velocity): If you trust your exit velocity
measurement, you can calculate TOF from the measured parameters. But this
partially defeats the purpose — you'd still be assuming no drag.

**Step 4: Record everything in a spreadsheet, then put it in code**

**Exercise 2.5a**: Plan a data collection session. Write out:
1. How many distances will you measure?
2. How many shots per distance for statistical confidence?
3. What equipment do you need?
4. How long will it take?
5. What do you do if a measurement seems wrong?

### Session 2 Key Takeaways

1. A LUT replaces physics equations with **measured data** — you record what
   actually works and interpolate between measurements.
2. The LUT recursion algorithm has the **exact same structure** as our
   parametric iteration loop. The only difference is where TOF/RPM/hood
   values come from.
3. LUT's main advantage: measured TOF **inherently captures air drag** and
   other real-world effects that physics models ignore.
4. LUT's main cost: you need **physical robot time** to collect data, and
   you have to redo it when hardware changes.

---

## Session 3: The Hybrid Approach — Best of Both Worlds

### 3.1 Why Hybrid?

The pure LUT approach has its own weaknesses:
- Need to re-measure if ANY hardware changes (new wheels, different ball type)
- Gaps between measured distances rely on interpolation accuracy
- Can't easily adapt to conditions (battery voltage, worn wheels)

Our parametric approach is already good for RPM and hood angle. Its main
weakness is the **time-of-flight calculation** ignoring drag.

The hybrid: **Keep our parametric model for RPM + hood angle. Add a TOF LUT
for the velocity compensation loop only.**

This gives us:
- Parametric RPM/hood: smooth, no gaps, works at any distance
- Empirical TOF: captures drag, spin, and real-world flight behavior
- Velocity compensation: more accurate lead because TOF is closer to reality

### 3.2 Architecture: The Strategy Pattern

We want the system to be easily toggled between approaches. The cleanest
way is something called the **Strategy pattern** — a classic software design
technique where you define an interface for an algorithm, write multiple
implementations of it, and swap between them at runtime.

If you've taken AP CS, think of it like this: you already know that a
`List<String>` can be backed by `ArrayList` or `LinkedList`. The code that
uses the list doesn't care which one — it just calls `list.add()`. The
Strategy pattern is the same idea applied to our shot calculation: the
`ShootingCoordinator` doesn't care HOW the shot is calculated, it just
calls `strategy.calculateHubShot()`.

Here is the design:

```
                    ┌─────────────────────┐
                    │ ShotStrategy        │ ← interface
                    │                     │
                    │ + calculateShot()   │
                    │ + getName()         │
                    └──────────┬──────────┘
                               │
              ┌────────────────┼────────────────┐
              │                │                │
   ┌──────────▼──────┐ ┌──────▼──────┐ ┌───────▼─────────┐
   │ Parametric       │ │ LUT         │ │ Hybrid          │
   │ Strategy         │ │ Strategy    │ │ Strategy        │
   │                  │ │             │ │                  │
   │ (current code)   │ │ (pure LUT) │ │ (parametric RPM │
   │                  │ │             │ │  + LUT TOF)     │
   └──────────────────┘ └─────────────┘ └─────────────────┘
```

The `ShootingCoordinator` holds a reference to the active `ShotStrategy`
and delegates to it. A dashboard button or tunable toggles between them.

### 3.3 Step-by-Step Implementation Plan

This is the most code-intensive session. It's normal if it takes longer than
2 hours — the important thing is understanding each piece before moving on.

Here is exactly what you will build, file by file. **Do not write any code
until you have read and understood all steps.**

#### Step 1: Create the `ShotStrategy` interface

**File**: `src/main/java/frc/robot/subsystems/shooting/ShotStrategy.java`

This interface defines the contract for any shot calculation approach:

```java
public interface ShotStrategy {

    /** Human-readable name for dashboard display. */
    String getName();

    /**
     * Calculate a hub shot with velocity compensation.
     *
     * @param robotPose       Current robot pose on the field
     * @param fieldSpeeds     Field-relative robot velocity
     * @param hubTarget       3D position of the hub center
     * @param turretPos       3D position of the turret on the field
     * @param config          Turret geometry config
     * @param currentTurretAngleDeg  Current turret angle for wrap optimization
     * @param effectiveMinDeg Turret min angle
     * @param effectiveMaxDeg Turret max angle
     * @param hoodMinAngleDeg Hood min angle
     * @param hoodMaxAngleDeg Hood max angle
     * @return ShotResult with all parameters needed to fire
     */
    ShotCalculator.ShotResult calculateHubShot(
        Pose2d robotPose,
        ChassisSpeeds fieldSpeeds,
        Translation3d hubTarget,
        ShotCalculator.TurretConfig config,
        double currentTurretAngleDeg,
        double effectiveMinDeg,
        double effectiveMaxDeg,
        double hoodMinAngleDeg,
        double hoodMaxAngleDeg);
}
```

Think about why the interface signature matches `ShotCalculator.calculateHubShot`
almost exactly. This makes it a drop-in replacement.

#### Step 2: Wrap the existing code as `ParametricShotStrategy`

**File**: `src/main/java/frc/robot/subsystems/shooting/ParametricShotStrategy.java`

This class simply delegates to the existing `ShotCalculator.calculateHubShot()`.
All of our current code stays untouched.

```java
public class ParametricShotStrategy implements ShotStrategy {

    @Override
    public String getName() {
        return "Parametric";
    }

    @Override
    public ShotCalculator.ShotResult calculateHubShot(
            Pose2d robotPose,
            ChassisSpeeds fieldSpeeds,
            Translation3d hubTarget,
            ShotCalculator.TurretConfig config,
            double currentTurretAngleDeg,
            double effectiveMinDeg,
            double effectiveMaxDeg,
            double hoodMinAngleDeg,
            double hoodMaxAngleDeg) {

        // Delegate to existing code — zero behavior change
        return ShotCalculator.calculateHubShot(
            robotPose, fieldSpeeds, hubTarget, config,
            currentTurretAngleDeg, effectiveMinDeg, effectiveMaxDeg,
            hoodMinAngleDeg, hoodMaxAngleDeg);
    }
}
```

**Exercise 3.3a**: Why is it important that the parametric strategy is a
thin wrapper with no new logic? Think about what happens when something
goes wrong with your new strategy mid-match and you need to fall back.

#### Step 3: Build the `LUTShotStrategy`

**File**: `src/main/java/frc/robot/subsystems/shooting/LUTShotStrategy.java`

This is where the real learning happens. You need to:

1. **Define three `InterpolatingDoubleTreeMap`s** — for TOF, RPM, and hood angle
2. **Populate them with placeholder data** (you'll replace with real measurements later)
3. **Implement the LUT recursion algorithm from Section 2.3**
4. **Calculate the turret angle** (reuse `ShotCalculator.calculateOutsideTurretAngle`)

Here's the skeleton — you fill in the algorithm:

```java
public class LUTShotStrategy implements ShotStrategy {

    private final InterpolatingDoubleTreeMap tofTable = new InterpolatingDoubleTreeMap();
    private final InterpolatingDoubleTreeMap rpmTable = new InterpolatingDoubleTreeMap();
    private final InterpolatingDoubleTreeMap hoodAngleTable = new InterpolatingDoubleTreeMap();

    public LUTShotStrategy() {
        // TODO: Populate with measured data
        // tofTable.put(distanceMeters, timeOfFlightSeconds);
        // rpmTable.put(distanceMeters, rpm);
        // hoodAngleTable.put(distanceMeters, hoodAngleDeg);
    }

    @Override
    public String getName() {
        return "LUT";
    }

    @Override
    public ShotCalculator.ShotResult calculateHubShot(
            Pose2d robotPose,
            ChassisSpeeds fieldSpeeds,
            Translation3d hubTarget,
            ShotCalculator.TurretConfig config,
            double currentTurretAngleDeg,
            double effectiveMinDeg,
            double effectiveMaxDeg,
            double hoodMinAngleDeg,
            double hoodMaxAngleDeg) {

        // Step 1: Calculate turret field position (reuse ShotCalculator helper)

        // Step 2: Calculate static distance to hub

        // Step 3: Initial TOF guess from table

        // Step 4: Iterative refinement (3-5 iterations)
        //   - predictTargetPos using current tof
        //   - recalculate distance to predicted target
        //   - look up new tof from table

        // Step 5: Look up RPM and hood angle for final distance

        // Step 6: Calculate turret angle (reuse ShotCalculator helper)

        // Step 7: Build and return ShotResult
    }
}
```

**Key decisions you need to make:**
- How many iterations? (Start with 3, same as our current code)
- What happens if the aim distance falls outside your table range?
  (Clamp? Extrapolate? Return unachievable?)
- Should you use the LUT's RPM or still use TrajectoryOptimizer for RPM?

**Exercise 3.3b**: Implement the `calculateHubShot` method. Before writing
code, write pseudocode on a whiteboard. Have another student review it. Then
translate to Java.

#### Step 4: Build the `HybridShotStrategy`

**File**: `src/main/java/frc/robot/subsystems/shooting/HybridShotStrategy.java`

This combines the best of both worlds:
- **TOF from the LUT** (captures real drag)
- **RPM + hood angle from TrajectoryOptimizer** (smooth, no gaps)

The implementation is similar to `LUTShotStrategy`, but:
- The iteration loop uses `tofTable.get(distance)` for TOF
- After convergence, it calls `TrajectoryOptimizer.calculateOptimalShot()`
  for the final RPM and hood angle (instead of looking those up in a table)

Think of it as: "Use the LUT to figure out WHERE to aim (lead compensation),
then use the parametric model to figure out HOW to aim (RPM/hood)."

```java
public class HybridShotStrategy implements ShotStrategy {

    // Only need a TOF table — RPM and hood come from TrajectoryOptimizer
    private final InterpolatingDoubleTreeMap tofTable = new InterpolatingDoubleTreeMap();

    public HybridShotStrategy() {
        // TODO: Populate with measured TOF data only
    }

    @Override
    public String getName() {
        return "Hybrid";
    }

    @Override
    public ShotCalculator.ShotResult calculateHubShot(
            Pose2d robotPose,
            ChassisSpeeds fieldSpeeds,
            Translation3d hubTarget,
            ShotCalculator.TurretConfig config,
            double currentTurretAngleDeg,
            double effectiveMinDeg,
            double effectiveMaxDeg,
            double hoodMinAngleDeg,
            double hoodMaxAngleDeg) {

        // Step 1: Turret field position

        // Step 2: Static distance

        // Step 3: TOF from LUT (not from physics)

        // Step 4: Iterate to find aim point (using LUT TOF at each step)

        // Step 5: TrajectoryOptimizer.calculateOptimalShot() for final RPM/hood

        // Step 6: Turret angle

        // Step 7: Return ShotResult
    }
}
```

**Exercise 3.3c**: What advantage does the hybrid have over pure LUT when
you're at a distance between your measured points (e.g., 3.7m when you
measured at 3m and 4m)?

#### Step 5: Integrate into `ShootingCoordinator`

Modify `ShootingCoordinator.java` to hold a `ShotStrategy` and use it:

1. Add a field: `private ShotStrategy activeStrategy;`
2. Add a `LoggedTunableNumber` to select strategy (0=Parametric, 1=LUT, 2=Hybrid)
3. In `updateShotCalculation`, replace the direct call to
   `ShotCalculator.calculateHubShot()` with `activeStrategy.calculateHubShot()`
4. Log which strategy is active: `Logger.recordOutput("Shots/Strategy", activeStrategy.getName())`
5. Check the tunable each cycle and swap strategies if it changed

**Important**: The pass shot calculation does NOT need to use the strategy
pattern — it's a different kind of shot with different physics. Keep it
as-is for now.

**Exercise 3.3d**: What are the risks of swapping strategies mid-match?
Should you allow it? What safeguards would you add?

### Session 3 Key Takeaways

1. The **Strategy pattern** lets you swap between different shot calculation
   approaches without changing the code that calls them.
2. The parametric wrapper should be a **thin delegate** with zero new logic —
   this guarantees you can always fall back to known-good behavior.
3. The hybrid approach (LUT TOF + parametric RPM/hood) is often the best
   tradeoff: you get drag-corrected aim points without needing to measure
   RPM and hood angle by hand.
4. Good interface design means the `ShootingCoordinator` doesn't care which
   strategy is active — it just calls `calculateHubShot()`.

---

## Session 4: Testing, Tuning, and Comparison

### 4.1 Simulation Testing (Before You Touch the Real Robot)

Our simulation environment lets you test all three strategies without
hardware. Here's how:

1. **Deploy to sim** (`./gradlew simulateJava`)
2. **Open AdvantageScope** and connect
3. **Drive the robot in sim** using keyboard/gamepad
4. **Toggle the strategy tunable** on the dashboard
5. **Compare**: For the same robot path, do the three strategies produce
   different aim points? Different RPMs? Different hood angles?

**What to log and compare:**

| Metric | Key in AdvantageScope |
|--------|-----------------------|
| Active strategy | `Shots/Strategy` |
| Aim point offset | `Turret/Shot/VelocityCompensation/AimOffsetM` |
| Exit velocity | `Turret/Shot/ExitVelocityMps` |
| Hood angle | `Turret/Shot/HoodAngleDeg` |
| Launch angle | `Turret/Shot/LaunchAngleDeg` |
| Target RPM | `SmartLaunch/Target/RPM` |
| Distance to target | `Turret/Shot/DistanceToTargetM` |

**Exercise 4.1a**: Create a sim test:
1. Position robot at (3, 4), facing the hub
2. Drive at constant 2 m/s toward the hub
3. Record the aim offset for all three strategies at distances 6m, 5m, 4m, 3m
4. Make a table comparing them. Which strategy leads more? Why?

### 4.2 Real Robot Data Collection

Once you've verified the code works in sim, it's time to collect real data
to populate the LUT tables.

**Data Collection Protocol:**

```
Equipment needed:
  - Measuring tape (8m+) or laser distance meter
  - Phone with 240fps slow-motion camera
  - Laptop with AdvantageScope open
  - Spreadsheet (Google Sheets works)

Procedure for each distance (2m, 3m, 4m, 5m, 6m, 7m):

  1. Position robot at measured distance from hub center
  2. Switch to Parametric strategy
  3. Let system calculate RPM and hood angle
  4. Fire 5 shots — record how many score
  5. If < 4/5 score, manually adjust RPM ±50 and hood ±1° until consistent
  6. Record FINAL values: (distance, RPM, hoodAngle)
  7. Record time-of-flight:
     - Start phone slow-mo recording
     - Fire 3 shots
     - Count frames from ball leaving shooter to ball entering hub
     - TOF = average(frames) / 240
  8. Move to next distance
```

**Spreadsheet template:**

| Distance (m) | RPM  | Hood Angle (deg) | TOF (s) | Shots Scored (out of 5) | Notes |
|---------------|------|-------------------|---------|--------------------------|-------|
| 2.0           |      |                   |         |                          |       |
| 3.0           |      |                   |         |                          |       |
| 4.0           |      |                   |         |                          |       |
| 5.0           |      |                   |         |                          |       |
| 6.0           |      |                   |         |                          |       |
| 7.0           |      |                   |         |                          |       |

### 4.3 Populating the LUT From Real Data

Once you have your spreadsheet, update the constructors in
`LUTShotStrategy` and `HybridShotStrategy`:

```java
// Example with real data (replace with YOUR measurements)
tofTable.put(2.0, 0.42);
tofTable.put(3.0, 0.58);
tofTable.put(4.0, 0.78);
tofTable.put(5.0, 1.01);
tofTable.put(6.0, 1.28);
tofTable.put(7.0, 1.59);
```

**Tip**: Compare your measured TOF to what the parametric model calculates.
The parametric value will be SHORTER because it ignores drag. The difference
tells you how much drag matters at each distance. If the difference is small
(< 0.05s), drag isn't a big factor and the hybrid approach won't help much.

### 4.4 Head-to-Head Comparison Test

Now the fun part: testing which approach actually scores better while moving.

**Test protocol:**

```
For each strategy (Parametric, LUT, Hybrid):

  Trial 1: Stationary shots at 4m (baseline — all should be equal)
    - Fire 10 shots, record scores

  Trial 2: Slow strafe at 1 m/s, perpendicular to hub, at 4m distance
    - Fire 10 shots, record scores

  Trial 3: Fast strafe at 2 m/s, perpendicular to hub, at 4m distance
    - Fire 10 shots, record scores

  Trial 4: Driving toward hub from 6m to 3m at ~1.5 m/s
    - Fire as many shots as possible, record total/scored

  Trial 5: Match-realistic path (pre-plan a short auto trajectory)
    - Run 3 times per strategy, record scores
```

Record everything. Build a comparison chart.

### 4.5 Advanced: Adding a Velocity Filter

If you notice over-leading (shots going past the hub when the driver
decelerates), consider adding a velocity filter to smooth the robot
speed input.

This would go in `ShootingCoordinator` before the strategy is called:

```java
// Exponential moving average for field speeds
private double filteredVx = 0;
private double filteredVy = 0;
private static final double ALPHA = 0.15; // lower = smoother, more lag

private ChassisSpeeds getFilteredSpeeds(ChassisSpeeds raw) {
    filteredVx = ALPHA * raw.vxMetersPerSecond + (1 - ALPHA) * filteredVx;
    filteredVy = ALPHA * raw.vyMetersPerSecond + (1 - ALPHA) * filteredVy;
    return new ChassisSpeeds(filteredVx, filteredVy, raw.omegaRadiansPerSecond);
}
```

**Exercise 4.5a**: What is the tradeoff of the `ALPHA` parameter?
- ALPHA = 1.0: No filtering (current behavior)
- ALPHA = 0.5: Moderate smoothing
- ALPHA = 0.1: Heavy smoothing (lots of lag)

Try different values and see how they affect shot accuracy during
acceleration and deceleration.

### Session 4 Key Takeaways

1. **Always test in sim first.** It's free, fast, and catches most bugs
   before you waste robot time.
2. Data collection is tedious but essential. A good spreadsheet is the
   difference between "our LUT works" and "our LUT kinda works sometimes."
3. The head-to-head comparison test is the **only thing that actually
   matters** — theory is great, but scored-shots-per-attempt is the metric.
4. A velocity filter (EMA) is a simple first attempt at smoothing the
   over-leading problem. Session 5 offers a more principled solution.

---

## Session 5: Smarter Velocity Prediction

### 5.1 The Problem With Instantaneous Velocity

This session is the most physics-heavy. The math is all kinematics you've
seen in physics class — no calculus required. If any equation looks
unfamiliar, try plugging in numbers first. The intuition follows.

By now you've built the strategy system, collected data, and run comparison
tests. If you ran Trial 4 (driving toward the hub) carefully, you probably
noticed something: **shots fired while the driver is decelerating tend to
miss long** (past the hub), and shots fired while accelerating miss short.

Why? Look at `ShotCalculator.predictTargetPos()` (line 185):

```java
double predictedX = target.getX() - fieldSpeeds.vxMetersPerSecond * timeOfFlight;
double predictedY = target.getY() - fieldSpeeds.vyMetersPerSecond * timeOfFlight;
```

This assumes the robot moves at **constant velocity** during the entire ball
flight. But that's almost never true:

```
Scenario: Robot moving at 2.0 m/s, driver releases joystick, ball TOF = 0.7s

What we calculate:     lead = 2.0 m/s × 0.7s = 1.40m
What actually happens: robot decelerates from 2.0 → 0.0 m/s over ~0.3s
                       actual displacement ≈ 0.30m
                       we over-led by 1.10m!
```

This is the single biggest accuracy problem in shoot-on-the-move, and it
applies equally to ALL three strategies (Parametric, LUT, Hybrid) because
they all use the same `predictTargetPos` function.

### 5.2 Understanding the Physics of Robot Deceleration

When the driver releases the stick, the swerve drive doesn't stop instantly.
The robot decelerates based on:
- Wheel friction with the carpet
- Motor braking (back-EMF)
- The drive controller's deceleration limits

For a typical FRC swerve, the deceleration profile looks roughly like:

```
Speed
  │
  │ ████
  │     ████
  │         ████
  │             ████
  │                 ████
  │                     ████
  └──────────────────────────── Time
  t=0                    t≈0.3s
  (stick released)       (stopped)
```

The displacement during deceleration is the area under this curve. For
constant deceleration `a`, the kinematic equation is:

```
displacement = v₀·t + ½·a·t²

Where:
  v₀ = initial velocity (what we measure now)
  a  = acceleration (negative when decelerating)
  t  = time of flight
```

Compare to what we currently calculate:

```
current:  displacement = v₀·t          (assumes constant velocity)
better:   displacement = v₀·t + ½·a·t² (accounts for acceleration)
```

The correction term `½·a·t²` is what we're missing.

### 5.3 Three Approaches to Smarter Prediction

#### Approach A: Acceleration-Compensated Prediction

Estimate the robot's current acceleration from recent velocity history, then
use kinematics to predict a better displacement.

**How to estimate acceleration:**

The robot's odometry gives us velocity every 20ms. We can estimate
acceleration by looking at the change in velocity over a short window:

```java
// Store recent velocities (ring buffer of last N cycles)
private static final int ACCEL_WINDOW = 5; // 5 cycles = 100ms
private double[] recentVx = new double[ACCEL_WINDOW];
private double[] recentVy = new double[ACCEL_WINDOW];
private int velocityIndex = 0;

public void recordVelocity(ChassisSpeeds fieldSpeeds) {
    recentVx[velocityIndex] = fieldSpeeds.vxMetersPerSecond;
    recentVy[velocityIndex] = fieldSpeeds.vyMetersPerSecond;
    velocityIndex = (velocityIndex + 1) % ACCEL_WINDOW;
}

public double[] estimateAcceleration() {
    // Oldest sample is at velocityIndex (just got overwritten = oldest)
    // Newest sample is at velocityIndex - 1
    int oldest = velocityIndex;
    int newest = (velocityIndex - 1 + ACCEL_WINDOW) % ACCEL_WINDOW;

    double dt = ACCEL_WINDOW * 0.020; // 100ms window
    double ax = (recentVx[newest] - recentVx[oldest]) / dt;
    double ay = (recentVy[newest] - recentVy[oldest]) / dt;

    return new double[] {ax, ay};
}
```

**Then modify the prediction:**

```java
public static Translation3d predictTargetPosWithAccel(
        Translation3d target,
        ChassisSpeeds fieldSpeeds,
        double ax, double ay,
        double timeOfFlight) {

    // Kinematic prediction: x = v*t + 0.5*a*t²
    double displacementX = fieldSpeeds.vxMetersPerSecond * timeOfFlight
                         + 0.5 * ax * timeOfFlight * timeOfFlight;
    double displacementY = fieldSpeeds.vyMetersPerSecond * timeOfFlight
                         + 0.5 * ay * timeOfFlight * timeOfFlight;

    return new Translation3d(
        target.getX() - displacementX,
        target.getY() - displacementY,
        target.getZ());
}
```

**Pros**: Physically motivated, adapts to real deceleration rate.
**Cons**: Acceleration estimate is noisy (velocity changes are small per
cycle). Sensitive to the window size — too small = noisy, too large = laggy.

**Exercise 5.3a**: Work through the math. If the robot is at 2.0 m/s and
decelerating at -6.0 m/s², with a TOF of 0.7s:
1. What does our current code predict for displacement?
2. What does the acceleration-compensated version predict?
3. At what TOF does the acceleration term matter more than 10cm?

*Hint for self-checking*: For part 1, displacement = v×t = 2.0 × 0.7 = 1.40m.
For part 2, add the ½at² term. For part 3, solve |½ × (-6) × t²| > 0.10.

#### Approach B: Command-Based Prediction

Instead of measuring what the robot IS doing, look at what the driver is
COMMANDING it to do. If the joystick is at zero, the robot is about to stop.

The idea: the drive subsystem already knows the commanded `ChassisSpeeds`
from the joystick. We can use that as a better predictor of future velocity.

```java
// In ShootingCoordinator, receive both measured and commanded speeds
private Supplier<ChassisSpeeds> commandedSpeedsSupplier; // from drive joystick

public Translation3d predictTargetWithCommanded(
        Translation3d target,
        ChassisSpeeds measuredSpeeds,
        ChassisSpeeds commandedSpeeds,
        double timeOfFlight) {

    // Estimate: robot will transition from measured to commanded
    // over approximately 0.2-0.3 seconds (drive response time)
    double transitionTime = 0.25; // tune this

    double effectiveVx, effectiveVy;
    if (timeOfFlight <= transitionTime) {
        // Short flight — mostly current velocity
        double blend = timeOfFlight / transitionTime;
        effectiveVx = measuredSpeeds.vxMetersPerSecond * (1 - blend * 0.5)
                    + commandedSpeeds.vxMetersPerSecond * (blend * 0.5);
        effectiveVy = measuredSpeeds.vyMetersPerSecond * (1 - blend * 0.5)
                    + commandedSpeeds.vyMetersPerSecond * (blend * 0.5);
    } else {
        // Longer flight — weight toward commanded velocity
        double fractionAtCurrent = transitionTime / timeOfFlight;
        effectiveVx = measuredSpeeds.vxMetersPerSecond * fractionAtCurrent
                    + commandedSpeeds.vxMetersPerSecond * (1 - fractionAtCurrent);
        effectiveVy = measuredSpeeds.vyMetersPerSecond * fractionAtCurrent
                    + commandedSpeeds.vyMetersPerSecond * (1 - fractionAtCurrent);
    }

    return new Translation3d(
        target.getX() - effectiveVx * timeOfFlight,
        target.getY() - effectiveVy * timeOfFlight,
        target.getZ());
}
```

**Pros**: Doesn't need noisy acceleration estimates. Directly captures driver
intent — if they let go of the stick, we immediately know to reduce the lead.
**Cons**: Requires plumbing the commanded speeds from the drive subsystem to
the shooting system. The `transitionTime` constant is a guess.

**Exercise 5.3b**: Think about edge cases:
1. Driver is holding stick forward (commanded = measured). What happens?
2. Driver slams stick in opposite direction. What happens?
3. Auto mode — commanded speeds come from trajectory follower. Is this better
   or worse than teleop?

#### Approach C: Blended Prediction (Stretch Goal — Team 4744's Insight)

*This approach uses linear regression, which you may not have seen yet.
It's here for completeness and for students who want a challenge. If the
math below feels unfamiliar, skip to Section 5.4 — Approaches A and B
are fully sufficient.*

Team 4744 (Ninjas) from the Chief Delphi thread noted that the hardest part
isn't predicting pose — it's predicting **future velocity**. Their approach:
fit a regression line through recent velocity samples and extrapolate.

```
Recent velocity samples (vx over last 200ms):

    vx
     │     *
     │   *   *
     │  *      *
     │ *         *  ← regression line
     │*             *  ← extrapolated
     └──────────────────── time
     -200ms    now    +TOF
```

This is essentially what Approach A does, but framed differently: instead
of computing a single acceleration value, you fit a line to velocity history
and extrapolate it forward.

```java
// Linear regression on recent velocity samples
// Returns: predicted average velocity over the next `duration` seconds

public double predictAverageVelocity(double[] recentSamples, double dt, double duration) {
    int n = recentSamples.length;
    // Fit line: v(t) = v0 + a*t
    // Using simple linear regression
    double sumT = 0, sumV = 0, sumTV = 0, sumTT = 0;
    for (int i = 0; i < n; i++) {
        double t = i * dt;
        sumT += t;
        sumV += recentSamples[i];
        sumTV += t * recentSamples[i];
        sumTT += t * t;
    }
    double a = (n * sumTV - sumT * sumV) / (n * sumTT - sumT * sumT); // slope (acceleration)
    double v0 = (sumV - a * sumT) / n; // intercept at t=0

    // Current velocity (end of sample window)
    double vNow = v0 + a * (n - 1) * dt;

    // Average velocity over next `duration` seconds:
    // integral of (vNow + a*t) from 0 to duration, divided by duration
    return vNow + 0.5 * a * duration;
}
```

**Pros**: Smooths noisy acceleration naturally (regression is a built-in filter).
**Cons**: Most complex to implement. Assumes acceleration is linear (constant
jerk), which isn't always true.

### 5.4 Implementation Guide

Pick ONE approach to start with. We recommend **Approach A** (acceleration-
compensated) because:
- It's the simplest to implement
- The math is easy to verify
- It integrates cleanly into the existing `predictTargetPos` pattern

**Where it goes in the code:**

The velocity prediction is **independent of the shot strategy**. All three
strategies (Parametric, LUT, Hybrid) call `predictTargetPos` in their
iteration loop. So you modify the prediction once and all strategies benefit.

**Step 1**: Add acceleration estimation to `ShootingCoordinator`
- Store a ring buffer of recent field speeds
- Update it every `periodic()` cycle
- Compute acceleration from the buffer

**Step 2**: Create a new `predictTargetPos` overload in `ShotCalculator`
- Takes acceleration as additional parameters
- Uses `displacement = v*t + 0.5*a*t²` instead of `displacement = v*t`

**Step 3**: Add a tunable to toggle between simple and acceleration-compensated
- `Shots/VelocityPrediction/UseAcceleration` (boolean, default false)
- This lets you A/B test in the same match

**Step 4**: Wire it through the strategy interface
- The `ShotStrategy.calculateHubShot()` signature needs the acceleration
  data, OR the coordinator pre-computes a "predicted speeds" object

Think carefully about Step 4 — should acceleration be part of the strategy
interface, or should it be applied before the strategy sees the speeds?

**Exercise 5.4a**: Design discussion — if you pre-filter the speeds before
passing them to the strategy, the strategy doesn't know the difference
between filtered and unfiltered speeds. Is that good (simpler interface) or
bad (strategy can't make informed decisions)? Argue both sides.

### 5.5 Testing Velocity Prediction

The best test for this is **shoot-while-decelerating**:

```
Test: Drive at 2 m/s toward the hub. At 5m distance, release the stick
and fire immediately.

                    ┌── fire here
                    ▼
  ●═══════════════►◯ · · · · · ▣
  start (8m)      5m           hub

With instantaneous velocity:  lead = 2.0 × TOF (too much — robot is stopping)
With acceleration comp:       lead = (2.0 × TOF) + (0.5 × (-6) × TOF²) (smaller, correct)
```

Run this test 10 times with each prediction mode and compare accuracy.

**What to log:**

| Metric | Key |
|--------|-----|
| Measured velocity | `Turret/Shot/VelocityCompensation/RobotSpeedMps` |
| Estimated acceleration | `Shots/VelocityPrediction/AccelMagnitude` |
| Predicted displacement | `Shots/VelocityPrediction/PredictedDisplacementM` |
| Simple displacement | `Shots/VelocityPrediction/SimpleDisplacementM` |
| Aim offset delta | `Shots/VelocityPrediction/AccelCorrectionM` |

The `AccelCorrectionM` value tells you exactly how much the acceleration
term changed the aim point. If it's consistently < 2cm, acceleration
compensation isn't helping and you can skip it.

### 5.6 When Velocity Prediction Matters (and When It Doesn't)

An important insight: velocity prediction matters MOST when:
- The robot is **changing speed** (accelerating or decelerating)
- The TOF is **long** (the `½at²` term grows with t²)
- The speed is **high** (bigger initial lead = bigger error if wrong)

It matters LEAST when:
- The robot is at **constant velocity** (a ≈ 0, no correction needed)
- The TOF is **short** (close to hub, correction is tiny)
- The robot is **slow** (small lead, small error either way)

This means the acceleration correction is most valuable for **long-range
shots taken during driver transitions** — exactly the hardest shots in a
match. For short-range stationary shots, all approaches give the same answer.

**Exercise 5.6a**: Calculate the acceleration correction `½at²` for these
scenarios and decide which ones benefit from Approach A:

| Scenario | Speed | Accel | TOF | ½at² | Worth it? |
|----------|-------|-------|-----|------|-----------|
| Close, steady | 1.0 m/s | 0 m/s² | 0.4s | ? | |
| Far, steady | 1.5 m/s | 0 m/s² | 1.0s | ? | |
| Close, braking | 2.0 m/s | -6 m/s² | 0.4s | ? | |
| Far, braking | 2.0 m/s | -6 m/s² | 1.0s | ? | |
| Far, accelerating | 0.5 m/s | +4 m/s² | 1.0s | ? | |

*Hint*: Two of these rows have zero acceleration, so ½at² = 0. For the
others, just plug in. You should find that "Far, braking" has a correction
of 3.0m — enormous. "Close, braking" is 0.48m — still significant. The
pattern: it matters when |a| is large AND TOF is long.

### 5.7 What About Pass Shots?

Everything in this session applies to pass shots too — and arguably matters
**more** for them.

Look at `ShotCalculator.calculatePassShot()` — it calls the same
`predictTargetPos()` function to lead the target. But pass shots are
different in two ways that make the acceleration problem worse:

1. **Longer TOF.** Pass shots are typically lower-velocity, longer-arc lobs.
   A hub shot at 5m might have a TOF of ~0.8s; a pass over the same distance
   might be 1.2s+. Since the acceleration correction is `½at²`, doubling the
   TOF **quadruples** the error.

2. **Larger distances.** You're often passing 6-8+ meters across the field.
   The combination of long distance (high TOF) and high robot speed (driving
   into position) means the correction term can easily be 0.5m or more.

The good news: because `predictTargetPos` is shared infrastructure, the
acceleration-compensated version you build in this session fixes both shot
types with zero extra work. You don't need a strategy pattern for passes —
the parametric pass calculation is fine. You just need the aim point to be
correct, and that's exactly what better velocity prediction gives you.

**Exercise 5.7a**: A pass shot has TOF = 1.3s. The robot is at 2.5 m/s and
decelerating at -5 m/s². Calculate:
1. The over-lead with the current (constant velocity) prediction
2. The corrected lead with acceleration compensation
3. Compare the error to the width of a robot (~0.7m). Could this be the
   difference between a catchable and uncatchable pass?

*Hint for self-checking*: The constant-velocity lead is 2.5 × 1.3 = 3.25m.
The acceleration correction ½at² = 0.5 × (-5) × 1.69 = -4.225m. Think
about what that means — the robot will actually STOP before the ball
arrives (v₀/|a| = 0.5s < 1.3s TOF), so you'd also want to clamp the
prediction. This is a real edge case worth discussing.

### Session 5 Key Takeaways

1. The biggest accuracy problem in shoot-on-the-move is the **constant
   velocity assumption** — the robot is almost never at constant velocity
   when the driver is maneuvering.
2. The correction term `½at²` grows with the **square** of TOF, so it
   matters most for long-range shots and passes.
3. Acceleration can be estimated from recent velocity history. The tradeoff
   is noise (short window) vs. lag (long window).
4. This fix lives in `predictTargetPos` — shared infrastructure — so it
   benefits ALL strategies and both hub shots and pass shots.

---

## Summary: Comparison of Approaches

| Aspect | Parametric (Current) | LUT (Pure Lookup) | Hybrid (LUT TOF + Parametric RPM) |
|--------|---------------------|--------------------|------------------------------------|
| **Setup time** | None — just physics | Hours of measurement | 30 min (TOF only) |
| **Air drag** | Ignored | Captured in data | Captured in TOF data |
| **Distance gaps** | None — continuous | Interpolation only | Best of both |
| **Hardware changes** | Update constants | Re-measure everything | Re-measure TOF only |
| **Complexity** | Medium | Low | Medium |
| **Velocity prediction** | Add in Session 5 | Add in Session 5 | Add in Session 5 |
| **Our recommendation** | Keep as default | Build for comparison | Try as upgrade path |

---

## References

- [Chief Delphi: Poll — How Are You Shooting on the Move?](https://www.chiefdelphi.com/t/poll-how-are-you-shooting-on-the-move/514641)
- [Chief Delphi: Recursive TOF Fire Control Simulator (Interactive)](https://www.chiefdelphi.com/t/recursive-time-of-flight-fire-control-simulator-for-frc-docs-preview/513819)
- [Chief Delphi: Shoot on the Move from the Code Perspective](https://www.chiefdelphi.com/t/shoot-on-the-move-from-the-code-perspective/511815)
- [Chief Delphi: Time of Flight Determination](https://www.chiefdelphi.com/t/time-of-flight-determination/512542/25)
- [Chief Delphi: Real-Time vs Lookup Table for Parabolic Shooter Angle](https://www.chiefdelphi.com/t/real-time-vs-lookup-table-for-parabolic-shooter-angle/512732)
- [WPILib Docs: InterpolatingDoubleTreeMap](https://github.wpilib.org/allwpilib/docs/release/java/edu/wpi/first/math/interpolation/InterpolatingDoubleTreeMap.html)
- [6328 Mechanical Advantage 2026 Build Thread](https://www.chiefdelphi.com/t/frc-6328-mechanical-advantage-2026-build-thread/509595/421)

---

## File Map: Where Everything Lives

```
Existing files (READ THESE FIRST):
  ShotCalculator.java         → Physics, TOF, velocity compensation, turret angles
  TrajectoryOptimizer.java    → Two-point trajectory solve, RPM/hood calculation
  ShootingCoordinator.java    → Orchestration, auto-shoot, dispatching

Files you will create:
  ShotStrategy.java           → Interface (Step 1)
  ParametricShotStrategy.java → Wraps existing code (Step 2)
  LUTShotStrategy.java        → Pure lookup table approach (Step 3)
  HybridShotStrategy.java     → LUT TOF + parametric RPM/hood (Step 4)

Files you will modify:
  ShootingCoordinator.java    → Add strategy selection + delegation (Step 5)
```

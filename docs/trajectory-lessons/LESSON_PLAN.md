# From Turret to Trajectory: A Robotics Physics Journey

## Welcome, Future Robot Engineers!

In this lesson series, you'll transform a simple turret test platform into a sophisticated scoring machine. Along the way, you'll learn motor control, physics simulation, ballistic calculations, and how to build complex robot behaviors - all while seeing your code come to life in simulation!

**What you'll build:**
- A flywheel launcher that shoots balls
- System identification to tune motor performance
- A physics-based trajectory optimizer
- Real-time 3D ball flight simulation
- Coordinated shooting commands

---

## How This Course Works

Each lesson follows a pattern:

1. **Think First** - We pose a problem and ask you to think about how to solve it
2. **Experiment** - Try your ideas in simulation
3. **Learn** - See the solution and understand why it works
4. **Extend** - Challenge problems to deepen understanding

You'll work with Claude as your coding partner. You can ask questions, propose solutions, and have Claude help you implement and test your ideas.

**Run the simulator frequently!** After each section, run:
```bash
./gradlew simulateJava
```
Then open AdvantageScope to see your robot in action.

---

## Course Overview

| Module | Topic | Key Concepts |
|--------|-------|--------------|
| 1 | Meet Your Turret | Existing code, simulation basics |
| 2 | Building a Launcher | Motor control, flywheel physics |
| 3 | Tuning with Science | System identification (SysId) |
| 4 | The Physics of Flight | Projectile motion, optimal trajectories |
| 5 | Simulating Reality | 3D physics, collision detection |
| 6 | Putting It Together | Command composition, state machines |

---

# Module 1: Meet Your Turret

## Learning Objectives
- Understand the existing codebase structure
- Run the simulation and see the turret in action
- Understand the IO pattern for hardware abstraction

## Explore: What's Already Here?

Your starting point is **TurretBot** - a test platform with:
- A rotating turret (but no launcher!)
- Virtual driving (keyboard/joystick controls where the robot "is" without actual wheels)
- Basic aiming calculations

### Activity 1.1: Run the Simulation

```bash
./gradlew simulateJava
```

Open AdvantageScope and connect to the simulation. You should see:
- A robot that can be positioned on the field
- A turret that rotates to aim at targets
- BUT... no way to actually shoot anything!

### Activity 1.2: Explore the Code

Look at these key files:
- `src/main/java/frc/robot/subsystems/turret/Turret.java` - The turret subsystem
- `src/main/java/frc/robot/TurretBotConfig.java` - Robot configuration
- `src/main/java/frc/robot/subsystems/turret/TurretIOSim.java` - Simulation IO

**Discussion Questions:**
1. What does the IO pattern give us? Why separate `Turret` from `TurretIOSim`?
2. How does the turret know where to aim?
3. What's missing that prevents us from scoring?

---

# Module 2: Building a Launcher

## Learning Objectives
- Understand flywheel physics
- Implement motor velocity control
- Create a subsystem with the IO pattern

## Think First: How Do Flywheels Work?

Before we write code, let's think about the physics:

### Thought Experiment 2.1: Velocity Transfer

You have two spinning wheels (flywheels) and you drop a ball between them.

**Questions to discuss:**
1. If the wheels spin at 3000 RPM, how fast does the ball come out?
2. Is it the same speed as the wheel surface? Faster? Slower? Why?
3. What physical factors affect the transfer efficiency?

*Take a few minutes to think about this before reading on...*

<details>
<summary>Click to reveal the physics</summary>

The ball's exit velocity depends on:
- **Wheel surface velocity** = RPM × 2π × radius / 60 (converts RPM to m/s)
- **Efficiency factor** (typically 60-90%) - accounts for:
  - Ball compression
  - Slip between ball and wheel
  - Energy lost to deformation

Formula: `exitVelocity = (RPM × 2π × radius / 60) × efficiency`

A 4-inch diameter wheel at 3000 RPM with 70% efficiency:
- Surface velocity = 3000 × 2π × 0.0508 / 60 = 15.96 m/s
- Exit velocity = 15.96 × 0.70 = **11.17 m/s** (about 25 mph)
</details>

### Thought Experiment 2.2: Dual Motors

We're using two motors on a single flywheel assembly.

**Questions:**
1. Why use two motors instead of one?
2. Should they spin in the same direction or opposite?
3. What happens if one motor spins faster than the other?

## Build: Create the Launcher Subsystem

Now let's build! You'll create:
1. `LauncherIO.java` - The interface
2. `LauncherIOSim.java` - Simulation implementation
3. `Launcher.java` - The subsystem

### Step 2.1: Design the Interface

**Your task:** Before looking at the solution, design what methods `LauncherIO` should have.

Think about:
- What inputs do we read FROM the motors?
- What outputs do we send TO the motors?

*Try designing this yourself, then ask Claude to help implement it...*

### Step 2.2: Implement the Simulation

The simulation needs to model:
- Motor physics (inertia, acceleration)
- Current RPM tracking
- Response to voltage commands

**Challenge:** Can you make the simulated flywheel take realistic time to spin up? A real flywheel might take 1-2 seconds to reach full speed.

### Step 2.3: Wire It Up

Add the launcher to `RobotContainer.java` and create bindings to control it.

## Checkpoint: Test Your Launcher

Run the simulation and verify:
- [ ] Flywheel spins up when commanded
- [ ] RPM displays correctly in AdvantageScope
- [ ] Flywheel takes realistic time to spin up and down
- [ ] Velocity is logged and matches expectations

---

# Module 3: Tuning with Science (System Identification)

## Learning Objectives
- Understand feedforward control
- Learn what SysId measures and why
- Implement characterization routines

## Think First: Why Tune?

Your launcher works, but how accurate is it? If you command 3000 RPM, do you actually get 3000 RPM?

### Thought Experiment 3.1: Feedforward vs Feedback

**Scenario:** You want the flywheel at exactly 2500 RPM.

**Feedback-only approach:**
1. Command 0 volts
2. Measure: 0 RPM (error = 2500)
3. Increase voltage based on error
4. Measure: 1000 RPM (error = 1500)
5. Keep adjusting...

**Feedforward approach:**
1. Calculate: "2500 RPM needs about 8.5 volts"
2. Command 8.5 volts immediately
3. Minor PID corrections for the remaining error

**Questions:**
1. Which approach reaches the target faster?
2. How do we figure out that "2500 RPM needs 8.5 volts"?
3. What if the battery voltage changes during a match?

## The SysId Process

System Identification (SysId) helps us measure:

- **kS (Static friction):** Minimum voltage to overcome friction and start moving
- **kV (Velocity constant):** Volts per unit velocity at steady state
- **kA (Acceleration constant):** Volts per unit acceleration (optional for flywheels)

The feedforward equation: `voltage = kS × sign(velocity) + kV × velocity`

### Activity 3.1: Run Characterization

We'll implement a quasistatic test:
1. Slowly ramp up voltage (0.1V per second)
2. Record voltage and resulting velocity
3. Plot voltage vs velocity
4. Find the line of best fit: slope = kV, y-intercept = kS

**Your task:** Implement `feedforwardCharacterization()` command in the launcher.

## Build: Add SysId to Your Launcher

1. Create a characterization command that ramps voltage
2. Log voltage and velocity each loop
3. Use AdvantageScope or a spreadsheet to analyze the data
4. Update your feedforward constants

### Challenge Problem 3.1: Velocity Limits

During SysId, the flywheel might spin dangerously fast. How do you add safety limits?

**Hint:** What should happen if velocity exceeds a threshold during characterization?

## Checkpoint: Verify Tuning

After SysId, your launcher should:
- [ ] Reach commanded RPM within 0.5 seconds
- [ ] Hold RPM steady (±50 RPM) under load
- [ ] kS and kV values are logged and sensible

---

# Module 4: The Physics of Flight

## Learning Objectives
- Apply projectile motion equations
- Understand constrained optimization
- Calculate optimal launch parameters

## Think First: The Trajectory Problem

You're at position (x=0, z=0.5m) and need to hit a target at (x=5m, z=1.8m).

### Thought Experiment 4.1: Infinite Solutions?

**Question:** Given a fixed target, how many different trajectories can hit it?

*Think about this before revealing the answer...*

<details>
<summary>Reveal</summary>

**Infinitely many!** For any target, you can:
- Shoot fast at a low angle
- Shoot slow at a high angle
- Anything in between!

Each (velocity, angle) pair that satisfies projectile motion equations is a valid solution.
</details>

### Thought Experiment 4.2: Adding Constraints

The hub has a lip at 1.83m height, and we want the ball to land in the center at 1.43m.

**New constraints:**
1. Ball must clear the lip (height > 1.83m at edge)
2. Ball must be DESCENDING when it crosses the lip
3. Ball must land near the center (not bounce off the far wall)

**Question:** With these constraints, how many valid trajectories are there for a given distance?

<details>
<summary>Reveal</summary>

**Exactly ONE!** Given:
- The descent angle (angle of the ball's path as it crosses the lip)
- The distance to the target

There is exactly one (velocity, launch_angle) combination that satisfies all constraints.

This is the key insight that makes our trajectory optimizer work!
</details>

## The Math: Two-Point Trajectory

The trajectory must pass through two points:
- **Point A (hub edge):** Where ball crosses the lip, descending at specified angle
- **Point B (hub center):** Where ball should land

Projectile motion equation:
```
y = x × tan(θ) - g × x² / (2 × v² × cos²(θ))
```

Given two points (x₁, y₁) and (x₂, y₂), we can solve for both θ and v.

### Activity 4.1: Derive the Solution

**Challenge:** Given the projectile equation and two points, derive formulas for:
1. tan(θ) in terms of x₁, y₁, x₂, y₂
2. v² in terms of θ, x₁, y₁, g

*Hint: Substitute both points into the equation, then eliminate one variable...*

## Build: Create TrajectoryOptimizer

Your optimizer needs to:
1. Calculate hub edge height from descent angle
2. Solve for launch angle and velocity
3. Validate all constraints (angle limits, RPM limits, peak height)
4. Return the optimal shot parameters

### Step 4.1: Hub Geometry

First, understand the target:
```
Hub lip height: 1.83m
Hub center height: 1.43m
Hub radius: 0.53m (distance from edge to center)
```

### Step 4.2: Implement the Math

Create `TrajectoryOptimizer.java` with:
- `calculateOptimalShot(turretPosition, target)` - main entry point
- `calculateTrajectoryThroughTwoPoints(...)` - the math
- Validation and logging

### Step 4.3: Integrate with TurretCalculator

Update `TurretCalculator.java` to:
- Use `TrajectoryOptimizer` for optimal RPM/angle
- Convert between RPM and velocity
- Feed results to turret visualizer

## Checkpoint: Visualize Trajectories

Run simulation and verify:
- [ ] Trajectory arc displays in AdvantageScope
- [ ] Adjusting descent angle changes the trajectory shape
- [ ] Invalid shots (too far, wrong angle) are rejected with clear messages

---

# Module 5: Simulating Reality

## Learning Objectives
- Understand physics simulation concepts
- Implement collision detection
- Create visual feedback for testing

## Think First: What Should We Simulate?

### Thought Experiment 5.1: Minimum Viable Simulation

If we can only simulate a few things, what's most important?

**Rank these by importance:**
- [ ] Ball gravity
- [ ] Ball-wall collisions
- [ ] Ball-ball collisions
- [ ] Ball-robot collisions
- [ ] Air resistance
- [ ] Ball spin/magnus effect

*Discuss with your team before reading on...*

<details>
<summary>Our prioritization</summary>

1. **Gravity** - Essential, balls must fall!
2. **Ball-wall collisions** - Essential for field boundaries and hub
3. **Ball-robot collisions** - Important for realistic robot interaction
4. **Ball-ball collisions** - Nice to have, expensive to compute
5. **Air resistance** - Minimal effect for our distances/speeds
6. **Spin/magnus** - Cool but complex, skip for now
</details>

### Thought Experiment 5.2: Performance vs Accuracy

The field can have 400+ balls. Checking every ball against every other ball is O(n²) = 160,000 checks per frame!

**Question:** How can we reduce this?

*Hint: Think about spatial locality...*

<details>
<summary>The solution: Spatial Grid</summary>

Divide the field into a grid of cells. Each ball is only in one cell. Only check collisions within neighboring cells!

If cells are 0.25m and field is ~16m × 8m, that's 64×32 = 2048 cells.
Most cells have 0-2 balls. Collision checks drop from 160,000 to a few hundred.
</details>

## Explore: FuelSim Deep Dive

Read through `src/main/java/frc/robot/util/FuelSim.java` and find:

1. **The Fuel class** - How is ball state stored?
2. **The physics loop** - How is gravity applied?
3. **Collision handling** - How do bounces work?
4. **The spatial grid** - How is it implemented?

### Activity 5.1: Trace a Collision

Walk through the code mentally or with debug prints:
1. Ball is at (5.0, 4.0, 0.5) moving at (3.0, 0, -2.0) m/s
2. After one timestep (0.02s / 5 subticks), where is it?
3. If it hits the ground, how is velocity changed?

## Build: Integrate FuelSim

1. Register the robot with FuelSim in `RobotContainer`
2. Spawn starting fuel positions
3. Add shooting to spawn balls with correct position and velocity
4. Log and visualize in AdvantageScope

### Challenge Problem 5.1: Coefficient of Restitution

The code has `FIELD_COR = Math.sqrt(22 / 51.5)` (~0.653).

**Questions:**
1. What is coefficient of restitution?
2. How would you measure it for a real ball on carpet?
3. Why might COR be different for ball-ball vs ball-wall collisions?

## Checkpoint: Watch Balls Fly!

Run simulation and verify:
- [ ] Balls spawn in starting positions
- [ ] Shooting creates balls with correct velocity
- [ ] Balls follow realistic arcs (gravity works!)
- [ ] Balls bounce off walls, hub, and robot
- [ ] Scoring is counted when balls enter hub

---

# Module 6: Putting It Together

## Learning Objectives
- Compose complex behaviors from simple commands
- Understand state machines and conditions
- Implement match phase awareness

## Think First: The Shooting Sequence

### Thought Experiment 6.1: What Must Be True to Shoot?

Before pulling the trigger, what conditions must be met?

**Brainstorm a list:**
- ???
- ???
- ???

*Try to list at least 5 conditions before revealing...*

<details>
<summary>Our conditions</summary>

1. **Turret at target** - Must be aimed correctly (within 2°)
2. **Launcher at speed** - Flywheel must be at setpoint RPM (within 50 RPM)
3. **Hub is active** - Game rules: can only score during certain phases
4. **Fuel available** - Must have balls to shoot
5. **Minimum delay elapsed** - Prevent jamming from shooting too fast
6. **Hood at angle** - Launch angle must be correct
</details>

### Thought Experiment 6.2: Shooting While Moving

If the robot is driving while shooting, what extra challenges exist?

**Questions:**
1. Where should the turret aim?
2. How does robot velocity affect the ball's trajectory?
3. How do you predict where the target will be when the ball arrives?

## Build: ShootingCommands

Create `ShootingCommands.java` with three phases:

### Phase 1: Shoot Until Empty
- Robot stationary
- Turret aims, launcher spins up
- Shoot all available fuel
- Stop when empty

### Phase 2: Continuous Shooting
- Integrate with intake
- Keep launcher spinning
- Shoot whenever conditions are met

### Phase 3: Shoot While Moving
- Driver controls movement
- Turret tracks target with lead compensation
- Predict where target will be based on time of flight

### Step 6.1: Match Phase Tracking

The 2026 game has complex scoring rules:
- Auto (0-20s): Both hubs active
- Shift phases: Hubs alternate based on auto winner
- End game (130-160s): Both hubs active

Implement `MatchPhaseTracker.java` to:
- Track current match time
- Determine which hub(s) are active
- Provide this info to shooting commands

### Step 6.2: Zone-Aware Aiming

Based on robot position:
- In alliance zone → Aim at scoring hub
- In neutral/opponent zone → Aim at pass target

Implement `TurretAimingHelper` with hysteresis to prevent flickering.

## Checkpoint: Full Scoring System

Run simulation through a mock match:
- [ ] Auto: Robot shoots into hub
- [ ] Teleop: Hub phases switch correctly
- [ ] Shooting respects all conditions
- [ ] Moving shots work with lead compensation
- [ ] Score is tracked correctly

---

# Final Challenge: Optimization Competition

Now that you have a working system, let's optimize!

## Challenge 1: Speed
How fast can you empty 10 balls into the hub from various distances?

**Metrics:**
- Time from start to 10 scored
- Number of misses

## Challenge 2: Accuracy
How accurate is your system at various distances?

**Test positions:**
- 3 meters from hub
- 5 meters from hub
- 7 meters from hub (max range?)

**Metrics:**
- Shots made / shots attempted
- Average deviation from center

## Challenge 3: Moving Shots
Can you score while driving in a circle around the hub?

**Setup:**
- Drive in a 2m radius circle at 1 m/s
- Continuously shoot

**Metrics:**
- Score rate (balls/minute)
- Miss rate

## Challenge 4: Parameter Tuning

Experiment with these tunable parameters:
- Descent angle (60° default)
- Launch efficiency (0.70 default)
- Minimum/maximum clearance
- Shoot delay between balls

Can you find better values than the defaults?

---

# Appendix A: Key Equations

## Projectile Motion
```
x(t) = v₀ × cos(θ) × t
y(t) = v₀ × sin(θ) × t - ½ × g × t²
```

## Flywheel Velocity
```
surfaceVelocity = RPM × 2π × radius / 60
exitVelocity = surfaceVelocity × efficiency
```

## Feedforward Control
```
voltage = kS × sign(velocity) + kV × velocity + kA × acceleration
```

## Two-Point Trajectory Solution
```
tan(θ) = (y₁×x₂² - y₂×x₁²) / (x₁×x₂×(x₂ - x₁))
K = (x₁×tan(θ) - y₁) / x₁²
v² = g / (2×K×cos²(θ))
```

---

# Appendix B: Useful Tools

## Simulation
```bash
./gradlew simulateJava
```

## AdvantageScope Visualizations
- `FuelSim/Fuels` - 3D ball positions
- `Shooter/Trajectory/*` - Planned trajectory
- `Turret/CurrentAngle` - Where turret is pointing
- `Launcher/VelocityRPM` - Flywheel speed

## Tunable Parameters (in NetworkTables)
- `Trajectory/DescentAngleDeg`
- `Shooter/LaunchEfficiency`
- `Shooter/WheelRadiusMeters`

---

# Appendix C: Getting Help

Working with Claude as your coding partner:

**Good prompts:**
- "I'm trying to understand how the spatial grid collision detection works. Can you walk me through it?"
- "My flywheel isn't reaching the target RPM. What should I check?"
- "How do I add logging to see my trajectory calculations?"

**Better prompts:**
- "The ball is landing 0.5m short of the hub. I've checked that exit velocity is correct. What else could cause this?"
- "I want to add a new constraint that prevents shooting if the robot is rotating faster than 30°/sec. Where should this check go?"

**Great prompts:**
- "I hypothesize that the efficiency factor should be distance-dependent because at longer distances, the ball compresses more. How would I test this hypothesis and implement it if correct?"

---

Good luck, and have fun building your scoring machine!

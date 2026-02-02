# Senior Track: Trajectory Optimization Deep Dive

## Overview

You'll work through the key engineering challenges we solved building this scoring system. Each section poses a problem, asks you to think about it, then shows our solution.

**Time:** ~3-4 hours
**Prerequisites:** Basic physics (projectile motion), Java familiarity
**Style:** Independent work with Claude as your coding partner

---

## Section 1: The Optimization Problem (45 min)

### The Challenge

You have a flywheel launcher and an adjustable hood. Given a target at distance `d` and height `h`, find the optimal:
- **RPM** (wheel speed)
- **Launch angle** (hood position)

### Think First (10 min)

**Question 1:** For a fixed target, how many (velocity, angle) pairs can hit it?

**Question 2:** If there are multiple solutions, how do we pick the "best" one?

**Question 3:** What physical constraints limit our choices?

*Write down your answers before continuing...*

---

### The Insight: Constrained Optimization

The hub has geometry that constrains the problem:

```
Hub lip:     1.83m height, 0.53m radius
Hub center:  1.43m height (where ball should land)
```

**Key constraint:** The ball must be DESCENDING when it crosses the lip, at a specific angle (we use 60°).

This means the trajectory must pass through two points:
1. Hub edge - at lip height, descending at 60°
2. Hub center - where it lands

**With two points defined, there's exactly ONE trajectory!**

### The Math (20 min)

Projectile motion:
```
y = x·tan(θ) - g·x² / (2·v²·cos²(θ))
```

Let K = g/(2·v²·cos²θ), simplifying to:
```
y = x·tan(θ) - K·x²
```

With two points (x₁,y₁) and (x₂,y₂):
```
y₁ = x₁·tan(θ) - K·x₁²
y₂ = x₂·tan(θ) - K·x₂²
```

**Your task:** Solve for tan(θ) and K.

<details>
<summary>Solution</summary>

Subtract the equations:
```
y₁ - y₂ = (x₁ - x₂)·tan(θ) - K·(x₁² - x₂²)
```

Rearranging both equations and solving:
```
tan(θ) = (y₁·x₂² - y₂·x₁²) / (x₁·x₂·(x₂ - x₁))
K = (x₁·tan(θ) - y₁) / x₁²
v² = g / (2·K·cos²(θ))
```
</details>

### Code Review (15 min)

Open `src/main/java/frc/robot/subsystems/hood/TrajectoryOptimizer.java`

Find:
1. Where is the descent angle used?
2. How are the two points calculated?
3. What validation happens after the math?

---

## Section 2: Motor Tuning with SysId (45 min)

### The Challenge

You command 2500 RPM. How do you get there quickly and accurately?

### Feedforward vs Feedback

**Feedback only:**
- Measure current speed
- Increase voltage if too slow
- Decrease if too fast
- Problem: Always playing catch-up

**Feedforward + Feedback:**
- Calculate: "2500 RPM needs ~8.5V"
- Apply that voltage immediately
- Use feedback for fine corrections
- Much faster response!

### The Feedforward Equation

```
voltage = kS·sign(v) + kV·v + kA·a
```

Where:
- **kS:** Voltage to overcome static friction
- **kV:** Volts per unit velocity (steady-state)
- **kA:** Volts per unit acceleration (often ignored for flywheels)

### Measuring kS and kV

**Quasistatic test:**
1. Slowly ramp voltage (0.1 V/sec)
2. Record voltage and velocity at each timestep
3. Plot velocity (x) vs voltage (y)
4. Linear regression: slope = kV, y-intercept = kS

**Your task:** Look at the characterization code in the launcher subsystem. How does it implement safety limits during the test?

### Hands-On

If you have time, run the characterization:
1. Start simulation
2. Trigger the characterization command
3. Export data to a spreadsheet
4. Calculate kS and kV
5. Compare to the values in the code

---

## Section 3: The Simulation (45 min)

### The Challenge

We want to test shooting without a real robot. How do we simulate ball physics realistically?

### Key Decisions

**What to simulate:**
- Gravity (essential)
- Field boundary collisions (essential)
- Ball-robot collisions (important)
- Ball-ball collisions (expensive, optional)

**What to skip:**
- Air resistance (minimal effect)
- Ball spin/magnus (complex, small effect)

### Performance Optimization

With 400+ balls, naive collision detection is O(n²) = 160,000 checks per frame!

**Solution: Spatial grid**
- Divide field into 0.25m cells
- Each ball lives in one cell
- Only check collisions with neighboring cells
- Reduces to O(n) in practice

### Code Exploration

Open `src/main/java/frc/robot/util/FuelSim.java`

Find:
1. The `Fuel` class - how is ball state stored?
2. The `step()` method - how is physics applied?
3. The spatial grid - how is it implemented?
4. Collision response - how do bounces work?

**Question:** What is coefficient of restitution and how is it used?

---

## Section 4: Command Composition (45 min)

### The Challenge

Shooting requires coordination:
- Turret must be aimed (within 2°)
- Launcher must be at speed (within 50 RPM)
- Hub must be active (game rules)
- Ball must be available

How do we compose these conditions?

### The State Machine Approach

```
IDLE → SPINNING_UP → AIMING → READY → SHOOTING → (repeat or IDLE)
```

Each transition has conditions that must be met.

### Code Pattern

Look at `src/main/java/frc/robot/commands/ShootingCommands.java`

Notice:
- Conditions are checked with suppliers (dynamic values)
- Commands are composed with `alongWith`, `andThen`
- State is tracked for multi-shot sequences

### Moving Shots

When the robot is driving, we need to "lead" the target:
1. Calculate time of flight
2. Predict where target will appear to be (accounting for our motion)
3. Aim at the predicted position
4. Iterate for accuracy

**Find:** The `calculateMovingShot` method in `TurretCalculator.java`. How many iterations does it use? Why iterate at all?

---

## Section 5: Challenge Problems (30+ min)

Pick one or more to explore deeper:

### Challenge A: Distance-Dependent Efficiency

**Hypothesis:** Launch efficiency might vary with distance because ball compression changes with velocity.

**Task:**
1. Add a distance parameter to the efficiency calculation
2. Test if a linear model (efficiency = a + b·distance) improves accuracy
3. Measure in simulation at 3m, 5m, 7m distances

### Challenge B: Moving Target

**Scenario:** The hub is on a rotating platform (it's not, but imagine).

**Task:**
1. Add hub velocity to the targeting calculation
2. How does this change the lead calculation?
3. What's the maximum hub velocity we can track?

### Challenge C: Energy Optimization

**Problem:** Higher RPM uses more battery.

**Task:**
1. For a given distance, find the MINIMUM RPM that still scores
2. Add an "eco mode" that uses minimum energy
3. Compare battery usage over a simulated match

### Challenge D: Spin Effects

**Research:** The Magnus effect causes spinning balls to curve.

**Task:**
1. Add ball spin to FuelSim
2. Implement Magnus force: F = ½·ρ·A·Cl·v²
3. Does backspin help or hurt accuracy?

---

## Wrap-Up

### Key Takeaways

1. **Constrained optimization** - Adding constraints can make problems easier, not harder
2. **Feedforward control** - Predicting what you need beats reacting to errors
3. **Simulation tradeoffs** - Good enough beats perfect if it runs fast
4. **Command composition** - Complex behaviors from simple pieces

### Going Further

- Read the WPILib docs on feedforward control
- Explore the AdvantageKit logging system
- Look at how other teams solve similar problems (Chief Delphi)

### Your Turn

Pick a feature and improve it. Ideas:
- Better visualization of trajectories
- Automatic parameter tuning
- Shot success prediction
- Multi-ball trajectory planning

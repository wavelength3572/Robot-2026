# Instructor Guide: Trajectory Lessons

## Teaching Philosophy

This curriculum uses **guided discovery** - students think through problems before seeing solutions. The goal is to develop physics intuition and engineering thinking, not just copy code.

## Key Commits for Reference

Students start at commit `88b425c` (turretbot merge). The full solution evolved through these commits:

| Commit | What Was Added |
|--------|----------------|
| `1b566f0` | Launcher subsystem with dual NEO Vortex motors |
| `ebf84cd` | SysId characterization and feedforward support |
| `18639ee` | SysId velocity limiting and tuned feedforward gains |
| `1325b9e` | TrajectoryOptimizer for hybrid RPM+hood control |
| `31c97d4` | Improved turret aiming, control, and simulation |
| `953214e` | Updated launcher control and simulation |
| `bbb1d64` | Enhanced fuel simulation with physics and scoring |
| `faf620c` | ShootingCommands and match phase tracking |

Students can peek at these commits if stuck, but encourage them to try first!

## Module-by-Module Notes

### Module 1: Meet Your Turret

**Goals:**
- Familiarization with codebase
- Understanding IO pattern
- Comfort with simulation

**Common Issues:**
- AdvantageScope connection problems → Check simulation is actually running
- Can't find turret controls → Look in `ButtonsAndDashboardBindings.java`

**Discussion Points:**
- Why separate hardware IO from subsystem logic?
- What makes TurretBot useful for testing?

---

### Module 2: Building a Launcher

**Goals:**
- Create first real subsystem
- Understand flywheel physics
- Motor control basics

**Key Concepts to Emphasize:**
- Velocity vs position control
- Flywheel inertia means RPM changes aren't instant
- Efficiency factor: real balls don't exit at wheel surface speed

**Common Mistakes:**
- Forgetting to apply efficiency factor
- Not modeling spinup/spindown time in simulation
- Voltage vs velocity confusion

**Hints if Stuck:**
- Look at existing `TurretIOSim.java` for simulation patterns
- `LauncherIOSparkFlex.java` in the solution shows real hardware setup
- Efficiency around 0.70 is typical for compression-style launchers

**Solution Location:** `subsystems/launcher/` directory

---

### Module 3: Tuning with SysId

**Goals:**
- Understand feedforward control
- Run characterization routines
- Appreciate measurement vs guessing

**Key Concepts:**
- kS = voltage to overcome static friction
- kV = volts per unit velocity at steady state
- Plot velocity vs voltage to find the line

**Common Mistakes:**
- Not ramping slowly enough (need quasi-static)
- Forgetting safety limits during characterization
- Confusing kS sign (should be positive)

**Hints if Stuck:**
- Ramp rate around 0.1-0.25 V/s works well
- Stop characterization if velocity exceeds safe limit
- Real values: kS around 0.1-0.3V, kV around 0.018-0.025 V/(rad/s)

**Solution Location:** `feedforwardCharacterization()` in launcher commands

---

### Module 4: Physics of Flight

**Goals:**
- Apply projectile motion equations
- Understand constrained optimization
- Derive and implement two-point trajectory math

**This is the hardest module!** Take time with the math.

**Key Insight:**
The descent angle constraint is what makes the problem uniquely solvable. Without it, there are infinite trajectories.

**Derivation Steps:**
1. Write projectile equation: `y = x*tan(θ) - g*x²/(2v²cos²θ)`
2. Let `K = g/(2v²cos²θ)` to simplify: `y = x*tan(θ) - K*x²`
3. Two points give two equations, two unknowns (tan(θ) and K)
4. Solve for tan(θ) first: `tan(θ) = (y₁x₂² - y₂x₁²)/(x₁x₂(x₂-x₁))`
5. Back-substitute to find K, then v

**Common Mistakes:**
- Sign errors in the equations
- Confusing height relative to ground vs relative to turret
- Forgetting to check constraints (angle limits, RPM limits, peak height)

**Hints if Stuck:**
- Walk through with specific numbers first
- Example: x₁=4m, y₁=1.5m, x₂=5m, y₂=1.0m
- Check solution by plugging back into original equation

**Solution Location:** `TrajectoryOptimizer.java`

---

### Module 5: Simulating Reality

**Goals:**
- Understand physics simulation
- Appreciate performance vs accuracy tradeoffs
- Integrate shooting with simulation

**Key Concepts:**
- Euler integration (simple but works)
- Coefficient of restitution for bounces
- Spatial grid optimization for collision detection

**Performance Tips:**
- Reduce neutral zone balls for faster simulation
- 5 subticks per frame is good balance
- Spatial grid is O(n) vs naive O(n²)

**Common Issues:**
- Balls fall through floor → Check collision order, subtick count
- Balls don't bounce right → Check COR values
- Simulation too slow → Reduce ball count, check grid implementation

**Solution Location:** `FuelSim.java`

---

### Module 6: Putting It Together

**Goals:**
- Command composition
- State machine thinking
- Game strategy implementation

**Key Patterns:**
- Check conditions before acting
- Use suppliers for dynamic values
- Separate concerns (tracking vs deciding vs acting)

**Match Phase Logic (2026 game):**
```
0-20s: AUTO - both hubs active
20-30s: TRANSITION - both active
30-55s: SHIFT_1 - loser of auto scores
55-80s: SHIFT_2 - winner of auto scores
80-105s: SHIFT_3 - loser of auto scores
105-130s: SHIFT_4 - winner of auto scores
130-160s: END_GAME - both hubs active
```

**Common Mistakes:**
- Checking conditions in wrong order
- Not handling edge cases (what if hub becomes inactive mid-shot?)
- Forgetting to reset state between matches

**Solution Location:** `ShootingCommands.java`, `MatchPhaseTracker.java`

---

## Assessment Ideas

### Formative (During Lessons)
- Can student explain why the design choice was made?
- Can student predict what will happen before running simulation?
- Can student debug when something goes wrong?

### Summative (End of Lessons)
- Modify a parameter and predict/measure the effect
- Add a new feature (e.g., different ball size)
- Explain the system to a teammate

### Challenge Metrics
- Time to empty 10 balls into hub
- Accuracy from various distances
- Score rate while driving

---

## Common Questions

**Q: Why not use pre-built solutions like WPILib's feedforward classes?**
A: Learning by building helps students understand what's inside the black box. Once they understand, they can use library versions.

**Q: How accurate is the simulation?**
A: Good enough for development and algorithm testing. Real robot will need tuning for factors like:
- Actual ball compression
- Motor performance variations
- Field surface differences

**Q: What if students want to explore beyond the lessons?**
A: Encourage it! Ideas:
- Air resistance modeling
- Ball spin/magnus effect
- Vision-based target tracking
- Multiple robot coordination

---

## Setup for Multiple Students

If running as a workshop:

1. Each student forks the repo
2. Everyone starts from same baseline: `git checkout 88b425c`
3. Students create their own branches
4. Can share interesting solutions/optimizations

Or for pair programming:
- One student drives, one navigates
- Switch every 15-20 minutes
- Compare approaches at end

---

## Estimated Pacing

| Session | Duration | Content |
|---------|----------|---------|
| 1 | 2 hours | Modules 1-2 (setup + launcher) |
| 2 | 2 hours | Module 3 (SysId) + start Module 4 |
| 3 | 3 hours | Module 4 (trajectory math is hard!) |
| 4 | 2 hours | Module 5 (simulation) |
| 5 | 2 hours | Module 6 (commands) |
| 6 | 2 hours | Challenges + competition |

Total: ~13 hours over 6 sessions

Can be compressed for experienced students or expanded for beginners.

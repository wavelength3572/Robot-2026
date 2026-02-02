# Freshman Track: Intro to Robot Scoring

## Welcome!

In this hands-on session, you'll learn how our robot shoots balls into the hub. You'll see physics in action through simulation and tune parameters to improve scoring!

**Time:** ~2 hours
**What you'll do:** Explore, experiment, and compete!

---

## Setup

```bash
git checkout lessons/freshman-start
./gradlew simulateJava
```

This branch has the full scoring system working - you'll explore and tune it!

---

## Part 1: See It In Action (20 min)

### 1.1 Launch the Simulation

If you haven't already:
```bash
./gradlew simulateJava
```

Open **AdvantageScope** and connect. You should see:
- A robot on the field
- A turret that rotates
- Balls flying through the air!

### 1.2 Explore the Controls

Try these in the simulator:
- Move the robot around the field
- Watch the turret auto-aim at the hub
- See balls launch and (hopefully) score!

**Discussion:**
- What do you notice about how the turret moves?
- Do all shots go in? Why might some miss?

---

## Part 2: How Does Shooting Work? (20 min)

### The Three Key Parts

| Part | What It Does |
|------|--------------|
| **Turret** | Rotates to aim at the target |
| **Launcher** | Spinning wheels that throw the ball |
| **Hood** | Adjusts the launch angle (up/down) |

### Watch in AdvantageScope

Find these values and watch them change:
- `Turret/CurrentAngle` - Where is the turret pointing?
- `Launcher/VelocityRPM` - How fast are the wheels spinning?
- `Shooter/Trajectory/*` - The predicted ball path

**Activity:** Move the robot closer and farther from the hub. What changes?

<details>
<summary>What should you see?</summary>

- Farther away → wheels spin faster (higher RPM)
- Farther away → launch angle changes
- The trajectory arc gets longer
</details>

---

## Part 3: The Physics (Conceptual) (20 min)

### Why Does Distance Matter?

Think about throwing a ball at a basketball hoop:
- **Close up:** Gentle toss, high arc
- **Far away:** Throw hard, flatter arc

Our robot does the same thing automatically!

### The Magic Formula

The robot calculates:
1. **How far** is the target?
2. **How fast** should the ball go? (RPM)
3. **What angle** should we launch at?

All three must be right, or the ball misses!

### Quick Experiment

In AdvantageScope, find `Shooter/LaunchEfficiency` (under Tunable Numbers).

**Question:** What do you think "launch efficiency" means?

<details>
<summary>Answer</summary>

The wheels spin at, say, 3000 RPM. But the ball doesn't come out quite that fast - some energy is lost to friction and ball compression.

Efficiency of 0.70 means the ball exits at 70% of the wheel surface speed.
</details>

---

## Part 4: Tuning Challenge! (30 min)

Now the fun part - can you make it score better?

### Challenge 1: Efficiency Tuning

1. In AdvantageScope, find `Shooter/LaunchEfficiency`
2. Current value is 0.70
3. Try changing it to 0.60, 0.80, 0.90
4. **Observe:** What happens to the shots?

**Goal:** Which value scores the most consistently from 5 meters?

### Challenge 2: Descent Angle

1. Find `Trajectory/DescentAngleDeg`
2. Current value is 60 degrees
3. Try 45, 60, 75, 90

**Think about:** A steeper descent means the ball is coming down more vertically. Why might that help?

### Challenge 3: Speed Competition!

Work with a partner:
1. Position robot at the 5-meter mark
2. Time how long it takes to score 5 balls
3. Try different parameter combinations
4. **Fastest time wins!**

---

## Part 5: Watch the Code (20 min)

Let's peek at how this works in code.

### With Claude's Help

Ask Claude:
> "Show me how the robot decides what RPM to use for shooting"

Or:
> "Where does the trajectory get calculated?"

### Key Files to Explore

| File | What It Does |
|------|--------------|
| `Turret.java` | Aims at the target |
| `Launcher.java` | Controls wheel speed |
| `TrajectoryOptimizer.java` | Calculates the best shot |

**You don't need to understand all the code!** Just get a feel for how the pieces connect.

---

## Part 6: Wrap-Up Discussion (10 min)

**Questions to discuss:**

1. What surprised you about how the robot shoots?
2. If you were designing this, what would you do differently?
3. What other robots have you seen that shoot things? (Basketball, frisbees, etc.)

**Challenge for later:**
- Can you make the robot score while driving?
- What makes that harder than shooting while stopped?

---

## Bonus: Quick Code Change

If time permits, try this with Claude:

> "Can you help me add a log message that prints whenever a ball scores?"

This introduces you to:
- Finding the right file
- Making a small change
- Seeing your change work in simulation

---

## What's Next?

When you're ready to go deeper:
- **Senior Track** covers the real math and physics
- You'll derive the trajectory equations yourself
- You'll learn how we tune motors with science (SysId)

Great job today!

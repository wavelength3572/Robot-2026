# Getting Started: Trajectory Lessons

## Overview

This lesson series takes you from a basic turret test platform to a sophisticated scoring machine with physics-based trajectory optimization. You'll work with Claude Code as your coding partner.

## Prerequisites

- Basic Java knowledge
- Git basics
- WPILib installed (2026 or later)
- AdvantageScope installed
- Claude Code CLI installed

## Setup

### 1. Clone and Checkout the Starter Branch

If you're starting fresh, checkout the turretbot baseline:

```bash
git checkout 88b425c
git checkout -b my-trajectory-learning
```

This gives you the starting point: a turret that can aim, but no launcher or shooting!

### 2. Verify the Simulation Works

```bash
./gradlew simulateJava
```

You should see DriverStation appear. Open AdvantageScope and connect to the simulation.

### 3. Start Claude Code

```bash
claude
```

Tell Claude you're working through the trajectory lessons:
> "I'm working through the trajectory optimization lessons. I'm starting at the turretbot baseline and want to build up to a full scoring system. Let's start with Module 1."

## Lesson Structure

Each module has:
- **Think First**: Discussion questions before coding
- **Build**: Hands-on implementation
- **Checkpoint**: Things to verify before moving on

## Working with Claude

Claude will:
- Help you implement features
- Explain concepts when you're stuck
- Review your code and suggest improvements
- Run simulations and debug with you

You should:
- Try to think through problems before asking for solutions
- Ask "why" questions to deepen understanding
- Experiment with parameters and observe results
- Take notes on what you learn

## Progress Tracking

As you complete each module, commit your work:

```bash
git add -A
git commit -m "Complete Module X: [description]"
```

This lets you revisit your progress and see how the system evolved.

## Timeline Suggestion

| Module | Estimated Time |
|--------|---------------|
| 1: Meet Your Turret | 30-45 minutes |
| 2: Building a Launcher | 1-2 hours |
| 3: Tuning with SysId | 1-1.5 hours |
| 4: Physics of Flight | 2-3 hours |
| 5: Simulating Reality | 1-2 hours |
| 6: Putting It Together | 2-3 hours |

Total: 8-12 hours depending on depth of exploration

## Need Help?

If you're stuck:
1. Re-read the relevant section of LESSON_PLAN.md
2. Ask Claude specific questions about what's confusing
3. Check the reference implementation on the main branch
4. Look at the existing code for patterns to follow

## Key Files You'll Create/Modify

| Module | Files |
|--------|-------|
| 2 | `Launcher.java`, `LauncherIO.java`, `LauncherIOSim.java` |
| 3 | Updates to `Launcher.java` for characterization |
| 4 | `TrajectoryOptimizer.java`, updates to `TurretCalculator.java` |
| 5 | Updates to `FuelSim.java`, `RobotContainer.java` |
| 6 | `ShootingCommands.java`, `MatchPhaseTracker.java` |

Good luck and have fun!

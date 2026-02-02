# Instructor Guide

## Two Tracks

| Track | Audience | Time | Focus |
|-------|----------|------|-------|
| **Freshman** | New programmers | ~2 hours | Visual, hands-on, fun |
| **Senior** | Experienced programmers | ~3-4 hours | Math, physics, coding |

---

## Freshman Track Notes

### Goals
- Get excited about robotics
- See physics in action
- Learn by experimenting
- Light exposure to code

### Session Flow
| Part | Time | Activity |
|------|------|----------|
| 1 | 20 min | Launch sim, explore |
| 2 | 20 min | Understand the parts |
| 3 | 20 min | Conceptual physics |
| 4 | 30 min | **Tuning competition!** |
| 5 | 20 min | Code exploration |
| 6 | 10 min | Wrap-up discussion |

### Tips
- Let them break things! Wrong parameters = learning
- The competition keeps it engaging
- Pair students up for discussion parts
- Don't get stuck on code details - concepts matter more

### Common Questions
- **"Why does efficiency exist?"** - Ball compresses, energy lost
- **"Why 60° descent angle?"** - Steeper = less likely to bounce out
- **"Can we make it shoot farther?"** - Yes, but constraints limit it

---

## Senior Track Notes

### Goals
- Understand constrained optimization
- Apply projectile motion math
- Learn feedforward control concepts
- Build intuition for engineering tradeoffs

### Session Flow
| Section | Time | Topic |
|---------|------|-------|
| 1 | 45 min | Trajectory optimization math |
| 2 | 45 min | Motor tuning (SysId) |
| 3 | 45 min | Physics simulation |
| 4 | 45 min | Command composition |
| 5 | 30+ min | Challenge problems |

### Tips
- Let them derive the math themselves - struggle is learning
- Encourage exploring the actual code
- Challenge problems are optional - pick based on interest
- Students can work independently with Claude

### Key Insights to Emphasize
1. **Two-point constraint** makes infinite solutions become one
2. **Feedforward** predicts, feedback corrects
3. **Spatial grid** turns O(n²) into O(n)
4. **Command composition** = complex from simple

---

## Key Code Locations

| Concept | File |
|---------|------|
| Trajectory math | `subsystems/hood/TrajectoryOptimizer.java` |
| RPM ↔ velocity | `subsystems/turret/TurretCalculator.java` |
| Ball physics | `util/FuelSim.java` |
| Shooting logic | `commands/ShootingCommands.java` |
| Match phases | `util/MatchPhaseTracker.java` |

---

## Reference Commits

Students can peek at these if stuck:

| Commit | Feature |
|--------|---------|
| `88b425c` | Starting point (turretbot) |
| `1b566f0` | Launcher subsystem added |
| `1325b9e` | TrajectoryOptimizer added |
| `bbb1d64` | FuelSim enhanced |
| `faf620c` | ShootingCommands added |

---

## Troubleshooting

**Simulation won't start:**
- Check `./gradlew build` succeeds first
- Make sure correct JDK version (17+)

**AdvantageScope won't connect:**
- Simulation must be running first
- Try localhost:5800

**Values not updating:**
- Check robot is enabled in DriverStation
- Verify correct NT topic paths

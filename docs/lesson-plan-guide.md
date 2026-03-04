# Student Lesson Plans - Master Reference

## How to Create a New Lesson Plan

### Step 1: Create a Student Branch
```bash
# Find the commit to branch from (before the feature exists)
git log --oneline -20

# Create branch from that commit
git branch StudentFeatureName <commit-hash>
git checkout StudentFeatureName

# Create docs folder if needed
mkdir -p docs
```

### Step 2: Lesson Plan Template

Create `docs/<feature-name>-lesson.md` with this structure:

```markdown
# Teaching Plan: [Feature Name]

## Session Overview
[1-2 sentence description]
Total time: ~XX minutes

**Prerequisites:**
- [What students need before starting]

---

## Prompt 1: Discovery (X min)

### Student Prompt:
\```
[The prompt students will give to Claude]
\```

### Learning Goals:
- [What students should learn]

### Discussion Points:
- [Questions to ask after Claude responds]

---

## Prompt 2: [Next Step] (X min)
[Repeat pattern...]

---

## Bonus Challenges

### Challenge A: [Name]
[Description and prompt]

---

## Tips for Mentors
1. [Facilitation tips]
```

### Step 3: Commit and Push
```bash
git add docs/<feature-name>-lesson.md
git commit -m "Add <feature> lesson plan for student training"
git push -u origin StudentFeatureName
```

---

## Lesson Plan Ideas - Prioritized

### Ready Now
| Topic | Branch | Status |
|-------|--------|--------|
| Turret Enhancement | `StudentTurretEnhancement` | ✅ Complete |

### High Priority (Create Soon)
| Topic | Complexity | Time | Notes |
|-------|------------|------|-------|
| Turret Limit Handling | Medium | 45-60 min | Soft limits, robot rotation coordination |
| Autonomous Paths | Medium-High | 60-90 min | PathPlanner basics, 4 paths |
| Shooter Characterization | Medium | 45-60 min | PID tuning, distance tables |
| Hopper/Indexer State Machine | Medium | 45-60 min | Fuel counting, feed sequence |
| Hardware Bringup | Medium | 60+ min | Hands-on, less Claude Code |

### Medium Priority
| Topic | Complexity | Time | Notes |
|-------|------------|------|-------|
| Alliance Hub Lighting | Medium | 45-60 min | Fun, visual feedback |
| Vision/AprilTag | High | 90+ min | Blocked by 2026 layout |
| Driver Practice Mode | Low-Medium | 30-45 min | Telemetry, cycle times |
| Intake Subsystem | Medium | 45-60 min | Blocked by mech design |

### Lower Priority / Future
| Topic | Complexity | Notes |
|-------|------------|-------|
| Object/Robot Detection | High | ML pipelines, collision avoidance |
| Climb Code | Medium-High | Blocked by mech design |
| Match Replay Analysis | Medium | Post-match tools |
| Pit Display | Low-Medium | QoL for competition |
| Simulation Accuracy | Medium | Tune sim to match real |

---

## Detailed Topic Descriptions

### Turret Limit Handling
**Context:** Our turret likely won't have full 360° rotation capability. Current code assumes unlimited rotation.

**Code issues to address:**
- `TurretConstants`: MIN/MAX angles assume ±180°
- `TurretIOSim`: Uses `enableContinuousInput()` which assumes wraparound
- `calculateTurretAngle()`: Always picks shortest path, even if blocked by limits
- No soft limit enforcement in `setAngle()`

**Features to implement:**
- Soft limits to prevent exceeding physical range (e.g., ±135°)
- "Target reachable" detection method
- Robot rotation coordination when turret can't reach target
- Dashboard/LED indicator for turret state (in range, at limit, target unreachable)

**Prompts to develop:**
- "What happens if we command a turret angle outside its physical limits?"
- "Add soft limits to prevent the turret from exceeding ±135 degrees"
- "The turret uses continuous input wrapping - why is this a problem with limited rotation?"
- "Create a method `isTargetReachable(targetAngle)` that returns false if outside limits"
- "When the turret can't reach a target, how should we coordinate robot rotation to compensate?"

### Autonomous Paths
**4 paths to implement:**
1. **Path A:** Shoot preloads only (simplest backup)
2. **Path B:** Shoot preloads → Depot pickup → Shoot → Climb
3. **Path C:** Shoot preloads → Human player pickup → Climb
4. **Path D:** Shoot from trench → Traverse neutral (pass/pickup) → Shoot at other trench

**Prompts to develop:**
- "What commands do we need for autonomous?" (shoot, intake, drive, climb)
- "Create a PathPlanner path from starting position to depot"
- "Create a sequential command group for Path A"
- "Add a parallel command to run intake while driving"

### Alliance Hub Lighting
**Features:**
- Robot LEDs match alliance hub state
- Track shift timing for driver anticipation
- Dashboard widget showing hub state

**Prompts to develop:**
- "What information does the game provide about hub state?"
- "Create an LED subsystem that can display patterns"
- "Track the hub shift timer and change LED color"
- "Add a dashboard indicator showing current hub state"

### Shooter Characterization
**Features:**
- Shooter velocity PID tuning
- Distance-to-velocity lookup tables
- Hood angle adjustment (if applicable)

**Prompts to develop:**
- "Analyze the shooter subsystem and find what needs tuning"
- "Create a SysId routine for the shooter"
- "Build an interpolating tree map for distance to velocity"
- "Add logging to track shot accuracy vs distance"

### Hopper/Indexer State Machine
**Features:**
- Track fuel count in hopper
- Indexer sequencing for consistent feeding
- "Ready to shoot" state detection

**Prompts to develop:**
- "Design a state machine for the indexer"
- "Add beam break sensors to count fuel"
- "Create a 'ready to shoot' trigger"
- "Handle edge cases: jamming, partial loads"

### Hardware Bringup Checklist
**Less Claude Code, more hands-on:**
1. Verify CAN bus connections
2. Test each motor individually
3. Verify encoder readings
4. Test gyro/IMU
5. Verify limit switches/sensors
6. Drive test (teleop)
7. Run SysId characterization

---

## Teaching Tips

### Before the Session
- [ ] Create student branch from correct commit
- [ ] Add lesson plan doc to branch
- [ ] Test that starting code has the "bug" or missing feature
- [ ] Prepare AdvantageScope/dashboard for visualization

### During the Session
- Rotate "driver" role between students
- Pause after each prompt to discuss
- Let students catch Claude's mistakes
- Encourage follow-up questions

### Common Student Prompts When Stuck
- "Can you explain what this code does?"
- "The build failed, can you fix it?"
- "The behavior is wrong - it does X but should do Y"
- "Can you add logging so we can debug this?"

### After the Session
- Have students commit their work
- Consider merging to main branch (gives ownership)
- Discuss what they learned
- Preview next session's topic

---

## Branch Reference

| Branch | Purpose |
|--------|---------|
| `test-merged-2026` | Clean integration branch |
| `StudentTurretEnhancement` | Turret lesson (ready) |
| `DHSTurretEnhancements-reference` | Mentor's turret solution (reference only) |
| `Student<Feature>` | Future student lesson branches |

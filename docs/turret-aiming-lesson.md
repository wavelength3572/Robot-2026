# Teaching Plan: Guided Prompts for Turret Aiming Feature

## Session Overview

Students will guide Claude Code through building alliance-aware turret aiming with zone detection. Total time: ~45-60 minutes.

**Prerequisites:**
- Make sure you're on the `StudentTurretEnhancement` branch
- The turret currently aims at placeholder coordinates that are the same for both alliances (this is the bug we'll fix!)

---

## Prompt 1: Discovery (5-10 min)

### Student Prompt:
```
Look at how the turret auto-aiming works in Drive.java. What field coordinates
is it currently aiming at? Are there any problems with the current implementation
for a game where Blue and Red alliances have their own hubs on opposite sides
of the field?
```

### Learning Goals:
- Students learn to ask Claude to analyze existing code
- Claude should find that BLUE_HUB and RED_HUB have identical coordinates
- Discussion: Why is this a problem?

### Discussion Points:
- What does the code currently do? (Reads from Constants.FieldPositions)
- Why would both alliances aiming at the same spot be wrong?
- What are field coordinates? (Origin at Blue corner, +X toward Red)

---

## Prompt 2: Fix Hub Coordinates (10 min)

### Student Prompt:
```
In the 2026 Rebuilt game, each alliance has a HUB that is 158.6 inches (4.03 meters)
from their alliance wall, centered on the field width. The field is 16.54m long
and 8.23m wide.

Can you update Constants.FieldPositions with the correct coordinates for both
the Blue hub and Red hub?
```

### Learning Goals:
- Providing specific measurements helps Claude do the math
- Students see how to translate game manual specs into code

### Expected Result:
- Blue Hub: (4.03, 4.115)
- Red Hub: (12.51, 4.115)  ← This is the key fix!

### Discussion Points:
- How did we calculate Red hub X? (16.54 - 4.03 = 12.51)
- Why is Y the same for both? (Both centered on field width)

---

## Prompt 3: Test and Verify (5 min)

### Student Prompt:
```
Run the simulation and let me test. I want to verify the turret aims at the
correct hub based on alliance color.
```

### Learning Goals:
- Importance of testing changes
- Using simulation to verify behavior

### Activity:
- Run sim as Blue alliance - turret should aim at (4.03, 4.115)
- Run sim as Red alliance - turret should aim at (12.51, 4.115)
- Check Turret/TargetX and Turret/TargetY in AdvantageScope

---

## Prompt 4: Introduce Zone Concept (10 min)

### Student Prompt:
```
I want to add a new feature. When the robot is in our alliance zone (close to
our hub), it should aim at the hub to shoot. But when we're in the neutral zone
or opponent's zone, we should aim to PASS back to teammates in our alliance zone
instead of shooting.

The alliance zone extends 4.03m from each alliance wall. Can you help me design
how this logic would work?
```

### Learning Goals:
- Students learn to describe requirements before jumping to code
- Claude should propose zone boundaries and logic flow
- Introduces the concept of a helper class

### Discussion Points:
- What X values define "Blue's shooting zone"? (X < ~5.5m)
- What X values define "Red's shooting zone"? (X > ~11.0m)
- Where should we aim when passing?

---

## Prompt 5: Create the Helper Class (10 min)

### Student Prompt:
```
Create a TurretAimingHelper class in frc.robot.util that:
1. Has an enum for AimMode (SHOOT or PASS)
2. Has a method that takes robot position and alliance, and returns the target
   coordinates and whether we're shooting or passing
3. When shooting, aim at our hub
4. When passing, aim back toward our alliance zone - specifically at 1/3 of
   the way from the side wall to center (to avoid the hub's net structure)
```

### Learning Goals:
- Creating utility classes with clean separation
- Using enums and records in Java
- Students see how specific requirements lead to specific code

### Discussion Points:
- Why a separate helper class? (Cleaner, testable, reusable)
- What's a Java record? (Immutable data carrier)
- Why 1/3 from the wall? (Avoid the net behind the hub)

---

## Prompt 6: Wire It Up (5 min)

### Student Prompt:
```
Update Drive.java to use the TurretAimingHelper instead of the current hub
coordinate logic. Also add logging so we can see the AimMode, TargetX, and
TargetY in AdvantageScope.
```

### Learning Goals:
- Integrating new code into existing systems
- Importance of logging for debugging

---

## Prompt 7: Final Testing (10 min)

### Student Prompt:
```
Run the simulation so I can test all the scenarios.
```

### Test Cases:
| Alliance | Robot Position | Expected Mode | Expected Target |
|----------|---------------|---------------|-----------------|
| Blue | (2, 4) | SHOOT | (4.03, 4.115) |
| Blue | (8, 2) | PASS | (2.0, 1.37) |
| Blue | (8, 6) | PASS | (2.0, 6.86) |
| Red | (14, 4) | SHOOT | (12.51, 4.115) |
| Red | (8, 2) | PASS | (14.54, 1.37) |

---

## Bonus Challenges (if time permits)

### Challenge A: Hysteresis
```
Add hysteresis to prevent rapid switching between SHOOT and PASS modes when
the robot is right at the zone boundary.
```

**Hint:** Consider adding a small buffer zone (e.g., 0.5m) so the mode only
switches when you've moved clearly into the new zone, not when hovering at
the boundary.

### Challenge B: Zone-Specific Passing Strategy

**Discussion prompt for students:**
"Right now we pass the same way whether we're in the neutral zone or deep in
the opponent's alliance zone. Should we pass differently based on WHERE we are?"

Consider:
- From neutral zone: Pass to teammates near our hub (current behavior)
- From opponent's alliance zone: We can't shoot the full field length, so
  maybe pass to our alliance TRENCH instead? (shorter, safer pass)

```
Update the TurretAimingHelper to have different pass targets based on whether
the robot is in the neutral zone vs the opponent's alliance zone. When in the
opponent's zone, aim for our alliance trench area instead of deep in our zone.
```

### Challenge C: Operator Targeting Mode (Advanced)

**Concept:** Let the operator use a second InterLink controller to manually
aim at a specific field location for direct robot-to-robot passing.

Requirements:
1. Operator uses joystick to move a targeting reticle on a field visualization
2. The turret aims at the reticle location
3. (Advanced) Calculate the launch velocity needed to land fuel at that spot

```
Create an operator targeting system where:
1. Add a "manual targeting" mode that the operator can enable
2. Use the operator's joystick X/Y to control a target position on the field
3. Display the target position in AdvantageScope as a visual reticle
4. Have the turret aim at that operator-selected position
5. (Bonus) Calculate and display the required launch velocity based on distance
```

**Why this is valuable:**
- Real game scenario: Direct passes to a specific teammate
- Teaches operator interface design
- Introduces trajectory/physics calculations
- Shows how to visualize targeting in AdvantageScope

---

## Tips for Mentors

1. **Let students type the prompts** - even if they copy them, the act of entering them helps learning

2. **Pause after each prompt** - discuss what Claude did and why before moving on

3. **Encourage follow-up questions** - if Claude's response is unclear, have students ask for clarification

4. **Point out the patterns:**
   - Asking Claude to analyze before changing
   - Providing specific measurements/requirements
   - Testing incrementally
   - Creating helper classes for complex logic

5. **Common issues:**
   - If Claude makes math errors, have students catch and correct them
   - If code doesn't compile, prompt: "Can you fix the compilation error?"
   - If behavior is wrong, prompt: "The turret is aiming at X but should aim at Y, can you investigate?"

---

## Reference: 2026 Rebuilt Field Geometry

```
Field: 16.54m x 8.23m
Origin: Blue alliance corner (bottom-left from Blue driver perspective)
+X: Toward Red alliance wall
+Y: Left from Blue driver station

Blue Hub: 4.03m from Blue wall (X = 4.03)
Red Hub: 4.03m from Red wall (X = 16.54 - 4.03 = 12.51)
Both Hubs: Centered on field width (Y = 4.115)

Alliance Zone: 4.03m deep from each alliance wall
Neutral Zone: Center of field between alliance zones
```

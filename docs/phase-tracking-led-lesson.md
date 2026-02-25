# Teaching Plan: Match Phase Tracking & LED Indicators
## For 3 Freshmen with FTC + 2025 IndicatorLight Experience

## About This Lesson
You already know how to control LEDs from working with last year's `IndicatorLight.java`.
Now you're going to make LEDs that are **game-aware** — they'll automatically tell
the driver when it's safe to score based on the REBUILT phase-shift timing. Along the
way, you'll find a real bug in our match timer code and learn how the field talks
to the robot.

**Total time:** 2 sessions x 45-60 minutes

**Team of 3 — Rotating Roles:**
| Role | What You Do |
|------|-------------|
| **Driver** (types code) | You're at the keyboard. Type what the team decides. |
| **Navigator** (reads/guides) | You have the docs open. Read code aloud, spot errors. |
| **Tester** (runs/checks) | You run the code, check AdvantageScope output, verify behavior. |

Rotate roles every ~15 minutes so everyone gets each job.

---

## What You Already Know (from last session with 2025 LEDs)

Take a second and remember what you learned working with `IndicatorLight.java`:

| Concept | What you saw in 2025 code |
|---------|--------------------------|
| `AddressableLED` + `AddressableLEDBuffer` | The hardware interface — create a buffer, set colors, push to strip |
| Pre-built color buffers | `IndicatorLight` made a separate buffer for red, green, blue, etc. in the constructor |
| `setActiveBuffer()` | Copies a pre-built buffer to the live buffer |
| `periodic()` on a subsystem | The method that runs every 20ms (50x/sec) and decides what to display |
| Effects like `doRainbow()`, `dynamicBlink()` | Timer-based animations using `Timer.getFPGATimestamp()` |
| `publishLEDsToDashboardFlipped()` | Sending LED colors to AdvantageScope for testing without hardware |

**The new question for 2026:** Last year's LEDs showed things like coral status and
alignment. This year, the LEDs need to answer one critical question:
**"Can we score RIGHT NOW, or do we have to wait for our shift?"**

To answer that, the LEDs need to know what phase of the match we're in. That's
the `MatchPhaseTracker` — and it has a bug.

---

## Pre-Session Homework (10 min, do before meeting)

**All 3 students:**
1. Open the Rebuilt Match Timer on your phone: https://rbgk.github.io/frc-rebuilt-timer/
2. Run through a full match. Watch the phases change.
3. Write down your answers:
   - How long is auto? 20
   - How long is each shift? 25
   - How long is end game? 30
   - What happens between auto and the first shift? delay
   - How many seconds warning do you get before a shift change? 7

---

## Session 1: The Game Timer & Its Bug (~45 min)
**Goal:** Understand the match phase system, find the bug, understand the fix.

### Activity 1: The REBUILT Timeline (10 min — whiteboard, whole group)

**Mentor draws on the board:**
```
|--- AUTO ---|-- DELAY --|-- TRANS --|--- SHIFT 1 ---|--- SHIFT 2 ---|--- SHIFT 3 ---|--- SHIFT 4 ---|--- END GAME ---|
    20 sec      5 sec       5 sec        25 sec           25 sec           25 sec           25 sec           30 sec
              (disabled)  (both on)   (loser scores)  (winner scores)  (loser scores)  (winner scores)   (both on)
```

**Key questions:**
- "If we won auto, which shifts are ours?" (2 and 4)
- "If we lost auto, which shifts are ours?" (1 and 3)
- "What should the LEDs show during shift 1 if we WON auto?" (Inactive — not our turn!)
- "What happens if the code THINKS it's End Game during Auto?" (This is the bug)

### Activity 2: Bug Hunt (20 min — Navigator reads code aloud, group discusses)

**Navigator:** Open `src/main/java/frc/robot/util/MatchPhaseTracker.java`.

**The game: find two bugs. Here are your clues.**

**Clue 1 — Look at the enum (lines 38-57):**
Read the phase names aloud. These are like the states the `IndicatorLight` used
(remember how it had states like `LED_State` that controlled which effect played?).
Same idea — the match has states, and the code picks which one we're in.

**Clue 2 — Look at `getCurrentPhase()` (starts around line 126):**
```
Ask: "What does this return if NO phase matches?" → END_GAME (the default)
Ask: "When would no phase match?" → Keep looking...
```

**Clue 3 — Look at `getMatchTime()` (starts around line 184):**
```
Line 186: return Timer.getMatchTime() > 0 ? (15 - Timer.getMatchTime()) : 0;

Ask: "What number does this use for auto duration?"
Answer: 15

Ask: "But you just saw on the rebuilt timer — how long is REBUILT auto?"
Answer: 20 seconds!

Ask: "At the start of auto, Timer.getMatchTime() returns 20. What's 15 - 20?"
Answer: -5

Ask: "Is -5 between 0 and 20? Between 20 and 30? Between ANY phase boundaries?"
Answer: No! Nothing matches!

Ask: "So what does getCurrentPhase() return?"
Answer: END_GAME!
```

**Bug 1 found!** The first 5 seconds of every match, the robot thinks it's End Game.

**Clue 4 — Now look at lines 198-199 (the practice mode fallback):**
```java
return Timer.getFPGATimestamp() % 160;
```

**Exercise:** Each student picks a number (seconds since robot powered on):
- Student A: 347 seconds → 347 % 160 = 27 → TRANSITION phase
- Student B: 892 seconds → 892 % 160 = 92 → SHIFT_3 phase
- Student C: 131 seconds → 131 % 160 = 131 → END_GAME phase

**Bug 2 found!** Practice mode gives random phases depending on when the robot was turned on.

### Activity 3: The Fix (15 min — Navigator reads updated code, group discusses)

**Navigator:** The bug has already been fixed. Open the CURRENT `MatchPhaseTracker.java`.

**Key insight:** Instead of converting countdown → elapsed time (which caused the math
bug), the fix uses the teleop countdown timer DIRECTLY:

```
remaining > 130  →  Transition
105 to 130       →  Shift 1
 80 to 105       →  Shift 2
 55 to  80       →  Shift 3
 30 to  55       →  Shift 4
  0 to  30       →  End Game
```

**Read `getTeleopPhaseFromRemaining()` together.** It's just a chain of `if`s. Compare
to the old approach — which is easier to understand?

**Then the cool part — FMS game data:**

```
Remember in FTC, the field didn't send your robot any data?
In FRC, the field sends a SECRET MESSAGE after auto: either 'R' or 'B'.
This tells us which alliance's hub goes dark first — meaning that
alliance won auto.

So instead of a human frantically toggling a dashboard switch,
the code reads it automatically from DriverStation.getGameSpecificMessage().
```

**Read `getWeWonAuto()`** — show how it checks FMS data first, then falls back to
the dashboard. Ask: "Why keep the dashboard fallback?" (Practice matches don't have FMS!)

**And the practice mode fix:**
Instead of the random `% 160`, the new code starts a stopwatch when the robot
is first enabled and simulates real match timing. Just like the rebuilt timer app
on your phone, but running on the robot.

---

## Session 2: Build Game-Aware LEDs (~45-60 min)
**Goal:** Wire up LEDs that react to match phase, using what you already know from 2025.

### Activity 4: From 2025 to 2026 — What's Different? (10 min — group discussion)

**Mentor leads:**

"In 2025, `IndicatorLight.java` chose colors based on things like coral status
and alignment. It had methods like `red()`, `green()`, `doBlink()`, and the
`periodic()` method picked which effect to run.

In 2026, `MatchPhaseLEDs.java` does the same thing, but it picks effects based
on **match phase and hub status** instead of coral status."

**Navigator:** Open `src/main/java/frc/robot/subsystems/led/MatchPhaseLEDs.java`.

**Compare with what you remember from 2025:**

| 2025 `IndicatorLight` | 2026 `MatchPhaseLEDs` | Same or different? |
|----------------------|----------------------|-------------------|
| Pre-built `redBuffer`, `greenBuffer`, etc. | Pre-built `blueBuffer`, `redBuffer`, etc. | Same pattern! |
| `setActiveBuffer()` copies a buffer | `setBuffer()` copies a buffer | Same pattern! |
| `periodic()` checks state and picks effect | `periodic()` checks phase and picks effect | Same pattern! |
| `doRainbow()` for idle | `doRainbow()` for disabled | Same pattern! |
| `dynamicBlink()` with variable speed | `doShiftWarningBlink()` with variable speed | Same idea! New twist: speed depends on countdown |
| Coral suppliers for state | `MatchPhaseTracker` for state | Different source, same approach |

**Ask:** "What's the one big thing `doShiftWarningBlink()` does that 2025's
`dynamicBlink()` didn't?" (Answer: The blink speed changes based on how many
seconds until the shift — faster as it gets closer, like a countdown timer you can see.)

### Activity 5: Design Your Phase LED Table (10 min — paper + markers)

Each student gets paper. Using what you know from 2025 effects, design what the
LEDs should show. You know the building blocks — now pick which ones to use:

| Match State | Your LED Design | Which 2025 effect is it closest to? |
|-------------|----------------|-------------------------------------|
| Disabled (pits) | _____________ | `doRainbow()`? `doBlueOmbre()`? |
| Auto (both hubs on) | _____________ | Solid `green()`? `doParty()`? |
| Our Hub ACTIVE | _____________ | Solid alliance color? Progress bar? |
| Our Hub INACTIVE | _____________ | Off? Dim? `doBlink()` slow? |
| Shift Warning (7 sec) | _____________ | `dynamicBlink()` speeding up? |
| End Game (GO!) | _____________ | `doExplosionEffect()`? Fast pulse? |

**Present designs (1 min each), agree on one, write it on the board.**

### Activity 6: Wire It Into the Robot (15 min — Driver codes)

You know this pattern from 2025 — adding a subsystem to the robot. But now
you're doing it yourselves instead of just reading it.

**Step 1: Add LED config to RobotConfig.java**

Open `src/main/java/frc/robot/RobotConfig.java`. Scroll to the bottom. Add:

```java
// ========== LED Configuration ==========
/** Whether this robot has addressable LEDs. */
default boolean hasLEDs() {
    return false;
}

/** PWM port for the addressable LED strip. */
default int getLedPwmPort() {
    return 0;
}

/** Number of LEDs in the strip. */
default int getLedCount() {
    return 0;
}
```

**Step 2: Override in your robot's config file**

Find SquareBotConfig.java or MainBotConfig.java. Add:

```java
@Override
public boolean hasLEDs() { return true; }

@Override
public int getLedPwmPort() { return 9; } // ask mentor for actual port

@Override
public int getLedCount() { return 30; } // count your actual LEDs
```

**Step 3: Create it in RobotContainer.java**

Remember how 2025 had `new IndicatorLight()` somewhere in RobotContainer?
Same idea. Add the field near the other subsystems:

```java
private final MatchPhaseLEDs matchPhaseLEDs;
```

And in the constructor (after other subsystems):

```java
if (Constants.getRobotConfig().hasLEDs()) {
    matchPhaseLEDs = new MatchPhaseLEDs(
        Constants.getRobotConfig().getLedPwmPort(),
        Constants.getRobotConfig().getLedCount());
} else {
    matchPhaseLEDs = null;
}
```

Don't forget the import! `import frc.robot.subsystems.led.MatchPhaseLEDs;`

**Tester: Read back what was typed and check for typos.**

### Activity 7: Implement Your Design (10-15 min — all 3 collaborate)

Open `MatchPhaseLEDs.java` and modify the `periodic()` method to match your
team's design from the board.

**Starter ideas based on effects you already know from 2025:**

**If your design uses a progress bar** (filling/draining LEDs):
```java
// In the "hub active" section of periodic():
double fraction = tracker.getTimeRemainingInPhase() / 25.0; // 25-second shifts
int litLEDs = (int)(fraction * ledCount);
for (int i = 0; i < ledCount; i++) {
    if (i < litLEDs) {
        buffer.setLED(i, isBlue ? Color.kBlue : Color.kRed);
    } else {
        buffer.setLED(i, Color.kBlack);
    }
}
```

**If your design uses a "traffic light" approach:**
```java
// Split strip into zones, all same color
Color stateColor;
if (!hubActive) {
    stateColor = Color.kRed;        // Can't score
} else if (timeUntilShift <= 7.0) {
    stateColor = Color.kYellow;     // Shift coming soon
} else {
    stateColor = Color.kGreen;      // Go score!
}
for (int i = 0; i < ledCount; i++) {
    buffer.setLED(i, stateColor);
}
```

**If you want something fancier** — you saw `doExplosionEffect()` and
`doSearchlightSingleEffect()` in the 2025 code. Same techniques work here:
use `Timer.getFPGATimestamp()` to animate over time, use `Math.sin()` for
pulsing, use a moving index for chase effects.

### Activity 8: Test It (5-10 min — Tester leads)

```
Deploy and open AdvantageScope. Find these entries:
  - Match/MatchPhase — should show AUTO, TRANSITION, SHIFT_1, etc.
  - Match/TeleopRemaining — countdown from 135
  - Match/OurHubActive — true/false
  - LED/Phase and LED/HubActive — what the LEDs are reacting to

Run a simulated match. Verify:
  1. Phase never shows END_GAME during auto (bug is fixed!)
  2. LEDs change at the right times
  3. Shift warning blink speeds up as the shift approaches
  4. End game looks different from normal shifts
```

**Pro tip from 2025:** Remember `publishLEDsToDashboardFlipped()`? You can
add a similar method to see LED colors in AdvantageScope even without the
physical strip connected. Great for testing in sim.

### Wrap-Up (5 min)

- Each student: "What's one thing from this lesson that connected to what we
  learned with the 2025 LEDs?"
- "What effect from 2025 would you most want to bring into the 2026 code?"
- Commit your work!

---

## Bonus Challenges

### Challenge A: Port Your Favorite 2025 Effect
Pick one effect from `IndicatorLight.java` that wasn't used above (like
`doExplosionEffect()`, `doSearchlightSingleEffect()`, `doPokadot()`, or
`doSegmentParty()`). Adapt it for a match phase. For example:
- `doExplosionEffect()` when we score
- `doSearchlightSingleEffect()` during end game to create urgency

### Challenge B: Score Celebration Flash
Add a method `triggerScoreEffect()` that overrides normal phase display
with a white flash for 0.5 seconds. Remember how 2025 tracked `LED_State`
to handle interrupting effects? Same concept.

### Challenge C: "Ready to Shoot" Indicator
Add an overlay that shows launcher readiness on top of the phase colors:
- Not spinning → dim the phase color to 30%
- Spinning up → normal brightness
- At speed → add white sparkle on top (random LEDs flash white briefly)

### Challenge D: Match Dashboard Widget
Publish a string array of hex colors to NetworkTables, one per LED. This lets
AdvantageScope render your LED strip as a custom widget. Look at how
`publishLEDsToDashboardFlipped()` worked in 2025 for the approach.

---

## Tips for Mentors

### Before Session 1
- [ ] Have all 3 students run the Rebuilt Match Timer on their phones (full match)
- [ ] Make sure the codebase builds on the development laptop
- [ ] Have the 2025 IndicatorLight.java open as a reference (they'll want to look back)
- [ ] Have colored markers and paper ready for the LED design activity
- [ ] Pre-load AdvantageScope with a log file showing the old bug (END_GAME during auto) if available

### Leveraging Their 2025 Experience
- **Start from what they know.** When they see `doRainbow()`, ask "remember this from 2025?"
- **Use comparison, not explanation.** Instead of teaching AddressableLED from scratch, ask
  "How is this constructor different from IndicatorLight's?" Let them spot the similarities.
- **Let them be the experts.** They know LED effects. Let them propose which 2025 effects
  to reuse. The new part is the *game logic*, not the LED mechanics.
- **The pre-built buffer pattern is familiar.** They've seen `redBuffer`, `greenBuffer` etc.
  Don't re-teach this. Just confirm "same pattern as 2025" and move on.

### What's Actually New for Them
Focus teaching time on these concepts (the LED stuff they already get):
1. **Enums** — 2025 might have used simpler state tracking. Explain enums as "a variable
   that can only be one of a fixed set of values."
2. **Singleton** — `MatchPhaseTracker.getInstance()`. Explain: "There's only one match
   happening, so there's only one tracker. Everyone shares it."
3. **DriverStation API** — `getMatchTime()`, `getGameSpecificMessage()`, `isAutonomous()`.
   This is the big FTC→FRC difference. FTC fields don't send game data to robots.
4. **RobotConfig pattern** — How `hasLEDs()` with a default of `false` means robots
   without LEDs just skip it. FTC doesn't have this pattern of optional subsystems.

### Pacing for 2 Sessions
With their LED background, 2 sessions should be enough:
- **Session 1** is mostly conceptual (game rules, bug hunting, understanding the fix).
  They'll be engaged because they're finding real bugs, not reading a textbook.
- **Session 2** is mostly hands-on (wiring, coding, testing). They'll move fast here
  because the LED patterns are familiar territory.
- If Session 2 runs short, start on a bonus challenge.
- If Session 1 runs long (good discussions!), you can push Activity 3 (the fix) to
  the start of Session 2.

### After Each Session
- Have students commit with a message describing what they did
- Take a photo of their LED design on paper (engineering notebook)
- Ask: "What would you teach a teammate who wasn't here today?"

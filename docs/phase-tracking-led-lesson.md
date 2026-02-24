# Teaching Plan: Match Phase Tracking & LED Indicators
## Tailored for 3 Freshmen with FTC Experience

## About This Lesson
Students will learn how the 2026 REBUILT game's phase-shifting works, find and
understand a real bug in our match timer code, and build an LED system that tells
drivers when they can score. This is real competition code — their work will run
on the robot at events.

**Total time:** 3 sessions x 45-60 minutes (one per meeting, or combine if time allows)

**Team of 3 — Rotating Roles:**
| Role | What You Do |
|------|-------------|
| **Driver** (types code) | You're at the keyboard. Type what the team decides. |
| **Navigator** (reads/guides) | You have the docs open. Read code aloud, spot errors. |
| **Tester** (runs/checks) | You run the code, check AdvantageScope output, verify behavior. |

Rotate roles every ~15 minutes so everyone gets each job.

---

## Before You Start: FTC → FRC Translation Guide

You already know a lot from FTC! Here's how the concepts map:

| FTC Concept | FRC Equivalent | Notes |
|-------------|---------------|-------|
| `OpMode` with `init()` / `loop()` | `Robot.java` with `robotInit()` / `robotPeriodic()` | Same idea: init runs once, periodic runs every cycle |
| `gamepad1.a`, `gamepad1.left_stick_y` | `OperatorInterface` + button bindings | FRC separates the controller mapping into its own class |
| `telemetry.addData("key", value)` | `Logger.recordOutput("key", value)` | FRC logs to a file + live dashboard instead of phone screen |
| `DcMotor` / `Servo` | Subsystem classes (like `Turret`, `Hood`) | FRC wraps each mechanism in a "subsystem" with its own file |
| Autonomous / TeleOp OpModes | `autonomousInit()` / `teleopInit()` in Robot.java | FRC switches modes automatically via the Field Management System |
| No equivalent | `DriverStation.getMatchTime()` | FRC robots get a live countdown timer from the field |
| No equivalent | `DriverStation.getGameSpecificMessage()` | The field sends game-specific data (this year: who won auto) |
| REV Blinkin LED driver | `AddressableLED` + `AddressableLEDBuffer` | FRC controls each LED individually via code — way more flexible |

---

## Pre-Session Homework (10 min, do before meeting)

**All 3 students:**
1. Open the Rebuilt Match Timer on your phone: https://rbgk.github.io/frc-rebuilt-timer/
2. Run it once through a full match. Watch the phases change.
3. Answer these questions (write down your answers):
   - How long is auto? ____
   - How long is each shift? ____
   - How long is end game? ____
   - What happens between auto and the first shift? ____
   - How many seconds warning do you get before a shift change? ____

---

## Session 1: Understanding the Game Timer (~45 min)
**Goal:** Understand how match phases work and find a real bug in our code.

### Activity 1: The REBUILT Game Refresher (10 min — whole group)

**Mentor leads this discussion at a whiteboard / screen:**

Draw the match timeline on the board:
```
|--- AUTO ---|-- DELAY --|-- TRANSITION --|--- SHIFT 1 ---|--- SHIFT 2 ---|--- SHIFT 3 ---|--- SHIFT 4 ---|--- END GAME ---|
    20 sec      5 sec        5 sec            25 sec           25 sec           25 sec           25 sec           30 sec
              (disabled)  (both hubs on)   (loser scores)  (winner scores)  (loser scores)  (winner scores)   (both hubs on)
```

**Questions to ask:**
- "If we won auto, during which shifts can we score?" (Shift 2 and 4)
- "If we LOST auto, when can we score?" (Shift 1 and 3)
- "Why does this matter for our robot code?" (We need to know when to shoot!)
- "What happens if our code thinks it's End Game when it's actually Auto?" (Hint: this is the bug we're going to find)

### Activity 2: Read the Code Together (15 min — all 3, Navigator reads aloud)

**Navigator:** Open `src/main/java/frc/robot/util/MatchPhaseTracker.java` in the editor.

**Mentor guidance:** Don't read the whole file. Focus on these three spots:

**Spot 1 — The phases (lines 38-57).** Read them aloud.
```
Ask: "What's an enum?"
FTC analogy: It's like having a variable that can ONLY be certain values —
like if you had a GamePhase variable that could only be AUTO, TELEOP, or ENDGAME.
In FTC you might have used strings or ints for this. Enums are safer because
the compiler catches typos.
```

**Spot 2 — getCurrentPhase() (lines 126-137).** Read it aloud.
```
Ask: "What does this method do in plain English?"
Answer: "It checks what time it is in the match and returns which phase we're in."
Ask: "What does it return if NO phase matches?" (line 136 — END_GAME!)
```

**Spot 3 — getMatchTime() (lines 184-203).** This is where the bug lives.
```
Ask: "What number does line 186 use for auto duration?"
Answer: 15
Ask: "But how long is auto in REBUILT?"
Answer: 20 seconds!
Ask: "So what happens at the very start of the match when the timer says 20 seconds left?"
Let them work it out: 15 - 20 = -5. Negative five!
Ask: "And what phase has a start time of -5?" None of them!
Ask: "So what does getCurrentPhase() return?" END_GAME! (the default on line 136)
```

**Celebrate:** They just found a real bug that caused "END_GAME" to appear in our
match logs during auto. This is the kind of bug that costs points at competition.

### Activity 3: The Practice Mode Bug (10 min — Driver at keyboard)

**Mentor prompts the Driver:**

"Now look at line 198-199. When we're practicing without the field system,
what does the code do?"

```java
return Timer.getFPGATimestamp() % 160;
```

**Explain simply:** `Timer.getFPGATimestamp()` is like a stopwatch that starts when
the robot turns on. The `% 160` means "divide by 160 and take the remainder."

**Exercise:** Have each student pick a random number of seconds (like "the robot has been
on for 347 seconds" or "892 seconds"). Calculate `number % 160` and look up which
phase that falls in:
- 0-20 = AUTO, 20-30 = TRANSITION, 30-55 = SHIFT 1, etc.

**Ask:** "Is this a good way to track match phases during practice?"
**Answer:** No! It gives random phases depending on how long the robot has been on.

### Activity 4: Quick Recap (5 min)

Each student says one thing they learned. Write them on the board.

**Key takeaways to reinforce:**
- The code assumed 15-second auto, but REBUILT has 20-second auto
- When math gives a negative number, no phase matches, so it defaults to END_GAME
- Practice mode was cycling through random phases instead of simulating a real match

---

## Session 2: Understanding the Fix & Designing LEDs (~45 min)
**Goal:** See how the bug was fixed, learn about FMS game data, design LED behavior.

### Activity 5: The Better Approach (15 min — Navigator reads, group discusses)

**Navigator:** Open the UPDATED `MatchPhaseTracker.java` (the one with the fix already applied).

**Key concept to explain first:**
```
In FRC, the field sends your robot a countdown timer during teleop.
It starts at 135 and counts down to 0. The WPILib documentation says
to use THIS number directly to know what phase you're in:

  remaining > 130  →  Transition (both hubs on)
  105 to 130       →  Shift 1
   80 to 105       →  Shift 2
   55 to  80       →  Shift 3
   30 to  55       →  Shift 4
    0 to  30       →  End Game

This is WAY simpler than trying to convert countdown time to elapsed time!
```

**Read getTeleopPhaseFromRemaining() together** — it's just a chain of `if` statements.
Compare to the old approach. Ask: "Which is easier to understand?"

**Then explain FMS game data:**
```
The field also sends a secret message after auto: either 'R' or 'B'.
This tells us which alliance's hub goes dark first — which means
that alliance won auto! So we don't need a human to toggle a switch
on the dashboard.

In FTC, you didn't have anything like this. FRC fields are "smarter"
and send data to the robot.
```

**Read getWeWonAuto()** — show how it checks `getGameSpecificMessage()` first,
then falls back to the dashboard toggle.

### Activity 6: Design LED Patterns (15 min — whiteboard/paper, all 3 contribute)

**Give each student a piece of paper and colored markers/pencils.**

**Prompt:** "Design what the LEDs should show for each situation. Draw it out.
Remember: the driver is stressed, moving fast, and can barely glance at the robot.
Your signal needs to be OBVIOUS."

Each student designs their own version for these states:
| State | What's Happening |
|-------|-----------------|
| Disabled | Robot is off, sitting in the pits |
| Auto | Autonomous is running, both hubs active |
| Our Hub Active | We CAN score right now! |
| Our Hub Inactive | We CANNOT score, wait for our turn |
| Shift Change Coming (7 sec warning) | A shift is about to happen! |
| End Game | Last 30 seconds, both hubs active, GO GO GO |

**After 5 minutes:** Each student presents their design (1 minute each).

**Group discussion:**
- "Which design is clearest from across the field?"
- "Should 'active' and 'inactive' use totally different colors, or same color with a blink?"
- "What does the rebuilt timer app do?" (Flashes and vibrates 7 seconds before shifts)

**Agree on one design** the team will implement. Write it down clearly.

### Activity 7: Read the LED Scaffolding Code (15 min — Navigator reads)

**Navigator:** Open `src/main/java/frc/robot/subsystems/led/MatchPhaseLEDs.java`.

**Walk through these concepts (mentor helps explain):**

**1. "What is AddressableLED?"**
```
FTC analogy: You may have used a REV Blinkin to set LED colors.
That gives you ONE color for the whole strip. AddressableLED lets
you control EACH LED individually — like pixels on a screen.
You create a "buffer" (like a pixel array), set colors in it,
then push it to the hardware.
```

**2. "What is a SubsystemBase?"**
```
FTC analogy: In FTC, your OpMode has everything in one file.
In FRC, each mechanism gets its own class called a "subsystem."
The robot framework calls periodic() on every subsystem every 20ms
(50 times per second), just like FTC's loop() method.
```

**3. Walk through periodic():**
- Gets the current phase from MatchPhaseTracker
- Gets whether our hub is active
- Picks the right LED pattern based on phase + hub status
- Pushes colors to hardware

**4. Look at doShiftWarningBlink():**
```
Ask: "What does 'fraction' represent?"
Answer: How far we are into the warning window (1.0 = just started, 0.0 = shift is NOW)
Ask: "What does blinkPeriod control?"
Answer: How fast the LEDs flash — smaller number = faster flash
```

---

## Session 3: Hands-On Coding (~45-60 min)
**Goal:** Modify the LED code, wire it into the robot, test it.

### Activity 8: Wire LEDs into the Robot (20 min — Driver codes, Navigator guides, Tester reviews)

**Step-by-step instructions (Tester reads these aloud one at a time):**

**Step 1: Add LED configuration to RobotConfig.java**

Open `src/main/java/frc/robot/RobotConfig.java`. Scroll to the bottom (after the
intake section). Add these three methods:

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

**Ask before moving on:** "Why do these default to false/0?" (Because not every robot
has LEDs. Robots without them just return false for `hasLEDs()` and never create
the subsystem.)

**Step 2: Override in your robot's config**

Find your robot's config file (SquareBotConfig.java or MainBotConfig.java).
Add the overrides with your actual values:

```java
@Override
public boolean hasLEDs() {
    return true;
}

@Override
public int getLedPwmPort() {
    return 9;  // <-- ask mentor which PWM port
}

@Override
public int getLedCount() {
    return 30; // <-- count your actual LEDs
}
```

**Step 3: Create the subsystem in RobotContainer.java**

Open `RobotContainer.java`. Near the other subsystem declarations at the top
(around line 89), add:

```java
private final MatchPhaseLEDs matchPhaseLEDs;
```

Add the import at the top of the file:
```java
import frc.robot.subsystems.led.MatchPhaseLEDs;
```

In the constructor, after the other subsystems are created (around line 420,
after the FuelSim initialization), add:

```java
// Create LED subsystem if this robot has LEDs
if (Constants.getRobotConfig().hasLEDs()) {
    matchPhaseLEDs = new MatchPhaseLEDs(
        Constants.getRobotConfig().getLedPwmPort(),
        Constants.getRobotConfig().getLedCount());
} else {
    matchPhaseLEDs = null;
}
```

**Tester:** Read back what was typed. Does it look right? Any typos?

### Activity 9: Customize Your LED Design (15 min — all 3 collaborate)

Now implement the LED design your team agreed on in Session 2.

**Choose ONE of these to implement (pick based on your team's design):**

**Option A: Progress Bar**
During active shifts, fill LEDs from left to right showing how much scoring
time is left. When time runs out, the bar is empty.

*Hint:* In the `periodic()` method, during hub-active shifts, calculate:
```java
double fraction = tracker.getTimeRemainingInPhase() / 25.0; // shifts are 25 seconds
int litLEDs = (int)(fraction * ledCount);
for (int i = 0; i < ledCount; i++) {
    if (i < litLEDs) {
        buffer.setLED(i, isBlue ? Color.kBlue : Color.kRed);
    } else {
        buffer.setLED(i, Color.kBlack);
    }
}
```

**Option B: Traffic Light**
Split the strip into 3 zones. All three show the same color:
- Green = our hub is active, go score!
- Red = our hub is inactive, don't shoot
- Yellow = shift change coming in the next 7 seconds

**Option C: Your Custom Design**
Implement whatever your team designed on paper. Ask a mentor if you get stuck
on the API for a specific effect.

### Activity 10: Test with AdvantageScope (10 min — Tester leads)

**Mentor helps set up AdvantageScope if students haven't used it before.**

```
AdvantageScope is like FTC's telemetry, but WAY more powerful.
It records everything and lets you scroll back through time.

1. Deploy code to the robot (or run in simulation)
2. Open AdvantageScope and connect
3. Find these log entries:
   - Match/MatchPhase — should show AUTO, TRANSITION, SHIFT_1, etc.
   - Match/TeleopRemaining — countdown from 135
   - Match/OurHubActive — true/false
   - LED/Phase — what the LEDs think the phase is
   - LED/HubActive — what the LEDs think about hub status
4. Run a simulated match and watch the values change
5. Verify: Does it EVER say END_GAME during auto? (It shouldn't anymore!)
```

### Wrap-Up Discussion (5 min)

**Questions for the team:**
- "What was the most surprising thing you learned?"
- "How is FRC code different from FTC code?"
- "What would you add to the LEDs next?"

---

## Bonus Challenges (for students who finish early or want homework)

### Challenge A: Score Celebration Flash
Add a method `triggerScoreEffect()` that makes all LEDs flash white 3 times
quickly, then returns to normal. You'll need:
- A boolean `scoreEffectActive`
- A timestamp `scoreEffectStartTime`
- Logic in `periodic()` to run the flash for ~0.5 seconds then stop

### Challenge B: Pit Display Mode
When the robot is disabled and not in a match, show a slow rainbow animation
or your team number in LEDs. The scaffolding code already has `doRainbow()` —
wire it up so it only runs when disabled.

### Challenge C: "Am I Ready to Shoot?" Indicator
Connect to the `ShootingCoordinator` to show:
- Red = launcher not spinning
- Yellow = launcher spinning up
- Green = ready to fire!

### Challenge D: Dashboard Preview
Publish LED colors to NetworkTables so you can see them in AdvantageScope even
without physical LEDs on the robot. Look at how `Logger.recordOutput()` works
for arrays.

---

## Tips for Mentors Working with These Students

### Before Session 1
- [ ] Have all 3 students install the Rebuilt Match Timer on their phones and run through it
- [ ] Make sure the codebase builds on the development laptop
- [ ] Print out the "FTC → FRC Translation Guide" table above for each student
- [ ] Have colored markers and paper ready for the LED design activity
- [ ] Pre-load AdvantageScope with a log file showing the old bug (END_GAME during auto) if available

### Pacing Tips
- Freshmen need **more time than you think** to read unfamiliar code
- When a student says "I get it" — ask them to explain it back. They often don't fully get it.
- The whiteboard timeline drawing (Activity 1) is critical context. Don't skip it.
- If they're stuck on Activity 8, have them just get ONE LED to turn on first, then add complexity
- It's OK if they don't finish all activities. Understanding > completing.

### FTC-Specific Pitfalls to Watch For
- They may try to put everything in one file (FTC habit). Explain the subsystem pattern.
- They may not understand why `periodic()` doesn't need a while loop (FTC's `loop()` is called by the system, same here).
- They may be confused by `null` checks — FTC usually doesn't have optional subsystems.
- They may not know what a singleton is (`getInstance()`). Explain: "It's a single shared instance, like if there was only one gamepad and everyone accessed the same one."

### Key Concepts to Reinforce
1. **Real bugs cost real points** — the END_GAME-during-auto bug means the robot might not shoot when it should
2. **The field talks to your robot** — FMS game data is free information, use it
3. **Drivers can't read dashboards** — colors they can see from 50 feet away beat tiny text
4. **Test before competition** — the rebuilt match timer app lets you validate your timing at home
5. **FRC code is a team sport** — Driver/Navigator/Tester roles mirror real software development

### After Each Session
- Have students commit their work with a message describing what they did
- Take a photo of their LED design on paper (for the engineering notebook)
- Ask: "What would you teach a teammate who wasn't here today?"

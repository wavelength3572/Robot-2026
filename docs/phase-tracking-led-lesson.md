# Teaching Plan: Match Phase Tracking & LED Indicators

## Session Overview
Students will understand how the 2026 REBUILT game's phase-shifting works, investigate
timing bugs in the match phase tracker, integrate FMS game data, and build an LED
indicator system that gives drivers real-time visual feedback about hub activity.

Total time: ~90-120 minutes (can be split into 2 sessions)

**Prerequisites:**
- Basic Java / WPILib knowledge
- Understanding of the 2026 REBUILT game rules (shift phases, hub activity)
- Access to robot or simulation environment
- Read the [Rebuilt Match Timer](https://rbgk.github.io/frc-rebuilt-timer/) to understand timing
- Read the [WPILib 2026 Game Data docs](https://docs.wpilib.org/en/stable/docs/yearly-overview/2026-game-data.html)

**Reference Materials:**
- Rebuilt Match Timer app: https://rbgk.github.io/frc-rebuilt-timer/
- Chief Delphi discussion: https://www.chiefdelphi.com/t/rebuilt-match-timer/513356
- WPILib 2026 game data: https://docs.wpilib.org/en/stable/docs/yearly-overview/2026-game-data.html
- 2025 IndicatorLight (reference): https://github.com/wavelength3572/Robot-2025/blob/main/src/main/java/frc/robot/subsystems/LED/IndicatorLight.java

---

## Session 1: Understanding Phase Tracking (~45-60 min)

### Prompt 1: Discovery — Read the Timer Code (10 min)

#### Student Prompt:
```
Read the file src/main/java/frc/robot/util/MatchPhaseTracker.java and explain:
1. What match phases does it track?
2. How does it determine the current phase?
3. How does it know if our hub is active?
```

#### Learning Goals:
- Understand the MatchPhase enum and what each phase represents
- Understand how DriverStation.isAutonomous() / isTeleop() works
- Understand the relationship between Timer.getMatchTime() and elapsed match time

#### Discussion Points:
- "What does Timer.getMatchTime() actually return? Is it counting up or down?"
- "What's the difference between autonomous, teleop, and the gap between them?"
- "Why does the FMS disable the robot briefly between auto and teleop?"

---

### Prompt 2: Find the Bugs (15 min)

#### Student Prompt:
```
Look at the getMatchTime() method in MatchPhaseTracker.java. The 2026 game
has 20-second autonomous, but what value does line 186 use? What happens
when Timer.getMatchTime() returns 20 at the start of auto?

Also, what happens in practice mode (line 198-199) when we're not connected
to FMS? Try some example values of Timer.getFPGATimestamp() and see what
phase you'd get.
```

#### Learning Goals:
- Find the 15 vs 20 second auto duration bug
- Understand that negative elapsed time (-5) causes END_GAME to appear during auto
- Understand why `FPGA_timestamp % 160` gives random phases in practice mode

#### Discussion Points:
- "What did the log show during auto? Why was it saying END_GAME?"
- "What's the difference between an FMS match and a practice session from the code's perspective?"
- "The rebuilt match timer community app confirms auto is 20 seconds. Where else can you verify game timing? (Answer: game manual, WPILib docs)"

---

### Prompt 3: Learn from WPILib Docs (10 min)

#### Student Prompt:
```
Read the WPILib 2026 game data documentation. It defines match phases using
teleop REMAINING time:
- Transition: remaining > 130s
- Shift 1: 105-130s remaining
- Shift 2: 80-105s remaining
- Shift 3: 55-80s remaining
- Shift 4: 30-55s remaining
- End Game: 0-30s remaining

How is this different from our code's approach? Which approach is simpler
and less error-prone?
```

#### Learning Goals:
- Understand that using remaining time directly (from Timer.getMatchTime()) is simpler than converting to elapsed time
- See that the WPILib docs are the authoritative source for phase boundaries
- Understand the advantage of matching official documentation exactly

#### Discussion Points:
- "Why is converting countdown → elapsed time error-prone?" (must know exact auto duration, delay duration, etc.)
- "What does DriverStation.getGameSpecificMessage() return for 2026?"
- "When does the game data message arrive? (Answer: ~3 seconds after auto ends)"

---

### Prompt 4: Understand the Fix (15 min)

#### Student Prompt:
```
Read the updated MatchPhaseTracker.java. Compare the new getCurrentPhase()
and getTeleopRemainingTime() with the old getMatchTime() approach. What
changed and why? Specifically:

1. How does it determine the phase during teleop now?
2. How does it handle FMS game data for "who won auto"?
3. How does practice mode work without an FMS timer?
4. What is resetForNewMatch() and when is it called?
```

#### Learning Goals:
- See the direct-remaining-time approach vs elapsed-time conversion
- Understand FMS game data integration (`getGameSpecificMessage()`)
- Understand the practice timer fallback mechanism
- Understand why state needs to be reset between matches

#### Discussion Points:
- "What's the advantage of checking FMS game data over a manual dashboard toggle?"
- "Why do we still keep the dashboard tunable as a fallback?"
- "What happens if the FMS game data hasn't arrived yet and we're in Shift 1?"

---

## Session 2: Building the LED System (~45-60 min)

### Prompt 5: Design the LED Behavior (10 min)

#### Student Prompt:
```
We want LEDs that tell the driver at a glance:
- What phase of the match are we in?
- Can we score right now (is our hub active)?
- Is a shift change coming soon?

Using the match phases from MatchPhaseTracker, design a table that maps
each state to an LED behavior. Consider: solid colors, blinking, brightness,
and speed of blink.

Think about what the rebuilt timer app does with its 7-second audio/flash
warnings before each shift.
```

#### Learning Goals:
- Think about visual communication under stress (drivers can't read dashboards easily)
- Design a clear, unambiguous signal system
- Understand the rebuilt timer's approach to pre-shift warnings

#### Discussion Points:
- "Can a driver read text on a dashboard during a match? What about seeing a color?"
- "How does the rebuilt timer app warn about upcoming shifts? (7-second audio + flash)"
- "What's more useful: knowing the current phase name, or knowing 'you can score NOW'?"

---

### Prompt 6: Read the 2025 Reference Code (10 min)

#### Student Prompt:
```
Look at the 2025 IndicatorLight.java reference code (link in prerequisites).
It uses WPILib's AddressableLED and AddressableLEDBuffer. Note:
1. How it creates pre-built color buffers in the constructor
2. How periodic() updates the LEDs each cycle
3. How it implements effects like blinking, rainbow, and dynamic blink
4. How it uses Timer.getFPGATimestamp() for timing

Now look at our scaffolding in src/main/java/frc/robot/subsystems/led/MatchPhaseLEDs.java.
How is it structured similarly? What features could you add?
```

#### Learning Goals:
- Understand AddressableLED hardware interface
- See the pre-built buffer pattern for efficiency
- Understand timing-based LED effects

#### Discussion Points:
- "Why pre-build buffers instead of setting colors each cycle?" (Performance)
- "What PWM port will our LED strip use? How do we configure that?"
- "The 2025 code has 20 LEDs. How many will we have?"

---

### Prompt 7: Extend the LED System (20 min)

#### Student Prompt:
```
Add the following features to MatchPhaseLEDs.java:

1. A "progress bar" effect during active shifts: fill LEDs from left to right
   as time remaining in the phase decreases, so drivers can see how much
   scoring time is left without looking at the dashboard.

2. A "celebration" effect when we score (triggered externally). Flash white
   3 times quickly, then return to normal. You'll need a method like
   triggerScoreEffect() and state tracking.

3. Wire the LED subsystem into RobotContainer.java. Look at how other
   subsystems are instantiated and add the LED subsystem following the
   same pattern. You'll need to add hasLEDs() and getLedPort()/getLedCount()
   to RobotConfig.java.
```

#### Learning Goals:
- Implement a non-trivial LED effect using time-based math
- Design a triggered animation with state management
- Practice integrating a new subsystem into the robot framework

#### Discussion Points:
- "How do you calculate what fraction of LEDs to light for a progress bar?"
- "For the score effect, what happens if we score twice quickly? Does the animation restart?"
- "Why use the subsystem pattern (extends SubsystemBase) instead of just a utility class?"

---

### Prompt 8: Test and Iterate (10 min)

#### Student Prompt:
```
Deploy the code in simulation and observe the LED logging output in
AdvantageScope. Check:
1. Does the phase change correctly during a simulated match?
2. Does the LED pattern match what you expect for each phase?
3. Does the shift warning blink speed up as the shift approaches?
4. Test the score celebration effect by calling it manually.

If anything is wrong, add more logging to debug it.
```

#### Learning Goals:
- Test LED behavior through simulation logging
- Debug timing-based code using AdvantageScope
- Iterate on visual design based on observed behavior

---

## Bonus Challenges

### Challenge A: Dashboard LED Visualization
Publish LED colors to SmartDashboard/NetworkTables so you can see them in
AdvantageScope even without physical LEDs. Look at how the 2025 code's
`publishLEDsToDashboardFlipped()` method works.

### Challenge B: Pit Display Mode
When the robot is disabled in the pits (no match), show a slow rainbow or
team-number animation instead of match phase data.

### Challenge C: Shot Readiness Integration
Connect to ShootingCoordinator to show shot readiness on the LEDs:
- Red = not ready (launcher spinning up)
- Yellow = almost ready (launcher near setpoint)
- Green = ready to fire

### Challenge D: FMS Game Data Testing
Write a unit test that simulates `DriverStation.getGameSpecificMessage()`
returning different values ('R', 'B', empty) and verify that `getWeWonAuto()`
returns the correct result for both alliance colors.

---

## Tips for Mentors

### Before the Session
- [ ] Have students play with the [Rebuilt Match Timer](https://rbgk.github.io/frc-rebuilt-timer/) on their phones
- [ ] Review the [WPILib 2026 game data page](https://docs.wpilib.org/en/stable/docs/yearly-overview/2026-game-data.html)
- [ ] Set up AdvantageScope to show Match/* log entries
- [ ] If testing on real robot, have an addressable LED strip ready (WS2812B/NeoPixel)
- [ ] Have the 2025 IndicatorLight.java code open as reference

### During the Session
- Let students discover the auto timing bug themselves before showing the fix
- Use the rebuilt timer app running alongside simulation to verify timing
- Encourage students to think about driver experience (what info do they need?)
- The LED design prompt is intentionally open-ended — let teams disagree and iterate

### Key Concepts to Reinforce
1. **Match timing is critical** — getting the phase wrong means shooting at an inactive hub
2. **FMS is the source of truth** — always prefer FMS data over manual input
3. **Practice mode needs to be realistic** — drivers need to practice with correct timing
4. **Visual signals beat dashboard text** — under match stress, colors > numbers
5. **Test with the timer** — the rebuilt match timer app is a great validation tool

### After the Session
- Have students commit their work to the repository
- Run a practice match with the new LEDs (sim or real)
- Discuss: "What other information would be useful on the LEDs?"
- Preview: integration with shot readiness, intake status, climb ready

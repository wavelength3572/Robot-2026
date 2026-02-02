# Trajectory Optimization Lessons

Learn how our robot's scoring system works! Choose your track:

## Freshman Track (~2 hours)
**[FRESHMAN_TRACK.md](FRESHMAN_TRACK.md)**

A hands-on introduction focused on:
- Seeing the simulation in action
- Understanding the key components (turret, launcher, hood)
- Tuning parameters and competing for best scores
- Light code exploration with Claude

**Best for:** New programmers, visual learners, or anyone wanting a fun intro

---

## Senior Track (~3-4 hours)
**[SENIOR_TRACK.md](SENIOR_TRACK.md)**

A deep dive into the engineering:
- Deriving the trajectory optimization math
- Understanding feedforward motor control and SysId
- Exploring the physics simulation
- Challenge problems for further exploration

**Best for:** Experienced programmers ready for physics and calculus

---

## Quick Start

### Freshman Track
```bash
git checkout lessons/freshman-start
./gradlew simulateJava
```
Everything works! Explore and tune parameters.

### Senior Track
```bash
git checkout lessons/senior-start
./gradlew simulateJava
```
Starts at turretbot baseline - you'll build the scoring system!

---

**Then open AdvantageScope**, connect to the simulation, and start your track.

---

## For Instructors

See [INSTRUCTOR_GUIDE.md](INSTRUCTOR_GUIDE.md) for:
- Pacing suggestions
- Common questions and answers
- Key code locations for reference

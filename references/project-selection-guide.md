# Project Selection Guide

## Purpose

This guide helps select appropriate daily coding practice projects that:
1. Avoid duplicates
2. Build progressive skills
3. Balance difficulty appropriately
4. Maintain engagement

## Selection Process

### Step 1: Review Recent Projects

Check `PROJECT_INDEX.md` for the past 7 days:
- What technologies have been covered?
- Are there any repeated themes?
- What's the current skill level trajectory?

### Step 2: Identify Skill Gaps

Look for underrepresented areas:
- Graphics pipeline stages not yet explored
- Data structures not yet implemented
- Algorithms not yet practiced
- Real-world applications not yet built

### Step 3: Choose Appropriate Difficulty

**Too Easy** (Avoid):
- Repeating mastered concepts
- Trivial variations on past projects
- Projects completable in <15 minutes

**Just Right** (Target):
- Introduces 1-2 new concepts
- Builds on existing knowledge
- Requires 30-60 minutes
- Produces visible results

**Too Hard** (Defer):
- Requires prerequisites not yet learned
- Would take >2 hours
- Too many new concepts at once

### Step 4: Verify Uniqueness

Run the duplicate check:
```bash
./scripts/check_duplicate.sh "Proposed Project Name"
```

If it passes, proceed. If it fails, go back to Step 2.

## Category Guidelines

### 🔥 High Priority (Immediate)

These projects:
- Build directly on recent work
- Fill obvious gaps
- Have clear learning value
- Produce portfolio-worthy results

**Selection criteria**: Choose when you have momentum in an area.

### ⭐ Medium Priority (1-2 Weeks)

These projects:
- Require foundational work first
- Are more complex
- May need external resources
- Represent significant milestones

**Selection criteria**: Schedule when you've built sufficient foundation.

### 💡 Low Priority (1+ Months)

These projects:
- Are advanced topics
- Need substantial prerequisites
- May require extended time
- Represent long-term goals

**Selection criteria**: Keep as motivation, defer until ready.

### 🎮 Game-Related

These projects:
- Focus on practical game dev
- Combine multiple techniques
- May be larger in scope
- Have immediate application

**Selection criteria**: Mix into regular schedule for variety.

## Red Flags

Avoid projects that:

1. **Repeat core algorithms**
   - ❌ "Perlin Noise Terrain" after "Perlin Noise Textures"
   - ✅ "Simplex Noise" (different algorithm entirely)

2. **Lack clear learning goals**
   - ❌ "Another ray tracer with slightly different scene"
   - ✅ "Ray tracer with BVH acceleration" (new concept: BVH)

3. **Are too incremental**
   - ❌ "Change the ball colors in yesterday's renderer"
   - ✅ "Add reflection and refraction" (major new features)

4. **Have no visible output**
   - ❌ "Optimize noise function performance"
   - ✅ "Real-time noise visualization" (optimization + new UI)

## Green Flags

Prefer projects that:

1. **Build progressive skills**
   - Basic → Intermediate → Advanced
   - Example: Simple RT → RT with Shadows → RT with Reflections

2. **Cross-pollinate domains**
   - Graphics + Physics
   - Algorithms + Visualization
   - Theory + Practice

3. **Have immediate feedback**
   - Visual output
   - Interactive demos
   - Measurable results

4. **Create portfolio pieces**
   - Screenshots worth sharing
   - Code worth showing employers
   - Concepts worth blogging about

## Example Decision Trees

### Scenario: "I want to do more graphics"

```
Recent projects: Basic ray tracing, shadow ray tracing
Gap identified: No global illumination yet

Options:
1. Path tracing (Monte Carlo integration) ← HIGH PRIORITY
2. Photon mapping ← MEDIUM (complex, defer)
3. Screen-space techniques ← DIFFERENT DOMAIN

Choice: Path tracing
Reason: Natural progression, introduces new concept (MC sampling)
```

### Scenario: "Yesterday was intensive"

```
Recent projects: Complex path tracer (2 hours)
State: Mentally tired, need something lighter

Options:
1. Another complex renderer ← TOO SOON
2. Simple geometric algorithm ← GOOD PALETTE CLEANSER
3. Optimization task ← NO NEW LEARNING

Choice: Bresenham circle algorithm
Reason: Short, different domain, visible results, builds fundamentals
```

## Weekly Review

Every 7 days, check:

1. **Variety**: Am I covering diverse topics?
2. **Depth**: Am I building on foundations or just skimming?
3. **Gaps**: What major areas have I not touched?
4. **Momentum**: What's energizing vs. draining?

Adjust the upcoming week's selections based on this review.

## Common Pitfalls

### Pitfall 1: "I'll just make it slightly different"

**Symptom**: Projects that differ only in minor parameters
**Fix**: Require at least ONE major new concept per project

### Pitfall 2: "I'm comfortable here"

**Symptom**: Clustering around one topic too long
**Fix**: Force domain switches every 2-3 projects

### Pitfall 3: "This looks cool, let's try it"

**Symptom**: Jumping to advanced topics without foundation
**Fix**: Check prerequisites in priority list

### Pitfall 4: "I'll skip the boring stuff"

**Symptom**: Avoiding fundamental algorithms
**Fix**: Balance flashy projects with fundamental ones

## Success Metrics

A good project selection results in:

- ✅ Completed within target time (30-60 min)
- ✅ Learned at least one new concept
- ✅ Produced shareable artifact
- ✅ Feel motivated for next project
- ✅ No sense of repetition

If any of these fail, adjust selection criteria.

# NVIDIA Isaac ROS Visual SLAM as a FusionCore input

**Platform:** any robot running Isaac ROS Visual SLAM on Jetson Orin or Thor
**Status: Untested.** The plumbing is verified by reading both sides. Nobody has run
this combination end to end, and this page says so rather than implying otherwise. If
you run it, open an issue with what happened and this page gets your numbers and a
real badge.

---

## The short version

No code is needed on either side. FusionCore's VSLAM input subscribes to
`nav_msgs/Odometry`:

```cpp
// fusioncore_ros/src/fusion_node.cpp
vslam_sub_ = create_subscription<nav_msgs::msg::Odometry>(vslam_topic_, ...)
```

Isaac ROS Visual SLAM publishes `nav_msgs/Odometry`. So the integration is one line:

```yaml
vslam.topic: "/visual_slam/tracking/odometry"
```

Full config: [`isaac_ros_vslam.yaml`](../../fusioncore_ros/config/isaac_ros_vslam.yaml)

**Confirm the topic name against your own graph first.** It has changed between Isaac
ROS releases, and a wrong name fails silently: FusionCore subscribes, receives
nothing, and runs happily on the remaining sensors while you assume VSLAM is being
fused.

```bash
ros2 topic list | grep visual_slam
ros2 topic info <that topic>        # must be nav_msgs/msg/Odometry
```

---

## Why these are different layers, not competitors

This comes up often enough to be worth stating plainly.

Isaac ROS Visual SLAM answers: **based on what the cameras and IMU saw, how have I
moved?** That is a measurement. Like every measurement it has failure modes, and
NVIDIA documents them: blank walls, darkness, repetitive texture, dust, rain, and a
discontinuous pose jump whenever tracking is lost and the map re-initialises.

FusionCore answers a different question: **given GNSS, wheel odometry, IMU and visual
odometry, all imperfect and some of them wrong right now, what is the best estimate
of the robot's state, and which of these should I stop believing?**

```
   camera ──► Isaac ROS VSLAM ──┐
                                │
   IMU ─────────────────────────┼──► FusionCore ──► state + uncertainty
   wheel odometry ──────────────┤
   GNSS ────────────────────────┘
```

A robot with only VSLAM is blind the moment VSLAM is blind. A robot with only GNSS is
lost under canopy. The estimator's job is deciding what to trust, continuously, and
that job does not go away because one of the inputs got better.

---

## The parameter that matters most here

```yaml
vslam.reinit_n: 10
```

On tracking loss the VSLAM map re-initialises and the reported pose jumps
discontinuously. **That jump is not motion and must not be fused as motion.** After
this many consecutive gate rejections FusionCore re-anchors the VSLAM origin rather
than fighting it.

Lower it if your scene causes frequent relocalisation. Raise it if a genuinely noisy
patch is triggering re-anchors that were not needed. This is the parameter to reach
for first if fused output jumps when VSLAM recovers.

---

## Use the same IMU

Feed FusionCore the same IMU that Isaac ROS is using for its visual-inertial
odometry. A different IMU means the two disagree in ways neither can explain, and the
disagreement looks like a tuning problem rather than a wiring one.

---

## What would make this a certified config

Anyone with the hardware can produce this in an afternoon:

1. Run the two together and confirm `/fusion/odom` responds to motion.
2. Cover the camera or drive into a dark or featureless area, and record what the
   fused output does while VSLAM loses tracking and recovers. That is the case the
   pairing exists for.
3. If outdoors, do the same across a GNSS dropout.
4. Report the topic name and Isaac ROS version you used, because that is the part
   most likely to have drifted from this page.

Numbers, even rough ones, beat a feature list. Open an issue and this page gets
rewritten around them.

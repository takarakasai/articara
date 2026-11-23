# articara joints (Blender add-on)

Build a robot in Blender and open it in articara with working joints.

## Install

Blender 4.2 or newer: *Edit → Preferences → Add-ons → Install from Disk…*
→ `articara_joints.py`, then enable **articara joints**.

## Use

1. Model each rigid part as its own object and parent it to the part it
   moves against (*Ctrl+P → Object*). The object's current placement is
   the joint's zero position.
2. Select a child part, open the sidebar (**N**) → **articara** tab:
   - **Fixed / Revolute / Continuous / Prismatic** — the joint to the parent.
   - **X / Y / Z / −X / −Y / −Z** — the axis, in the object's local axes
     (or type any vector). The viewport draws it as an arrow
     (orange: revolute, yellow: continuous, blue: prismatic).
   - **Lower / Upper** — limits, in degrees for rotations and meters for
     slides. **Effort / Velocity** — 0 means undeclared.
   - **Add Joint name / Mass / Collision** — optional: a custom joint name,
     the link mass in kg (articara computes the inertia from the meshes),
     and whether to reuse the meshes as collision geometry.
3. **Export for articara (.glb)** — exports with custom properties
   included and lists any problems (a movable joint without a parent,
   lower > upper, …).
4. Open the `.glb` in articara (*Open…*, or
   `cargo run --example convert_robot -- robot.glb robot.misa`).

Keyframed motion of the joint objects (rotation for revolute / continuous,
location for prismatic) comes along too: each animation becomes a sequence
in articara, with one pose per exported keyframe.

The panel only edits plain custom properties, so the same keys also work
without the add-on (*Object Properties → Custom Properties*, exported with
*Include → Custom Properties*). The full list is in the docstring of
[`articara_joints.py`](articara_joints.py).

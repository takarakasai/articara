"""articara joints — author robot joints in Blender for articara's glTF import.

articara reads a node's glTF ``extras`` (Blender: the object's custom
properties, exported with *Include → Custom Properties*) to turn the node
into a movable joint. This add-on edits exactly those properties from a
sidebar panel (3D View → N → articara), draws each joint's axis in the
viewport, and exports the scene with the right glTF settings.

Properties (see ``NodeProps`` in articara's ``src/gltf_io.rs``):

    joint_type      "revolute" | "continuous" | "prismatic" | "fixed"
    joint_axis      [x, y, z] in the object's local axes
    joint_lower     lower limit (rad / m)
    joint_upper     upper limit (rad / m)
    joint_effort    max torque / force (0 = undeclared)
    joint_velocity  max speed (0 = undeclared)
    joint_name      joint name (default "<object>_joint")
    mass            link mass in kg (inertia computed from the meshes)
    collision       reuse the object's meshes as collision geometry

The joint connects the object to its parent object; the object's current
transform is the joint's zero position.

Install: Edit → Preferences → Add-ons → Install from Disk → this file.
"""

bl_info = {
    "name": "articara joints",
    "author": "articara",
    "version": (1, 0, 0),
    "blender": (4, 2, 0),
    "location": "3D View → Sidebar → articara",
    "description": "Author robot joints for articara's glTF import",
    "category": "Import-Export",
}

import math

import bpy
from mathutils import Vector

JOINT_TYPES = ("fixed", "revolute", "continuous", "prismatic")
MOVABLE = ("revolute", "continuous", "prismatic")
AXES = {
    "X": (1.0, 0.0, 0.0),
    "Y": (0.0, 1.0, 0.0),
    "Z": (0.0, 0.0, 1.0),
    "-X": (-1.0, 0.0, 0.0),
    "-Y": (0.0, -1.0, 0.0),
    "-Z": (0.0, 0.0, -1.0),
}
AXIS_COLORS = {"revolute": (1.0, 0.55, 0.1, 1.0), "continuous": (1.0, 0.85, 0.1, 1.0), "prismatic": (0.2, 0.7, 1.0, 1.0)}


def joint_type(obj):
    return str(obj.get("joint_type", "fixed")).lower()


def _ui(obj, key, **kwargs):
    """Set the UI hints (unit, range, …) of a custom property."""
    obj.id_properties_ui(key).update(**kwargs)


def _set_limit_units(obj):
    """Show limits in degrees for rotations and meters for slides."""
    unit = "DISTANCE" if joint_type(obj) == "prismatic" else "ANGLE"
    for key in ("joint_lower", "joint_upper"):
        if key in obj:
            _ui(obj, key, subtype=unit)


def joint_axis_world(obj):
    """Joint axis in world space (unit vector), or None."""
    axis = obj.get("joint_axis")
    if axis is None or len(axis) != 3:
        axis = (0.0, 0.0, 1.0)
    v = obj.matrix_world.to_3x3() @ Vector(axis)
    return v.normalized() if v.length > 1e-12 else None


class ARTICARA_OT_set_joint_type(bpy.types.Operator):
    """Make the selected objects joints of this type (relative to their parent)"""

    bl_idname = "articara.set_joint_type"
    bl_label = "Set Joint Type"
    bl_options = {"REGISTER", "UNDO"}

    joint_type: bpy.props.EnumProperty(items=[(t, t.capitalize(), "") for t in JOINT_TYPES])

    def execute(self, context):
        for obj in context.selected_objects or [context.object]:
            if self.joint_type == "fixed":
                # Fixed is the default: drop the joint keys entirely.
                for key in ("joint_type", "joint_axis", "joint_lower", "joint_upper", "joint_effort", "joint_velocity"):
                    if key in obj:
                        del obj[key]
                continue
            previous = joint_type(obj)
            obj["joint_type"] = self.joint_type
            if "joint_axis" not in obj:
                obj["joint_axis"] = [0.0, 0.0, 1.0]
                _ui(obj, "joint_axis", subtype="XYZ")
            if self.joint_type == "continuous":
                for key in ("joint_lower", "joint_upper"):
                    if key in obj:
                        del obj[key]
            elif previous != self.joint_type or "joint_lower" not in obj:
                # New default range whenever the unit changes.
                lo, hi = (0.0, 0.1) if self.joint_type == "prismatic" else (-math.pi, math.pi)
                obj["joint_lower"], obj["joint_upper"] = lo, hi
            for key in ("joint_effort", "joint_velocity"):
                if key not in obj:
                    obj[key] = 0.0
                    _ui(obj, key, min=0.0)
            _set_limit_units(obj)
        return {"FINISHED"}


class ARTICARA_OT_set_axis(bpy.types.Operator):
    """Set the joint axis of the selected objects (object-local)"""

    bl_idname = "articara.set_axis"
    bl_label = "Set Joint Axis"
    bl_options = {"REGISTER", "UNDO"}

    axis: bpy.props.EnumProperty(items=[(k, k, "") for k in AXES])

    def execute(self, context):
        for obj in context.selected_objects or [context.object]:
            if joint_type(obj) in MOVABLE:
                obj["joint_axis"] = list(AXES[self.axis])
                _ui(obj, "joint_axis", subtype="XYZ")
        return {"FINISHED"}


class ARTICARA_OT_toggle_prop(bpy.types.Operator):
    """Add or remove an optional link property on the selected objects"""

    bl_idname = "articara.toggle_prop"
    bl_label = "Toggle Property"
    bl_options = {"REGISTER", "UNDO"}

    key: bpy.props.EnumProperty(items=[("mass", "Mass", ""), ("collision", "Collision", ""), ("joint_name", "Joint name", "")])

    def execute(self, context):
        active = context.object
        add = self.key not in active
        for obj in context.selected_objects or [active]:
            if not add:
                if self.key in obj:
                    del obj[self.key]
            elif self.key == "mass":
                obj["mass"] = 1.0
                _ui(obj, "mass", min=0.0, soft_max=100.0, subtype="NONE", description="Link mass in kg")
            elif self.key == "collision":
                obj["collision"] = True
            else:
                obj["joint_name"] = f"{obj.name}_joint"
        return {"FINISHED"}


class ARTICARA_OT_export(bpy.types.Operator):
    """Export the scene as .glb with the settings articara needs (custom properties on)"""

    bl_idname = "articara.export_glb"
    bl_label = "Export for articara (.glb)"

    filepath: bpy.props.StringProperty(subtype="FILE_PATH")
    filter_glob: bpy.props.StringProperty(default="*.glb", options={"HIDDEN"})

    def invoke(self, context, event):
        if not self.filepath:
            stem = bpy.path.basename(bpy.data.filepath).rsplit(".", 1)[0] or "robot"
            self.filepath = stem + ".glb"
        context.window_manager.fileselect_add(self)
        return {"RUNNING_MODAL"}

    def execute(self, context):
        problems = validate(context.scene)
        for p in problems:
            self.report({"WARNING"}, p)
        bpy.ops.export_scene.gltf(filepath=bpy.path.ensure_ext(self.filepath, ".glb"), export_format="GLB", export_extras=True)
        self.report({"INFO"}, f"Exported {self.filepath}")
        return {"FINISHED"}


def validate(scene):
    """Problems articara would silently work around on import."""
    problems = []
    for obj in scene.objects:
        jt = joint_type(obj)
        if jt not in JOINT_TYPES:
            problems.append(f"{obj.name}: unknown joint_type '{jt}' (imported as fixed)")
        if jt in MOVABLE and obj.parent is None:
            problems.append(f"{obj.name}: {jt} joint has no parent object; it will move against the scene root")
        if jt in ("revolute", "prismatic") and obj.get("joint_lower", 0.0) > obj.get("joint_upper", 0.0):
            problems.append(f"{obj.name}: joint_lower > joint_upper")
        if jt in MOVABLE and joint_axis_world(obj) is None:
            problems.append(f"{obj.name}: zero-length joint_axis")
    return problems


class ARTICARA_PT_joint(bpy.types.Panel):
    bl_label = "articara Joint"
    bl_space_type = "VIEW_3D"
    bl_region_type = "UI"
    bl_category = "articara"

    def draw(self, context):
        layout = self.layout
        obj = context.object
        if obj is None:
            layout.label(text="Select an object")
            return
        jt = joint_type(obj)
        parent = obj.parent.name if obj.parent else "(scene root)"
        layout.label(text=f"{obj.name}  →  parent: {parent}")

        row = layout.row(align=True)
        for t in JOINT_TYPES:
            op = row.operator("articara.set_joint_type", text=t.capitalize(), depress=(jt == t))
            op.joint_type = t

        if jt in MOVABLE:
            col = layout.column(align=True)
            col.label(text="Axis (object-local)")
            row = col.row(align=True)
            current = tuple(round(v, 6) for v in obj.get("joint_axis", (0.0, 0.0, 1.0)))
            for name, vec in AXES.items():
                row.operator("articara.set_axis", text=name, depress=(current == vec)).axis = name
            col.prop(obj, '["joint_axis"]', text="")
            if jt != "continuous":
                col = layout.column(align=True)
                col.prop(obj, '["joint_lower"]', text="Lower")
                col.prop(obj, '["joint_upper"]', text="Upper")
            col = layout.column(align=True)
            col.prop(obj, '["joint_effort"]', text="Effort")
            col.prop(obj, '["joint_velocity"]', text="Velocity")

        layout.separator()
        for key, label in (("joint_name", "Joint name"), ("mass", "Mass (kg)"), ("collision", "Collision")):
            row = layout.row(align=True)
            if key in obj:
                row.prop(obj, f'["{key}"]', text=label)
                row.operator("articara.toggle_prop", text="", icon="X").key = key
            else:
                row.operator("articara.toggle_prop", text=f"Add {label}", icon="ADD").key = key

        layout.separator()
        layout.prop(context.scene, "articara_show_axes")
        for p in validate(context.scene):
            layout.label(text=p, icon="ERROR")
        layout.operator("articara.export_glb", icon="EXPORT")


# ─── Viewport axis overlay ──────────────────────────────────────────────

_draw_handle = None


def _draw_axes():
    if not getattr(bpy.context.scene, "articara_show_axes", False):
        return
    import gpu
    from gpu_extras.batch import batch_for_shader

    shader = gpu.shader.from_builtin("UNIFORM_COLOR")
    gpu.state.line_width_set(3.0)
    gpu.state.depth_test_set("NONE")
    for obj in bpy.context.visible_objects:
        jt = joint_type(obj)
        axis = joint_axis_world(obj) if jt in MOVABLE else None
        if axis is None:
            continue
        origin = obj.matrix_world.translation
        size = max(obj.dimensions.length * 0.5, 0.05)
        tip = origin + axis * size
        # Shaft plus a small arrow head.
        side = axis.orthogonal().normalized() * size * 0.12
        back = tip - axis * size * 0.2
        coords = [origin - axis * size * 0.3, tip, tip, back + side, tip, back - side]
        batch = batch_for_shader(shader, "LINES", {"pos": coords})
        shader.uniform_float("color", AXIS_COLORS[jt])
        batch.draw(shader)
    gpu.state.line_width_set(1.0)


classes = (
    ARTICARA_OT_set_joint_type,
    ARTICARA_OT_set_axis,
    ARTICARA_OT_toggle_prop,
    ARTICARA_OT_export,
    ARTICARA_PT_joint,
)


def register():
    global _draw_handle
    for cls in classes:
        bpy.utils.register_class(cls)
    bpy.types.Scene.articara_show_axes = bpy.props.BoolProperty(name="Show joint axes", default=True)
    if not bpy.app.background:
        _draw_handle = bpy.types.SpaceView3D.draw_handler_add(_draw_axes, (), "WINDOW", "POST_VIEW")


def unregister():
    global _draw_handle
    if _draw_handle is not None:
        bpy.types.SpaceView3D.draw_handler_remove(_draw_handle, "WINDOW")
        _draw_handle = None
    del bpy.types.Scene.articara_show_axes
    for cls in reversed(classes):
        bpy.utils.unregister_class(cls)


if __name__ == "__main__":
    register()

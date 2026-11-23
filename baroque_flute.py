from pathlib import Path

import cadquery as cq

class BaroqueFlute:
    class Body:
        bore_diameter = 19
        foot_bore_diameter = 14
        length_above_blow_hole = 70
        length_below_blow_hole = 520
        wall_thickness = 3

    class Embouchure:
        blow_hole_diameter = 8
        cork_distance = 19
        cork_thickness = 2

    class ToneHoles:
        body_split_after_hole = 3
        diameter = 7
        positions_from_blow_hole = (220, 260, 300, 340, 380, 420, 470)

    class Joints:
        class ORing:
            cross_section = 1.5
            groove_width = 2
            squeeze = 0.2

        head_joint_from_blow_hole = 130
        length = 20
        foot_joint_margin = 5
        thickness = 1.5

    class Key:
        class Hinge:
            radial_extent = 7.5
            width = 15

        class Pad:
            plug_length = 1.2
            radius = 6.5
            seat_depth = 2
            seat_height = 1
            seat_wall = 1

        class Spring:
            angle_deg = 45
            base_thickness = 1.5
            distance_from_pivot = 10
            length = 8
            tip_thickness = 0.5

        angle_deg = 6
        length = 40
        pivot_diameter = 1
        pivot_offset = 4
        profile_file = Path(__file__).with_name('key.dxf')
        thickness = 4
        width = 5

    class Clearances:
        hinge_slot = 0.2
        joint = 0.2
        key_pivot = 0.2
        pad_key_slot = 1
        pad_plug = 0.1

    def __init__(self):
        self.tube = self.build_tube()

    def z(self, position_from_blow_hole):
        return self.Body.length_above_blow_hole + position_from_blow_hole

    @property
    def length(self):
        return self.Body.length_above_blow_hole + self.Body.length_below_blow_hole

    @property
    def inner_radius(self):
        return self.Body.bore_diameter/2

    @property
    def foot_inner_radius(self):
        return self.Body.foot_bore_diameter/2

    @property
    def outer_radius(self):
        return self.inner_radius + self.Body.wall_thickness

    @property
    def socket_outer_radius(self):
        return self.outer_radius + self.Joints.thickness

    @property
    def hinge_outer_radius(self):
        return self.socket_outer_radius + self.Key.Hinge.radial_extent

    @property
    def key_pivot_radius(self):
        return self.socket_outer_radius + self.Key.pivot_offset

    @property
    def key_pivot_hole_radius(self):
        return (self.Key.pivot_diameter + self.Clearances.key_pivot) / 2

    @property
    def o_ring_groove_radius(self):
        o_ring = self.Joints.ORing
        return self.outer_radius + self.Clearances.joint - o_ring.cross_section * (1 - o_ring.squeeze)

    @property
    def o_ring_groove_positions(self):
        overlap_middle = self.Joints.length/4
        return (
            self.head_joint_z + overlap_middle,
            self.body_joint_z + overlap_middle,
            self.foot_joint_z - overlap_middle
        )

    @property
    def tone_hole_radius(self):
        return self.ToneHoles.diameter/2

    @property
    def head_joint_z(self):
        return self.z(self.Joints.head_joint_from_blow_hole)

    @property
    def body_joint_z(self):
        positions = self.ToneHoles.positions_from_blow_hole
        split = self.ToneHoles.body_split_after_hole
        return self.z((positions[split - 1] + positions[split]) / 2)

    @property
    def foot_joint_z(self):
        return self.keyed_hole_z - self.tone_hole_radius - self.Joints.foot_joint_margin - self.Joints.length/2

    @property
    def keyed_hole_z(self):
        return self.z(self.ToneHoles.positions_from_blow_hole[-1])

    def bore(self):
        z_axis = cq.Vector(0, 0, 1)

        head = cq.Solid.makeCylinder(self.inner_radius, self.head_joint_z, cq.Vector(0, 0, 0), z_axis)

        body = cq.Solid.makeCone(
            self.inner_radius, self.foot_inner_radius,
            self.foot_joint_z - self.head_joint_z,
            cq.Vector(0, 0, self.head_joint_z), z_axis
        )

        foot = cq.Solid.makeCylinder(
            self.foot_inner_radius, self.length - self.foot_joint_z,
            cq.Vector(0, 0, self.foot_joint_z), z_axis
        )

        return cq.Workplane('XY').add(head).union(body).union(foot)

    def build_tube(self):
        blow_hole_radius = self.Embouchure.blow_hole_diameter/2

        tube = (
            cq.Workplane('XY')
            .circle(self.outer_radius)
            .extrude(self.length)
            .cut(self.bore())
        )

        tube = tube.cut(
            cq.Workplane('YZ')
            .circle(blow_hole_radius)
            .extrude(self.outer_radius)
            .translate((0, 0, self.z(0)))
        )

        cork_top = self.z(0) - self.Embouchure.cork_distance

        cork = (
            cq.Workplane('XY')
            .circle(self.outer_radius)
            .extrude(self.Embouchure.cork_thickness)
            .translate((0, 0, cork_top - self.Embouchure.cork_thickness))
        )

        tube = tube.union(cork)

        for position in self.o_ring_groove_positions:
            tube = tube.cut(
                cq.Workplane('XY')
                .circle(self.outer_radius)
                .circle(self.o_ring_groove_radius)
                .extrude(self.Joints.ORing.groove_width)
                .translate((0, 0, position - self.Joints.ORing.groove_width/2))
            )

        for position in self.ToneHoles.positions_from_blow_hole:
            tube = tube.cut(
                cq.Workplane('YZ')
                .circle(self.tone_hole_radius)
                .extrude(self.outer_radius)
                .translate((0, 0, self.z(position)))
            )

        return tube

    def section(self, start, end=None):
        piece = self.tube

        if start > 0:
            piece = (
                piece.faces('<Z')
                .workplane(-start)
                .split(False, True)
            )

        if end is not None:
            piece = (
                piece.faces('<Z')
                .workplane(-(end - start))
                .split(True, False)
            )

        return piece

    def joint_at(self, z, joint):
        return joint.translate((0, 0, z - self.Joints.length/2))

    def socket(self, bonded_below):
        length = self.Joints.length
        socket_radius = self.outer_radius + self.Clearances.joint

        sleeve = (
            cq.Workplane('XY')
            .circle(self.socket_outer_radius)
            .extrude(length)
            .cut(
                cq.Workplane('XY')
                .circle(socket_radius)
                .extrude(length)
            )
        )

        bond = (
            cq.Workplane('XY')
            .circle(socket_radius)
            .circle(self.outer_radius)
            .extrude(length/2)
            .translate((0, 0, 0 if bonded_below else length/2))
        )

        return sleeve.union(bond)

    def head(self):
        return self.section(0, self.head_joint_z).union(self.joint_at(self.head_joint_z, self.socket(bonded_below=True)))

    def upper_body(self):
        return self.section(self.head_joint_z, self.body_joint_z).union(self.joint_at(self.body_joint_z, self.socket(bonded_below=True)))

    def lower_body(self):
        return self.section(self.body_joint_z, self.foot_joint_z)

    def foot(self):
        piece = self.section(self.foot_joint_z)

        piece = piece.union(
            cq.Workplane('YZ')
            .circle(self.tone_hole_radius)
            .circle(self.tone_hole_radius + self.Key.Pad.seat_wall)
            .extrude(self.Key.Pad.seat_depth + self.Key.Pad.seat_height)
            .translate((self.outer_radius - self.Key.Pad.seat_depth, 0, self.keyed_hole_z))
            .cut(
                cq.Workplane('XY')
                .circle(self.inner_radius)
                .extrude(self.length)
            )
        )

        joint_length = self.Joints.length

        hinge_box_center = (self.hinge_outer_radius / 2, 0, joint_length/2)

        hinge = (
            cq.Workplane('XY')
            .circle(self.hinge_outer_radius)
            .extrude(joint_length)
            .intersect(
                cq.Workplane('XY')
                .box(self.hinge_outer_radius, self.Key.Hinge.width, joint_length)
                .translate(hinge_box_center)
            )
            .cut(
                cq.Workplane('XY')
                .circle(self.outer_radius + self.Clearances.joint)
                .extrude(joint_length)
            )
        )

        hinge = (
            hinge.cut(
                cq.Workplane('XY')
                .box(self.hinge_outer_radius, self.Key.width + self.Clearances.hinge_slot, joint_length)
                .translate(hinge_box_center)
            )
            .cut(
                cq.Workplane('XZ')
                .cylinder(self.Key.Hinge.width, self.key_pivot_hole_radius)
                .translate((self.key_pivot_radius, 0, joint_length/2))
            )
        )

        return piece.union(self.joint_at(self.foot_joint_z, self.socket(bonded_below=False).union(hinge)))

    def to_key_pivot(self, shape):
        return (
            shape.rotate((0, 1, 0), (0, 0, 0), self.Key.angle_deg)
            .translate((self.key_pivot_radius, 0, self.foot_joint_z))
        )

    def key(self):
        key = self.Key

        pad = (
            cq.Workplane('YZ')
            .cylinder(key.thickness, key.Pad.radius)
            .union(
                cq.Workplane('YZ')
                .cylinder(key.Pad.plug_length * 2, self.tone_hole_radius - self.Clearances.pad_plug)
                .translate((-key.thickness/2, 0, 0))
            )
        )

        pad = pad.translate(
            (self.outer_radius + key.thickness/2 + key.Pad.seat_height, 0, self.keyed_hole_z)
        )

        spring = (
            cq.Workplane('XZ')
            .polyline([
                (-key.Spring.base_thickness/2, 0),
                (key.Spring.base_thickness/2, 0),
                (key.Spring.tip_thickness/2, key.Spring.length),
                (-key.Spring.tip_thickness/2, key.Spring.length)
            ])
            .close()
            .extrude(key.width)
            .translate((0, key.width/2, 0))
            .rotate((0, 1, 0), (0, 0, 0), key.Spring.angle_deg)
            .translate((0, 0, -key.Spring.distance_from_pivot))
        )

        shaft = (
            cq.importers.importDXF(key.profile_file)
            .wires()
            .toPending()
            .mirrorY()
            .extrude(key.thickness)
            .translate((0, key.length/2, -key.thickness/2))
            .rotate((0, 0, 0), (0, 0, 1), 90)
            .rotate((0, 0, 0), (0, 1, 0), 90)
            .cut(
                cq.Workplane('XZ')
                .cylinder(key.Hinge.width, self.key_pivot_hole_radius)
            )
            .cut(spring)
        )

        pad_key_slot = (
            cq.Workplane('YZ')
            .box(key.width, key.length + self.Clearances.pad_key_slot, key.thickness)
        )

        shaft = self.to_key_pivot(shaft)
        spring = self.to_key_pivot(spring)
        pad_key_slot = self.to_key_pivot(pad_key_slot)

        pad = pad.cut(pad_key_slot)

        return shaft, spring, pad

GREEN = 'hsl(120, 100%, 45%)'
YELLOW = 'hsl(55, 100%, 50%)'

flute = BaroqueFlute()

show_object(flute.head(), GREEN)
show_object(flute.upper_body(), GREEN)
show_object(flute.lower_body(), GREEN)
show_object(flute.foot(), GREEN)

show_object(flute.key(), YELLOW)

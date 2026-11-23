from pathlib import Path

import cadquery as cq

class BaroqueFlute:
    class Body:
        bore_diameter = 19
        foot_bore_diameter = 14
        length_above_blow_hole = 70
        length_below_blow_hole = 520
        wall_thickness = 1.5

    class Embouchure:
        blow_hole_diameter = 8
        cork_gap_to_blow_hole_edge = 4
        cork_thickness = 2

    class ToneHoles:
        body_split_after_hole = 3
        diameter = 7
        positions_from_blow_hole = (220, 260, 300, 340, 380, 420, 470)

    class Joints:
        head_joint_from_blow_hole = 130
        length = 20
        tail_joint_margin = 5
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
        joint = 0.05
        key_pivot = 0.2
        pad_key_slot = 1
        pad_plug = 0.1

    def __init__(self):
        self.flute = self.build_tube()

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
    def joint_outer_radius(self):
        return self.outer_radius + self.Joints.thickness

    @property
    def hinge_outer_radius(self):
        return self.joint_outer_radius + self.Key.Hinge.radial_extent

    @property
    def key_pivot_radius(self):
        return self.joint_outer_radius + self.Key.pivot_offset

    @property
    def key_pivot_hole_radius(self):
        return (self.Key.pivot_diameter + self.Clearances.key_pivot) / 2

    @property
    def tone_hole_radius(self):
        return self.ToneHoles.diameter/2

    @property
    def first_cut(self):
        return self.z(self.Joints.head_joint_from_blow_hole)

    @property
    def second_cut(self):
        positions = self.ToneHoles.positions_from_blow_hole
        split = self.ToneHoles.body_split_after_hole
        return self.z((positions[split - 1] + positions[split]) / 2)

    @property
    def tail_cut(self):
        return self.keyed_hole_z - self.tone_hole_radius - self.Joints.tail_joint_margin - self.Joints.length/2

    @property
    def keyed_hole_z(self):
        return self.z(self.ToneHoles.positions_from_blow_hole[-1])

    def bore(self):
        z_axis = cq.Vector(0, 0, 1)

        head = cq.Solid.makeCylinder(self.inner_radius, self.first_cut, cq.Vector(0, 0, 0), z_axis)

        body = cq.Solid.makeCone(
            self.inner_radius, self.foot_inner_radius,
            self.tail_cut - self.first_cut,
            cq.Vector(0, 0, self.first_cut), z_axis
        )

        foot = cq.Solid.makeCylinder(
            self.foot_inner_radius, self.length - self.tail_cut,
            cq.Vector(0, 0, self.tail_cut), z_axis
        )

        return cq.Workplane('XY').add(head).union(body).union(foot)

    def build_tube(self):
        blow_hole_radius = self.Embouchure.blow_hole_diameter/2

        flute = (
            cq.Workplane('XY')
            .circle(self.outer_radius)
            .extrude(self.length)
            .cut(self.bore())
        )

        flute = flute.cut(
            cq.Workplane('YZ')
            .circle(blow_hole_radius)
            .extrude(self.outer_radius)
            .translate((0, 0, self.z(0)))
        )

        cork_top = self.z(0) - blow_hole_radius - self.Embouchure.cork_gap_to_blow_hole_edge

        cork = (
            cq.Workplane('XY')
            .circle(self.outer_radius)
            .extrude(self.Embouchure.cork_thickness)
            .translate((0, 0, cork_top - self.Embouchure.cork_thickness))
        )

        flute = flute.union(cork)

        for position in self.ToneHoles.positions_from_blow_hole:
            flute = flute.cut(
                cq.Workplane('YZ')
                .circle(self.tone_hole_radius)
                .extrude(self.outer_radius)
                .translate((0, 0, self.z(position)))
            )

        return flute

    def section(self, start, end=None):
        piece = self.flute

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

    def joint_at(self, z, joint=None):
        if joint is None:
            joint = self.joint()

        return joint.translate((0, 0, z - self.Joints.length/2))

    def joint(self):
        joint = (
            cq.Workplane('XY')
            .circle(self.joint_outer_radius)
            .extrude(self.Joints.length)
            .cut(
                cq.Workplane('XY')
                .circle(self.outer_radius + self.Clearances.joint)
                .extrude(self.Joints.length)
            )
        )

        return joint

    def head_piece(self):
        return self.section(0, self.first_cut).union(self.joint_at(self.first_cut))

    def first_piece(self):
        return self.section(self.first_cut, self.second_cut).union(self.joint_at(self.second_cut))

    def second_piece(self):
        return self.section(self.second_cut, self.tail_cut)

    def tail_piece(self):
        piece = self.section(self.tail_cut)

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
                .circle(self.outer_radius)
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

        return piece.union(self.joint_at(self.tail_cut, self.joint().union(hinge)))

    def to_key_pivot(self, shape):
        return (
            shape.rotate((0, 1, 0), (0, 0, 0), self.Key.angle_deg)
            .translate((self.key_pivot_radius, 0, self.tail_cut))
        )

    def key(self):
        key = self.Key
        pad = self.Key.Pad

        spring_params = key.Spring

        stopper = (
            cq.Workplane('YZ')
            .cylinder(key.thickness, pad.radius)
            .union(
                cq.Workplane('YZ')
                .cylinder(pad.plug_length * 2, self.tone_hole_radius - self.Clearances.pad_plug)
                .translate((-key.thickness/2, 0, 0))
            )
        )

        stopper = stopper.translate(
            (self.outer_radius + key.thickness/2 + pad.seat_height, 0, self.keyed_hole_z)
        )

        spring = (
            cq.Workplane('XZ')
            .polyline([
                (-spring_params.base_thickness/2, 0),
                (spring_params.base_thickness/2, 0),
                (spring_params.tip_thickness/2, spring_params.length),
                (-spring_params.tip_thickness/2, spring_params.length)
            ])
            .close()
            .extrude(key.width)
            .translate((0, key.width/2, 0))
            .rotate((0, 1, 0), (0, 0, 0), spring_params.angle_deg)
            .translate((0, 0, -spring_params.distance_from_pivot))
        )

        shaft = (
            cq.importers.importDXF(self.Key.profile_file)
            .wires()
            .toPending()
            .mirrorY()
            .extrude(key.thickness)
            .translate((0, key.length/2, -key.thickness/2))
            .rotate((0, 0, 0), (0, 0, 1), 90)
            .rotate((0, 0, 0), (0, 1, 0), 90)
            .cut(
                cq.Workplane('XZ')
                .cylinder(self.Key.Hinge.width, self.key_pivot_hole_radius)
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

        stopper = stopper.cut(pad_key_slot)

        return shaft, spring, stopper

GREEN = 'hsl(120, 100%, 45%)'
YELLOW = 'hsl(55, 100%, 50%)'

flute = BaroqueFlute()

show_object(flute.head_piece(), GREEN)
show_object(flute.first_piece(), GREEN)
show_object(flute.second_piece(), GREEN)
show_object(flute.tail_piece(), GREEN)

show_object(flute.key(), YELLOW)

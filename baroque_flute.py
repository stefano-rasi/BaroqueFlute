import os
from dataclasses import dataclass

import cadquery as cq


@dataclass(frozen=True)
class Body:
    bore_diameter: float = 19
    wall_thickness: float = 1.5
    length_above_blow_hole: float = 70
    length_below_blow_hole: float = 520


@dataclass(frozen=True)
class Embouchure:
    blow_hole_diameter: float = 8
    cork_gap_to_blow_hole_edge: float = 4
    cork_thickness: float = 2


@dataclass(frozen=True)
class ToneHoles:
    positions_from_blow_hole: tuple = (220, 260, 300, 340, 380, 420, 470)
    diameter: float = 7
    body_split_after_hole: int = 3


@dataclass(frozen=True)
class Joints:
    length: float = 20
    thickness: float = 1.5
    head_joint_from_blow_hole: float = 130
    tail_joint_margin: float = 5


@dataclass(frozen=True)
class Hinge:
    width: float = 15
    radial_extent: float = 7.5


@dataclass(frozen=True)
class Key:
    angle_deg: float = 6
    width: float = 5
    thickness: float = 4
    length: float = 40
    pivot_offset: float = 4
    pivot_diameter: float = 1


@dataclass(frozen=True)
class Pad:
    radius: float = 6.5
    plug_length: float = 1.2
    seat_wall: float = 1
    seat_height: float = 1
    seat_depth: float = 2


@dataclass(frozen=True)
class Spring:
    angle_deg: float = 45
    length: float = 8
    distance_from_pivot: float = 10
    tip_thickness: float = 0.5
    base_thickness: float = 1.5


@dataclass(frozen=True)
class Clearances:
    joint: float = 0.05
    hinge_slot: float = 0.2
    key_pivot: float = 0.2
    pad_plug: float = 0.1
    pad_key_slot: float = 1


class BaroqueFlute:
    def __init__(
        self,
        key_file,
        body=Body(),
        embouchure=Embouchure(),
        tone_holes=ToneHoles(),
        joints=Joints(),
        hinge=Hinge(),
        key=Key(),
        pad=Pad(),
        spring=Spring(),
        clearances=Clearances()
    ):
        self.key_file = key_file

        self.body = body
        self.embouchure = embouchure
        self.tone_holes = tone_holes
        self.joints = joints
        self.hinge = hinge
        self.key_params = key
        self.pad = pad
        self.spring = spring
        self.clearances = clearances

        self.flute = self._build_tube()

    def _z(self, position_from_blow_hole):
        return self.body.length_above_blow_hole + position_from_blow_hole

    @property
    def length(self):
        return self.body.length_above_blow_hole + self.body.length_below_blow_hole

    @property
    def inner_radius(self):
        return self.body.bore_diameter/2

    @property
    def outer_radius(self):
        return self.inner_radius + self.body.wall_thickness

    @property
    def joint_outer_radius(self):
        return self.outer_radius + self.joints.thickness

    @property
    def hinge_outer_radius(self):
        return self.joint_outer_radius + self.hinge.radial_extent

    @property
    def key_pivot_radius(self):
        return self.joint_outer_radius + self.key_params.pivot_offset

    @property
    def key_pivot_hole_radius(self):
        return (self.key_params.pivot_diameter + self.clearances.key_pivot) / 2

    @property
    def tone_hole_radius(self):
        return self.tone_holes.diameter/2

    @property
    def first_cut(self):
        return self._z(self.joints.head_joint_from_blow_hole)

    @property
    def second_cut(self):
        positions = self.tone_holes.positions_from_blow_hole
        split = self.tone_holes.body_split_after_hole
        return self._z((positions[split - 1] + positions[split]) / 2)

    @property
    def tail_cut(self):
        return self.keyed_hole_z - self.tone_hole_radius - self.joints.tail_joint_margin - self.joints.length/2

    @property
    def keyed_hole_z(self):
        return self._z(self.tone_holes.positions_from_blow_hole[-1])

    def _build_tube(self):
        blow_hole_radius = self.embouchure.blow_hole_diameter/2

        flute = (
            cq.Workplane('XY')
            .circle(self.inner_radius)
            .circle(self.outer_radius)
            .extrude(self.length)
        )

        flute = flute.cut(
            cq.Workplane('YZ')
            .circle(blow_hole_radius)
            .extrude(self.outer_radius)
            .translate((0, 0, self._z(0)))
        )

        cork_top = self._z(0) - blow_hole_radius - self.embouchure.cork_gap_to_blow_hole_edge

        cork = (
            cq.Workplane('XY')
            .circle(self.outer_radius)
            .extrude(self.embouchure.cork_thickness)
            .translate((0, 0, cork_top - self.embouchure.cork_thickness))
        )

        flute = flute.union(cork)

        for position in self.tone_holes.positions_from_blow_hole:
            flute = flute.cut(
                cq.Workplane('YZ')
                .circle(self.tone_hole_radius)
                .extrude(self.outer_radius)
                .translate((0, 0, self._z(position)))
            )

        return flute

    def _section(self, start, end=None):
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

    def _joint_at(self, z, joint=None):
        if joint is None:
            joint = self.joint()

        return joint.translate((0, 0, z - self.joints.length/2))

    def joint(self):
        joint = (
            cq.Workplane('XY')
            .circle(self.joint_outer_radius)
            .extrude(self.joints.length)
            .cut(
                cq.Workplane('XY')
                .circle(self.outer_radius + self.clearances.joint)
                .extrude(self.joints.length)
            )
        )

        return joint

    def head_piece(self):
        return self._section(0, self.first_cut).union(self._joint_at(self.first_cut))

    def first_piece(self):
        return self._section(self.first_cut, self.second_cut).union(self._joint_at(self.second_cut))

    def second_piece(self):
        return self._section(self.second_cut, self.tail_cut)

    def tail_piece(self):
        piece = self._section(self.tail_cut)

        piece = piece.union(
            cq.Workplane('YZ')
            .circle(self.tone_hole_radius)
            .circle(self.tone_hole_radius + self.pad.seat_wall)
            .extrude(self.pad.seat_depth + self.pad.seat_height)
            .translate((self.outer_radius - self.pad.seat_depth, 0, self.keyed_hole_z))
            .cut(
                cq.Workplane('XY')
                .circle(self.inner_radius)
                .extrude(self.length)
            )
        )

        joint_length = self.joints.length
        hinge_box_center = (self.hinge_outer_radius / 2, 0, joint_length/2)

        hinge = (
            cq.Workplane('XY')
            .circle(self.hinge_outer_radius)
            .extrude(joint_length)
            .intersect(
                cq.Workplane('XY')
                .box(self.hinge_outer_radius, self.hinge.width, joint_length)
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
                .box(self.hinge_outer_radius, self.key_params.width + self.clearances.hinge_slot, joint_length)
                .translate(hinge_box_center)
            )
            .cut(
                cq.Workplane('XZ')
                .cylinder(self.hinge.width, self.key_pivot_hole_radius)
                .translate((self.key_pivot_radius, 0, joint_length/2))
            )
        )

        return piece.union(self._joint_at(self.tail_cut, self.joint().union(hinge)))

    def _to_key_pivot(self, shape):
        return (
            shape.rotate((0, 1, 0), (0, 0, 0), self.key_params.angle_deg)
            .translate((self.key_pivot_radius, 0, self.tail_cut))
        )

    def key(self):
        key = self.key_params
        pad = self.pad
        spring_params = self.spring

        stopper = (
            cq.Workplane('YZ')
            .cylinder(key.thickness, pad.radius)
            .union(
                cq.Workplane('YZ')
                .cylinder(pad.plug_length * 2, self.tone_hole_radius - self.clearances.pad_plug)
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
            cq.importers.importDXF(self.key_file)
            .wires()
            .toPending()
            .mirrorY()
            .extrude(key.thickness)
            .translate((0, key.length/2, -key.thickness/2))
            .rotate((0, 0, 0), (0, 0, 1), 90)
            .rotate((0, 0, 0), (0, 1, 0), 90)
            .cut(
                cq.Workplane('XZ')
                .cylinder(self.hinge.width, self.key_pivot_hole_radius)
            )
            .cut(spring)
        )

        pad_key_slot = (
            cq.Workplane('YZ')
            .box(key.width, key.length + self.clearances.pad_key_slot, key.thickness)
        )

        shaft = self._to_key_pivot(shaft)
        spring = self._to_key_pivot(spring)
        pad_key_slot = self._to_key_pivot(pad_key_slot)

        stopper = stopper.cut(pad_key_slot)

        return shaft, spring, stopper


KEY_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'key.dxf')

flute = BaroqueFlute(KEY_FILE)

GREEN = 'hsl(120, 100%, 45%)'
YELLOW = 'hsl(55, 100%, 50%)'

show_object(flute.head_piece(), GREEN)
show_object(flute.first_piece(), GREEN)
show_object(flute.second_piece(), GREEN)
show_object(flute.tail_piece(), GREEN)

show_object(flute.key(), YELLOW)

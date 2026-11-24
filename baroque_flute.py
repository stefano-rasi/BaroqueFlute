from math import radians, tan
from pathlib import Path
from functools import cached_property

import cadquery as cq

class BaroqueFlute:
    BORE_PROFILE = (
        (113.2, 17.2),
        (216, 15.5),
        (251.8, 14.9),
        (286.7, 14.4),
        (342.4, 13.8),
        (375.4, 13.2),
        (410.3, 12.5),
        (468.8, 11.4)
    )

    BORE_DIAMETER = 17.2

    WALL_THICKNESS = 3.2

    CORK_THICKNESS = 2
    CORK_DISTANCE = 17.2

    JOINT_LENGTH = 20
    JOINT_THICKNESS = 1.5

    PAD_RADIUS = 6.5
    PAD_SEAT_WALL = 1
    PAD_SEAT_DEPTH = 2
    PAD_SEAT_HEIGHT = 1
    PAD_PLUG_LENGTH = 1.2

    BLOW_HOLE_WIDTH = 7.5
    BLOW_HOLE_LENGTH = 8.5

    FOOT_JOINT_MARGIN = 5

    KEY_WIDTH = 5
    KEY_LENGTH = 40
    KEY_THICKNESS = 4
    KEY_PIVOT_OFFSET = 4
    KEY_PROFILE_FILE = Path(__file__).with_name('key.dxf')
    KEY_ANGLE_DEGREES = 6
    KEY_PIVOT_DIAMETER = 1
    KEY_ROTATION_DEGREES = 30

    HINGE_WIDTH = 15
    HINGE_RADIAL_EXTENT = 7.5

    O_RING_SQUEEZE = 0.2
    O_RING_GROOVE_WIDTH = 2
    O_RING_CROSS_SECTION = 1.5

    BODY_SPLIT_AFTER_HOLE = 3

    JOINT_CLEARANCE = 0.2
    PAD_PLUG_CLEARANCE = 0.1
    KEY_PIVOT_CLEARANCE = 0.2
    HINGE_SLOT_CLEARANCE = 0.2
    PAD_KEY_SLOT_CLEARANCE = 1

    LENGTH_ABOVE_BLOW_HOLE = 70
    LENGTH_BELOW_BLOW_HOLE = 509.3

    HEAD_JOINT_FROM_BLOW_HOLE = 130

    SPRING_LENGTH = 8
    SPRING_ANGLE_DEGREES = 45
    SPRING_TIP_THICKNESS = 0.5
    SPRING_BASE_THICKNESS = 1.5
    SPRING_DISTANCE_FROM_PIVOT = 10

    TONE_HOLE_DIAMETERS = (6.7, 6.5, 5.6, 5.7, 5.8, 4.7, 6.4)
    TONE_HOLE_POSITIONS = (221.9, 257.9, 294, 351, 381.9, 417.4, 474.7)
    TONE_HOLE_UNDERCUT_DEGREES = 7

    EMBOUCHURE_BAND_LENGTH = 30
    EMBOUCHURE_WALL_THICKNESS = 5
    EMBOUCHURE_UNDERCUT_DEGREES = 7

    def __init__(self):
        self.length = self.LENGTH_ABOVE_BLOW_HOLE + self.LENGTH_BELOW_BLOW_HOLE

        self.inner_radius = self.BORE_DIAMETER/2
        self.outer_radius = self.inner_radius + self.WALL_THICKNESS

        self.keyed_hole_radius = self.TONE_HOLE_DIAMETERS[-1]/2

        self.socket_radius = self.outer_radius + self.JOINT_CLEARANCE
        self.socket_outer_radius = self.outer_radius + self.JOINT_THICKNESS

        self.hinge_outer_radius = self.socket_outer_radius + self.HINGE_RADIAL_EXTENT

        self.key_pivot_radius = self.socket_outer_radius + self.KEY_PIVOT_OFFSET
        self.key_pivot_hole_radius = (self.KEY_PIVOT_DIAMETER + self.KEY_PIVOT_CLEARANCE) / 2

        self.o_ring_groove_radius = self.socket_radius - self.O_RING_CROSS_SECTION * (1 - self.O_RING_SQUEEZE)

        self.embouchure_outer_radius = self.inner_radius + self.EMBOUCHURE_WALL_THICKNESS

        split = self.BODY_SPLIT_AFTER_HOLE

        self.head_joint_z = self.z(self.HEAD_JOINT_FROM_BLOW_HOLE)
        self.body_joint_z = self.z((self.TONE_HOLE_POSITIONS[split - 1] + self.TONE_HOLE_POSITIONS[split]) / 2)
        self.keyed_hole_z = self.z(self.TONE_HOLE_POSITIONS[-1])
        self.foot_joint_z = self.keyed_hole_z - self.keyed_hole_radius - self.FOOT_JOINT_MARGIN - self.JOINT_LENGTH/2

        self.o_ring_groove_positions = (
            self.head_joint_z + self.JOINT_LENGTH/4,
            self.body_joint_z + self.JOINT_LENGTH/4,
            self.foot_joint_z - self.JOINT_LENGTH/4
        )

        self.key_rotation = cq.Location(cq.Vector(0, 0, 0), cq.Vector(0, 0, 1), self.KEY_ROTATION_DEGREES)

        self.key_pivot = self.key_rotation * cq.Location(
            cq.Vector(self.key_pivot_radius, 0, self.foot_joint_z),
            cq.Vector(0, -1, 0),
            self.KEY_ANGLE_DEGREES
        )

    def z(self, position_from_blow_hole):
        return self.LENGTH_ABOVE_BLOW_HOLE + position_from_blow_hole

    def ring(self, outer_radius, inner_radius, height):
        return cq.Workplane('XY').circle(outer_radius).circle(inner_radius).extrude(height)

    def blow_hole(self):
        undercut_growth = 2 * self.embouchure_outer_radius * tan(radians(self.EMBOUCHURE_UNDERCUT_DEGREES))

        return (
            cq.Workplane('YZ')
            .ellipse((self.BLOW_HOLE_WIDTH + undercut_growth)/2, (self.BLOW_HOLE_LENGTH + undercut_growth)/2)
            .extrude(self.embouchure_outer_radius, taper=self.EMBOUCHURE_UNDERCUT_DEGREES)
            .translate((0, 0, self.z(0)))
        )

    def tone_hole(self, diameter, z):
        undercut_growth = 2 * self.outer_radius * tan(radians(self.TONE_HOLE_UNDERCUT_DEGREES))

        return (
            cq.Workplane('YZ')
            .circle((diameter + undercut_growth)/2)
            .workplane(offset=self.outer_radius)
            .circle(diameter/2)
            .loft(ruled=True)
            .translate((0, 0, z))
        )

    @cached_property
    def tube(self):
        profile = [(diameter/2, self.z(position)) for position, diameter in self.BORE_PROFILE]

        foot_radius = profile[-1][0]

        bore = (
            cq.Workplane('XZ')
            .polyline([(0, 0), (self.inner_radius, 0)] + profile + [(foot_radius, self.length), (0, self.length)])
            .close()
            .revolve(360, (0, 0, 0), (0, 1, 0))
        )

        tube = cq.Workplane('XY').circle(self.outer_radius).extrude(self.length).cut(bore)

        embouchure_band = self.ring(self.embouchure_outer_radius, self.inner_radius, self.EMBOUCHURE_BAND_LENGTH)

        tube = tube.union(embouchure_band.translate((0, 0, self.z(0) - self.EMBOUCHURE_BAND_LENGTH/2)))

        tube = tube.cut(self.blow_hole())

        cork_top = self.z(0) - self.CORK_DISTANCE

        tube = tube.union(
            cq.Workplane('XY')
            .circle(self.outer_radius)
            .extrude(self.CORK_THICKNESS)
            .translate((0, 0, cork_top - self.CORK_THICKNESS))
        )

        for position in self.o_ring_groove_positions:
            groove = self.ring(self.outer_radius, self.o_ring_groove_radius, self.O_RING_GROOVE_WIDTH)

            tube = tube.cut(groove.translate((0, 0, position - self.O_RING_GROOVE_WIDTH/2)))

        for position, diameter in zip(self.TONE_HOLE_POSITIONS[:-1], self.TONE_HOLE_DIAMETERS[:-1]):
            tube = tube.cut(self.tone_hole(diameter, self.z(position)))

        keyed_hole = self.tone_hole(self.TONE_HOLE_DIAMETERS[-1], self.keyed_hole_z)

        tube = tube.cut(keyed_hole.rotate((0, 0, 0), (0, 0, 1), self.KEY_ROTATION_DEGREES))

        return tube

    def segment(self, start, end=None):
        piece = self.tube

        if start > 0:
            piece = piece.faces('<Z').workplane(-start).split(keepBottom=True)

        if end is not None:
            piece = piece.faces('<Z').workplane(-(end - start)).split(keepTop=True)

        return piece

    def socket(self, z, bonded_below):
        sleeve = self.ring(self.socket_outer_radius, self.socket_radius, self.JOINT_LENGTH)

        bond = self.ring(self.socket_radius, self.outer_radius, self.JOINT_LENGTH/2).translate((0, 0, 0 if bonded_below else self.JOINT_LENGTH/2))

        return sleeve.union(bond).translate((0, 0, z - self.JOINT_LENGTH/2))

    def pad_seat(self):
        return (
            cq.Workplane('YZ')
            .circle(self.keyed_hole_radius)
            .circle(self.keyed_hole_radius + self.PAD_SEAT_WALL)
            .extrude(self.PAD_SEAT_DEPTH + self.PAD_SEAT_HEIGHT)
            .translate((self.outer_radius - self.PAD_SEAT_DEPTH, 0, self.keyed_hole_z))
            .cut(cq.Workplane('XY').circle(self.inner_radius).extrude(self.length))
            .rotate((0, 0, 0), (0, 0, 1), self.KEY_ROTATION_DEGREES)
        )

    def hinge(self):
        box_center = (self.hinge_outer_radius/2, 0, self.JOINT_LENGTH/2)

        return (
            cq.Workplane('XY')
            .circle(self.hinge_outer_radius)
            .extrude(self.JOINT_LENGTH)
            .intersect(cq.Workplane('XY').box(self.hinge_outer_radius, self.HINGE_WIDTH, self.JOINT_LENGTH).translate(box_center))
            .cut(cq.Workplane('XY').circle(self.socket_radius).extrude(self.JOINT_LENGTH))
            .cut(cq.Workplane('XY').box(self.hinge_outer_radius, self.KEY_WIDTH + self.HINGE_SLOT_CLEARANCE, self.JOINT_LENGTH).translate(box_center))
            .cut(cq.Workplane('XZ').cylinder(self.HINGE_WIDTH, self.key_pivot_hole_radius).translate((self.key_pivot_radius, 0, self.JOINT_LENGTH/2)))
            .translate((0, 0, self.foot_joint_z - self.JOINT_LENGTH/2))
            .rotate((0, 0, 0), (0, 0, 1), self.KEY_ROTATION_DEGREES)
        )

    def head(self):
        return self.segment(0, self.head_joint_z).union(self.socket(self.head_joint_z, bonded_below=True))

    def upper_body(self):
        return self.segment(self.head_joint_z, self.body_joint_z).union(self.socket(self.body_joint_z, bonded_below=True))

    def lower_body(self):
        return self.segment(self.body_joint_z, self.foot_joint_z)

    def foot(self):
        return (
            self.segment(self.foot_joint_z)
            .union(self.pad_seat())
            .union(self.socket(self.foot_joint_z, bonded_below=False).union(self.hinge()))
        )

    def key(self):
        spring = (
            cq.Workplane('XZ')
            .polyline([
                (-self.SPRING_BASE_THICKNESS/2, 0),
                (self.SPRING_BASE_THICKNESS/2, 0),
                (self.SPRING_TIP_THICKNESS/2, self.SPRING_LENGTH),
                (-self.SPRING_TIP_THICKNESS/2, self.SPRING_LENGTH)
            ])
            .close()
            .extrude(self.KEY_WIDTH)
            .translate((0, self.KEY_WIDTH/2, 0))
            .rotate((0, 0, 0), (0, -1, 0), self.SPRING_ANGLE_DEGREES)
            .translate((0, 0, -self.SPRING_DISTANCE_FROM_PIVOT))
        )

        shaft = (
            cq.importers.importDXF(self.KEY_PROFILE_FILE)
            .wires()
            .toPending()
            .mirrorY()
            .extrude(self.KEY_THICKNESS)
            .translate((0, self.KEY_LENGTH/2, -self.KEY_THICKNESS/2))
            .rotate((0, 0, 0), (0, 0, 1), 90)
            .rotate((0, 0, 0), (0, 1, 0), 90)
            .cut(cq.Workplane('XZ').cylinder(self.HINGE_WIDTH, self.key_pivot_hole_radius))
            .cut(spring)
        )

        pad_key_slot = cq.Workplane('YZ').box(self.KEY_WIDTH, self.KEY_LENGTH + self.PAD_KEY_SLOT_CLEARANCE, self.KEY_THICKNESS).val().moved(self.key_pivot)

        pad = (
            cq.Workplane('YZ')
            .cylinder(self.KEY_THICKNESS, self.PAD_RADIUS)
            .union(
                cq.Workplane('YZ')
                .cylinder(self.PAD_PLUG_LENGTH * 2, self.keyed_hole_radius - self.PAD_PLUG_CLEARANCE)
                .translate((-self.KEY_THICKNESS/2, 0, 0))
            )
            .translate((self.outer_radius + self.KEY_THICKNESS/2 + self.PAD_SEAT_HEIGHT, 0, self.keyed_hole_z))
            .rotate((0, 0, 0), (0, 0, 1), self.KEY_ROTATION_DEGREES)
            .cut(pad_key_slot)
        )

        return shaft.val().moved(self.key_pivot), spring.val().moved(self.key_pivot), pad

GREEN = 'hsl(120, 100%, 45%)'
YELLOW = 'hsl(55, 100%, 50%)'

flute = BaroqueFlute()

show_object(flute.head(), GREEN, 0.5)
show_object(flute.upper_body(), GREEN, 0.5)
show_object(flute.lower_body(), GREEN, 0.5)
show_object(flute.foot(), GREEN, 0.5)

show_object(flute.key(), YELLOW, 0.5)

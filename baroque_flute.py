from pathlib import Path
from functools import cached_property

import cadquery as cq

class BaroqueFlute:
    BORE_DIAMETER = 19

    WALL_THICKNESS = 1.5

    CORK_DISTANCE = 8
    CORK_THICKNESS = 2

    JOINT_LENGTH = 20
    JOINT_THICKNESS = 1.5

    PAD_RADIUS = 6.5
    PAD_SEAT_WALL = 1
    PAD_SEAT_DEPTH = 2
    PAD_SEAT_HEIGHT = 1
    PAD_PLUG_LENGTH = 1.2

    FOOT_JOINT_MARGIN = 5

    BLOW_HOLE_DIAMETER = 8

    KEY_WIDTH = 5
    KEY_LENGTH = 40
    KEY_THICKNESS = 4
    KEY_PIVOT_OFFSET = 4
    KEY_PROFILE_FILE = Path(__file__).with_name('key.dxf')
    KEY_ANGLE_DEGREES = 6
    KEY_PIVOT_DIAMETER = 1

    HINGE_WIDTH = 15
    HINGE_RADIAL_EXTENT = 7.5

    TONE_HOLE_DIAMETER = 7
    TONE_HOLE_POSITIONS = (220, 260, 300, 340, 380, 420, 470)

    BODY_SPLIT_AFTER_HOLE = 3

    JOINT_CLEARANCE = 0.05
    PAD_PLUG_CLEARANCE = 0.1
    KEY_PIVOT_CLEARANCE = 0.2
    HINGE_SLOT_CLEARANCE = 0.2
    PAD_KEY_SLOT_CLEARANCE = 1

    LENGTH_ABOVE_BLOW_HOLE = 70
    LENGTH_BELOW_BLOW_HOLE = 520

    HEAD_JOINT_FROM_BLOW_HOLE = 130

    MAGNET_DIAMETER = 4
    MAGNET_THICKNESS = 2
    MAGNET_DISTANCE_FROM_PIVOT = 6.5
    MAGNET_GAP = 0.5
    MAGNET_BOSS_WALL = 1
    MAGNET_POCKET_CLEARANCE = 0.1

    def __init__(self):
        self.length = self.LENGTH_ABOVE_BLOW_HOLE + self.LENGTH_BELOW_BLOW_HOLE

        self.inner_radius = self.BORE_DIAMETER/2
        self.outer_radius = self.inner_radius + self.WALL_THICKNESS

        self.tone_hole_radius = self.TONE_HOLE_DIAMETER/2

        self.socket_radius = self.outer_radius + self.JOINT_CLEARANCE
        self.socket_outer_radius = self.outer_radius + self.JOINT_THICKNESS

        self.hinge_outer_radius = self.socket_outer_radius + self.HINGE_RADIAL_EXTENT

        self.key_pivot_radius = self.socket_outer_radius + self.KEY_PIVOT_OFFSET
        self.key_pivot_hole_radius = (self.KEY_PIVOT_DIAMETER + self.KEY_PIVOT_CLEARANCE) / 2

        split = self.BODY_SPLIT_AFTER_HOLE

        self.head_joint_z = self.z(self.HEAD_JOINT_FROM_BLOW_HOLE)
        self.body_joint_z = self.z((self.TONE_HOLE_POSITIONS[split - 1] + self.TONE_HOLE_POSITIONS[split]) / 2)
        self.keyed_hole_z = self.z(self.TONE_HOLE_POSITIONS[-1])
        self.foot_joint_z = self.keyed_hole_z - self.tone_hole_radius - self.FOOT_JOINT_MARGIN - self.JOINT_LENGTH/2

        self.key_pivot = cq.Location(
            cq.Vector(self.key_pivot_radius, 0, self.foot_joint_z),
            cq.Vector(0, -1, 0),
            self.KEY_ANGLE_DEGREES
        )

    def z(self, position_from_blow_hole):
        return self.LENGTH_ABOVE_BLOW_HOLE + position_from_blow_hole

    def pipe(self, outer_radius, inner_radius, height):
        return cq.Workplane('XY').circle(outer_radius).circle(inner_radius).extrude(height)

    def hole(self, diameter, z):
        return cq.Workplane('YZ').circle(diameter/2).extrude(self.outer_radius).translate((0, 0, z))

    @cached_property
    def tube(self):
        tube = self.pipe(self.outer_radius, self.inner_radius, self.length)

        tube = tube.cut(self.hole(self.BLOW_HOLE_DIAMETER, self.z(0)))

        cork_top = self.z(0) - self.CORK_DISTANCE

        tube = tube.union(
            cq.Workplane('XY')
            .circle(self.outer_radius)
            .extrude(self.CORK_THICKNESS)
            .translate((0, 0, cork_top - self.CORK_THICKNESS))
        )

        for position in self.TONE_HOLE_POSITIONS:
            tube = tube.cut(self.hole(self.TONE_HOLE_DIAMETER, self.z(position)))

        return tube

    def segment(self, start, end=None):
        piece = self.tube

        if start > 0:
            piece = piece.faces('<Z').workplane(-start).split(keepBottom=True)

        if end is not None:
            piece = piece.faces('<Z').workplane(-(end - start)).split(keepTop=True)

        return piece

    def socket(self, z, bonded_below):
        sleeve = self.pipe(self.socket_outer_radius, self.socket_radius, self.JOINT_LENGTH)

        bond = self.pipe(self.socket_radius, self.outer_radius, self.JOINT_LENGTH/2).translate((0, 0, 0 if bonded_below else self.JOINT_LENGTH/2))

        return sleeve.union(bond).translate((0, 0, z - self.JOINT_LENGTH/2))

    def pad_seat(self):
        return (
            cq.Workplane('YZ')
            .circle(self.tone_hole_radius)
            .circle(self.tone_hole_radius + self.PAD_SEAT_WALL)
            .extrude(self.PAD_SEAT_DEPTH + self.PAD_SEAT_HEIGHT)
            .translate((self.outer_radius - self.PAD_SEAT_DEPTH, 0, self.keyed_hole_z))
            .cut(cq.Workplane('XY').circle(self.inner_radius).extrude(self.length))
        )

    def magnet_pocket(self, x_from):
        diameter = self.MAGNET_DIAMETER + self.MAGNET_POCKET_CLEARANCE * 2
        depth = self.MAGNET_THICKNESS + self.MAGNET_POCKET_CLEARANCE

        return (
            cq.Workplane('YZ')
            .circle(diameter/2)
            .extrude(depth)
            .translate((x_from, 0, self.MAGNET_DISTANCE_FROM_PIVOT))
        )

    def magnet_seat(self):
        # Key-local frame: the key underside faces -X, the pad is towards +Z.
        boss_top = -self.KEY_THICKNESS/2 - self.MAGNET_GAP
        boss_bottom = self.outer_radius - self.key_pivot_radius
        pocket_bottom = boss_top - self.MAGNET_THICKNESS - self.MAGNET_POCKET_CLEARANCE

        boss = (
            cq.Workplane('YZ')
            .circle(self.MAGNET_DIAMETER/2 + self.MAGNET_POCKET_CLEARANCE + self.MAGNET_BOSS_WALL)
            .extrude(boss_top - boss_bottom)
            .translate((boss_bottom, 0, self.MAGNET_DISTANCE_FROM_PIVOT))
        )

        return boss.val().moved(self.key_pivot), self.magnet_pocket(pocket_bottom).val().moved(self.key_pivot)

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
        )

    def head(self):
        return self.segment(0, self.head_joint_z).union(self.socket(self.head_joint_z, bonded_below=True))

    def upper_body(self):
        return self.segment(self.head_joint_z, self.body_joint_z).union(self.socket(self.body_joint_z, bonded_below=True))

    def lower_body(self):
        return self.segment(self.body_joint_z, self.foot_joint_z)

    def foot(self):
        boss, pocket = self.magnet_seat()

        foot = (
            self.segment(self.foot_joint_z)
            .union(self.pad_seat())
            .union(self.socket(self.foot_joint_z, bonded_below=False).union(self.hinge()))
        )

        return foot.union(boss).cut(pocket)

    def key(self):
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
            .cut(self.magnet_pocket(-self.KEY_THICKNESS/2))
        )

        pad_key_slot = cq.Workplane('YZ').box(self.KEY_WIDTH, self.KEY_LENGTH + self.PAD_KEY_SLOT_CLEARANCE, self.KEY_THICKNESS).val().moved(self.key_pivot)

        pad = (
            cq.Workplane('YZ')
            .cylinder(self.KEY_THICKNESS, self.PAD_RADIUS)
            .union(
                cq.Workplane('YZ')
                .cylinder(self.PAD_PLUG_LENGTH * 2, self.tone_hole_radius - self.PAD_PLUG_CLEARANCE)
                .translate((-self.KEY_THICKNESS/2, 0, 0))
            )
            .translate((self.outer_radius + self.KEY_THICKNESS/2 + self.PAD_SEAT_HEIGHT, 0, self.keyed_hole_z))
            .cut(pad_key_slot)
        )

        return shaft.val().moved(self.key_pivot), pad

GREEN = 'hsl(120, 100%, 45%)'
YELLOW = 'hsl(55, 100%, 50%)'

OPACITY = 0.5

flute = BaroqueFlute()

show_object(flute.head(), GREEN, OPACITY)
show_object(flute.upper_body(), GREEN, OPACITY)
show_object(flute.lower_body(), GREEN, OPACITY)
show_object(flute.foot(), GREEN, OPACITY)

show_object(flute.key(), YELLOW, OPACITY)

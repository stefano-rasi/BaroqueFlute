from pathlib import Path

import cadquery as cq

def ring(outer_radius, inner_radius, height):
    return cq.Workplane('XY').circle(outer_radius).circle(inner_radius).extrude(height)

def radial_hole(radius, length, z):
    return cq.Workplane('YZ').circle(radius).extrude(length).translate((0, 0, z))

class BaroqueFlute:
    BORE_DIAMETER = 19
    FOOT_BORE_DIAMETER = 14
    LENGTH_ABOVE_BLOW_HOLE = 70
    LENGTH_BELOW_BLOW_HOLE = 520
    WALL_THICKNESS = 3

    BLOW_HOLE_DIAMETER = 8
    CORK_DISTANCE = 19
    CORK_THICKNESS = 2

    BODY_SPLIT_AFTER_HOLE = 3
    TONE_HOLE_DIAMETER = 7
    TONE_HOLE_POSITIONS = (220, 260, 300, 340, 380, 420, 470)

    FOOT_JOINT_MARGIN = 5
    HEAD_JOINT_FROM_BLOW_HOLE = 130
    JOINT_LENGTH = 20
    JOINT_THICKNESS = 1.5

    O_RING_CROSS_SECTION = 1.5
    O_RING_GROOVE_WIDTH = 2
    O_RING_SQUEEZE = 0.2

    KEY_ANGLE_DEG = 6
    KEY_LENGTH = 40
    KEY_PIVOT_DIAMETER = 1
    KEY_PIVOT_OFFSET = 4
    KEY_PROFILE_FILE = Path(__file__).with_name('key.dxf')
    KEY_THICKNESS = 4
    KEY_WIDTH = 5

    HINGE_RADIAL_EXTENT = 7.5
    HINGE_WIDTH = 15

    PAD_PLUG_LENGTH = 1.2
    PAD_RADIUS = 6.5
    PAD_SEAT_DEPTH = 2
    PAD_SEAT_HEIGHT = 1
    PAD_SEAT_WALL = 1

    SPRING_ANGLE_DEG = 45
    SPRING_BASE_THICKNESS = 1.5
    SPRING_DISTANCE_FROM_PIVOT = 10
    SPRING_LENGTH = 8
    SPRING_TIP_THICKNESS = 0.5

    HINGE_SLOT_CLEARANCE = 0.2
    JOINT_CLEARANCE = 0.2
    KEY_PIVOT_CLEARANCE = 0.2
    PAD_KEY_SLOT_CLEARANCE = 1
    PAD_PLUG_CLEARANCE = 0.1

    def __init__(self):
        self.length = self.LENGTH_ABOVE_BLOW_HOLE + self.LENGTH_BELOW_BLOW_HOLE

        self.inner_radius = self.BORE_DIAMETER/2
        self.foot_inner_radius = self.FOOT_BORE_DIAMETER/2
        self.outer_radius = self.inner_radius + self.WALL_THICKNESS
        self.socket_radius = self.outer_radius + self.JOINT_CLEARANCE
        self.socket_outer_radius = self.outer_radius + self.JOINT_THICKNESS
        self.hinge_outer_radius = self.socket_outer_radius + self.HINGE_RADIAL_EXTENT
        self.key_pivot_radius = self.socket_outer_radius + self.KEY_PIVOT_OFFSET
        self.key_pivot_hole_radius = (self.KEY_PIVOT_DIAMETER + self.KEY_PIVOT_CLEARANCE) / 2
        self.tone_hole_radius = self.TONE_HOLE_DIAMETER/2
        self.o_ring_groove_radius = self.socket_radius - self.O_RING_CROSS_SECTION * (1 - self.O_RING_SQUEEZE)

        split = self.BODY_SPLIT_AFTER_HOLE

        self.head_joint_z = self.z(self.HEAD_JOINT_FROM_BLOW_HOLE)
        self.body_joint_z = self.z((self.TONE_HOLE_POSITIONS[split - 1] + self.TONE_HOLE_POSITIONS[split]) / 2)
        self.keyed_hole_z = self.z(self.TONE_HOLE_POSITIONS[-1])
        self.foot_joint_z = self.keyed_hole_z - self.tone_hole_radius - self.FOOT_JOINT_MARGIN - self.JOINT_LENGTH/2

        self.o_ring_groove_positions = (
            self.head_joint_z + self.JOINT_LENGTH/4,
            self.body_joint_z + self.JOINT_LENGTH/4,
            self.foot_joint_z - self.JOINT_LENGTH/4
        )

        self.tube = self.build_tube()

    def z(self, position_from_blow_hole):
        return self.LENGTH_ABOVE_BLOW_HOLE + position_from_blow_hole

    def build_tube(self):
        z_axis = cq.Vector(0, 0, 1)

        bore = (
            cq.Workplane('XY')
            .add(cq.Solid.makeCylinder(self.inner_radius, self.head_joint_z, cq.Vector(0, 0, 0), z_axis))
            .union(cq.Solid.makeCone(self.inner_radius, self.foot_inner_radius, self.foot_joint_z - self.head_joint_z, cq.Vector(0, 0, self.head_joint_z), z_axis))
            .union(cq.Solid.makeCylinder(self.foot_inner_radius, self.length - self.foot_joint_z, cq.Vector(0, 0, self.foot_joint_z), z_axis))
        )

        tube = cq.Workplane('XY').circle(self.outer_radius).extrude(self.length).cut(bore)

        tube = tube.cut(radial_hole(self.BLOW_HOLE_DIAMETER/2, self.outer_radius, self.z(0)))

        cork_top = self.z(0) - self.CORK_DISTANCE

        tube = tube.union(
            cq.Workplane('XY')
            .circle(self.outer_radius)
            .extrude(self.CORK_THICKNESS)
            .translate((0, 0, cork_top - self.CORK_THICKNESS))
        )

        for position in self.o_ring_groove_positions:
            groove = ring(self.outer_radius, self.o_ring_groove_radius, self.O_RING_GROOVE_WIDTH)

            tube = tube.cut(groove.translate((0, 0, position - self.O_RING_GROOVE_WIDTH/2)))

        for position in self.TONE_HOLE_POSITIONS:
            tube = tube.cut(radial_hole(self.tone_hole_radius, self.outer_radius, self.z(position)))

        return tube

    def section(self, start, end=None):
        piece = self.tube

        if start > 0:
            piece = piece.faces('<Z').workplane(-start).split(keepBottom=True)

        if end is not None:
            piece = piece.faces('<Z').workplane(-(end - start)).split(keepTop=True)

        return piece

    def socket(self, z, bonded_below):
        sleeve = ring(self.socket_outer_radius, self.socket_radius, self.JOINT_LENGTH)

        bond = ring(self.socket_radius, self.outer_radius, self.JOINT_LENGTH/2).translate((0, 0, 0 if bonded_below else self.JOINT_LENGTH/2))

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
        return self.section(0, self.head_joint_z).union(self.socket(self.head_joint_z, bonded_below=True))

    def upper_body(self):
        return self.section(self.head_joint_z, self.body_joint_z).union(self.socket(self.body_joint_z, bonded_below=True))

    def lower_body(self):
        return self.section(self.body_joint_z, self.foot_joint_z)

    def foot(self):
        return (
            self.section(self.foot_joint_z)
            .union(self.pad_seat())
            .union(self.socket(self.foot_joint_z, bonded_below=False).union(self.hinge()))
        )

    def to_key_pivot(self, shape):
        return (
            shape.rotate((0, 0, 0), (0, -1, 0), self.KEY_ANGLE_DEG)
            .translate((self.key_pivot_radius, 0, self.foot_joint_z))
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
            .rotate((0, 0, 0), (0, -1, 0), self.SPRING_ANGLE_DEG)
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

        pad_key_slot = cq.Workplane('YZ').box(self.KEY_WIDTH, self.KEY_LENGTH + self.PAD_KEY_SLOT_CLEARANCE, self.KEY_THICKNESS)

        pad = (
            cq.Workplane('YZ')
            .cylinder(self.KEY_THICKNESS, self.PAD_RADIUS)
            .union(
                cq.Workplane('YZ')
                .cylinder(self.PAD_PLUG_LENGTH * 2, self.tone_hole_radius - self.PAD_PLUG_CLEARANCE)
                .translate((-self.KEY_THICKNESS/2, 0, 0))
            )
            .translate((self.outer_radius + self.KEY_THICKNESS/2 + self.PAD_SEAT_HEIGHT, 0, self.keyed_hole_z))
            .cut(self.to_key_pivot(pad_key_slot))
        )

        return self.to_key_pivot(shaft), self.to_key_pivot(spring), pad

GREEN = 'hsl(120, 100%, 45%)'
YELLOW = 'hsl(55, 100%, 50%)'

flute = BaroqueFlute()

show_object(flute.head(), GREEN)
show_object(flute.upper_body(), GREEN)
show_object(flute.lower_body(), GREEN)
show_object(flute.foot(), GREEN)

show_object(flute.key(), YELLOW)

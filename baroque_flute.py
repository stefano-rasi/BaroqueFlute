import cadquery as cq

class BaroqueFlute:
    CAP_POSITION = 4
    CAP_THICKNESS = 2

    BORE_THICKNESS = 1.5

    BLOW_HOLE_POSITION = 70

    LENGTH = 520

    TONE_HOLE_POSITIONS = [
        220,
        260,
        300,
        340,
        380,
        420,
        470
    ]

    HEAD_PIECE_LENGTH = 200

    TAIL_PIECE_POSITION = 5

    JOINT_LENGTH = 20
    JOINT_THICKNESS = 1.5
    JOINT_TOLERANCE = 0.05

    HINGE_WIDTH = 15
    HINGE_HEIGHT = 7.5

    HINGE_TOLERANCE = 0.2

    KEY_ANGLE = 6

    KEY_WIDTH = 5
    KEY_HEIGHT = 4
    KEY_LENGTH = 40

    KEY_HOLE_POSITION = 4
    KEY_HOLE_DIAMETER = 1 + 0.2

    def __init__(
        self,

        bore_diameter,

        blow_hole_diameter,
        tone_hole_diameter,

        key_file
    ):
        self.key_file = key_file

        self.bore_diameter = bore_diameter
        self.tone_hole_diameter = tone_hole_diameter

        self.length = self.BLOW_HOLE_POSITION + self.LENGTH

        flute = (
            cq.Workplane('XY')
            .circle(self.inner_radius)
            .circle(self.outer_radius)
            .extrude(self.length)
        )

        flute = flute.cut(
            cq.Workplane('YZ')
            .circle(blow_hole_diameter/2)
            .extrude(self.outer_radius)
            .translate((0, 0, self.BLOW_HOLE_POSITION))
        )

        cap = (
            cq.Workplane('XY')
            .circle(self.outer_radius)
            .extrude(self.CAP_THICKNESS)
            .translate((0, 0, self.BLOW_HOLE_POSITION - blow_hole_diameter/2 - self.CAP_POSITION - self.CAP_THICKNESS))
        )

        flute = flute.union(cap)

        for position in self.TONE_HOLE_POSITIONS:
            flute = flute.cut(
                cq.Workplane('YZ')
                .circle(self.tone_hole_diameter/2)
                .extrude(self.outer_radius)
                .translate((0, 0, self.BLOW_HOLE_POSITION + position))
            )

        self.flute = flute

    @property
    def inner_radius(self):
        return self.bore_diameter/2

    @property
    def outer_radius(self):
        return self.inner_radius + self.BORE_THICKNESS

    @property
    def joint_outer_radius(self):
        return self.outer_radius + self.JOINT_THICKNESS

    @property
    def hinge_outer_radius(self):
        return self.joint_outer_radius + self.HINGE_HEIGHT

    @property
    def key_pivot_radius(self):
        return self.joint_outer_radius + self.KEY_HOLE_POSITION

    @property
    def first_cut(self):
        return self.HEAD_PIECE_LENGTH

    @property
    def second_cut(self):
        return self.BLOW_HOLE_POSITION + (self.TONE_HOLE_POSITIONS[2] + self.TONE_HOLE_POSITIONS[3]) / 2

    @property
    def tail_cut(self):
        return self.BLOW_HOLE_POSITION + self.TONE_HOLE_POSITIONS[-1] - self.tone_hole_diameter/2 - self.TAIL_PIECE_POSITION - self.JOINT_LENGTH/2

    @property
    def last_tone_hole_z(self):
        return self.BLOW_HOLE_POSITION + self.TONE_HOLE_POSITIONS[-1]

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

        return joint.translate((0, 0, z - self.JOINT_LENGTH/2))

    def joint(self):
        joint = (
            cq.Workplane('XY')
            .circle(self.joint_outer_radius)
            .extrude(self.JOINT_LENGTH)
            .cut(
                cq.Workplane('XY')
                .circle(self.outer_radius + self.JOINT_TOLERANCE)
                .extrude(self.JOINT_LENGTH)
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
            .circle(self.tone_hole_diameter/2)
            .circle(self.tone_hole_diameter/2 + 1)
            .extrude(3)
            .translate((self.outer_radius - 2, 0, self.last_tone_hole_z))
            .cut(
                cq.Workplane('XY')
                .circle(self.inner_radius)
                .extrude(self.length)
            )
        )

        hinge_box_center = (self.hinge_outer_radius / 2, 0, self.JOINT_LENGTH/2)

        hinge = (
            cq.Workplane('XY')
            .circle(self.hinge_outer_radius)
            .extrude(self.JOINT_LENGTH)
            .intersect(
                cq.Workplane('XY')
                .box(self.hinge_outer_radius, self.HINGE_WIDTH, self.JOINT_LENGTH)
                .translate(hinge_box_center)
            )
            .cut(
                cq.Workplane('XY')
                .circle(self.outer_radius)
                .extrude(self.JOINT_LENGTH)
            )
        )

        hinge = (
            hinge.cut(
                cq.Workplane('XY')
                .box(self.hinge_outer_radius, self.KEY_WIDTH + self.HINGE_TOLERANCE, self.JOINT_LENGTH)
                .translate(hinge_box_center)
            )
            .cut(
                cq.Workplane('XZ')
                .cylinder(self.HINGE_WIDTH, self.KEY_HOLE_DIAMETER/2)
                .translate((self.key_pivot_radius, 0, self.JOINT_LENGTH/2))
            )
        )

        return piece.union(self._joint_at(self.tail_cut, self.joint().union(hinge)))

    def _to_key_pivot(self, shape):
        return (
            shape.rotate((0, 1, 0), (0, 0, 0), self.KEY_ANGLE)
            .translate((self.key_pivot_radius, 0, self.tail_cut))
        )

    def key(self):
        STOPPER_LENGTH = 1.2
        STOPPER_RADIUS = 6.5

        STOPPER_TOLERANCE = 0.1
        STOPPER_TOLERANCE_2 = 1

        SPRING_ANGLE = 90 - 45

        SPRING_LENGTH = 8
        SPRING_POSITION = 10

        SPRING_TOP_THICKNESS = 0.5
        SPRING_BASE_THICKNESS = 1.5

        stopper = (
            cq.Workplane('YZ')
            .cylinder(self.KEY_HEIGHT, STOPPER_RADIUS)
            .union(
                cq.Workplane('YZ')
                .cylinder(STOPPER_LENGTH * 2, self.tone_hole_diameter/2 - STOPPER_TOLERANCE)
                .translate((-self.KEY_HEIGHT/2, 0, 0))
            )
        )

        stopper = stopper.translate(
            (self.outer_radius + self.KEY_HEIGHT/2 + 1, 0, self.last_tone_hole_z)
        )

        spring = (
            cq.Workplane('XZ')
            .polyline([
                (-SPRING_BASE_THICKNESS/2, 0),
                (SPRING_BASE_THICKNESS/2, 0),
                (SPRING_TOP_THICKNESS/2, SPRING_LENGTH),
                (-SPRING_TOP_THICKNESS/2, SPRING_LENGTH)
            ])
            .close()
            .extrude(self.KEY_WIDTH)
            .translate((0, (self.KEY_WIDTH)/2, 0))
            .rotate((0, 1, 0), (0, 0, 0), SPRING_ANGLE)
            .translate((0, 0, -SPRING_POSITION))
        )

        shaft = (
            cq.importers.importDXF(self.key_file)
            .wires()
            .toPending()
            .mirrorY()
            .extrude(self.KEY_HEIGHT)
            .translate((0, self.KEY_LENGTH/2, -self.KEY_HEIGHT/2))
            .rotate((0, 0, 0), (0, 0, 1), 90)
            .rotate((0, 0, 0), (0, 1, 0), 90)
            .cut(
                cq.Workplane('XZ')
                .cylinder(self.HINGE_WIDTH, self.KEY_HOLE_DIAMETER/2)
            )
            .cut(spring)
        )

        shaft_tolerance = (
            cq.Workplane('YZ')
            .box(self.KEY_WIDTH, self.KEY_LENGTH + STOPPER_TOLERANCE_2, self.KEY_HEIGHT)
        )

        shaft = self._to_key_pivot(shaft)
        spring = self._to_key_pivot(spring)
        shaft_tolerance = self._to_key_pivot(shaft_tolerance)

        stopper = stopper.cut(shaft_tolerance)

        return shaft, spring, stopper

import os

KEY_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'key.dxf')

flute = BaroqueFlute(
    19, 8, 7,
    KEY_FILE
)

GREEN = 'hsl(120, 100%, 45%)'
YELLOW = 'hsl(55, 100%, 50%)'

show_object(flute.head_piece(), GREEN)
show_object(flute.first_piece(), GREEN)
show_object(flute.second_piece(), GREEN)
show_object(flute.tail_piece(), GREEN)

show_object(flute.key(), YELLOW)

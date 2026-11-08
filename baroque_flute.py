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

        self.tail_piece_start = self.TONE_HOLE_POSITIONS[-1] - tone_hole_diameter/2 - self.TAIL_PIECE_POSITION - self.JOINT_LENGTH/2

        length = self.BLOW_HOLE_POSITION + self.LENGTH

        flute = (
            cq.Workplane('XY')
            .circle(self.bore_diameter/2)
            .circle(self.bore_diameter/2 + self.BORE_THICKNESS)
            .extrude(length)
        )

        flute = flute.cut(
            cq.Workplane('YZ')
            .circle(blow_hole_diameter/2)
            .extrude(self.bore_diameter/2 + self.BORE_THICKNESS)
            .translate((0, 0, self.BLOW_HOLE_POSITION))
        )

        cap = (
            cq.Workplane('XY')
            .circle(self.bore_diameter/2 + self.BORE_THICKNESS)
            .extrude(self.CAP_THICKNESS)
            .translate((0, 0, self.BLOW_HOLE_POSITION - blow_hole_diameter/2 - self.CAP_POSITION - self.CAP_THICKNESS))
        )

        flute = flute.union(cap)

        for position in self.TONE_HOLE_POSITIONS:
            flute = flute.cut(
                cq.Workplane('YZ')
                .circle(self.tone_hole_diameter/2)
                .extrude(self.bore_diameter/2 + self.BORE_THICKNESS)
                .translate((0, 0, self.BLOW_HOLE_POSITION + position))
            )

        self.flute = flute

    def joint(self):
        joint = (
            cq.Workplane('XY') 
            .circle(self.bore_diameter/2 + self.BORE_THICKNESS + self.JOINT_THICKNESS)
            .extrude(self.JOINT_LENGTH)
            .cut(
                cq.Workplane('XY') 
                .circle(self.bore_diameter/2 + self.BORE_THICKNESS + self.JOINT_TOLERANCE)
                .extrude(self.JOINT_LENGTH)
            )
        )

        return joint

    def head_piece(self):
        piece = (
            self.flute.faces('<Z')
            .workplane(-self.HEAD_PIECE_LENGTH)
            .split(True, False)
        )

        joint = (
            self.joint()
            .translate((0, 0, self.HEAD_PIECE_LENGTH - self.JOINT_LENGTH/2))
        )

        piece = piece.union(joint)

        return piece

    def first_piece(self):
        start = self.HEAD_PIECE_LENGTH

        end = self.BLOW_HOLE_POSITION + self.TONE_HOLE_POSITIONS[2] + self.tone_hole_diameter/2 + (self.TONE_HOLE_POSITIONS[3] - self.tone_hole_diameter/2 - self.TONE_HOLE_POSITIONS[2] - self.tone_hole_diameter/2) / 2

        piece = (
            self.flute.faces('<Z')
            .workplane(-start)
            .split(False, True)
            .faces('<Z')
            .workplane(-(end - start))
            .split(True, False)
        )

        joint = (
            self.joint()
            .translate((0, 0, end - self.JOINT_LENGTH/2))
        )

        piece = piece.union(joint)

        return piece

    def second_piece(self):
        start = self.BLOW_HOLE_POSITION + self.TONE_HOLE_POSITIONS[2] + self.tone_hole_diameter/2 + (self.TONE_HOLE_POSITIONS[3] - self.tone_hole_diameter/2 - self.TONE_HOLE_POSITIONS[2] - self.tone_hole_diameter/2) / 2

        end = self.BLOW_HOLE_POSITION + self.tail_piece_start

        piece = (
            self.flute.faces('<Z')
            .workplane(-start)
            .split(False, True)
            .faces('<Z')
            .workplane(-(end - start))
            .split(True, False)
        )

        return piece

    def tail_piece(self):
        start = self.BLOW_HOLE_POSITION + self.tail_piece_start

        piece = (
            self.flute.faces('<Z')
            .workplane(-start)
            .split(False, True)
        )

        piece = piece.union(
            cq.Workplane('YZ')
            .circle(self.tone_hole_diameter/2)
            .circle(self.tone_hole_diameter/2 + 1)
            .extrude(3)
            .translate((self.bore_diameter/2 + self.BORE_THICKNESS - 2, 0, self.BLOW_HOLE_POSITION + self.TONE_HOLE_POSITIONS[-1]))
            .cut(
                cq.Workplane('XY')
                .circle(self.bore_diameter/2)
                .extrude(self.BLOW_HOLE_POSITION + self.LENGTH)
            )
        )

        joint = self.joint()

        hinge = (
            cq.Workplane('XY') 
            .circle(self.bore_diameter/2 + self.BORE_THICKNESS + self.JOINT_THICKNESS + self.HINGE_HEIGHT)
            .extrude(self.JOINT_LENGTH)
            .intersect(
                cq.Workplane('XY') 
                .box(self.bore_diameter/2 + self.BORE_THICKNESS + self.JOINT_THICKNESS + self.HINGE_HEIGHT, self.HINGE_WIDTH, self.JOINT_LENGTH)
                .translate(((self.bore_diameter/2 + self.BORE_THICKNESS + self.JOINT_THICKNESS + self.HINGE_HEIGHT) / 2, 0, self.JOINT_LENGTH/2))
            )
            .cut(
                cq.Workplane('XY') 
                .circle(self.bore_diameter/2 + self.BORE_THICKNESS)
                .extrude(self.JOINT_LENGTH)
            )
        )

        hinge = (
            hinge.cut(
                cq.Workplane('XY') 
                .box(self.bore_diameter/2 + self.BORE_THICKNESS + self.JOINT_THICKNESS + self.HINGE_HEIGHT, self.KEY_WIDTH + self.HINGE_TOLERANCE, self.JOINT_LENGTH)
                .translate(((self.bore_diameter/2 + self.BORE_THICKNESS + self.JOINT_THICKNESS + self.HINGE_HEIGHT) / 2, 0, self.JOINT_LENGTH/2))
            )
            .cut(
                cq.Workplane('XZ')
                .cylinder(self.HINGE_WIDTH, self.KEY_HOLE_DIAMETER/2)
                .translate((self.bore_diameter/2 + self.BORE_THICKNESS + self.JOINT_THICKNESS + self.KEY_HOLE_POSITION, 0, self.JOINT_LENGTH/2))
            )
        )

        joint = (
            joint.union(hinge)
            .translate((0, 0, start - self.JOINT_LENGTH/2))
        )

        piece = piece.union(joint)

        return piece

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
            (self.bore_diameter/2 + self.BORE_THICKNESS + self.KEY_HEIGHT/2 + 1, 0, self.BLOW_HOLE_POSITION + self.TONE_HOLE_POSITIONS[-1])
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

        shaft = (
            shaft.rotate((0, 1, 0), (0, 0, 0), self.KEY_ANGLE)
            .translate((self.bore_diameter/2 + self.BORE_THICKNESS + self.JOINT_THICKNESS + self.KEY_HOLE_POSITION, 0, self.BLOW_HOLE_POSITION + self.tail_piece_start))
        )

        spring = (
            spring.rotate((0, 1, 0), (0, 0, 0), self.KEY_ANGLE)
            .translate((self.bore_diameter/2 + self.BORE_THICKNESS + self.JOINT_THICKNESS + self.KEY_HOLE_POSITION, 0, self.BLOW_HOLE_POSITION + self.tail_piece_start))
        )

        shaft_tolerance = (
            cq.Workplane('YZ')
            .box(self.KEY_WIDTH, self.KEY_LENGTH + STOPPER_TOLERANCE_2, self.KEY_HEIGHT)
            .rotate((0, 1, 0), (0, 0, 0), self.KEY_ANGLE)
            .translate((self.bore_diameter/2 + self.BORE_THICKNESS + self.JOINT_THICKNESS + self.KEY_HOLE_POSITION, 0, self.BLOW_HOLE_POSITION + self.tail_piece_start))
        )

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
from Frame import Frame
from Main import Main
from NodeGenerator import NodeGenerator
from Pixel import Pixel
from BoundedIntFrameHeight import BoundedIntFrameHeight
from BoundedIntFrameWidth import BoundedIntFrameWidth
from Operation import Operation
import random

row = []
for x in range(0, Main.x_resolution):
    column = []
    for y in range(0, Main.y_resolution):
        r = random.randint(0, 255)
        g = random.randint(0, 255)
        b = random.randint(0, 255)
        column.append(Pixel(r, g, b, 255))
    row.append(column)

frame = Frame(row)

i0 = NodeGenerator(frame)
i1 = Operation()
c2 = 0
c3 = 0
i4 = -14
i4 = i1.actALessThanB(c3,c3)
i6 = -5
i7 = i0.constructNodeRectangle(793,448,246,568)
i8 = 36
i8 = i1.actAdd(i6,i7.determineAverageBlue())
i10 = -5
c3 = i1.actRemainder(i7.determineAverageBlue(),i10)
i12 = i0.constructNodeRectangle(611,250,416,0)
i13 = 1
i13 = i1.actDivide(i6,i12.determineAverageGreen())
i15 = -54
c3 = i1.actAGreaterThanB(i15,c2)
c2 = i1.actAGreaterThanB(i4,i6)
i18 = 80
c3 = i1.actAGreaterThanB(i8,i18)
i20 = i0.constructNodeRectangle(54,192,41,436)
i21 = i0.constructNodeRectangle(245,86,374,530)
c2 = i1.actLcd(i20.determineAverageBlue(),i21.determineAverageGreen())
i23 = -69
i24 = 49
i24 = i1.actMultiply(i23,i10)
i26 = 2
i24 = i1.actDivide(i26,c2)
i28 = -88
i29 = 34
i29 = i1.actGcd(c3,i28)
i31 = i0.constructNodeRectangle(131,342,92,360)
c2 = i1.actGcd(i20.determineAverageRed(),i31.determineAverageRed())
i4 = i1.actGcd(c2,c2)
i34 = i0.constructNodeRectangle(26,138,163,266)
i35 = 83
i36 = -65
i36 = i1.actAdd(i34.determineAverageBlue(),i35)
i35 = i1.actSubtract(i8,i21.determineAverageRed())
i39 = 95
i8 = i1.actAGreaterThanB(c3,i39)
i8 = i1.actMultiply(c3,c2)
i42 = 76
c3 = i1.actLcd(i42,i34.determineAverageBlue())
i39 = i1.actLcd(i20.determineAverageBlue(),i28)
i45 = -87
i45 = i1.actAdd(i28,c3)
i47 = i0.constructNodeRectangle(511,408,506,581)
i29 = i1.actMultiply(i47.determineAverageRed(),i21.determineAverageRed())
i13 = i1.actRemainder(c3,c3)
i50 = 55
c3 = i1.actRemainder(i50,i24)
i52 = 69
i53 = -51
i53 = i1.actMultiply(i21.determineAverageRed(),i52)
i55 = 60
i56 = 29
c2 = i1.actALessThanB(i55,i56)

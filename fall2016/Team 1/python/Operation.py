

class Operation:

    def __init__(self):
        pass

    def actAdd(self, a, b):
        return a + b

    def actSubtract(self, a, b):
        return a - b

    def actMultiply(self, a, b):
        return a * b

    def actDivide(self, a, b):
        if (b == 0):
            return a
        return a / b

    def actRemainder(self, a, b):
        if (b == 0):
            return a
        return a % b

    def actALessThanB(self, a, b):
        if (a < b):
            return 1
        else:
            return 0

    def actAGreaterThanB(self, a, b):
        if (a > b):
            return 1
        else:
            return 0

    def actGcd(self, a, b):
        if (a == 0):
            return b
        if (a < 0 or b < 0):
            return a
        while(b != 0):
            if (a > b):
                a = a - b
            else:
                b = b - 1
        return a

    def actLcd(self, a, b):
        if (self.actGcd(a, b) == 0):
            return a
        return (a * b / self.actGcd(a, b))

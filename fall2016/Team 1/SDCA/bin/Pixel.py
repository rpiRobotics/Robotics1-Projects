

class Pixel:
    red = 0
    green = 0
    blue = 0
    maxValue = 0

    def __init__(self, red, green, blue, maxValue):
        self.red = red
        self.green = green
        self.blue = blue
        self.maxValue = maxValue

if __name__ == "__main__":
    testPixel = Pixel(1, 2, 3, 4)
    print("Red      1 = " + str(testPixel.red))
    print("Green    2 = " + str(testPixel.green))
    print("Blue     3 = " + str(testPixel.blue))
    print("MaxValue 4 = " + str(testPixel.maxValue))

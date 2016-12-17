from Pixel import Pixel

class Frame:

    pixelGrid = []

    def __init__(self, pixelGrid):
        self.pixelGrid = pixelGrid

    def getPixelAt(self, x, y):
        return self.pixelGrid[x][y]

    def getFrameWidth(self):
        return len(self.pixelGrid)

    def getFrameHeight(self):
        return len(self.pixelGrid[0])


if __name__ == "__main__":
    pixelA = Pixel(1, 2, 3, 4)
    pixelB = Pixel(2, 3, 4, 4)
    pixelC = Pixel(0, 1, 2, 4)
    pixelD = Pixel(3, 3, 3, 4)

    pixelGrid = [[pixelA, pixelB], [pixelC, pixelD]]

    testFrame = Frame(pixelGrid)

    v1 = testFrame.getPixelAt(1, 1)
    v2 = testFrame.getFrameWidth()
    v3 = testFrame.getFrameHeight()

    print(v1)
    print(v2)
    print(v3)

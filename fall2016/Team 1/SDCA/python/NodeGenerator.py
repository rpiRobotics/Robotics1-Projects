from Frame import Frame
from Pixel import Pixel
from Node import Node

class NodeGenerator:

    frame = None

    def __init__(self, frame):
        self.frame = frame

    def constructNodeRectangle(self, x1, x2, y1, y2):
        lowerX = min(x1, x2)
        upperX = max(x1, x2)
        lowerY = min(y1, y2)
        upperY = max(y1, y2)

        pixelsInNode = []
        for x in range(lowerX, upperX):
            for y in range(lowerY, upperY):
                pixelsInNode.append(self.frame.getPixelAt(x, y))

        return Node(pixelsInNode)

if __name__ == "__main__":
    pixelA = Pixel(1, 2, 3, 4)
    pixelB = Pixel(2, 3, 4, 4)
    pixelC = Pixel(0, 1, 2, 4)
    pixelD = Pixel(3, 3, 3, 4)

    pixelGrid = [[pixelA, pixelB], [pixelC, pixelD]]

    testFrame = Frame(pixelGrid)
    testNodeGenerator = NodeGenerator(testFrame)

    v1 = testNodeGenerator.constructNodeRectangle(0, 0, 1, 0)

    print(v1)

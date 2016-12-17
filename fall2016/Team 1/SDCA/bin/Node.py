from Pixel import Pixel

class Node:

    pixelsInNode = []

    def __init__(self, pixels):
        self.pixelsInNode = pixels

    def determineAverageRed(self):
        if (len(self.pixelsInNode) == 0):
            return 0
        red_sum = 0
        for pixel in self.pixelsInNode:
            red_sum += pixel.red
        red_sum = red_sum / len(self.pixelsInNode)
        return red_sum

    def determineAverageGreen(self):
        if (len(self.pixelsInNode) == 0):
            return 0
        green_sum = 0
        for pixel in self.pixelsInNode:
            green_sum += pixel.green
        green_sum = green_sum / len(self.pixelsInNode)
        return green_sum

    def determineAverageBlue(self):
        if (len(self.pixelsInNode) == 0):
            return 0
        blue_sum = 0
        for pixel in self.pixelsInNode:
            blue_sum += pixel.blue
        blue_sum = blue_sum / len(self.pixelsInNode)
        return blue_sum

    
if __name__ == "__main__":
    pixelA = Pixel(1, 2, 3, 4)
    pixelB = Pixel(0, 1, 2, 4)
    pixels = [pixelA, pixelB]

    testNode = Node(pixels)
    v1 = testNode.determineAverageRed()
    v2 = testNode.determineAverageGreen()
    v3 = testNode.determineAverageBlue()

    print(v1)
    print(v2)
    print(v3)

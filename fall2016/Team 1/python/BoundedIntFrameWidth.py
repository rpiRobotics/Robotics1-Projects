import Main

class BoundedIntFrameWidth:

    value = 0
    minimum = 0
    maximum = Main.Main.x_resolution - 1

    def __init__(self, value):
        self.value = value

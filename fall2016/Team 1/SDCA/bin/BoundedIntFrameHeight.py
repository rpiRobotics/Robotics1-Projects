import Main

class BoundedIntFrameHeight:

    value = 0
    minimum = 0
    maximum = Main.Main.y_resolution - 1

    def __init__(self, value):
        self.value = value

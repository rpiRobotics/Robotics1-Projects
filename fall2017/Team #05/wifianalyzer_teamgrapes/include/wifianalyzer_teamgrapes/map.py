import numpy as np

class Map:
    def __init__(self, turnArray):
        self.map_array=[
            ((0,0),(-1,-1,1,-1)), #0
            ((0,-1),(0,-1,2,-1)), #1
            ((0,-2),(1,4,3,-1)),  #2
            ((0,-3),(2,-1,5,-1)), #3
            ((1,-2),(-1,6,-1,2)), #4
            ((0,-4),(3,-1,7,-1)), #5
            ((2,-2),(8,9,10,4)),  #6
            ((0,-5),(5,11,-1,-1)),#7
            ((2,-1),(12,-1,6,-1)),#8
            ((3,-2),(-1,13,-1,6)),#9
            ((2,-3),(6,-1,14,-1)),#10
            ((1,-5),(-1,15,-1,7)),#11
            ((2,0),(16,-1,8,-1)), #12
            ((4,-2),(17,-1,18,9)),#13
            ((2,-4),(10,-1,15,-1)),#14
            ((2,-5),(14,19,-1,11)),#15
            ((2,1),(-1,-1,12,20)),#16
            ((4,-1),(21,-1,13,-1)),#17
            ((4,-3),(13,-1,22,-1)),#18
            ((3,-5),(-1,23,-1,15)),#19
            ((1,1),(24,16,-1,-1)),#20
            ((4,0),(-1,-1,17,25)),#21
            ((4,-4),(18,-1,23,-1)),#22
            ((4,-5),(22,-1,-1,19)),#23
            ((1,2),(-1,26,20,-1)),#24
            ((3,0),(27,21,-1,-1)),#25
            ((2,2),(-1,28,-1,24)),#26
            ((3,1),(28,-1,25,-1)),#27
            ((3,2),(-1,-1,27,26))]#28
        
        self.cur_index = 0
        self.next_index = 1
        self.cur_head = 3
        self.turn_array = turnArray
        
    def north(self):
        return self.map_array[self.cur_index][1][0]
        
    def east(self):
        return self.map_array[self.cur_index][1][1]
        
    def south(self):
        return self.map_array[self.cur_index][1][2]
    
    def west(self):
        return self.map_array[self.cur_index][1][3]
    
    def atRedLine(self):
        if len(self.turn_array) == 0:
            return -1
            
        turnDir = self.turn_array.pop(0)
        intDir = (turnDir + self.cur_head) % 4
        
        intindex = self.map_array[self.cur_index][1][self.cur_head]
        
        return self.map_array[intindex][1][intDir]
    
    def findNextIntersection(self):
        
    
    def nextOnRoad(self):
        self.cur_index = self.next_index
        
        

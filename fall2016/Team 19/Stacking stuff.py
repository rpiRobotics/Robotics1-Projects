
height = 15
length = 30

if cnt == 40:
    
    init = []
    fin = []
    x_in = -90
    y_in = 210
    z_in = 12
    
    x_out = 30
    y_out = 210
    z_out = 12
    
    for i in range(6):
        temp = kin.getNthPosition(x_in,y_in,z_in,i)
        init.append([temp[0],temp[1],temp[2],temp[3]])
        
        temp = kin.getNthPosition(x_out,y_out,z_out,i)
        fin.append([temp[0],temp[1],temp[2],temp[3]]) 
        
    init.reverse()
    
    
    for i in range(3):
        self.moveblock(init[i], fin[i])
    
    
    self.setPos(0,217,170,0,0)
    
    
    for i in range(3):
        self.moveblock(init[3+i], fin[3+i])
    
    
    
    
    self.setPos(0,150,25,0,0)
    self.setPos(-90,150,25,0,0)
    self.setPos(-90,210,25,0,0)
    
    for i in range(-90, -45, 5):
        self.setPos(i,210,25,0,0)
        
    for i in range(210, 170, -5):
        self.setPos(-45,i,25,0,0)
        
    for i in range(-45, 45, 5):
        self.setPos(i,170,25,0,0)
        
    for i in range(170, 210, 5):
        self.setPos(45, i, 25,0,0)
        
    for i in range(45, 90, 5):
        self.setPos(i,210,25,0,0)
        
    self.setJointPositions(0,0,0,0,0)
    
        
        
        
        
        
        
        
    
    

    self.setPos(0,150,45,0,0)
    self.setPos(0,150-length,height,0,0)
    
    for i in range(3):
        self.setPos(0, 150 + length*(i-1), (i+1) * height, 0,0)
        self.setPos(0, 150 + length*i, (i+1)*height, 0,0)
    
    




    
            self.setPos(0,150,13,0,1)
            self.setPos(-90,210,100,0,1)                

            self.setPos(-90,150,100,90-self.base_rotation,1)

            self.setPos(-90,210,30,90-self.base_rotation,1)

            time.sleep(8)
            for i in range(-90,91,30):
                print i
                self.setPos(i,210,30,90-self.base_rotation,1)
                time.sleep(3)

            self.setPos(90,210,15,90-self.base_rotation,0)

    for i in range(150,271,30):
        self.setPos(0, i, 9,0,0)
    """


    #if cnt > 90 and cnt< 180 and cnt % 10 ==0:
        #print('yolo')
        #self.setJointPositions(50, 40, 5, -50,0)
        #send_absolute_position(10,50,50,0,1)
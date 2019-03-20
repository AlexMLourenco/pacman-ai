import random
import sys
import json
import asyncio
import websockets
import os
import heapq
import math
from mapa import Map



class Cell(object):

    def __init__(self, x, y, reachable, cross):
        self.x = x
        self.y = y
        self.parent = None
        self.reachable = reachable
        self.cross = cross
        self.g = 0
        self.h = 0
        self.f = 0

    def __lt__(self,other):
        return self.f < other.f

    def __str__(self):
        return "(" + str(self.x) + "," + str(self.y) + ")" 

class AStar(object):

    def __init__(self, map):
        self.opened = []    # open list
        heapq.heapify(self.opened)
        self.closed = set() # visited cells list
        self.cells = [] # grid cells
        self.grid_height = None
        self.grid_width = None
        self.map = map
        self.start = None
        self.end = None
        self._energy = None
        self._ghosts = None
        self._boost = None
                
    def set_start(self, start):
        self.start= self.get_cell(start.x,start.y)

    def set_end(self, end):
        self.end= self.get_cell(end.x,end.y)

    def update_status(self, energy,ghosts,boost):
        self._energy = energy
        self._ghosts = ghosts
        self._boost = boost

    def init_grid(self, width, height, walls):
        self.grid_height = height
        self.grid_width = width
        for x in range(self.grid_width):
            for y in range(self.grid_height):
               
                if self.map.is_wall((x, y)):
                    reachable = False
                else:
                    reachable = True

                self.cells.append(Cell(x, y, reachable,((x==0 or y == 0 or x == self.grid_width -1 or y == self.grid_height -1) and reachable)))

    def get_heuristic(self, cell):
        return 10 * (abs(cell.x - self.end.x) + abs(cell.y - self.end.y))

    def get_cell(self, x, y):
        return self.cells[x * self.grid_height + y]

    def get_adjacent_cells(self, cell):
        cells = []
        if cell.x < self.grid_width-1:
            cells.append(self.get_cell(cell.x+1, cell.y))
        if cell.y > 0:
            cells.append(self.get_cell(cell.x, cell.y-1))
        if cell.x > 0:
            cells.append(self.get_cell(cell.x-1, cell.y))
        if cell.y < self.grid_height-1:
            cells.append(self.get_cell(cell.x, cell.y+1))

        #cross to other side
        if (cell.x == 0):
            cells.append(self.get_cell(self.grid_width-1, cell.y))

        if (cell.x == self.grid_width-1):
            cells.append(self.get_cell(0, cell.y))

        if cell.y == 0:
            cells.append(self.get_cell(cell.x, self.grid_height-1))

        if cell.y == self.grid_height-1:
            cells.append(self.get_cell(cell.x, 0))
            

        return cells

    def get_path(self):
        cell = self.end
        if cell == None:
            return None
        path = [(cell.x, cell.y)]
        while cell.parent is not self.start:
            cell = cell.parent
            if (cell == None):
                break
            path.append((cell.x, cell.y))

        path.append((self.start.x, self.start.y))
        path.reverse()
        return path

    def update_cell(self, adj, cell):
        adj.g = cell.g + 10 
        adj.h = self.get_heuristic(adj)
        adj.parent = cell
        adj.f = adj.h + adj.g

    def calc_distance(self, cross, point_x,point_y):
              
        x_cross = None
        y_cross = None
        if (cross.x == 0):
            x_cross = self.grid_width-1
            y_cross = cross.y
                
        if (cross.y == 0):
            x_cross = cross.x
            y_cross = self.grid_height-1

        if (cross.x == self.grid_width-1):
            x_cross = 0
            y_cross = cross.y

        if (cross.y == self.grid_height-1):
            x_cross = cross.x
            y_cross = 0

        x = abs(point_x - x_cross)
        y = abs(point_y - y_cross)
        return math.sqrt(math.pow(x,2) + math.pow(y,2))

    def is_ghost_in_path(self,path,length):
        cnt = 0
        for p in path:
            x,y = p
            if cnt == length:
                break
            for ghost in self._ghosts:                
          
               if (ghost[0][0] == x and ghost[0][1] == y and ghost[1]==False):
                   return True  
            cnt=cnt+1
        return False

    def is_cell_in_list(self,x,y,list):
        for c in list:
            if (c[0] == x and c[1] == y):
                return True
        return False    
    
    def get_closest_point(self,pacman,excluded):
        g = list()
        min_path = 99999999
        min_path_ghost = 99999999
        min_path_boost = 99999999
        closest_energy = None
        closest_ghost = None
        closest_boost = None
        distance = None
        distance_ghost = None 
        distance_boost = None

        for boost in self._boost:
            for ghost in self._ghosts:
                if ghost[1] == True:
                    continue
                x = abs(boost[0] - pacman.x)
                y = abs(boost[1] - pacman.y)
                distance_boost = math.sqrt(math.pow(x,2) + math.pow(y,2))
                if distance_boost < min_path_boost and not self.is_cell_in_list(boost[0],boost[1],excluded):
                    min_path_boost = distance_boost
                    closest_boost = boost 

        for energy in self._energy: # Calculate distance between pacman and this energy point   

            reachable = True    
            for ghost in self._ghosts:  # if energy point is on pacman and packman cant be eaten
                if energy[0]  == ghost[0][0] and energy[1] == ghost[0][1] and ghost[1]==False:
                    g.append(energy)    
                    reachable = False

            if (reachable): # calculate the nearest distance to the next energy point
                x = abs(energy[0] - pacman.x)
                y = abs(energy[1] - pacman.y)
                distance = math.sqrt(math.pow(x,2) + math.pow(y,2))
                if distance < min_path and not self.is_cell_in_list(energy[0],energy[1],excluded):
                    min_path = distance
                    closest_energy = energy 
        
        for ghost in self._ghosts:
            if ghost[1] == False:
                continue
            x = abs(ghost[0][0] - pacman.x)
            y = abs(ghost[0][1] - pacman.y)
            distance_ghost = math.sqrt(math.pow(x,2) + math.pow(y,2))
            if distance_ghost< min_path_ghost and not self.is_cell_in_list(ghost[0][0],ghost[0][1],excluded):
                min_path_ghost = distance_ghost
                closest_ghost = [ghost[0][0],ghost[0][1]] 

        #cross side
        for cell in self.cells:
            if (cell.cross ==True):               
                x = abs(cell.x - pacman.x)
                y = abs(cell.y - pacman.y)
                distance = math.sqrt(math.pow(x,2) + math.pow(y,2))
                for energy in self._energy:                
                    d2 = self.calc_distance(cell,energy[0],energy[1])
                    if distance+d2 < min_path and not self.is_cell_in_list(energy[0],energy[1],excluded):                   
                        min_path = distance+d2
                        closest_energy = energy 

                for ghost in self._ghosts:    
                    if ghost[1] == False: 
                        continue            
                    d2 = self.calc_distance(cell,ghost[0][0],ghost[0][1])
                    if distance+d2 < min_path_ghost and not self.is_cell_in_list(ghost[0][0],ghost[0][1],excluded):
                        min_path_ghost = distance+d2
                        closest_ghost = [ghost[0][0],ghost[0][1]] 

                for boost in self._boost:    
                    if ghost[1] == True: 
                        continue            
                    d3 = self.calc_distance(cell,ghost[0][0],ghost[0][1])
                    if distance+d2 < min_path_boost and not self.is_cell_in_list(boost[0],boost[1],excluded):
                        min_path_boost = distance+d3
                        closest_boost = [boost[0],boost[1]] 
        
        if (closest_ghost != None):
            if (min_path_ghost > 4 and min_path <=4):
                return closest_energy
            else:
                return closest_ghost
        
        else:
            return closest_energy

    def walls_to_array(self): #TODO: Refactor 
        x, y = self.map.size
        walls = []
        for column in range(x):
            for line in range(y):
                if self.map.is_wall((column, line)):
                    walls.append((column, line))

    def solve(self):   
        heapq.heappush(self.opened, (self.start.f, self.start)) # add starting cell to open heap queue
        while len(self.opened):
            f, cell = heapq.heappop(self.opened)    # pop cell from heap queue
            self.closed.add(cell)                   # add cell to closed list so we don't process it twice
            if cell is self.end:                    # if ending cell, return found path
                return self.get_path()
            
            adj_cells = self.get_adjacent_cells(cell)   # get adjacent cells for cell
            for adj_cell in adj_cells:
                if adj_cell.reachable and adj_cell not in self.closed:
                    if (adj_cell.f, adj_cell) in self.opened:   # if adj cell in open list, check if current path is
                        if adj_cell.g > cell.g + 10:            # better than the one previously found
                            self.update_cell(adj_cell, cell)    # for this adj cell.
                    else:
                        self.update_cell(adj_cell, cell)
                        heapq.heappush(self.opened, (adj_cell.f, adj_cell)) # add adj cell to open list

class Student:

    def __init__(self, server,port,name):
        loop = asyncio.get_event_loop()
        SERVER = os.environ.get('SERVER', server)
        PORT = os.environ.get('PORT', port)
        NAME = os.environ.get('NAME', name)
       
        loop.run_until_complete(self.agent_loop("{}:{}".format(SERVER,PORT), NAME))
    
    def move(self,pacman, pos, width, height):
        if(pacman.x == 0 and pos[0] == width-1):
            key = "a"
        elif (pacman.x == width-1 and pos[0] == 0):
            key = "d"
        elif (pacman.y == height-1 and pos[1] == 0):
            key = "s"
        elif (pacman.y == 0 and pos[1] == height-1):
            key = "w"
        elif (pos[0] > pacman.x):
            key = "d"
        elif (pos[0] < pacman.x):
            key = "a"
        elif(pos[1] > pacman.y):
            key = "s"
        elif(pos[1] < pacman.y):
            key = "w"
        return key

    async def agent_loop(self, server_address = "localhost:8000", agent_name="student"):
        async with websockets.connect("ws://{}/player".format(server_address)) as websocket:

            # Receive information about static game properties 
            await websocket.send(json.dumps({"cmd": "join", "name": agent_name}))
            msg = await websocket.recv()
            game_properties = json.loads(msg) 
             
            map = Map(game_properties['map'])
            width, height = map.size

            while True: 
                r = await websocket.recv()
                state = json.loads(r)   # receive game state
                if not state['lives']:  # if there're no more lifes left
                    print("GAME OVER")
                    return

                if not state['energy'] and not state['boost']:  # if there'ra no more energy and boost left
                    print ("WIN")
                    print("Lives: {}" .format(state['lives']))
                    print("Score: {}" .format(state['score']))
                    return
            
                agent = AStar(map)
                agent.update_status(state['energy'],state['ghosts'],state['boost']) # get newest information ghost, energy and boost
                
                start = Cell(state['pacman'][0],state['pacman'][1],True, False)    # initial position of the pacman

                walls = agent.walls_to_array()
                agent.init_grid(width, height, walls) # uses walls and ghosts as walls
                agent.set_start(start)

                l = list()
                search_path = True
                while(search_path):
                    temp = agent.get_closest_point(start,l)
                    if (temp == None):
                        end = Cell(l[0][0],l[0][1],True,False)
                        search_path = False
                    else:
                        end = Cell(temp[0],temp[1],True,False)
                        agent.set_end(end)
                        agent.solve()
                        path = agent.get_path()
                        if (len(path) > 1 and agent.is_ghost_in_path(path,3) ):
                            l.append(temp)
                        else:
                            search_path = False

                await websocket.send(json.dumps({"cmd": "key", "key": self.move(start,path[1],width,height)}))   # send new key     

s = Student('localhost','8000','student')
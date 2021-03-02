# This is necessary to find the main code
import sys
from queue import PriorityQueue

sys.path.insert(0, '../bomberman')
# Import necessary stuff
from entity import CharacterEntity
from colorama import Fore, Back


class TestCharacter(CharacterEntity):

    def do(self, wrld):
        # scan whole board for any monsters or other players
        monsters = 0
        players = 0
        path = dict()
        # get player location
        goal = (wrld.exitcell[0], wrld.exitcell[1])  # this could be empty
        p = wrld.me(self)

        # if wrld.monsters():  # no monsters in feild, so just A* it
        path = self.Astar(wrld, (p.x, p.y), goal) # traverse as linked list?
        fpath = [goal]
        while not (p.x, p.y) in fpath:
            fpath.append(path.get(fpath[-1]))
        # get first move of path and make the move
        fpath.reverse()
        print(fpath)

        # make the move!!!!
        move = (fpath[1][0]-fpath[0][0] ,fpath[1][1]-fpath[0][1])
        print(move)
        self.move(move[0],move[1])

    def getDistanceTo(self, cur, goal):
        return abs(cur[0] - goal[0]) + abs(cur[1] - goal[1])

    def getPlayerNeighbors(self, startcoords, wrld):
        # Go through the possible 8-moves of the monster
        coords = []
        # Loop through delta x
        for dx in [-1, 0, 1]:
            # Avoid out-of-bound indexing
            if (startcoords[0] + dx >= 0) and (startcoords[0] + dx < wrld.width()):
                # Loop through delta y
                for dy in [-1, 0, 1]:
                    # Make sure the monster is moving
                    if (dx != 0) or (dy != 0):
                        # Avoid out-of-bound indexing
                        if (startcoords[1] + dy >= 0) and (startcoords[1] + dy < wrld.height()):
                            # No need to check impossible moves
                            if not wrld.wall_at(startcoords[0] + dx, startcoords[1] + dy):
                                # make a list of moves or make a new world with each move?
                                coords.append((startcoords[0] + dx, startcoords[1] + dy))
                                # Set move in wrld
                                # entity.move(dx, dy)
                                # Get new world
                                # (newwrld, events) = wrld.next()
                                # TODO: do something with newworld and events
        return coords

    def Astar(self, wrld, start, goal):  # start and goal are (x,y) tuples
        frontier = PriorityQueue()
        frontier.put(start, 0)
        came_from = dict()
        cost_so_far = dict()
        #came_from[start] = None
        cost_so_far[start] = 0

        while not frontier.empty():
            current = frontier.get()

            if current == goal:
                print(came_from)
                return came_from

            for next in self.getPlayerNeighbors(current,wrld):

                self.set_cell_color(next[0], next[1], Fore.CYAN)
                new_cost = cost_so_far[current] + 1  # graph.cost(current, next) #change this to use bomb maybe
                if next not in cost_so_far or new_cost < cost_so_far[next]:
                    cost_so_far[next] = new_cost
                    priority = new_cost + self.getDistanceTo(next, goal)
                    frontier.put(next, priority)
                    self.set_cell_color(next[0], next[1], Fore.MAGENTA)
                    came_from[next] = current
        print(came_from)
        print("Couldn't Plan Path to Goal")

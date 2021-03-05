# This is necessary to find the main code
import sys
from statemachine import StateMachine, State
from queue import PriorityQueue

sys.path.insert(0, '../bomberman')
# Import necessary stuff
from entity import CharacterEntity
from colorama import Fore, Back


class BombermanSM(StateMachine):
    idle = State('idle', initial=True)
    walk = State('walk')
    bomb = State('bomb') 

    # Idle is detection state
    idleToWalk = idle.to(walk)
    idleToBomb = idle.to(bomb)
    bombToWalk = bomb.to(walk)
    walkToIdle = walk.to(idle)

class TestCharacter(CharacterEntity):

    sm = BombermanSM()
    def do(self, wrld):
        # scan whole board for any monsters or other players
        monsters = 0
        players = 0
        path = dict()

        # get player location
        p = wrld.me(self)
        # check if a bomb has been placed
        if wrld.bomb_at(p.x, p.y):
            coords = []
            # Loop through delta x
            for dx in [-1, 1]:
                # Avoid out-of-bound indexing
                if (p.x + dx >= 0) and (p.x + dx < wrld.width()):
                    # Loop through delta y
                    for dy in [-1, 1]:
                        # Avoid out-of-bound indexing
                        if (p.y + dy >= 0) and (p.y + dy < wrld.height()):
                            # No need to check impossible moves
                            if not wrld.wall_at(p.x + dx, p.y + dy):  # allow walls spots
                                # make a list of moves or make a new world with each move?
                                # coords.append(p.x + dx, p.y + dy)
                                self.move(dx, dy) # take evasive action



        goal = (wrld.exitcell[0], wrld.exitcell[1])  # this could be empty

        # if wrld.monsters():  # no monsters in feild, so just A* it
        path = self.Astar(wrld, (p.x, p.y), goal)  # traverse as linked list?
        fpath = [goal]
        print(fpath)
        while not (p.x, p.y) in fpath:
            fpath.append(path.get(fpath[-1]))
        # get first move of path and make the move
        fpath.reverse()
        print(fpath)
        #check path for problems
        for cell in fpath:
            if wrld.bomb_at(cell[0], cell[1]) or wrld.explosion_at(cell[0], cell[1]):
                self.move(0, 0)



        #otherwise make the move!!!!
        move = (fpath[1][0] - fpath[0][0], fpath[1][1] - fpath[0][1])
        #print(fpath[1][0], fpath[1][1])

        # check if position to move to is a wall
        if (wrld.wall_at(fpath[1][0], fpath[1][1])):
            print("Wall here bitch")  # take evasive action
            self.place_bomb()


        self.move(move[0], move[1])

    def getDistanceTo(self, cur, goal):
        return abs(cur[0] - goal[0]) + abs(cur[1] - goal[1])

    def getPlayerNeighbors(self, startcoords, wrld, allowWalls):
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
                            if allowWalls or not wrld.wall_at(startcoords[0] + dx,
                                                              startcoords[1] + dy):  # allow walls spots
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
        # came_from[start] = None
        cost_so_far[start] = 0

        while not frontier.empty():
            current = frontier.get()

            if current == goal:
                print(came_from)
                return came_from

            for next in self.getPlayerNeighbors(current, wrld, True):

                self.set_cell_color(next[0], next[1], Fore.CYAN)
                wallcost = 1

                if wrld.wall_at(next[0], next[1]):
                    wallcost += 100

                new_cost = cost_so_far[current] + wallcost  # graph.cost(current, next) #change this to use bomb maybe
                if next not in cost_so_far or new_cost < cost_so_far[next]:
                    cost_so_far[next] = new_cost
                    priority = new_cost + self.getDistanceTo(next, goal)
                    frontier.put(next, priority)
                    self.set_cell_color(next[0], next[1], Fore.MAGENTA)
                    came_from[next] = current

        print(came_from)
        print("Couldn't Plan Path to Goal")

    # Getting expectimax
    def expectimax(node, is_max):
        # Condition for Terminal node
        if (node.left == None and node.right == None):
            return node.value;

        # Maximizer node. Chooses the max from the
        # left and right sub-trees
        if (is_max):
            return max(expectimax(node.left, False), expectimax(node.right, False))

        # Chance node. Returns the average of
        # the left and right sub-trees
        else:
            return (expectimax(node.left, True) + expectimax(node.right, True)) / 2;

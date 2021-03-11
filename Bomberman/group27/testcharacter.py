# This is necessary to find the main code
import math
from copy import deepcopy

from colorama import Fore, Back
from entity import CharacterEntity
from events import *
import sys
from statemachine import StateMachine, State
from queue import PriorityQueue
import itertools

from sensed_world import SensedWorld

sys.path.insert(0, '../bomberman')


# Import necessary stuff

class BombermanSM(StateMachine):
    walk = State('walk', initial=True)
    bomb = State('bomb')
    idleBomb = State('idleBomb')
    dodge = State('dodge')
    # Idle is detection state
    walkToBomb = walk.to(bomb)
    bombToDodge = bomb.to(dodge)
    walkToIdleBomb = walk.to(idleBomb)
    dodgeToIdleBomb = dodge.to(idleBomb)


class TestCharacter(CharacterEntity):
    def do(self, wrld):
        p = wrld.me(self)  # get player location
        sm = BombermanSM()

        if sm.current_state == BombermanSM.walk:
            goal = (wrld.exitcell[0], wrld.exitcell[1])  # this could be empty

            path = self.Astar(wrld, (p.x, p.y), goal)
            fpath = [goal]
            while not (p.x, p.y) in fpath:
                fpath.append(path.get(fpath[-1]))
            fpath.reverse()

            waitingForBomb = False
            for cell in fpath:  # check if the path made is safe
                if wrld.bomb_at(cell[0], cell[1]) or wrld.explosion_at(cell[0], cell[1]):
                    waitingForBomb = True
                    self.move(0, 0)  # if not, just wait

            if not waitingForBomb:  # if path is safe
                move = (fpath[1][0] - fpath[0][0], fpath[1][1] - fpath[0][1])

                if wrld.wall_at(fpath[1][0], fpath[1][1]):
                    sm.walkToBomb()
                
                #bomb if exit below you
                # dist = abs(p.x - wrld.exitcell[0])
                # if dist < 3: sm.walkToBomb()
                

                else:
                    scanRange = 3  # maybe change to 4 and use expectimax more
                    # we are attempting to move towards goal, check if we would be within range of monster
                    danger = False
                    for dx in range(-scanRange, scanRange + 1):
                        # Avoid out-of-bounds access
                        if (p.x + dx >= 0) and (p.x + dx < wrld.width()):
                            for dy in range(-scanRange, scanRange + 1):
                                # Avoid out-of-bounds access
                                if (p.y + dy >= 0) and (p.y + dy < wrld.height()):
                                    # Is a monster at this position?
                                    if wrld.monsters_at(p.x + dx, p.y + dy):
                                        danger = True
                                        # dist = abs(p.x - p.x + dx) + abs(p.y - p.y + dy)
                                        # if dist <= 2: sm.walkToBomb()
                                        # we gotta bolt, would be detected
                                        # call expectimax to find our best move

                    if danger:
                        # check new worlds for 3 layers in advance
                        depth = 3
                        root = Node.newNode(self, wrld, [])
                        root = Node.initExpectimax(self, depth, root, [])

                        # return move of best expectimax
                        move = Node.expectimax(self, root, True)

                        move = move[0][0][0]


                    #print(move)
                    self.move(move[0], move[1])  # execute move

        if sm.current_state == BombermanSM.bomb:
            # check if position to move to is a wall
            print("Wall here bitch")  # take evasive action
            self.place_bomb()
            sm.bombToDodge()

        if sm.current_state == BombermanSM.dodge:
            # check if a bomb has been placed
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
                                self.move(dx, dy)  # take evasive action
                                # sm.dodgeToIdleBomb()

        #if sm.current_state == BombermanSM.idleBomb:

    @staticmethod
    def getDistanceTo( cur, goal):
        return abs(cur[0] - goal[0]) + abs(cur[1] - goal[1])

    @staticmethod
    def getPlayerNeighbors( startCoords, wrld, allowWalls):
        # Go through the possible 8-moves of the monster
        coords = []
        # Loop through delta x
        for dx in [-1, 0, 1]:
            # Avoid out-of-bound indexing
            if (startCoords[0] + dx >= 0) and (startCoords[0] + dx < wrld.width()):
                # Loop through delta y
                for dy in [-1, 0, 1]:
                    # Make sure the monster is moving
                    if (dx != 0) or (dy != 0):
                        # Avoid out-of-bound indexing
                        if (startCoords[1] + dy >= 0) and (startCoords[1] + dy < wrld.height()):
                            # No need to check impossible moves
                            if allowWalls or not wrld.wall_at(startCoords[0] + dx, startCoords[1] + dy):  # allow walls spots
                                # make a list of moves or make a new world with each move?
                                coords.append((startCoords[0] + dx, startCoords[1] + dy))

        return coords

    @staticmethod
    def Astar(wrld, start, goal):  # start and goal are (x,y) tuples
        frontier = PriorityQueue()
        frontier.put(start, 0)
        came_from = dict()
        cost_so_far = dict()
        # came_from[start] = None
        cost_so_far[start] = 0

        while not frontier.empty():
            current = frontier.get()

            if current == goal:
                # print(came_from)
                return came_from

            for next in TestCharacter.getPlayerNeighbors(current, wrld, True):

                #self.set_cell_color(next[0], next[1], Fore.CYAN)
                wallcost = 1

                if wrld.wall_at(next[0], next[1]):
                    wallcost += 100

                # graph.cost(current, next) #change this to use bomb maybe
                new_cost = cost_so_far[current] + wallcost
                if next not in cost_so_far or new_cost < cost_so_far[next]:
                    cost_so_far[next] = new_cost
                    priority = new_cost + TestCharacter.getDistanceTo(next, goal)
                    frontier.put(next, priority)
                    #self.set_cell_color(next[0], next[1], Fore.MAGENTA)
                    came_from[next] = current

        # print(came_from)
        # print("Couldn't Plan Path to Goal")


class Node:

    def __init__(self, world, path):
        self.world = world
        self.path = path
        self.score = None
        self.children = []

    # Initializing Nodes to None
    def newNode(self, world, path):
        temp = Node(world, path)
        return temp

    # Getting expectimax
    def expectimax(self, node, is_max):
        # Condition for Terminal node
        if not node.children:  # if there is nothing in the child list
            return  node.score

        # Maximizer node. Chooses the max from the children
        if (is_max):
            #print(node.children)
            expectichildren = []
            for c in node.children:
                boi = Node.expectimax(self, c, False)

                expectichildren.append(boi)

            print(expectichildren)
            #maxset = max(expectichildren, key = lambda i : i[1])[0]
            print(max(expectichildren, key = lambda i : i[1]))
            return max(expectichildren, key = lambda i : i[1])

        # Chance node. Returns the average of
        # the left and right sub-trees
        else:
            totalSum = 0
            for child in node.children:
                score = Node.expectimax(self, child, True)
                totalSum+=score[1]
            return node.path, (totalSum / len(node.children))

    def initExpectimax(self, depth, root, events):
        p = root.world.me(self)
        # if p is None:
        #     root.score = tuple((root.path, -1000))
        #     root.children = []
        #     return root
        if depth != 1:  # if this isnt the bottom level, create list of children
            copywrld = SensedWorld.from_world(root.world)
            # calculate all monster money moves
            possiblemonstermoves = dict()

            for m in copywrld.monsters.values():  # get closest monsters position
                #print(m)
                # xdistance = math.fabs(p.x - m[0].x)#check how far we are from monster
                # ydistance = math.fabs(p.y - m[0].y)
                #distance = math.sqrt((p.x - m[0].x)**2+ (p.y - m[0].y)**2)
                calcpath = TestCharacter.Astar(root.world, (p.x, p.y), (m[0].x,m[0].y))
                fpath = [(m[0].x,m[0].y)]
                while not (p.x, p.y) in fpath:
                    fpath.append(calcpath.get(fpath[-1]))
                fpath.reverse()
                if len(fpath) < 4:
                    possiblemonstermoves[m[0]] = []  # create new entry for monster
                    for dx in [-1, 0, 1]:
                        # Avoid out-of-bound indexing
                        if (m[0].x + dx >= 0) and (m[0].x + dx < copywrld.width()):
                            # Loop through delta y
                            for dy in [-1, 0, 1]:
                                # Make sure the monster is moving
                                if (dx != 0) or (dy != 0):
                                    # Avoid out-of-bound indexing
                                    if (m[0].y + dy >= 0) and (m[0].y + dy < copywrld.height()):
                                        # No need to check impossible moves
                                        if not copywrld.wall_at(m[0].x + dx, m[0].y + dy):
                                            possiblemonstermoves[m[0]].append((dx, dy))
                else:
                    possiblemonstermoves[m[0]] = [(0,0)]  # if they are far away, just make them stationary


            # args = tuple(possiblemonstermoves.values())
            bigboi = list(itertools.product(*possiblemonstermoves.values()))

            for pdx in [-1, 0, 1]:  # check each possible player move
                # Avoid out-of-bound indexing
                if (p.x + pdx >= 0) and (p.x + pdx < copywrld.width()):
                    # Loop through delta y
                    for pdy in [-1, 0, 1]:
                        # Make sure the monster is moving
                        if (pdx != 0) or (pdy != 0):
                            # Avoid out-of-bound indexing
                            if (p.y + pdy >= 0) and (p.y + pdy < copywrld.height()):
                                # No need to check impossible moves
                                if not copywrld.wall_at(p.x + pdx, p.y + pdy):  #####
                                    CharacterEntity.move(self,pdx, pdy)  # apply player move
                                    # now apply all different monster moves
                                    for mo in bigboi:
                                        i = 0
                                        for m in copywrld.monsters.values():
                                            # Set move in wrld
                                            m[0].move(mo[i][0], mo[i][1])
                                            i += 1
                                        # Get new world
                                        (newWrld, events) = copywrld.next()  # get new world with moved entities
                                        death = False
                                        for e in events:
                                            if e.tpe == Event.CHARACTER_KILLED_BY_MONSTER:
                                                death = True
                                                newPath = deepcopy(root.path)
                                                newPath.append([(pdx, pdy)])
                                                badNode = Node.newNode(self, newWrld, [])
                                                badNode.score = tuple((newPath,0))
                                                root.children.append(badNode)

                                        if not death:
                                            newPath = deepcopy(root.path)
                                            newPath.append([(pdx, pdy)])
                                            root.children.append(Node.initExpectimax(self, depth - 1, Node.newNode(self, newWrld, newPath), events))

            return root # TODO something, maybe copy world more
        else:  # is bottom level, we need to evaluate each current level node
            score = 10000
            for e in events:
                if e.tpe == Event.CHARACTER_FOUND_EXIT:
                    score += 1000
                elif e.tpe == Event.CHARACTER_KILLED_BY_MONSTER:
                    score -= 1000
                    root.score = tuple((root.path, score))
                    return root
                elif e.tpe == Event.BOMB_HIT_CHARACTER:
                    score -= 1000
                    root.score = tuple((root.path, score))
                    return root
                elif e.tpe == Event.BOMB_HIT_MONSTER:
                    score += 100
                elif e.tpe == Event.BOMB_HIT_WALL:
                    score += 50

            goal = (root.world.exitcell[0], root.world.exitcell[1])  # this could be empty

            calcpath = TestCharacter.Astar(root.world, (p.x, p.y), goal)

            fpath = [goal]
            while not (p.x, p.y) in fpath:
                fpath.append(calcpath.get(fpath[-1]))
            fpath.reverse()
            print(fpath)
            #print(len(fpath))
            score -= 9 * len(fpath)  # penalize for longer paths
            print(root.path[0])
            if root.path[0][0][0] == 1:
                score += 9

            if root.path[0][0][1] == 1:
                score += 9

            for m in root.world.monsters.values():  # check how far we are from each monster
                if m[0].name == "stupid":
                    scanRange = 2
                elif m[0].name == "aggressive":
                    scanRange = 3
                elif m[0].name == "selfpreserving":
                    scanRange = 2

                # xdistance = math.fabs(p.x - m[0].x)
                # ydistance = math.fabs(p.y - m[0].y)
                #
                # if xdistance <= scanRange:
                #     score -= 10 * (scanRange-xdistance)
                # if ydistance <= scanRange:
                #     score -= 10 * (scanRange-ydistance)

                monsterpath = TestCharacter.Astar(root.world, (p.x, p.y), (m[0].x, m[0].y))
                mpath = [(m[0].x, m[0].y)]
                while not (p.x, p.y) in mpath:
                    mpath.append(monsterpath.get(mpath[-1]))
                mpath.reverse()
                if len(mpath) < scanRange:
                    score -= 10**(len(mpath))


            root.score = tuple((root.path, score))
            return root

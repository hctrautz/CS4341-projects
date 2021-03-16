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

#TODO 
#TODO Improve dodging state enemy detection, he mostly just idles 
#TODO 
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
    bombToWalk = bomb.to(walk)
    dodgeToWalk = dodge.to(walk)

class TestCharacter(CharacterEntity):
    def do(self, wrld):
        b = wrld.bombs.values()
        e = wrld.explosions.values()
        print(b)
        print(e)
        p = wrld.me(self)  # get player location
        sm = BombermanSM()

        if sm.current_state == BombermanSM.walk:
            goal = (wrld.exitcell[0], wrld.exitcell[1])  # this could be empty

            path = self.Astar(wrld, (p.x, p.y), goal)
            fpath = [goal]
            stuck = False;
            while not (p.x, p.y) in fpath:
                if not stuck:
                    try:
                        fpath.append(path.get(fpath[-1]))
                    except:
                        stuck = True
                        #sm.walkToBomb()
                        self.move(0,0)
                        break
            if not stuck:
                fpath.reverse()

                # waitingForBomb = False
                # for cell in fpath:  # check if the path made is safe
                #     if wrld.bomb_at(cell[0], cell[1]) or wrld.explosion_at(cell[0], cell[1]):
                #         waitingForBomb = True
                #         self.move(0, 0)  # if not, just wait

                # if not waitingForBomb:  # if path is safe
                #     move = (fpath[1][0] - fpath[0][0], fpath[1][1] - fpath[0][1])

                if wrld.wall_at(fpath[1][0], fpath[1][1]) and len(b) == 0:
                    sm.walkToBomb()

                if (p.x+1 in range(wrld.width())) and (p.y+1 in range(wrld.height())):
                    if (wrld.wall_at(p.x, p.y+1) or wrld.wall_at(p.x+1, p.y+1) or wrld.wall_at(p.x+1, p.y)) and len(b) == 0 and sm.current_state == BombermanSM.walk:
                        sm.walkToBomb()
                    
                for dx in range (1, wrld.width()):
                    # Avoid out-of-bound indexing
                    if (p.x + dx >= 0) and (p.x + dx < wrld.width()):
                    # Loop through delta y
                        for dy in range (1, wrld.height()):
                            # Avoid out-of-bound indexing
                            if  (p.x+1 in range(wrld.width())) and (p.y+1 in range(wrld.height())) and (p.y + dy >= 0) and (p.y + dy < wrld.height()):
                                if (wrld.exit_at(p.x, p.y + dy) or wrld.exit_at(p.x+1, p.y + dy)) and wrld.wall_at(p.x, p.y+1):
                                    if sm.current_state == BombermanSM.walk and len(b) == 0:
                                        sm.walkToBomb()
                else:
                    scanRange = 1
                    danger = False
                    for m in wrld.monsters.values():  # check how far we are from each monster
                        if m[0].name == "stupid":
                            scanRange = 2
                        if m[0].name == "aggressive":
                            scanRange = 4
                        if m[0].name == "selfpreserving":
                            scanRange = 3
                        # we are attempting to move towards goal, check if we would be within range of monster
                        xdistance = math.fabs(p.x - m[0].x)
                        ydistance = math.fabs(p.y - m[0].y)

                        if ydistance < 3:
                            if sm.current_state == BombermanSM.walk and len(b) == 0:
                                sm.walkToBomb()
                        elif m[0].x - scanRange <= p.x <= m[0].x + scanRange and  m[0].y - scanRange <= p.y <= m[0].y + scanRange:
                            danger = True
                        

                    if danger:
                        # check new worlds for 3 layers in advance
                        depth = 4
                        root = Node.newNode(SensedWorld.from_world(wrld), [])
                        root = Node.initExpectimax(self, depth, root, [])

                        # return move of best expectimax
                        move = Node.expectimax(root, True)
                        print(move)
                        move = move[0]

                    if not danger:
                        # path = self.Astar(wrld, (p.x, p.y), goal)
                        # fpath = [goal]
                        # while not (p.x, p.y) in fpath:
                        #     fpath.append(path.get(fpath[-1]))
                        # fpath.reverse()
                        move = (fpath[1][0] - fpath[0][0], fpath[1][1] - fpath[0][1])
                    # print(move)
                    
                    self.move(move[0], move[1])  # execute move

        if sm.current_state == BombermanSM.bomb:
            # check if position to move to is a wall
            print("Wall here bitch")  # take evasive action
            # TODO: Start Timer, make explosive cells movable until explosion_duration has elapsed 
            self.place_bomb()
            sm.bombToDodge()

        if sm.current_state == BombermanSM.dodge:
            # check if a bomb has been placed
            # Loop through delta x
            dodged = False
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
                                if not dodged:
                                    self.move(dx, dy)  # take evasive action
                                    dodged = True
                                    sm.dodgeToWalk()

    @staticmethod
    def getDistanceTo(cur, goal):
        return abs(cur[0] - goal[0]) + abs(cur[1] - goal[1])

    @staticmethod
    def getPlayerNeighbors(startCoords, wrld, allowWalls):
        # Go through the possible 8-moves
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
                            if (allowWalls or not wrld.wall_at(startCoords[0] + dx,
                                                              startCoords[1] + dy)) and (not wrld.bomb_at(startCoords[0] + dx,
                                                              startCoords[1] + dy) and not wrld.explosion_at(startCoords[0] + dx,
                                                              startCoords[1] + dy)): # allow walls spots
                                # make a list of moves or make a new world with each move?
                                coords.append((startCoords[0] + dx, startCoords[1] + dy))
        #print(coords)
        for b in wrld.bombs.values():
            for bx in range (-wrld.expl_range, wrld.expl_range+1):
                for by in range (-wrld.expl_range, wrld.expl_range+1):
                    if bx == 0 or by == 0:
                        position = (b.x + bx, b.y + by)
                        #print(position)
                        try: 
                            coords.remove(position)
                        except ValueError:
                            pass
        #print(coords)
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

                wallcost = 1

                if wrld.wall_at(next[0], next[1]):
                    wallcost += 100
                bombcost = 1
                if wrld.bomb_at(next[0], next[1]):
                    bombcost += 200
                if wrld.explosion_at(next[0], next[1]):
                    bombcost += 200


                # graph.cost(current, next) #change this to use bomb maybe
                new_cost = cost_so_far[current] + wallcost + bombcost
                if next not in cost_so_far or new_cost < cost_so_far[next]:
                    cost_so_far[next] = new_cost
                    priority = new_cost + TestCharacter.getDistanceTo(next, goal)
                    frontier.put(next, priority)
                    # self.set_cell_color(next[0], next[1], Fore.MAGENTA)
                    came_from[next] = current


class Node:

    def __init__(self, world, path):
        self.world = world
        self.path = path
        self.score = None
        self.children = []

    # Initializing Nodes to None
    @staticmethod
    def newNode(world, path):
        temp = Node(world, path)
        return temp

    # Getting expectimax
    @staticmethod
    def expectimax(node, is_max):
        # Condition for Terminal node
        if not node.children:  # if there is nothing in the child list
            return node.score

        # Maximizer node. Chooses the max from the children
        if is_max:
            expectichildren = []
            for c in node.children:
                boi = Node.expectimax(c, False)
                expectichildren.append(boi)
            # print(expectichildren)
            # print(max(expectichildren, key=lambda i: i[1]))
            return max(expectichildren, key=lambda i: i[1])

        # Chance node. Returns the average of
        # the left and right sub-trees
        else:
            totalSum = 0
            for child in node.children:
                score = Node.expectimax(child, True)
                totalSum += score[1]
            return node.path, (totalSum / len(node.children))

    @staticmethod
    def initExpectimax(ent, depth, root, events):
        p = root.world.me(ent)
        print(len(root.world.characters.values()))
        # if len(root.world.characters.values()) == 0:
        #     # newPath = deepcopy(root.path)
        #     # badNode = Node.newNode(root.world, [])
        #     # badNode.score = tuple((root.path, -1000))
        #     print("did we get here")
        #     root.score = tuple((root.path, -1000))
        #     return root
        try:
            p.x
        except:
            root.score = tuple((root.path, -1000))
            stuck = True
            return root
        if depth != 1:  # if this isnt the bottom level, create list of children
            # copywrld = SensedWorld.from_world(root.world)
            # calculate all monster money moves
            possiblemonstermoves = dict()

            for m in root.world.monsters.values(): #loop through the monsters, and create moves for each
                if m[0].name == "stupid":
                    scanRange = 2
                if m[0].name == "aggressive":
                    scanRange = 3
                if m[0].name == "selfpreserving":
                    scanRange = 2

                calcpath = TestCharacter.Astar(root.world, (m[0].x, m[0].y), (p.x, p.y))
                fpath = [(p.x, p.y)]
                stuck = False
                while not (m[0].x, m[0].y) in fpath:
                    if not stuck:
                        try:
                            fpath.append(calcpath.get(fpath[-1]))
                        except:
                            root.score = tuple((root.path, -1000))
                            stuck = True
                            return root

                fpath.reverse()

                if len(fpath) <= scanRange+1:
                    possiblemonstermoves[m[0]] = []  # create new entry for monster
                    # for dx in [-1, 0, 1]:
                    #     # Avoid out-of-bound indexing
                    #     if (m[0].x + dx >= 0) and (m[0].x + dx < root.world.width()):
                    #         # Loop through delta y
                    #         for dy in [-1, 0, 1]:
                    #             # Make sure the monster is moving
                    #             if (dx != 0) or (dy != 0):
                    #                 # Avoid out-of-bound indexing
                    #                 if (m[0].y + dy >= 0) and (m[0].y + dy < root.world.height()):
                    #                     # No need to check impossible moves
                    #                     if not root.world.wall_at(m[0].x + dx, m[0].y + dy):
                    #                         possiblemonstermoves[m[0]].append((dx, dy))

                    for dx in range(-scanRange, scanRange + 1):
                        # Avoid out-of-bounds access
                        if (m[0].x + dx >= 0) and (m[0].x + dx < root.world.width()):
                            for dy in range(-scanRange, scanRange + 1):
                                # Avoid out-of-bounds access
                                if (m[0].y + dy >= 0) and (m[0].y + dy < root.world.height()):
                                    # Is a character at this position?
                                    if root.world.characters_at(m[0].x + dx, m[0].y + dy):
                                        possiblemonstermoves[m[0]].append((dx, dy))

                else:
                    possiblemonstermoves[m[0]] = [(0, 0)]  # if they are far away, just make them stationary

            # args = tuple(possiblemonstermoves.values())
            bigboi = list(itertools.product(*possiblemonstermoves.values()))

            for pdx in [-1, 0, 1]:  # check each possible player move
                # Avoid out-of-bound indexing
                if (p.x + pdx >= 0) and (p.x + pdx < root.world.width()):
                    # Loop through delta y
                    for pdy in [-1, 0, 1]:
                        # Make sure the monster is moving
                        # TODO Allow no moving
                        # Avoid out-of-bound indexing
                        if (p.y + pdy >= 0) and (p.y + pdy < root.world.height()):
                            # No need to check impossible moves

                            if not root.world.wall_at(p.x + pdx, p.y + pdy):  #####
                                root.world.me(p).move(pdx, pdy) #apply player move
                                if not root.path:
                                    newPath = (pdx, pdy)
                                newNode = Node.newNode(SensedWorld.from_world(root.world), newPath)

                                # now apply all different monster moves
                                for mo in bigboi:
                                    i = 0
                                    for m in root.world.monsters.values():
                                        # Set move in wrld

                                        m[0].move(mo[i][0], mo[i][1])
                                        i += 1
                                    # Get new world
                                    (newWrld, events) = root.world.next()  # get new world with moved entities
                                    newNode.children.append(Node.initExpectimax(ent, depth - 1, Node.newNode(newWrld, root.path),events))
                                root.children.append(newNode)

            return root  # TODO something, maybe copy world more
        else:  # is bottom level, we need to evaluate each current level node
            score = 0
            for e in events:
                if e.tpe == Event.CHARACTER_FOUND_EXIT:
                    score += 5000
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
            score -= 5 * len(fpath)  # penalize for longer paths

            #give 1 point for every free space in the next world, this should discourage us from running into walls
            score += 2 * len(TestCharacter.getPlayerNeighbors((p.x, p.y), root.world, False))

            #score -= 2 * (root.world.exitcell[0] - p.x)

            # print(root.path[0])
            # if root.path[0][0][0] == 1 or root.path[0][0][0] == -1:
            #     score += 8
            #
            # if root.path[0][0][0] == 0:
            #     score += 3
            #
            # if root.path[0][0][1] == -1:
            #     score -= 5
            #
            # if root.path[0][0][1] == 0:
            #     score += 3
            #
            # if root.path[0][0][1] == 1:
            #     score += 9

            for m in root.world.monsters.values():  # check how far we are from each monster
                if m[0].name == "stupid":
                    scanRange = 2
                elif m[0].name == "aggressive":
                    scanRange = 4
                elif m[0].name == "selfpreserving":
                    scanRange = 3

                # xdistance = math.fabs(p.x - m[0].x)
                # ydistance = math.fabs(p.y - m[0].y)
                
                # if xdistance >= scanRange:
                #     score += 10 * (scanRange-xdistance)
                # if ydistance >= scanRange:
                #     score += 10 * (scanRange-ydistance)
                if m[0].y < p.y:
                    score += 15

                monsterpath = TestCharacter.Astar(root.world, (p.x, p.y), (m[0].x, m[0].y))
                mpath = [(m[0].x, m[0].y)]
                while not (p.x, p.y) in mpath:
                    try:
                        mpath.append(monsterpath.get(mpath[-1]))
                    except:
                        root.score = tuple((root.path, 1000))
                        return root

                mpath.reverse()
                if len(mpath) <= scanRange:
                    score -= 50 * (scanRange - len(mpath))
                elif len(mpath) > scanRange:
                    score += 50 * (len(mpath)-scanRange)

            root.score = tuple((root.path, score))
            return root

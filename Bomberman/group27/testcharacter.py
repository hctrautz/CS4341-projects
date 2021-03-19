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
    dodge = State('dodge')
    finish = State('finish')
    # Idle is detection state
    walkToBomb = walk.to(bomb)
    bombToDodge = bomb.to(dodge)
    bombToWalk = bomb.to(walk)
    dodgeToWalk = dodge.to(walk)
    walkToFinish = walk.to(finish)


class TestCharacter(CharacterEntity):

    def __init__(self, name, avatar, x, y, targets):
        super().__init__(name, avatar, x, y)
        self.previousPose = (0,0)
        self.stuckCounter = 0
        self.goalQ = targets
        self.blocked = False
        self.decoyGoal = (-69, -69)
        self.depth = 3
        self.chaseLength = 0
        self.bombFreq = 2
        self.resetTrigger = True
        self.TrackedBomb = None

    @staticmethod
    def getDistanceTo(cur, goal):
        return abs(cur[0] - goal[0]) + abs(cur[1] - goal[1])
    
    def do(self, wrld):

        bv = wrld.bombs.values()
        p = wrld.me(self)  # get player location
        sm = BombermanSM()
        self.previousPose = (p.x,p.y)

        while sm.current_state != BombermanSM.finish:
            if not self.goalQ:
                self.goalQ.append((wrld.exitcell[0], wrld.exitcell[1])) # if the Q is empty, add the goal
            if self.getDistanceTo((p.x, p.y), wrld.exitcell) <= 2:
                self.goalQ.clear()
                self.goalQ.append((wrld.exitcell[0], wrld.exitcell[1])) 

            if p.x == self.goalQ[0][0] and p.y == self.goalQ[0][1]: # if bomberman has reached next goal
                if not (p.x == wrld.exitcell[0] and p.y == wrld.exitcell[1]): # if not the actual goal
                    if len(bv) == 0: # check that no bomb has already been placed
                        self.goalQ.pop(0) # remove the goal from the Q
                        sm.walkToBomb() # place bomb at target

            if sm.current_state == BombermanSM.walk: # by default we are walking to the exit

                goal = self.goalQ[0]

                path = self.Astar(wrld, (p.x, p.y), goal, True) # compute path to goal
                fpath = [goal]
                while not (p.x, p.y) in fpath:
                    fpath.append(path.get(fpath[-1]))
                fpath.reverse()

                exploded = False
                tempGoal = self.decoyGoal

                # traverse the path to exit, if there is something in the way, set the decoygoal to the closest point before
                baddieList = [] #reset everyround

                for b in wrld.bombs.values():
                    if b.timer <= 2:  # if its within two turns of exploding, dont allow walking in explosions
                        for bx in range(-wrld.expl_range, wrld.expl_range + 1):
                            for by in range(-wrld.expl_range, wrld.expl_range + 1):
                                if bx == 0 or by == 0:
                                    baddieList.append((b.x + bx, b.y + by))


                if goal[0] in baddieList: # if the goal we are going to is in bomb range, swap with next goal
                    if len(self.goalQ) > 1:
                        tempo = self.goalQ[0]
                        self.goalQ[0] = self.goalQ[1]
                        self.goalQ[1] = tempo

                for cell in fpath[1:]:
                    if wrld.explosion_at(cell[0], cell[1]) or wrld.wall_at(cell[0], cell[1]) or (cell in baddieList):
                        print("couldnt get to real goal")
                        print(goal)
                        path = self.Astar(wrld, (p.x, p.y), self.decoyGoal, False) #reset path to traverse to current decoy cuz we suck
                        fpath = [self.decoyGoal]
                        while not (p.x, p.y) in fpath:
                            fpath.append(path.get(fpath[-1]))
                        fpath.reverse()

                        self.blocked = True
                        break
                    else:
                        tempGoal = (cell[0], cell[1])
                        self.blocked = False

                if not self.blocked: # if nothing is blocking us, continue to next goal
                    goal = self.goalQ[0]

                elif self.resetTrigger: # if we are allowing a new temp goal, and we are blocked
                    self.resetTrigger = False #reset trigger
                    goal = tempGoal #set the goal to the new saftey
                    self.decoyGoal = tempGoal

                    path = self.Astar(wrld, (p.x, p.y), self.decoyGoal, False)  # reset path to traverse to current decoy cuz we suck
                    fpath = [self.decoyGoal]
                    while not (p.x, p.y) in fpath:
                        fpath.append(path.get(fpath[-1]))
                    fpath.reverse()

                else: #else we are blocked and cannot reset, so keep goal as old saftey
                    goal = self.decoyGoal

                move = (-99 ,-99)
                scanRange = 1
                danger = False
                for m in wrld.monsters.values():  # check how far we are from each monster
                    if m[0].name == "stupid":
                        scanRange = 2 + 2
                    if m[0].name == "aggressive":
                        scanRange = 4 + 2
                    if m[0].name == "selfpreserving":
                        scanRange = 2 + 2

                    unreachable = False
                    monsterpath = TestCharacter.Astar(wrld, (m[0].x, m[0].y), (p.x, p.y), False)
                    mpath = [(p.x, p.y)]
                    while not (m[0].x, m[0].y) in mpath:
                        try:
                            mpath.append(monsterpath.get(mpath[-1]))
                        except(AttributeError):
                            unreachable = True
                            break

                    if not unreachable and len(mpath) <= scanRange:
                        danger = True
                        break

                if danger: # if we are in danger, use expectimax
                    self.chaseLength += 1
                    print(goal)
                    root = Node.initExpectimax(self, self.depth, Node.newNode(SensedWorld.from_world(wrld), []), [], goal)
                    result = Node.expectimax(root, True)

                    if self.chaseLength >= self.bombFreq and len(bv)<1:
                        self.chaseLength = 0
                        sm.walkToBomb()
                    else:
                        move = (result[0][0], result[0][1])
                        print("moving away from danger, no bomb")
                        print(move)
                        print("to reach this goal")
                        print(goal)


                else: # if not in danger, follow A*

                    if len(fpath) != 1:
                        move = (fpath[1][0] - p.x, fpath[1][1] - p.y)
                    else:
                        move = (0,0)
                    print("not in danger")
                    print(move)
                    print("to reach this goal")
                    print(fpath)

                    #double check to ensure saftey
                    if wrld.explosion_at(p.x + move[0],p.y + move[1]) or ((p.x + move[0],p.y + move[1]) in baddieList): # if there is an explosion where we are going, naw
                        for dx in [-1, 0, 1]:
                            # Avoid out-of-bound indexing
                            if (p.x + dx >= 0) and (p.x + dx < wrld.width()):
                                # Loop through delta y
                                for dy in [-1,0, 1]:
                                    # Avoid out-of-bound indexing
                                    if (p.y + dy >= 0) and (p.y + dy < wrld.height()):
                                        # No need to check impossible moves
                                        if (not wrld.wall_at(p.x + dx, p.y + dy)) and (not wrld.explosion_at(p.x + dx, p.y + dy)) and (not wrld.monsters_at(p.x + dx, p.y + dy)) and ((p.x + dx, p.y + dy) not in baddieList):
                                            move = (dx, dy)
                                            break
                                else:
                                    continue
                                break



                self.move(move[0], move[1])  # execute move
                if sm.current_state == BombermanSM.walk:
                    sm.walkToFinish()

            if sm.current_state == BombermanSM.bomb:
                self.place_bomb()
                for dx in [-1, 1]:
                    # Avoid out-of-bound indexing
                    if (p.x + dx >= 0) and (p.x + dx < wrld.width()):
                        # Loop through delta y
                        for dy in [-1,1]:
                            # Avoid out-of-bound indexing
                            if (p.y + dy >= 0) and (p.y + dy < wrld.height()):
                                # No need to check impossible moves
                                if not wrld.wall_at(p.x + dx, p.y + dy) and not wrld.explosion_at(p.x + dx, p.y + dy) :  # dont allow walls
                                    self.decoyGoal = (p.x + dx, p.y + dy)
                                    break
                        else:
                            continue
                        break


                print("placed bomb")
                print(self.decoyGoal)
                sm.bombToWalk()

        #put anything here that should trigger after the turn is over
        for b in bv:
            if b.timer == 0: #about to explode next turn to trigger reset
                self.TrackedBomb = (b.x,b.y)

            if (self.TrackedBomb is not None) and wrld.explosion_at(self.TrackedBomb[0], self.TrackedBomb[1]):
                self.resetTrigger = True

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
                            if allowWalls or (not (wrld.wall_at(startCoords[0] + dx, startCoords[1] + dy))):
                                # make a list of moves or make a new world with each move?
                                coords.append((startCoords[0] + dx, startCoords[1] + dy))
        return coords

    @staticmethod
    def Astar(wrld, start, goal, allowWalls):  # start and goal are (x,y) tuples
        frontier = PriorityQueue()
        frontier.put((0, start))
        came_from = dict()
        cost_so_far = dict()
        cost_so_far[start] = 0

        while not frontier.empty():
            current = frontier.get()

            if current[1] == goal:

                return came_from

            for next in TestCharacter.getPlayerNeighbors(current[1], wrld, allowWalls):

                wallcost = 1

                if wrld.wall_at(next[0], next[1]):
                    wallcost += 100
                if wrld.explosion_at(next[0], next[1]):
                    wallcost += 1000

                # graph.cost(current, next) #change this to use bomb maybe
                new_cost = cost_so_far[current[1]] + wallcost
                if next not in cost_so_far or new_cost < cost_so_far[next]:
                    cost_so_far[next] = new_cost
                    priority = new_cost + TestCharacter.getDistanceTo(next, goal)
                    frontier.put((priority, next))
                    # self.set_cell_color(next[0], next[1], Fore.MAGENTA)
                    came_from[next] = current[1]


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
    def initExpectimax(ent, depth, root, events, pathgoal):
        p = root.world.me(ent)
        try:
            p.x
        except(AttributeError):
            root.score = tuple((root.path, -1000))
            return root
        if depth != 1:  # if this isnt the bottom level, create list of children
            possiblemonstermoves = dict()
            unreachable = False
            for m in root.world.monsters.values(): # calculate all monster money moves
                if m[0].name == "stupid":
                    scanRange = 2 + 2
                if m[0].name == "aggressive":
                    scanRange = 3 + 2
                if m[0].name == "selfpreserving":
                    scanRange = 2 + 2

                calcpath = TestCharacter.Astar(root.world, (m[0].x, m[0].y), (p.x, p.y), False)
                fpath = [(p.x, p.y)]
                stuck = False
                while not (m[0].x, m[0].y) in fpath:
                    if not stuck:
                        try:
                            fpath.append(calcpath.get(fpath[-1]))
                        except(AttributeError):
                            unreachable = True
                            break

                if unreachable:
                    possiblemonstermoves[m[0]] = [(0, 0)]
                elif len(fpath) <= scanRange:
                    possiblemonstermoves[m[0]] = []  # create new entry for monster
                    if m[0].name == "stupid":
                        for dx in [-1, 0, 1]:
                            # Avoid out-of-bound indexing
                            if (m[0].x + dx >= 0) and (m[0].x + dx < root.world.width()):
                                # Loop through delta y
                                for dy in [-1, 0, 1]:
                                    # Make sure the monster is moving
                                    if (dx != 0) or (dy != 0):
                                        # Avoid out-of-bound indexing
                                        if (m[0].y + dy >= 0) and (m[0].y + dy < root.world.height()):
                                            # No need to check impossible moves
                                            if not root.world.wall_at(m[0].x + dx, m[0].y + dy):
                                                possiblemonstermoves[m[0]].append((dx, dy))
                    else: #use minimax
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

            bigboi = list(itertools.product(*possiblemonstermoves.values()))

            for pdx in [-1, 0, 1]:  # check each possible player move
                # Avoid out-of-bound indexing
                if (p.x + pdx >= 0) and (p.x + pdx < root.world.width()):
                    # Loop through delta y
                    for pdy in [-1, 0, 1]:
                        # Make sure the monster is moving
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
                                    newNode.children.append(Node.initExpectimax(ent, depth - 1, Node.newNode(newWrld, root.path),events, pathgoal))
                                root.children.append(newNode)

            return root
        else:  # is bottom level, we need to evaluate each current level node
            score = 0
            for e in events:
                if e.tpe == Event.CHARACTER_FOUND_EXIT:
                    score += 5000
                    root.score = tuple((root.path, score))
                    return root
                elif e.tpe == Event.CHARACTER_KILLED_BY_MONSTER:
                    score -= 1000
                    root.score = tuple((root.path, score))
                    return root
                elif e.tpe == Event.BOMB_HIT_CHARACTER:
                    score -= 1000
                    root.score = tuple((root.path, score))
                    return root

            calcpath = TestCharacter.Astar(root.world, (p.x, p.y), pathgoal, True)
            fpath = [pathgoal]
            while not (p.x, p.y) in fpath:
                fpath.append(calcpath.get(fpath[-1]))

            score -= 5 * len(fpath)  # penalize for longer paths

            # give 2 point for every free space in the next world, this should discourage us from running into walls
            score += 2 * len(TestCharacter.getPlayerNeighbors((p.x, p.y), root.world, False))

            for m in root.world.monsters.values():  # check how far we are from each monster
                if m[0].name == "stupid":
                    scanRange = 2 + 1
                elif m[0].name == "aggressive":
                    scanRange = 3 + 1
                elif m[0].name == "selfpreserving":
                    scanRange = 2 + 1

                monsterpath = TestCharacter.Astar(root.world, (m[0].x, m[0].y), (p.x, p.y), False)
                mpath = [(p.x, p.y)]
                unreachable = False
                while not (m[0].x, m[0].y) in mpath:
                    try:
                        mpath.append(monsterpath.get(mpath[-1]))
                    except(AttributeError):
                        unreachable = True
                        break

                if not unreachable and len(mpath) <= scanRange:
                    score -= 50 * (scanRange - len(mpath))

            root.score = tuple((root.path, score))
            return root

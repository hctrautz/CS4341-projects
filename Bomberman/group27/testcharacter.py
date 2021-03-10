# This is necessary to find the main code
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
                                        # we gotta bolt, would be detected
                                        # call expectimax to find our best move

                    if danger:
                        # check new worlds for 3 layers in advance
                        depth = 2
                        root = Node.newNode(wrld, [])
                        root = Node.initExpectimax(self, depth, root, [])
                        #root.value.append(Node.initExpectimax(self, wrld, [], p, 2, root, []))

                        # return move of best expectimax
                        move = Node.expectimax(root, True)
                        print(move)
                        move = move[0][0]
                        print(move)

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
                                coords.append(
                                    (startcoords[0] + dx, startcoords[1] + dy))
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
                # print(came_from)
                return came_from

            for next in self.getPlayerNeighbors(current, wrld, True):

                self.set_cell_color(next[0], next[1], Fore.CYAN)
                wallcost = 1

                if wrld.wall_at(next[0], next[1]):
                    wallcost += 100

                # graph.cost(current, next) #change this to use bomb maybe
                new_cost = cost_so_far[current] + wallcost
                if next not in cost_so_far or new_cost < cost_so_far[next]:
                    cost_so_far[next] = new_cost
                    priority = new_cost + self.getDistanceTo(next, goal)
                    frontier.put(next, priority)
                    self.set_cell_color(next[0], next[1], Fore.MAGENTA)
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
    def newNode(world, path):
        temp = Node(world, path);
        return temp;

    # Getting expectimax
    def expectimax(node, is_max):
        # Condition for Terminal node
        if not node.children: # if there is nothing in the child list
            return node.path, node.score

        # Maximizer node. Chooses the max from the children
        if (is_max):
            #print(node.children)
            expectichildren = []
            for c in node.children:
                expectichildren.append(Node.expectimax(c, False))


            maxset = max(expectichildren, key = lambda i : i[1])[0]
            print(max(expectichildren, key = lambda i : i[1]))
            return maxset

        # Chance node. Returns the average of
        # the left and right sub-trees
        else:
            totalSum = 0
            for child in node.children:
                path, score = Node.expectimax(child, True)
                totalSum+=score
            return node.path, (totalSum / len(node.children))

    def initExpectimax(self, depth, root, events):
        p = root.world.me(self)
        if depth != 1:  # if this isnt the bottom level, create list of children
            copywrld = SensedWorld.from_world(root.world)
            # calculate all monster money moves
            possiblemonstermoves = dict()
            # print(copywrld.monsters.values())
            for m in copywrld.monsters.values():  # get closest monsters position
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

            # print(possiblemonstermoves)
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
                                    self.move(pdx, pdy)  # apply player move
                                    # now apply all different monster moves
                                    for mo in bigboi:
                                        i = 0
                                        for m in copywrld.monsters.values():
                                            # Set move in wrld
                                            m[0].move(mo[i][0], mo[i][1])
                                            i += 1
                                        # Get new world
                                        (newWrld, events) = copywrld.next()  # get new world with moved entities
                                        newPath = deepcopy(root.path)
                                        newPath.append([(pdx, pdy)])
                                        root.children.append(Node.initExpectimax(self, depth - 1, Node.newNode(newWrld, newPath), events))

            return root # TODO something, maybe copy world more
        else:  # is bottom level, we need to evaluate each current level node
            score = 0
            for e in events:
                if e.tpe == Event.CHARACTER_FOUND_EXIT:
                    score += 1000
                elif e.tpe == Event.CHARACTER_KILLED_BY_MONSTER:
                    score -= 1000
                elif e.tpe == Event.BOMB_HIT_CHARACTER:
                    score -= 1000
                elif e.tpe == Event.BOMB_HIT_MONSTER:
                    score + - 100
                elif e.tpe == Event.BOMB_HIT_WALL:
                    score += 50



            goal = (root.world.exitcell[0], root.world.exitcell[1])  # this could be empty
            calcpath = self.Astar(root.world, (p.x, p.y), goal)
            score -= 2 * len(calcpath)  # penalize for longer paths

            for m in root.world.monsters.values():  # check how far we are from each monster
                distance = p.x - m[0].x + p.y - m[0].y
                if distance < 3:
                    score -= 2 * distance
            #print(tuple((path, score)))
            root.score = tuple((root.path, score))
            return root

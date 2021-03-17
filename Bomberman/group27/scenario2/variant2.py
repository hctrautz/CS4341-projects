# This is necessary to find the main code
import sys
sys.path.insert(0, '../../bomberman')
sys.path.insert(1, '..')

# Import necessary stuff
import random
from game import Game
from monsters.stupid_monster import StupidMonster

# TODO This is your code!
sys.path.insert(1, '../group27')
from testcharacter import TestCharacter

# Create the game
# TODO Change this if you want different random choices
g = Game.fromfile('map.txt')
g.add_monster(StupidMonster("stupid", # name
                            "S",      # avatar
                            3, 9      # position
))

# TODO Add your character
g.add_character(TestCharacter("me", # name
                              "C",  # avatar
                              0, 0,  # position
                              [(0, 2), (7, 2), (0, 6), (7, 6), (0, 10), (7, 10), (0, 14), (7, 14)]    # targets
))

# Run!
g.go(50)

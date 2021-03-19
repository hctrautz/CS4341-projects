import sys
from group27.testcharacter import TestCharacter

sys.path.insert(0, '../bomberman')

# Import necessary stuff
from game import Game
from monsters.stupid_monster import StupidMonster
from monsters.selfpreserving_monster import SelfPreservingMonster



def variant11(g):
    g.add_character(TestCharacter("me",  # name
                                     "C",  # avatar
                                     0, 0,  # position
                                     []))
    # Use this if you want to proceed automatically
    g.go(1)
    if g.world.scores["me"] > 0:
        return True


def variant12(g):
    g.add_monster(StupidMonster("stupid",  # name
                                "S",  # avatar
                                3, 9  # position
                                ))

    g.add_character(TestCharacter("me",  # name
                                     "C",  # avatar
                                     0, 0,  # position
                                     []))

    # Run!
    g.go(1)
    if g.world.scores["me"] > 0:
        return True


def variant13(g):
    g.add_monster(SelfPreservingMonster("selfpreserving",  # name
                                        "S",  # avatar
                                        3, 9,  # position
                                        1  # detection range
                                        ))

    g.add_character(TestCharacter("me",  # name
                                     "C",  # avatar
                                     0, 0,  # position
                                     []))

    # Run!
    g.go(1)
    if g.world.scores["me"] > 0:
        return True


def variant14(g):
    g.add_monster(SelfPreservingMonster("aggressive",  # name
                                        "A",  # avatar
                                        3, 13,  # position
                                        2  # detection range
                                        ))

    g.add_character(TestCharacter("me",  # name
                                     "C",  # avatar
                                     0, 0,  # position
                                     []))

    # Run!
    g.go(1)
    if g.world.scores["me"] > 0:
        return True


def variant15(g):
    g.add_monster(StupidMonster("stupid",  # name
                                "S",  # avatar
                                3, 5,  # position
                                ))
    g.add_monster(SelfPreservingMonster("aggressive",  # name
                                        "A",  # avatar
                                        3, 13,  # position
                                        2  # detection range
                                        ))

    g.add_character(TestCharacter("me",  # name
                                     "C",  # avatar
                                     0, 0,  # position
                                     [(7,6),(7,14)]))

    # Run!
    g.go(1)
    if g.world.scores["me"] > 0:
        return True




def variant21(g):
    g.add_character(TestCharacter("me",  # name
                                     "C",  # avatar
                                     0, 0,  # position
                                  [(0,2),(0,6),(0,10),(0,14)]))
    # Use this if you want to proceed automatically
    g.go(1)
    if g.world.scores["me"] > 0:
        return True


def variant22(g):
    g.add_monster(StupidMonster("stupid",  # name
                                "S",  # avatar
                                3, 9  # position
                                ))

    g.add_character(TestCharacter("me",  # name
                                     "C",  # avatar
                                     0, 0,  # position
                                  [(0,2),(0,6),(0,10),(0,14)]))

    # Run!
    g.go(1)
    if g.world.scores["me"] > 0:
        return True


def variant23(g):
    g.add_monster(SelfPreservingMonster("selfpreserving",  # name
                                        "S",  # avatar
                                        3, 9,  # position
                                        1  # detection range
                                        ))

    g.add_character(TestCharacter("me",  # name
                                     "C",  # avatar
                                     0, 0,  # position
                                  [(0, 2), (7, 2), (0, 6), (7, 6), (0, 10), (7, 10), (0, 14), (7, 14)]))

    # Run!
    g.go(1)
    if g.world.scores["me"] > 0:
        return True


def variant24(g):
    g.add_monster(SelfPreservingMonster("aggressive",  # name
                                        "A",  # avatar
                                        3, 13,  # position
                                        2  # detection range
                                        ))

    g.add_character(TestCharacter("me",  # name
                                     "C",  # avatar
                                     0, 0,  # position
                                  [(0, 2), (7, 2), (0, 6), (7, 6), (0, 10), (7, 10), (0, 14)]))

    # Run!
    g.go(1)
    if g.world.scores["me"] > 0:
        return True


def variant25(g):
    g.add_monster(StupidMonster("stupid",  # name
                                "S",  # avatar
                                3, 5,  # position
                                ))
    g.add_monster(SelfPreservingMonster("aggressive",  # name
                                        "A",  # avatar
                                        3, 13,  # position
                                        2  # detection range
                                        ))

    g.add_character(TestCharacter("me",  # name
                                     "C",  # avatar
                                     0, 0,  # position
                                  [(0, 2), (7, 2), (0, 6), (7, 6), (0, 10), (7, 10), (0, 14)]))
                                  #[(2, 0), (7, 0), (2, 3), (7, 3), (0, 10), (3, 10), (7, 10), (0, 14), (3, 14)]))

    # Run!
    g.go(1)
    if g.world.scores["me"] > 0:
        return True


def main():
    wins1 = 0
    wins2 = 0
    wins3 = 0
    wins4 = 0
    wins5 = 0
    wins1_2 = 0
    wins2_2 = 0
    wins3_2 = 0
    wins4_2 = 0
    wins5_2 = 0
    for _ in range(10):
        g = Game.fromfile('scenario1/map.txt', sprite_dir="../bomberman/sprites/")
        if variant11(g):
            wins1 += 1
    for _ in range(10):
        g = Game.fromfile('scenario1/map.txt', sprite_dir="../bomberman/sprites/")
        if variant12(g):
            wins2 += 1
    for _ in range(10):
        g = Game.fromfile('scenario1/map.txt', sprite_dir="../bomberman/sprites/")
        if variant13(g):
            wins3 += 1
    for _ in range(10):
        g = Game.fromfile('scenario1/map.txt', sprite_dir="../bomberman/sprites/")
        if variant14(g):
            wins4 += 1
    for _ in range(10):
        g = Game.fromfile('scenario1/map.txt', sprite_dir="../bomberman/sprites/")
        if variant15(g):
            wins5 += 1
    for _ in range(10):
        g = Game.fromfile('scenario2/map.txt', sprite_dir="../bomberman/sprites/")
        if variant21(g):
            wins1_2 += 1
    for _ in range(10):
        g = Game.fromfile('scenario2/map.txt', sprite_dir="../bomberman/sprites/")
        if variant22(g):
            wins2_2 += 1
    for _ in range(10):
        g = Game.fromfile('scenario2/map.txt', sprite_dir="../bomberman/sprites/")
        if variant23(g):
            wins3_2 += 1
    for _ in range(10):
        g = Game.fromfile('scenario2/map.txt', sprite_dir="../bomberman/sprites/")
        if variant24(g):
            wins4_2 += 1
    for _ in range(10):
        g = Game.fromfile('scenario2/map.txt', sprite_dir="../bomberman/sprites/")
        if variant25(g):
            wins5_2 += 1
    print(f'We won {wins1} out of 10 for variant 1')
    print(f'We won {wins2} out of 10 for variant 2')
    print(f'We won {wins3} out of 10 for variant 3')
    print(f'We won {wins4} out of 10 for variant 4')
    print(f'We won {wins5} out of 10 for variant 5')
    print(f'We won {wins1_2} out of 10 for variant 1_2')
    print(f'We won {wins2_2} out of 10 for variant 2_2')
    print(f'We won {wins3_2} out of 10 for variant 3_2')
    print(f'We won {wins4_2} out of 10 for variant 4_2')
    print(f'We won {wins5_2} out of 10 for variant 5_2')
    print("freq 8")

if __name__ == "__main__":
    # execute only if run as a script
    main()
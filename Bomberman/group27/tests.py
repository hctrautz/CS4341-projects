import sys
from group27.testcharacter import TestCharacter

sys.path.insert(0, '../bomberman')

# Import necessary stuff
from game import Game
from monsters.stupid_monster import StupidMonster
from monsters.selfpreserving_monster import SelfPreservingMonster


def variant1(g):
    g.add_character(TestCharacter("me",  # name
                                     "C",  # avatar
                                     0, 0,  # position
                                     ))
    # Use this if you want to proceed automatically
    g.go(1)
    if g.world.scores["me"] > 0:
        return True


def variant2(g):
    g.add_monster(StupidMonster("stupid",  # name
                                "S",  # avatar
                                3, 9  # position
                                ))

    g.add_character(TestCharacter("me",  # name
                                     "C",  # avatar
                                     0, 0,  # position
                                     ))

    # Run!
    g.go(1)
    if g.world.scores["me"] > 0:
        return True


def variant3(g):
    g.add_monster(SelfPreservingMonster("selfpreserving",  # name
                                        "S",  # avatar
                                        3, 9,  # position
                                        1  # detection range
                                        ))

    g.add_character(TestCharacter("me",  # name
                                     "C",  # avatar
                                     0, 0,  # position
                                     ))

    # Run!
    g.go(1)
    if g.world.scores["me"] > 0:
        return True


def variant4(g):
    g.add_monster(SelfPreservingMonster("aggressive",  # name
                                        "A",  # avatar
                                        3, 13,  # position
                                        2  # detection range
                                        ))

    g.add_character(TestCharacter("me",  # name
                                     "C",  # avatar
                                     0, 0,  # position
                                     ))

    # Run!
    g.go(1)
    if g.world.scores["me"] > 0:
        return True


def variant5(g):
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
                                     ))

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
    for _ in range(20):
        g = Game.fromfile('scenario1/map.txt', sprite_dir="../bomberman/sprites/")
        if variant1(g):
            wins1 += 1
    for _ in range(20):
        g = Game.fromfile('scenario1/map.txt', sprite_dir="../bomberman/sprites/")
        if variant2(g):
            wins2 += 1
    for _ in range(20):
        g = Game.fromfile('scenario1/map.txt', sprite_dir="../bomberman/sprites/")
        if variant3(g):
            wins3 += 1
    for _ in range(20):
        g = Game.fromfile('scenario1/map.txt', sprite_dir="../bomberman/sprites/")
        if variant4(g):
            wins4 += 1
    for _ in range(20):
        g = Game.fromfile('scenario1/map.txt', sprite_dir="../bomberman/sprites/")
        if variant5(g):
            wins5 += 1
    # for _ in range(20):
    #     g = Game.fromfile('scenario2/map.txt', sprite_dir="../bomberman/sprites/")
    #     if variant1(g):
    #         wins1_2 += 1
    # for _ in range(20):
    #     g = Game.fromfile('scenario2/map.txt', sprite_dir="../bomberman/sprites/")
    #     if variant2(g):
    #         wins2_2 += 1
    # for _ in range(20):
    #     g = Game.fromfile('scenario2/map.txt', sprite_dir="../bomberman/sprites/")
    #     if variant3(g):
    #         wins3_2 += 1
    # for _ in range(20):
    #     g = Game.fromfile('scenario2/map.txt', sprite_dir="../bomberman/sprites/")
    #     if variant4(g):
    #         wins4_2 += 1
    # for _ in range(20):
    #     g = Game.fromfile('scenario2/map2.txt', sprite_dir="../bomberman/sprites/")
    #     if variant5(g):
    #         wins5_2 += 1
    print(f'We won {wins1} out of 20 for variant 1')
    print(f'We won {wins2} out of 20 for variant 2')
    print(f'We won {wins3} out of 20 for variant 3')
    print(f'We won {wins4} out of 20 for variant 4')
    print(f'We won {wins5} out of 20 for variant 5')
    print(f'We won {wins1_2} out of 20 for variant 1_2')
    print(f'We won {wins2_2} out of 20 for variant 2_2')
    print(f'We won {wins3_2} out of 20 for variant 3_2')
    print(f'We won {wins4_2} out of 20 for variant 4_2')
    print(f'We won {wins5_2} out of 20 for variant 5_2')


if __name__ == "__main__":
    # execute only if run as a script
    main()

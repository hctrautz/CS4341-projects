import math
import agent
import numpy as np
from scipy.signal import convolve2d

import time
from functools import reduce

###########################
# Alpha-Beta Search Agent #
###########################
from board import Board


class AlphaBetaAgent(agent.Agent):
    """Agent that uses alpha-beta search"""

    # Class constructor.
    def __init__(self, name):
        super().__init__(name)

    # Heuristic that determines the maximum depth based on turn
    def __depth_heuristic(self, state):
        # Max # of Turns = W * H 
        # 21 is the median - Depth of 7 or 6
        # -(1/# of turns) 
        # -(1/8x - 2.449)^2 + 6
        turn = 0
        for r in state.board:
            for c in r:
                if c != 0:
                    turn += 1
        if turn < 2:
            depth = 0
        else:
            depth = -((1 / (state.w + (state.w // 2)) * turn) - (state.h / 3)) ** 2 + 6
            if depth < 1:
                depth = 2
        # if turn < 2:
        #     # Don't have to do heruistic calculation on the first move 
        self.x = state.h
        self.y = state.w
        self.n = state.n
        # if turn <state.h:
        #     depth = 2
        # elif turn < 2 * state.h + state.w:
        #     depth = 4
        # elif (turn < (state.h-2) * (state.w-1) and state.n == 4) or turn < (state.h-1) * (state.w-1):
        #     depth = 6
        return depth

    # Computes the value, action of a max value node in pruning
    def __max_value(self, state, depth, alpha, beta):
        win_state = state.get_outcome()
        # If there is a win condtion for the current player assign a value higher than the utility function will ever produce
        if win_state == self.player:
            return 1000, -1
        # If there is a win condtion for the opposing player assign a value lower than the utility function will ever produce
        elif win_state != 0:
            return -1000, -1
        if len(state.free_cols()) == 0:
            return 0, -1
        if depth >= 0:
            utility = self.__utility(state.board, state.player)
            return utility, -1

        else:
            best = (-math.inf, -1)
            for s, a in self.__get_successors(state):
                new_utility = self.__min_value(s, depth + 1, alpha, beta)
                if new_utility >= best[0]:
                    best = (new_utility, a)
                alpha = max(alpha, best[0])
                if best[0] >= beta:
                    return best
        return best

    # Computes the value, action of a max value node in pruning
    def __min_value(self, state, depth, alpha, beta):

        win_state = state.get_outcome()
        if win_state == self.player:
            return 1000
        elif win_state != 0:
            return -1000
        if len(state.free_cols()) == 0:
            return 0
        if depth >= 0:
            return self.__utility(state.board, state.player)
        else:
            worst = math.inf

            for s, a in self.__get_successors(state):
                new_utility, a = self.__max_value(s, depth + 1, alpha, beta)
                worst = min(worst, new_utility)
                beta = min(beta, worst)
                if worst <= alpha:
                    return worst
        return worst

    # Pick a column for the agent to play (External interface).
    def go(self, brd):
        """Search for the best move (choice of column for the token)"""
        turn = 0
        depth = -self.__depth_heuristic(brd)
        utility, action = self.__max_value(brd, depth, -math.inf, math.inf)
        if action < 0 or action > (self.y - 1):
            action = abs( self.y // 2)  # This line should not be relevant unless something goes wrong and the function returns an action
        return action

    # Get the successors of the given board.
    def __get_successors(self, brd):
        """Returns the reachable boards from the given board brd. The return value is a tuple (new board state, column number where last token was added)."""
        # Get possible actions
        freecols = brd.free_cols()
        # Are there legal actions left?
        if not freecols:
            return []
        # Make a list of the new boards along with the corresponding actions
        succ = []
        for col in freecols:
            # Clone the original board
            nb = brd.copy()
            # Add a token to the new board
            # (This internally changes nb.player, check the method definition!)
            nb.add_token(col)
            # Add board to list of successors
            succ.append((nb, col))
        return succ

    def givelinepoints(self, i):
        mid = np.ceil(self.n / 2)  # doesnt work 100% for size 10 board
        return mid - np.abs(mid - i)

    def findhole(self, board, startrow, startcol, dx, dy):
        for i in range(self.n):
            if board[startrow + i * dy][startcol + i * dx] == 0:
                return startrow + i * dy, startcol + i * dx
        return startrow + self.n * dy, startcol + self.n * dx
    # Utility function that takes a board_state and its player value
    def __utility(self, board, player):
        heur = [0, 0]  # score for each player

        # copy boards
        player1mask = np.copy(np.array(board))
        player2mask = np.copy(player1mask)

        # apply mask for player 1
        player1mask[player1mask > 1] = 0
        # apply mask for player 2
        player2mask[player2mask < 2] = 0
        player2mask = np.true_divide(player2mask, 2)

        masks = [player1mask, player2mask]

        # create different dynamically sized kernels
        horizontal_kernel = np.array([np.ones([self.n])])
        vertical_kernel = np.transpose(horizontal_kernel)
        diag1_kernel = np.eye(self.n, dtype=np.uint8)
        diag2_kernel = np.fliplr(diag1_kernel)
        detection_kernels = [horizontal_kernel, vertical_kernel, diag1_kernel, diag2_kernel]

        # get player 1 convmap
        # get player 2 convmap
        # p1threats = []
        # p2threats = []
        # isolatedp1threats = []
        # isolatedp2threats = []
        # compare to make sure overlapping areas dont get scored
        for kernel in range(len(detection_kernels)):  # check all directions
            p1threats = []
            p2threats = []
            isolatedp1threats = []
            isolatedp2threats = []
            p1validoddthreats = 0
            p2validoddthreats = 0
            p2valideventhreats = 0
            convp1 = convolve2d(masks[0], detection_kernels[kernel], mode="valid")  # create convolution matrix
            convp2 = convolve2d(masks[1], detection_kernels[kernel], mode="valid")
            if (convp1 == self.n-1).any():  # any winning move can be made
                heur[0] += 999
                SCORE = heur[0] - heur[1]
                if self.player == 2:  # invert if the player is 2
                    SCORE = SCORE * -1
                return SCORE
            if (convp2 == self.n-1).any():  # any winning move can be made
                heur[1] += 999
                SCORE = heur[0] - heur[1]
                if self.player == 2:  # invert if the player is 2
                    SCORE = SCORE * -1
                return SCORE


            for i in range(len(convp1)):  # length
                for j in range(len(convp1[i])):  # width

                    if convp1[i][j] != 0:  # something in the line from p1
                        if convp2[i][j] == 0:  # nothing blocking from opponent
                            heur[0] += self.givelinepoints(i + 1)  # give small points to p1
                            if convp1[i][j] >= 1:
                                p1threats.append((i, j, kernel))  # add threats for p1
                    elif convp2[i][j] != 0:  # something in line from p2 and not from p1
                        heur[1] += self.givelinepoints(i + 1)  # give small points to p2
                        if convp2[i][j] >= 1:
                            p2threats.append((i, j, kernel))  # add threats for p2

            # convert threat tuple into row and col of actual threat

            for i in range(len(p1threats)):
                if kernel == 0:  # if horizontal_kernel
                    isolatedp1threats.append(self.findhole(board, p1threats[i][0], p1threats[i][1], 1, 0))
                elif kernel == 1:  # if vertical_kernel
                    isolatedp1threats.append(self.findhole(board, p1threats[i][0], p1threats[i][1], 0, 1))
                elif kernel == 2:  # if diag1_kernel
                    isolatedp1threats.append(self.findhole(board, p1threats[i][0], p1threats[i][1], 1, 1))
                elif kernel == 3:  # if diag2_kernel
                    isolatedp1threats.append(self.findhole(board, p1threats[i][0], p1threats[i][1], 1, -1))

            for i in range(len(p2threats)):
                if kernel == 0:  # if horizontal_kernel
                    isolatedp2threats.append(self.findhole(board, p2threats[i][0], p2threats[i][1], 1, 0))
                elif kernel == 1:  # if vertical_kernel
                    isolatedp2threats.append(self.findhole(board, p2threats[i][0], p2threats[i][1], 0, 1))
                elif kernel == 2:  # if diag1_kernel
                    isolatedp2threats.append(self.findhole(board, p2threats[i][0], p2threats[i][1], 1, 1))
                elif kernel == 3:  # if diag2_kernel
                    isolatedp2threats.append(self.findhole(board, p2threats[i][0], p2threats[i][1], 1, -1))

            # threat conditional checking

            for i in range(len(isolatedp1threats)):
                curthreat = isolatedp1threats[i]
                # condition 1
                if (curthreat[0]+1 % 2) != 0:  # if p1curthreat is odd
                    if not any(col == curthreat[1] and row < curthreat[0] for row, col in isolatedp2threats):  # and no even threats below it in same col from p2
                        p1validoddthreats += 1
                        if not any(row+1 % 2 != 0 for row, col in isolatedp2threats):  # and p2 has no odd threat in other columns
                            heur[0] += 100  # give heavy points to p1

            for i in range(len(isolatedp2threats)):
                curthreat = isolatedp2threats[i]

                if curthreat[0]+1 % 2 != 0:  # if p2curthreat is odd
                    if not any(col == curthreat[1] and row < curthreat[0] for row, col in isolatedp1threats):
                        p2validoddthreats += 1
                if curthreat[0]+1 % 2 == 0:  # if p2curthreat is even
                    if not any(col == curthreat[1] and row < curthreat[0] for row, col in isolatedp1threats):
                        p2valideventhreats += 1

            # condition 2
            if heur[0] == 0:  # if instead
                if p2validoddthreats > p2validoddthreats:  # if player 1 has greater number of valid odd threats than p2 has valid odd threats
                    if p2valideventhreats == 0:  # and p2 has no even threats
                        heur[0] += 100  # give heavy points to p1

            # condition 3
            if heur[0] == 0:  # if neither of the above held true
                if p2valideventhreats != 0:  # and p2 has any valid even threats
                    heur[1] += 100  # give heavy points to p2

        SCORE = heur[0] - heur[1]  # final score is the players score - opponent score
        if self.player == 2:  # invert if the player is 2
            SCORE = SCORE * -1
        #print(heur, SCORE, board)
        return SCORE


# Final agent for class tournament (After testing and crude optimization)
THE_AGENT = AlphaBetaAgent("Group27")

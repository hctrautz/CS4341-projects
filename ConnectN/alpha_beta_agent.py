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
            depth = -((1 / (state.w + (state.w / 2)) * turn) - (state.h / 3)) ** 2 + 6
            if depth < 1:
                depth = 1
        # if turn < 2:
        #     # Don't have to do heruistic calculation on the first move 
        self.__board_x = state.h
        self.__board_y = state.w
        self.__connect_n = state.n
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
        if win_state == state.player:
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
                new_utility = self.__min_value(s, 1, alpha, beta)
                if new_utility >= best[0]:
                    best = (new_utility, a)
                alpha = max(alpha, best[0])
                if best[0] >= beta:
                    return best
        return best

    # Computes the value, action of a max value node in pruning
    def __min_value(self, state, depth, alpha, beta):

        win_state = state.get_outcome()
        if win_state == state.player:
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
                new_utility, a = self.__max_value(s, 1, alpha, beta)
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
        if action < 0 or action > (self.__board_y - 1):
            print("NEVER BE IN HERE")
            action = abs(
                action % self.__board_y)  # This line should not be relevant unless something goes wrong and the function returns an action
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
        # print(succ)
        # print(nb.board)
        # print(nb.player)
        return succ

    def my_is_line_at(self, x, y, dx, dy, board, p):
        """Return True if a line of identical tokens exists starting at (x,y) in direction (dx,dy)"""
        token = p
        count = 1
        while count <= self.__connect_n:  # searches for 4 of the same token in a row
            currx = x + (dx * count)  # finds coordinates for space in the given direction
            curry = y + (dy * count)
            if (
                    currx < 0 or currx >= self.__board_x):  # returns false if requested space is not on the board (out of bounds)
                # print(count)
                return count
            if (curry < 0 or curry >= self.__board_y):
                # print(count)
                return count
            if board[currx][curry] == token:  # if the token is found, increments counter and loops
                count += 1
            else:  # if the value doesn't equal the token
                # print(count)
                return count
        # print(count)
        return count

    # # Avoid out-of-bounds errors
    #  if ((x + (self.__connect_n - 1) * dx >= self.__board_y) or
    #          (y + (self.__connect_n - 1) * dy < 0) or (y + (self.__connect_n - 1) * dy >= self.__board_x)):
    #      return 0
    #  # Get token at (x,y)
    #  t = board[y][x]
    #  # Go through elements
    #  for i in range(1, self.__connect_n):
    #      if board[y + i * dy][x + i * dx] != t:
    #          return i
    #  return self.__connect_n

    def my_is_any_line_at(self, x, y, board, p):
        """Return True if a line of identical tokens exists starting at (x,y) in any direction"""
        return (np.max([self.my_is_line_at(x, y, 1, 0, board, p) ** 3,  # Horizontal
                        self.my_is_line_at(x, y, 0, 1, board, p) ** 3,  # Vertical
                        self.my_is_line_at(x, y, 1, 1, board, p) ** 3,  # Diagonal up
                        self.my_is_line_at(x, y, 1, -1, board, p) ** 3]))  # Diagonal down

    def evaluate_window(self, window, player):
        score = 0
        opp_player = 1
        if player == 1:
            opp_player = 2

        if window.count(player) == self.__connect_n:
            score += 100
        elif window.count(player) == self.__connect_n - 1 and window.count(0) == 1:
            score += 5
        elif (window.count(player) == self.__connect_n // 2 and window.count(0) == self.__connect_n - (self.__connect_n // 2)) or (window.count(player) == self.__connect_n - (self.__connect_n // 2) and window.count(0) == self.__connect_n // 2):
            score += 2
        
        if window.count(opp_player) == self.__connect_n - 1 and window.count(0) == 1:
            score -= 4
        print("Final Score:", score)
        return score

    # Utility function that takes a board_state and its player value
    def __utility(self, board, player):
        
        score = 0
	    ## Score center column
        center_array = [int(i) for i in list(board[:][self.__board_y // 2])]
        center_count = center_array.count(player)
        score += center_count * 3

	## Score Horizontal
        for r in range(self.__board_x):
            row_array = [int(i) for i in list(board[0:][r])]
            for c in range(self.__board_y - (self.__board_y - self.__connect_n)):
                window = row_array[c:c+self.__connect_n]
                score += self.evaluate_window(window, player)

	## Score Vertical
        for c in range(self.__board_y):
            col_array = [int(i) for i in list(board[c][0:])]
            for r in range(self.__board_x - (self.__board_x // 2)):
                window = col_array[r:r+self.__connect_n]
                score += self.evaluate_window(window, player)

	## Score posiive sloped diagonal
        for r in range(self.__board_x - (self.__board_x // 2)):
            for c in range(self.__board_y - (self.__board_y - self.__connect_n)):
                window = [board[r+i][c+i] for i in range(self.__connect_n)]
                score += self.evaluate_window(window, player)

        for r in range(self.__board_x - (self.__board_x // 2)):
            for c in range(self.__board_y - (self.__board_y - self.__connect_n)):
                window = [board[r+(self.__board_x // 2)-i][c+i] for i in range(self.__connect_n)]
                score += self.evaluate_window(window, player)

        return score
        # print("\n")
        # scores = [0,
        #           0]  # Array that stores the score of both players as the function loops through the cells and directions
        # aboard = np.array(board)

        # columnheights = []
        # curcol = []
        # for i in range(0, self.__board_y):
        #     curcol.append(aboard[:,i])
        #
        #     colheight = np.count_nonzero(curcol[i])
        #     columnheights.append(colheight)
        #     #print(columnheights)

        # for col in range(0, self.__board_y):  # for 0 to width
        #     for row in range(0, self.__board_x):
        #         token = board[row][col]
        #         if token != 0:
        #             scores[token - 1] += self.my_is_any_line_at(row, col, board, token)
        #     print(scores, scores[1] - scores[0])

        # score = scores[1] - scores[0]
        # if player == 2:
        #     score = scores[0] - scores[1]
        # return score;  # Returns the score for the state


# Final agent for class tournament (After testing and crude optimization)
THE_AGENT = AlphaBetaAgent("Group27")

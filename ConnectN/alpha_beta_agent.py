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
                new_utility = self.__min_value(s,  depth + 1, alpha, beta)
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
        return succ



    def my_is_line_at(self, x, y, dx, dy,board,p):
        """Return True if a line of identical tokens exists starting at (x,y) in direction (dx,dy)"""
        token = p
        if (token == 0):  # skips spaces with no token
            return False
        count = 1
        while (count < 4):  # searches for 4 of the same token in a row
            currx = x + (dx * count)  # finds coordinates for space in the given direction
            curry = y + (dy * count)
            if (currx < 0 or currx >= self.__board_x):  # returns false if requested space is not on the board (out of bounds)
                return count
            if (curry < 0 or curry >= self.__board_y):
                return count
            if (board[currx][curry] == token):  # if the token is found, increments counter and loops
                count += 1
            else:  # if the value doesn't equal the token
                return count
        return count

       # # Avoid out-of-bounds errors
        # if ((x + (self.__connect_n - 1) * dx >= self.__board_y) or
        #         (y + (self.__connect_n - 1) * dy < 0) or (y + (self.__connect_n - 1) * dy >= self.__board_x)):
        #     return False
        # # Get token at (x,y)
        # t = board[y][x]
        # # Go through elements
        # for i in range(1, self.__connect_n):
        #     if board[y + i * dy][x + i * dx] != t:
        #         return i
        # return self.__connect_n


    def my_is_any_line_at(self, x, y,board,p):
        """Return True if a line of identical tokens exists starting at (x,y) in any direction"""
        return (np.sum([self.my_is_line_at(x, y, 1, 0,board,p)**3,   # Horizontal
                self.my_is_line_at(x, y, 0, 1,board,p)**3, # Vertical
                self.my_is_line_at(x, y, 1, 1,board,p)**3,  # Diagonal up
                self.my_is_line_at(x, y, 1, -1,board,p)**3]))   # Diagonal down



    # Utility function that takes a board_state and its player value
    def __utility(self, board, player):

        scores = [0,0]  # Array that stores the score of both players as the function loops through the cells and directions

        aboard = np.array(board)
        colsheights = []
        for cols in range(0, self.__board_x):
            colsheights = []
            curcol = []
            for i in range(0, self.__board_y):
                curcol.append(aboard[:,i])
                colheight = np.count_nonzero(curcol[i])
                colsheights.append(colheight)


        for p in range(1,3):#for both players
            for col in range(self.__board_y): #for 0 to width
                    if(colsheights[col] != 0): # checking that highest column is not 0
                        scores[p-1] = self.my_is_any_line_at(col, colsheights[col], board, p)
                        #print(board)
                        print(scores, p,col,colsheights[col])

        score = scores[1] - scores[0]
        # if player == 2:
        #     score = -1 * score
        return score;  # Returns the score for the state


# Final agent for class tournament (After testing and crude optimization)
THE_AGENT = AlphaBetaAgent("Group27")

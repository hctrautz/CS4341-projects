import math
import agent
import numpy as np
from scipy.signal import convolve2d


import time
from functools import reduce

###########################
# Alpha-Beta Search Agent #
###########################

class AlphaBetaAgent(agent.Agent):
    """Agent that uses alpha-beta search"""

    # Class constructor.
    def __init__(self, name, score_weights, offensiveness):
        super().__init__(name)
        # Max search depth
        self.__score_weights = score_weights
        self.__offensiveness = offensiveness

    # Heuristic that determines the maximum depth based on turn
    def __depth_heuristic(self, state):
        turn = 0
        for r in state.board:
            for c in r:
                if c != 0:
                    turn += 1
        if turn < 2:
            self.__board_x = state.h
            self.__board_y = state.w
            self.__connect_n = state.n
        if turn <state.h:
            depth = 2
        elif turn < 2 * state.h + state.w:
            depth = 4
        elif (turn < (state.h-2) * (state.w-1) and state.n == 4) or turn < (state.h-1) * (state.w-1):
            depth = 6
        else:
            depth = (state.h * state.w) - turn
        return depth
    # Computes the value, action of a max value node in pruning
    def __max_value(self, state, depth, alpha, beta):
        win_state = state.get_outcome()
        if win_state == state.player:
            return self.__score_weights[4], -1
        elif win_state != 0:
            return -self.__score_weights[4], -1
        if len(state.free_cols()) == 0:
            return 0, -1
        if depth >= 0:
            utility = self.__utility(state.board, state.player)
            return utility, -1
        
        else:
            best = (-math.inf,-1)
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
        if win_state == state.player:
            return self.__score_weights[4]
        elif win_state != 0:
            return -self.__score_weights[4]
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
        if action < 0 or action > 6:
            action = abs(action % 7) #This line should not be relevant unless something goes wrong and the function returns an action
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
            succ.append((nb,col))
        return succ



    # def submat(mat, startRow, startCol,n):
    #     empty = np.zeros(n*2-1, n*2-1)#create correct sized empty array
    #
    #     result[:a.shape[0],:a.shape[1]]
    #
    #     return mat[:startRow:startRow+(n*2-1),startCol:startCol+(n*2-1)]
    # # create submat centering on the position being played
    # centered = self.submat(board, curRow - (n - 1), curCol - (n - 1), n)

    #Utility function that takes a board_state and its player value
    def __utility(self, board, player):
        heur = [0,0] #score for each player
        #given a board, calcuate the number of connected pieces for both players on whole board

        #copy boards
        player1mask = np.copy(np.array(board))
        player2mask = np.copy(player1mask)

        #apply mask for player 1
        player1mask[player1mask > 1] = 0
        #apply mask for player 2
        player2mask[player2mask < 2] = 0
        player2mask = np.true_divide(player2mask,2)

        masks = [player1mask, player2mask]
        #print(masks)
        #create different dynamically sized kernels
        horizontal_kernel = np.array([np.ones([self.__connect_n])])

        vertical_kernel = np.transpose(horizontal_kernel)
        diag1_kernel = np.eye(self.__connect_n, dtype=np.uint8)
        diag2_kernel = np.fliplr(diag1_kernel)
        detection_kernels = [horizontal_kernel, vertical_kernel, diag1_kernel, diag2_kernel]

        for p in range(0,2):
            for kernel in detection_kernels:
                heur[p] += np.sum(convolve2d(masks[p], kernel, mode="valid"))

        #print(heur)

        SCORE =  heur[0]-heur[1]
        if player == 2:
            SCORE = SCORE * -1
        print(SCORE)
        return SCORE




        # scores = [0,0] #Array that stores the score of both players as the function loops through the cells and directions
        #
        #                 #Adds score based on sequence and weights predefined in the class
        #             scores[this-1] += self.__score_weights[sequence]
        # score = self.__offensiveness * scores[0] - (1 - self.__offensiveness) * scores[1]
        # if player == 2:
        #     score = -1 * score
        return score #Returns the score for the state
#Final agent for class tournament (After testing and crude optimization)
THE_AGENT = AlphaBetaAgent("Group27", [0,10,50,5000,1000000], 0.35)
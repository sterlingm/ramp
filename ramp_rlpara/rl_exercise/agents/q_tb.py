import numpy as np
from colorama import init as clr_ama_init
from colorama import Fore
clr_ama_init(autoreset = True)

class QTable(object):
    """Q table agent (si version).

    Q table
    -------
        Q[s, a] = Q[x, y; encoded_dD]

        Each (x,y) is a single motion state.

        x ~ [0, 10]
        y ~ [0, 10]
        encoded_dD ~ [0, 2]: -0.1, 0.0, +0.1
    """
    def __init__(self):
        super(QTable, self).__init__()
        self.nb_actions = 3
        Q = np.random.rand(11, 11, self.nb_actions)



    def getQValues(self, ob):
        """Using average Q values.

        Arguments
        ---------
            ob: observation, multiple motion states.

        Return
        ------
            Average Q values.
        """
        if len(ob) == 0:
            print(Fore.RED + 'Cannot get Q values by empty observation')
            return np.zeros((self.nb_actions,))

        cnt = 0
        Q_sum = np.zeros((self.nb_actions,))
        for s in ob: # for each single motion state
            Q_sum += self.Q[s[0]][s[1]]
            cnt += 1

        Q_ave = 1.0 * Q_sum / cnt
        return Q_ave



    def epsilonGreedy(self, q_values, e=0.1):
        """Select action by e-greedy policy.

        Arguments
        ---------
            q_values: Q values.

            e: the probability of random selection.

        Return
        ------
            Encoded action.
        """
        if np.random.rand() < e:
            return np.random.randint(0, nb_actions)
        else:
            return np.argmax(q_values)



    def forward(self, ob):
        """Using average Q value to select a action.

        Don't average action.

        Arguments
        ---------
            ob: observation, multiple motion states.

        Return
        ------
            Encoded action.
        """
        q_values = self.getQValues(ob)
        return self.epsilonGreedy(q_values)
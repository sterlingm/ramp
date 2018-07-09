#!/usr/bin/env python


class TestPath:

    def __init__(self, numStates):
        self.numStates = numStates


class Obstacle:

    def __init__(self, tp):
        self.testPath = tp
        self.inState = 0
        



def GetABTCs(obs, numObs, abtc, allAbtcs):
    #print 'In GetABTCs, abtc: %s' % abtc
 
    allAbtcs.append(abtc[:])

    for i in range(len(abtc)-1, 0, -1):
        # Get difference in ob states
        diff = abtc[i] - abtc[i-1]

        # Check that we are not at the end
        if diff < obs[i-1].testPath.numStates-1:
            abtc[i] += 1

            # If incrementing a middle obstacle, put all remaining obstacle states equal to new state value
            # e.g. if 3 obs, and setting ob 2 state to 1, then set ob 3 state to 1 so the next abtc is 0,1,1
            if i < len(abtc):
                abtc[i+1:] = [abtc[i] for aa in abtc[i+1:]]

            # Do recursion
            GetABTCs(obs, numObs, abtc, allAbtcs)

    #return allAbtcs



def main():
    print 'In main'

    tp = TestPath(6)

    print 'Number of obstacle states: %s' % tp.numStates

    ob = Obstacle(tp)
    obs = [ob]
    
# Append the second ob
    obs.append(ob)
    obs.append(ob)
    #obs.append(ob)
    #obs.append(ob)


    #total = ComputeTotal(obs)
    #print 'ABTCs: %s' % total
    #print 'Total number of ABTCs: %s' % len(total)

    abtcs = []
    initialABTC = [0 for i in range(0,len(obs))]
    GetABTCs(obs, len(obs), initialABTC, abtcs)
    print 'ABTCs: %s' % abtcs
    print 'Total number of ABTCs: %s' % len(abtcs)


if __name__ == '__main__':
    main()
    print 'Exiting normally'

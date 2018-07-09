#!/usr/bin/env python


class TestPath:

    def __init__(self, numStates):
        self.numStates = numStates


class Obstacle:

    def __init__(self, tp):
        self.testPath = tp
        self.inState = 0


def ComputeTotal(obs):
    print 'In ComputeTotal'
    result = []


    states = range(0, obs[0].testPath.numStates)
    print states

    for i in range(0, obs[0].testPath.numStates):
        print i

        # Set the first obstacle's state
        obs[0].inState = i

        # At this state, the next obstacle can start
        #result += 1
        #print 'Adding 1 with Ob 0 at state %s' % i

        # Compute when next obstacle starts
        # next obstacle can start at any state so do a full for loop
        for j in range(0, obs[1].testPath.numStates):
            print j

            # At this state, the next obstacle can start
            if len(obs) > 2:
                print 'Adding 1 with Ob 0 at state %s and Ob 1 at state %s' % (i, j)

                abtc = [0, i, i+j]
                result.append(abtc)

        
        

    print 'Exiting ComputeTotal'
    return result


def main():
    print 'In main'
    tp = TestPath(3)
    print tp.numStates

    ob = Obstacle(tp)
    obs = [ob]
    
# Append the second ob
    obs.append(ob)
    obs.append(ob)


    total = ComputeTotal(obs)
    print 'ABTCs: %s' % total
    print 'Total number of ABTCs: %s' % len(total)


if __name__ == '__main__':
    main()
    print 'Exiting normally'

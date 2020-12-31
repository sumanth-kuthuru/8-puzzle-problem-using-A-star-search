# -*- coding: utf-8 -*-
"""
8-puzzle problem using A* search algorithm
@author: Sumanth Kuthuru
"""
import numpy as np
from copy import deepcopy
import collections

print("Enter input by giving spaces example:1 2 3 4 5 6 7 8 0 ")
initial_state = list(map(int,input("Enter Initial state:").split()))               #input initial state
initial_state = np.array(initial_state)
goal_state = list(map(int,input("Enter goal state:").split()))                #input goal state
goal_state = np.array(goal_state)
heuristic = 0      # heuristic= 0 for manhattan and 1 for misplaced tiles


# function to calculate hn considering manhattan distance heuristic
def manhattan_distance(state):
    copy = state
    mhtndist = 0
    for i, list_item in enumerate(copy):
        if list_item != 0:
            for j,list_item_final in enumerate(goal_state):
                if list_item_final == list_item:
                    lr = j
                    break
            row1,col1 = int(i/ 3) , i% 3
            row2,col2 = int((lr) / 3), (lr) % 3
            mhtndist += abs(row1-row2) + abs(col1 - col2)  #caluculating manhattan by comparing current state and goal state
    return mhtndist

#function to calculate hn considering misplaced tiles heuristic
def misplaced_tiles(state):
    copy = state
    mis_dist=0
    for i, list_item in enumerate(copy):
        if list_item != 0 and goal_state[i] != list_item:
            mis_dist = mis_dist+1
    return  mis_dist

#function to calculate child nodes of the node to be expanded
def childNodes(current_state):
    global frontier                          #frontier list for those child nodes of expanded nodes
    global expanded                          #expanden list for expanded node
    global nodeid                            #nodeid unique for each node
    moves = np.array(
        [
            ([0, 1, 2], -3),
            ([6, 7, 8], 3),
            ([0, 3, 6], -1),
            ([2, 5, 8], 1)
        ],
        dtype=[
            ('pos', list),
            ('ind', int)
        ]
    )

    gn=current_state[1]+1
    state = current_state[0]
    loc = int(np.where(state == 0)[0])
    parentid = current_state[4]
    for m in moves:
        if loc not in m['pos']:
            nodepresent = 0           # To check if node is present
            child = deepcopy(state)
            delta_loc = loc + m['ind']
            child[loc], child[delta_loc] = child[delta_loc], child[loc]  #exchanging values

            for i in expanded:             #checking for child node duplicates in expanded
                if(i[0]==child).all():
                    nodepresent = 1

            for i in frontier:               #checking for child node duplicates in frontier
                if (i[0] == child).all():
                    nodepresent = 1

            if nodepresent == 0:

                if (heuristic == 0):
                    hn = manhattan_distance(child)
                else:
                    hn = misplaced_tiles(child)
                fn=gn + hn
                nodeid=nodeid+1                            #incrementing nodeid for each node genereated

                frontier=np.append(frontier, np.array([(child, gn, hn, fn, nodeid, parentid)], STATE), 0)  # frontier node gets appended with all the values

#function to check if the node is goal state
def goalTest(current_state):
    global STATE
    STATE = [
        ('current_state', list),
        ('gn', int),
        ('hn', int),
        ('fn', int),
        ('nodeid',int),
        ('parentid',int)
    ]
    global frontier
    global expanded
    global nodeid
    nodeid = 0                             #nodeid of node at depth zero or initial state
    if(heuristic==0):
        hn=manhattan_distance(current_state)
    else:
        hn=misplaced_tiles(current_state)
    frontier = np.array([(current_state, 0, hn, 0 + hn, 0, -1)], STATE)    #initial state gets appended to frontier

    empty=np.array([0,0,0,0,0,0,0,0,0])                                      #expanded 1st time initialization
    expanded=np.array([(empty, 0, 0, 0, 0, 0)], STATE)
    expanded=np.delete(expanded, 0, 0)                                  #making it empty

    while True:
        length_queques = len(frontier) + len(expanded)
        if length_queques >3000:                      #checking if total nodes are crossing the threshold value
            break
        a=frontier[0]
        s=a[0]
        if (s == goal_state).all():                    #comparing with goal state
            return len(expanded), nodeid
        frontier = np.delete(frontier, 0, 0)    #deleting the node expanded
        expanded=np.append(expanded, np.array([(a[0], a[1], a[2], a[3], a[4], a[5])], STATE), 0) #appending visited node to expanded
        childNodes(a)                         #generated childNodes of selected node from frontier
        frontier = np.sort(frontier, kind='mergesort', order=['fn', 'nodeid'])  #sorting as new childNodes enter frontier so the least fn value gets priority
    return 0,0

#function to find the path to the goal state
def solutionpath(frontier, expanded):
    lastelement = frontier[0][0]
    parentid= frontier[0][5]
    allNodes = np.concatenate((frontier, expanded), axis=0)
    de = collections.deque([])
    de.append(lastelement)
    while(parentid != -1):                   #we trace back from the last element discovered to find the path to goal state using parentid
        for i in allNodes:
            if i[4] == parentid:
                de.appendleft(i[0])
                parentid = i[5]
                break
    print('cost to reach goal state:', len(de)-1)

    for i in de:
        print(np.reshape(i,(3,3)),'\n')

#main function to print solutionpath using both heuristics
def main():
    global frontier
    global expanded
    global heuristic


    comparearrays = (np.sort(initial_state) == np.sort(goal_state)).all()     #bool value to check if input is correct
    if not comparearrays:
        print('incorrect input')
        return
    else:
        nodes_expanded,nodes_generated = goalTest(initial_state)                #To get no.of nodes expanded and generated
        if(nodes_expanded==0 and nodes_generated ==0):
            print('cannot solve')
            return
    print("A* Manhattan DIstance")

    print('nodes generated:',nodes_generated)
    print('nodes expanded',nodes_expanded)

    solutionpath(frontier, expanded)         #calling solutionpath function
    print("A* Misplaced Tiles")
    heuristic = 1                                                #set heuristic = 1 to use misplaced tiles heuristic
    frontier=[]                                         #empty both frontier and expanded
    expanded=[]
    nodes_expanded, nodes_generated = goalTest(initial_state) #calling goalTest which returns no of nodes generated and expanded
    if (nodes_expanded == 0 and nodes_generated == 0):           #return 0,0 if no solution
        print('cannot solve')
        return
    print('nodes generated:',nodes_generated)
    print('nodes expanded',nodes_expanded)
    solutionpath(frontier, expanded)          #finding goalTest path hn=misplaced tiles


if __name__ == "__main__":
    main()

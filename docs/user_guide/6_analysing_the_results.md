# Analysing the results

These benchmarks help us answer the following questions.

## Which is the fastest planner?
Lazy planners (LBKPIECE, SBL) give low solved percent in general (always substantially below the others) and don’t give best time/quality results, so are not used anymore
RRTstar and both PRM always take all the time available, because they are optimization planners.

for cartesian queries:
RRTConnect, TRRT and PRK seems to be ok, KPIECE seems to be worse

BKPIECE is slow compared to other non-lazy planners; RTTConnect seems to be good

Planners that we have found are the best
RRTConnect
EST
KPIECE


## Which is the planner that solved the most number of queries?
## Which is the planner that gives us the best quality plans?
## Is it the same to plan in joint or cartesian space?
## How much is the variation between trials of the same query?
## What is the best configuration of the arm joints range that give us the best planning results?
## What is the difference of the planners testing them in different scenes?
## What is the best set of parameters that give us the best planning results?
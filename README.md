# FoundationAI
A coursework about using different searching methods
====================
This course work asks me to solve the 8 puzzle problem using searching method.
![alt text](https://github.com/bagaabu/FoundationAI/blob/master/8 puzzles problem.png "fig 1")

In order to compare the different performances of several searching methods, i implement BFS(Breadth-First-Search), DFS(Depth-First-Search), Greedy Search, IDS(Iterative Deepening Search), Astart Search, Bi-direction Astart Search.

Conclusion
=========
The four methods (BFS,DFS,Greedy Search,IDS) have their own advantages; DFS can give solutions when dealing with
big-step-puzzle, but waste too much steps; BFS can find the optimal solution in small step puzzle,
however it not good in doing long step puzzle; IDS is an average way to gives a solution but may
not be optimal and search too much nodes; Greedy search searches far less states and give the
solution, but not really optimal.Bidirectional A-star searches less than A-star, and provides a similar solution compared with
A-star, so I believe Bidirectional A-star is actually better than A-star.



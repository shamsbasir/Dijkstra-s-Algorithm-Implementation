# Dijkstra-s-Algorithm-Implementation

/* 
 * Author: Shamsulhaq Basir
 * How To run on Linux/Mac OS: g++ -O3 -Wall Homework2.cpp -std=c++11
 * ./a.out
 * average cost for Graph with 0.20 is : 4
 * average cost for Graph with 0.40 is : 2
 */

/* PriorityQueue is used to find the closest distance to the starting node without 
 * looking through all the edges which greatly reduces the computation time from O(V) to 
 * O(log(V)) and that will ultimately make the run time complexity of the Dijkstra algorithm
 * O(Elog(V))
 */

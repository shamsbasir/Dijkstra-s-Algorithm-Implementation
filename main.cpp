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

#include<iostream>
#include<vector>
#include<cstdlib>
#include<fstream>
#include<ctime>
#include<random>
#include<climits>
#include<iomanip>
#include<algorithm>
#include<unordered_map>

using namespace std;

// This function gives every node a character name
inline char intTochar(int i){
	char result;
	(i<26)?result='A'+i:result='a'+i-26;
return result;

}


// This class is to abstract the range of the weigts
class range{
	public:
	int min; 
	int max;
	range(int min, int max):min(min),max(max){};
	range(){};
	friend std::ostream & operator<<(std::ostream &out, const range &R);
    	
};

// this function is to overload the << operator 
ostream & operator<<(ostream &out, const range &R){
	out<<"{"<<R.min<<", "<<R.max<<"}"<<endl;
	return out;
}

// Graph class that includes sufficient methods for user friendlines
class Graph{
private:
	// the number of vertices 
	int vertices;

	// probibility of the edge existance 
	float density;
	
	// range of weights
	range R; 
	
	// matrix of the G
	int **G; 

public:
	// constructor
	Graph(int vertices, float density, range R);

	// returns the number of vertices in the G
	int V(void);

	// returns the number of edges in the G
	int E(void);

	// tests whether there is an edge from node x to node y.
	bool adjacent(int x, int y);

 	//lists all nodes y such that there is an edge from x to y.
	 vector<int> neighbors(int x);

	// adds to G the edge from x to y, if it is not there.
	void add(int x, int y, int cost);

	//removes the edge from x to y, if it is there.
	void delete_edge(int x, int y); 

	//returns the value associated to the edge (x,y).
	int get_edge_value(int  x, int y);

	//sets the value associated to the edge (x,y) to v.
	void set_edge_value (int x, int y, int v); 
	
	// display the graph
	void show();

	// destructor
	~Graph();

};


Graph::Graph(int vertices, float density, range R){

	this->vertices  = vertices;
	this->density   = density;
	this->R 		= R; 
	srand(time(0));
	G = new int*[vertices];
    
       for (int i = 0; i < vertices -1 ; i++){
             G[i] = new int[vertices];
        }     


	  for (int i =0; i < vertices -1; ++i){
            for (int j =0; j < vertices -1 ; ++j){

                if (i == j){
                   G[i][j] = 0;
               
				 }else{
					
                   G[i][j] = G[j][i] = (static_cast<float>(rand())/static_cast<float>(RAND_MAX) < density)
								  		*static_cast<int>(rand()%(R.max-1)+R.min);
               }
        	}   
    	}

}


// display the graph
void Graph::show(){

	cout<<"Graph Properties are as follows: \n";
	cout<<"vertices = "<<vertices<<endl;
	cout<<"density  = "<<density<<endl;
	cout<<"range    = "<<R<<endl;	
	
	cout<<endl<<"   ";

	for(int i = 0 ; i < vertices -1; i++){
		cout<<intTochar(i)<<" ";
	}
	cout<<endl;	
	

	for(int i = 0 ; i < vertices -1; i++){
		cout<<intTochar(i)<<"  ";
		for (int j = 0 ; j < vertices -1; j++){
			if(G[i][j] !=0){
			cout<<G[i][j]<<" ";
			}else
			{
			cout<<"-"<<" ";
			}		
	    }
	cout<<"\n";
}
}

 // returns the number of vertices in the G

int Graph::V(void){
	return vertices;

}

 // returns the number of edges in the G
int Graph::E(void){
	int edgeNumber = 0;

	for(int i = 0 ; i < vertices -1; i++){
		for (int j = 0 ; j < vertices-1; j++){
		
			if(G[i][j] != 0){
				edgeNumber++;
			 }
		}
	}
return edgeNumber;
}

// tests whether there is an edge from node x to node y.
bool Graph::adjacent(int x, int y){
 	if(G[x][y] !=0){
		  return true;
	}
	else {
		return false;
	}

}

//lists all nodes y such that there is an edge from x to y.
vector<int> Graph::neighbors(int x){
	vector<int> neighbors;
	
	for (int i= 0 ; i < vertices -1; i++){
		if(G[x][i] !=0){
		neighbors.push_back(i);
		}
	}
	return neighbors;
}

// adds to G the edge from x to y, if it is not there.
void Graph::add(int x, int y , int cost){
		
	if(G[x][y] == 0){
		G[x][y] = cost;
	 }
	else 
	{
	cout<<"Edge between"<<x<<"and"<<y<<"exist"<<endl;
	}

}

//removes the edge from x to y, if it is there.
void Graph::delete_edge(int x, int y){
	
	G[x][y] = 0;

}

//returns the value associated to the edge (x,y).
int Graph::get_edge_value(int  x, int y){
	return G[x][y]; 

}

//sets the value associated to the edge (x,y) to v.
void Graph::set_edge_value(int x, int y , int v){

	G[x][y] = v;

}

// destructor
Graph::~Graph(){
    
	for (int i = 0 ; i < vertices -1; i++){
            delete G[i];
    }

	delete [] G;
}



typedef struct QueNodeType{
	
	int node;
	int distance;
	QueNodeType(int node = -1, int distance = INT_MAX):
			node(node),distance(distance){};
}Qtype;


//The value of the PriorityQueue is to always have access to the vertex with the next shortest link
//in the shortest path calculation at the top of the queue.
class PriorityQueue{

private:
	// vector of Queue elements
	vector<Qtype>PQ;

public:

	//constructor
	PriorityQueue(int &vertices);
	
	//changes the priority (node value) of queue element	
	void chgPriority(Qtype &element); 

	//removes the top element of the queue.
	void minPriority(void);

	//does the queue contain queue_element.
	bool contains(Qtype &element);

	//insert queue_element into queue
	void Insert(Qtype &element);

	//returns the top element of the queue.
	Qtype top();

	//return the number of queue_elements.
	int size();
   
	//sorting the queue
 	void sortQueue(); 
	
	// destructor
	~PriorityQueue(){};

};
// default constructor

PriorityQueue::PriorityQueue(int &vertices){
	
	Qtype element;

	for(int i = 0 ; i < vertices ; i++){
		element.node = i;
		element.distance = INT_MAX;
		}

}

//changes the priority of queue element
void PriorityQueue::chgPriority(Qtype &element){
	for (auto &it: PQ){
		if(it.node == element.node){
			it.distance = element.distance;
		}
		}
	sortQueue();

}

//sorting function for sorting the vector of nodes
bool sortByDistance(Qtype q1, Qtype q2){
	return (q1.distance < q2.distance);
}


void PriorityQueue::sortQueue(){
	sort(PQ.begin(), PQ.end(), sortByDistance);
}


//removes the top element of the queue.
void PriorityQueue::minPriority(void)
{
	PQ.erase(PQ.begin());
}



//does the queue contain queue_element.
 bool PriorityQueue:: contains(Qtype &element)
{
	bool result;
	for (auto it = PQ.begin(); it != PQ.end(); it++)
	{
		if(it->node == element.node && it->distance == element.distance){
			result =  true;
		}
		else 
		{
			result = false;
		}
	}
	return result;
}


//insert queue_element into queue 
void PriorityQueue:: Insert(Qtype &element)
{
	PQ.push_back(element);
	sortQueue();
}



//returns the top element of the queue.
Qtype PriorityQueue::top()
{
	return PQ.front();
}



//return the number of queue_elements.
int PriorityQueue:: size()
{
	return PQ.size();
}


class ShortestPath{

private:

// to store previous nodes for keeping the path
vector<int> previous;

// distance vector
vector<int> min_dist;

 
public:
// default constructor

ShortestPath(){};

//list of vertices in G(V,E)
vector<int> vertices( Graph &G);

//find shortest path between u-w and returns the sequence of vertices representing shorest path u-v1-v2-…-vn-w.
vector<int> path(Graph &G, int u, int w);

// return the path cost associated with the shortest path.
int path_size(Graph &G, int u, int w);

// destructor
~ShortestPath(){};

};




//list of vertices in G(V,E)
vector<int>ShortestPath::vertices(Graph &G)
{
	vector<int> List;
		for(int i = 0 ; i < G.V(); i++){
			List.push_back(i);
		}

return List;
}
int ShortestPath::path_size(Graph &G, int u, int w){
	
	// calculate the path
	path(G,u,w);	

	if(min_dist[w] == INT_MAX){
		// we want return a negative number so we can avoid adding it
		return -1;
	}else
	{
	return min_dist[w];
	}
}

// return the path vector from u to w 
vector<int>ShortestPath::path(Graph &G, int u, int w){

	// path vector	 
	vector<int>path;

	// number of vertices	
	int vertices = G.V();

	// fill the priority queue with all the vertices 
	PriorityQueue pq(vertices);
	
	// initial source point of the graph
	Qtype start(u,0);

	// update the source point in the priority queue 
	pq.Insert(start);	

	// set the source in the priority queue
    pq.chgPriority(start);	
	
	// initialy min_dist = 0 ;
	min_dist.clear();
	
	min_dist.resize(vertices,INT_MAX);

	min_dist[u] = 0 ;
	
	// clear the previous vector
	previous.clear();

	previous.resize(vertices,-1);

    while(pq.size() != 0 ){	

	// get the top element with minimum distance
	Qtype topElement = pq.top();
	
	int dist = topElement.distance;
	int u    = topElement.node;
	
	// remove the top element since it is picked 
	pq.minPriority();  

	if (dist > min_dist[u]){
			continue;
	}	
	// get all the neighbors of the topElement
	vector<int> Nbrs = G.neighbors(topElement.node);

	for(auto &it: Nbrs){
		
		int v 	             = it;
		int weight           = G.get_edge_value(u,v);
		int dist_tru_u       = dist + weight;
	
			if(dist_tru_u < min_dist[v]){
				// update the minimum distance
				min_dist[v]     = dist_tru_u;
				
				//update previous 				
				previous[v]    = u;
	
				// update the node in the priority queue 
				Qtype tmp;
				tmp.node 	 = v;
				tmp.distance = min_dist[v];	

				pq.Insert(tmp); // now this node has the highest priority

			}

		 }


	}

	
	if(min_dist[w] == INT_MAX){
		return path;
	}
	else
	{	
	int cursor = w;
	do{
		path.push_back(cursor);
		cursor = previous[cursor];
		}while(cursor != -1);
		
 	reverse(path.begin(), path.end());

	
	   return path;
	}
}

// This class is used to simulate the Dijkstra's algorithm for
// a specific graph
class MonteCarlo{

public:

MonteCarlo(){};
// average path function
int averagePath(Graph &G, int source);


};

// This function find the average cost from source to all 
// other vertices with a connected edge
int MonteCarlo::averagePath(Graph &G, int source)
{	
	// number of vertices
	int size  = G.V();
	
	// total cost
	int cost  = 0;
	
	// number of routes
	int count = 1;

	ShortestPath sp;	
	for (int it = 0; it < size; it++){
			// check to see if the distances are meaningful
			if(sp.path_size(G,source,it) > 0){
				// distance found
				cost  += sp.path_size(G,source,it); 
				count += 1;
			}	

	}
	// return the average route
	return cost/count;


}



/* Beginning of the main driver */

int main(){

// weight range
range R(1,10);

// number of nodes
int nodes 	= 50;

// density 
float density1 = 0.20;

// density
float density2 = 0.40;
 
{
// Graph instantiation
Graph G1(nodes,density1,R);

// display the graph
//G1.show();

// Monte Carlo Simulation;
MonteCarlo sim1;

//initial vertex
int source = 0;

cout<<"average cost for Graph with 0.20 is : "<<sim1.averagePath(G1,source)<<endl;

}
// Graph instantiation
Graph G2(nodes,density2,R);

// display the graph
//G2.show();

MonteCarlo sim2;

int src = 0;

cout<<"average cost for Graph with 0.40 is : "<<sim2.averagePath(G2,src)<<endl;


/* End of the program */ 

return EXIT_SUCCESS;
}

/*
*
* main.cpp
*
* Created on Nov 23 2020
*       Author: Animesh Nema
*
*/
#include"GridSearchDFS.h"

int main()
{
    //Initialize the grid. (rows,columns, probability of creating obstacle).
    GridSearchDFS dfs(5,5,20);

    dfs.createGrid();
    dfs.printGrid();

    //Perform Depth First Search (start_row,start_column).
    dfs.depthFirstSearch(0,0);
    dfs.printPath();
}

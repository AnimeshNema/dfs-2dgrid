/*
*
* GridSearchDFS.h
*
* Created on Nov 23 2020
*       Author: Animesh Nema
*
*/
#pragma once
#include <iostream>
#include <queue>
#include<stack>
#include<vector>

///\class GridSearchDFS
///\brief The This class creates a 2D grid with obstacles and performs Depth First Search algorithm to find a path, if one exists. 

class GridSearchDFS
{
private:
	uint8_t m_row; ///< number of rows in the grid
	uint8_t m_column; ///< number of columns in the grid.
	uint8_t m_obstacle_percentage; ///< Percent probability of obstacle being generated. (Range 0-100)
	bool m_goal_reached; ///< Goal reached check condition
	std::stack<std::pair<uint8_t, uint8_t> > m_path; ///< Path list
	std::stack<std::pair<uint8_t, uint8_t> > m_stack; ///< DFS Stack (Node Exploration Priority).
	std::vector<std::vector<uint8_t> > m_grid; /// 2D Grid.
	
public:
	/**
	 * Constructor used to initialize the grid 
	 * @param row - Row size of the grid
	 * @param column - Column size of the grid
	 * @param obstacle_percentage - The probability by which a point could be obstacle. 
	 */
	GridSearchDFS(uint8_t row, uint8_t column, uint8_t obstacle_percentage);

	///Destructor
	~GridSearchDFS();

	/**
	 * CreateGrid - Creates a 2D grid with obstacles. 
	 */
	void createGrid();

	/**
	 * printGrid - displays the grid on the console.
	 */
	void printGrid();

	/**
	 * depthFirstSearch - Implements Depth First Search Algorithm based on given starting point.
	 * @param start_row - Starting row of search.
	 * @param start_column - Starting column of search.
	 */
	void depthFirstSearch(uint8_t start_row, uint8_t start_column);

	/**
	 * printPath - Displays the path co-ordinates.
	 */
	void printPath();

	/**
	 * updatePath - Removes unwanted nodes from the path.
	 * @param curr_node - Current Node to be added in the Path.
	 */
	void updatePath(uint8_t& curr_row, uint8_t& curr_column);

	void printPathCoordinates();
};


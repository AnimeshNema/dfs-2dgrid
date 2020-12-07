/*
*
* GridSearchDFS.cpp
*
* Created on Nov 23 2020
*       Author: Animesh Nema
*
*/
#include "GridSearchDFS.h"
#include<iostream>
#include<stdlib.h>
#include<time.h>


GridSearchDFS::GridSearchDFS(uint8_t row, uint8_t column, uint8_t obstacle_percentage)
	:m_row(row), m_column(column), m_obstacle_percentage(obstacle_percentage), m_goal_reached(false)
{
    std::vector<std::vector<uint8_t>> grid(m_row, std::vector<uint8_t>(m_column));
    m_grid = grid;
}

GridSearchDFS::~GridSearchDFS()
{}

void GridSearchDFS::createGrid()
{
    uint8_t random_number;
    srand(time(NULL));

    for (uint8_t i = 0; i < m_row; i++) {

        for (uint8_t j = 0; j < m_column; j++)
        {
            random_number = rand() % 100 + 1;     //Generates a random number between 1-100

            if (random_number < m_obstacle_percentage)
            {
                m_grid[i][j] = 1;   // 1 Represents Obstacles.
            }

            else
            {
                m_grid[i][j] = 0;  // 0 represents free space.
            }
        }
    }
}

void GridSearchDFS::printGrid()
{
    if (m_row > 20 || m_column > 20)
    {
        std::cout << "Grid too big to print on Console" << std::endl;
    }
    else
    {
        for (uint8_t i = 0; i < m_row; i++)
        {
            for (uint8_t j = 0; j < m_column; j++)
            {
                std::cout << int(m_grid[i][j]) << " ";
            }
            std::cout << "\n";
        }
    }
}

void GridSearchDFS::updatePath(uint8_t& curr_row, uint8_t& curr_column)
{
    uint8_t* previous_row_path = &(m_path.top()).first;
    uint8_t* previous_column_path = &(m_path.top()).second;

    if ((curr_row == *previous_row_path) || (curr_column == *previous_column_path))
    {
        m_path.push(std::make_pair(curr_row,curr_column));
        m_grid[curr_row][curr_column] = 4;      // 4 represents Path.
    }
    else
    {
        m_grid[*previous_row_path][*previous_column_path] = 2; //mark as visited
        m_path.pop();
        updatePath(curr_row,curr_column);
    }
}

void GridSearchDFS::depthFirstSearch(uint8_t start_row, uint8_t start_column)
{
    if (m_grid[start_row][start_column] == 1 || m_grid[m_row - 1][m_column - 1] == 1)
    {
        std::cout << "No Path Exists as Start/Goal nodes are blocked." << std::endl;
    }
    else
    {
        m_grid[start_row][start_column] = 4; //mark as visited

        uint8_t* current_row = &start_row;
        uint8_t* current_column = &start_column;

        m_stack.push(std::make_pair(*current_row, *current_column));
        m_path.push(std::make_pair(*current_row, *current_column));

        //Directions (left, up, right, down)
        int8_t direction_row[4] = { 0,-1,0,1 };
        int8_t direction_column[4] = { -1,0,1,0 };

        while (m_stack.size() > 0)
        {
            std::pair<uint8_t, uint8_t> current_node = m_stack.top();
            m_stack.pop();

            current_row = &(current_node.first);
            current_column = &(current_node.second);

            if (m_grid[*current_row][*current_column] != 4)     //mark as visited
            {
                updatePath(*current_row, *current_column);
            }

            for (uint8_t i = 0; i < 4; i++)
            {
                uint8_t neighbour_row = *current_row + direction_row[i];
                uint8_t neighbour_column = *current_column + direction_column[i];

                if ((neighbour_row >= 0) && (neighbour_column >= 0)
                    && (neighbour_row <= m_row - 1) && (neighbour_column <= m_column - 1)
                    && (m_grid[neighbour_row][neighbour_column] == 0))
                {
                    m_stack.push(std::make_pair(neighbour_row, neighbour_column));
                    m_grid[neighbour_row][neighbour_column] = 2;

                    if ((neighbour_row == (m_row - 1)) && (neighbour_column == (m_column - 1)))
                    {
                        std::cout << "GOAL REACHED." << std::endl;
                        m_grid[neighbour_row][neighbour_column] = 4;
                        m_path.push(std::make_pair(neighbour_row, neighbour_column));
                        m_goal_reached = true;
                        break;
                    }
                }
            }
            if (m_goal_reached == true)
            {
                break;
            }
        }

        std::pair<uint8_t, uint8_t>& lastPoint = m_path.top();
        if ((lastPoint.first, lastPoint.second) != (m_row - 1, m_column - 1))
        {
            std::cout << "No Path Found!" << std::endl;
        }
    }
}

void GridSearchDFS::printPath()
{
    if (m_goal_reached)
    {
        if (m_row <= 20 && m_column <= 20)
        {
            for (uint8_t i = 0; i < m_row; i++)
            {
                for (uint8_t j = 0; j < m_column; j++)
                {
                    if (m_grid[i][j] == 4)
                    {
                        std::cout << '-'<<" ";
                    }
                    else if(m_grid[i][j] == 1 )
                    {
                        std::cout << '1'<<" ";
                    }
                    else
                    {
                        std::cout << 'o'<<" ";
                    }
                }
                std::cout << "\n";
            }
        }
        else
        {
            std::cout << "Grid too big to print Solution" << std::endl;
        }
    }
    else
    {
        std::cout << " " << std::endl;
    }

}

void GridSearchDFS::printPathCoordinates()
{
    while(!m_path.empty())
    {
        std::pair<uint8_t, uint8_t>& points = m_path.top();
        std::cout << int(points.first) << "," << int(points.second) << std::endl;
        m_path.pop();
    }
}

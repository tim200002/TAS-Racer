#pragma once

#include <vector>

template<typename T> class Grid{
    public:
    Grid(unsigned int size_x,  unsigned int size_y){
        this->size_x = size_x;
        this->size_y = size_y;
        grid = new T[size_x * size_y];
    }


    Grid(const Grid& oldGrid){
        size_x = oldGrid.size_x;
        size_y = oldGrid.size_y;
        grid = new T[size_x * size_y];
        for(size_t i = 0; i < oldGrid.size_x * oldGrid.size_y; ++i){
            grid[i] = oldGrid.grid[i];
        }
    }


    Grid(unsigned int size_x, unsigned int size_y, std::vector<T> values){
        grid = new T[size_x * size_y];
        this->size_x = size_x;
        this->size_y = size_y;

        for(size_t i = 0; i <values.size(); ++i){
            grid[i] = values[i];
        }
    }

    ~Grid(){
        delete[] grid;
    }

    T get_value(unsigned int x, unsigned int y){
        return grid[y*size_x + x];
    }

    void set_value(unsigned int x, unsigned int y, T value){
        grid[y*size_x + x] = value;
    }

    T* getGrid(){
        return grid;
    }

    unsigned int size_x;
    unsigned int size_y;

    private:
    T* grid;
    
};
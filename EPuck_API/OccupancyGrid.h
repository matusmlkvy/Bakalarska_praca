#ifndef OCCUPANCY_GRID_H_
#define OCCUPANCY_GRID_H_

#include <cstdint>
#include <stdio.h>
#include <string.h>
#include <array>
//#include <vcruntime_string.h>
#include "defs.h"

#define RoundDiv(x, div) (((x) + (div)/2)/(div))

namespace EPuck
{

	class EPUCK_API OccupancyGrid
	{
	public:
		// Creates new occupancy grid (length unit is tick, NED convention)
		OccupancyGrid(int32_t _xmin, int32_t _ymin, int32_t _xmax, int32_t _ymax, uint32_t _grid);

		// Destructor
		~OccupancyGrid();

		// Adds measurement at given coordinates. Returns false when out of bounds.
		bool set(int32_t x, int32_t y, float val);

		// Returns occupancy at given coordinates
		float get(int32_t x, int32_t y) const;

		// Returns boudaries of the grid
		int32_t xMin() const { return xmin; }
		int32_t yMin() const { return ymin; }
		int32_t xMax() const { return xmax; }
		int32_t yMax() const { return ymax; }

		// Returns grid size
		uint32_t gridSize() const { return grid; }

		// Assign operator. Maps one grid to another
		const OccupancyGrid& operator=(const OccupancyGrid& g);

		// Casting constructor
		template <size_t ROW, size_t COL>
		OccupancyGrid(int32_t _xmin, int32_t _ymin, uint32_t _grid, const std::array<std::array<int, COL>, ROW>& _data)
		{
			xmin = _xmin;
			ymin = _ymin;
			xrange = _grid * ROW;
			yrange = _grid * COL;

			xmax = _xmin + xrange;
			ymax = _ymin + yrange;
			rows = ROW;
			cols = COL;
			grid = _grid;

			data = new float[rows * cols];
			
			for (size_t r = 0; r < rows; r++)
				for (size_t c = 0; c < cols; c++)
					data[c + cols * r] = 1.0 - (float)_data[r][c];

		}

	private:
		int32_t xmin;
		int32_t xmax;
		int32_t ymin;
		int32_t ymax;
		uint32_t grid;
		uint32_t xrange;
		uint32_t yrange;
		uint32_t rows;
		uint32_t cols;

		float* data;

		// returns index corresponding to the coordinates
		int32_t index(int32_t x, int32_t y) const;
	};

}

#endif // OCCUPANCY_GRID_H_
#include "OccupancyGrid.h"
#include <utility>

namespace EPuck 
{

	// Creates new occupancy grid (length unit is tick, NED convention)
	OccupancyGrid::OccupancyGrid(int32_t _xmin, int32_t _ymin, int32_t _xmax, int32_t _ymax, uint32_t _grid)
	{
		if (_xmin > _xmax)
			std::swap(_xmin, _xmax);

		if (_ymin > _ymax)
			std::swap(_xmin, _xmax);

		xmin = _xmin;
		xmax = _xmax;
		ymin = _ymin;
		ymax = _ymax;

		xrange = xmax - xmin;
		yrange = ymax - ymin;
		rows = xrange / grid + 1;
		cols = yrange / grid + 1;

		data = new float[rows * cols];
	}

	OccupancyGrid::~OccupancyGrid()
	{
		delete[] data;
		data = NULL;
		rows = 0;
		cols = 0;
	}

	// Adds measurement at given coordinates. Returns false when out of bounds.
	bool OccupancyGrid::set(int32_t x, int32_t y, float val)
	{
		int32_t idx = index(x, y);
		if (idx < 0)
			return false;

		data[idx] = val;
		return true;
	}

	// Returns occupancy at given coordinates
	float OccupancyGrid::get(int32_t x, int32_t y) const
	{
		int32_t idx = index(x, y);
		if (idx < 0)
			return 0.0;

		return data[idx];
	}

	// Assign operator. Maps one grid to another
	const OccupancyGrid& OccupancyGrid::operator=(const OccupancyGrid& g)
	{
		if (this == &g)
			return *this;

		if (xmin == g.xmin && xmax == g.xmax && ymin == g.ymin && ymax == g.ymax && grid == g.grid)
		{
			// direct copy of data
			memcpy(data, g.data, rows * cols * sizeof(float));
			return *this;
		}

		// point-to-point assign
		uint32_t idx = 0;
		for (uint32_t kx = 0; kx < g.rows; kx++)
		{
			int32_t x = xmin + kx * grid;
			for (uint32_t ky = 0; ky < g.cols; ky++)
			{
				int32_t y = ymin + ky * grid;
				set(x, y, g.data[idx++]);
			}
		}
		return *this;
	}


	// returns index corresponding to the coordinates
	int32_t OccupancyGrid::index(int32_t x, int32_t y) const
	{
		int32_t kx = RoundDiv(x - xmin, (int32_t)grid);
		int32_t ky = RoundDiv(y - ymin, (int32_t)grid);

		if (kx < 0 || kx >= (int32_t)cols || ky < 0 || ky >= (int32_t)rows)
			return -1;

		// x-row-first
		return (kx + ky * cols);
	}

}
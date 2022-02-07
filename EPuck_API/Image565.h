#ifndef IMAGE565_H_
#define IMAGE565_H_

#include <vector>
#include <stdexcept>
#include "pixel.h"

// Disable enforcing DLL interface for private members
#pragma warning(push)
#pragma warning(disable: 4251)

namespace EPuck
{
	class EPUCK_API Image565
	{
	private:
		std::vector<Pixel565>		itsPixels;
		int							itsWidth;
		int							itsHeight;

	public:
		Image565(int width = 0, int height = 0)
			:itsPixels(width*height*sizeof(Pixel565)),
			itsWidth(width),
			itsHeight(height) {}


		Image565(const Image565& im)
			:itsPixels(im.itsPixels),
			itsWidth(im.itsWidth),
			itsHeight(im.itsHeight) {}

		~Image565()
		{}

		// Fills image from raw data.
		void Fill(const Pixel565* src, int width, int height)
		{
			this->itsWidth = width;
			this->itsHeight = height;
			this->itsPixels.assign(src, src + width * height);
		}

		// Returns pixel at given position
		Pixel565& Pixel(int x, int y)
		{
			if (x >= this->itsWidth || y >= this->itsHeight)
				throw std::out_of_range("Index out of bounds.");

			return this->itsPixels[y * this->itsWidth + x];
		}

		// Returns count of pixels of the image
		int size() const { return this->itsWidth * this->itsHeight; }

		// Returns width of the image
		int width() const { return this->itsWidth; }

		// Returns height of the image
		int height() const { return this->itsHeight; }

		// Returns C-style data buffer
		Pixel565* data() { return this->itsPixels.data(); }
	};

}

#pragma warning(pop)

#endif /* IMAGE565_H_*/

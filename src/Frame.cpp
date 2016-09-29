/*
 * Frame.cpp
 *
 *  Created on: Feb 19, 2009
 *      Author: njoubert
 *      Modified: sidch
 */

#include "Frame.hpp"

Frame::Frame(int w, int h)
: image(w, h, 3)
{
}

Frame::~Frame()
{
}

void
Frame::setColor(Sample const & s, RGB c)
{
  c.clip(0, 1);

  int xi = (int)std::floor(s.x() * image.width());
  int yi = (int)std::floor((1 - s.y()) * image.height());
  unsigned char * pixel = image.pixel(yi, xi);
  pixel[0] = c.getBMPR(0, 1);
  pixel[1] = c.getBMPG(0, 1);
  pixel[2] = c.getBMPB(0, 1);
}

void
Frame::save(std::string const & path)
{
	// Image final(image.width(), image.height(), 3);
	// 	unsigned char * pixel;
	// 	  unsigned char * pixel1;
	// 	  unsigned char * pixel2;
	// 	  unsigned char * pixel3;
	// 	  unsigned char * pixel4;
	// 	  unsigned char* finalpixel;

	// for (int xi = 0; xi < image.width(); ++xi)
	// {
	// 	for (int yi = 0; yi < image.height(); ++yi)
	// 	{
	// 		finalpixel = final.pixel(yi, xi);

	// 	  pixel = image.pixel(yi, xi);
	// 	  if(yi < (image.height()-1)){
	// 	  pixel1 = image.pixel(yi+1, xi);}
	// 	  else
	// 	  {
	// 	  	pixel1 = image.pixel(yi, xi);
	// 	  }
	// 	  if(yi > 0){
	// 	  pixel2 = image.pixel(yi-1, xi);}
	// 	    else
	// 	  {
	// 	  	pixel2 = image.pixel(yi, xi);
	// 	  }

	// 	  if(xi < (image.width()-1)){
	// 	  pixel3 = image.pixel(yi, xi+1);}
	// 	    else
	// 	  {
	// 	  	pixel3 = image.pixel(yi, xi);
	// 	  }

	// 	  if(xi > 0){
	// 	  pixel4 = image.pixel(yi, xi-1);}
	// 	    else
	// 	  {
	// 	  	pixel4 = image.pixel(yi, xi);
	// 	  }
	// 	   finalpixel[0] = (pixel[0] + pixel1[0] + pixel2[0] + pixel3[0] + pixel4[0])/5;
	// 	   finalpixel[1] = (pixel[1] + pixel1[1] + pixel2[1] + pixel3[1] + pixel4[1])/5;
	// 	   finalpixel[2] = (pixel[2] + pixel1[2] + pixel2[2] + pixel3[2] + pixel4[2])/5;
	// 	}
	// }
  if (!image.save(path))
    std::cerr << "Could not save frame to " << path << std::endl;
}

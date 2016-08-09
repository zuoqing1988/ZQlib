#ifndef _ZQ_GAUSSIAN_PYRAMID_H_
#define _ZQ_GAUSSIAN_PYRAMID_H_

#pragma once

#include "ZQ_DoubleImage.h"
#include <vector>
#include <iostream>

namespace ZQ
{
	template<class T>
	class ZQ_GaussianPyramid
	{
	private:
		std::vector<ZQ_DImage<T>> ImPyramid;
		int nLevels;
		double ratio;
	public:
		ZQ_GaussianPyramid(void){nLevels = 0; ratio = 0;}
		~ZQ_GaussianPyramid(void){ImPyramid.clear();}
		double ConstructPyramid(const ZQ_DImage<T>& image, const double ratio = 0.5, const int minWidth = 16);
		int nlevels() const {return nLevels;};
		ZQ_DImage<T>& Image(int index) { return ImPyramid[index]; };
	};


	/*********************************************************************************/
	/********************************** definitions **********************************/
	/*********************************************************************************/

	template<class T>
	double ZQ_GaussianPyramid<T>::ConstructPyramid(const ZQ_DImage<T> &image, const double ratio, const int minWidth)
	{
		
		if(ratio > 0.9 || ratio < 0.4)
			this->ratio = 0.5;
		else
			this->ratio = ratio;

		// first decide how many levels
		nLevels = log((double)minWidth/image.width())/log(this->ratio)+1;
		ImPyramid.clear();
		for(int i = 0;i < nLevels;i++)
		{
			ZQ_DImage<T> tmp;
			ImPyramid.push_back(tmp);
		}
		ImPyramid[0].copyData(image);
		double baseSigma = (1.0/this->ratio-1);
		int n = log(0.25)/log(this->ratio);
		double nSigma=baseSigma*n;
		for(int i = 1;i < nLevels;i++)
		{
			ZQ_DImage<T> foo;
			if(i <= n)
			{
				double sigma = baseSigma*i;
				image.GaussianSmoothing(foo,sigma,sigma*3);
				foo.imresizeBicubic(ImPyramid[i],pow(this->ratio,i));
				//foo.imresize(ImPyramid[i],pow(ratio,i));
			}
			else
			{
				ImPyramid[i-n].GaussianSmoothing(foo,nSigma,nSigma*3);
				double rate = (double)pow(this->ratio,i)*image.width()/foo.width();
				foo.imresizeBicubic(ImPyramid[i],rate);
				//foo.imresize(ImPyramid[i],rate);
			}
		}
		return this->ratio;
	}
}




#endif

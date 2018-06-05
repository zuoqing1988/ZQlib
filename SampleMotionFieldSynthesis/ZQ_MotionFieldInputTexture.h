#ifndef _ZQ_MOTION_FIELD_INPUT_TEXTURE_H_
#define _ZQ_MOTION_FIELD_INPUT_TEXTURE_H_
#pragma once

#include "ZQ_DoubleImage.h"

class ZQ_MotionFieldInput
{
public:
	virtual bool TranslateToMotionField(ZQ::ZQ_DImage<float>& field) = 0;
};


class ZQ_MotionFieldInputTexture : ZQ_MotionFieldInput
{
public:
	ZQ_MotionFieldInputTexture();
	~ZQ_MotionFieldInputTexture();

private:
	ZQ::ZQ_DImage<float> texture;

public:
	void SetTexture(const ZQ::ZQ_DImage<float>& tex){texture = tex;}

	bool TranslateToMotionField(ZQ::ZQ_DImage<float>& field);
};


ZQ_MotionFieldInputTexture::ZQ_MotionFieldInputTexture()
{

}

ZQ_MotionFieldInputTexture::~ZQ_MotionFieldInputTexture()
{

}

bool ZQ_MotionFieldInputTexture::TranslateToMotionField(ZQ::ZQ_DImage<float>& field)
{
	int width = texture.width();
	int height = texture.height();
	int nChannels = texture.nchannels();
	if (nChannels != 1)
		return false;

	ZQ::ZQ_DImage<float> gx, gy;

	texture.dx(gx, true);
	texture.dy(gy, true);

	field.allocate(width, height, 2);

	float* gx_data = gx.data();
	float* gy_data = gy.data();
	float* field_data = field.data();

	for (int h = 0; h < height; h++)
	{
		for (int w = 0; w < width; w++)
		{
			field_data[(h*width + w) * 2 + 0] = -gy_data[h*width + w];
			field_data[(h*width + w) * 2 + 1] = gx_data[h*width + w];
		}
	}

	return true;
}

#endif
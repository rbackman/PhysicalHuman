#pragma once

#include "util_serializable.h"
#include <gsim/gs_image.h>

class DepthImage :public Serializable
{
	GsImage* _image;

};
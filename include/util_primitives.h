# pragma once

#include <gsim/gs_array.h>

class Curve;
class GsVec2;


void drawLine(GsVec2 a, GsVec2 b);
int drawCurve(Curve* curve);
void drawCircle(float rad,GsVec2 pos);
void drawPolyline(GsArray<GsVec2> poly);
void drawPoints(GsArray<GsVec2> poly);
void drawRec(GsVec2 c,GsVec2 d);
void drawPoint(GsVec2 p);
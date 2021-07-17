#include "util_curve.h"

Curve::~Curve()
{

	delete _line;

}

Curve::Curve()
{
	_line = new SnLines();
	_line->visible(true);
	cCol = GsColor::red;
	cpCol= GsColor::green;
	curveMode = BEZIER; //BSPLINEQUAD; //BEZIER;

	selection = 9999;
	divPerPair = 3;
	closed = 0;
	selectionState = 0;
	controlPoly = 1;
	T=1;
	first = 1;
	vis = true;
}




int Curve::addPoint(GsVec pt)
{
   p.push(pt);
   return p.size();
}
void Curve::clear()
{
	p.remove(0,p.size());
	c.remove(0,c.size());
	_line->init();
}
void Curve::update()
{
	switch(curveMode)
	{
	
		case BEZIER:
			c.remove(0,c.size());
			for(float t=0;t<1;t+=0.01f)
			{
				c.push(eval_bezier(t));
			}
		break;
		case BSPLINECUBIC:
			c.remove(0,c.size());
			
			for ( float t =3; t<=T*(float)p.size(); t+=0.01f )
			{
				c.push(eval_bspline(t,3));
			}
			
		break;
		case BSPLINEQUAD:
			c.remove(0,c.size());
			
			for ( float t =2; t<=T*(float)p.size(); t+=0.01f )
			{
				c.push(eval_bspline(t,2));
			}
			
		break;

	}
	

	_line->init();
	_line->begin_polyline();
	_line->color(cCol);
	
	if(curveMode == LINEAR)
	{
		for(int j=0;j<p.size();j++)
		{
			_line->push(p.get(j));
		}
	}
	else
	{
		for(int j=0;j<c.size();j++)
		{
			_line->push(c.get(j));
		}
	}

	

	 _line->end_polyline ();
	

}


GsVec Curve::eval_lagrange ( float t)
{
int n = p.size()-1;
t = t * n; //t 0 -> n
GsVec pt;

for(int i=0; i <= n; i++)
	{
		float bi=1.0;
		for ( int j=0; j <= n; j++ )
		{
		
				if(i==j){/*skip this */}
				else
				{
					bi*=(t-j)/(i-j);
				}
		}
		pt = p[i]*(float)bi + pt;
	}

return pt;
}


float C(int n, int i)
{
return (float)(fact(n) / ( fact(i) * fact(n-i) ));
}

float B(int n, int i, float t)
{
return C(n,i)*pow(t,i)*pow(1-t,n-i);
}

GsVec Curve::eval_bezier ( float t)
{
int n = p.size();
GsVec pt;
	for(int i=0; i<n; i++)
	{
		pt = p[i]*B(n-1,i,t) + pt;
	}
return pt;
}

float N ( int i, int k, float u )
 {
	float ui=float(i);
	if ( k==1 ) 
	 return ui<=u && u<ui+1? 1.0f:0;
	else  
	return   ((u-ui)/(k-1)) * N(i,k-1,u) + ((ui+k-u)/(k-1)) * N(i+1,k-1,u);
 }


GsVec Curve::eval_bspline ( float u,int k )
{
	//for cubic start u at 3
	//knot vector [0,1,2,3,4...10] 
	//t>[0,1] --> U>[3,7] = u = 4*t+3
	int n = p.size()-1;
	GsVec pt;
	for(int i=0; i <= n; i++)
	{
		pt = p[i]*N(i,k+1,u) + pt;
	}
return pt;
}


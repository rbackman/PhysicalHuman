
#include "util_trajectory.h"
#include "util_primitives.h"

#include "util.h"
Trajectory::~Trajectory()
{
	init();
}
Trajectory::Trajectory(trajectory_type type)
{
	selectionState = CURVE_NOT_SELECTED;
	tangSelected = CURVE_TANG_PI;
	selection = 0;
	lastPhase=0;
	curve_color = GsColor::blue;
	curve_width = 1;
	show_tangents = true;
	show_control_points = true;

	complement=0;
	_curveType = type;
	loopContinuity = false;
	flip = false;
	rest = 0;
	duration = 1.0f;
	dt =0.01f;
}



# include <fltk/gl.h>


GsVec2 Trajectory::getPiPos(int i)
{
	GsVec2 pt = p[i] ;
	if (i>0)
	{
		if(abs(t[i] - STRAIGHT_TNG)<0.01f)
		{
			
			pt = interp(p[i-1],p[i],0.66f);
		}
		else if(abs(t[i] - FREE_TNG)<0.01f)
		{
			if(i>0 && i < t.size()-1)
			{
				GsVec dp = p[i-1] - p[i+1];
				dp.len(abs(p[i-1].x - p[i].x)*0.5f);
				pt = pt + dp;
			}
		}
		else
		{
			float tg = t[i-1];
			if(t[i-1]<0)
				pt = pt - GsVec2((p[i].x - p[i-1].x)*0.5f,0.0f);
			else
				pt = pt - GsVec2((p[i].x - p[i-1].x)*(1.0f - t[i-1]),0.0f);
		}
	}
	return pt;
}
GsVec2 Trajectory::getPoPos(int i)
{

	GsVec2 pt = p[i] ;

	if (i<p.size()-1)
	{
		if((abs(t[i] - STRAIGHT_TNG)<0.01f))
		{
			pt = interp(p[i],p[i+1],0.33f);
		}
		else if(abs(t[i] - FREE_TNG)<0.01f)
		{
			if(i>0 && i < t.size()-1)
			{
				GsVec dp = p[i-1] - p[i+1];
				dp.len(abs(p[i].x - p[i+1].x)*0.5f);
				pt = pt - dp;
			}
		}
		else
		{
			pt = pt + GsVec2((p[i+1].x - p[i].x)*t[i],0.0f);
		}
	}
	return pt;
}
void Trajectory::draw()
{
	GsArray<GsVec2> pts;
	//updateTangents();
	glTranslate(GsVec(0.0f,-rest,0.0f));

	if(_curveType==TRAJ_BEZIER)
	{	

		if(show_control_points)
		{
			
		glColor(point_color);
		glPointSize((float)point_size);
		drawPoints(p);
		if(selectionState == CURVE_POINT_SELECTED)
		{
			glColor(GsColor::green);
			glPointSize((float)point_size+8.0f);
			drawPoint(p.get(selection));
		}
		}
	//	drawPolyline(p);

		if(show_tangents)
		{
			if(selectionState != CURVE_NOT_SELECTED)
			{
				pts.push(getPiPos(selection));
				pts.push(getPoPos(selection));
				glColor(point_color);
				glPointSize((float)point_size);
				drawPoints(pts);
			}
			
		}
		
	}
	else if(_curveType==TRAJ_STEP)
	{
		drawPoints(p);
		
		glColor3f ( 0,1,0);
		drawPoints(pts);
	}
	glColor(curve_color);
	glLineWidth((float)curve_width);
	drawPolyline(c);

	glTranslate(GsVec(0.0f,rest,0.0f));
}
void Trajectory::init()
{
	t.size(0);
	p.size(0);
	c.size(0);
	_sample_limits.size(0);
}
void Trajectory::removePoint(int j)
{
	for (int i=0;i<_sample_limits.size();i++)
	{
		if(_sample_limits[i].index == j)
		{
			_sample_limits.remove(i);
		}
	}
	if(j<t.size())t.remove(j);
	p.remove(j);
	update();
}


int Trajectory::numSamplePoints()
{
	return _sample_limits.size();
}

sample_data* Trajectory::sample( int i )
{
	return &_sample_limits[i];
}

void Trajectory::addSample( sample_data p )
{
	_sample_limits.push(p);
}

void Trajectory::removeSample( int sampleIdx )
{
	_sample_limits.remove(sampleIdx);
}
bool Trajectory::press(GsVec2 pt)
{
	pt.y+=rest;
	for (int it=0; it<p.size(); it++ )
	{
		if ( dist(p.get(it), pt )<= 0.1f )
		{

			if(selectionState==CURVE_REMOVE_POINT)
			{
				if(it>0 && it<p.size())
				{
					removePoint(it);
					return true;
				}
			}
			else if(selectionState==CURVE_MOVE_POINT)
			{
			
				//printf("pt %d selected\n",it);
				selectionState = CURVE_POINT_SELECTED;
				selection = it;
				return true;
			}

			else if(selectionState == CURVE_STRAIGHTEN_TANGENT&&_curveType==TRAJ_BEZIER)
			{
				t[it] = STRAIGHT_TNG;
				return true;
			}
			else if(selectionState == CURVE_FLATTEN_TANGENT&&_curveType==TRAJ_BEZIER)
			{
				t[it] = FLAT_TNG;
				return true;
			}
			else if(selectionState == CURVE_FREE_TANGENT &&_curveType==TRAJ_BEZIER)
			{
				t[it] = FREE_TNG;
				return true;
			}
			else if(selectionState == CURVE_EDIT_BOUNDS)
			{
				selection = it;
				phout<<" bounds select time\n";
			}
		
		}
	}
	

	if(selectionState==CURVE_ADD_POINT)
	{
		selection = addPoint(pt);
		selectionState = CURVE_POINT_SELECTED;
		return true;
	}
	else
	{
		if(_curveType==TRAJ_BEZIER)
		{
			if ( dist( getPiPos(selection) , pt ) <= 0.04f )
			{
				selectionState = CURVE_TANG_SELECTED;
				tangSelected = CURVE_TANG_PI;
				return true;
			}
			if ( dist(getPoPos(selection), pt ) <= 0.04f )
			{
				selectionState = CURVE_TANG_SELECTED;
				tangSelected = CURVE_TANG_PO;
				return true;
			}
		}
	}
	

	selectionState=CURVE_NOT_SELECTED;
	
	return false;

}

bool Trajectory::drag(GsVec2 pt)
{
	

// 	if(pt.x > MAX_X)pt.x=MAX_X;
 	if(pt.x < MIN_X)pt.x=MIN_X;
// 	if(pt.y > MAX_Y)pt.y=MAX_Y;
// 	if(pt.y < MIN_Y)pt.y=MIN_Y;

	pt.y+=rest;

	switch(selectionState)
	{
	case CURVE_NOT_SELECTED:
		return false;
		break;
	case CURVE_TANG_SELECTED://selState 2 is tangent edit mode
		{

			
			
			//	printf("curve sel pt:%d : tgSel:%d \n",psel,tsel);
			if(tangSelected ==0) break;
			else if (tangSelected==CURVE_TANG_PI)
			{
				//make sure the in tangent is never to the right of the point
				if(pt.x + 0.001f > p[selection].x)
				{
					pt.x = p[selection].x-0.001f;
				}
				if(selection>0)//should only edit in tangent of points that arent the first
				{
					if(t[selection-1]>=0)
					{
						t[selection-1] = (pt.x - p[selection -1].x)/(p[selection].x - p[selection-1].x)   ; 
						if(t[selection]<0)t[selection]=0;
						if(t[selection]>1)t[selection]=1;
					}
				}
			}
			else if(tangSelected==CURVE_TANG_PO)
			{
				//make sure the out tangent is never to the left of the point
				if(pt.x - 0.001f < p[selection].x)
				{
					pt.x = p[selection].x+0.001f;
				}
				if(selection<p.size()-1)//should not edit out tangent of last point
				{
					if(t[selection]>=0)
					{
						t[selection] =  (pt.x - p[selection].x)/(p[selection+1].x - p[selection].x)   ; 
						if(t[selection]<0)t[selection]=0;
						if(t[selection]>1)t[selection]=1;
					}
				}
			}
			
			return true;

		}break;
	case CURVE_POINT_SELECTED: // there is a selection
		{ 
		//	phout<<"selection "<<selection<<" size "<< p.size()<<gsnl;
			p.get(selection) = pt;

				//check if it has moved past the left most vert and swap 
			if(selection>0 && selection <= p.size()-1)
			{
					if( p.get(selection-1).x > p.get(selection).x )
					{
						phout<<"moved to the left so swaping\n";
						GsVec2 temp = p.get(selection-1);
						p.get(selection-1) = p.get(selection);
						p.get(selection)=temp;	
						if(_curveType==TRAJ_BEZIER)
						{
							float tg = t[selection];
							t[selection] = t[selection-1];
							t[selection-1] = tg;
						}
						selection--;

					}
			}
			
			if(selection>=0 && selection < p.size()-1)
			{
					if( p.get(selection+1).x < p.get(selection).x)
					{
						phout<<"moved to the right so swaping\n";
						GsVec2 temp = p.get(selection+1);
						p.get(selection+1) = p.get(selection);
						p.get(selection)=temp;
						if(_curveType==TRAJ_BEZIER)
						{
							float tg = t[selection];
							t[selection] = t[selection+1];
							t[selection+1] = tg;
						}
						selection++;
					}

			}
			
			
			return true;

		}break;
	}
	return false;
}
int Trajectory::addPoint(GsVec2 pt,float pi)
{
	int idx = addPoint(pt);

	if(_curveType == TRAJ_BEZIER)
	{
		if (idx<t.size())
		{
			t[idx] =  pi;
		}
	}
	else
	{
		phout<<"trying to add weights to a linear curve\n";
	}
	return idx;
}
int Trajectory::addPoint(GsVec2 pt)
{
	//first check to see if there are any points to the right of the new point

	if(_curveType==TRAJ_BEZIER )
	{
		if(p.size()>0)
		{
			for(int i=0;i<p.size();i++)
			{
				if(p.get(i).x > pt.x)
				{

					p.insert(i) = pt;
					t.insert(i) = 0.5f;
				
					for (int j=0;j<_sample_limits.size();j++)
					{
						if(_sample_limits[j].index>=i)
						{
							_sample_limits[j].index++;
						}
					}

					return i;
				}
			}
		}

		//there was nothing to the right so just push in a new point
		p.push(pt);
		t.push(0.5f);

		return p.size()-1;
	}
	else if(_curveType == TRAJ_STEP)
	{
		if(p.size()>0)
		{
			for(int i=0;i<p.size();i++)
			{
				if(p.get(i).x > pt.x)
				{

					p.insert(i) = pt;
				
					return i;
				}
			}
		}

		//there was nothing to the right so just push in a new point but no tangent
		p.push(pt);


		return p.size()-1;
	}
	else if(_curveType == TRAJ_LINEAR)
	{

		if(c.size()>0)
		{
			for(int i=0;i<c.size();i++)
			{
				if(c.get(i).x > pt.x)
				{
					c.insert(i) = pt;
					return i;
				}
			}
		}

		//there was nothing to the right so just push in a new point and tangent
		c.push(pt);
		

		return c.size()-1;
	
	}
	return 0;
}





void Trajectory::update()
{
	if(_curveType == TRAJ_LINEAR)
	{
		return;
	}
	
	if (p.size()==0)
	{
		return;
	}

	
/*	p.get(0).x = MIN_X;
	if(p.size()>1)
	{
		if(p.top().x>MAX_X)
			p.top().x = MAX_X;

	}
	*/

	if(_curveType==TRAJ_STEP)
	{

		for(int i=0;i<p.size();i++)
		{
			if(p[i].y>=0.25f)p[i].y=0.5f;
			if(p[i].y<0.25f)p[i].y=0;
		}
		c.size(0);
		c.push(p[0]);
		for(int i=1;i<p.size();i++)
		{
			if (p[i-1].y != p[i].y)
			{
				if (p[i-1].y==0.5f)
				{
					c.push(GsVec2(p[i].x,0.5f));
					c.push(p[i]);
				}
				else
				{
					c.push(GsVec2(p[i].x,0.0f));
					c.push(p[i]);
				}
			}
			else
			{
				c.push(p[i]);
			}
		}
		c.push(c.top());
		c.top().x = duration;
	}
	else if(_curveType==TRAJ_BEZIER)
	{
		if(loopContinuity && selectionState==CURVE_POINT_SELECTED  )
		{

			bool flipped = false;

			if( selection == 0 )
			{
				if(complement==0)//no complement so set the last y pos to the first for looping
				{
					if(flip )
					{
						flipped = true;
						p.get(p.size()-1).y =  2*rest -p.get(0).y;
					}
					else
						p.get(p.size()-1).y = p.get(0).y;
				}
				else	
				{
					//there is a complement so set its y pos to the first y pos of this traj
					complement->p.get(complement->p.size()-1).y = p.get(0).y;
					complement->update();
				}
			}

			if(selection == p.size()-1 )
			{
				if(complement==0)
				{
					if(flip)
					{
						if(!flipped) p.get(0).y = 2*rest - p.get(p.size()-1).y;
					}
					else
						p.get(0).y = p.get(p.size()-1).y;
				}
				else
				{
					complement->p.get(0).y = p.get(p.size()-1).y;
				}

			}


		}

		c.size(0);

		//start by adding points 0-1 for each pair of control points
		float length = 1.0f;
		if(p.size()==0)
		{
			c.push(GsVec2(0.0f,rest));
			c.push(GsVec2(1.0f,rest));
		}
		else if(p.size()==1)
		{
			c.push(GsVec2(0.0f,p[0].y));
			c.push(GsVec2(1.0f,p[0].y));
		}
		else
		{
			

			for(int i=1;i<p.size();i++)
			{
				GsVec2 p1 = p[i-1];
				GsVec2 p2 = getPoPos(i-1);
				GsVec2 p3 = getPiPos(i);
				GsVec2 p4 = p[i];
				for(float t=0;t<1.0;t+=0.01f)
				{
					c.push(evalBez(t,p1,p2,p3,p4));
				}
			}
		}
		//then refresh the curves with a consistent dt step
		GsArray<GsVec2> cn; /*!curve points*/
		for(float i=0; i<duration ; i += dt)
		{
			float v = getOutput(i);
			//phout<<"output is "<<v<<gsnl;
			cn.push(GsVec2(i,v));
		}
		c.remove(0,c.size());
		c.push(cn);
		c.push(c.top());
		c.top().x = duration;
	}
	//duration = p.top().x;

}
GsVec2 getResult(float tol, float x,GsVec2 p1,GsVec2 p2,GsVec2 p3,GsVec2 p4)
{
	float leftB = 0.0f;
	float rightB = 1.0f;
	float mid;
	GsVec2 output; 
	float error = 1; 

	while(fabs(error)>tol)
	{
		mid = leftB + (rightB - leftB)/2.0f;
		output = evalBez(mid,p1,p2,p3,p4);
		error =  x - output.x;
		phout<<"midpoint: "<<mid<<" error "<<error<<gsnl;
		if(error>0)
			leftB=mid;
		else
			rightB=mid;
	}
	return output;
}
float Trajectory::exactOutput(float x,float tol)
{
	for(int i=0;i<p.size();i++)
	{
		if(p.get(i).x > x)
		{
			//we passed the point so it must be between this and the last
			lastP = getResult(tol,x,p.get(i-1),getPoPos(i-1),getPiPos(i),p.get(i));
		}
	}
	return lastP.y;
}
//this is done in a very inefficient way for now
float Trajectory::getClosestOutput(float x)
{
	if(x<0)x=0;
	if(x>1)x=1;
	lastPhase=x;
//assumming the number of curve points is equal to the timeStep
	int idx = (int)((c.size())*x+2*dt);
	if(idx>=c.size())idx=c.size()-1;
	lastP = c.get(idx);
	return lastP.y;
}

//this is done in a very inefficient way for now
float Trajectory::getOutput(float x)
{
	if(c.size()<=0)
		return rest;

	if(x>c.top().x)
		return c.top().y;

	if(x<c[0].x && c.size()>0)
		return c[0].y;

	for(int i=0;i<c.size();i++)
	{
		if(c.get(i).x > x)
		{
			//we passed the point so it must be between this and the last
			double nx,ny,nt;
			bool hits = gs_lines_intersect(c.get(i-1).x , c.get(i-1).y ,c.get(i).x,c.get(i).y , x,0,x,1,nx,ny,nt);
			if(!hits)
			{
				phout<<"something went wrong with line intersect.. you should really fix this anyway\n";
			}
			lastP.x=(float)nx;
			lastP.y=(float)ny;

			return lastP.y;
		}
	}
	return lastP.y;

}

void Trajectory::mergeControlPoints( float tolerance )
{
	if(_curveType == TRAJ_BEZIER)
	{
		int startNum = p.size();
		for (int i=0;i<p.size()-2;i++)
		{
			GsVec2 currentP = p[i];
			bool merging = true;
			while(merging)
			{
				if(i+1>=p.size())
					merging = false;
				else
				{
					GsVec2 newP = p[i+1];
					if (dist(currentP,newP)<tolerance)
					{
						p.remove(i+1);
					}
					else
						merging = false;
				}
			}
		
			/*
			bool merging = true;
			int startIdx = i;
			i++;
			while(merging)
			{ 
				if (p.size()< 2)
				{
					merging = false;
				}
				else if( i == p.size()-2 && i > 0)
				{
					merging = false;
					if(dist(p[p.size()-1],p[i])<tolerance)
					{
						p.remove(i);
					}
				}
				else if(dist(p[startIdx],p[i])<tolerance)
				{
					p.remove(i);
				}
				else
				{
					merging = false;
				}
	
			}*/

		}

#ifdef DEBUG_PRINT
		//phout<<"  merged  "<<startNum - p.size() <<gsnl;
#endif
		
		update();
	}

}

float Trajectory::avgSlope( int i,int num)
{
	float slope = 0;
	int pts = 0;
	int start =0;
	int end=0;
	for (int j=1;j<num;j++)
	{
		start = i-j;
		end = i+j;
		if (start<0)
		{
			start = 0;
		}
		if (end>c.size()-1)
		{
			end = c.size()-1;
		}
		if (i-j>=0 && i+j<c.size())
		{
			slope += (c[start].y - c[end].y) / (c[start].x - c[end].x);
			pts++;
		}
	}
	if (pts > 0)
	{
		return slope/pts;
	}
	return 0;
}

GsVec2 Trajectory::getPoint( int i )
{
	if(_curveType==TRAJ_LINEAR)
		return c[i];
	else
		return p[i];
}

int Trajectory::numPoints()
{
	if(_curveType==TRAJ_LINEAR)
		return c.size();
	else
		return p.size();
}

float Trajectory::getTangent( int i )
{
	if(i<t.size())
		return t[i]; 
	
	return 0;
}

void Trajectory::setPoint( int idx, GsVec2 np )
{
	if(_curveType==TRAJ_LINEAR)
		return c[idx] = np;
	else
		return p[idx] = np;
}

trajectory_type Trajectory::getCurveType()
{
	return _curveType;
}

void Trajectory::setCurveType( trajectory_type t )
{
	show_tangents = false;
	show_control_points = false;
	if(_curveType == TRAJ_BEZIER)
	{
	
		show_tangents = true;
		show_control_points = true;

		c.size(0);
		c.push(p);
	}
	else if(_curveType == TRAJ_STEP)
	{
		show_control_points = true;
	}
	_curveType = t;
}

int Trajectory::numCurvePoints()
{
	return c.size();
}

GsVec2 Trajectory::getCurvePoint( int i )
{
	return c[i];
}

void Trajectory::setTangent( int j, float wt )
{
	if(_curveType==TRAJ_BEZIER)
	{
		if(j<t.size())
			t[j] = wt;
	}
}

void Trajectory::setRange( float mn,float rst, float mx )
{
	max = mx;
	min = mn;
	rest = rst;
}

void Trajectory::randomize()
{
	for(int j=0;j<_sample_limits.size();j++)
	{
		sample_data p = _sample_limits.get(j);

		GsVec2 np = GsVec2( 
			gs_random(p.rest.x + p.min.x,p.rest.x +p.max.x),
			gs_random(p.rest.y +p.min.y,p.rest.y + p.max.y));
		setPoint(p.index,np);
	}
}

void Trajectory::removeSamples()
{
	_sample_limits.size(0);
}

sample_data Trajectory::getSample( int i )
{
	return _sample_limits[0];
}

void Trajectory::scale( float v,bool scaleY )
{
	duration = v*duration;
	if(_curveType == TRAJ_LINEAR)
	{
		for (int i=0;i<c.size();i++)
		{
			c[i].x *= v;
			if(scaleY)
				c[i].y *= v;
		}
	}
	else
	{
		for (int i=0;i<p.size();i++)
		{
			p[i].x *= v;
			if(scaleY)
				p[i].y *= v;
		}
	}
	update();
}

GsArray<GsVec2>* Trajectory::getCurveArray()
{
	return &c;
}

int Trajectory::numTangents()
{
	return t.size();
}







float BezStuff(int n, int i, float t)
{
	return (float)(fact(n) / ( fact(i) * fact(n-i) ))*pow(t,i)*pow(1-t,n-i);
}

GsVec2 evalBez ( float t, GsVec2 p1, GsVec2 p2, GsVec2 p3, GsVec2 p4)
{
	GsVec2 pt;

	pt	+= p1*BezStuff(3,0,t);
	pt	+= p2*BezStuff(3,1,t);
	pt	+= p3*BezStuff(3,2,t);
	pt	+= p4*BezStuff(3,3,t);

	return pt;
}



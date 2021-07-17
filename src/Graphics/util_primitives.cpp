

# include "util_primitives.h"
#include <math.h>
#include "common.h"

#include "util_curve.h"
# include <fltk/gl.h>
#include <vector>


void drawLine(GsVec2 a, GsVec2 b)
{
	glBegin ( GL_LINES);	
	glVertex2f (a.x, a.y);
	glVertex2f (b.x, b.y);
	glEnd();
}

void drawCircle(float rad,GsVec2 pos)
{
	glBegin ( GL_LINE_STRIP );
	for	   ( unsigned i=0; i<=360; i+=10 ) 
	{
		glVertex2f ( pos.x+rad*cos(GS_TORAD(i)), pos.y+rad*sin(GS_TORAD(i)));
	}
	glEnd();
}
void drawPolyline(GsArray<GsVec2> poly)
{	
	glBegin ( GL_LINE_STRIP );
	for	   ( int i=0; i<poly.size(); i++ ) {
		glVertex2f ( poly.get(i).x, poly.get(i).y );
	}
	glEnd();
}
void drawPoints(GsArray<GsVec2> poly)
{
	glBegin ( GL_POINTS );

	for	   ( int i=0; i<poly.size(); i++ ) 
	{
		glVertex2f (poly.get(i).x, poly.get(i).y  );			
	}
	glEnd();
}
void drawPoint(GsVec2 p)
{
	glBegin ( GL_POINTS );
	glVertex2f (p.x, p.y  );			
	glEnd();
}
void drawRec( GsVec2 c,GsVec2 d )
{
	float z =0;
	glBegin ( GL_QUADS );
	d = d/2.0f;
	GsVec2 tl; tl.x   = c.x - d.x;   tl.y = c.y+d.y;
	GsVec2 tr; tr.x   = c.x + d.x;   tr.y = c.y+d.y;
	GsVec2 ll; ll.x   = c.x - d.x;   ll.y = c.y-d.y;
	GsVec2 lr; lr.x   = c.x + d.x;   lr.y = c.y-d.y;


	glNormal3f ( 0, 0, 1 );
	glVertex3f ( tl.x, tl.y, z );
	glVertex3f (tr.x, tr.y, z);
	glVertex3f ( lr.x, lr.y, z );
	glVertex3f ( ll.x, ll.y, z );


	glEnd ();
}

/*int drawCurve(Curve* c)
{
	unsigned i;


	if(c->curveMode==BEZIERPIECES)
	{
		drawCurvePieces(c);
		glPointSize ( 10 );
		glColor3f ( 1,0,0);
		glBegin ( GL_POINTS );
		for ( i=0; i<c->p.size(); i++ )
			glVertex2f ( c->p.get(i).x, c->p.get(i).y );
		glEnd();

		for ( i=0; i<c->p.size(); i++ ) 
		{

			if( c->selectionState && i ==c->selection)
			{
				GsVec pi = c->t.get(i)->pi;
				GsVec po = c->t.get(i)->po;

				glPointSize ( 6 );
				glBegin ( GL_POINTS );
				glColor3f ( 0,0,1);
				glVertex3f ( pi.x,pi.y,pi.z  );
				glVertex3f ( po.x, po.y ,po.z );
				glEnd();
			}
		}
		for ( i=0; i<c->p.size(); i++ ) 
		{
			if( c->selectionState && i ==c->selection)
			{
				GsVec pi = c->t.get(i)->pi;
				GsVec po = c->t.get(i)->po;
				GsVec p = c->p.get(i);

				glBegin(GL_LINES);
				glVertex3f ( pi.x,pi.y,pi.z );
				glVertex3f ( p.x,p.y,p.z);

				glVertex3f ( po.x, po.y ,po.z );
				glVertex3f ( p.x,p.y,p.z);
				glEnd();
			}
		}
		return BEZIERPIECES;
	}
	else if(c->controlPoly)
	{
		glColor3f ( c->cpCol.r , c->cpCol.g, c->cpCol.b );
		drawPolyline(c->p);
		// draw control points
		glColor3f ( c->cCol.r , c->cCol.g, c->cCol.b );
		glBegin ( GL_POINTS );
		for ( i=0; i<c->p.size(); i++ ) glVertex3f ( c->p.get(i).x, c->p.get(i).y,c->p.get(i).z  );
		glEnd();

	}
	//draw curve

	if(c->p.size()>2)
	{
		float dt =(float)(1.0/(c->divPerPair*(c->p.size()-1)));
		GsArray<GsVec> curve;

		switch(c->curveMode)
		{

		case LEGRANGE:
			for ( float t =0.0; t<=c->T + dt/2; t+=dt )
			{	
				curve.push(c->eval_lagrange(t));
			}
			break;
		case BEZIER:
			for ( float t =0.0; t<=c->T  + dt/2; t+=dt )
			{
				GsVec p = c->eval_bezier(t);
				curve.push(p);
			}
			break;	
		case BSPLINEQUAD:
			for ( float t =2; t<=c->T*c->p.size(); t+=dt )
			{
				curve.push(c->eval_bspline(t,2));
			}

			break;
		case BSPLINECUBIC:
			for ( float t =3; t<=c->T*c->p.size(); t+=dt )
			{
				curve.push(c->eval_bspline(t,3));
			}
			break;
		}
		glColor3f ( 0, 0, 1 );
		drawPolyline(curve);
	}
	else if(c->p.size()==1)
	{
		glColor3f ( 0, 0, 1 );
		drawLine( c->p.get(0) - GsVec(10,0,0) , c->p.get(0) + GsVec(10,0,0) );

	}
	else
	{
		glColor3f ( 0, 0, 1 );
		drawLine( c->p.get(0) - GsVec(5,0,0) , c->p.get(0) );
		drawLine( c->p.get(1) , c->p.get(1) + GsVec(5,0,0) );
		drawLine( c->p.get(0) , c->p.get(1));

	}


	return 1;		
}
*/
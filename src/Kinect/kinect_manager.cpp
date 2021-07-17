
#include "kinect_manager.h"
#include "kinect_marker.h"
#include "util_manipulator.h"
#include <gsim/sn_points.h>
#include "point_cloud.h"
#include <gsim/gs_image.h>

KinectManager::~KinectManager()
{
	for(int i=0;i<_markers.size();i++)
	{
		delete _markers[i];
	}
	_markers.size(0);
}
void KinectManager::applyParameters()
{
	_kinects[0]->use_color = pBool(kinect_manager_use_color);
	_kinects[0]->use_depth = pBool(kinect_manager_use_depth);
	clipBox.set(pVec(kinect_manager_clip_box_position),pFloat(kinect_manager_clip_box_size));
	_clipManip->visible(pBool(kinect_manager_clip_box_visible));
	_clipBoxVis->visible(pBool(kinect_manager_clip_box_visible));
	_clipBoxVis->box(clipBox);
	_clipBoxVis->render_mode(gsRenderModeLines);
	_clipManip->translation(pVec(kinect_manager_clip_box_position));
	_currentCloudPoints->visible(pBool(kinect_manager_draw_output));
	_marker_grp->visible(pBool(kinect_manager_draw_markers));

}
static void manipCallback ( SnManipulator* mnp,const GsEvent& ev, void* udata )
{
	KinectManager* hv = ((KinectManager*)udata);
	Manipulator* manip = (Manipulator*)mnp;
	hv->setClipBox(manip->globalPosition());

	//manip->evaluate();
}

KinectManager::KinectManager():Serializable("KinectManager")
{
	if(!loadParametersFromFile("../data/kinect/default.cfg"))
	{
		phout<<"couldn't find kinect file\n";
	}
	CHECK_STRING(kinect_manager_marker_types);
	CHECK_STRING(kinect_manager_marker_list);
	CHECK_BOOL(kinect_manager_use_clip_box);  /*bool only consider points in a box*/
	CHECK_FLOAT(kinect_manager_clip_box_size); /*vec size of clip box*/
	CHECK_VEC(kinect_manager_clip_box_position); /*vec position of clip box*/
	CHECK_FLOAT(kinect_manager_marker_max_v); /* maximum velocity a finger can move in one frame*/
	CHECK_FLOAT(kinect_manager_marker_sample); /*oversampling for marker position*/
	CHECK_INT(kinect_manager_marker_color_variance); /*int range of color distance to consider for markers*/
	CHECK_BOOL(kinect_manager_use_color); /*bool: use color stream to create mesh*/
	CHECK_BOOL(kinect_manager_use_depth); /*bool: use depth stream to create mesh*/
	CHECK_BOOL(kinect_manager_use_all_points); /*bool: force all points to be considered for debuging*/
	CHECK_BOOL(kinect_manager_track_colors); /*bool: use markers*/
	CHECK_BOOL(kinect_manager_draw_markers);
	CHECK_BOOL(kinect_manager_draw_mesh); /*bool: update SnColorSurface*/
	CHECK_BOOL(kinect_manager_draw_output); /*bool: show final pointcloud after processing*/
	CHECK_BOOL(kinect_manager_hand_proximity); /*bool: limit pointcloud processing to near the hand marker*/
	CHECK_FLOAT(kinect_manager_hand_dist); /*float max distance from hand to consider*/
	CHECK_FLOAT(kinect_manager_range);
	CHECK_FLOAT(kinect_manager_cutoff);
	CHECK_FLOAT(kinect_manager_angle);
	CHECK_INT(kinect_manager_hand_color_variance); /*int color variance for hand*/
	CHECK_BOOL(kinect_manager_hand_color_proximity); /*bool use color variance or not*/
	CHECK_BOOL(kinect_manager_clip_box_visible);
	_grp = new SnGroup;
	_new_data = false;
	_currentCloudID = -1;
	_selectedCloudPoints = new SnPoints;
	_image = new GsImage;

	_currentCloudPoints = new SnPoints;
	_clipBoxVis = new SnBox;
	_clipBoxVis->box(clipBox);
	_clipBoxVis->render_mode(gsRenderModeLines);
	_marker_grp = new SnGroup;
	_grp->add(_marker_grp);
	_grp->add(_clipBoxVis);
	_grp->add(_currentCloudPoints);

	_grp->add(_selectedCloudPoints);
	_hand = 0;
	_currentMarker = 0;

	_kinects.push(new Kinect);
	_numKinects = 1;
	_currentKinectID = 0;

	_clipManip = new Manipulator(GsVec(),GsVec(0.06,0.06,0.06) );
	_grp->add(_clipManip);
	_clipManip->callback(manipCallback,this);
	_clipManip->translation(pVec(kinect_manager_clip_box_position));

	applyParameters();
	for(int k=0;k<_numKinects;k++)
	{
		for(float i=0;i<DEPTH_SIZE_X;i++)
		{
			for(float j=0;j<DEPTH_SIZE_Y;j++)
			{
				_kinects[k]->setPointPos((int)i,(int)j,i/DEPTH_SIZE_X,j/DEPTH_SIZE_X,0);
			}
		}
	}


#ifdef NOTDEF
	_markers.push(new KinectMarker("Orange", GsColor(254,89,59)) );

	_markers.push(new KinectMarker("Pink", GsColor(255,2,138)) );

	_markers.push(new KinectMarker("Purple", GsColor(81,11,120)) );

	_markers.push(new KinectMarker("Red", GsColor(224,1,60)) );

	_markers.push(new KinectMarker("Green", GsColor(139,241,129)) );


	for(int i=0;i<_markers.size();i++)
	{

		mainwin->ui_viewer->_fingerGroup->add(_markers.get(i)->model->getGrp());
	}
#endif

}


float KinectManager::minDepth()
{
	return pFloat(kinect_manager_range,0);
}
float KinectManager::maxDepth()
{
	return pFloat(kinect_manager_range,1);
}
float KinectManager::cutoff()
{
	return pFloat(kinect_manager_range,2);
}

void KinectManager::update()
{
	min_x = 0;
	min_y = 0;
	max_x = DEPTH_SIZE_X;
	max_y = DEPTH_SIZE_Y;

	if(_kinects.size()==0)return;

	if(_kinects[0]->hasNewData())
	{
		_new_data = true;
		memcpy(points,_kinects[0]->getPoints(),sizeof(point3D)*NUM_POINTS);

		prunePointsFromActivity();

		if(pBool(kinect_manager_use_clip_box))
		{
			prunePointsFromClipBox();
		}
		else
		{
			prunePoints(minDepth(),maxDepth());
		}

		if(pBool(kinect_manager_track_colors))
		{
			for(int f=0;f<_markers.size();f++)
			{
				_markers[f]->points.size(0);
			}

			if(_hand)
			{
				for (int i=min_x;i<max_x;i++)
				{
					for (int j=min_y;j<max_y;j++)
					{
						int id = element(i,j);
						if(points[id].active)
						{
							_hand->check(points[id].col,points[id].pos,pInt(kinect_manager_marker_color_variance));
						}
					}
				}

				_hand->update();

				if(pBool(kinect_manager_hand_proximity))
					prunePoints(_hand->position,pFloat(kinect_manager_hand_dist));

				if(pBool(kinect_manager_hand_color_proximity))
					prunePoints(_hand->getColor(),pInt(kinect_manager_hand_color_variance));

			}


			for(int f=1;f<_markers.size();f++)
			{
				for (int i=min_x;i<max_x;i++)
				{
					for (int j=min_y;j<max_y;j++)
					{
						int id = element(i,j);
						if(points[id].active)
						{
							_markers[f]->check(points[id].col,points[id].pos,pInt(kinect_manager_marker_color_variance));
						}
					}
				}
				_markers.get(f)->update();
			}

			//phout<<"end lock\n";
			//kinect->unlock();
		}

		int width = max_x - min_x;
		int height = max_y - min_y;
		//phout<<"image width "<<width<<" height "<<height<<gsnl;

		_image->init(width,height);

		float min_z = 2000000;
		float max_z = 0;


		for (int i=min_x;i<max_x;i++)
		{
			for (int j=min_y;j<max_y;j++)
			{
				_id = element(i,j);
				if(points[_id].active)
				{
					float dis = points[_id].pos.len();
					if(dis<min_z)min_z = dis;
					if(dis>max_z)max_z = dis;
				}
			}
		}

		float z_dist = max_z-min_z;
		if(z_dist>0)
		{
			for (int i=min_x;i<max_x;i++)
			{
				for (int j=min_y;j<max_y;j++)
				{
					_id = element(i,j);
					if(points[_id].active)
					{
						float dis = (points[_id].pos.len() - min_z)/z_dist;

						_image->ptpixel(height-(j-min_y)-1 ,i-min_x)->set(dis,dis,dis);
					}
					else
					{
						_image->ptpixel(height-(j-min_y)-1 ,i-min_x)->set(0,0,0);
					}
				}
			}
		}
		else
		{

		}



		//_image->vertical_mirror();

		if(pBool(kinect_manager_draw_output))
		{
			_currentCloudPoints->init();
			for (int i=min_x;i<max_x;i++)
			{
				for (int j=min_y;j<max_y;j++)
				{
					int id = element(i,j);
					if(points[id].active)
						_currentCloudPoints->push(points[id].pos,points[id].col,8.0f);

				}
			}

		}

	}

	

}

void KinectManager::makeMarker(GsString nme, GsColor cl, GsVec pos )
{
	_markers.push(new KinectMarker(nme,cl));
	getStringParameter(kinect_manager_marker_list)->val.push(nme);
	getStringParameter(kinect_manager_marker_list)->save = true;
	_markers.top()->position = pos;
	_markers.top()->model->setPosition(pos);
	_markers.top()->sample  = pFloat(kinect_manager_marker_sample);
	_markers.top()->max_v   = pFloat(kinect_manager_marker_max_v);

	if(_markers.size()==1)
	{
		_hand = _markers.top();
	}
	_currentMarker = _markers.top();

	_marker_grp->add(_markers.top()->model->getGrp());

	phout<<"added finger "<<_markers.top()->name()<<" with color "<<cl<<" and position "<<pos<<gsnl;

}

void KinectManager::clearMarkers()
{
	for(int i=0;i<_markers.size();i++)
	{
		delete _markers[i];
	}
	_markers.size(0);
	_hand = 0;
}

void KinectManager::deleteLastMarker()
{
	if (_markers.size()>0)
	{
		delete _markers.top();
		_markers.pop();
	}
	if(_markers.size()==0)
		_hand = 0;
}
GsVec KinectManager::getLocalMarkerPosition()
{
	return _fingerStartPos - getFingerPosition();
}
GsVec KinectManager::getFingerPosition()
{
	GsVec p;
	for(int i=0;i<numMarkers();i++)
	{
		p += _markers[i]->position;
	}
	p/=(float)_markers.size();

	p.y = -p.y;
	p = p*0.1f;
	p = limitVec(p,-0.5,0.5);

	return p;
}

void KinectManager::calibrateFingers()
{
	_fingerStartPos = getFingerPosition();

	if(numMarkers()==0 || numMarkers()==1)
		_fingerStartVec.set(1,0,0);

	_fingerStartDist = dist(_markers[0]->position,_markers[1]->position);

	if(numMarkers()==2)
		_fingerStartVec = _markers[1]->position - _markers[0]->position;
	else if(numMarkers()>=3)
	{
		GsVec v1 = _markers[1]->position - _markers[0]->position;
		GsVec v2 = _markers[2]->position - _markers[0]->position;
		v1.normalize();
		v2.normalize();
		_fingerStartVec = cross(v1,v2);
	}

}

GsQuat KinectManager::getOrientation()
{
	//need atleast two fingers for orientation
	if(numMarkers()==0 || numMarkers()==1)
		return GsQuat();

	GsVec fingerVec;

	if(numMarkers()==2)
		fingerVec = _markers[1]->position - _markers[0]->position;
	else if(numMarkers()>=3)
	{
		GsVec v1 = _markers[1]->position - _markers[0]->position;
		GsVec v2 = _markers[2]->position - _markers[0]->position;
		v1.normalize();
		v2.normalize();
		fingerVec = cross(v1,v2);
	}



	fingerVec.normalize();
	GsQuat q(_fingerStartVec,fingerVec);


	float a = -q.angle();
	a *= 0.6f;
	a = limitFloat(a,-gspidiv2,gspidiv2);
	q.set(q.axis(),a);

	return q;
}

KinectMarker* KinectManager::getMarker( int i )
{
	return _markers[i];
}

KinectMarker* KinectManager::getMarker( GsString name )
{
	for (int i=0;i<numMarkers();i++)
	{
		if(getMarker(i)->name()==name)
			return getMarker(i);
	}

	phout<<"couldn't find finger named "<<name<<gsnl;
	return 0;
}

float KinectManager::getFingerVal( int i )
{
	if(_markers.size()<2 || i >= _markers.size()||i<0)return 0;

	return _fingerStartDist - dist(_markers[i]->position,_markers[i+1]->position);
}

int KinectManager::init()
{
	for (int k=0;k<_numKinects;k++)
	{
		_kinects[k]->init();
	}


	return getNumKinects();
}



void KinectManager::selectMarker( GsString selected )
{
	for (int i=0;i<numMarkers();i++)
	{
		if(getMarker(i)->name() == selected)
		{
			_currentMarker = getMarker(i);
			return;
		}
	}
	phout<<"couldn't find finger "<<selected<<gsnl;
	_currentMarker = 0;
}


bool KinectManager::captureHandCloud( float val )
{
	if(_hand)
	{
		_handPoints.size(0);

		PointCloud* p = new PointCloud;
		GsImage* img = new GsImage;
		img->init(_image->w(),_image->h());
		img->buffer() = _image->buffer();
		for (int i=min_x;i<max_x;i++)
		{
			for (int j=min_y;j<max_y;j++)
			{
				_id = element(i,j);
				if(points[_id].active)
				{
					_handPoints.push(points[_id].pos);	
				}
			}
		}
	
		p->setPoints(_handPoints);
		p->setValue(val);
	
		_clouds.push(p);
		_images.push(img);

		_currentCloudID = _clouds.size()-1;

		return true;
	}
	return false;
}
void KinectManager::saveHandCloud(GsString filename)
{
	GsString imagename = filename;
	imagename<<".bmp";
	filename<<".cld";

	PointCloud* p = currentCloud();
	GsImage* img = currentImage();
	
	//phout<<"p file "<<p->toString()<<gsnl;

	GsOutput file;
	p->setFileName(filename);

	if(file.open(filename))
	{
		file<<p->toString();
		file.close();
		img->vertical_mirror();
		img->save(imagename);
		img->vertical_mirror();
		phout<<"saved file "<<filename<<gsnl;
	}
	else
	{
		phout<<"couldn't save files ";
	}
	
}
void KinectManager::selectCloud( int cld )
{
	_selectedCloudPoints->init();
	if(cld<_clouds.size() && cld>=0)
	{
		_currentCloudID = cld;
		PointCloud* p = currentCloud();
		for (int i=0;i<p->points.size();i++)
		{
			//phout<<"adding point "<<p->points[i]<<gsnl;
			_selectedCloudPoints->push(p->points[i],GsColor::red,5.0f);
		}

	}
	else
	{
		phout<<"cloud "<<cld<<" not loaded ";
	}
}
int calcImageDistance(GsImage* i1, GsImage* i2)
{
	if(i1->w()*i1->h() == 0)
		return 200000000;

	int dis = 0;
	int res ;
	GsColor* c1;
	GsColor* c2;
	int i2x;
	int i2y;
	for (int i=0;i<i1->w();i++)
	{
		for (int j=0;j<i1->w();j++)
		{
			c1 = i1->ptpixel(j,i);
			i2x = i*(float)i2->w()/i1->w();
			i2y = j*(float)i2->h()/i1->h();
			c2 = i2->ptpixel(i2y,i2x);
			res = abs(c1->r-c2->r);
			dis += res*res;
		}
	}
	return dis/(i1->w()*i1->h());
}
#include <stdlib.h>
#include <math.h>

void KinectManager::findClosestCloud()
{
	int minid = -1;
	int minval = 20000000;
	
	/*
		PointCloud* p = new PointCloud;
	p->setPoints(_hand->points);
	for (int i=0;i<_clouds.size();i++)
	{
		float dis =	_clouds[i]->check(p);
		//phout<<"dis of cloud "<<i<<" is "<<dis<<gsnl;
		if(dis<minval)
		{
			minval = dis;
			minid = i;
		}

	}
	*/
	for (int i=0;i<_images.size();i++)
	{
		int dis = calcImageDistance(_image,_images[i]);
		if(dis<minval)
		{
			minval = dis;
			minid = i;
		}

	}
	if(minid!=-1)
	{
		phout<<"hand: "<<minid<<"  val: "<<minval<< gsnl;
		
	}
	else
	{
		//phout<<"no good cloud found\n";
		
	}
	_currentCloudID = minid;

}

void KinectManager::initClouds()
{
	for (int i=0;i<_clouds.size();i++)
	{
		delete _clouds[i];
	}
	for (int i=0;i<_images.size();i++)
	{
		delete _images[i];
	}
	_images.size(0);
	_clouds.size(0);
}

void KinectManager::loadCloudFromFile( GsString file )
{
	_clouds.push() = new PointCloud;
	_images.push() = new GsImage;

	_clouds.top()->load(file);
	GsString imagename = file;
	remove_extension(imagename);
	imagename<<".bmp";
	if(!_images.top()->load(imagename))
	{
		phout<<"could not load image "<<imagename<<gsnl;
	}
	else
	{
		_images.top()->vertical_mirror();
	}
}
void KinectManager::startPrune()
{
	_min_x = max_x;
	_min_y = max_y;
	_max_x = min_x;
	_max_y = min_y;
}
void KinectManager::endPrune()
{
	min_x = _min_x;
	min_y = _min_y;
	max_x = _max_x;
	max_y = _max_y;
}
void KinectManager::validPoint(int i,int j)
{
	if(i<_min_x) _min_x = i;
	if(i>_max_x) _max_x = i;
	if(j<_min_y) _min_y = j;
	if(j>_max_y) _max_y = j;
}
void KinectManager::prunePointsFromClipBox()
{
	
	startPrune();

	for (int i=min_x;i<max_x;i++)
	{
		for (int j=min_y;j<max_y;j++)
		{
			_id = element(i,j);
			if(points[_id].active)
			{
				if(!clipBox.contains(points[_id].pos))
				{
					points[_id].active = false;
				}
				else
				{
					validPoint(i,j);
				}
			}

		}
	}

	endPrune();
}

void KinectManager::prunePointsFromActivity()
{
	startPrune();

	for (int i=min_x;i<max_x;i++)
	{
		for (int j=min_y;j<max_y;j++)
		{
			_id = element(i,j);
			if(points[_id].active)
				validPoint(i,j);
		}
	}

	endPrune();
}


void KinectManager::prunePoints( GsVec pos,float rad )
{
	startPrune();

	for (int i=min_x;i<max_x;i++)
	{
		for (int j=min_y;j<max_y;j++)
		{
			_id = element(i,j);
			if(dist(points[_id].pos,pos)>rad)
				points[_id].active=false;
			else
			{
				validPoint(i,j);
			}
		}
	}

	endPrune();

}

void KinectManager::prunePoints( float min,float max )
{
	startPrune();

	for (int i=min_x;i<max_x;i++)
	{
		for (int j=min_y;j<max_y;j++)
		{
			_id = element(i,j);
			if(points[_id].pos.z<min ||points[_id].pos.z>max )
				points[_id].active=false;
			else
			{
				validPoint(i,j);
			}
		}
	}
	endPrune();
}

void KinectManager::prunePoints( GsColor col,int var )
{
	startPrune();

	for (int i=min_x;i<max_x;i++)
	{
		for (int j=min_y;j<max_y;j++)
		{
			_id = element(i,j);
			if( col_dist(col,points[_id].col) >var)
				points[_id].active=false;
			else
			{
				validPoint(i,j);
			}
		}
	}
	endPrune();
}

bool KinectManager::getPointActivity( int i )
{
	return points[i].active;
}

void KinectManager::setPointActive( int x, int y, bool p )
{
	points[element(x,y)].active = p;
}

void KinectManager::setClipBox( GsVec pos )
{
	setP(kinect_manager_clip_box_position,pos);
	clipBox.set(pos,pFloat(kinect_manager_clip_box_size));
	_clipBoxVis->box(clipBox);
	_clipBoxVis->render_mode(gsRenderModeLines);
}

void KinectManager::setClipManip( GsVec pos )
{
	_clipManip->translation(pos);
	setClipBox(pos);
}

void KinectManager::load( GsString filename )
{
	Serializable::setParametersFromFile(filename);
	_marker_grp->remove_all();
	for(int i=0;i<_markers.size();i++)
	{
		delete _markers[i];
	}
	_markers.size(0);

	for (int i=0;i<sizeOfParameter(kinect_manager_marker_list);i++)
	{
		GsString mname = pString(kinect_manager_marker_list,i);
		KinectMarker* m = new KinectMarker(filename,mname);
		if(i==0)
			_hand = m;
		_marker_grp->add(m->model->getGrp());
		_markers.push(m);
		_markers.top()->sample  = pFloat(kinect_manager_marker_sample);
		_markers.top()->max_v   = pFloat(kinect_manager_marker_max_v);

	}
	applyParameters();
}

#include <gsim/gs_image.h>

bool KinectManager::getHandImage(GsImage* img)
{
	img->init(_image->w(),_image->h());
	img->buffer() = _image->buffer();

	
	return true;
}

bool KinectManager::getCloudImage( GsImage* img )
{
	if(_currentCloudID==-1)
		return false;

	GsImage* im = currentImage();
	img->init(im->w(),im->h());
	img->buffer() = im->buffer();

	return true;
}

void KinectManager::saveCloud( int i )
{

	GsString filename = "../data/pointclouds/";
	filename<< getShortFileName() <<"/" <<_clouds[i]->getShortFileName();
	
	GsString imagename = filename;
	imagename<<".bmp";
	filename<<".cld";

	PointCloud* p = _clouds[i];
	GsImage* img = _images[i];

	GsOutput file;
	p->setFileName(filename);

	if(file.open(filename))
	{
		file<<p->toString();
		file.close();
		img->vertical_mirror();
		img->save(imagename);
		img->vertical_mirror();
		phout<<"saved file "<<filename<<gsnl;
	}
	else
	{
		phout<<"couldn't save files ";
	}
}

bool KinectManager::hasNewData()
{
	if(_new_data)
	{
		_new_data = false;
		return true;
	}
	return false;
}




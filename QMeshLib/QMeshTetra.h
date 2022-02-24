///////////////////////////////////////////////////////////////
////// written by ToM
///////////////////////////////////////////////////////////////
#pragma once

#ifndef DEF_TRGLTETRA
#define DEF_TRGLTETRA

#include "../GLKLib/GLKObList.h"

//frequently used operation
#define MIN(a,b)	((a)<(b))?(a):(b)
#define MAX(a,b)	((a)>(b))?(a):(b)
#define INF    (1e8)
#define NINF    (-1e8)

class QMeshEdge;
class QMeshPatch;
class QMeshNode;
class QMeshFace;

class QMeshTetra :
	public GLKObject
{
public:
	QMeshTetra(void);
	~QMeshTetra(void);

	bool _judgeInsideTet(double* pos);
	bool _sameSide(double* pnt1, double* pnt2, double* pnt3, double* pnt4, double* queryPnt);
	void _crossProduct(double* vecA, double* vecB, double* vecC);
	double _dotProduct(double* vecA, double* vecB);
	bool _judgeIsInPntList(QMeshNode* _node);
	bool IsFixed();

//           1
//			/\
//		 e1/ |\e4
//		  /e3| \
//		 /_e5___\
//		2 \  |  / 4
//		 e2 \|/ e6
//           3
//	in anti-clockwise order
//  face1: 1,2,3
//	face2: 2,4,3
//	face3: 3,4,1
//	face4: 4,2,1


	bool selected;
	bool inner; //for cross section view
	int m_nIdentifiedPatchIndex;

	GLKPOSITION Pos;

private:
	int indexno;
	QMeshPatch *meshSurface;	// MESHSURFACE contain this triangle
	QMeshFace * faces[4];		//	4 faces
	bool isNormal[4];			// is normail direction

	double volume;

	GLKObList attachedList;

	//BOOL flags[8];

public:
	int GetIndexNo();		//from 1 to n
	void SetIndexNo( const int _index = 1 );

	void SetMeshSurfacePtr(QMeshPatch* _mesh);
	QMeshPatch* GetMeshSurfacePtr();

	QMeshFace * GetFaceRecordPtr( const int whichFace );
	//void SetFaceRecordPtr( const int whichFace, QMeshFace * _face = NULL);
	void SetFaceRecordPtr( const int whichFace, QMeshFace * _face);

	int GetFaceIndex(QMeshFace* face);

	QMeshEdge * GetEdgeRecordPtr( const int whichEdge );
	int GetEdgeIndex(QMeshEdge* edge);
		
	void GetNodePos( const int whichNode, double &xx, double &yy, double &zz);
	QMeshNode * GetNodeRecordPtr( int whichNode );
	int GetNodeIndex(QMeshNode* node);

	bool IsNormalDirection( const int whichFace );
	void SetDirectionFlag( const int whichFace, const int toBe = true );

	//BOOL GetAttribFlag( const int whichBit );
	//void SetAttribFlag( const int whichBit, const BOOL toBe = TRUE );

	GLKObList& GetAttachedList() {return attachedList;};

	void CalCenterPos(double &xx, double &yy, double &zz);

	double CalVolume(double t[][3]);
	double CalVolume();
	double GetVolume();

	double CalSolidAngle(const int whichNode);
	double CalSolidAngle(double p[3][3], double pp[3]);

	bool CalTetraBarycentry(double p[3], double t[4][3], double &_p, double &_q, double &_r, double &_s);
	bool CalTetraBarycentry(double p[3], double &_p, double &_q, double &_r, double &_s); //find barycentry in this tetra

	void BarycentryToPosition(double _p, double _q, double _r, double _s, double t[4][3], double p[3]);
	void BarycentryToPosition(double _p, double _q, double _r, double _s, double p[3]);

	bool show_innerTet_split;

	//int nodeindex[4];

	bool isChamber[4] = { false,false,false,false };
	bool chamberElement = false;
	bool isRigid = false;

	//used to construct AABB Tree
	void CalCenCalBoundingBox(void);
	std::set<QMeshTetra*> CalBoundaryTetList(void);
	std::set<QMeshTetra*> _boundaryTetSet;
	double _center[3];
	double _lowerBounding[3];
	double _upperBounding[3];
	int _idx = -1;					//start from 0.

	//used to calculate determinant.
	Eigen::Matrix4d eigenMat[5];			//for fast speed

	double pos1[3], pos2[3], pos3[3], pos4[3];
	bool _boundaryFlag = false;		//true -> boundary

};

#endif 
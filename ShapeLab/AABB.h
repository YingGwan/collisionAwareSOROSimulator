#pragma once
#include "AABBTreeStructure.h"
#include "PolygenMesh.h"
#include "QMeshPatch.h"
#include "QMeshTetra.h"
#include "QMeshNode.h"
#include "QMeshEdge.h"
#include "QMeshFace.h"
#include <string>
#include <set>
#include "PQP.h"

#define TET_OBSTACLE
#define ATTACHMENT_SELF_COLLISION 0.1
#define EPSILON_SELF_COLLISION 0.05



using namespace std;
/// <summary>
/// Operation Set of AABB 
/// </summary>
class AABBManager
{
public:
	AABBManager();
	~AABBManager();

public:
	//AABB 
	AABB Union(AABB A, AABB B);
	bool _intersect_triangle(RAY R, Eigen::Vector3d A, Eigen::Vector3d B, Eigen::Vector3d C, double &t, double &u, double &v, Eigen::Vector3d &N);		//check whether a ray intersecting with one triangle
	void _printBoundingBoxInfo(std::string _name, AABB _box);
	void _insert2Tree(int _flag,int _nodeIdx, AABB _ele);
	void splitBoundingBox(AABB _old, AABB* left, AABB* right);

	//construct AABB Tree via top-bot way
	void TreeConstructionTop2Bot(QMeshPatch* _tetMesh);
	void _recursiveConstruction(int _nodeIdx, int _flag, AABB _old,int _recursiveLayer=1);

	//rebuild the AABB tree
	void TreeConstructionTop2Bot_rebuild(QMeshPatch* _tetMesh);
	void _recursiveConstruction_rebuild(int _nodeIdx, int _flag, AABB _old, int _recursiveLayer = 1);

	//refit the AABB tree
	void TreeConstructionTop2Bot_refit(QMeshPatch* _tetMesh);
	void _recursivePostOrderTraversal_refit(int _nodeIdx, int _flag, int _recursiveLayer);

	//draw bounding box of AABB tree
	void _addSingleAABBboundingBox(AABB _aabb, QMeshPatch* _patch, int _depthIdx = 0);	//depth index used to draw color
	void _addSingleAABBboundingBox_compatiblewith_BuildAllTreeNode(AABBTREEArrayNode* _treeNode, AABB _aabb, QMeshPatch* _patch, int _depthIdx = 0);	//depth index used to draw color
	void _updateSingleAABBboundingBox_compatiblewith_BuildAllTreeNode(AABBTREEArrayNode* _treeNode, AABB _aabb, QMeshPatch* _patch, int _depthIdx = 0);	//depth index used to draw color

	void _addSameTreeDepthBoundingBox(int depth);
	void _buildAllTreeNodeBoundingBox(void);
	void _updateAllTreeNodeBoundingBox(void);
	//PolygenMesh* GenerateVisualizationMesh(int NameIdx = 0);
	void UpdateVisualizationMesh_refit(void);
	void ReBuildVisualizationMesh_rebuild(void);

	int GetTreeDepth(void);

	//query point
	void SelfCollisionDetection(void);
	void SelfCollisionDetectionCorrespondenceChecking(void);
	void SelfCollisionDetectionCorrespondenceChecking_softFinger(void);
	void _detectSprintCorrespondance(QMeshNode* node);


	bool _singlePntQuery(QMeshNode* _node);
	void _recursiveQuery(QMeshNode* _node, int _nodeIdx, int _flag, int _recursiveLayer = 1);
	bool _isCollided(double *_pnt, AABB _queryBox);

	//debug function used in building query point
	void _addSingleAABBboundingBox_debugQuery(AABB _aabb, QMeshPatch* _patch, int _depthIdx = 0);	//depth index used to draw color
	void _addSingleTetrahedron_debugQuery(QMeshTetra *tet, QMeshPatch* _patch);
	//QMeshPatch* SendDebugBoundingBoxPatch2MainWindow(void);

	//collision response
	void SelfCollisionCorrespondenceCalculation(void);
	//void SelfCollisionResponse(void);

	//self collision detection
	//how to visualize the collision result: 
	//1. draw give edge a flag: meaning this should be collided box

	//QMeshPatch* _debugPatch;

	void GetVolumeObstacleMesh(QMeshPatch* _patch);
	void ObstacleTreeConstructionTop2Bot_rebuild(void);
	bool _isConstructed = false;

	void _insert2Tree_obstacle(int _flag, int _nodeIdx, AABB _ele);
	void _recursiveConstruction_rebuild_obstacle(int _nodeIdx, int _flag, AABB _old, int _recursiveLayer);
	void TreeConstructionTop2Bot_refit_obstacle(void);
	void _recursivePostOrderTraversal_refit_obstacle(int _nodeIdx, int _flag, int _recursiveLayer);
	void CollisionWithEnvQueryChecking(void);
	bool _singlePntQueryWithTree(QMeshNode* _node, aabbTree* _aabbtree, QMeshTetra** _tetraPtrArray);
	void _recursiveQueryWithTree(aabbTree* _aabbTree, QMeshNode* _node, int _nodeIdx, int _flag, int _recursiveLayer);

	/////////////////////////////////////////////////////////////////
	//Treat this as an operator: environmental obstacle
	void GetTetMesh(QMeshPatch* tetMesh);
	Eigen::VectorXd CollisionWithEnvQueryCheckingReturnResult(int chamberIdx);
	void UpdateEnvCollisionStatus(QMeshPatch* tetMesh);
	void MarkBoundaryFaceForTetMesh(QMeshPatch* tetMesh);
	void SumUpCollisionResultWithEnv(QMeshPatch* tetMesh, Eigen::VectorXd& collidedResult);
	/////////////////////////////////////////////////////////////////

private:

	bool init_softFingerSelfCollision = false;
	bool selfCollisionTree_built = false;

	//fast travel when building edge list
	GLKPOSITION startIndexfromPos = NULL;
	int startIndexfromPos_int = 0;

	//refit position flag
	GLKPOSITION startIndexPos = NULL;
	QMeshTetra** TetraPtrArray;
	int TetraPtrArrayLen;
	AABB _init;
	bool threshold_selfCollisionSpring_maximum = 3.0; // unit mm


	//Tree structure containing AABB Tree
	QMeshPatch* _tetModelMesh;						//model that represents the original model
	aabbTree *_tree;
	bool _treeInit = false;

	QMeshPatch* _treeVisualizationMesh;
	PolygenMesh* _refitBoundingBoxPoly = NULL;		//used to redraw the bounding box of refit results
	int counter[10] = {0};

	bool _judgeInsideTet(double* pos, QMeshTetra* tetra);
	bool _judgeIsInPntList(QMeshNode* _node, QMeshTetra* tetra);
	/*bool _intersect_triangle(RAY R, Eigen::Vector3d A, Eigen::Vector3d B, Eigen::Vector3d C,
		double& t, double& u, double& v, Eigen::Vector3d& N);*/

	bool _sameSide(double* pnt1, double* pnt2, double* pnt3, double* pnt4, double* queryPnt);
	double _dotProduct(double* vecA, double* vecB);
	void _crossProduct(double* vecA, double* vecB, double* vecC);

	

public:
	//////////////////////////////////////////////////////
	/////Collision with env: volume represented by tree///
	//////////////////////////////////////////////////////

	aabbTree* _obstacleTree;
	bool memoryAllocated__obstacleTree = false;

	QMeshPatch* _obstacleMesh;

	QMeshTetra** TetraPtrArray_obstacle;
	bool memoryAllocated__TetraPtrArray_obstacle = false;

	int TetraPtrArrayLen_obstacle;
	AABB _init_obstacle;

	int counter_init_env = 0;

	//visualization bounding box
	QMeshPatch* _treeVisualizationMesh_obstacle;
	PolygenMesh* _refitBoundingBoxPoly_obstacle = NULL;		//used to redraw the bounding box of refit results

	//fast travel when building edge list
	GLKPOSITION startIndexfromPos_obstacle = NULL;
	int startIndexfromPos_int_obstacle = 0;

	//refit position flag
	GLKPOSITION startIndexPos_obstacle = NULL;

	//epsilon: distance between true boundary B2 and outer boundary B3. collision with env
	double epsilon = 0.005;
	double epsilon_spring = 0.05;

	QMeshFace** _faceArray_volumeSurface_obstacle;	//_faceArray that represents the obstacle's boundary face list
	bool memoryAllocated_faceArray_volumeSurface_obstacle = false;

	int _len_faceArray_volumeSurface_obstacle;		//len of _faceArray_volumeSurface_obstacle
	PQP_Model* _obstaclePQPModel_volumeSurface;		//obstacle volume mesh model's surface PQP Model
	bool memoryAllocated_obstaclePQPModel_volumeSurface = false;

	QMeshPatch* _corresPatch;

	//volume env mesh: 
	double coeff_extension_spring_env = 0.05f;			//spring attach location: 0.06		
	double coeff_extension_epsilon_env = 0.005;

	//////////////////////////////////////////////////////
	//Collision with env: volume represented by tree End//
	//////////////////////////////////////////////////////


	/////////////////////////////////////////////////////////
	//Self Collision Detection: volume represented by tree//
	////////////////////////////////////////////////////////

	//self collision detection
	//how to visualize the collision result: 
	//1. draw give edge a flag: meaning this should be collided box

	QMeshPatch* _debugPatch;
	QMeshPatch* _corresPatch_selfCollision;	//self collision
	double coeff_extension_spring = ATTACHMENT_SELF_COLLISION;			//spring attach location: 0.06		
	double coeff_extension_epsilon = EPSILON_SELF_COLLISION;
	/////////////////////////////////////////////////////////
	//////////////Self Collision Detection Ends/////////////
	////////////////////////////////////////////////////////
};



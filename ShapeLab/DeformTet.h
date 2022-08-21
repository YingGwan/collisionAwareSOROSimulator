#pragma once
#include <QStandardItemModel>
#include "..\packages\Eigen\Eigen"
#include "../QMeshLib/PolygenMesh.h"
#include "..\GLKLib\GLKObList.h"
#include "..\GLKLib\GLKGeometry.h"
#include "..\GLKLib\GLKMatrixLib.h"
#include "..\QMeshLib\QMeshPatch.h"
#include "..\QMeshLib\QMeshNode.h"
#include "..\QMeshLib\QMeshTetra.h"

/////////////////////////Guoxin Fang/////////////////////////
//For fast soft robot simulation with collision project//////
/////////////////////////////////////////////////////////////

class DeformTet
{
public:
	DeformTet();
	~DeformTet();

	void PreProcess();
	void setMeshNewPos(double distance = 0.3);
	void getExpansionRatio(double ratio);
	bool Run(int loop = 1);

	void setLastCoordinate(QMeshPatch* _patch);
	void SelectFixRegion(QMeshPatch* _patch);

	//Read mesh from platform
	void SetMesh(QMeshPatch* mesh);
	
	void SetObstacleMesh(QMeshPatch* _patch, bool _isVolume);

	double supportFreeAngle;
	Eigen::Vector3d printDir;
	void ShifttoInitialPos();

private:

	void ClearAll();
	void Initialization();
	void FillMatrixA();
	void FactorizeMatrixA();
	void FillVectorB();
	bool Solve();

	void LocalProjection();
	void LocalProjection_singleTetSVD(QMeshTetra* Tetra);

	bool UpdateVertex();
	void ComputeLocalGoalAndInverse();

	int vertNum, eleNum;

	//Define all matrix used in computings
	Eigen::SparseMatrix<double> matA;
	Eigen::SparseMatrix<double> matAT;
	Eigen::SimplicialLDLT <Eigen::SparseMatrix<double>> Solver;
	//SparseQR <Eigen::SparseMatrix<double>, Eigen::COLAMDOrdering<int>> Solver;
	//SparseLU<SparseMatrix<double>> Solver;
	std::vector<Eigen::MatrixXd> LocalCoord, InverseP, LocalGoal;
	std::vector<Eigen::VectorXd> VectorXPosition, VectorBSide, VectorXPosition_Last;

	std::vector<QMeshTetra*> tetraSet;


private:
	QMeshPatch* TetMesh;
	QMeshPatch* CollisionPairMesh;
	QMeshPatch* _obstacleMesh;
	bool _obstacleVolumeMesh = false;		//whether obstacle mesh is volume mesh
//area of collision handling
private:
	
	bool _inCollisionPlane(double x, double z);


public:

	void getRefreshingTool(GLKLib* glk, GLKObList* polygenMeshList, QStandardItemModel* treemodel);
	void AddPolygenToList(PolygenMesh* mesh);

	//Main function.
	bool RunWithCollisionResponse(std::vector<Eigen::MatrixXd> &initShape, int loop = 1);

	//collision detection function.
	bool CollisionDetection();
	//collision response: Fill Vector B.
	void FillVectorB_CR();		

	//collision response version
	void PreProcess_CR();
	void Initialization_CR();
	void ComputeLocalGoalAndInverse_CR();
	void FillMatrixA_CR();
	void FactorizeMatrixA_CR();
    void LocalProjection_CR();

	//self collision and collision with env
	void Initialization_CR_All();
	void PreProcess_CR_All();
	void FillMatrixA_CR_All();
	void FactorizeMatrixA_CR_All();
	void ComputeLocalGoalAndInverse_CR_All();
	void DeformTet::ComputeLocalGoalAndInverse_CR_All(std::vector<Eigen::MatrixXd>& initShape);
	void LocalProjection_CR_All();
	void FillVectorB_CR_All();
	double _calEleVolume(Eigen::MatrixXd& _tet);

	//var
	bool flag_UpdateMatAMatB = false;
	int springNum = 0;
	double weightSpring = 5.0;
	double weightHard = 5.0;
	double weightChamble = 0.3;

	double coeff_extension_spring = 0.001; //0.02f;

	//environment spring
	int springNum_env = 0;
	double weightSpring_env = 0.5;		//too strong: 25 ok: 1
	double springExtension_env = 0.02;	//not use now

	GLKObList* PolygenMeshListPtr;
	GLKLib* pGLK;
	QStandardItemModel* treeModel;


	
	double material_omega = 0.05;		//material rigidity
	double expansion_ratio = 1.0;		//chamber tetrahedron expansion ratio

};


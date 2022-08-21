#include "stdafx.h"
#include "DeformTet.h"
#include <QCoreApplication>
#include <QMeshEdge.h>

#include <omp.h>

#include <iostream>
#include <fstream>

using namespace std;
using namespace Eigen;

#ifdef _DEBUG
//#define new DEBUG_NEW
//#undef THIS_FILE
//static char THIS_FILE[] = __FILE__;
#endif

#define VELE 4
#define FACTOR 3


//we need to change the file in ./selectionFile/CollisionExample/Obstacle.obj
#define CD_XMIN (-35.0)
#define CD_XMAX (35.0)
#define CD_ZMIN (-10.0)
#define CD_ZMAX (10.0)

#define CD_YVAL (27)
#define CD_YC1	CD_YVAL
#define CD_YC2	(CD_YC1-0.1)			//here, 0.1 is the parameter to leave for the soft spring

//-0.1


DeformTet::DeformTet()
{

}

DeformTet::~DeformTet()
{
	ClearAll();
}

void DeformTet::ClearAll()
{
	/*if (FactorizedA) { LinearSolver::DeleteF(FactorizedA); FactorizedA = 0; }
	if (VectorX) { delete VectorX; VectorX = 0; }
	if (VectorB) { delete VectorB; VectorB = 0; }
	if (matA) delete matA;
	if (LocalCoord) delete[] LocalCoord;
	if (LocalGoal) delete[] LocalGoal;
	if (InverseP) delete[] InverseP;*/
}

void DeformTet::SetMesh(QMeshPatch* mesh) {
	TetMesh = mesh;
	qDebug("Set mesh size: %d",TetMesh->GetTetraList().GetCount());
	tetraSet.resize(TetMesh->GetTetraNumber());
	int index = 0;
	for (GLKPOSITION Pos = TetMesh->GetTetraList().GetHeadPosition(); Pos;) {
		QMeshTetra* Tetra = (QMeshTetra*)TetMesh->GetTetraList().GetNext(Pos);
		tetraSet[index] = Tetra; index++;
	}
}

void DeformTet::SetObstacleMesh(QMeshPatch* _patch, bool _isVolume)
{
	_obstacleMesh = _patch;

	_obstacleVolumeMesh = _isVolume;
}

void DeformTet::PreProcess()
{
	Initialization();
	ComputeLocalGoalAndInverse();

	FillMatrixA();
	FactorizeMatrixA();
	cout << "finish preprocess the system" << endl;
}

void DeformTet::PreProcess_CR()
{
	Initialization_CR();
	ComputeLocalGoalAndInverse_CR();

	FillMatrixA_CR();
	FactorizeMatrixA_CR();
	cout << "finish preprocess the system" << endl;
}


void DeformTet::FactorizeMatrixA_CR()
{

	Eigen::SparseMatrix<double> matATA(vertNum, vertNum);

	matAT = matA.transpose();
	matATA = matAT * matA;

	Solver.compute(matATA);
	printf("end factorize materix A\n");



}

void DeformTet::FillMatrixA_CR()
{

	//give memory to sparse matrix, to accerate the insert speed
	matA.reserve(VectorXi::Constant(VELE * eleNum + springNum, 1000));

	float c1 = -1.0 / VELE, c2 = 1 + c1;

	for (GLKPOSITION Pos = TetMesh->GetTetraList().GetHeadPosition(); Pos;) {
		QMeshTetra* Tetra = (QMeshTetra*)TetMesh->GetTetraList().GetNext(Pos);
		if (Tetra->GetIndexNo() < 0) continue;

		int fdx = Tetra->GetIndexNo() * VELE;
		double weight = 1.0;

		if (Tetra->isChamber[0]) weight = weightChamble;
		if (Tetra->isRigid) weight = weightHard;

		int vdxArr[VELE];
		for (int i = 0; i < VELE; i++) vdxArr[i] = Tetra->GetNodeRecordPtr(i + 1)->GetIndexNo();

		for (int i = 0; i < VELE; i++) {
			if (vdxArr[i] < 0) continue;
			for (int j = 0; j < VELE; j++) {
				if (vdxArr[j] < 0) continue;
				if (i == j) matA.insert(fdx + j, vdxArr[i]) = c2 * weight;
				else matA.insert(fdx + j, vdxArr[i]) = c1 * weight;
			}
		}
	}

	//traverse the node list and add spring to the last several rows of A matrix
	int fdx = VELE * eleNum;
	int springIdx = 0;
	for (GLKPOSITION Pos = TetMesh->GetNodeList().GetHeadPosition(); Pos;) {
		QMeshNode* Node = (QMeshNode*)TetMesh->GetNodeList().GetNext(Pos);

		//if node should be added spring
		if (Node->isCollided == true)
		{
			//std::cout << "collision node!!!" << std::endl;
			int verIdx = Node->GetIndexNo();
			matA.insert(fdx + springIdx, verIdx) = weightSpring * 1.0;


			//barycentric coordinate of re-tracing point
			int Q[3];
			double B[3];
			for (int k = 0; k < 3; k++)
			{
				Q[k] = Node->_collidedFacePtr->GetNodeRecordPtr(k)->GetIndexNo();
				//barycentric Coordinate relation: corresponding to three node
				B[k] = Node->_barycentricCoord[k];

			}
			//qDebug("Spring: %d\nFace's point index: %d %d %d", springIdx, Q[0], Q[1], Q[2]);
			//qDebug("Collided point barycentric: %lf %lf %lf", B[0], B[1], B[2]);

			matA.insert(fdx + springIdx, Q[0]) = -weightSpring * B[0];
			matA.insert(fdx + springIdx, Q[1]) = -weightSpring * B[1];
			matA.insert(fdx + springIdx, Q[2]) = -weightSpring * B[2];

			springIdx++;



		}




	}



	matA.makeCompressed();

}

void DeformTet::ComputeLocalGoalAndInverse_CR()
{

	//For the deformation usage, every tetra should remain its initial shape
	for (GLKPOSITION Pos = TetMesh->GetTetraList().GetHeadPosition(); Pos;) {
		QMeshTetra* Tet = (QMeshTetra*)TetMesh->GetTetraList().GetNext(Pos);
		int fdx = Tet->GetIndexNo(); if (fdx < 0) continue;

		QMeshNode* nodes[VELE];
		Eigen::MatrixXd P = Eigen::MatrixXd::Zero(3, VELE);
		double center[3] = { 0 };

		for (int i = 0; i < VELE; i++) {
			nodes[i] = Tet->GetNodeRecordPtr(i + 1);
			nodes[i]->GetCoord3D_last(P(0, i), P(1, i), P(2, i));
			for (int j = 0; j < 3; j++) center[j] += P(j, i);
		} for (int j = 0; j < 3; j++) center[j] /= VELE;


		double expandRatio = 5.0;
		for (int i = 0; i < VELE; i++) {
			for (int j = 0; j < 3; j++) {
				if (Tet->isChamber[0]) 
					P(j, i) = (P(j, i) - center[j]) * pow(expandRatio, 1.0 / 3); // for pneumatic-driven soft robot simulation
				else P(j, i) -= center[j];
			}
		}


		//solving pseudoInverse with GLKMATRIX Lib, Eigen may occur error here
		LocalGoal[fdx] = P;
		InverseP[fdx] = (P.transpose()).completeOrthogonalDecomposition().pseudoInverse();

		if (InverseP[fdx].mean() - InverseP[fdx].mean() != 0) cout << InverseP[fdx] << " pseudo inverse ERROR!" << std::endl;

	}
	printf("Finish Compute local inverse!\n");


}

void DeformTet::getExpansionRatio(double ratio)
{
	expansion_ratio = ratio;
}

void DeformTet::setMeshNewPos(double distance)
{

	/*double maxHight = -99999.0;
	for (GLKPOSITION pos = TetMesh->GetNodeList().GetHeadPosition(); pos != nullptr;) {
		QMeshNode* node = (QMeshNode*)TetMesh->GetNodeList().GetNext(pos);

		double pp[3] = { 0 };
		node->GetCoord3D(pp[0], pp[1], pp[2]);
		if (pp[1] > maxHight) maxHight = pp[1];


	}*/

	for (GLKPOSITION pos = TetMesh->GetNodeList().GetHeadPosition(); pos != nullptr;) {
		QMeshNode* node = (QMeshNode*)TetMesh->GetNodeList().GetNext(pos);

		double pp[3] = { 0 };
		node->GetCoord3D(pp[0], pp[1], pp[2]);

		if (node->isHandle)
			node->SetCoord3D(pp[0], pp[1] + distance, pp[2]);

	}

}

bool DeformTet::Run(int loop)
{
	for (int i = 0; i < loop; i++) {
		LocalProjection();
		FillVectorB();
		Solve();
		if (!UpdateVertex()) return false;
		//ShifttoInitialPos();
	}
	return true;
}

void DeformTet::setLastCoordinate(QMeshPatch* _patch)
{
	double xx, yy, zz;
	for (GLKPOSITION Pos = _patch->GetNodeList().GetHeadPosition(); Pos;)
	{
		QMeshNode* node = (QMeshNode*)_patch->GetNodeList().GetNext(Pos);
		node->GetCoord3D(xx,yy,zz);
		node->SetCoord3D_last(xx,yy,zz);
		node->isCollided = false;
		node->isCollided_last = false;
		node->isCollided_env = false;
		node->isCollided_env_last = false;

	}
}

void DeformTet::SelectFixRegion(QMeshPatch* _patch)
{

	qDebug("Fix Region Selection...");
	Eigen::Vector3d pos, min;
	Eigen::Vector3d posMin = {1000,1000,1000};
	for (GLKPOSITION Pos = _patch->GetNodeList().GetHeadPosition(); Pos;)
	{
		QMeshNode* node = (QMeshNode*)_patch->GetNodeList().GetNext(Pos);

		if (node->shirftSelect == true)
		{
			node->isFixed = true;

		}
		else
			node->isFixed = false;


		//node->GetCoord3D(pos);
		//if (pos[0] < posMin[0])
		//	posMin[0] = pos[0];
	}

	//for (GLKPOSITION Pos = _patch->GetNodeList().GetHeadPosition(); Pos;)
	//{
	//	QMeshNode* node = (QMeshNode*)_patch->GetNodeList().GetNext(Pos);
	//	node->GetCoord3D(pos);
	//	
	//	if (fabs(pos[0] - posMin[0]) < 0.3)
	//	{
	//		node->isFixed = true;
	//	}
	//	else
	//		node->isFixed = false;
	//}

}

bool DeformTet::CollisionDetection()
{
	//Eigen::MatrixXd POS(3, 1);
	//bool flag = false;
	//for (GLKPOSITION Pos = TetMesh->GetNodeList().GetHeadPosition(); Pos;) 
	//{
	//	QMeshNode* node = (QMeshNode*)TetMesh->GetNodeList().GetNext(Pos);

	//	//identify whether the node is inner
	//	node->inner = true;
	//	for (GLKPOSITION edgePos = node->GetEdgeList().GetHeadPosition(); edgePos;)
	//	{
	//		QMeshEdge* edge = (QMeshEdge*)((node->GetEdgeList()).GetNext(edgePos));
	//		
	//		if (edge->inner==false)
	//		{
	//			node->inner = false;
	//			break;
	//		}
	//	}
	//	node->GetCoord3D(POS(0,0), POS(1,0), POS(2,0));
	//	
	//	//further check whether the skin point is among the xz range of collision plane
	//	if(node->inner==false)
	//		node->inCollidedRange = _inCollisionPlane(POS(0, 0), POS(2, 0));
	//	
	//	//further check whether the point in the collision range would penetrate into the inner of obstacle
	//		/////final version: here we should let the following functions to be finished
	//		//find the closest point on the obstacle 
	//		
	//		//get the normal of the closest point

	//		//calculate whether it is penetrate into the obstacle

	//		/////quick version: directly compare the y value
	//	node->flag_toEstablish_last = node->flag_toEstablish;
	//	if (node->inCollidedRange == true)
	//	{
	//		//if the point is in the range of collided range
	//		//we will check the collision status: for visualization only
	//		if (POS(1, 0) > CD_YVAL)
	//		{
	//			node->isCollided = true;

	//			double nodePos[3];
	//			node->GetCoord3D(nodePos[0], nodePos[1], nodePos[2]);
	//			node->closestPntOnObstacle[0] = nodePos[0];
	//			node->closestPntOnObstacle[1] = CD_YVAL;
	//			node->closestPntOnObstacle[2] = nodePos[2];

	//			flag = true;
	//		}
	//		else {
	//			node->isCollided = false;
	//		}

	//		//if the point is in the inner side of C_1, then it should establish a new spring 
	//		if (POS(1, 0) > CD_YC1)
	//		{
	//			node->flag_toEstablish = true;
	//			//we should also provide the attachment position of this spring
	//			double nodePos[3];
	//			node->GetCoord3D(nodePos[0], nodePos[1], nodePos[2]);
	//		
	//			node->springAttachPos[0] = nodePos[0];
	//			node->springAttachPos[1] = CD_YC2;
	//			node->springAttachPos[2] = nodePos[2];

	//		}
	//			
	//		//if the point is outside the inner side of C_2, then it should destroy the attached spring 
	//		else if (POS(1, 0) < CD_YC2)
	//			node->flag_toEstablish = false;

	//		

	//		//if there is at least one point in the collision, we should let go into the collision response phase
	//		if (node->flag_toEstablish == true)
	//		{
	//			flag = true;
	//		}
	//		
	//	}
	//		
	//	//we should leave a flag to indicate the flag state in the last run
	//	
	//}

	////create the polygen to show the collision 
	//if (flag == true)
	//{
	//	static bool flag_initCorre_1 = false;
	//	if (!flag_initCorre_1)
	//	{
	//		flag_initCorre_1 = true;

	//		PolygenMesh* correPoly1 = new PolygenMesh(TRI);
	//		correPoly1->draw_idx = 2;
	//		correPoly1->setModelName("CollisionPair");
	//		QMeshPatch* mesh = new QMeshPatch;
	//		CollisionPairMesh = mesh;
	//		for (GLKPOSITION Pos = TetMesh->GetNodeList().GetHeadPosition(); Pos;)
	//		{
	//			QMeshNode* node = (QMeshNode*)TetMesh->GetNodeList().GetNext(Pos);
	//			if (node->isCollided == true)
	//			{

	//				double POS[3];
	//				node->GetCoord3D(POS[0],POS[1],POS[2]);

	//				QMeshNode* originalNode = new QMeshNode;
	//				originalNode->SetCoord3D(POS[0], POS[1], POS[2]);
	//				mesh->GetNodeList().AddTail(originalNode);

	//				QMeshNode* colliNode = new QMeshNode;
	//				colliNode->SetCoord3D(node->closestPntOnObstacle[0], node->closestPntOnObstacle[1], node->closestPntOnObstacle[2]);
	//				mesh->GetNodeList().AddTail(colliNode);

	//				QMeshEdge* edge = new QMeshEdge;
	//				edge->SetStartPoint(originalNode);
	//				edge->SetEndPoint(colliNode);
	//				mesh->GetEdgeList().AddTail(edge);

	//			}
	//		}
	//		correPoly1->GetMeshList().AddTail(mesh);
	//		AddPolygenToList(correPoly1);
	//	}
	//	else
	//	{
	//		CollisionPairMesh->ClearAll();

	//		for (GLKPOSITION Pos = TetMesh->GetNodeList().GetHeadPosition(); Pos;)
	//		{
	//			QMeshNode* node = (QMeshNode*)TetMesh->GetNodeList().GetNext(Pos);
	//			if (node->isCollided == true)
	//			{

	//				double POS[3];
	//				node->GetCoord3D(POS[0], POS[1], POS[2]);

	//				QMeshNode* originalNode = new QMeshNode;
	//				originalNode->SetCoord3D(POS[0], POS[1], POS[2]);
	//				CollisionPairMesh->GetNodeList().AddTail(originalNode);

	//				QMeshNode* colliNode = new QMeshNode;
	//				colliNode->SetCoord3D(node->closestPntOnObstacle[0], node->closestPntOnObstacle[1], node->closestPntOnObstacle[2]);
	//				CollisionPairMesh->GetNodeList().AddTail(colliNode);

	//				QMeshEdge* edge = new QMeshEdge;
	//				edge->SetStartPoint(originalNode);
	//				edge->SetEndPoint(colliNode);
	//				CollisionPairMesh->GetEdgeList().AddTail(edge);

	//			}
	//		}
	//	}
	//	

	//	
	//}
	
	//return flag;
	return true;
}


void DeformTet::getRefreshingTool(GLKLib* glk, GLKObList* polygenMeshList, QStandardItemModel* treemodel)
{
	PolygenMeshListPtr = polygenMeshList;
	pGLK = glk;
	treeModel = treemodel;
}

void DeformTet::AddPolygenToList(PolygenMesh* mesh)
{
	mesh->BuildGLList(mesh->m_bVertexNormalShading);
	pGLK->AddDisplayObj(mesh, true);
	PolygenMeshListPtr->AddTail(mesh);

	//update tree
	treeModel->clear();
	for (GLKPOSITION pos = PolygenMeshListPtr->GetHeadPosition(); pos != nullptr;) {
		PolygenMesh* polygenMesh = (PolygenMesh*)PolygenMeshListPtr->GetNext(pos);
		QString modelName = QString::fromStdString(polygenMesh->getModelName());
		QStandardItem* modelListItem = new QStandardItem(modelName);
		modelListItem->setCheckable(true);
		modelListItem->setCheckState(Qt::Checked);
		treeModel->appendRow(modelListItem);
	}
	pGLK->refresh(true);
}

void DeformTet::Initialization_CR_All()
{
	static bool flag_init = false;
	//set node and face number

	if (!flag_init)
	{
		//first execution: should build all matrices
		flag_init = true;

		vertNum = 0;
		for (GLKPOSITION pos = TetMesh->GetNodeList().GetHeadPosition(); pos != nullptr;) {
			QMeshNode* node = (QMeshNode*)TetMesh->GetNodeList().GetNext(pos);
			double pp[3] = { 0 };
			//node->GetCoord3D_last(pp[0], pp[1], pp[2]);
			//node->SetCoord3D(pp[0], pp[1], pp[2]);
			if (node->isFixed == false && node->isHandle == false)
				node->SetIndexNo(vertNum++);
			else node->SetIndexNo(-1);

		}
		cout << "non-fixed vertex Num = " << vertNum << endl;
		cout << "all vertices are " << TetMesh->GetNodeNumber() << endl;

		eleNum = 0;
		for (GLKPOSITION Pos = TetMesh->GetTetraList().GetHeadPosition(); Pos;) {
			QMeshTetra* Tet = (QMeshTetra*)TetMesh->GetTetraList().GetNext(Pos);
			if (Tet->IsFixed()) Tet->SetIndexNo(-1); // as long as single vertex is fixed, the tet is fixed
			else Tet->SetIndexNo(eleNum++);
		}
		cout << "non-fixed element Num = " << eleNum << endl;



		springNum = 0;
		for (GLKPOSITION pos = TetMesh->GetNodeList().GetHeadPosition(); pos != nullptr;)
		{
			QMeshNode* node = (QMeshNode*)TetMesh->GetNodeList().GetNext(pos);
			
			bool checkFaceFix = false;

			//check whether this node is in collision
			//if (node->isCollided == true)springNum++;

			/*for (int k = 0; k < 3; k++)
			{
				if (node->_collidedFacePtr->GetNodeRecordPtr(k)->isFixed)
				{
					checkFaceFix = true;
					break;
				}
			}
			if (checkFaceFix && node->isCollided == true) {
				node->isCollided = false;
			}*/
			if(node->isCollided)
				springNum++;

		}
		qDebug("Spring number is %d",springNum);

		springNum_env = 0;
		for (GLKPOSITION pos = TetMesh->GetNodeList().GetHeadPosition(); pos != nullptr;)
		{
			QMeshNode* node = (QMeshNode*)TetMesh->GetNodeList().GetNext(pos);
			//check whether this node is in collision
			if (node->isCollided_env == true)	springNum_env++;
		}
		qDebug("Spring env number is %d", springNum_env);
		//initialize all computing matrix
		matA.resize(VELE * eleNum + springNum + springNum_env, vertNum);							//A  Mat should added several springs
		matAT.resize(vertNum, VELE * eleNum + springNum + springNum_env);						//AT Mat should added several springs

		LocalCoord.resize(eleNum);
		InverseP.resize(eleNum);
		LocalGoal.resize(eleNum);
		VectorXPosition.resize(3);
		VectorXPosition_Last.resize(3);
		VectorBSide.resize(3);

		for (int i = 0; i < 3; i++) {
			VectorXPosition[i] = Eigen::VectorXd::Zero(vertNum);
			VectorXPosition_Last[i] = Eigen::VectorXd::Zero(vertNum);
			VectorBSide[i] = Eigen::VectorXd::Zero(eleNum * VELE + springNum + springNum_env);	//B  Mat should added several springs
		}

		qDebug("Assign memory...");
		flag_UpdateMatAMatB = true;	//should update B matrix as well


	}
	else
	{
		vertNum = 0;
		for (GLKPOSITION pos = TetMesh->GetNodeList().GetHeadPosition(); pos != nullptr;) {
			QMeshNode* node = (QMeshNode*)TetMesh->GetNodeList().GetNext(pos);
			double pp[3] = { 0 };
			//node->GetCoord3D_last(pp[0], pp[1], pp[2]);
			//node->SetCoord3D(pp[0], pp[1], pp[2]);
			if (node->isFixed == false && node->isHandle == false)
				node->SetIndexNo(vertNum++);
			else node->SetIndexNo(-1);

		}
		cout << "non-fixed vertex Num = " << TetMesh->GetNodeNumber() << endl;

		eleNum = 0;
		for (GLKPOSITION Pos = TetMesh->GetTetraList().GetHeadPosition(); Pos;) {
			QMeshTetra* Tet = (QMeshTetra*)TetMesh->GetTetraList().GetNext(Pos);
			if (Tet->IsFixed()) Tet->SetIndexNo(-1); // as long as single vertex is fixed, the tet is fixed
			else Tet->SetIndexNo(eleNum++);
		}
		cout << "non-fixed element Num = " << eleNum << endl;



		springNum = 0;
		for (GLKPOSITION pos = TetMesh->GetNodeList().GetHeadPosition(); pos != nullptr;)
		{
			QMeshNode* node = (QMeshNode*)TetMesh->GetNodeList().GetNext(pos);
			//check whether this node is in collision
			if (node->isCollided == true)springNum++;
		}
		qDebug("Spring number is %d", springNum);
		springNum_env = 0;
		for (GLKPOSITION pos = TetMesh->GetNodeList().GetHeadPosition(); pos != nullptr;)
		{
			QMeshNode* node = (QMeshNode*)TetMesh->GetNodeList().GetNext(pos);
			//check whether this node is in collision
			if (node->isCollided_env == true)	springNum_env++;
		}
		qDebug("Spring env number is %d", springNum_env);
		//initialize all computing matrix
		matA.resize(VELE * eleNum + springNum + springNum_env, vertNum);							//A  Mat should added several springs
		matAT.resize(vertNum, VELE * eleNum + springNum + springNum_env);						//AT Mat should added several springs

		LocalCoord.resize(eleNum);
		//These two terms can be kept fixed
		InverseP.resize(eleNum);
		LocalGoal.resize(eleNum);

		VectorXPosition.resize(3);
		VectorXPosition_Last.resize(3);
		VectorBSide.resize(3);

		for (int i = 0; i < 3; i++) {
			VectorXPosition[i] = Eigen::VectorXd::Zero(vertNum);
			VectorXPosition_Last[i] = Eigen::VectorXd::Zero(vertNum);
			VectorBSide[i] = Eigen::VectorXd::Zero(eleNum * VELE + springNum + springNum_env);	//B  Mat should added several springs
		}


		flag_UpdateMatAMatB = true;	//should update B matrix as well

		qDebug("Assign memory...");

	}

	//this->ComputeLocalGoalAndInverse();
}

void DeformTet::FillMatrixA_CR_All()
{
	//give memory to sparse matrix, to accerate the insert speed
	matA.reserve(VectorXi::Constant(VELE * eleNum + springNum + springNum_env, 1000));

	float c1 = -1.0 / VELE, c2 = 1 + c1;

	for (GLKPOSITION Pos = TetMesh->GetTetraList().GetHeadPosition(); Pos;) {
		QMeshTetra* Tetra = (QMeshTetra*)TetMesh->GetTetraList().GetNext(Pos);
		if (Tetra->GetIndexNo() < 0) continue;

		int fdx = Tetra->GetIndexNo() * VELE;
		double weight = 1.0;

		if (Tetra->isRigid) weight = this->weightHard;

		int vdxArr[VELE];
		for (int i = 0; i < VELE; i++) vdxArr[i] = Tetra->GetNodeRecordPtr(i + 1)->GetIndexNo();

		for (int i = 0; i < VELE; i++) {
			if (vdxArr[i] < 0) continue;
			for (int j = 0; j < VELE; j++) {
				if (vdxArr[j] < 0) continue;
				if (i == j) matA.insert(fdx + j, vdxArr[i]) = c2 * weight;
				else matA.insert(fdx + j, vdxArr[i]) = c1 * weight;
			}
		}
	}

	//qDebug("Fill Mat A 1...");
	//self collision spring
	//traverse the node list and add spring to the last several rows of A matrix
	int fdx = VELE * eleNum;
	int springIdx = 0;
	for (GLKPOSITION Pos = TetMesh->GetNodeList().GetHeadPosition(); Pos;) {
		QMeshNode* Node = (QMeshNode*)TetMesh->GetNodeList().GetNext(Pos);

		//if node should be added spring
		if (Node->isCollided == true)
		{
			int verIdx = Node->GetIndexNo();
			matA.insert(fdx + springIdx, verIdx) = weightSpring * 1.0;


			//barycentric coordinate of re-tracing point
			int Q[3];
			double B[3];
			for (int k = 0; k < 3; k++)
			{
				Q[k] = Node->_collidedFacePtr->GetNodeRecordPtr(k)->GetIndexNo();
				//barycentric Coordinate relation: corresponding to three node
				B[k] = Node->_barycentricCoord[k];

			}
			//qDebug("Spring: %d\nFace's point index: %d %d %d", springIdx,Q[0], Q[1], Q[2]);
			//qDebug("Collided point barycentric: %lf %lf %lf", B[0], B[1], B[2]);

			matA.insert(fdx + springIdx, Q[0]) = -weightSpring * B[0];
			matA.insert(fdx + springIdx, Q[1]) = -weightSpring * B[1];
			matA.insert(fdx + springIdx, Q[2]) = -weightSpring * B[2];

			springIdx++;



		}

	}
	//qDebug("Fill Mat A 2...");
	//traverse the node list and add spring to the last several rows of A matrix
	fdx = VELE * eleNum + springNum;
	springIdx = 0;
	for (GLKPOSITION Pos = TetMesh->GetNodeList().GetHeadPosition(); Pos;) {
		QMeshNode* Node = (QMeshNode*)TetMesh->GetNodeList().GetNext(Pos);

		//if node should be added spring
		if (Node->isCollided_env == true)
		{
			int verIdx = Node->GetIndexNo();
			matA.insert(fdx + springIdx, verIdx) = weightSpring_env * 1.0;

			springIdx++;
		}

	}


	//qDebug("Fill Mat A 3...");

	matA.makeCompressed();
	
	

	//qDebug("Fill Mat A...\n");
}

void DeformTet::FactorizeMatrixA_CR_All()
{
	static bool flag_init = false;
	if (!flag_init)
	{
		flag_init = true;
		Eigen::SparseMatrix<double> matATA(vertNum, vertNum);

		matAT = matA.transpose();
		matATA = matAT * matA;

		Solver.compute(matATA);
		printf("end factorize materix A\n");
	}
	else
	{
		//if the spring structure is updated, we should update A Mat
		if (flag_UpdateMatAMatB)
		{
			Eigen::SparseMatrix<double> matATA(vertNum, vertNum);

			matAT = matA.transpose();
			matATA = matAT * matA;

			Solver.compute(matATA);
			printf("end factorize materix A for the spring structure\n");

		}
		//Otherwise, we should not update A Mat
		else {
			//new
			Eigen::SparseMatrix<double> matATA(vertNum, vertNum);

			matAT = matA.transpose();
			matATA = matAT * matA;

			Solver.compute(matATA);
			printf("end factorize materix A for the spring structure\n");
		}

	}

	qDebug("Factorization finished...\n");

}


//_tet is a 3x4 matrix
double DeformTet::_calEleVolume(Eigen::MatrixXd& _tet)
{
	Eigen::Matrix4d volumeMat;
	//for each point
	for (int i = 0; i < 4; i++)
		//for x y z
		for (int j = 0; j < 4; j++)
		{
			if (j < 3)
				volumeMat(i, j) = _tet(j, i);
			else
				volumeMat(i, j) = 1.0;
		}
	//cout << "Volume mat is \n" << volumeMat << endl;
	return volumeMat.determinant();
}


void DeformTet::ComputeLocalGoalAndInverse_CR_All()
{

	//
	//static bool global_init = false;

	//rigid pos: original pos
	//deformed pos: now pos
	Eigen::MatrixXd rigidPos = Eigen::MatrixXd::Zero(3, 4);
	Eigen::MatrixXd deformedPos = Eigen::MatrixXd::Zero(3, 4);
	int outputCount = 0;
	
	int counter_body = 0;
	int counter_chamber = 0;

	
	//For the deformation usage, every tetra should remain its initial shape
	for (GLKPOSITION Pos = TetMesh->GetTetraList().GetHeadPosition(); Pos;) {
		QMeshTetra* Tet = (QMeshTetra*)TetMesh->GetTetraList().GetNext(Pos);
		int fdx = Tet->GetIndexNo(); if (fdx < 0) continue;

		


		QMeshNode* nodes[VELE];
		Eigen::MatrixXd P = Eigen::MatrixXd::Zero(3, VELE);

		double center[3] = { 0,0,0 };			//initial center
		double centerCur[3] = { 0,0,0 };		//current center


		for (int i = 0; i < VELE; i++) {
			nodes[i] = Tet->GetNodeRecordPtr(i + 1);
			nodes[i]->GetCoord3D_last(P(0, i), P(1, i), P(2, i));
			nodes[i]->GetCoord3D_last(rigidPos(0, i), rigidPos(1, i), rigidPos(2, i));
			nodes[i]->GetCoord3D(deformedPos(0, i), deformedPos(1, i), deformedPos(2, i));

			for (int j = 0; j < 3; j++)
			{
				center[j] += P(j, i);
				centerCur[j] += deformedPos(j, i);
			}
		}

		for (int j = 0; j < 3; j++)
		{
			center[j] /= (double)VELE;
			centerCur[j] /= (double)VELE;
		}

		double expandRatio = 1.0;
		for (int i = 0; i < VELE; i++) {
			for (int j = 0; j < 3; j++) {
	
				rigidPos(j, i) -= (center[j]);
				deformedPos(j, i) -= (centerCur[j]);
			}
		}

		
		///////////////////Eigen SVD decomposition/////////////////////////

		
		JacobiSVD<Eigen::MatrixXd> svd(rigidPos * deformedPos.transpose(), ComputeThinU | ComputeThinV);
		Eigen::MatrixXd R = svd.matrixU() * svd.matrixV().transpose();

		//deformPos : 3x4
		//here, volume is not kept here
		LocalGoal[fdx] = (1 - material_omega) * R * deformedPos + material_omega * rigidPos;
		//
		double expansionRatio = fabs(_calEleVolume(rigidPos) / _calEleVolume(LocalGoal[fdx]));
		/*if (fdx % 1000 == 0)
		{
			printf("(%5d) (%.3lf,%.3lf) expansion ratio is %lf\n", fdx, _calEleVolume(rigidPos), _calEleVolume(LocalGoal[fdx]), expansionRatio);
			cout << "initial mat is\n" << rigidPos << endl << "current mat is\n" << deformedPos << endl;
			cout << "mixed mat is\n" << LocalGoal[fdx] << endl << endl << endl;
		}*/
		Eigen::Vector3d center_localGoal;
		//3x4 
		center_localGoal = LocalGoal[fdx].rowwise().mean();
		for (int j = 0; j < 4; j++) LocalGoal[fdx].col(j) = LocalGoal[fdx].col(j) - center_localGoal;

		LocalGoal[fdx] = LocalGoal[fdx] * pow(expansionRatio, 1.0 / 3);
		//qDebug("Body tet..");
		


		if (Tet->isChamber[0] == false)
			counter_body++;
		else
			counter_chamber++;
	
	}
	printf("Finish Compute local inverse!\nFinish Compute Local Goal...\n");

	qDebug("\n*************************************\n");
	qDebug("counter_body is %d and counter_chamber is %d and sum is %d", counter_body, counter_chamber, counter_body+ counter_chamber);
	qDebug("total number of tet element is %d", TetMesh->GetTetraNumber());

	qDebug("\n*************************************\n");
}

void DeformTet::ComputeLocalGoalAndInverse_CR_All(std::vector<Eigen::MatrixXd>& initShape)
{

	/*qDebug("\n*****************************************\nIn Collision Handling: %d\n*****************************************\n");
	int i = 0;
	for (int k = 0; k < 2; k++)
	{
		qDebug("-----%d-----", k);
		qDebug("%3.3lf\t%3.3lf\t%3.3lf\t%3.3lf", initShape[k](0, 0), initShape[k](0, 1), initShape[k](0, 2), initShape[k](0, 3));
		qDebug("%3.3lf\t%3.3lf\t%3.3lf\t%3.3lf", initShape[k](1, 0), initShape[k](1, 1), initShape[k](1, 2), initShape[k](1, 3));
		qDebug("%3.3lf\t%3.3lf\t%3.3lf\t%3.3lf", initShape[k](2, 0), initShape[k](2, 1), initShape[k](2, 2), initShape[k](2, 3));
		qDebug("%3.3lf\t%3.3lf\t%3.3lf\t%3.3lf", initShape[k](3, 0), initShape[k](3, 1), initShape[k](3, 2), initShape[k](3, 3));
		qDebug("----------\n");
	}*/
	//
	//static bool global_init = false;

	//rigid pos: original pos
	//deformed pos: now pos
	Eigen::MatrixXd rigidPos = Eigen::MatrixXd::Zero(3, 4);
	Eigen::MatrixXd deformedPos = Eigen::MatrixXd::Zero(3, 4);
	int outputCount = 0;

	int counter_body = 0;
	int counter_chamber = 0;


	//For the deformation usage, every tetra should remain its initial shape
	for (GLKPOSITION Pos = TetMesh->GetTetraList().GetHeadPosition(); Pos;) {
		QMeshTetra* Tet = (QMeshTetra*)TetMesh->GetTetraList().GetNext(Pos);
		int fdx = Tet->GetIndexNo(); if (fdx < 0) continue;




		QMeshNode* nodes[VELE];
		Eigen::MatrixXd P = Eigen::MatrixXd::Zero(3, VELE);

		double center[3] = { 0,0,0 };			//initial center
		double centerCur[3] = { 0,0,0 };		//current center


		for (int i = 0; i < VELE; i++) {
			nodes[i] = Tet->GetNodeRecordPtr(i + 1);
			nodes[i]->GetCoord3D_last(P(0, i), P(1, i), P(2, i));
			nodes[i]->GetCoord3D_last(rigidPos(0, i), rigidPos(1, i), rigidPos(2, i));
			nodes[i]->GetCoord3D(deformedPos(0, i), deformedPos(1, i), deformedPos(2, i));

			for (int j = 0; j < 3; j++)
			{
				center[j] += P(j, i);
				centerCur[j] += deformedPos(j, i);
			}
		}

		for (int j = 0; j < 3; j++)
		{
			center[j] /= (double)VELE;
			centerCur[j] /= (double)VELE;
		}

		double expandRatio = 1.0;
		for (int i = 0; i < VELE; i++) {
			for (int j = 0; j < 3; j++) {

				rigidPos(j, i) -= (center[j]);
				deformedPos(j, i) -= (centerCur[j]);
			}
		}

		//we should keep body element
		//we should keep body element
		//rigidPos = initShape[bodyEleIndex];
		/*if (Tet->isChamber[0] == false)
			rigidPos = initShape[counter_body];*/

		///////////////////Eigen SVD decomposition/////////////////////////


		JacobiSVD<Eigen::MatrixXd> svd(rigidPos * deformedPos.transpose(), ComputeThinU | ComputeThinV);
		Eigen::MatrixXd R = svd.matrixU() * svd.matrixV().transpose();

		//deformPos : 3x4
		//here, volume is not kept here
		LocalGoal[fdx] = (1 - material_omega) * R * deformedPos + material_omega * rigidPos;
		//
		double expansionRatio = fabs(_calEleVolume(rigidPos) / _calEleVolume(LocalGoal[fdx]));
		/*if (fdx % 1000 == 0)
		{
			printf("(%5d) (%.3lf,%.3lf) expansion ratio is %lf\n", fdx, _calEleVolume(rigidPos), _calEleVolume(LocalGoal[fdx]), expansionRatio);
			cout << "initial mat is\n" << rigidPos << endl << "current mat is\n" << deformedPos << endl;
			cout << "mixed mat is\n" << LocalGoal[fdx] << endl << endl << endl;
		}*/
		Eigen::Vector3d center_localGoal;
		//3x4 
		center_localGoal = LocalGoal[fdx].rowwise().mean();
		for (int j = 0; j < 4; j++) LocalGoal[fdx].col(j) = LocalGoal[fdx].col(j) - center_localGoal;

		LocalGoal[fdx] = LocalGoal[fdx] * pow(expansionRatio, 1.0 / 3);
		//qDebug("Body tet..");



		if (Tet->isChamber[0] == false)
			counter_body++;
		else
			counter_chamber++;

	}
	printf("Finish Compute local inverse!\nFinish Compute Local Goal...\n");

	qDebug("\n*************************************\n");
	qDebug("counter_body is %d and counter_chamber is %d and sum is %d", counter_body, counter_chamber, counter_body + counter_chamber);
	qDebug("total number of tet element is %d", TetMesh->GetTetraNumber());

	qDebug("\n*************************************\n");
}

void DeformTet::LocalProjection_CR_All()
{
	//qDebug("TetMesh num is %d", TetMesh->GetTetraNumber());
#pragma omp parallel   
	{
#pragma omp for 
		for (int i = 0; i < TetMesh->GetTetraNumber(); i++) {

			int fdx;
			QMeshTetra* Tetra;
			try 
			{
				Tetra = tetraSet[i];
				fdx = Tetra->GetIndexNo();
				if (fdx < 0) continue;

				this->LocalProjection_singleTetSVD(Tetra);
			}
			catch (...)
			{
				qDebug("Tetra %d idx %d",i,fdx);
			}
			 
		}
	}

}

//debugging
void DeformTet::FillVectorB_CR_All()
{
	{
		double c1 = -1.0 / VELE, c2 = 1 + c1;

		int Core = 12;
		int EachCore = eleNum / Core + 1;
		//int n = 0;

#pragma omp parallel   
		{
#pragma omp for  
			for (int omptime = 0; omptime < Core; omptime++)
			{
				//int BeginTetraIndex = EachCore * omptime + 1;
				//				if (TetraIndex > TetraNumSum) break;
				for (GLKPOSITION Pos = TetMesh->GetTetraList().GetHeadPosition(); Pos;)
				{
					QMeshTetra* Tetra = (QMeshTetra*)TetMesh->GetTetraList().GetNext(Pos);

					if (Tetra->GetIndexNo() < omptime * EachCore) continue;
					else if (Tetra->GetIndexNo() > (1 + omptime) * EachCore) break;

					//handle the tetrahedron with some points are hard constraint: handle or fixture
					if (Tetra->GetIndexNo() != -1)
					{
						int fdx = Tetra->GetIndexNo();
						//double center[3]; face->CalCenterPos(center[0], center[1], center[2]);

						double weight = 1.0;
						if (Tetra->isRigid) weight = this->weightHard;

						double center[3] = { 0 };
						for (int i = 0; i < VELE; i++) for (int j = 0; j < 3; j++) center[j] += LocalCoord[fdx](j, i);
						for (int i = 0; i < 3; i++) center[i] /= VELE;

						Eigen::MatrixXd minusSum = Eigen::MatrixXd::Zero(4, 3);
						for (int i = 0; i < VELE; i++)
						{
							for (int j = 0; j < 3; j++) {
								VectorBSide[j](fdx* VELE + i) = (LocalCoord[fdx](j, i) - center[j]) * weight;
							}

							QMeshNode* node = Tetra->GetNodeRecordPtr(i + 1);
							if (node->GetIndexNo() < 0)
							{
								//calculate when the point of this tetrahedron is under hard constraint
								Eigen::MatrixXd left(4, 1), right(1, 3), minus(4, 3);
								node->GetCoord3D(right(0, 0), right(0, 1), right(0, 2));
								for (int k = 0; k < 4; k++)
								{
									if (k == i)
										left(k, 0) = c2;
									else
										left(k, 0) = c1;
								}

								minus = left * right;
								minusSum = minusSum + minus;

							}

						}

						//if the node is a hard constraint
						for (int i = 0; i < VELE; i++)
							for (int j = 0; j < 3; j++)
							{
								VectorBSide[j](fdx* VELE + i) = VectorBSide[j](fdx* VELE + i) - minusSum(i, j);
							}
					}


				}



			}
		}
	}

	////fill the collision response string 
	//self collision spring
	int fdx = VELE * eleNum;
	int springIdx = 0;
	double springPos[3] = { 0.0,0.0,0.0 };
	for (GLKPOSITION Pos = TetMesh->GetNodeList().GetHeadPosition(); Pos;) {
		QMeshNode* Node = (QMeshNode*)TetMesh->GetNodeList().GetNext(Pos);

		//if node should be added spring
		if (Node->isCollided == true)
		{
			int verIdx = Node->GetIndexNo();
			//each node there should be a array recording the position of the spring attachment
			for (int j = 0; j < 3; j++)
				//here should be the spring original position
				VectorBSide[j](fdx + springIdx) = weightSpring * coeff_extension_spring * Node->_rayDir[j];


			springIdx++;
		}



	}


	//collision with environment
	fdx = VELE * eleNum + springNum;
	springIdx = 0;
	for (GLKPOSITION Pos = TetMesh->GetNodeList().GetHeadPosition(); Pos;) {
		QMeshNode* Node = (QMeshNode*)TetMesh->GetNodeList().GetNext(Pos);

		//if node should be added spring
		if (Node->isCollided_env == true)
		{
			int verIdx = Node->GetIndexNo();
			//each node there should be a array recording the position of the spring attachment
			for (int j = 0; j < 3; j++)
				//here should be the spring original position
				VectorBSide[j](fdx + springIdx) = weightSpring_env * Node->closestPntOnObstacle[j];


			springIdx++;
		}



	}


}

void DeformTet::PreProcess_CR_All()
{
	Initialization_CR_All();
	//ComputeLocalGoalAndInverse_CR_All();

	FillMatrixA_CR_All();
	FactorizeMatrixA_CR_All();
	cout << "finish preprocess the system" << endl;
}

bool DeformTet::RunWithCollisionResponse(std::vector<Eigen::MatrixXd>& initShape, int loop)
{
	
	//collision detection with environment and correspondence calculation
	//CollisionDetection();
	if (TetMesh->collidedPntNum == 0 && TetMesh->collidedPntNum_env == 0)
	{
		//there is no collision, so return.
		return true;
	}

	//qDebug("Self Collision Point: %d, While Collision Env Point: %d", TetMesh->collidedPntNum, TetMesh->collidedPntNum_env);
	PreProcess_CR_All();
	//qDebug("Pre-processing done");
	ComputeLocalGoalAndInverse_CR_All(initShape);		//local goal should be updated.
	//qDebug("Local goal update done");
	for (int i = 0; i < loop; i++) 
	{
		//qDebug("Iter: %d",i);
		LocalProjection_CR_All();				//MAYBE PROBLEM
		//qDebug("Local projection done");		
		FillVectorB_CR_All();
		//qDebug("Fill vector B");
		Solve();
		//qDebug("Solve");
		if (!UpdateVertex()) return false;		//update vertex position and store old position
		////ShifttoInitialPos();
		//qDebug("Running local-global iteration: %d", i);
	}



	return true;
}



void DeformTet::Initialization()
{

	////set node and face number
	//vertNum = 0;
	//for (GLKPOSITION pos = TetMesh->GetNodeList().GetHeadPosition(); pos != nullptr;) {
	//	QMeshNode* node = (QMeshNode*)TetMesh->GetNodeList().GetNext(pos);
	//	double pp[3] = { 0 };
	//	node->GetCoord3D_last(pp[0], pp[1], pp[2]);
	//	node->SetCoord3D(pp[0], pp[1], pp[2]);
	//	if (node->isFixed == false && node->isHandle == false)
	//		node->SetIndexNo(vertNum++);
	//	else node->SetIndexNo(-1);

	//}
	//cout << "non-fixed vertex Num = " << TetMesh->GetNodeNumber() << endl;

	//eleNum = 0;
	//for (GLKPOSITION Pos = TetMesh->GetTetraList().GetHeadPosition(); Pos;) {
	//	QMeshTetra* Tet = (QMeshTetra*)TetMesh->GetTetraList().GetNext(Pos);
	//	if (Tet->IsFixed()) Tet->SetIndexNo(-1); // as long as single vertex is fixed, the tet is fixed
	//	else Tet->SetIndexNo(eleNum++);
	//}
	//cout << "non-fixed element Num = " << eleNum << endl;


	////initialize all computing matrix
	//matA.resize(VELE * eleNum, vertNum);
	//matAT.resize(vertNum, VELE * eleNum);

	//LocalCoord.resize(eleNum);
	//InverseP.resize(eleNum);
	//LocalGoal.resize(eleNum);
	//VectorXPosition.resize(3);
	//VectorXPosition_Last.resize(3);
	//VectorBSide.resize(3);

	//for (int i = 0; i < 3; i++) {
	//	VectorXPosition[i] = Eigen::VectorXd::Zero(vertNum);
	//	VectorXPosition_Last[i] = Eigen::VectorXd::Zero(vertNum);
	//	VectorBSide[i] = Eigen::VectorXd::Zero(eleNum * VELE);
	//}

	////this->ComputeLocalGoalAndInverse();
}

void DeformTet::Initialization_CR()
{

		//first execution: should build all matrices
		
		vertNum = 0;
		for (GLKPOSITION pos = TetMesh->GetNodeList().GetHeadPosition(); pos != nullptr;) {
			QMeshNode* node = (QMeshNode*)TetMesh->GetNodeList().GetNext(pos);
			double pp[3] = { 0 };
			/*node->GetCoord3D_last(pp[0], pp[1], pp[2]);
			node->SetCoord3D(pp[0], pp[1], pp[2]);*/
			if (node->isFixed == false && node->isHandle == false)
				node->SetIndexNo(vertNum++);
			else node->SetIndexNo(-1);

		}
		cout << "non-fixed vertex Num = " << TetMesh->GetNodeNumber() << endl;

		eleNum = 0;
		for (GLKPOSITION Pos = TetMesh->GetTetraList().GetHeadPosition(); Pos;) {
			QMeshTetra* Tet = (QMeshTetra*)TetMesh->GetTetraList().GetNext(Pos);
			//if (Tet->IsFixed()) Tet->SetIndexNo(-1); // as long as single vertex is fixed, the tet is fixed
			//else Tet->SetIndexNo(eleNum++);
			Tet->SetIndexNo(eleNum++);
		}
		cout << "non-fixed element Num = " << eleNum << endl;



		springNum = 0;
		for (GLKPOSITION pos = TetMesh->GetNodeList().GetHeadPosition(); pos != nullptr;)
		{
			QMeshNode* node = (QMeshNode*)TetMesh->GetNodeList().GetNext(pos);
			//check whether this node is in collision
			if(node->isCollided == true)springNum++;
		}
		cout << "spring Num = " << springNum << endl;

		//initialize all computing matrix
		matA.resize(VELE * eleNum + springNum, vertNum);							//A  Mat should added several springs
		matAT.resize(vertNum, VELE * eleNum + springNum);						//AT Mat should added several springs

		LocalCoord.resize(eleNum);
		InverseP.resize(eleNum);
		LocalGoal.resize(eleNum);
		VectorXPosition.resize(3);
		VectorXPosition_Last.resize(3);
		VectorBSide.resize(3);

		for (int i = 0; i < 3; i++) {
			VectorXPosition[i] = Eigen::VectorXd::Zero(vertNum);
			VectorXPosition_Last[i] = Eigen::VectorXd::Zero(vertNum);
			VectorBSide[i] = Eigen::VectorXd::Zero(eleNum * VELE + springNum);	//B  Mat should added several springs
		}


		flag_UpdateMatAMatB = true;	//should update B matrix as well


}

void DeformTet::ComputeLocalGoalAndInverse()
{
	//For the deformation usage, every tetra should remain its initial shape
	for (GLKPOSITION Pos = TetMesh->GetTetraList().GetHeadPosition(); Pos;) {
		QMeshTetra* Tet = (QMeshTetra*)TetMesh->GetTetraList().GetNext(Pos);
		int fdx = Tet->GetIndexNo(); if (fdx < 0) continue;

		QMeshNode* nodes[VELE];
		Eigen::MatrixXd P = Eigen::MatrixXd::Zero(3, VELE);
		double center[3] = { 0 };

		for (int i = 0; i < VELE; i++) {
			nodes[i] = Tet->GetNodeRecordPtr(i + 1);
			nodes[i]->GetCoord3D_last(P(0, i), P(1, i), P(2, i));
			for (int j = 0; j < 3; j++) center[j] += P(j, i);
		} for (int j = 0; j < 3; j++) center[j] /= VELE;


		double expandRatio = 1.0;
		for (int i = 0; i < VELE; i++) {
			for (int j = 0; j < 3; j++) {
				if (Tet->isChamber[0]) P(j, i) = (P(j, i) - center[j]) * pow(expandRatio, 1.0 / 3); // for pneumatic-driven soft robot simulation
				else P(j, i) -= center[j];
			}
		}


		//solving pseudoInverse with GLKMATRIX Lib, Eigen may occur error here
		LocalGoal[fdx] = P;
		InverseP[fdx] = (P.transpose()).completeOrthogonalDecomposition().pseudoInverse();

		if (InverseP[fdx].mean() - InverseP[fdx].mean() != 0) cout << InverseP[fdx] << " pseudo inverse ERROR!" << std::endl;

		//GLKMatrix GLKP(3, VELE), GLKInverseP(3, VELE);
		//InverseP[fdx] = Eigen::MatrixXd::Zero(3, 4);

		//for (int i = 0; i < 3; i++) { for (int j = 0; j < 4; j++)  GLKP(i, j) = P(i, j); }

		//GLKMatrix TP(VELE, 3), GLKATA(3, 3);
		//GLKMatrixLib::Transpose(GLKP, 3, VELE, TP);
		//GLKMatrixLib::Mul(GLKP, TP, 3, VELE, 3, GLKATA);

		//if (!GLKMatrixLib::Inverse(GLKATA, 3)) {
		//	printf("ERROR in finding Inverse!\n");
		//	getchar();
		//}
		//GLKMatrixLib::Mul(GLKATA, GLKP, 3, 3, VELE, GLKInverseP);

		//for (int i = 0; i < 3; i++) { for (int j = 0; j < 4; j++) InverseP[fdx](i, j) = GLKInverseP(i, j); }
	}
	printf("Finish Compute local inverse!\n");
}

void DeformTet::FillMatrixA()
{
	//give memory to sparse matrix, to accerate the insert speed
	matA.reserve(VectorXi::Constant(VELE * eleNum, 1000));

	float c1 = -1.0 / VELE, c2 = 1 + c1;

	for (GLKPOSITION Pos = TetMesh->GetTetraList().GetHeadPosition(); Pos;) {
		QMeshTetra* Tetra = (QMeshTetra*)TetMesh->GetTetraList().GetNext(Pos);
		if (Tetra->GetIndexNo() < 0) continue;

		int fdx = Tetra->GetIndexNo() * VELE;
		double weight = 1.0;

		int vdxArr[VELE];
		for (int i = 0; i < VELE; i++) vdxArr[i] = Tetra->GetNodeRecordPtr(i + 1)->GetIndexNo();

		for (int i = 0; i < VELE; i++) {
			if (vdxArr[i] < 0) continue;
			for (int j = 0; j < VELE; j++) {
				if (vdxArr[j] < 0) continue;
				if (i == j) matA.insert(fdx + j, vdxArr[i]) = c2 * weight;
				else matA.insert(fdx + j, vdxArr[i]) = c1 * weight;
			}
		}
	}

	matA.makeCompressed();
}

void DeformTet::FactorizeMatrixA()
{
	Eigen::SparseMatrix<double> matATA(vertNum, vertNum);

	matAT = matA.transpose();
	matATA = matAT * matA;

	Solver.compute(matATA);
	printf("end factorize materix A\n");
}

void DeformTet::FillVectorB()
{
	{
		double c1 = -1.0 / VELE, c2 = 1 + c1;

		int Core = 12;
		int EachCore = eleNum / Core + 1;
		//int n = 0;

#pragma omp parallel   
		{
#pragma omp for  
			for (int omptime = 0; omptime < Core; omptime++) {
				//int BeginTetraIndex = EachCore * omptime + 1;
				//				if (TetraIndex > TetraNumSum) break;
				for (GLKPOSITION Pos = TetMesh->GetTetraList().GetHeadPosition(); Pos;) {
					QMeshTetra* Tetra = (QMeshTetra*)TetMesh->GetTetraList().GetNext(Pos);

					if (Tetra->GetIndexNo() < omptime * EachCore) continue;
					else if (Tetra->GetIndexNo() > (1 + omptime) * EachCore) break;

					if (Tetra->GetIndexNo() != -1)
					{
						int fdx = Tetra->GetIndexNo();
						//double center[3]; face->CalCenterPos(center[0], center[1], center[2]);

						double weight = 1.0;

						double center[3] = { 0 };
						for (int i = 0; i < VELE; i++) for (int j = 0; j < 3; j++) center[j] += LocalCoord[fdx](j, i);
						for (int i = 0; i < 3; i++) center[i] /= VELE;

						Eigen::MatrixXd minusSum = Eigen::MatrixXd::Zero(4,3);
						for (int i = 0; i < VELE; i++)
						{
							for (int j = 0; j < 3; j++) {
								VectorBSide[j](fdx* VELE + i) = (LocalCoord[fdx](j, i) - center[j]) * weight;
							}

							QMeshNode* node = Tetra->GetNodeRecordPtr(i + 1);
							if (node->GetIndexNo() < 0)
							{
								//calculate when the point of this tetrahedron is under hard constraint
								Eigen::MatrixXd left(4, 1), right(1, 3), minus(4, 3);
								node->GetCoord3D(right(0, 0), right(0, 1), right(0, 2));
								for (int k = 0; k < 4; k++)
								{
									if (k == i)
										left(k, 0) = c2;
									else
										left(k, 0) = c1;
								}

								minus = left * right;
								minusSum = minusSum + minus;
								
							}
							
						}

						//if the node is a hard constraint
						for (int i = 0; i < VELE; i++)
							for (int j = 0; j < 3; j++)
							{
								VectorBSide[j](fdx* VELE + i) = VectorBSide[j](fdx* VELE + i)- minusSum(i,j);
							}
					}
				}
			}
		}
	}
}

bool DeformTet::Solve()
{
#pragma omp parallel
	{
#pragma omp for
		for (int i = 0; i < 3; i++) {
			Eigen::VectorXd ATB = matAT * VectorBSide[i];
			VectorXPosition[i] = Solver.solve(ATB);
		}
	}
	return true;
}

void DeformTet::LocalProjection()
{
#pragma omp parallel   
	{
#pragma omp for 
		for (int i = 0; i < TetMesh->GetTetraNumber(); i++) {

			QMeshTetra* Tetra = tetraSet[i];
			int fdx = Tetra->GetIndexNo(); if (fdx < 0) continue;

			this->LocalProjection_singleTetSVD(Tetra);
		}
	}
}

void DeformTet::LocalProjection_CR()
{
#pragma omp parallel   
	{
#pragma omp for 
		for (int i = 0; i < TetMesh->GetTetraNumber(); i++) {

			QMeshTetra* Tetra = tetraSet[i];
			int fdx = Tetra->GetIndexNo(); if (fdx < 0) continue;

			this->LocalProjection_singleTetSVD(Tetra);
		}
	}
}



void DeformTet::LocalProjection_singleTetSVD(QMeshTetra* Tetra)
{
	
	//int fdx = Tetra->GetIndexNo();
	//double center[3] = { 0 };
	//Tetra->CalCenterPos(center[0], center[1], center[2]);

	////This tP should be put ahead to see if it works for the energy function.
	//Eigen::MatrixXd tP = Eigen::MatrixXd::Zero(VELE, 3); QMeshNode* nodes[VELE];
	//for (int i = 0; i < VELE; i++) {
	//	nodes[i] = Tetra->GetNodeRecordPtr(i + 1);
	//	nodes[i]->GetCoord3D(tP(i, 0), tP(i, 1), tP(i, 2));
	//}
	//for (int i = 0; i < VELE; i++) for (int j = 0; j < 3; j++) tP(i, j) -= center[j];

	//Eigen::Matrix3d T = Matrix3d::Zero(3, 3),
	//	T_transpose = Matrix3d::Zero(3, 3), R = Matrix3d::Zero(3, 3);
	//T = InverseP[fdx] * tP;
	//T_transpose = T.transpose();

	///////////////////Eigen SVD decomposition/////////////////////////

	//JacobiSVD<Eigen::MatrixXd> svd(T_transpose, ComputeThinU | ComputeThinV);
	//Matrix3d V = svd.matrixV(), U = svd.matrixU();
	//R = U * V.transpose();

	//LocalCoord[fdx] = R * LocalGoal[fdx];

	//new 
	
	int fdx = Tetra->GetIndexNo();
	double center[3] = { 0 };
	Tetra->CalCenterPos(center[0], center[1], center[2]);

	//This tP should be put ahead to see if it works for the energy function.
	Eigen::MatrixXd tP = Eigen::MatrixXd::Zero(VELE, 3); QMeshNode* nodes[VELE];
	for (int i = 0; i < VELE; i++) {
		nodes[i] = Tetra->GetNodeRecordPtr(i + 1);
		nodes[i]->GetCoord3D(tP(i, 0), tP(i, 1), tP(i, 2));
	}
	for (int i = 0; i < VELE; i++) for (int j = 0; j < 3; j++) tP(i, j) -= center[j];

	try 
	{
	//in global blending, we want to calculate rotation matrix 
	//This is the new one
	//from element-wise targetShape (3x4) to currentShape (3x4)
	//SVD(currentShape * targetShape^T)
	//								 3x4			      4x3
		JacobiSVD<Eigen::MatrixXd> svd(tP.transpose() * LocalGoal[fdx].transpose(), ComputeThinU | ComputeThinV);
		LocalCoord[fdx] = svd.matrixU() * svd.matrixV().transpose() * LocalGoal[fdx];


	}
	catch (...)
	{
		auto a = tP.transpose() * LocalGoal[fdx].transpose();
		qDebug("\nidx: %d is:\n", fdx);
		cout << a << endl << "*****************\n";
	}
	

	
}

bool DeformTet::UpdateVertex()
{
	for (GLKPOSITION Pos = TetMesh->GetNodeList().GetHeadPosition(); Pos;) {
		QMeshNode* node = (QMeshNode*)TetMesh->GetNodeList().GetNext(Pos);
		int idx = node->GetIndexNo();
		//double xx, yy, zz;
		//node->GetCoord3D(xx, yy, zz);
		if (idx > -1)
			node->SetCoord3D(VectorXPosition[0](idx), VectorXPosition[1](idx), VectorXPosition[2](idx));
			//node->SetCoord3D(xx, VectorXPosition[1](idx), zz);
		//node->SetCoord3D(VectorXPosition[0](idx), VectorXPosition[1](idx), zz);

	}

	return true;
}

void DeformTet::ShifttoInitialPos() {
	//double CoordLast[3] = { 0 }, CoordCurrent[3] = { 0 }, shift[3] = { 0 };
	//int nodeNum = 0;
	//for (GLKPOSITION Pos = TetMesh->GetNodeList().GetHeadPosition(); Pos;) {
	//	QMeshNode* node = (QMeshNode*)TetMesh->GetNodeList().GetNext(Pos);
	//	if (node->selected == true) continue;
	//	node->GetCoord3D_last(CoordLast[0], CoordLast[1], CoordLast[2]);
	//	node->GetCoord3D(CoordCurrent[0], CoordCurrent[1], CoordCurrent[2]);
	//	for (int i = 0; i < 3; i++) shift[i] += CoordCurrent[i] - CoordLast[i];
	//	nodeNum++;
	//}
	//for (int i = 0; i < 3; i++) shift[i] /= (double)nodeNum;
	////cout << nodeNum << " , " << shift[0] <<" , "<< shift[1] << " , " << shift[2] << endl;
	//for (GLKPOSITION Pos = TetMesh->GetNodeList().GetHeadPosition(); Pos;) {
	//	QMeshNode* node = (QMeshNode*)TetMesh->GetNodeList().GetNext(Pos);
	//	node->GetCoord3D(CoordCurrent[0], CoordCurrent[1], CoordCurrent[2]);
	//	node->SetCoord3D(CoordCurrent[0] - shift[0], CoordCurrent[1] - shift[1], CoordCurrent[2] - shift[2]);
	//	node->CalNormal();
	//}


	double pp[3] = { 0 };
	int select_num = 0;
	for (GLKPOSITION Pos = TetMesh->GetNodeList().GetHeadPosition(); Pos;) {
		QMeshNode* Node = (QMeshNode*)TetMesh->GetNodeList().GetNext(Pos);
		if (Node->shirftSelect == true)select_num++;
	}

	Eigen::MatrixXd shiftCurrent = Eigen::MatrixXd::Constant(select_num, 3, 1.0);
	Eigen::Vector3d shiftCenterCurrent = Eigen::Vector3d::Zero();

	Eigen::MatrixXd shiftOrig = Eigen::MatrixXd::Constant(select_num, 3, 1.0);
	Eigen::Vector3d shiftCenterOrig = Eigen::Vector3d::Zero();


	select_num = 0;
	double shift[3] = { 0 };
	for (GLKPOSITION Pos = TetMesh->GetNodeList().GetHeadPosition(); Pos;) {
		QMeshNode* Node = (QMeshNode*)TetMesh->GetNodeList().GetNext(Pos);
		if (Node->shirftSelect == true)
		{
			Node->GetCoord3D_last(shiftOrig(select_num, 0), shiftOrig(select_num, 1), shiftOrig(select_num, 2));
			Node->GetCoord3D(shiftCurrent(select_num, 0), shiftCurrent(select_num, 1), shiftCurrent(select_num, 2));
			select_num++;
		}
	}

	for (int index = 0; index < select_num; index++) {
		for (int i = 0; i < 3; i++) {
			shiftCenterCurrent[i] += shiftCurrent(index, i) / select_num;
			shiftCenterOrig[i] += shiftOrig(index, i) / select_num;
		}
			
	}
	
	for (int i = 0; i < select_num; i++) {
		shiftCurrent.row(i) -= shiftCenterCurrent;
		shiftOrig.row(i) -= shiftCenterOrig;
	}
	Eigen::JacobiSVD<Eigen::MatrixXd> svd(shiftOrig.transpose() * shiftCurrent, Eigen::ComputeThinU | Eigen::ComputeThinV);
	Eigen::Matrix3d rotateMat = svd.matrixU() * (svd.matrixV().transpose());

	Eigen::Vector3d t = -rotateMat * shiftCenterCurrent + shiftCenterOrig;


	Eigen::Vector3d centerPos = Eigen::Vector3d::Zero();
	for (GLKPOSITION Pos = TetMesh->GetNodeList().GetHeadPosition(); Pos;) {
		QMeshNode* Node = (QMeshNode*)TetMesh->GetNodeList().GetNext(Pos);
		Vector3d pp;
		Node->GetCoord3D(pp);
		pp = rotateMat * pp + t;
		Node->SetCoord3D(pp);
		centerPos += pp;
	}

	centerPos /= TetMesh->GetNodeNumber();
	if (centerPos[0] < 0) {
		for (GLKPOSITION Pos = TetMesh->GetNodeList().GetHeadPosition(); Pos;) {
			QMeshNode* Node = (QMeshNode*)TetMesh->GetNodeList().GetNext(Pos);
			Vector3d pp;
			Node->GetCoord3D(pp);
			Node->SetCoord3D(-pp(0),pp(1),pp(2));
		}
	}


}

bool DeformTet::_inCollisionPlane(double x, double z)
{
	if (x< CD_XMIN || x>CD_XMAX || z< CD_ZMIN || z>CD_ZMAX)
		return false;
	else
		return true;
}


////////collision response////////////////
void DeformTet::FillVectorB_CR()
{
	{
		double c1 = -1.0 / VELE, c2 = 1 + c1;

		int Core = 12;
		int EachCore = eleNum / Core + 1;
		//int n = 0;

#pragma omp parallel   
		{
#pragma omp for  
			for (int omptime = 0; omptime < Core; omptime++) 
			{
				//int BeginTetraIndex = EachCore * omptime + 1;
				//				if (TetraIndex > TetraNumSum) break;
				for (GLKPOSITION Pos = TetMesh->GetTetraList().GetHeadPosition(); Pos;) 
				{
					QMeshTetra* Tetra = (QMeshTetra*)TetMesh->GetTetraList().GetNext(Pos);

					if (Tetra->GetIndexNo() < omptime * EachCore) continue;
					else if (Tetra->GetIndexNo() > (1 + omptime) * EachCore) break;

					//handle the tetrahedron with some points are hard constraint: handle or fixture
					if (Tetra->GetIndexNo() != -1)
					{
						int fdx = Tetra->GetIndexNo();
						//double center[3]; face->CalCenterPos(center[0], center[1], center[2]);

						double weight = 1.0;
						if (Tetra->isChamber[0]) weight = weightChamble;
						if (Tetra->isRigid) weight = weightHard;

						double center[3] = { 0 };
						for (int i = 0; i < VELE; i++) for (int j = 0; j < 3; j++) center[j] += LocalCoord[fdx](j, i);
						for (int i = 0; i < 3; i++) center[i] /= VELE;

						Eigen::MatrixXd minusSum = Eigen::MatrixXd::Zero(4, 3);
						for (int i = 0; i < VELE; i++)
						{
							for (int j = 0; j < 3; j++) {
								VectorBSide[j](fdx* VELE + i) = (LocalCoord[fdx](j, i) - center[j]) * weight;
							}

							QMeshNode* node = Tetra->GetNodeRecordPtr(i + 1);
							if (node->GetIndexNo() < 0)
							{
								//calculate when the point of this tetrahedron is under hard constraint
								Eigen::MatrixXd left(4, 1), right(1, 3), minus(4, 3);
								node->GetCoord3D(right(0, 0), right(0, 1), right(0, 2));
								for (int k = 0; k < 4; k++)
								{
									if (k == i)
										left(k, 0) = c2;
									else
										left(k, 0) = c1;
								}

								minus = left * right;
								minusSum = minusSum + minus;

							}

						}

						//if the node is a hard constraint
						for (int i = 0; i < VELE; i++)
							for (int j = 0; j < 3; j++)
							{
								VectorBSide[j](fdx* VELE + i) = VectorBSide[j](fdx* VELE + i) - minusSum(i, j);
							}
					}

					
				}

				

			}
		}
	}

	////fill the collision response string 
	//traverse around the node list to see whether it should be attached a spring
	//traverse the node list and add spring to the last several rows of A matrix
	int fdx = VELE * eleNum;
	int springIdx = 0;
	double springPos[3] = {0.0,0.0,0.0};
	for (GLKPOSITION Pos = TetMesh->GetNodeList().GetHeadPosition(); Pos;) {
		QMeshNode* Node = (QMeshNode*)TetMesh->GetNodeList().GetNext(Pos);

		//if node should be added spring
		if (Node->isCollided == true)
		{
			int verIdx = Node->GetIndexNo();
			//each node there should be a array recording the position of the spring attachment
			for (int j = 0; j < 3; j++)
				//here should be the spring original position
				VectorBSide[j](fdx + springIdx) = weightSpring * coeff_extension_spring * Node->_rayDir[j];


			springIdx++;
		}
		


	}


}
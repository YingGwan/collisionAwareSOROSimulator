#include "MainWindow.h"
#include "ui_MainWindow.h"
#include "soroPneumaticKinematics.h"
#include "AABB.h"
#include "DeformTet.h"

void MainWindow::createActions_SoftRobotFunction() {

	connect(ui->pushButton_generateTETMesh, SIGNAL(released()), this, SLOT(soroSimulation_highExpandingModel()));
	//connect(ui->pushButton_generateTETMesh, SIGNAL(released()), this, SLOT(soroSimulation_dataGeneration()));
	connect(ui->pushButton_runCollisionChecking, SIGNAL(released()), this, SLOT(collisionCheckingByAABBTree()));

	
}
//soroPneumaticKinematics* soroIKOperator;    //Deformation
/*------------------------------------------------------*/
// new function for high-expanding ratio soft robot design
/*------------------------------------------------------*/
//main class object
meshOperation* meshOperator;                //Actuation element remeshing
soroPneumaticKinematics* soroIKOperator;    //Deformation
AABBManager* collisionOperator;				//collision detection and correspondence calculation
DeformTet* deformTetOperator;				//deformation with collision
std::vector<Eigen::MatrixXd> initShape;
void MainWindow::soroSimulation_highExpandingModel()
{
	//static int counter_run = 0;
	//counter_run++;
	////fingerNew
	////twisting
	//std::string modelName = "fingerNew";
	//if (counter_run == 1)
	//{
	//	meshOperator = new meshOperation;
	//	double expandVolume = 5.0;

	//	/* ---- input model ---- */
	//	body_init = new QMeshPatch;
	//	chamber_init = new QMeshPatch;
	//	tetMesh = new QMeshPatch;
	//	soroIKOperator = new soroPneumaticKinematics;

	//	collisionOperator = new AABBManager;
	//	deformTetOperator = new DeformTet;

	//	deformTetOperator->getExpansionRatio(ui->doubleSpinBox_A1->value());
	//	this->inputSoftRobotModel(body_init, chamber_init, tetMesh, meshOperator, modelName);
	//}



	///* ---- simulation parameter ---- */
	//int remeshTime = ui->spinBox_iterTime->value();

	//Eigen::VectorXd chamberVolume(remeshTime + 1);
	//std::string tetgenCommand = "Ya0.5";
	//
	//int maxitertime = 1000;

	//long time = clock();

	///* begin iteration */
	//for (int iter = 0; iter < remeshTime; iter++)
	//{

	//	std::cout << " -------------------------------------------------- " <<
	//		std::endl << " begin iteration #" << iter << std::endl <<
	//		" -------------------------------------------------- " << std::endl << std::endl;

	//	/* -----------------------------------------
	//	 generate/update TET mesh for simulation
	//	-------------------------------------------*/

	//	if (iter == 0 && counter_run==1) {
	//		meshOperator->tetMeshGeneration_outerSkin_Chamber(body_init, chamber_init, tetMesh, tetgenCommand);
	//		//InputEnvironmentObstacle();
	//		//pGLK->refresh(true);
	//		//return;
	//		meshOperator->materialSpaceSelection_rigid(tetMesh, modelName);
	//		//meshOperator->buildTopologyConnection_chamberSelection_FAST(tetMesh, body_init, chamber_init, true);
	//		if(ui->checkBox_readChamberRegion->isChecked()==true)
	//			meshOperator->chamberSelection(tetMesh, chamber_init,true);
	//		else
	//			meshOperator->chamberSelection(tetMesh, chamber_init, false);
	//		//meshOperator->chamberSelection_PQP(tetMesh, chamber_init, true);
	//	}
	//	else 
	//	{
	//		std::vector<Eigen::Vector3d> rigidNodePosSet; 
	//		meshOperator->saveRigidRegion(tetMesh, rigidNodePosSet, true);
	//		//meshOperator->tetMeshGeneration_outerSkinProtection_ChamberRemesh(body_init, chamber_init, tetMesh, tetgenCommand);
	//		meshOperator->tetMeshGeneration_outerSkinProtection_ChamberRemesh_New(body_init, chamber_init, tetMesh, tetgenCommand);
	//		meshOperator->loadRigidRegion(tetMesh, rigidNodePosSet);			
	//	}

	//	std::cout << std::endl << " ---- update chamber region selection..." << std::endl;

	//	//meshOperator->topologyCheck(tetMesh);

	//	/* build connection between tetMesh and chamber / body mesh --- Guanatee the topology of mesh won't change */
	//	//meshOperator->chamberSelection(tetMesh, chamber_init);
	//	//meshOperator->chamberSelection_PQP(tetMesh, chamber_init);

	//	meshOperator->buildTopologyConnection(tetMesh, body_init, chamber_init);
	//	meshOperator->selectShift_rigidRegion(tetMesh, body_init);
	//	std::cout << std::endl << " ---- build topology connection between tetMesh and chamber mesh..." << std::endl;
	//	
	//	/* -----------------------------------------
	//	 Run Simulation
	//	-------------------------------------------*/
	//	long simTime = clock();
	//	if (iter == 0 && counter_run == 1) chamberVolume(iter) = meshOperator->chamberVolumeEvaluation(tetMesh);

	//	soroIKOperator->getMesh(tetMesh, maxitertime);
	//	/*soroIKOperator = new soroPneumaticKinematics(tetMesh, maxitertime);*/
	//	soroIKOperator->expandRatio[0] = ui->doubleSpinBox_A1->value();

	//	/*if (iter == 0) soroIKOperator->expandRatio[0] = expandVolume;
	//	else soroIKOperator->expandRatio[0] = 2 * (iter + 1) / iter;*/

	//	if (iter == 0 && counter_run == 1) soroIKOperator->buildSimulationSystem(initShape, true);
	//	else soroIKOperator->buildSimulationSystem(initShape, false);

	//	qDebug("8");
	//	//soroIKOperator->updateExpandRatio(maxitertime);
	//	soroIKOperator->hyberSolverSingle(maxitertime, false);
	//	qDebug("9");
	//	//delete soroIKOperator;
	//	printf("Simulation time: %3.3lf s\n", (double)(clock() - simTime) / (double)(CLOCKS_PER_SEC));

	//	//meshOperator->topologyCheck(tetMesh);
	//	//meshOperator->updateOBJMesh_chamber(chamber_init); meshOperator->updateOBJMesh_skin(body_init);
	//	//meshOperator->outputSimulationResult(tetMesh, body_init, chamber_init, modelName, iter, true, false);
	//	/*if (counter_run == 2)
	//	{
	//		pGLK->refresh(true);
	//		return;
	//	}*/

	//	/* collision checking and response */
	//	if (modelName == "fingerNew") 
	//	{
	//		static bool init_aabb = false;
	//		qDebug("10");
	//		deformTetOperator->setLastCoordinate(tetMesh);	//first set last coord.
	//		qDebug("11");
	//		if (!init_aabb)
	//		{
	//			init_aabb = true;
	//			collisionOperator->TreeConstructionTop2Bot(tetMesh);
	//			deformTetOperator->SetMesh(tetMesh);
	//			InputEnvironmentObstacle();
	//			qDebug("12");
	//		}
	//		else
	//		{
	//			deformTetOperator->SetMesh(tetMesh);
	//			collisionOperator->TreeConstructionTop2Bot_rebuild(tetMesh);			//deformable obj AABB Tree
	//			qDebug("12.5");
	//		}
	//		
	//		collisionOperator->TreeConstructionTop2Bot_refit_obstacle();			//refit obstacle AABB Tree
	//		qDebug("13");

	//		for (int collIter = 0; collIter < 4; collIter++) {
	//			//Refit deformable object AABB Tree
	//			collisionOperator->TreeConstructionTop2Bot_refit(tetMesh);
	//			qDebug("%d, 14", collIter);
	//			//Self collision detection of deformable object
	//			collisionOperator->SelfCollisionDetectionCorrespondenceChecking();
	//			qDebug("%d, 15", collIter);
	//			//Collision checking with env and find the correspondence
	//			collisionOperator->CollisionWithEnvQueryChecking();
	//			qDebug("%d, 16", collIter);
	//			////Collision checking with env and find the correspondence
	//			deformTetOperator->RunWithCollisionResponse(30);
	//			qDebug("%d, 17", collIter);
	//		}

	//	}

	//	/* -----------------------------------------
	//	 update pos, compute statics and output mesh
	//	-------------------------------------------*/

	//	chamberVolume(iter + 1) = meshOperator->chamberVolumeEvaluation(tetMesh);
	//
	//	//meshOperator->updateOBJMesh_chamber(tetMesh,chamber_init);
	//	meshOperator->updateOBJMesh_chamber(chamber_init); meshOperator->updateOBJMesh_skin(body_init);

	//	meshOperator->laplacianSmoothSurface(chamber_init, tetMesh);

	//	meshOperator->outputSimulationResult(tetMesh, body_init, chamber_init, modelName, iter, true, true);


	//}
	//printf(" \n TIME - simulation takes %ld ms.\n", clock() - time);


	//std::cout << std::endl << "Chamber Volume = " <<
	//std::endl << chamberVolume / chamberVolume(0) << std::endl;
	//
	//

	//pGLK->refresh(true);
	//return;

}

void MainWindow::soroGenerateTetraMesh_withChamber() {

	/* ----- detect chamber tetrahedral ---- */

	PolygenMesh* soroBody; PolygenMesh* chamberMesh;
	int meshIndex = 0;
	for (GLKPOSITION pos = polygenMeshList.GetHeadPosition(); pos != nullptr;) {
		PolygenMesh* thispolygenMesh = (PolygenMesh*)polygenMeshList.GetNext(pos);
		if (meshIndex == 0) soroBody = thispolygenMesh;
		else chamberMesh = thispolygenMesh;
		meshIndex++;
	}
	QMeshPatch* body = (QMeshPatch*)soroBody->GetMeshList().GetHead();
	QMeshPatch* chamber = (QMeshPatch*)chamberMesh->GetMeshList().GetHead();

	// generate a new tet model
	PolygenMesh* polygenMesh = this->_buildPolygenMesh("TETModel");
	polygenMesh->isVolume = true;

	QMeshPatch* tetMesh = new QMeshPatch;
	polygenMesh->meshList.AddTail(tetMesh);


	meshOperation* meshOperator = new meshOperation;
	meshOperator->tetMeshGeneration_outerSkin_Chamber(body, chamber, tetMesh, "Ya1.5");

	/* select chamber region */
	meshOperator->chamberSelection(tetMesh, chamber);

	pGLK->refresh(true);

}

void MainWindow::inputSoftRobotModel(
	QMeshPatch* body_init, QMeshPatch* chamber_init, QMeshPatch* tetMesh,
	meshOperation* meshOperator, std::string modelName) 
{

	//bool modelCombined = ui->checkBox_combinedInput->ischecked();
	bool modelCombined = ui->checkBox_combinedInput->isChecked();
	PolygenMesh* surfaceMesh_body = this->_buildPolygenMesh(modelName+"_body");
	surfaceMesh_body->drawIdx = 0;
	surfaceMesh_body->meshList.AddTail(body_init);
	//surfaceMesh_body->setTransparent();

	PolygenMesh* surfaceMesh_chamber = this->_buildPolygenMesh(modelName+"_chamber");
	surfaceMesh_chamber->meshList.AddTail(chamber_init);
	surfaceMesh_chamber->drawIdx = 1;
	//surfaceMesh_chamber->setTransparent();

	/* generate TET mesh for simulation */
	PolygenMesh* volumeMesh = this->_buildPolygenMesh(modelName+"_TET");
	volumeMesh->isVolume = true;
	volumeMesh->drawIdx = 2;
	volumeMesh->meshList.AddTail(tetMesh);

	if (modelCombined) {
		/* seperate the mesh into body and chamber */
		QMeshPatch* inputMesh = new QMeshPatch;
		std::string inputFile = "../model/" + modelName + ".obj";

		inputMesh->inputOBJFile((char*)inputFile.c_str(), false);
		meshOperator->seperateMesh(inputMesh, body_init, chamber_init);
		inputMesh->ClearAll(); delete inputMesh;
	}
	else {
		/* input chamber and body as SURFACE mesh */
		std::string bodyFile = "../model/" + modelName + "_body.obj";
		body_init->inputOBJFile((char*)bodyFile.c_str(), false);

		std::string chamberFile = "../model/" + modelName + "_chamber.obj";
		chamber_init->inputOBJFile((char*)chamberFile.c_str(), false);
	}

	std::cout << " -- finish input initial model" << std::endl << std::endl;
	
	meshOperator->selectShift_rigidRegion(body_init, modelName);

}


void MainWindow::inputSoftRobotModelWithShifting(
	QMeshPatch* body_init, QMeshPatch* chamber_init, QMeshPatch* tetMesh,
	meshOperation* meshOperator, std::string modelName, std::string modelIdx, Eigen::Vector3d shift)
{
	//bool modelCombined = ui->checkBox_combinedInput->ischecked();
	bool modelCombined = ui->checkBox_combinedInput->isChecked();
	PolygenMesh* surfaceMesh_body = this->_buildPolygenMesh(modelName + modelIdx + "_body");
	surfaceMesh_body->drawIdx = 0;
	surfaceMesh_body->bVertexNormalShading = false;
	surfaceMesh_body->meshList.AddTail(body_init);
	//surfaceMesh_body->setTransparent();

	PolygenMesh* surfaceMesh_chamber = this->_buildPolygenMesh(modelName + modelIdx + "_chamber");
	surfaceMesh_chamber->meshList.AddTail(chamber_init);
	surfaceMesh_chamber->drawIdx = 1;
	surfaceMesh_chamber->bVertexNormalShading = false;
	//surfaceMesh_chamber->setTransparent();

	/* generate TET mesh for simulation */
	PolygenMesh* volumeMesh = this->_buildPolygenMesh(modelName + modelIdx + "_TET");
	volumeMesh->isVolume = true;
	volumeMesh->drawIdx = 2;
	volumeMesh->meshList.AddTail(tetMesh);

	if (modelCombined) {
		/* seperate the mesh into body and chamber */
		QMeshPatch* inputMesh = new QMeshPatch;
		std::string inputFile = "../model/" + modelName + ".obj";

		inputMesh->inputOBJFile((char*)inputFile.c_str(), false);
		meshOperator->seperateMesh(inputMesh, body_init, chamber_init);
		inputMesh->ClearAll(); delete inputMesh;
	}
	else {
		/* input chamber and body as SURFACE mesh */
		std::string bodyFile = "../model/" + modelName + "_body.obj";
		body_init->inputOBJFile((char*)bodyFile.c_str(), false);

		std::string chamberFile = "../model/" + modelName + "_chamber.obj";
		chamber_init->inputOBJFile((char*)chamberFile.c_str(), false);
	}

	

	std::cout << " -- finish input initial model" << std::endl << std::endl;

	meshOperator->selectShift_rigidRegion(body_init, modelName);

	Eigen::Vector3d nodePos;
	for (GLKPOSITION pos = body_init->GetNodeList().GetHeadPosition(); pos != nullptr;)
	{
		QMeshNode* Node = (QMeshNode*)body_init->GetNodeList().GetNext(pos);
		Node->GetCoord3D(nodePos[0], nodePos[1], nodePos[2]);
		nodePos = nodePos + shift;
		Node->SetCoord3D(nodePos[0], nodePos[1], nodePos[2]);
	}

	for (GLKPOSITION pos = chamber_init->GetNodeList().GetHeadPosition(); pos != nullptr;)
	{
		QMeshNode* Node = (QMeshNode*)chamber_init->GetNodeList().GetNext(pos);
		Node->GetCoord3D(nodePos[0], nodePos[1], nodePos[2]);
		nodePos = nodePos + shift;
		Node->SetCoord3D(nodePos[0], nodePos[1], nodePos[2]);
	}

	for (GLKPOSITION pos = tetMesh->GetNodeList().GetHeadPosition(); pos != nullptr;)
	{
		QMeshNode* Node = (QMeshNode*)tetMesh->GetNodeList().GetNext(pos);
		Node->GetCoord3D(nodePos[0], nodePos[1], nodePos[2]);
		nodePos = nodePos + shift;
		Node->SetCoord3D(nodePos[0], nodePos[1], nodePos[2]);
	}

	tetMesh->freeformChamberIdx = std::stoi(modelIdx);


}


void MainWindow::soroSimulation_dataGeneration() {
	for (int itertime = 1; itertime < 7; itertime++) {
		for (int actuTime = 1; actuTime < itertime + 3; actuTime++)
		{
			this->iterationTimeSOROSimulation = itertime;
			this->actuatorPara_twisting = 1.0 + 1.5 / (itertime + 2) * actuTime;
			//std::cout << this->actuatorPara_twisting << std::endl;
			//if (itertime == 1 && actuTime == 0) continue;
			this->soroSimulation_highExpandingModel();
		}
	}
}

void MainWindow::collisionCheckingByAABBTree() {

	/*PolygenMesh* pMesh = NULL;
	for (GLKPOSITION pos = polygenMeshList.GetHeadPosition(); pos != nullptr;) {
		PolygenMesh* polygen = (PolygenMesh*)polygenMeshList.GetNext(pos);
		if (polygen->getModelName() == "TET") { pMesh = polygen; break; }
	}
	if (!pMesh) return;*/
	PolygenMesh* pMesh = (PolygenMesh*)polygenMeshList.GetHead();
	QMeshPatch* patch = (QMeshPatch*)pMesh->GetMeshList().GetHead();

	AABBManager* aabbManager = new AABBManager;
	long startT = clock();
	aabbManager->TreeConstructionTop2Bot(patch);
	printf("Time Elapse of AABB Tree Construction: %3.3lf s\n", (double)(clock() - startT) / (double)(CLOCKS_PER_SEC));
	
	aabbManager->SelfCollisionDetection();
	aabbManager->SelfCollisionCorrespondenceCalculation();


}


void MainWindow::InputEnvironmentObstacle(void)
{
	std::string filePrefix = "..\\model\\";
	std::string filePostfix = "Cuboid9.tet";
	PolygenMesh* obstaclePoly = new PolygenMesh();
	char* cstr = new char[(filePrefix + filePostfix).length() + 1];
	strcpy(cstr, (filePrefix + filePostfix).c_str());

	obstaclePoly->ImportTETFile(cstr, filePostfix);
	//obstaclePoly->draw_idx = 1;

	polygenMeshList.AddTail(obstaclePoly);
	obstaclePoly->BuildGLList(obstaclePoly->m_bVertexNormalShading);
	pGLK->AddDisplayObj(obstaclePoly, true);

	obstaclePoly->bVertexNormalShading = false;
	QMeshPatch* _obPatch = (QMeshPatch*)obstaclePoly->GetMeshList().GetHead();

	double POS[3];
	int faceIdx = 0;
	//in order to visit faster
	/*_obPatch->m_faceArray = new QMeshFace * [_obPatch->GetFaceNumber()];
	for (GLKPOSITION pos = _obPatch->GetFaceList().GetHeadPosition(); pos != nullptr;)
	{
		QMeshFace* face = (QMeshFace*)_obPatch->GetFaceList().GetNext(pos);
		_obPatch->m_faceArray[faceIdx] = face;
		faceIdx++;
	}*/

	for (GLKPOSITION pos = _obPatch->GetNodeList().GetHeadPosition(); pos != nullptr;)
	{
		QMeshNode* Node = (QMeshNode*)_obPatch->GetNodeList().GetNext(pos);
		Node->GetCoord3D(POS[0], POS[1], POS[2]);
		Node->SetCoord3D(POS[0] - 20, POS[1] + 30, POS[2]);
	}

	deformTetOperator->SetObstacleMesh(_obPatch, true);
	collisionOperator->GetVolumeObstacleMesh(_obPatch);

	//build volume obstacle aabb tree
	collisionOperator->ObstacleTreeConstructionTop2Bot_rebuild();
	qDebug("Input obstacle volume mesh and build AABB tree ends...");

}

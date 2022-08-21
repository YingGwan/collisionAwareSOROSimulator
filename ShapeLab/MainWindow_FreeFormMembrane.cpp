#include "MainWindow.h"
#include "ui_MainWindow.h"
#include<cmath>
void MainWindow::SignalSlotConnection(void)
{
    //void InputMem(void);
    connect(ui->pushButton_inputFourChambers, SIGNAL(released()), this, SLOT(InputFreeFormMembrane()));
    connect(ui->pushButton_GenerateChamberTetMesh, SIGNAL(released()), this, SLOT(GenerateChamberTetMesh()));
    connect(ui->pushButton_ChamberDeformation, SIGNAL(released()), this, SLOT(ChamberDeformation()));

    connect(ui->pushButton_inputMem, SIGNAL(released()), this, SLOT(InputMem()));

    connect(ui->pushButton_CollisionChecking, SIGNAL(released()), this, SLOT(CollisionChecking()));
    connect(ui->pushButton_CollisionResponse, SIGNAL(released()), this, SLOT(CollisionResponse()));
 
    //pushButton_inputMem
    connect(ui->pushButton_MoveBall, SIGNAL(released()), this, SLOT(MoveBall()));
    //

    connect(ui->pushButton_trajectoryGeneration, SIGNAL(released()), this, SLOT(TrajGeneration()));
    connect(ui->pushButton_showAABB_Tree, SIGNAL(released()), this, SLOT(ShowAABBTree()));
    
    //checkBox_onlyLeafNode :stateChanged
    connect(ui->checkBox_onlyLeafNode, SIGNAL(stateChanged(int)), this, SLOT(CheckBox_onlyLeafNode(int)));
    connect(ui->checkBox_overlappingNode, SIGNAL(stateChanged(int)), this, SLOT(CheckBox_overlappingNode(int)));


    //AABB Depth Visualizer
    connect(ui->horizontalSlider_AABBTree, SIGNAL(valueChanged(int)), ui->lcdNumber_AABBTree, SLOT(display(int)));
    //connect(ui->horizontalSlider_2, SIGNAL(sliderReleased()), this, SLOT(switchIteration()));
    connect(ui->horizontalSlider_AABBTree, SIGNAL(valueChanged(int)), this, SLOT(ChangeAABBVisualizedDepth(int)));


    connect(ui->pushButton_outputBody, SIGNAL(released()), this, SLOT(OutputFiles()));
    connect(ui->pushButton_outputChamber, SIGNAL(released()), this, SLOT(OutputFiles()));
    connect(ui->pushButton_outputTet, SIGNAL(released()), this, SLOT(OutputFiles()));
    //

    
}

void MainWindow::OutputFiles(void)
{
    //
    auto signalSender = (QPushButton*)sender(); //sender() holds the instance of the sender
    qDebug("%s", signalSender->objectName().toStdString().c_str());
    //lineEdit_bodyOutputName
    //lineEdit_chamberOutputName
    //lineEdit_tetOutputName
    if (signalSender->objectName().toStdString().compare(string("pushButton_outputTet"))==0)
    {

        qDebug("output tet...");

        string name = ui->lineEdit_tetOutputName->text().toStdString();
        name = string("../output/") + name + ".tet";
        char buffer[50];
        strcpy(buffer, name.c_str());
        ChamberArray_tet[0]->outputTETFile(buffer);

    }

    if (signalSender->objectName().toStdString().compare(string("pushButton_outputChamber")) == 0)
    {
        qDebug("output chamber...");

        string name = ui->lineEdit_chamberOutputName->text().toStdString();
        name = string("../output/") + name + ".obj";
        char buffer[50];
        strcpy(buffer, name.c_str());
        ChamberArray_chamber[0]->outputOBJFile(buffer);


    }

    if (signalSender->objectName().toStdString().compare(string("pushButton_outputBody")) == 0)
    {
        qDebug("output body...");

        string name = ui->lineEdit_bodyOutputName->text().toStdString();
        name = string("../output/") + name + ".obj";
        char buffer[50];
        strcpy(buffer, name.c_str());
        ChamberArray_body[0]->outputOBJFile(buffer);

    }

}

void MainWindow::ChangeAABBVisualizedDepth(int sliderValue)
{
    int depthIdx = ui->horizontalSlider_AABBTree->value();

    _aabbVisualizationPoly->draw_depth_idx = depthIdx;
    pGLK->refresh(true);
}

void MainWindow::CheckBox_onlyLeafNode(int checkBoxState)
{
    if (checkBoxState == 0)
        _aabbVisualizationPoly->_drawAllNode = true;
    else
        _aabbVisualizationPoly->_drawAllNode = false;
    pGLK->refresh(true);
}

void MainWindow::CheckBox_overlappingNode(int checkBoxState)
{
    if (checkBoxState == 0)
        _aabbVisualizationPoly->_drawOverlapping = false;
    else
        _aabbVisualizationPoly->_drawOverlapping = true;

}


void MainWindow:: TrajGeneration(void) {

    this->InputFreeFormMembrane();
    this->InputMem();
    this->MoveBall();

    double meshGenTime = 0.0;
    double deformationTime = 0.0;
    double collisonCheckTime = 0.0;
    double collRespTime = 0.0;

    int iter = 13;
    Eigen::MatrixXd tipPosSet = Eigen::MatrixXd::Zero(iter, 3);

    for (int i = 0; i < iter; i++) {

        long simTime = clock();
        this->GenerateChamberTetMesh();
        meshGenTime += (double)(clock() - simTime) / (double)(CLOCKS_PER_SEC); simTime = clock();

        this->ChamberDeformation();
        deformationTime += (double)(clock() - simTime) / (double)(CLOCKS_PER_SEC); simTime = clock();

        // for(int j=0;j<3;j++) this->CollisionChecking();
        collRespTime += (double)(clock() - simTime) / (double)(CLOCKS_PER_SEC); simTime = clock();

        Eigen::Vector3d tipPos = Eigen::Vector3d::Zero();
        int nodeNum = 0;
        for (GLKPOSITION pos = ChamberArray_tet[0]->GetNodeList().GetHeadPosition(); pos != nullptr;)
        {
            QMeshNode* Node = (QMeshNode*)ChamberArray_tet[0]->GetNodeList().GetNext(pos);
            if (Node->isTerminalChecking == false) continue;

            Eigen::Vector3d nodePos;
            Node->GetCoord3D(nodePos);
            tipPos += nodePos; nodeNum++;
        }
        tipPos /= nodeNum;
        tipPosSet.row(i) = tipPos;
    }
    std::cout << meshGenTime << endl << deformationTime << endl << collRespTime <<endl;
    std::cout << tipPosSet <<std::endl;
    pGLK->refresh(true);

}


/*Input Soft Finger*/
void MainWindow::InputFreeFormMembrane(void)
{
    qDebug("Input chambers...");
    double yShift;
    meshOperation* meshOperator = new meshOperation;
    
   
    QMeshPatch* body = new QMeshPatch;
    QMeshPatch* chamber = new QMeshPatch;
    QMeshPatch* tet = new QMeshPatch;
    Eigen::Vector3d shifting;
    double xShift, zShift;

    //distance is 100.
    xShift = 0;
    zShift = 0;

    shifting[0] = xShift;
    shifting[1] = 0;
    shifting[2] = zShift;

    //twisting twistingHalf
    //                                                                          chamber index  shifting vec
    inputSoftRobotModelWithShifting(body,chamber,tet, meshOperator,"fingerNew", to_string(1), shifting);
        
    ChamberArray_body.push_back(body);
    ChamberArray_chamber.push_back(chamber);
    ChamberArray_tet.push_back(tet);
   
    pGLK->refresh(true);
    delete meshOperator;
   
}


//Generate Tet Mesh
void MainWindow::GenerateChamberTetMesh(void)
{
    static bool init = false;
    qDebug("Generate tet mesh...");
    
    /* -----------------------------------------
         generate/update TET mesh for simulation
    -------------------------------------------*/

    std::string tetgenCommand = "Ya2.0";
    if (!init)
    {
        
        for (int i = 0; i < MAN_CHAMBER_SIZE; i++)
        {
            
            meshOperation* meshOperator = new meshOperation;

            //generate TET Mesh
            //meshOperator->tetMeshGeneration_outerSkin_Chamber(ChamberArray_body[i], ChamberArray_chamber[i], ChamberArray_tet[i], tetgenCommand);
            //
            ///*Select the rigid node*/
            //meshOperator->materialSpaceSelection_rigid(ChamberArray_tet[i], "fingerNew");
           
            ///*Select the inflation tet*/
            //meshOperator->chamberSelection(ChamberArray_tet[i], ChamberArray_chamber[i], false);
           
            ///*Save the generated tet mesh*/
            //meshOperator->MannequinTetMeshOutput(ChamberArray_tet[i],i+1);

            ///*Save the calculated selection*/
            //meshOperator->MannequinTetMeshSelectionOutput(ChamberArray_tet[i], i + 1);

            ////read saved result
            /*Read saved tet mesh*/
            meshOperator->tetMeshReading( ChamberArray_tet[i], i+1);

            /*Select the rigid node Keep*/
            meshOperator->materialSpaceSelection_rigid(ChamberArray_tet[i], "fingerNew");

            /*Select the inflation tet*/
            meshOperator->readChamberSelection(ChamberArray_tet[i], i+1);


            delete meshOperator;
            
        }
        init = true;
    }
    else
    {
        //update TET mesh
        //memory leakage here

        

        for (int i = 0; i < MAN_CHAMBER_SIZE; i++)
        {
            meshOperation* meshOperator = new meshOperation;
            std::vector<Eigen::Vector3d> rigidNodePosSet;
            meshOperator->laplacianSmoothSurface(ChamberArray_chamber[i], ChamberArray_tet[i]);

            meshOperator->saveRigidRegion(ChamberArray_tet[i], rigidNodePosSet, true);
            //meshOperator->tetMeshGeneration_outerSkinProtection_ChamberRemesh(body_init, chamber_init, tetMesh, tetgenCommand);
            meshOperator->tetMeshGeneration_outerSkinProtection_ChamberRemesh_New(ChamberArray_body[i], ChamberArray_chamber[i], ChamberArray_tet[i], tetgenCommand);
            meshOperator->loadRigidRegion(ChamberArray_tet[i], rigidNodePosSet);
            delete meshOperator;
        }
       

    }

    for (int i = 0; i < MAN_CHAMBER_SIZE; i++)
    {
        meshOperation* meshOperator = new meshOperation;
        meshOperator->buildTopologyConnection(ChamberArray_tet[i], ChamberArray_body[i], ChamberArray_chamber[i]);
        meshOperator->selectShift_rigidRegion(ChamberArray_tet[i], ChamberArray_body[i]);
        std::cout << std::endl << " ---- build topology connection between tetMesh and chamber mesh..." << std::endl;
        delete meshOperator;
    }


   
   
}

//finger deformation
void MainWindow::ChamberDeformation(void)
{

    GenerateChamberTetMesh();               //generate tet mesh
    static int counter_pressed = 0;
    counter_pressed++;
    qDebug("Chamber deformation...");
    static bool init = false;
    static QDoubleSpinBox** boxPtr;
    

    if (!init)
    {
        boxPtr = new QDoubleSpinBox * [4];
        boxPtr[0] = ui->doubleSpinBox_Chamber1;
        boxPtr[1] = ui->doubleSpinBox_Chamber2;
        boxPtr[2] = ui->doubleSpinBox_Chamber3;
        boxPtr[3] = ui->doubleSpinBox_Chamber4;
    }
    /* -----------------------------------------
         Run Simulation
     -------------------------------------------*/
    long simTime = clock();
    int maxitertime = 1000;
    for (int i = 0; i < MAN_CHAMBER_SIZE; i++)
    {
        soroPneumaticKinematics* soroIKOperator = new soroPneumaticKinematics;
        soroIKOperator->getMesh(ChamberArray_tet[i], maxitertime);
        //soroIKOperator->expandRatio[0] = boxPtr[i]->value();

        double initialValue = boxPtr[i]->value();
       /* if (counter_pressed >= 2)
        {
            initialValue = (double)counter_pressed / (double)(counter_pressed - 1);
        }*/



        soroIKOperator->expandRatio[0] = initialValue;
        if (!init)
        {
            soroIKOperator->buildSimulationSystem(initShape[i], true);
            soroIKOperator->hyberSolverSingle(maxitertime, true);
        }
        else
        {
            soroIKOperator->buildSimulationSystem(initShape[i], false);
            soroIKOperator->hyberSolverSingle(maxitertime, true);
        }

        //in initShape[i], there will be X tet representing X
        //in initShape[i][j], there will be 3x4 mat
        qDebug("initial shape vector length is %d", initShape[i].size());
        qDebug("initial shape mat is %d %d", initShape[i][0].rows(), initShape[i][0].cols());
        //output initial shape
        qDebug("\n*****************************************\nBeing Pressed: %d\n*****************************************\n", counter_pressed);
        for (int k = 0; k < 2; k++)
        {
            qDebug("-----%d-----",k);
            qDebug("%3.3lf\t%3.3lf\t%3.3lf\t%3.3lf", initShape[i][k](0, 0), initShape[i][k](0, 1), initShape[i][k](0, 2), initShape[i][k](0, 3));
            qDebug("%3.3lf\t%3.3lf\t%3.3lf\t%3.3lf", initShape[i][k](1, 0), initShape[i][k](1, 1), initShape[i][k](1, 2), initShape[i][k](1, 3));
            qDebug("%3.3lf\t%3.3lf\t%3.3lf\t%3.3lf", initShape[i][k](2, 0), initShape[i][k](2, 1), initShape[i][k](2, 2), initShape[i][k](2, 3));
            qDebug("%3.3lf\t%3.3lf\t%3.3lf\t%3.3lf", initShape[i][k](3, 0), initShape[i][k](3, 1), initShape[i][k](3, 2), initShape[i][k](3, 3));
            qDebug("----------\n");
        }

        meshOperation* meshOperator = new meshOperation;
        //ChamberArray_tet[i], ChamberArray_body[i], ChamberArray_chamber[i]
        meshOperator->updateOBJMesh_chamber(ChamberArray_chamber[i]);
        meshOperator->updateOBJMesh_skin(ChamberArray_body[i]);
        meshOperator->outputSimulationResult(ChamberArray_tet[i], ChamberArray_body[i], ChamberArray_chamber[i], "twistingResult", counter_pressed, true, true);

        delete meshOperator;
        delete soroIKOperator;
    }

    //check body element number
    int bodyCount = 0;
    for (GLKPOSITION pos = ChamberArray_tet[0]->GetTetraList().GetHeadPosition(); pos != nullptr;)
    {
        QMeshTetra* Tetra = (QMeshTetra*)ChamberArray_tet[0]->GetTetraList().GetNext(pos);
        if (Tetra->isChamber[0] == false)
        {
            bodyCount++;
        }

    }
    qDebug("Simulation time: %3.3lf s\n", (double)(clock() - simTime) / (double)(CLOCKS_PER_SEC));
    qDebug("Body element number is %d", bodyCount);


    
    DeformTet* deformTetOperator = new DeformTet;
    
    delete deformTetOperator;
    init = true;
    pGLK->refresh(true);
}


//input obstacle
void MainWindow::InputMem(void)
{
    qDebug("Input outer membrane...");

    //std::string filePrefix = "..\\model\\";
    //std::string chamberFile = "../model/Cuboid9.tet";
    //std::string chamberFile = "../model/blockObstacle.tet";
    std::string chamberFile = "../model/tapeObstacle.tet";

    

    
    //std::string chamberFile = "../model/newTwistingobstacle.tet";
    QMeshPatch* memPatch = new QMeshPatch;
    memPatch->inputTETFile((char*)chamberFile.c_str(), false);

    /* read TET mesh for simulation */
    PolygenMesh* volumeMesh = this->_buildPolygenMesh("Obstacle_TET");
    volumeMesh->GetMeshList().AddTail(memPatch);
    volumeMesh->resetTransparent();
    volumeMesh->bVertexNormalShading = false;
    //volumeMesh->isVolume = true;

    volumeMesh->drawIdx = 3;
    Eigen::Vector3d nodePos, shift;

    //shift mesh

    /*shift[0] = -20;
    shift[1] = 30;
    shift[2] = 0;*/

   /* shift[0] = -70;
    shift[1] = 58;
    shift[2] = 0;

    for (GLKPOSITION pos = memPatch->GetNodeList().GetHeadPosition(); pos != nullptr;)
    {
        QMeshNode* Node = (QMeshNode*)memPatch->GetNodeList().GetNext(pos);
        Node->GetCoord3D(nodePos);
        nodePos = nodePos + shift;
        Node->SetCoord3D(nodePos);
    }

    for (GLKPOSITION pos = memPatch->GetNodeList().GetHeadPosition(); pos != nullptr;)
    {
        QMeshNode* Node = (QMeshNode*)memPatch->GetNodeList().GetNext(pos);
        Node->GetCoord3D(nodePos);

        nodePos = (nodePos * 70/50) / 1.5;

        Node->SetCoord3D(nodePos);
    }*/


    //now the ball is at the original





    //select fix region
    //calculate max and min for x and z axis
    double xMin, xMax, zMin, zMax;
    xMin = 10000;
    xMax = -10000;
    zMin = 10000;
    zMax = -10000;

    /* for (GLKPOSITION pos = memPatch->GetNodeList().GetHeadPosition(); pos != nullptr;)
     {
         QMeshNode* Node = (QMeshNode*)memPatch->GetNodeList().GetNext(pos);
         Node->GetCoord3D(nodePos);
         if (nodePos[0] < xMin)
             xMin = nodePos[0];
         if (nodePos[0] > xMax)
             xMax = nodePos[0];

         if (nodePos[2] < zMin)
             zMin = nodePos[2];
         if (nodePos[2] > zMax)
             zMax = nodePos[2];

     }*/

     /* for (GLKPOSITION pos = memPatch->GetNodeList().GetHeadPosition(); pos != nullptr;)
      {
          QMeshNode* Node = (QMeshNode*)memPatch->GetNodeList().GetNext(pos);
          Node->GetCoord3D(nodePos);
          if (fabs(nodePos[0] - xMin) < 0.003 || fabs(nodePos[0] - xMax) < 0.003 || fabs(nodePos[2] - zMin) < 0.003 || fabs(nodePos[2] - zMax) < 0.003)
          {
              static int count = 0;
              count++;

              Node->isFixed = true;
          }


      }*/

    _freeformMem = memPatch;

    pGLK->refresh(true);

}


//collision checking and response
//deformable object: ChamberArray_tet[0]
//environment obstacle: _freeformMem

PolygenMesh* testSelfCollidedPoly;
void MainWindow::CollisionChecking(void)
{
    
 //   //input self-collided example
 //   std::string filePrefixTET = "..\\output\\";		//..\\selectionFile\\CollisionExample\\
	//AABB\\1.tet  //iter_1.tet
 //   std::string  filePostfixTET = "SelfCollisionExample.tet"; //horse.tet tetrahedron_test.tet CollisionExample\\CollisionExample.tet CollisionExample\\finger_selfcoll.tet CUBE0.tet
 //   char* cstrTET = new char[(filePrefixTET + filePostfixTET).length() + 1];
 //   strcpy(cstrTET, (filePrefixTET + filePostfixTET).c_str());
 //   PolygenMesh* polygenMesh = new PolygenMesh();
 //   polygenMesh->drawIdx = 0;

 //   

 //   polygenMesh->ImportTETFile(cstrTET, filePostfixTET);
 //   qDebug("After input tet");
 //   //polygenMesh->setModelName("self_collided_example");
 //   polygenMeshList.AddTail(polygenMesh);
 //   polygenMesh->BuildGLList(polygenMesh->m_bVertexNormalShading);
 //   pGLK->AddDisplayObj(polygenMesh, true);
 //   QMeshPatch* _patch = (QMeshPatch*)polygenMesh->GetMeshList().GetHead();
 //   testSelfCollidedPoly = polygenMesh;
 //   updateTree();
 //   pGLK->refresh(true);
 //   


 //   printf("Finish input selection \n");
 //   pGLK->refresh(true);
 //   ChamberArray_tet.push_back(_patch);

 //   InputMem();
 //   MoveBall();
 //   
 //   // /*Normal Code*/
 //   qDebug("Collision Checking and Correspondence Calculation...");
 //   static bool init_aabb = false;

 //   DeformTet* deformTetOperator = new DeformTet;

 //   AABBManager* collisionOperator = new AABBManager[MAN_CHAMBER_SIZE];
 //   deformTetOperator->setLastCoordinate(ChamberArray_tet[0]);	//first set last coord.
 //   deformTetOperator->SetMesh(ChamberArray_tet[0]);
 //   
 //   for (int i = 0; i < MAN_CHAMBER_SIZE; i++)
 //   {
 //       //obstacle tree construction
 //       collisionOperator[i].GetVolumeObstacleMesh(_freeformMem);
 //       collisionOperator[i].ObstacleTreeConstructionTop2Bot_rebuild();

 //       //mark obstacle boundary face
 //       collisionOperator[i].MarkBoundaryFaceForTetMesh(_freeformMem);
 //   }

 //   for (int i = 0; i < MAN_CHAMBER_SIZE; i++)
 //   {
 //       //for each chamber tet, find the collision 
 //       collisionOperator[i].GetTetMesh(ChamberArray_tet[i]);

 //       //self collision tree rebuild
 //       collisionOperator[i].TreeConstructionTop2Bot_rebuild(ChamberArray_tet[i]);

 //   }



 //   //
 //   //collision response
 //   for (int collIter = 0; collIter < 2; collIter++) {

 //       //environmental collision checking and correspondence calculation
 //       //Collision checking with env and find the correspondence

 //                                                                //number of RoI points
 //       //below is used to debug
 //       //zero is not collision, while one is in collision
 //       Eigen::VectorXd collidedSumResult = Eigen::VectorXd::Zero(ChamberArray_tet[0]->GetNodeNumber());

 //       //update collision status for last and this iteration
 //       collisionOperator[0].UpdateEnvCollisionStatus(ChamberArray_tet[0]);

 //       // check collision result with env
 //       for (int i = 0; i < MAN_CHAMBER_SIZE; i++)
 //       {
 //           //each collision checking should return a vector indicating the result
 //           //collisionOperator[i].CollisionWithEnvQueryChecking();
 //           Eigen::VectorXd _collisionResult = collisionOperator[i].CollisionWithEnvQueryCheckingReturnResult(i + 1);
 //           qDebug("Size is %d and %d", collidedSumResult.size(), _collisionResult.size());
 //           collidedSumResult = collidedSumResult + _collisionResult;

 //       }

 //       //summarize collision situation 
 //       collisionOperator[0].SumUpCollisionResultWithEnv(ChamberArray_tet[0], collidedSumResult);
 //       qDebug("Checking collision with Env ends");
 //       //check self collision
 //       collisionOperator[0].SelfCollisionDetectionCorrespondenceChecking_softFinger();

 //       

 //       //deformTetOperator->RunWithCollisionResponse(initShape[0], 20);

 //   }



 //   /*Visualize self-collision checking correspondence*/
 //   static bool init_corre_selfCollision = false;
 //   if (!init_corre_selfCollision)
 //   {

 //       corrspondencePolySelfCollision_global = new PolygenMesh;
 //       corrspondencePolySelfCollision_global->setModelName("Correspondence-SelfCollision");
 //       corrspondencePolySelfCollision_global->BuildGLList(corrspondencePolySelfCollision_global->m_bVertexNormalShading);
 //       pGLK->AddDisplayObj(corrspondencePolySelfCollision_global, true);
 //       polygenMeshList.AddTail(corrspondencePolySelfCollision_global);
 //       updateTree();

 //       corrspondencePolySelfCollision_global->drawIdx = 11;
 //       corrspondencePolySelfCollision_global->GetMeshList().AddTail(collisionOperator[0]._corresPatch_selfCollision);

 //       init_corre_selfCollision = true;
 //   }
 //   else
 //   {
 //       corrspondencePolySelfCollision_global->ClearAll();
 //       corrspondencePolySelfCollision_global->GetMeshList().AddTail(collisionOperator[0]._corresPatch_selfCollision);
 //   }


 //   /*Visualize collision with env correspondence checking*/
 //   static bool init_corre = false;
 //   if (!init_corre)
 //   {
 //       //_corresPatch
 //       corrspondencePoly_global = new PolygenMesh;
 //       corrspondencePoly_global->setModelName("Correspondence-CollisionEnv");
 //       corrspondencePoly_global->BuildGLList(corrspondencePoly_global->m_bVertexNormalShading);
 //       pGLK->AddDisplayObj(corrspondencePoly_global, true);
 //       polygenMeshList.AddTail(corrspondencePoly_global);
 //       updateTree();
 //       corrspondencePoly_global->drawIdx = 11;
 //       corrspondencePoly_global->GetMeshList().AddTail(collisionOperator[0]._corresPatch);
 //       init_corre = true;
 //   }
 //   else
 //   {
 //       corrspondencePoly_global->ClearAll();
 //       corrspondencePoly_global->GetMeshList().AddTail(collisionOperator[0]._corresPatch);

 //   }


 //   //visualize bounding box
 //   static bool init_collidedBox= false;
 //   if (!init_collidedBox)
 //   {
 //       collidedBoxPolySelfCollision_global = new PolygenMesh;
 //       collidedBoxPolySelfCollision_global->setModelName("Collided BoundingBox");
 //       
 //       collidedBoxPolySelfCollision_global->GetMeshList().AddTail(collisionOperator[0]._debugPatch);
 //       polygenMeshList.AddTail(collidedBoxPolySelfCollision_global);
 //       collidedBoxPolySelfCollision_global->BuildGLList(collidedBoxPolySelfCollision_global->m_bVertexNormalShading);
 //       pGLK->AddDisplayObj(collidedBoxPolySelfCollision_global, true);
 //       collidedBoxPolySelfCollision_global->drawIdx = 4;




 //       collidedTetPolySelfCollision_global = new PolygenMesh;
 //       collidedTetPolySelfCollision_global->setModelName("Collided Tet");

 //       collidedTetPolySelfCollision_global->GetMeshList().AddTail(collisionOperator[0]._debugPatch_tetDraw);
 //       polygenMeshList.AddTail(collidedTetPolySelfCollision_global);
 //       collidedTetPolySelfCollision_global->BuildGLList(collidedTetPolySelfCollision_global->m_bVertexNormalShading);
 //       pGLK->AddDisplayObj(collidedTetPolySelfCollision_global, true);
 //       collidedTetPolySelfCollision_global->drawIdx = 4;





 //       updateTree();

 //       init_collidedBox = true;
 //   }
 //   else
 //   {

 //       collidedBoxPolySelfCollision_global->ClearAll();
 //       collidedBoxPolySelfCollision_global->GetMeshList().AddTail(collisionOperator[0]._debugPatch);

 //       collidedTetPolySelfCollision_global->ClearAll();
 //       collidedTetPolySelfCollision_global->GetMeshList().AddTail(collisionOperator[0]._debugPatch_tetDraw);

 //   }

 //   delete deformTetOperator;
 //   delete[] collisionOperator;
 //   pGLK->refresh(true);



    /*Normal Code*/
    
    qDebug("Collision Checking and Correspondence Calculation...");
    static bool init_aabb = false;
 
    DeformTet* deformTetOperator = new DeformTet;
   
    AABBManager* collisionOperator = new AABBManager[MAN_CHAMBER_SIZE];
    deformTetOperator->setLastCoordinate(ChamberArray_tet[0]);	//first set last coord.
    deformTetOperator->SetMesh(ChamberArray_tet[0]);
    deformTetOperator->SelectFixRegion(ChamberArray_tet[0]);
    for (int i = 0; i < MAN_CHAMBER_SIZE; i++)
    {
        //obstacle tree construction
        collisionOperator[i].GetVolumeObstacleMesh(_freeformMem);
        collisionOperator[i].ObstacleTreeConstructionTop2Bot_rebuild();	

        //mark obstacle boundary face
        collisionOperator[i].MarkBoundaryFaceForTetMesh(_freeformMem);
    }
    
    for (int i = 0; i < MAN_CHAMBER_SIZE; i++)
    {
        //for each chamber tet, find the collision 
        collisionOperator[i].GetTetMesh(ChamberArray_tet[i]);

        //self collision tree rebuild
        collisionOperator[i].TreeConstructionTop2Bot_rebuild(ChamberArray_tet[i]);

    }


    
    //
    //collision response
    for (int collIter = 0; collIter < 3; collIter++) {

        //environmental collision checking and correspondence calculation
        //Collision checking with env and find the correspondence

                                                                 //number of RoI points
        //below is used to debug
        //zero is not collision, while one is in collision
        Eigen::VectorXd collidedSumResult = Eigen::VectorXd::Zero(ChamberArray_tet[0]->GetNodeNumber());

        //update collision status for last and this iteration
        collisionOperator[0].UpdateEnvCollisionStatus(ChamberArray_tet[0]);

       // check collision result with env
        for (int i = 0; i < MAN_CHAMBER_SIZE; i++)
        {
            //each collision checking should return a vector indicating the result
            //collisionOperator[i].CollisionWithEnvQueryChecking();
            Eigen::VectorXd _collisionResult = collisionOperator[i].CollisionWithEnvQueryCheckingReturnResult(i+1);
            qDebug("Size is %d and %d", collidedSumResult.size(), _collisionResult.size());
            collidedSumResult = collidedSumResult + _collisionResult;
            
        }

        //summarize collision situation 
        collisionOperator[0].SumUpCollisionResultWithEnv(ChamberArray_tet[0], collidedSumResult);
        //check self collision
        collisionOperator[0].SelfCollisionDetectionCorrespondenceChecking_softFinger();

        qDebug("Checking collision with Env ends");
        
        deformTetOperator->RunWithCollisionResponse(initShape[0],100);
        
    }


    
   /*Visualize self-collision checking correspondence*/
    static bool init_corre_selfCollision = false;
    if (!init_corre_selfCollision)
    {
        
        corrspondencePolySelfCollision_global = new PolygenMesh;
        corrspondencePolySelfCollision_global->setModelName("Correspondence-SelfCollision");
        corrspondencePolySelfCollision_global->BuildGLList(corrspondencePolySelfCollision_global->m_bVertexNormalShading);
        pGLK->AddDisplayObj(corrspondencePolySelfCollision_global, true);
        polygenMeshList.AddTail(corrspondencePolySelfCollision_global);
        updateTree();

        corrspondencePolySelfCollision_global->drawIdx = 11;
        corrspondencePolySelfCollision_global->GetMeshList().AddTail(collisionOperator[0]._corresPatch_selfCollision);

        init_corre_selfCollision = true;
    }
    else
    {
        corrspondencePolySelfCollision_global->ClearAll();
        corrspondencePolySelfCollision_global->GetMeshList().AddTail(collisionOperator[0]._corresPatch_selfCollision);
    }

    
    /*Visualize collision with env correspondence checking*/
    static bool init_corre = false;
    if (!init_corre)
    {
        //_corresPatch
        corrspondencePoly_global = new PolygenMesh;
        corrspondencePoly_global->setModelName("Correspondence-CollisionEnv");
        corrspondencePoly_global->BuildGLList(corrspondencePoly_global->m_bVertexNormalShading);
        pGLK->AddDisplayObj(corrspondencePoly_global, true);
        polygenMeshList.AddTail(corrspondencePoly_global);
        updateTree();
        corrspondencePoly_global->drawIdx = 11;
        corrspondencePoly_global->GetMeshList().AddTail(collisionOperator[0]._corresPatch);
        init_corre = true;
    }
    else
    {
        corrspondencePoly_global->ClearAll();
        corrspondencePoly_global->GetMeshList().AddTail(collisionOperator[0]._corresPatch);

    }


    //visualize bounding box
    static bool init_collidedBox = false;
    if (!init_collidedBox)
    {
        collidedBoxPolySelfCollision_global = new PolygenMesh;
        collidedBoxPolySelfCollision_global->setModelName("Collided BoundingBox");
    
        collidedBoxPolySelfCollision_global->GetMeshList().AddTail(collisionOperator[0]._debugPatch);
        polygenMeshList.AddTail(collidedBoxPolySelfCollision_global);
        collidedBoxPolySelfCollision_global->BuildGLList(collidedBoxPolySelfCollision_global->m_bVertexNormalShading);
        pGLK->AddDisplayObj(collidedBoxPolySelfCollision_global, true);
        collidedBoxPolySelfCollision_global->drawIdx = 4;
    
        collidedTetPolySelfCollision_global = new PolygenMesh;
        collidedTetPolySelfCollision_global->setModelName("Collided Tet");
    
        collidedTetPolySelfCollision_global->GetMeshList().AddTail(collisionOperator[0]._debugPatch_tetDraw);
        polygenMeshList.AddTail(collidedTetPolySelfCollision_global);
        collidedTetPolySelfCollision_global->BuildGLList(collidedTetPolySelfCollision_global->m_bVertexNormalShading);
        pGLK->AddDisplayObj(collidedTetPolySelfCollision_global, true);
        collidedTetPolySelfCollision_global->drawIdx = 4;
    
   
        updateTree();
    
        init_collidedBox = true;
    }
    else
    {
    
        collidedBoxPolySelfCollision_global->ClearAll();
        collidedBoxPolySelfCollision_global->GetMeshList().AddTail(collisionOperator[0]._debugPatch);
    
        collidedTetPolySelfCollision_global->ClearAll();
        collidedTetPolySelfCollision_global->GetMeshList().AddTail(collisionOperator[0]._debugPatch_tetDraw);
    
    }

    delete deformTetOperator;
    delete[] collisionOperator;
  
    meshOperation* meshOperator = new meshOperation;
    meshOperator->updateOBJMesh_chamber(ChamberArray_chamber[0]);
    meshOperator->updateOBJMesh_skin(ChamberArray_body[0]);


    delete meshOperator;
    pGLK->refresh(true);
}


void MainWindow::CollisionResponse(void)
{
    qDebug("Collision Response Calculation...");


}

void MainWindow::MoveBall(void)
{
    double move[3];

    move[0] = ui->doubleSpinBox_ballMoveX->value();
    move[1] = ui->doubleSpinBox_ballMoveY->value();
    move[2] = ui->doubleSpinBox_ballMoveZ->value();
   
    double nodepos[3];
    for (GLKPOSITION pos = _freeformMem->GetNodeList().GetHeadPosition(); pos != nullptr;)
    {
        QMeshNode* Node = (QMeshNode*)_freeformMem->GetNodeList().GetNext(pos);
        Node->GetCoord3D(nodepos[0], nodepos[1], nodepos[2]);
        for (int i = 0; i < 3; i++)
        {
            nodepos[i] += (move[i]);
        }

        Node->SetCoord3D(nodepos[0], nodepos[1], nodepos[2]);
        Node->SetCoord3D_last(nodepos[0], nodepos[1], nodepos[2]);
    }

    pGLK->refresh(true);
    //
}


/*Update AABB Tree*/
void MainWindow::ShowAABBTree(void)
{
    
    static bool init_poly = false;
    if (init_poly == false)
    {
        _aabbVisualizationPoly = new PolygenMesh;
        _aabbVisualizationPoly->setModelName("AABB Tree");
        _aabbVisualizationPoly->drawIdx = 4;
        _aabbVisualizationPoly->_drawOverlapping = false;
        
        _aabbVisualizationMesh = new QMeshPatch;
        _aabbVisualizationPoly->GetMeshList().AddTail(_aabbVisualizationMesh);

        _aabbVisualizationPoly->BuildGLList(_aabbVisualizationPoly->m_bVertexNormalShading);
        pGLK->AddDisplayObj(_aabbVisualizationPoly, true);
        polygenMeshList.AddTail(_aabbVisualizationPoly);
        updateTree();

        init_poly = true;
    }

    if (ui->checkBox_showAABB->isChecked() == true)
    {
        qDebug("Generate AABB Tree....");
        AABBManager* _aabbManager = new AABBManager;
        //This operator will handle both the first run and normal run
        _aabbManager->TreeConstructionTop2Bot_rebuild(ChamberArray_tet[0]);
        _aabbManager->operator_updateAABBTree(_aabbVisualizationPoly);

        //set horizontal slider maximum
        qDebug("AABB Tree Depth is %d (From Zero)", _aabbManager->GetTreeDepth());
        ui->horizontalSlider_AABBTree->setMaximum(_aabbManager->GetTreeDepth());

        delete _aabbManager;
        qDebug("Bounding Box Set Visualizer Established...");
    }
    else
    {
        qDebug("AABB Init finished");
    }
  

    

    pGLK->refresh(true);

}
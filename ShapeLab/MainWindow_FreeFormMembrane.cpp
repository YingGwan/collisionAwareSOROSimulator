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
    connect(ui->pushButton_applyrotation, SIGNAL(released()), this, SLOT(ApplyRotation()));
    //
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
    inputSoftRobotModelWithShifting(body,chamber,tet, meshOperator,"twisting", to_string(1), shifting);
        
    ChamberArray_body.push_back(body);
    ChamberArray_chamber.push_back(chamber);
    ChamberArray_tet.push_back(tet);
   
    pGLK->refresh(true);
    delete meshOperator;
   
}


void MainWindow::GenerateChamberTetMesh(void)
{
    static bool init = false;
    qDebug("Generate tet mesh...");
    
    /* -----------------------------------------
         generate/update TET mesh for simulation
    -------------------------------------------*/

    std::string tetgenCommand = "Ya1.0";
    if (!init)
    {
        
        for (int i = 0; i < MAN_CHAMBER_SIZE; i++)
        {
            
            meshOperation* meshOperator = new meshOperation;

            //////generate TET Mesh
            //meshOperator->tetMeshGeneration_outerSkin_Chamber(ChamberArray_body[i], ChamberArray_chamber[i], ChamberArray_tet[i], tetgenCommand);
            //
            ///*Select the rigid node*/
            //meshOperator->materialSpaceSelection_rigid(ChamberArray_tet[i], "twisting");
           
            ///*Select the inflation tet*/
            //meshOperator->chamberSelection(ChamberArray_tet[i], ChamberArray_chamber[i], false);
           
            ///*Save the generated tet mesh*/
            //meshOperator->MannequinTetMeshOutput(ChamberArray_tet[i],i+1);

            ///*Save the calculated selection*/
            //meshOperator->MannequinTetMeshSelectionOutput(ChamberArray_tet[i], i + 1);

            ////read saved result
            ///*Read saved tet mesh*/
            meshOperator->tetMeshReading( ChamberArray_tet[i], i+1);

            /*Select the rigid node Keep*/
            meshOperator->materialSpaceSelection_rigid(ChamberArray_tet[i], "twisting");

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


    ui->pushButton_CollisionChecking->setEnabled(true);
   
}

void MainWindow::ChamberDeformation(void)
{

    GenerateChamberTetMesh();
    static int counter_pressed = 0;
    counter_pressed++;
    qDebug("Chamber deformation...");
    static bool init = false;
    static QDoubleSpinBox** boxPtr;
    static std::vector<Eigen::MatrixXd> initShape[MAN_CHAMBER_SIZE];

    if (!init)
    {
        boxPtr = new QDoubleSpinBox * [4];
        boxPtr[0] = ui->doubleSpinBox_Chamber1;
        //boxPtr[1] = ui->doubleSpinBox_Chamber2;
        //boxPtr[2] = ui->doubleSpinBox_Chamber3;
        //boxPtr[3] = ui->doubleSpinBox_Chamber4;
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
        qDebug("Expansion ratio is %lf",initialValue);
       /* if (counter_pressed >= 2)
        {
            initialValue = (double)counter_pressed / (double)(counter_pressed - 1);
        }*/



        soroIKOperator->expandRatio[0] = initialValue;
        if (!init)
        {
            soroIKOperator->buildSimulationSystem(initShape[i], true);
           /* soroIKOperator->hyberSolverSingle(maxitertime, true);*/

            soroIKOperator->hyberSolverSingle(maxitertime, false);
        }
        else
        {
           /* soroIKOperator->buildSimulationSystem(initShape[i], false);
            soroIKOperator->hyberSolverSingle(maxitertime, true);*/

            soroIKOperator->buildSimulationSystem(initShape[i], true);
            soroIKOperator->hyberSolverSingle(maxitertime, false);
        }

        Eigen::Vector3d centerPos = Eigen::Vector3d::Zero();
        for (GLKPOSITION Pos = ChamberArray_tet[i]->GetNodeList().GetHeadPosition(); Pos;) {
            QMeshNode* Node = (QMeshNode*)ChamberArray_tet[i]->GetNodeList().GetNext(Pos);
            Eigen::Vector3d pos;  Node->GetCoord3D(pos);
            centerPos += pos;
        }
        centerPos /= (double)ChamberArray_tet[i]->GetNodeNumber();
        if (centerPos[1] > 90.0) {
            for (GLKPOSITION Pos = ChamberArray_tet[i]->GetNodeList().GetHeadPosition(); Pos;) {
                QMeshNode* Node = (QMeshNode*)ChamberArray_tet[i]->GetNodeList().GetNext(Pos);
                Eigen::Vector3d pos;  Node->GetCoord3D(pos);
                Node->SetCoord3D(pos(0), -1.0 * (pos(1) - 90.0) + 90.0, pos(2));
            }
        }

        meshOperation* meshOperator = new meshOperation;
        //ChamberArray_tet[i], ChamberArray_body[i], ChamberArray_chamber[i]
        meshOperator->updateOBJMesh_chamber(ChamberArray_chamber[i]);
        meshOperator->updateOBJMesh_skin(ChamberArray_body[i]);
        meshOperator->outputSimulationResult(ChamberArray_tet[i], ChamberArray_body[i], ChamberArray_chamber[i], "twistingResult", counter_pressed, true, true);

        delete meshOperator;
        delete soroIKOperator;
    }
    printf("Simulation time: %3.3lf s\n", (double)(clock() - simTime) / (double)(CLOCKS_PER_SEC));
    DeformTet* deformTetOperator = new DeformTet;
    deformTetOperator->setLastCoordinate(ChamberArray_tet[0]);	//first set last coord.
    delete deformTetOperator;
    init = true;

    pGLK->refresh(true);
}


//obstacle
void MainWindow::InputMem(void)
{
    qDebug("Input outer membrane...");

    //std::string filePrefix = "..\\model\\";
    //std::string chamberFile = "../model/Cuboid9.tet";
    std::string chamberFile = "../model/twisting_half_obstacle.tet";

    
    //std::string chamberFile = "../model/newTwistingobstacle.tet";
    QMeshPatch* memPatch = new QMeshPatch;
    memPatch->inputTETFile((char*)chamberFile.c_str(), false);

    /* read TET mesh for simulation */
    PolygenMesh* volumeMesh = this->_buildPolygenMesh("Obstacle_TET");
    volumeMesh->GetMeshList().AddTail(memPatch);
    volumeMesh->resetTransparent();
    //volumeMesh->isVolume = true;

    volumeMesh->drawIdx = 3;
    Eigen::Vector3d nodePos, shift;


    _freeformMem = memPatch;

    pGLK->refresh(true);

}


//deformable object: ChamberArray_tet[0]
//environment obstacle: _freeformMem
void MainWindow::CollisionChecking(void)
{
    /*qDebug("Collision Checking and Correspondence Calculation...");*/

    DeformTet* deformTetOperator2 = new DeformTet;
    //deformTetOperator2->SelectFixRegion(ChamberArray_tet[0]);
    delete deformTetOperator2;

    static bool init_aabb = false;
    //qDebug("10");
    DeformTet* deformTetOperator = new DeformTet;
   
    AABBManager* collisionOperator = new AABBManager[MAN_CHAMBER_SIZE];

    deformTetOperator->SetMesh(ChamberArray_tet[0]);
  // deformTetOperator->SelectFixRegion(ChamberArray_tet[0]);

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


 
    qDebug("\n********************\nCollision response starts...\n********************\n");
    //
    //collision response
    for (int collIter = 0; collIter <4; collIter++) {

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

            //debug usage
            //PolygenMesh* poly = this->_buildPolygenMesh("EnvironCorre");
            //poly->GetMeshList().AddTail(collisionOperator[i]._corresPatch);

           // qDebug("Size is %d and %d", collidedSumResult.size(), _collisionResult.size());
            collidedSumResult = collidedSumResult + _collisionResult;
            
        }
        
        //summarize collision situation 
        collisionOperator[0].SumUpCollisionResultWithEnv(ChamberArray_tet[0], collidedSumResult);

        //check self collision
        //collisionOperator[0].SelfCollisionDetectionCorrespondenceChecking_softFinger();

       /* for (GLKPOSITION Pos = ChamberArray_tet[0]->GetNodeList().GetHeadPosition(); Pos != NULL; )
        {
            QMeshNode* node = (QMeshNode*)(ChamberArray_tet[0]->GetNodeList().GetNext(Pos));
            node->isCollided = false;
            node->isCollided_env = false;
        }
        ChamberArray_tet[0]->collidedPntNum = 0;
        ChamberArray_tet[0]->collidedPntNum_env = 0;*/

        //qDebug("Checking collision with Env ends");
        //ChamberArray_tet[0]->collidedPntNum = 0;

        //Collision Response
        /*if (collIter == 1)
            break;*/

        
        deformTetOperator->RunWithCollisionResponse(10);
        qDebug("One collision response round finished...\n");
        
    }
    qDebug("\n********************\nCollision response finished...\n********************\n\n\n");
 

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

void MainWindow::ApplyRotation(void)
{
    double move[3];

    move[0] = ui->doubleSpinBox_rotationAngleX->value();
    move[1] = ui->doubleSpinBox_rotationAngleY->value();
    move[2] = ui->doubleSpinBox_rotationAngleZ->value();

    double nodepos[3];
    Eigen::Matrix3d m;
    m = Eigen::AngleAxisd(move[0], Eigen::Vector3d::UnitX())
        * Eigen::AngleAxisd(move[1], Eigen::Vector3d::UnitY())
        * Eigen::AngleAxisd(move[2], Eigen::Vector3d::UnitZ());

    Eigen::Vector3d pos3d;
    Eigen::Vector3d posSum = {0.0,0.0,0.0};
    for (GLKPOSITION pos = _freeformMem->GetNodeList().GetHeadPosition(); pos != nullptr;)
    {
        QMeshNode* Node = (QMeshNode*)_freeformMem->GetNodeList().GetNext(pos);
        Node->GetCoord3D(pos3d);
        posSum += pos3d;
    }
    posSum /= (double)(_freeformMem->GetNodeNumber());

    for (GLKPOSITION pos = _freeformMem->GetNodeList().GetHeadPosition(); pos != nullptr;)
    {
        QMeshNode* Node = (QMeshNode*)_freeformMem->GetNodeList().GetNext(pos);
        Node->GetCoord3D(pos3d);
        pos3d = m * (pos3d - posSum) + posSum;
        Node->SetCoord3D(pos3d);
    }

    pGLK->refresh(true);
}
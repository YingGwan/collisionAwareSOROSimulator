#include "meshOperation.h"
#include "tetgen.h"
#include "../packages/PQPLib/PQP.h"

#include <iostream>
#include <fstream>      // std::ifstream
#include <cstring>
#include <string>

void meshOperation::seperateMesh(QMeshPatch* inputMesh, QMeshPatch* body, QMeshPatch* chamber) {

    /* flooding method to detect input mesh - setup flag for all the node */
    int partNum = this->_segementMesh_withFlag(inputMesh);
    std::cout << "the input mesh contains " << partNum << " parts." << std::endl;
    if (partNum != 2) { std::cout << "ERROR, contains more than two parts" << std::endl; return; }

    bool firstChamber = this->_detectSegmentationOrder(inputMesh);

    if (firstChamber) {
        this->_generateNewMeshPart(inputMesh, chamber, 0);
        this->_generateNewMeshPart(inputMesh, body, 1);
    }
    else {
        this->_generateNewMeshPart(inputMesh, chamber, 1);
        this->_generateNewMeshPart(inputMesh, body, 0);
    }
   
}

void meshOperation::tetMeshGeneration_singleSurfaceMesh(
    QMeshPatch* inputMesh, QMeshPatch* outputMesh, std::string tetgenCommand) {

    tetgenio in, out;
    tetgenio::facet* f;
    tetgenio::polygon* p;

    // All indices start from 0.
    //in.firstnumber = 1;

    /* fill node list - tetgen */
    in.numberofpoints = inputMesh->GetNodeNumber();
    in.pointlist = new REAL[in.numberofpoints * 3];

    int nodeIndex = 0;
    for (GLKPOSITION Pos = inputMesh->GetNodeList().GetHeadPosition(); Pos;) {
        QMeshNode* Node = (QMeshNode*)inputMesh->GetNodeList().GetNext(Pos);
        Node->SetIndexNo(nodeIndex);
        Node->GetCoord3D(in.pointlist[3 * nodeIndex], in.pointlist[3 * nodeIndex + 1], in.pointlist[3 * nodeIndex + 2]);
        nodeIndex++;
    }

    /* fill face list - tetgen */
    in.numberoffacets = inputMesh->GetFaceNumber();
    in.facetlist = new tetgenio::facet[in.numberoffacets];
    in.facetmarkerlist = new int[in.numberoffacets];

    int faceIndex = 0;
    for (GLKPOSITION Pos = inputMesh->GetFaceList().GetHeadPosition(); Pos;) {
        QMeshFace* thisFace = (QMeshFace*)inputMesh->GetFaceList().GetNext(Pos);

        f = &in.facetlist[faceIndex];
        f->numberofpolygons = 1;
        f->polygonlist = new tetgenio::polygon[f->numberofpolygons];
        f->numberofholes = 0;
        f->holelist = NULL;

        p = &f->polygonlist[0];
        p->numberofvertices = 3;
        p->vertexlist = new int[p->numberofvertices];
        p->vertexlist[0] = thisFace->GetNodeRecordPtr(0)->GetIndexNo();
        p->vertexlist[1] = thisFace->GetNodeRecordPtr(1)->GetIndexNo();
        p->vertexlist[2] = thisFace->GetNodeRecordPtr(2)->GetIndexNo();

        faceIndex++;
    }

    /* Output the PLC to files 'barin.node' and 'barin.poly'. */ 
    //in.save_nodes((char*)"../barin");
    //in.save_poly((char*)"../barin");

    // Tetrahedralize the PLC. Switches are chosen to read a PLC (p),
    //   do quality mesh generation (q) with a specified quality bound
    // 
    //   (1.414), and apply a maximum volume constraint (a0.1).

    //tetrahedralize((char*)"pq1.414a0.1", &in, &out);
    const char* inputCommand = tetgenCommand.c_str();

    tetrahedralize((char*)inputCommand, &in, &out);


    /* rebuild mesh from both node and element list */
    //patch->ClearAll();

    int nodeNum = out.numberofpoints;
    float* nodeTable = (float*)malloc(sizeof(float) * nodeNum * 3);
    for (int i = 0; i < nodeNum * 3; i++) nodeTable[i] = out.pointlist[i];

    int tetraNum = out.numberoftetrahedra;
    unsigned int* tetTable = (unsigned int*)malloc(sizeof(unsigned int) * tetraNum * 4);
    for (int i = 0; i < tetraNum * 4; i++) tetTable[i] = out.tetrahedronlist[i];

    outputMesh->constructionFromVerTetTable_volumeMesh(nodeNum, nodeTable, tetraNum, tetTable);

    /* below are the code used before...... */
    /*std::string path = "../model/newTet.tet";
    std::ofstream tetOutput(path);
    tetOutput << nodeNum << " vertices" << std::endl;
    tetOutput << tetraNum << " tets" << std::endl;
    for (int i = 0; i < nodeNum; i++) {
        tetOutput << nodeTable[i * 3] << " " << nodeTable[i * 3 + 1] << " " << nodeTable[i * 3 + 2] << std::endl;
    }
    for (int i = 0; i < tetraNum; i++) {
        tetOutput << "4 " << tetTable[i * 4] << " " << tetTable[i * 4 + 1] << " " << tetTable[i * 4 + 2] << " " << tetTable[i * 4 + 3] << std::endl;
    }
    tetOutput.close();
     
    delete nodeTable, tetTable;

    const char* myCharArr = path.c_str();
    outputMesh->inputTETFile((char*)myCharArr, true);*/

}

void meshOperation::tetMeshGeneration_outerSkin_Chamber(
    QMeshPatch* skin, QMeshPatch* chamber, QMeshPatch* outputMesh, std::string tetgenCommand) {

    /* combine skin and chamber to an entire mesh */
    QMeshPatch* combinedMesh = new QMeshPatch;

    //this->_combineTwoSurfaceMesh(skin, chamber, outputMesh);
    this->_combineTwoSurfaceMesh(skin, chamber, combinedMesh);
    this->tetMeshGeneration_singleSurfaceMesh(combinedMesh, outputMesh, tetgenCommand);


    combinedMesh->ClearAll(); delete combinedMesh;

}

/* This function reading existing tetrahedron mesh: chamber Index is from 1 to 4. */
void meshOperation::tetMeshReading(QMeshPatch* outputMesh, int chamberIdx)
{
    qDebug("Reading Pre-generated tet mesh of chamber %d",chamberIdx);
    
    std::string chamberFile = "../model/preGeneratedTet/Chamber"+ std::to_string(chamberIdx)+".tet";
   
    outputMesh->inputTETFile((char*)chamberFile.c_str(), false);


}

/* This function generate new tet mesh, which protect the skin region */
void meshOperation::tetMeshGeneration_outerSkinProtection_ChamberRemesh(
    QMeshPatch* skin, QMeshPatch* chamber, QMeshPatch* tetMesh, std::string tetgenCommand) {

    /* combine skin and chamber to an entire mesh */
    QMeshPatch* combinedMesh = new QMeshPatch;
    this->_combineTwoSurfaceMesh(skin, chamber, combinedMesh);

    QMeshPatch* newTetMesh = new QMeshPatch;
    this->tetMeshGeneration_singleSurfaceMesh(combinedMesh, newTetMesh, tetgenCommand);
    combinedMesh->ClearAll(); delete combinedMesh;

    /* detect newly generated mesh region */
    //this->chamberSelection(newTetMesh, chamber);
    this->chamberSelection_PQP(newTetMesh, chamber, false);

    //this->buildTopologyConnection_chamberSelection_FAST(newTetMesh, skin, chamber, false);

    /* build new tet mesh */
    this->_updateNodeState(tetMesh);
    this->_updateNodeState(newTetMesh);

    this->_generateTetMesh_keepBodyEle_ChangeChamber(tetMesh, newTetMesh);

    /* finish the chamber region selection on updated mesh */
    //this->buildTopologyConnection_chamberSelection_FAST(tetMesh, skin, chamber, true);

}

/* This function generate new tet mesh, which protect the skin region */
void meshOperation::tetMeshGeneration_outerSkinProtection_ChamberRemesh_New(
    QMeshPatch* skin, QMeshPatch* chamber, QMeshPatch* tetMesh, std::string tetgenCommand) {

    QMeshPatch* newTetMesh = new QMeshPatch;
    this->tetMeshGeneration_singleSurfaceMesh(chamber, newTetMesh, tetgenCommand);

    int bodyEleNum = 0;
    for (GLKPOSITION Pos = tetMesh->GetTetraList().GetHeadPosition(); Pos;) {
        QMeshTetra* tet = (QMeshTetra*)tetMesh->GetTetraList().GetNext(Pos);
        if (tet->isChamber[0] == false)bodyEleNum++;
    }

    for (GLKPOSITION Pos = newTetMesh->GetTetraList().GetHeadPosition(); Pos;) {
        QMeshTetra* Tetra = (QMeshTetra*)newTetMesh->GetTetraList().GetNext(Pos);
        Tetra->isChamber[0] = true;
    }
    for (GLKPOSITION Pos = newTetMesh->GetFaceList().GetHeadPosition(); Pos;) {
        QMeshFace* thisFace = (QMeshFace*)newTetMesh->GetFaceList().GetNext(Pos);
        if (thisFace->GetLeftTetra() == NULL || thisFace->GetRightTetra() == NULL)
            thisFace->isChamberBoundary = true;
    }

    /* build new tet mesh */
    this->_updateNodeState(tetMesh);
    this->_updateNodeState(newTetMesh);

    this->_generateTetMesh_keepBodyEle_ChangeChamber(tetMesh, newTetMesh);

    /* update chamber selection */
    int index = 0;
    for (GLKPOSITION Pos = tetMesh->GetTetraList().GetHeadPosition(); Pos;) {
        QMeshTetra* Tetra = (QMeshTetra*)tetMesh->GetTetraList().GetNext(Pos);
        if (index < bodyEleNum) index++;
        else {Tetra->isChamber[0] = true; index++;}
    }
    
    newTetMesh->ClearAll(); delete newTetMesh;
    /* finish the chamber region selection on updated mesh */
    //this->buildTopologyConnection_chamberSelection_FAST(tetMesh, skin, chamber, true);

     /* also find out the chamber face - for simulation usage */
    for (GLKPOSITION Pos = tetMesh->GetFaceList().GetHeadPosition(); Pos;) {
        QMeshFace* thisFace = (QMeshFace*)tetMesh->GetFaceList().GetNext(Pos);
        int chamberTetNum = 0;
        if (thisFace->GetLeftTetra() != NULL && thisFace->GetRightTetra() != NULL) {
            if (thisFace->GetLeftTetra()->isChamber[0])     chamberTetNum++;
            if (thisFace->GetRightTetra()->isChamber[0])    chamberTetNum++;
        }

        if (chamberTetNum == 1) {
            thisFace->isChamberBoundary = true;
            for (int i = 0; i < 3; i++) {
                thisFace->GetNodeRecordPtr(i)->isChamberNode = true;
            }
        }
    }

    qDebug("body tet is %d", bodyEleNum);

}

void meshOperation::MannequinTetMeshSelectionOutput(QMeshPatch* mesh, int chamberIdx)
{
    FILE* fp;
    GLKPOSITION Pos;
    QMeshNode* node;
    QMeshTetra* tet;
    double xx, yy, zz;
    int i, num, index;
    std::string filename = "../model/preGeneratedTet/Chamber" + std::to_string(chamberIdx) + ".txt";

    fp = fopen(filename.c_str(), "w");
    if (!fp)
    {
        printf("===============================================\n");
        printf("Can not open the data file - TXT File Export!\n");
        printf("===============================================\n");
        return;
    }


    for (Pos = mesh->GetTetraList().GetHeadPosition(); Pos != NULL;) 
    {
        tet = (QMeshTetra*)(mesh->GetTetraList().GetNext(Pos));
        //num = face->GetEdgeNum();
        if(tet->isChamber[0])
            fprintf(fp, "%d\n", 1);
        else
            fprintf(fp, "%d\n", 0);
    }

    fclose(fp);

}

void meshOperation::MannequinTetMeshOutput(QMeshPatch* mesh, int chamberIdx)
{
    FILE* fp;
    GLKPOSITION Pos;
    QMeshNode* node;
    QMeshTetra* tet;
    double xx, yy, zz;
    int i, num, index;
    std::string filename = "../model/preGeneratedTet/Chamber" + std::to_string(chamberIdx) + ".tet";
       
    fp = fopen(filename.c_str(), "w");
    if (!fp)
    {

        printf("===============================================\n");
        printf("Can not open the data file - TET File Export!\n");
        printf("===============================================\n");
        return;
        /*std::ofstream creatFileStream;
        creatFileStream.open(filename.c_str());
        creatFileStream << " ";
        creatFileStream.close();
        Sleep(50);

        fp = fopen(filename.c_str(), "w");
        if (!fp)
        {
            printf("===============================================\n");
            printf("Still can not open - TET File Export!\n");
            printf("===============================================\n");

            return;
        }
        */
    }

    //fprintf(fp, "# The units used in this file are meters.\n");
    fprintf(fp, "%d vertices\n%d tets\n", mesh->GetNodeNumber(),mesh->GetTetraNumber());
    i = 0;
    for (Pos = mesh->GetNodeList().GetHeadPosition(); Pos != NULL; i++) {
        node = (QMeshNode*)(mesh->GetNodeList().GetNext(Pos));
        node->GetCoord3D(xx, yy, zz);
        node->SetIndexNo(i);
        //		fprintf(fp,"v %.5f %.5f %.5f\n",(float)yy,(float)zz,(float)xx);
        fprintf(fp, "%.8lf %.8lf %.8lf\n", xx, yy, zz);
        //		fprintf(fp,"v %.12f %.12f %.12f\n",(float)zz,(float)xx,(float)yy);
    }

    //fprintf(fp, "\n");

    for (Pos = mesh->GetTetraList().GetHeadPosition(); Pos != NULL;) {
        tet = (QMeshTetra*)(mesh->GetTetraList().GetNext(Pos));
        //num = face->GetEdgeNum();
       
        fprintf(fp, "4 ");

        index = tet->GetNodeRecordPtr(1)->GetIndexNo();
        fprintf(fp, "%d ", index);
        index = tet->GetNodeRecordPtr(2)->GetIndexNo();
        fprintf(fp, "%d ", index);
        index = tet->GetNodeRecordPtr(3)->GetIndexNo();
        fprintf(fp, "%d ", index);
        index = tet->GetNodeRecordPtr(4)->GetIndexNo();
        fprintf(fp, "%d ", index);

        fprintf(fp, "\n");

   
    }

    fclose(fp);
}

void meshOperation::_generateTetMesh_keepBodyEle_ChangeChamber(QMeshPatch* tetMesh, QMeshPatch* newTetMesh) {
    
    int nodeNum = 0;
    int tetraNum = 0;

    // compute nodeNum
    for (GLKPOSITION Pos = tetMesh->GetNodeList().GetHeadPosition(); Pos;) {
        QMeshNode* Node = (QMeshNode*)tetMesh->GetNodeList().GetNext(Pos);
        if (Node->nodeState == 1 || Node->nodeState == 2) {
            Node->nodeStateIndex = nodeNum;
            nodeNum++;
        }
    }
    for (GLKPOSITION Pos = newTetMesh->GetNodeList().GetHeadPosition(); Pos;) {
        QMeshNode* Node = (QMeshNode*)newTetMesh->GetNodeList().GetNext(Pos);
        if (Node->nodeState == 0) {
            Node->nodeStateIndex = nodeNum;
            nodeNum++;
        }
    }
    /* for newTetMesh, special process need to be made !! */
    std::vector<Eigen::Vector3d> surfacePos;
    for (GLKPOSITION Pos = tetMesh->GetNodeList().GetHeadPosition(); Pos;) {
        QMeshNode* Node = (QMeshNode*)tetMesh->GetNodeList().GetNext(Pos);
        Eigen::Vector3d pp;
        if (Node->nodeState == 1) { Node->GetCoord3D(pp); surfacePos.push_back(pp); }
    }

    int chamberIndex = 0;
    Eigen::VectorXi chamberNodeIndex = Eigen::VectorXi::Zero(surfacePos.size());
    for (GLKPOSITION Pos = tetMesh->GetNodeList().GetHeadPosition(); Pos;) {
        QMeshNode* Node = (QMeshNode*)tetMesh->GetNodeList().GetNext(Pos);
        if (Node->nodeState == 1) {
            chamberNodeIndex(chamberIndex) = Node->nodeStateIndex; chamberIndex++;
        }
    }
    //std::cout << chamberNodeIndex << std::endl;
    int newTetSurfaceNodeNum = 0;
    for (GLKPOSITION Pos = newTetMesh->GetNodeList().GetHeadPosition(); Pos;) {
        QMeshNode* Node = (QMeshNode*)newTetMesh->GetNodeList().GetNext(Pos);
        if (Node->nodeState == 1) {
            //Node->nodeStateIndex = chamberNodeIndex(newTetSurfaceNodeNum);
            newTetSurfaceNodeNum++;
            Eigen::Vector3d pp; Node->GetCoord3D(pp);
            for (int i = 0; i < surfacePos.size(); i++) {
                //double dis = (pp - surfacePos[i]).norm(); std::cout << dis << std::endl;
                if ((pp - surfacePos[i]).norm() < 0.01) {
                    Node->nodeStateIndex = chamberNodeIndex(i);
                    //std::cout << Node->nodeStateIndex << "," << i << std::endl;
                    break;
                }
            }
            if (Node->nodeStateIndex < 0) std::cout << "corresponding node not found!" << std::endl;
        }
    }
    //std::cout << newTetSurfaceNodeNum << "," << surfacePos.size() << std::endl;

    // compute tetNum
    for (GLKPOSITION Pos = tetMesh->GetTetraList().GetHeadPosition(); Pos;) {
        QMeshTetra* Tetra = (QMeshTetra*)tetMesh->GetTetraList().GetNext(Pos);
        if (Tetra->isChamber[0] == false) tetraNum++;
    }
    for (GLKPOSITION Pos = newTetMesh->GetTetraList().GetHeadPosition(); Pos;) {
        QMeshTetra* Tetra = (QMeshTetra*)newTetMesh->GetTetraList().GetNext(Pos);
        if (Tetra->isChamber[0] == true) tetraNum++;
    }

    /* build list */

    float* nodeTable = (float*)malloc(sizeof(float) * nodeNum * 3);
    unsigned int* tetTable = (unsigned int*)malloc(sizeof(unsigned int) * tetraNum * 4);

    /* build node list - skin and chamber mesh should not have intersection !!! */
    int nodeIndex = 0;  double pp[3];
    for (GLKPOSITION Pos = tetMesh->GetNodeList().GetHeadPosition(); Pos;) {
        QMeshNode* Node = (QMeshNode*)tetMesh->GetNodeList().GetNext(Pos);
        if (Node->nodeStateIndex < 0) continue;
        else {
            Node->GetCoord3D(pp[0], pp[1], pp[2]);
            for (int i = 0; i < 3; i++) nodeTable[3 * nodeIndex + i] = pp[i];
            nodeIndex++;
        }
    }
    for (GLKPOSITION Pos = newTetMesh->GetNodeList().GetHeadPosition(); Pos;) {
        QMeshNode* Node = (QMeshNode*)newTetMesh->GetNodeList().GetNext(Pos);
        if (Node->nodeState == 0) {
            Node->GetCoord3D(pp[0], pp[1], pp[2]);
            for (int i = 0; i < 3; i++) nodeTable[3 * nodeIndex + i] = pp[i];
            nodeIndex++;
        }
    }

    /* build tetra list */
    int tetIndex = 0;
    for (GLKPOSITION Pos = tetMesh->GetTetraList().GetHeadPosition(); Pos;) {
        QMeshTetra* Tetra = (QMeshTetra*)tetMesh->GetTetraList().GetNext(Pos);
        if (Tetra->isChamber[0] == false) {
            for (int i = 0; i < 4; i++)
                tetTable[4 * tetIndex + i] = Tetra->GetNodeRecordPtr(i + 1)->nodeStateIndex;
            tetIndex++;
        }
    }

    for (GLKPOSITION Pos = newTetMesh->GetTetraList().GetHeadPosition(); Pos;) {
        QMeshTetra* Tetra = (QMeshTetra*)newTetMesh->GetTetraList().GetNext(Pos);
        if (Tetra->isChamber[0]) {
            for (int i = 0; i < 4; i++)
                tetTable[4 * tetIndex + i] = Tetra->GetNodeRecordPtr(i + 1)->nodeStateIndex;
            tetIndex++;
        }
    }

    /*for (int i = 0; i < tetraNum; i++) {
        std::cout << tetTable[4 * i] << "," << tetTable[4 * i + 1] << "," << tetTable[4 * i + 2]
            << "," << tetTable[4 * i + 3] << "," << std::endl;
    }*/

    tetMesh->ClearAll();
    tetMesh->constructionFromVerTetTable_volumeMesh(nodeNum, nodeTable, tetraNum, tetTable);

    delete nodeTable; delete tetTable;
}

void meshOperation::_updateNodeState(QMeshPatch* tetMesh) {
    for (GLKPOSITION Pos = tetMesh->GetTetraList().GetHeadPosition(); Pos;) {
        QMeshTetra* Tetra = (QMeshTetra*)tetMesh->GetTetraList().GetNext(Pos);
        if (Tetra->isChamber[0] == false) {
            for (int i = 0; i < 4; i++) Tetra->GetNodeRecordPtr(i+1)->nodeState = 2;
        }
        else {
            for (int i = 0; i < 4; i++) Tetra->GetNodeRecordPtr(i+1)->nodeState = 0;
        }
    }
    for (GLKPOSITION Pos = tetMesh->GetFaceList().GetHeadPosition(); Pos;) {
        QMeshFace* thisFace = (QMeshFace*)tetMesh->GetFaceList().GetNext(Pos);
        if (thisFace->isChamberBoundary) {
            for (int i = 0; i < 3; i++) thisFace->GetNodeRecordPtr(i)->nodeState = 1;
        }
    }
}

// build topology connection and detect chamber region
void meshOperation::buildTopologyConnection_chamberSelection_FAST(
    QMeshPatch* tetMesh, QMeshPatch* body, QMeshPatch* chamber, bool updateConnection) {

    long time = clock();

    std::vector<QMeshNode*> bodyNodeSet(body->GetNodeNumber());
    int index = 0;
    for (GLKPOSITION Pos = body->GetNodeList().GetHeadPosition(); Pos;) {
        QMeshNode* Node = (QMeshNode*)body->GetNodeList().GetNext(Pos);
        bodyNodeSet[index] = Node; index++;
    }
    std::vector<QMeshNode*> chamberNodeSet(chamber->GetNodeNumber());
    index = 0;
    for (GLKPOSITION Pos = chamber->GetNodeList().GetHeadPosition(); Pos;) {
        QMeshNode* Node = (QMeshNode*)chamber->GetNodeList().GetNext(Pos);
        chamberNodeSet[index] = Node; index++;
    }
    Eigen::Vector3d ppChamber; chamberNodeSet[0]->GetCoord3D(ppChamber);

    /* assume all the topology are right in order!!! */
    index = 0; int chamberSIndex = 0;
    for (GLKPOSITION Pos = tetMesh->GetNodeList().GetHeadPosition(); Pos;) {
        QMeshNode* Node = (QMeshNode*)tetMesh->GetNodeList().GetNext(Pos);
        if (index < body->GetNodeNumber())
            bodyNodeSet[index]->connectTETNode = Node;
        else {
            Eigen::Vector3d pp; Node->GetCoord3D(pp);
            if ((pp - ppChamber).norm() < 0.001) { chamberSIndex = index; break; }
        }
        index++;
    }

    index = 0;
    for (GLKPOSITION Pos = tetMesh->GetNodeList().GetHeadPosition(); Pos;) {
        QMeshNode* Node = (QMeshNode*)tetMesh->GetNodeList().GetNext(Pos);
        if (index < chamberSIndex) {
            //Node->nodeState = 2; 
        }
        else if (index < chamberSIndex + chamber->GetNodeNumber()) {
            chamberNodeSet[index - chamberSIndex]->connectTETNode = Node;
            //Node->nodeState = 1;        
        }
        //else Node->nodeState = 2;
        index++;
    }

    /* verify the topology building */
    for (int i = 0; i < body->GetNodeNumber(); i++) {
        Eigen::Vector3d pp, pp1;
        bodyNodeSet[i]->GetCoord3D(pp); bodyNodeSet[i]->connectTETNode->GetCoord3D(pp1);
        if ((pp - pp1).norm() > 0.001) std::cout << (pp - pp1).norm() << ", body TOPLOGY ERROR!!!" << std::endl;
    }
    for (int i = 0; i < chamber->GetNodeNumber(); i++) {
        Eigen::Vector3d pp, pp1;
        chamberNodeSet[i]->GetCoord3D(pp); chamberNodeSet[i]->connectTETNode->GetCoord3D(pp1);
        if ((pp - pp1).norm() > 0.001) std::cout << "chamber TOPLOGY ERROR!!!" << std::endl;
    }

    this->chamberSelection(tetMesh, chamber);

//    /* generate chamber selection */
//    index = 0;
//    std::vector<QMeshNode*> tetNodeSet(tetMesh->GetNodeNumber());
//    for (GLKPOSITION Pos = tetMesh->GetNodeList().GetHeadPosition(); Pos;) {
//        QMeshNode* Node = (QMeshNode*)tetMesh->GetNodeList().GetNext(Pos);
//        tetNodeSet[index] = Node; index++;
//    }
//
//
//#pragma omp parallel   
//    {
//#pragma omp for 
//        for (int i = 0; i < tetMesh->GetNodeNumber(); i++) {
//            if (tetNodeSet[i]->nodeState == 1) continue;
//            else {
//                Eigen::Vector3d centerPos;
//                tetNodeSet[i]->GetCoord3D(centerPos(0), centerPos(1), centerPos(2));
//                bool insideChamber = this->_calculatePointInsideMesh(chamber, centerPos);
//                if (insideChamber) tetNodeSet[i]->nodeState = 0;
//            }
//        }
//    }
//
//    for (GLKPOSITION Pos = tetMesh->GetTetraList().GetHeadPosition(); Pos;) {
//        QMeshTetra* thisTetra = (QMeshTetra*)tetMesh->GetTetraList().GetNext(Pos);
//        for (int i = 0; i < 4; i++) {
//            if (thisTetra->GetNodeRecordPtr(i + 1)->nodeState == 0) {
//                thisTetra->isChamber[0] = true; break;
//            }
//        }
//        if(thisTetra->GetNodeRecordPtr(1)->nodeState == 1 && thisTetra->GetNodeRecordPtr(2)->nodeState == 1 &&
//            thisTetra->GetNodeRecordPtr(3)->nodeState == 1 && thisTetra->GetNodeRecordPtr(4)->nodeState == 1)
//            thisTetra->isChamber[0] = true;
//    }
//    for (GLKPOSITION Pos = tetMesh->GetFaceList().GetHeadPosition(); Pos;) {
//        QMeshFace* thisFace = (QMeshFace*)tetMesh->GetFaceList().GetNext(Pos);
//        int chamberTetNum = 0;
//        if (thisFace->GetLeftTetra() != NULL && thisFace->GetRightTetra() != NULL) {
//            if (thisFace->GetLeftTetra()->isChamber[0])     chamberTetNum++;
//            if (thisFace->GetRightTetra()->isChamber[0])    chamberTetNum++;
//        }
//        if (chamberTetNum == 1) thisFace->isChamberBoundary = true;
//    }
    
}

void meshOperation::chamberSelection_PQP(QMeshPatch* tetMesh, QMeshPatch* chamber, bool checkMode) {
    long time = clock();

    PQP_Model* pqpModel = new PQP_Model();
    pqpModel->BeginModel();  int index = 0;
    PQP_REAL p1[3], p2[3], p3[3];

    for (GLKPOSITION Pos = chamber->GetFaceList().GetHeadPosition(); Pos;) {
        QMeshFace* Face = (QMeshFace*)chamber->GetFaceList().GetNext(Pos);

        Face->GetNodeRecordPtr(0)->GetCoord3D(p1[0], p1[1], p1[2]);
        Face->GetNodeRecordPtr(1)->GetCoord3D(p2[0], p2[1], p2[2]);
        Face->GetNodeRecordPtr(2)->GetCoord3D(p3[0], p3[1], p3[2]);

        pqpModel->AddTri(p1, p2, p3, index);
        index++;

    }
    pqpModel->EndModel();

    double distance = 999999.99;

    /* build tet list by tetMesh */
    std::vector<QMeshTetra*> tetraSet_materialSpace(tetMesh->GetTetraNumber());
    int tetIndex = 0;
    for (GLKPOSITION Pos = tetMesh->GetTetraList().GetHeadPosition(); Pos;) {
        QMeshTetra* thisTetra = (QMeshTetra*)tetMesh->GetTetraList().GetNext(Pos);
        tetraSet_materialSpace[tetIndex] = thisTetra; tetIndex++;
    }

#pragma omp parallel   
    {
#pragma omp for 
        for (int i = 0; i < tetraSet_materialSpace.size(); i++) {
            QMeshTetra* each_Tetra = tetraSet_materialSpace[i];

            Eigen::Vector3d centerPos;
            each_Tetra->CalCenterPos(centerPos(0), centerPos(1), centerPos(2));
            PQP_DistanceResult dres;	dres.last_tri = pqpModel->last_tri;
            PQP_REAL p[3];
            for (int i = 0; i < 3; i++) p[i] = centerPos(i);

            PQP_Distance(&dres, pqpModel, p, 0.0, 0.0);

            float closestPt[3];	// closest point
            closestPt[0] = dres.p1[0];	closestPt[1] = dres.p1[1]; closestPt[2] = dres.p1[2];

            int minTriId = dres.last_tri->id;	//	closest triangle - note that ID index starts from zero

            QMeshFace* cloestFace;
            int faceIndex = 0;
            for (GLKPOSITION Pos = chamber->GetFaceList().GetHeadPosition(); Pos;) {
                QMeshFace* Face = (QMeshFace*)chamber->GetFaceList().GetNext(Pos);
                if (faceIndex == minTriId) {
                    cloestFace = Face;
                    break;
                }
                faceIndex++;
            }

            Eigen::Vector3d faceNormal; double d;
            cloestFace->CalPlaneEquation();
            cloestFace->GetPlaneEquation(faceNormal[0], faceNormal[1], faceNormal[2], d);

            Eigen::Vector3d cloestPos;
            for (int i = 0; i < 3; i++) cloestPos[i] = closestPt[i];

            if (checkMode) {
                if ((cloestPos - centerPos).dot(faceNormal) < 0.0001) {
                    if (each_Tetra->isChamber[0] == true) {}//std::cout << "good selection by PQP" << std::endl;
                    else std::cout << "error selection for chamber region!" << std::endl;
                }
                else {
                    if (each_Tetra->isChamber[0] == false) {}//std::cout << "good selection by PQP" << std::endl;
                    else std::cout << "error selection for body region!" << std::endl;
                }
            }
            else {
                if ((cloestPos - centerPos).dot(faceNormal) > 0.0001)
                    each_Tetra->isChamber[0] = true;
            }
            
        }
    }


    /* also find out the chamber face - for simulation usage */
    for (GLKPOSITION Pos = tetMesh->GetFaceList().GetHeadPosition(); Pos;) {
        QMeshFace* thisFace = (QMeshFace*)tetMesh->GetFaceList().GetNext(Pos);
        int chamberTetNum = 0;
        if (thisFace->GetLeftTetra() != NULL && thisFace->GetRightTetra() != NULL) {
            if (thisFace->GetLeftTetra()->isChamber[0])     chamberTetNum++;
            if (thisFace->GetRightTetra()->isChamber[0])    chamberTetNum++;
        }

        if (chamberTetNum == 1) {
            thisFace->isChamberBoundary = true;
            for (int i = 0; i < 3; i++) {
                thisFace->GetNodeRecordPtr(i)->isChamberNode = true;
            }
        }
    }

    printf(" \n TIME - detect chamber region takes %ld ms.\n", clock() - time);
    delete pqpModel;
}

void meshOperation::chamberSelection(QMeshPatch* tetMesh, QMeshPatch* chamber, bool readFile) {

    long time = clock();

    /* build tet list by tetMesh */
    std::vector<QMeshTetra*> tetraSet_materialSpace(tetMesh->GetTetraNumber());
    int tetIndex = 0;
    for (GLKPOSITION Pos = tetMesh->GetTetraList().GetHeadPosition(); Pos;) {
        QMeshTetra* thisTetra = (QMeshTetra*)tetMesh->GetTetraList().GetNext(Pos);
        tetraSet_materialSpace[tetIndex] = thisTetra; tetIndex++;
    }

    //detect the center of tet element of materialSpace in/out the convexHull surface/ model surface
    qDebug("Chamber selection: tet size %d...", tetraSet_materialSpace.size());
    long time1 = clock();

    
#pragma omp parallel   
        {
#pragma omp for 
            for (int i = 0; i < tetraSet_materialSpace.size(); i++) {
                QMeshTetra* each_Tetra = tetraSet_materialSpace[i];

                Eigen::Vector3d centerPos;
                each_Tetra->CalCenterPos(centerPos(0), centerPos(1), centerPos(2));
                each_Tetra->isChamber[0] = this->_calculatePointInsideMesh(chamber, centerPos);
                //std::cout << i << "-th element " << each_Tetra->chamberElement << std::endl;
               /* if (i % 100 == 0)
                {
                    qDebug("Finish %d", i);
                }*/
            }
        }

   
    printf(" \n TIME - isChamber takes %ld ms.\n", clock() - time1);
    qDebug("Chamber ends...");
    /* also find out the chamber face - for simulation usage */
    for (GLKPOSITION Pos = tetMesh->GetFaceList().GetHeadPosition(); Pos;) {
        QMeshFace* thisFace = (QMeshFace*)tetMesh->GetFaceList().GetNext(Pos);
        int chamberTetNum = 0;
        if (thisFace->GetLeftTetra() != NULL && thisFace->GetRightTetra() != NULL) {
            if (thisFace->GetLeftTetra()->isChamber[0])     chamberTetNum++;
            if (thisFace->GetRightTetra()->isChamber[0])    chamberTetNum++;
        }
    
        if (chamberTetNum == 1) {
            thisFace->isChamberBoundary = true;
            for (int i = 0; i < 3; i++) {
                thisFace->GetNodeRecordPtr(i)->isChamberNode = true;
            }
        }
    }

    printf(" \n TIME - detect chamber region takes %ld ms.\n", clock() - time);

}

void meshOperation::readChamberSelection(QMeshPatch* tetMesh, int chamberIdx)
{
    FILE* fp;
    char linebuf[256], buf[100];
    int nodeNum, i;	float xx, yy, zz;
    int tetraNum;
    auto time = clock();
    std::string filename = "../model/preGeneratedTet/Chamber" + std::to_string(chamberIdx) + ".txt";

    fp = fopen(filename.c_str(), "r");
    if (!fp) {
        printf("===============================================\n");
        printf("Can not open the data file - Selection File Import!\n");
        printf("===============================================\n");
        
    }

   

 
    int tetIndex = 0;
    
    for (GLKPOSITION Pos = tetMesh->GetTetraList().GetHeadPosition(); Pos;) {
        QMeshTetra* thisTetra = (QMeshTetra*)tetMesh->GetTetraList().GetNext(Pos);
        fgets(linebuf, 255, fp);
        int isSelected = -1;
        sscanf(linebuf, "%d\n", &isSelected);
        /*if(tetIndex<10)
            qDebug("%d is %d", tetIndex, isSelected);*/

        if(isSelected==0)
            thisTetra->isChamber[0] = false;
        else
            thisTetra->isChamber[0] = true;

        tetIndex++;
    }

    fclose(fp);


    tetIndex = 0;
    /* also find out the chamber face - for simulation usage */
    for (GLKPOSITION Pos = tetMesh->GetFaceList().GetHeadPosition(); Pos;) {
        QMeshFace* thisFace = (QMeshFace*)tetMesh->GetFaceList().GetNext(Pos);
        int chamberTetNum = 0;
        if (thisFace->GetLeftTetra() != NULL && thisFace->GetRightTetra() != NULL) {
            if (thisFace->GetLeftTetra()->isChamber[0])     chamberTetNum++;
            if (thisFace->GetRightTetra()->isChamber[0])    chamberTetNum++;
        }

        if (chamberTetNum == 1) {
            thisFace->isChamberBoundary = true;
            for (int i = 0; i < 3; i++) {
                thisFace->GetNodeRecordPtr(i)->isChamberNode = true;
            }
        }
    }

    printf(" \n TIME - read chamber region takes %ld ms.\n", clock() - time);

}

void meshOperation::updateOBJMesh_chamber(QMeshPatch* chamber) {
    Eigen::Vector3d pp;
    for (GLKPOSITION Pos = chamber->GetNodeList().GetHeadPosition(); Pos;) {
        QMeshNode* Node = (QMeshNode*)chamber->GetNodeList().GetNext(Pos);
        Node->connectTETNode->GetCoord3D(pp);
        Node->SetCoord3D(pp);
    }
}

void meshOperation::updateOBJMesh_chamber(
    QMeshPatch* tetMesh, QMeshPatch* chamber) {

    //////////////////////////// CHAMBER ///////////////////////////
    /* detect chamber element */
    int nodeNum = 0;
    int faceNum = 0;
    for (GLKPOSITION Pos = tetMesh->GetFaceList().GetHeadPosition(); Pos;) {
        QMeshFace* thisFace = (QMeshFace*)tetMesh->GetFaceList().GetNext(Pos);
        if (thisFace->isChamberBoundary) {
            for (int i = 0; i < 3; i++) thisFace->GetNodeRecordPtr(i)->isChamberNode = true;
            faceNum++;
        }
    }
    for (GLKPOSITION Pos = tetMesh->GetNodeList().GetHeadPosition(); Pos;) {
        QMeshNode* Node = (QMeshNode*)tetMesh->GetNodeList().GetNext(Pos);
        if (Node->isChamberNode) {
            Node->chamberIndex = nodeNum;
            nodeNum++;
        }
    }

    float* nodeTable = (float*)malloc(sizeof(float) * nodeNum * 3);
    unsigned int* faceTable = (unsigned int*)malloc(sizeof(unsigned int) * faceNum * 3);

    /* build node list - skin and chamber mesh should not have intersection !!! */
    int nodeIndex = 0;  double pp[3];
    for (GLKPOSITION Pos = tetMesh->GetNodeList().GetHeadPosition(); Pos;) {
        QMeshNode* Node = (QMeshNode*)tetMesh->GetNodeList().GetNext(Pos);
        if (Node->isChamberNode) {
            Node->GetCoord3D(pp[0], pp[1], pp[2]);
            for (int i = 0; i < 3; i++) nodeTable[3 * nodeIndex + i] = pp[i];
            nodeIndex++;
        }
    }

    /* build face list */
    int faceIndex = 0;
    for (GLKPOSITION Pos = tetMesh->GetFaceList().GetHeadPosition(); Pos;) {
        QMeshFace* thisFace = (QMeshFace*)tetMesh->GetFaceList().GetNext(Pos);
        if (thisFace->isChamberBoundary) {
            for (int i = 0; i < 3; i++)
                faceTable[faceIndex * 3 + i] = thisFace->GetNodeRecordPtr(i)->chamberIndex;
            faceIndex++;
        }
    }
    chamber->ClearAll();
    chamber->constructionFromVerFaceTable(nodeNum, nodeTable, faceNum, faceTable);
    
    delete nodeTable;
    delete faceTable;

    std::cout << "update chamber mesh shape" << std::endl;

}

void meshOperation::buildTopologyConnection(QMeshPatch* tetMesh, QMeshPatch* body_init, QMeshPatch* chamber_init) 
{

    for (GLKPOSITION Pos = tetMesh->GetFaceList().GetHeadPosition(); Pos;) {
        QMeshFace* thisFace = (QMeshFace*)tetMesh->GetFaceList().GetNext(Pos);
        thisFace->inner = true;
        if (thisFace->GetLeftTetra() == NULL || thisFace->GetRightTetra() == NULL) {
            thisFace->inner = false;
            for (int i = 0; i < 3; i++) thisFace->GetNodeRecordPtr(i)->isBoundaryNode = true;
        }
    }

    std::vector<QMeshNode*> bodyNodeSet(body_init->GetNodeNumber());
    int index = 0;
    for (GLKPOSITION Pos = body_init->GetNodeList().GetHeadPosition(); Pos;) {
        QMeshNode* Node = (QMeshNode*)body_init->GetNodeList().GetNext(Pos);
        bodyNodeSet[index] = Node; index++;
    }
#pragma omp parallel   
    {
#pragma omp for 
        for (int i = 0; i < body_init->GetNodeNumber(); i++) {
            Eigen::Vector3d pos;
            bodyNodeSet[i]->GetCoord3D(pos);
            for (GLKPOSITION Pos = tetMesh->GetNodeList().GetHeadPosition(); Pos;) {
                QMeshNode* tetNode = (QMeshNode*)tetMesh->GetNodeList().GetNext(Pos);
                if (tetNode->isBoundaryNode == false) continue;
                else {
                    Eigen::Vector3d pp;
                    tetNode->GetCoord3D(pp);
                    if ((pos - pp).norm() < 0.001) {
                        bodyNodeSet[i]->connectTETNode = tetNode;
                        bodyNodeSet[i]->findConnectTETNode = true;
                        //std::cout << tetNode->GetIndexNo() << "," << Node->GetIndexNo() << std::endl;
                        //std::cout << "connection found" << std::endl;
                        //break;
                    }
                }
            }
            if (!bodyNodeSet[i]->findConnectTETNode) std::cout << "WARNING!!! connected node not found (body)!!!!" << std::endl;
        }
    }

    std::vector<QMeshNode*> chamberNodeSet(chamber_init->GetNodeNumber());
    index = 0;
    for (GLKPOSITION Pos = chamber_init->GetNodeList().GetHeadPosition(); Pos;) {
        QMeshNode* Node = (QMeshNode*)chamber_init->GetNodeList().GetNext(Pos);
        chamberNodeSet[index] = Node; index++;
    }

#pragma omp parallel   
    {
#pragma omp for 
        for (int i = 0; i < chamber_init->GetNodeNumber(); i++) {
            Eigen::Vector3d pos;
            chamberNodeSet[i]->GetCoord3D(pos);
            for (GLKPOSITION Pos = tetMesh->GetNodeList().GetHeadPosition(); Pos;) {
                QMeshNode* tetNode = (QMeshNode*)tetMesh->GetNodeList().GetNext(Pos);
                if (tetNode->isChamberNode == false) continue;
                Eigen::Vector3d pp;
                tetNode->GetCoord3D(pp);
                if ((pos - pp).norm() < 0.001) {
                    chamberNodeSet[i]->connectTETNode = tetNode;
                    chamberNodeSet[i]->findConnectTETNode = true;
                    //std::cout << tetNode->GetIndexNo() << "," << Node->GetIndexNo() << std::endl;
                    //std::cout << "connection found" << std::endl;
                    //break;
                }
            }
            if (!chamberNodeSet[i]->findConnectTETNode) std::cout << "WARNING!!! connected node not found (chamber)!!!!" << std::endl;
        }
    }
}

void meshOperation::outputSimulationResult(
    QMeshPatch* tetMesh, QMeshPatch* body, QMeshPatch* chamber,
    std::string modelName, int iter, bool outputTET, bool collRespond)
{

    QMeshPatch* combinedMesh = new QMeshPatch;
    this->_combineTwoSurfaceMesh(body, chamber, combinedMesh);
    
    if (collRespond) {
        std::string outputCombinedMesh =
            "../model/output/" + modelName + "/iter_" + std::to_string(iter) + ".obj";
        combinedMesh->outputOBJFile((char*)outputCombinedMesh.c_str(), false);

        std::string outputChamberMesh =
            "../model/output/" + modelName + "/chamber_iter_" + std::to_string(iter) + ".obj";
        chamber->outputOBJFile((char*)outputChamberMesh.c_str(), false);

        std::string outputBodyMesh =
            "../model/output/" + modelName + "/body_iter_" + std::to_string(iter) + ".obj";
        body->outputOBJFile((char*)outputBodyMesh.c_str(), false);
    }
    else {
        std::string outputCombinedMesh =
            "../model/output/" + modelName + "/iter_" + std::to_string(iter) + "_beforeColl.obj";
        combinedMesh->outputOBJFile((char*)outputCombinedMesh.c_str(), false);

        std::string outputChamberMesh =
            "../model/output/" + modelName + "/chamber_iter_" + std::to_string(iter) + "_beforeColl.obj";
        chamber->outputOBJFile((char*)outputChamberMesh.c_str(), false);

        std::string outputBodyMesh =
            "../model/output/" + modelName + "/body_iter_" + std::to_string(iter) + "_beforeColl.obj";
        body->outputOBJFile((char*)outputBodyMesh.c_str(), false);
    }


    combinedMesh->ClearAll(); 
    delete combinedMesh;

    if (outputTET && collRespond) {
        //----tet file output
        std::ofstream tet_output("../model/output/" + modelName + "/iter_" + std::to_string(iter) + ".tet");
        tet_output << tetMesh->GetNodeNumber() << " vertices" << std::endl;
        tet_output << tetMesh->GetTetraNumber() << " tets" << std::endl;

        int index = 0;	double pp[3];
        for (GLKPOSITION posNode = tetMesh->GetNodeList().GetHeadPosition(); posNode != nullptr;) {
            QMeshNode* node = (QMeshNode*)tetMesh->GetNodeList().GetNext(posNode);
            node->SetIndexNo(index); index++;
            node->GetCoord3D(pp[0], pp[1], pp[2]);
            tet_output << pp[0] << " " << pp[1] << " " << pp[2] << std::endl;
        }
        for (GLKPOSITION posFace = tetMesh->GetTetraList().GetHeadPosition(); posFace != nullptr;) {
            QMeshTetra* tet = (QMeshTetra*)tetMesh->GetTetraList().GetNext(posFace);
            tet_output << "4 " << tet->GetNodeRecordPtr(1)->GetIndexNo()
                << " " << tet->GetNodeRecordPtr(2)->GetIndexNo()
                << " " << tet->GetNodeRecordPtr(3)->GetIndexNo()
                << " " << tet->GetNodeRecordPtr(4)->GetIndexNo() << std::endl;
        }
        tet_output.close();
    }

}

void meshOperation::updateOBJMesh_skin(
    QMeshPatch* tetMesh, QMeshPatch* skin) {
    
    //////////////////////////// SKIN ///////////////////////////
    /* detect skin element */
    int nodeNum = 0;
    int faceNum = 0;
    for (GLKPOSITION Pos = tetMesh->GetFaceList().GetHeadPosition(); Pos;) {
        QMeshFace* thisFace = (QMeshFace*)tetMesh->GetFaceList().GetNext(Pos);
        thisFace->inner = true;
        if (thisFace->GetLeftTetra() == NULL || thisFace->GetRightTetra() == NULL) {
            thisFace->inner = false;
            for (int i = 0; i < 3; i++) thisFace->GetNodeRecordPtr(i)->isBoundaryNode = true;
            faceNum++;
        }
    }
    for (GLKPOSITION Pos = tetMesh->GetNodeList().GetHeadPosition(); Pos;) {
        QMeshNode* Node = (QMeshNode*)tetMesh->GetNodeList().GetNext(Pos);
        if (Node->isBoundaryNode) {
            Node->boundaryIndex = nodeNum;
            nodeNum++;
        }
    }

    float* nodeTable = (float*)malloc(sizeof(float) * nodeNum * 3);
    unsigned int* faceTable = (unsigned int*)malloc(sizeof(unsigned int) * faceNum * 3);

    /* build node list - skin and chamber mesh should not have intersection !!! */
    int nodeIndex = 0;  double pp[3];
    for (GLKPOSITION Pos = tetMesh->GetNodeList().GetHeadPosition(); Pos;) {
        QMeshNode* Node = (QMeshNode*)tetMesh->GetNodeList().GetNext(Pos);
        if (Node->isBoundaryNode) {
            Node->GetCoord3D(pp[0], pp[1], pp[2]);
            for (int i = 0; i < 3; i++) nodeTable[3 * nodeIndex + i] = pp[i];
            nodeIndex++;
        }
    }

    /* build face list */
    int faceIndex = 0;
    for (GLKPOSITION Pos = tetMesh->GetFaceList().GetHeadPosition(); Pos;) {
        QMeshFace* thisFace = (QMeshFace*)tetMesh->GetFaceList().GetNext(Pos);
        if (thisFace->inner == false) {
            for (int i = 0; i < 3; i++)
                faceTable[faceIndex * 3 + i] = thisFace->GetNodeRecordPtr(i)->boundaryIndex;
            faceIndex++;
        }
    }
    skin->ClearAll();
    skin->constructionFromVerFaceTable(nodeNum, nodeTable, faceNum, faceTable);

    delete nodeTable;
    delete faceTable;

    std::cout << "update soft robot body mesh shape" << std::endl;

}

void meshOperation::updateOBJMesh_chamber_skin(QMeshPatch* chamber, QMeshPatch* skin) {

    Eigen::Vector3d pp;
    for (GLKPOSITION Pos = skin->GetNodeList().GetHeadPosition(); Pos;) {
        QMeshNode* Node = (QMeshNode*)skin->GetNodeList().GetNext(Pos);
        Node->connectTETNode->GetCoord3D(pp);
        Node->SetCoord3D(pp);
    }

    for (GLKPOSITION Pos = chamber->GetNodeList().GetHeadPosition(); Pos;) {
        QMeshNode* Node = (QMeshNode*)chamber->GetNodeList().GetNext(Pos);
        Node->connectTETNode->GetCoord3D(pp);
        Node->SetCoord3D(pp);
    }
}

void meshOperation::updateOBJMesh_skin(QMeshPatch* skin) {
    Eigen::Vector3d pp;
    for (GLKPOSITION Pos = skin->GetNodeList().GetHeadPosition(); Pos;) {
        QMeshNode* Node = (QMeshNode*)skin->GetNodeList().GetNext(Pos);
        Node->connectTETNode->GetCoord3D(pp);
        Node->SetCoord3D(pp);
    }
}

double meshOperation::chamberVolumeEvaluation(QMeshPatch* tetMesh) {
    double volume = 0.0;
    for (GLKPOSITION Pos = tetMesh->GetTetraList().GetHeadPosition(); Pos;) {
        QMeshTetra* tetra = (QMeshTetra*)tetMesh->GetTetraList().GetNext(Pos);
        tetra->CalVolume();
        if (tetra->isChamber[0]) volume +=  abs(tetra->GetVolume());
    }
    return volume;
}

void meshOperation::selectShift_rigidRegion(QMeshPatch* surfaceMesh, std::string modelName) {

    double maxX = -999990.99;
    double minX = 999990.99;

    double maxY = -999990.99;
    double minY = 999990.99;

    double maxZ = -999990.99;
    double minZ = 999990.99;

    double pp[3];
    for (GLKPOSITION Pos = surfaceMesh->GetNodeList().GetHeadPosition(); Pos;) {
        QMeshNode* node = (QMeshNode*)surfaceMesh->GetNodeList().GetNext(Pos);
        node->GetCoord3D(pp[0], pp[1], pp[2]);

        if (pp[0] > maxX) maxX = pp[0];
        if (pp[0] < minX) minX = pp[0];

        if (pp[1] > maxY) maxY = pp[1];
        if (pp[1] < minY) minY = pp[1];

        if (pp[2] > maxZ) maxZ = pp[2];
        if (pp[2] < minZ) minZ = pp[2];
    }
    
    for (GLKPOSITION Pos = surfaceMesh->GetNodeList().GetHeadPosition(); Pos;) {
        QMeshNode* node = (QMeshNode*)surfaceMesh->GetNodeList().GetNext(Pos);
        node->GetCoord3D(pp[0], pp[1], pp[2]);
        
        if (modelName == "twisting" && (pp[1] - minY < 0.1)) node->shirftSelect = true; // twisting
        if (modelName == "twisting" && (maxY - pp[1] < 0.1)) node->isTerminalChecking = true; // twisting
        
        if (modelName == "expanding" && (pp[2] - minZ < 0.5)) node->shirftSelect = true; // expanding
        
        if ((modelName == "fingerNew" || modelName == "fingerFinal") && (pp[0] - minX < 0.1)) node->shirftSelect = true; //finger       
        if ((modelName == "fingerNew" || modelName == "fingerFinal") && (maxX - pp[0] < 0.1)) node->isTerminalChecking = true; //finger - terminal

        if (modelName == "finger" && (pp[0] - minX < 0.1)) node->shirftSelect = true; //finger       
        if (modelName == "finger" && (maxX - pp[0] < 0.1)) node->isTerminalChecking = true; //finger - terminal
        
        if (modelName == "mannequim" && (pp[1] - minY < 0.01)) node->shirftSelect = true;
        if (modelName == "mannequim" && (maxY - pp[1] < 0.01)) {
            if (pp[2] * pp[2] + pp[0] * pp[0] < 10.0) node->isTerminalChecking = true;
        }

        //if (maxY - pp[1] < 0.1) node->shirftSelect = true;
        //if (pp[2] - minZ < 0.5) node->shirftSelect = true; // expanding

     
    }

}

void meshOperation::materialSpaceSelection_rigid(QMeshPatch* tetMesh, std::string modelName) {

    double maxX = -999990.99;
    double minX = 999990.99;
    double maxY = -999990.99;
    double minY = 999990.99;
    double maxZ = -999990.99;
    double minZ = 999990.99;

    double rigidDis = 1.0;

    double pp[3];
    for (GLKPOSITION Pos = tetMesh->GetNodeList().GetHeadPosition(); Pos;) {
        QMeshNode* node = (QMeshNode*)tetMesh->GetNodeList().GetNext(Pos);
        node->GetCoord3D(pp[0], pp[1], pp[2]);
        if (pp[0] > maxX) maxX = pp[0];
        if (pp[0] < minX) minX = pp[0];
        if (pp[1] > maxY) maxY = pp[1];
        if (pp[1] < minY) minY = pp[1];
        if (pp[2] > maxZ) maxZ = pp[2];
        if (pp[2] < minZ) minZ = pp[2];
    }
    for (GLKPOSITION Pos = tetMesh->GetNodeList().GetHeadPosition(); Pos;) {
        QMeshNode* node = (QMeshNode*)tetMesh->GetNodeList().GetNext(Pos);
        node->GetCoord3D(pp[0], pp[1], pp[2]);

        //if (modelName == "finger" && (maxY - pp[1] < 2.0) && (maxX - pp[0] > 13.0) && (pp[0] - minX > 13.0)) node->selectedRigid = true;
        if ((modelName == "fingerNew" || modelName == "fingerFinal") && (pp[1] - minY < 1.0) && (pp[0] - minX > 3.0) && (-pp[0] - maxX < 3.0)) node->selectedRigid = true;
        if (modelName == "finger" && (pp[1] - minY < 1.0)) node->selectedRigid = true;

        if (modelName == "twisting" && (pp[1] - minY < rigidDis)) node->selectedRigid = true;
        if (modelName == "twisting" && (-pp[1] + maxY < rigidDis)) node->selectedRigid = true;

        if (modelName == "mannequim" && (pp[1] - minY < 0.05)) node->selectedRigid = true;
        /*if (maxZ - pp[2] < rigidDis) node->selectedRigid = true;
        if (pp[2] - minZ < rigidDis) node->selectedRigid = true;*/
        //if (pp[0] - minX < rigidDis) node->selectedRigid = true;
    }

    for (GLKPOSITION Pos = tetMesh->GetTetraList().GetHeadPosition(); Pos;) {
        QMeshTetra* element = (QMeshTetra*)tetMesh->GetTetraList().GetNext(Pos);
        for (int i = 0; i < 4; i++) {
            if (element->GetNodeRecordPtr(i + 1)->selectedRigid == true) {
                element->isRigid = true;
                for (int j = 1; j < 5; j++) element->GetFaceRecordPtr(j)->isHardDraw = true;
                break;
            }
        }
    }
}


void meshOperation::saveRigidRegion(
    QMeshPatch* tetMesh, std::vector<Eigen::Vector3d>& rigidNodePosSet, bool bodyProtected) {

    for (GLKPOSITION Pos = tetMesh->GetNodeList().GetHeadPosition(); Pos;) {
        QMeshNode* node = (QMeshNode*)tetMesh->GetNodeList().GetNext(Pos);
        if (node->selectedRigid) {
            Eigen::Vector3d pp; node->GetCoord3D(pp);
            rigidNodePosSet.push_back(pp);
        }
    }

    if (bodyProtected == false) {
        for (GLKPOSITION Pos = tetMesh->GetTetraList().GetHeadPosition(); Pos;) {
            QMeshTetra* tet = (QMeshTetra*)tetMesh->GetTetraList().GetNext(Pos);
            if (tet->isRigid) {
                Eigen::Vector3d pp; tet->CalCenterPos(pp(0), pp(1), pp(2));
                rigidNodePosSet.push_back(pp);
            }
        }

    }
    
}

void meshOperation::loadRigidRegion(QMeshPatch* tetMesh, std::vector<Eigen::Vector3d>& rigidNodePosSet) {

    for (GLKPOSITION Pos = tetMesh->GetNodeList().GetHeadPosition(); Pos;) {
        QMeshNode* node = (QMeshNode*)tetMesh->GetNodeList().GetNext(Pos);
        for (int i = 0; i < rigidNodePosSet.size(); i++) {
            Eigen::Vector3d pp; node->GetCoord3D(pp);
            if ((pp - rigidNodePosSet[i]).norm() < 0.5) {
                node->selectedRigid = true;
                break;
            }
        }
    }

    for (GLKPOSITION Pos = tetMesh->GetTetraList().GetHeadPosition(); Pos;) {
        QMeshTetra* element = (QMeshTetra*)tetMesh->GetTetraList().GetNext(Pos);
        for (int i = 0; i < 4; i++) {
            if (element->GetNodeRecordPtr(i + 1)->selectedRigid == true) {
                element->isRigid = true;
                for (int j = 1; j < 5; j++) element->GetFaceRecordPtr(j)->isHardDraw = true;
                break;
            }
        }
    }


}

void meshOperation::laplacianSmoothSurface(QMeshPatch* chamber, QMeshPatch* tetMesh) {

    for (GLKPOSITION Pos = chamber->GetNodeList().GetHeadPosition(); Pos;) {
        QMeshNode* Node = (QMeshNode*)chamber->GetNodeList().GetNext(Pos);
        Eigen::Vector3d pos;
        Node->GetCoord3D(pos);
        for (GLKPOSITION Pos = tetMesh->GetNodeList().GetHeadPosition(); Pos;) {
            QMeshNode* tetNode = (QMeshNode*)tetMesh->GetNodeList().GetNext(Pos);

            Eigen::Vector3d pp;
            tetNode->GetCoord3D(pp);
            if ((pos - pp).norm() < 0.001) {
                Node->connectTETNode = tetNode;
                Node->findConnectTETNode = true;
                //std::cout << "connection found" << std::endl;
                break;
            }
        
        }
        if (!Node->findConnectTETNode) std::cout << "WARNING!!! connected node not found!!!!" << std::endl;
    }

    for (int i = 0; i < 3; i++) {
        for (GLKPOSITION Pos = chamber->GetNodeList().GetHeadPosition(); Pos;) {
            QMeshNode* thisNode = (QMeshNode*)chamber->GetNodeList().GetNext(Pos);

            double pp[3]; int neighNum = 1;
            thisNode->GetCoord3D(pp[0], pp[1], pp[2]);

            for (GLKPOSITION Pos = thisNode->GetEdgeList().GetHeadPosition(); Pos;) {
                QMeshEdge* neighEdge = (QMeshEdge*)thisNode->GetEdgeList().GetNext(Pos);

                QMeshNode* neighNode = neighEdge->GetStartPoint();
                if (neighNode == thisNode) neighNode = neighEdge->GetEndPoint();

                double p1[3];
                neighNode->GetCoord3D(p1[0], p1[1], p1[2]);

                for (int i = 0; i < 3; i++) pp[i] += p1[i];
                neighNum++;
            }
            for (int i = 0; i < 3; i++) pp[i] /= neighNum;
            thisNode->SetCoord3D(pp[0], pp[1], pp[2]);

            thisNode->connectTETNode->SetCoord3D(pp[0], pp[1], pp[2]);

        }
    }
    

}


void meshOperation::selectShift_rigidRegion(QMeshPatch* tetMesh, QMeshPatch* surfaceMesh) {
  
    for (GLKPOSITION Pos = surfaceMesh->GetNodeList().GetHeadPosition(); Pos;) {
        QMeshNode* node = (QMeshNode*)surfaceMesh->GetNodeList().GetNext(Pos);
        if (node->shirftSelect)
        {
            node->connectTETNode->shirftSelect = true;
            node->connectTETNode->isFixed = true;
        }
        else 
        {
            node->connectTETNode->shirftSelect = false;
            node->connectTETNode->isFixed = false;
        }
           
        if (node->isTerminalChecking)
            node->connectTETNode->isTerminalChecking = true;
        else
            node->connectTETNode->isTerminalChecking = false;
    }
    
}

void meshOperation::topologyCheck(QMeshPatch* tetMesh){
    int bodyEleNum = 0;
    for (GLKPOSITION Pos = tetMesh->GetTetraList().GetHeadPosition(); Pos;) {
        QMeshTetra* tet = (QMeshTetra*)tetMesh->GetTetraList().GetNext(Pos);
        int surfaceNode = 0;
        for (int i = 0; i < 4; i++) {
            if (tet->GetNodeRecordPtr(i + 1)->nodeState == 1) surfaceNode++;
        }
        if (surfaceNode == 4) std::cout << "special case tetra element!" << std::endl;

        if (tet->CalVolume() < 0) std::cout << "degeneration happen! volume = " << tet->CalVolume() << std::endl;

        if (tet->isChamber[0] == false)bodyEleNum++;
    }

    std::cout << std::endl << " Topology Checked - PASS!!, bodyEleNum - " << bodyEleNum << std::endl;

}


void meshOperation::_combineTwoSurfaceMesh(
    QMeshPatch* skin, QMeshPatch* chamber, QMeshPatch* combinedMesh) {

    int nodeNum = skin->GetNodeNumber() + chamber->GetNodeNumber();
    float* nodeTable = (float*)malloc(sizeof(float) * nodeNum * 3);

    int faceNum = skin->GetFaceNumber() + chamber->GetFaceNumber();
    unsigned int* faceTable = (unsigned int*)malloc(sizeof(unsigned int) * faceNum * 3);

    /* build node list - skin and chamber mesh should not have intersection !!! */
    int nodeIndex = 0;  double pp[3];
    for (GLKPOSITION Pos = skin->GetNodeList().GetHeadPosition(); Pos;) {
        QMeshNode* Node = (QMeshNode*)skin->GetNodeList().GetNext(Pos);
        Node->SetIndexNo(nodeIndex);    
        Node->GetCoord3D(pp[0], pp[1], pp[2]);
        for (int i = 0; i < 3; i++) nodeTable[3 * nodeIndex + i] = pp[i];
        nodeIndex++;
    }

    for (GLKPOSITION Pos = chamber->GetNodeList().GetHeadPosition(); Pos;) {
        QMeshNode* Node = (QMeshNode*)chamber->GetNodeList().GetNext(Pos);
        Node->SetIndexNo(nodeIndex);
        Node->GetCoord3D(pp[0], pp[1], pp[2]);
        for (int i = 0; i < 3; i++) nodeTable[3 * nodeIndex + i] = pp[i];
        nodeIndex++;
    }

    /* build face list */
    int faceIndex = 0;
    for (GLKPOSITION Pos = skin->GetFaceList().GetHeadPosition(); Pos;) {
        QMeshFace* thisFace = (QMeshFace*)skin->GetFaceList().GetNext(Pos);
        for (int i = 0; i < 3; i++)
            faceTable[faceIndex * 3 + i] = thisFace->GetNodeRecordPtr(i)->GetIndexNo();  
        faceIndex++;
    }

    for (GLKPOSITION Pos = chamber->GetFaceList().GetHeadPosition(); Pos;) {
        QMeshFace* thisFace = (QMeshFace*)chamber->GetFaceList().GetNext(Pos);
        for (int i = 0; i < 3; i++)
            faceTable[faceIndex * 3 + i] = thisFace->GetNodeRecordPtr(i)->GetIndexNo();
        faceIndex++;
    }

    combinedMesh->constructionFromVerFaceTable(nodeNum, nodeTable, faceNum, faceTable);

    delete nodeTable;
    delete faceTable;

}

bool meshOperation::_calculatePointInsideMesh(QMeshPatch* target_mesh, Eigen::Vector3d& orig) {
    // calculate distance between Pnt with faces of model
    Eigen::Vector3d dir = { 1.0,0.0,0.0 };


    int intersection_Time = 0;

    for (GLKPOSITION Pos = target_mesh->GetFaceList().GetHeadPosition(); Pos;) {
        QMeshFace* each_face = (QMeshFace*)target_mesh->GetFaceList().GetNext(Pos);

        double xx, yy, zz;
        each_face->GetNodeRecordPtr(0)->GetCoord3D(xx, yy, zz);
        Eigen::Vector3d v0 = { xx,yy,zz };

        each_face->GetNodeRecordPtr(1)->GetCoord3D(xx, yy, zz);
        Eigen::Vector3d v1 = { xx,yy,zz };

        each_face->GetNodeRecordPtr(2)->GetCoord3D(xx, yy, zz);
        Eigen::Vector3d v2 = { xx,yy,zz };


        if (this->_IntersectTriangle(orig, dir, v0, v1, v2))
            intersection_Time++;

    }
    //std::cout << "intersection Num " << intersection_Time << std::endl;
    if (intersection_Time % 2 != 0) {
        //std::cout << "in the mesh" << std::endl;
        return true;
    }
    else return false;
    //std::cout << "be out of mesh" << std::endl;
}

// Determine whether a ray intersect with a triangle
// Parameters
// orig: origin of the ray
// dir: direction of the ray
// v0, v1, v2: vertices of triangle
// t(out): weight of the intersection for the ray
// u(out), v(out): barycentric coordinate of intersection

bool meshOperation::_IntersectTriangle(const Eigen::Vector3d& orig, const Eigen::Vector3d& dir,
    Eigen::Vector3d& v0, Eigen::Vector3d& v1, Eigen::Vector3d& v2)
{
    // E1
    Eigen::Vector3d E1 = v1 - v0;

    // E2
    Eigen::Vector3d E2 = v2 - v0;

    // P
    Eigen::Vector3d P = dir.cross(E2);

    // determinant
    float det = E1.dot(P);

    // keep det > 0, modify T accordingly
    Eigen::Vector3d T;
    if (det > 0)
    {
        T = orig - v0;
    }
    else
    {
        T = v0 - orig;
        det = -det;
    }

    // If determinant is near zero, ray lies in plane of triangle
    if (det < 0.0001f)
        return false;

    // Calculate u and make sure u <= 1
    double t, u, v;
    u = T.dot(P);
    if (u < 0.0f || u > det)
        return false;

    // Q
    Eigen::Vector3d Q = T.cross(E1);

    // Calculate v and make sure u + v <= 1
    v = dir.dot(Q);
    if (v < 0.0f || u + v > det)
        return false;

    // Calculate t, scale parameters, ray intersects triangle
    t = E2.dot(Q);
    if (t < 0) return false;

    float fInvDet = 1.0f / det;
    t *= fInvDet;
    u *= fInvDet;
    v *= fInvDet;

    return true;
}

int meshOperation::_segementMesh_withFlag(QMeshPatch* inputMesh) {

    int partNum = 0;
    bool stopIter;
    do {
        stopIter = true;

        bool stopFlooding = false;
        int thisPartNodeNum = 0;
        int lastIterNodeNum = 0;

        /* initialize by select the first node */
        for (GLKPOSITION Pos = inputMesh->GetNodeList().GetHeadPosition(); Pos;) {
            QMeshNode* Node = (QMeshNode*)inputMesh->GetNodeList().GetNext(Pos);
            if (Node->partSegementProcessed == false) {
                Node->partIndex = partNum;
                thisPartNodeNum++;
                break;
            }
        }
        /* flooding algorithm */
        do {
            std::cout << "begin iteration -- num = ";
            lastIterNodeNum = thisPartNodeNum;
            for (GLKPOSITION Pos = inputMesh->GetNodeList().GetHeadPosition(); Pos;) {
                QMeshNode* Node = (QMeshNode*)inputMesh->GetNodeList().GetNext(Pos);
                if (Node->partIndex == partNum) {
                    for (GLKPOSITION Pos = Node->GetEdgeList().GetHeadPosition(); Pos;) {
                        QMeshEdge* edge = (QMeshEdge*)Node->GetEdgeList().GetNext(Pos);
                        QMeshNode* neighborNode;
                        if (edge->GetStartPoint() == Node) neighborNode = edge->GetEndPoint();
                        else neighborNode = edge->GetStartPoint();

                        if (!neighborNode->partSegementProcessed) {
                            neighborNode->partIndex = partNum;
                            //std::cout << "find one neighbor node!" << std::endl;
                            lastIterNodeNum++;
                        }

                    }
                    Node->partSegementProcessed = true;
                }
            }
            std::cout << thisPartNodeNum << ", " << lastIterNodeNum << std::endl;
            if (thisPartNodeNum == lastIterNodeNum) stopFlooding = true;
            thisPartNodeNum = lastIterNodeNum;
        } while (!stopFlooding);

        /* check if terminal condition is achieved */
        for (GLKPOSITION Pos = inputMesh->GetNodeList().GetHeadPosition(); Pos;) {
            QMeshNode* Node = (QMeshNode*)inputMesh->GetNodeList().GetNext(Pos);
            if (Node->partSegementProcessed == false) {
                stopIter = false;
                partNum++;
                std::cout << "iteration not break! Detect next section!" << std::endl;
                break;
            }
        }

    } while (!stopIter);

    /* setup face flag */
    for (GLKPOSITION Pos = inputMesh->GetFaceList().GetHeadPosition(); Pos;) {
        QMeshFace* face = (QMeshFace*)inputMesh->GetFaceList().GetNext(Pos);
        face->partIndex = face->GetNodeRecordPtr(0)->partIndex;
        if (face->GetNodeRecordPtr(1)->partIndex != face->partIndex ||
            face->GetNodeRecordPtr(2)->partIndex != face->partIndex)
            std::cout << "ERROR, the segemantation is not well conducted !!!!!" << std::endl; 
    }
    return partNum + 1;
}

void meshOperation::_generateNewMeshPart(QMeshPatch* inputMesh, QMeshPatch* newMesh, int partIndex) {

    //////////////////////////// CHAMBER ///////////////////////////
   /* detect chamber element */
    int nodeNum = 0;
    int faceNum = 0;
    for (GLKPOSITION Pos = inputMesh->GetFaceList().GetHeadPosition(); Pos;) {
        QMeshFace* thisFace = (QMeshFace*)inputMesh->GetFaceList().GetNext(Pos);
        if (thisFace->partIndex == partIndex) {
            faceNum++;
        }
    }
    for (GLKPOSITION Pos = inputMesh->GetNodeList().GetHeadPosition(); Pos;) {
        QMeshNode* Node = (QMeshNode*)inputMesh->GetNodeList().GetNext(Pos);
        if (Node->partIndex == partIndex) {
            Node->chamberIndex = nodeNum;
            nodeNum++;
        }
    }

    float* nodeTable = (float*)malloc(sizeof(float) * nodeNum * 3);
    unsigned int* faceTable = (unsigned int*)malloc(sizeof(unsigned int) * faceNum * 3);

    /* build node list - skin and chamber mesh should not have intersection !!! */
    int nodeIndex = 0;  double pp[3];
    for (GLKPOSITION Pos = inputMesh->GetNodeList().GetHeadPosition(); Pos;) {
        QMeshNode* Node = (QMeshNode*)inputMesh->GetNodeList().GetNext(Pos);
        if (Node->partIndex == partIndex) {
            Node->GetCoord3D(pp[0], pp[1], pp[2]);
            for (int i = 0; i < 3; i++) nodeTable[3 * nodeIndex + i] = pp[i];
            nodeIndex++;
        }
    }

    /* build face list */
    int faceIndex = 0;
    for (GLKPOSITION Pos = inputMesh->GetFaceList().GetHeadPosition(); Pos;) {
        QMeshFace* thisFace = (QMeshFace*)inputMesh->GetFaceList().GetNext(Pos);
        if (thisFace->partIndex == partIndex) {
            for (int i = 0; i < 3; i++)
                faceTable[faceIndex * 3 + i] = thisFace->GetNodeRecordPtr(i)->chamberIndex;
            faceIndex++;
        }
    }
    newMesh->constructionFromVerFaceTable(nodeNum, nodeTable, faceNum, faceTable);

    delete nodeTable;
    delete faceTable;

}

// false - skin mesh with index = 0
bool meshOperation::_detectSegmentationOrder(QMeshPatch* inputMesh) {

    int partNum1 = 0;
    int partNum2 = 0;

    for (GLKPOSITION Pos = inputMesh->GetNodeList().GetHeadPosition(); Pos;) {
        QMeshNode* Node = (QMeshNode*)inputMesh->GetNodeList().GetNext(Pos);
        if (Node->partIndex == 0) partNum1++;
        else if (Node->partIndex == 1) partNum2++;
    }

    if (partNum1 > partNum2) return false; // this means index = 0 should be the skin mesh
    else return true;
}


#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QSignalMapper>
#include <QStandardItemModel>
#include <QTimer>

#include "../GLKLib/GLKLib.h"
#include "../QMeshLib/PolygenMesh.h"

#include "meshOperation.h"

#include <iostream>
#include <fstream>      // std::ifstream
#include <cstring>

#include "ui_MainWindow.h"
#include "AABB.h"
#include "DeformTet.h"
#include "soroPneumaticKinematics.h"
#include "MannequinMacro.h"



namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

private:
    Ui::MainWindow *ui;
    GLKLib *pGLK;
    GLKObList polygenMeshList;

private:
    void createActions();
    void createTreeView();
    void createActions_SoftRobotFunction();

    PolygenMesh *getSelectedPolygenMesh();

    QSignalMapper *signalMapper;
    QStandardItemModel *treeModel;

protected:
    void dragEnterEvent(QDragEnterEvent *event);
    void dropEvent(QDropEvent *event);

private slots:
    void open();
    void save();
    void signalNavigation(int flag);
    void shiftToOrigin();
    void updateTree();
    void on_pushButton_clearAll_clicked();
    void on_treeView_clicked(const QModelIndex &index);
    void generateTetraMeshbyTetgen();
    PolygenMesh* _buildPolygenMesh(std::string name);
    void split_Show_TET_Mesh();

public slots:
    void soroGenerateTetraMesh_withChamber();

    void soroSimulation_highExpandingModel();
    void soroSimulation_dataGeneration();
    void collisionCheckingByAABBTree();

private:
    void MainWindow::inputSoftRobotModel(
        QMeshPatch* body_init, QMeshPatch* chamber_init, QMeshPatch* tetMesh,
        meshOperation* meshOperator, std::string modelName);

    void MainWindow::inputSoftRobotModelWithShifting(
        QMeshPatch* body_init, QMeshPatch* chamber_init, QMeshPatch* tetMesh,
        meshOperation* meshOperator, std::string modelName, std::string modelIdx, Eigen::Vector3d shift);


    double actuatorPara_twisting = 2.5; //.2.5 for twisting, 5.0 for expanding, 3.0 for finger
    int iterationTimeSOROSimulation = 1;


public:
    void InputEnvironmentObstacle(void);

    //QMeshPatch
    QMeshPatch* body_init;
    QMeshPatch* chamber_init;
    QMeshPatch* tetMesh;

    std::vector<Eigen::MatrixXd> initShape[MAN_CHAMBER_SIZE];

    /*Free-form Membrane Simulation*/
public:
    void SignalSlotConnection(void);


    //size is MAN_CHAMBER_SIZE
    QVector<QMeshPatch*> ChamberArray_body;         //body element
    QVector<QMeshPatch*> ChamberArray_chamber;      //actuation element: chamber
    QVector<QMeshPatch*> ChamberArray_tet;          //

    QMeshPatch* _freeformMem;       //ball obstacle
    PolygenMesh* corrspondencePolySelfCollision_global;     //self-collision correspondence polygenMesh
    PolygenMesh* collidedBoxPolySelfCollision_global;       //self-collision collided box polygenMesh
    PolygenMesh* collidedTetPolySelfCollision_global;       //self-collision collided tet polygenMesh



    PolygenMesh* corrspondencePoly_global;                  //collision with env




    PolygenMesh* _aabbVisualizationPoly;
    QMeshPatch* _aabbVisualizationMesh;

public slots:
    void InputFreeFormMembrane(void);
    void GenerateChamberTetMesh(void);
    void ChamberDeformation(void);
    void InputMem(void);
    void CollisionChecking(void);
    void CollisionResponse(void);

    void MoveBall(void);
    void TrajGeneration(void);

    void ShowAABBTree(void);

    void CheckBox_onlyLeafNode(int checkBoxState);
    void CheckBox_overlappingNode(int checkBoxState);

    void ChangeAABBVisualizedDepth(int sliderValue);


    void OutputFiles(void);
    
};

#endif // MAINWINDOW_H

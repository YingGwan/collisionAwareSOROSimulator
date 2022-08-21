/********************************************************************************
** Form generated from reading UI file 'MainWindow.ui'
**
** Created by: Qt User Interface Compiler version 5.12.3
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_MAINWINDOW_H
#define UI_MAINWINDOW_H

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QCheckBox>
#include <QtWidgets/QComboBox>
#include <QtWidgets/QDockWidget>
#include <QtWidgets/QDoubleSpinBox>
#include <QtWidgets/QFrame>
#include <QtWidgets/QGridLayout>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QLCDNumber>
#include <QtWidgets/QLabel>
#include <QtWidgets/QLineEdit>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QMenu>
#include <QtWidgets/QMenuBar>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QSlider>
#include <QtWidgets/QSpacerItem>
#include <QtWidgets/QSpinBox>
#include <QtWidgets/QStatusBar>
#include <QtWidgets/QTabWidget>
#include <QtWidgets/QToolBar>
#include <QtWidgets/QTreeView>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_MainWindow
{
public:
    QAction *actionOpen;
    QAction *actionFront;
    QAction *actionBack;
    QAction *actionTop;
    QAction *actionBottom;
    QAction *actionLeft;
    QAction *actionRight;
    QAction *actionIsometric;
    QAction *actionZoom_In;
    QAction *actionZoom_Out;
    QAction *actionZoom_All;
    QAction *actionZoom_Window;
    QAction *actionShade;
    QAction *actionMesh;
    QAction *actionNode;
    QAction *actionSave;
    QAction *actionSelectNode;
    QAction *actionSelectFace;
    QAction *actionShifttoOrigin;
    QAction *actionProfile;
    QAction *actionFaceNormal;
    QAction *actionNodeNormal;
    QAction *actionSelectEdge;
    QAction *actionGenerate;
    QAction *actionTest_1;
    QWidget *centralWidget;
    QHBoxLayout *horizontalLayout;
    QToolBar *navigationToolBar;
    QToolBar *toolBar;
    QStatusBar *statusBar;
    QToolBar *selectionToolBar;
    QDockWidget *dockWidget;
    QWidget *dockWidgetContents;
    QGridLayout *gridLayout;
    QSpacerItem *verticalSpacer_2;
    QTreeView *treeView;
    QTabWidget *tabWidget;
    QWidget *tab;
    QVBoxLayout *verticalLayout_2;
    QHBoxLayout *horizontalLayout_2;
    QCheckBox *checkBox_combinedInput;
    QLabel *label;
    QSpinBox *spinBox_iterTime;
    QComboBox *comboBox_planeDir;
    QSlider *horizontalSlider_slice_Multi_dir;
    QCheckBox *checkBox_readChamberRegion;
    QPushButton *pushButton_generateTETMesh;
    QPushButton *pushButton_runCollisionChecking;
    QDoubleSpinBox *doubleSpinBox_A1;
    QDoubleSpinBox *doubleSpinBox_A2;
    QDoubleSpinBox *doubleSpinBox_A3;
    QPushButton *pushButton_clearAll;
    QPushButton *pushButton_trajectoryGeneration;
    QPushButton *pushButton_CollisionResponse;
    QWidget *tab_2;
    QVBoxLayout *verticalLayout_3;
    QHBoxLayout *horizontalLayout_9;
    QCheckBox *checkBox_showAABB;
    QPushButton *pushButton_inputFourChambers;
    QPushButton *pushButton_inputMem;
    QPushButton *pushButton_GenerateChamberTetMesh;
    QPushButton *pushButton_showAABB_Tree;
    QFrame *line;
    QDoubleSpinBox *doubleSpinBox_Chamber1;
    QDoubleSpinBox *doubleSpinBox_Chamber2;
    QDoubleSpinBox *doubleSpinBox_Chamber3;
    QDoubleSpinBox *doubleSpinBox_Chamber4;
    QPushButton *pushButton_ChamberDeformation;
    QFrame *line_2;
    QLabel *label_2;
    QDoubleSpinBox *doubleSpinBox_ballMoveX;
    QDoubleSpinBox *doubleSpinBox_ballMoveY;
    QDoubleSpinBox *doubleSpinBox_ballMoveZ;
    QPushButton *pushButton_MoveBall;
    QPushButton *pushButton_CollisionChecking;
    QSpacerItem *verticalSpacer;
    QWidget *tab_3;
    QVBoxLayout *verticalLayout;
    QLabel *label_3;
    QHBoxLayout *horizontalLayout_3;
    QCheckBox *checkBox_onlyLeafNode;
    QSpacerItem *horizontalSpacer;
    QHBoxLayout *horizontalLayout_4;
    QCheckBox *checkBox_overlappingNode;
    QSpacerItem *horizontalSpacer_2;
    QHBoxLayout *horizontalLayout_5;
    QSlider *horizontalSlider_AABBTree;
    QLCDNumber *lcdNumber_AABBTree;
    QFrame *line_3;
    QLabel *label_4;
    QHBoxLayout *horizontalLayout_6;
    QLineEdit *lineEdit_bodyOutputName;
    QPushButton *pushButton_outputBody;
    QHBoxLayout *horizontalLayout_7;
    QLineEdit *lineEdit_chamberOutputName;
    QPushButton *pushButton_outputChamber;
    QHBoxLayout *horizontalLayout_8;
    QLineEdit *lineEdit_tetOutputName;
    QPushButton *pushButton_outputTet;
    QSpacerItem *verticalSpacer_3;
    QWidget *tab_4;
    QVBoxLayout *verticalLayout_4;
    QSpacerItem *verticalSpacer_4;
    QMenuBar *menuBar;
    QMenu *menuFile;
    QMenu *menuView;
    QMenu *menuSelect;

    void setupUi(QMainWindow *MainWindow)
    {
        if (MainWindow->objectName().isEmpty())
            MainWindow->setObjectName(QString::fromUtf8("MainWindow"));
        MainWindow->setEnabled(true);
        MainWindow->resize(1360, 1014);
        MainWindow->setMouseTracking(true);
        MainWindow->setFocusPolicy(Qt::StrongFocus);
        MainWindow->setAcceptDrops(true);
        actionOpen = new QAction(MainWindow);
        actionOpen->setObjectName(QString::fromUtf8("actionOpen"));
        QIcon icon;
        icon.addFile(QString::fromUtf8(":/resource/Open Folder.png"), QSize(), QIcon::Normal, QIcon::Off);
        actionOpen->setIcon(icon);
        actionFront = new QAction(MainWindow);
        actionFront->setObjectName(QString::fromUtf8("actionFront"));
        actionFront->setCheckable(false);
        QIcon icon1;
        icon1.addFile(QString::fromUtf8(":/resource/Front View.png"), QSize(), QIcon::Normal, QIcon::Off);
        actionFront->setIcon(icon1);
        actionBack = new QAction(MainWindow);
        actionBack->setObjectName(QString::fromUtf8("actionBack"));
        actionBack->setCheckable(false);
        QIcon icon2;
        icon2.addFile(QString::fromUtf8(":/resource/Back View.png"), QSize(), QIcon::Normal, QIcon::Off);
        actionBack->setIcon(icon2);
        actionTop = new QAction(MainWindow);
        actionTop->setObjectName(QString::fromUtf8("actionTop"));
        actionTop->setCheckable(false);
        QIcon icon3;
        icon3.addFile(QString::fromUtf8(":/resource/Top View.png"), QSize(), QIcon::Normal, QIcon::Off);
        actionTop->setIcon(icon3);
        actionBottom = new QAction(MainWindow);
        actionBottom->setObjectName(QString::fromUtf8("actionBottom"));
        actionBottom->setCheckable(false);
        QIcon icon4;
        icon4.addFile(QString::fromUtf8(":/resource/Bottom View.png"), QSize(), QIcon::Normal, QIcon::Off);
        actionBottom->setIcon(icon4);
        actionLeft = new QAction(MainWindow);
        actionLeft->setObjectName(QString::fromUtf8("actionLeft"));
        QIcon icon5;
        icon5.addFile(QString::fromUtf8(":/resource/Left View.png"), QSize(), QIcon::Normal, QIcon::Off);
        actionLeft->setIcon(icon5);
        actionRight = new QAction(MainWindow);
        actionRight->setObjectName(QString::fromUtf8("actionRight"));
        QIcon icon6;
        icon6.addFile(QString::fromUtf8(":/resource/Right View.png"), QSize(), QIcon::Normal, QIcon::Off);
        actionRight->setIcon(icon6);
        actionIsometric = new QAction(MainWindow);
        actionIsometric->setObjectName(QString::fromUtf8("actionIsometric"));
        QIcon icon7;
        icon7.addFile(QString::fromUtf8(":/resource/Isometric View.png"), QSize(), QIcon::Normal, QIcon::Off);
        actionIsometric->setIcon(icon7);
        actionZoom_In = new QAction(MainWindow);
        actionZoom_In->setObjectName(QString::fromUtf8("actionZoom_In"));
        QIcon icon8;
        icon8.addFile(QString::fromUtf8(":/resource/Zoom In.png"), QSize(), QIcon::Normal, QIcon::Off);
        actionZoom_In->setIcon(icon8);
        actionZoom_Out = new QAction(MainWindow);
        actionZoom_Out->setObjectName(QString::fromUtf8("actionZoom_Out"));
        QIcon icon9;
        icon9.addFile(QString::fromUtf8(":/resource/Zoom Out.png"), QSize(), QIcon::Normal, QIcon::Off);
        actionZoom_Out->setIcon(icon9);
        actionZoom_All = new QAction(MainWindow);
        actionZoom_All->setObjectName(QString::fromUtf8("actionZoom_All"));
        QIcon icon10;
        icon10.addFile(QString::fromUtf8(":/resource/Zoom All.png"), QSize(), QIcon::Normal, QIcon::Off);
        actionZoom_All->setIcon(icon10);
        actionZoom_Window = new QAction(MainWindow);
        actionZoom_Window->setObjectName(QString::fromUtf8("actionZoom_Window"));
        QIcon icon11;
        icon11.addFile(QString::fromUtf8(":/resource/Zoom Window.png"), QSize(), QIcon::Normal, QIcon::Off);
        actionZoom_Window->setIcon(icon11);
        actionShade = new QAction(MainWindow);
        actionShade->setObjectName(QString::fromUtf8("actionShade"));
        actionShade->setCheckable(true);
        QIcon icon12;
        icon12.addFile(QString::fromUtf8(":/resource/Shade.png"), QSize(), QIcon::Normal, QIcon::Off);
        actionShade->setIcon(icon12);
        actionMesh = new QAction(MainWindow);
        actionMesh->setObjectName(QString::fromUtf8("actionMesh"));
        actionMesh->setCheckable(true);
        QIcon icon13;
        icon13.addFile(QString::fromUtf8(":/resource/Mesh.png"), QSize(), QIcon::Normal, QIcon::Off);
        actionMesh->setIcon(icon13);
        actionNode = new QAction(MainWindow);
        actionNode->setObjectName(QString::fromUtf8("actionNode"));
        actionNode->setCheckable(true);
        QIcon icon14;
        icon14.addFile(QString::fromUtf8(":/resource/Node.png"), QSize(), QIcon::Normal, QIcon::Off);
        actionNode->setIcon(icon14);
        actionSave = new QAction(MainWindow);
        actionSave->setObjectName(QString::fromUtf8("actionSave"));
        QIcon icon15;
        icon15.addFile(QString::fromUtf8(":/resource/Save as.png"), QSize(), QIcon::Normal, QIcon::Off);
        actionSave->setIcon(icon15);
        actionSelectNode = new QAction(MainWindow);
        actionSelectNode->setObjectName(QString::fromUtf8("actionSelectNode"));
        QIcon icon16;
        icon16.addFile(QString::fromUtf8(":/resource/selectNode.png"), QSize(), QIcon::Normal, QIcon::Off);
        actionSelectNode->setIcon(icon16);
        actionSelectFace = new QAction(MainWindow);
        actionSelectFace->setObjectName(QString::fromUtf8("actionSelectFace"));
        QIcon icon17;
        icon17.addFile(QString::fromUtf8(":/resource/selectFace.png"), QSize(), QIcon::Normal, QIcon::Off);
        actionSelectFace->setIcon(icon17);
        actionShifttoOrigin = new QAction(MainWindow);
        actionShifttoOrigin->setObjectName(QString::fromUtf8("actionShifttoOrigin"));
        actionProfile = new QAction(MainWindow);
        actionProfile->setObjectName(QString::fromUtf8("actionProfile"));
        actionProfile->setCheckable(true);
        QIcon icon18;
        icon18.addFile(QString::fromUtf8(":/resource/Profile.png"), QSize(), QIcon::Normal, QIcon::Off);
        actionProfile->setIcon(icon18);
        actionFaceNormal = new QAction(MainWindow);
        actionFaceNormal->setObjectName(QString::fromUtf8("actionFaceNormal"));
        actionFaceNormal->setCheckable(true);
        QIcon icon19;
        icon19.addFile(QString::fromUtf8(":/resource/FaceNormal.png"), QSize(), QIcon::Normal, QIcon::Off);
        actionFaceNormal->setIcon(icon19);
        actionNodeNormal = new QAction(MainWindow);
        actionNodeNormal->setObjectName(QString::fromUtf8("actionNodeNormal"));
        actionNodeNormal->setCheckable(true);
        QIcon icon20;
        icon20.addFile(QString::fromUtf8(":/resource/NodeNormal.png"), QSize(), QIcon::Normal, QIcon::Off);
        actionNodeNormal->setIcon(icon20);
        actionSelectEdge = new QAction(MainWindow);
        actionSelectEdge->setObjectName(QString::fromUtf8("actionSelectEdge"));
        QIcon icon21;
        icon21.addFile(QString::fromUtf8(":/resource/selectEdge.png"), QSize(), QIcon::Normal, QIcon::Off);
        actionSelectEdge->setIcon(icon21);
        actionGenerate = new QAction(MainWindow);
        actionGenerate->setObjectName(QString::fromUtf8("actionGenerate"));
        actionTest_1 = new QAction(MainWindow);
        actionTest_1->setObjectName(QString::fromUtf8("actionTest_1"));
        centralWidget = new QWidget(MainWindow);
        centralWidget->setObjectName(QString::fromUtf8("centralWidget"));
        centralWidget->setMouseTracking(true);
        centralWidget->setAcceptDrops(true);
        horizontalLayout = new QHBoxLayout(centralWidget);
        horizontalLayout->setSpacing(6);
        horizontalLayout->setContentsMargins(11, 11, 11, 11);
        horizontalLayout->setObjectName(QString::fromUtf8("horizontalLayout"));
        MainWindow->setCentralWidget(centralWidget);
        navigationToolBar = new QToolBar(MainWindow);
        navigationToolBar->setObjectName(QString::fromUtf8("navigationToolBar"));
        navigationToolBar->setMovable(false);
        navigationToolBar->setIconSize(QSize(25, 25));
        navigationToolBar->setFloatable(false);
        MainWindow->addToolBar(Qt::TopToolBarArea, navigationToolBar);
        toolBar = new QToolBar(MainWindow);
        toolBar->setObjectName(QString::fromUtf8("toolBar"));
        toolBar->setMovable(false);
        toolBar->setFloatable(false);
        MainWindow->addToolBar(Qt::TopToolBarArea, toolBar);
        statusBar = new QStatusBar(MainWindow);
        statusBar->setObjectName(QString::fromUtf8("statusBar"));
        MainWindow->setStatusBar(statusBar);
        selectionToolBar = new QToolBar(MainWindow);
        selectionToolBar->setObjectName(QString::fromUtf8("selectionToolBar"));
        selectionToolBar->setEnabled(true);
        selectionToolBar->setMovable(false);
        selectionToolBar->setIconSize(QSize(25, 25));
        selectionToolBar->setFloatable(false);
        MainWindow->addToolBar(Qt::TopToolBarArea, selectionToolBar);
        dockWidget = new QDockWidget(MainWindow);
        dockWidget->setObjectName(QString::fromUtf8("dockWidget"));
        dockWidget->setEnabled(true);
        QSizePolicy sizePolicy(QSizePolicy::Minimum, QSizePolicy::Maximum);
        sizePolicy.setHorizontalStretch(0);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(dockWidget->sizePolicy().hasHeightForWidth());
        dockWidget->setSizePolicy(sizePolicy);
        dockWidgetContents = new QWidget();
        dockWidgetContents->setObjectName(QString::fromUtf8("dockWidgetContents"));
        dockWidgetContents->setLayoutDirection(Qt::LeftToRight);
        gridLayout = new QGridLayout(dockWidgetContents);
        gridLayout->setSpacing(6);
        gridLayout->setContentsMargins(11, 11, 11, 11);
        gridLayout->setObjectName(QString::fromUtf8("gridLayout"));
        verticalSpacer_2 = new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Expanding);

        gridLayout->addItem(verticalSpacer_2, 2, 0, 1, 1);

        treeView = new QTreeView(dockWidgetContents);
        treeView->setObjectName(QString::fromUtf8("treeView"));
        treeView->setEnabled(true);
        treeView->setMinimumSize(QSize(0, 320));
        treeView->setMaximumSize(QSize(16777215, 400));
        treeView->setProperty("showDropIndicator", QVariant(true));
        treeView->setIndentation(5);
        treeView->header()->setVisible(false);

        gridLayout->addWidget(treeView, 3, 0, 1, 1);

        tabWidget = new QTabWidget(dockWidgetContents);
        tabWidget->setObjectName(QString::fromUtf8("tabWidget"));
        tabWidget->setEnabled(true);
        tabWidget->setMaximumSize(QSize(16777215, 500));
        tabWidget->setIconSize(QSize(16, 16));
        tab = new QWidget();
        tab->setObjectName(QString::fromUtf8("tab"));
        verticalLayout_2 = new QVBoxLayout(tab);
        verticalLayout_2->setSpacing(6);
        verticalLayout_2->setContentsMargins(11, 11, 11, 11);
        verticalLayout_2->setObjectName(QString::fromUtf8("verticalLayout_2"));
        horizontalLayout_2 = new QHBoxLayout();
        horizontalLayout_2->setSpacing(6);
        horizontalLayout_2->setObjectName(QString::fromUtf8("horizontalLayout_2"));
        checkBox_combinedInput = new QCheckBox(tab);
        checkBox_combinedInput->setObjectName(QString::fromUtf8("checkBox_combinedInput"));
        checkBox_combinedInput->setEnabled(false);

        horizontalLayout_2->addWidget(checkBox_combinedInput);

        label = new QLabel(tab);
        label->setObjectName(QString::fromUtf8("label"));
        label->setEnabled(false);

        horizontalLayout_2->addWidget(label);

        spinBox_iterTime = new QSpinBox(tab);
        spinBox_iterTime->setObjectName(QString::fromUtf8("spinBox_iterTime"));
        spinBox_iterTime->setEnabled(false);
        spinBox_iterTime->setValue(1);

        horizontalLayout_2->addWidget(spinBox_iterTime);


        verticalLayout_2->addLayout(horizontalLayout_2);

        comboBox_planeDir = new QComboBox(tab);
        comboBox_planeDir->addItem(QString());
        comboBox_planeDir->addItem(QString());
        comboBox_planeDir->addItem(QString());
        comboBox_planeDir->setObjectName(QString::fromUtf8("comboBox_planeDir"));
        comboBox_planeDir->setEnabled(false);

        verticalLayout_2->addWidget(comboBox_planeDir);

        horizontalSlider_slice_Multi_dir = new QSlider(tab);
        horizontalSlider_slice_Multi_dir->setObjectName(QString::fromUtf8("horizontalSlider_slice_Multi_dir"));
        horizontalSlider_slice_Multi_dir->setOrientation(Qt::Horizontal);

        verticalLayout_2->addWidget(horizontalSlider_slice_Multi_dir);

        checkBox_readChamberRegion = new QCheckBox(tab);
        checkBox_readChamberRegion->setObjectName(QString::fromUtf8("checkBox_readChamberRegion"));
        checkBox_readChamberRegion->setEnabled(false);

        verticalLayout_2->addWidget(checkBox_readChamberRegion);

        pushButton_generateTETMesh = new QPushButton(tab);
        pushButton_generateTETMesh->setObjectName(QString::fromUtf8("pushButton_generateTETMesh"));
        pushButton_generateTETMesh->setEnabled(false);

        verticalLayout_2->addWidget(pushButton_generateTETMesh);

        pushButton_runCollisionChecking = new QPushButton(tab);
        pushButton_runCollisionChecking->setObjectName(QString::fromUtf8("pushButton_runCollisionChecking"));
        pushButton_runCollisionChecking->setEnabled(false);

        verticalLayout_2->addWidget(pushButton_runCollisionChecking);

        doubleSpinBox_A1 = new QDoubleSpinBox(tab);
        doubleSpinBox_A1->setObjectName(QString::fromUtf8("doubleSpinBox_A1"));
        doubleSpinBox_A1->setEnabled(false);
        doubleSpinBox_A1->setMinimum(1.000000000000000);
        doubleSpinBox_A1->setMaximum(100.000000000000000);
        doubleSpinBox_A1->setValue(1.300000000000000);

        verticalLayout_2->addWidget(doubleSpinBox_A1);

        doubleSpinBox_A2 = new QDoubleSpinBox(tab);
        doubleSpinBox_A2->setObjectName(QString::fromUtf8("doubleSpinBox_A2"));
        doubleSpinBox_A2->setEnabled(false);
        doubleSpinBox_A2->setMinimum(1.000000000000000);
        doubleSpinBox_A2->setMaximum(3.500000000000000);

        verticalLayout_2->addWidget(doubleSpinBox_A2);

        doubleSpinBox_A3 = new QDoubleSpinBox(tab);
        doubleSpinBox_A3->setObjectName(QString::fromUtf8("doubleSpinBox_A3"));
        doubleSpinBox_A3->setEnabled(false);
        doubleSpinBox_A3->setMinimum(1.000000000000000);
        doubleSpinBox_A3->setMaximum(3.500000000000000);

        verticalLayout_2->addWidget(doubleSpinBox_A3);

        pushButton_clearAll = new QPushButton(tab);
        pushButton_clearAll->setObjectName(QString::fromUtf8("pushButton_clearAll"));
        pushButton_clearAll->setEnabled(false);

        verticalLayout_2->addWidget(pushButton_clearAll);

        pushButton_trajectoryGeneration = new QPushButton(tab);
        pushButton_trajectoryGeneration->setObjectName(QString::fromUtf8("pushButton_trajectoryGeneration"));

        verticalLayout_2->addWidget(pushButton_trajectoryGeneration);

        pushButton_CollisionResponse = new QPushButton(tab);
        pushButton_CollisionResponse->setObjectName(QString::fromUtf8("pushButton_CollisionResponse"));

        verticalLayout_2->addWidget(pushButton_CollisionResponse);

        tabWidget->addTab(tab, QString());
        tab_2 = new QWidget();
        tab_2->setObjectName(QString::fromUtf8("tab_2"));
        verticalLayout_3 = new QVBoxLayout(tab_2);
        verticalLayout_3->setSpacing(6);
        verticalLayout_3->setContentsMargins(11, 11, 11, 11);
        verticalLayout_3->setObjectName(QString::fromUtf8("verticalLayout_3"));
        horizontalLayout_9 = new QHBoxLayout();
        horizontalLayout_9->setSpacing(6);
        horizontalLayout_9->setObjectName(QString::fromUtf8("horizontalLayout_9"));
        checkBox_showAABB = new QCheckBox(tab_2);
        checkBox_showAABB->setObjectName(QString::fromUtf8("checkBox_showAABB"));

        horizontalLayout_9->addWidget(checkBox_showAABB);


        verticalLayout_3->addLayout(horizontalLayout_9);

        pushButton_inputFourChambers = new QPushButton(tab_2);
        pushButton_inputFourChambers->setObjectName(QString::fromUtf8("pushButton_inputFourChambers"));

        verticalLayout_3->addWidget(pushButton_inputFourChambers);

        pushButton_inputMem = new QPushButton(tab_2);
        pushButton_inputMem->setObjectName(QString::fromUtf8("pushButton_inputMem"));

        verticalLayout_3->addWidget(pushButton_inputMem);

        pushButton_GenerateChamberTetMesh = new QPushButton(tab_2);
        pushButton_GenerateChamberTetMesh->setObjectName(QString::fromUtf8("pushButton_GenerateChamberTetMesh"));

        verticalLayout_3->addWidget(pushButton_GenerateChamberTetMesh);

        pushButton_showAABB_Tree = new QPushButton(tab_2);
        pushButton_showAABB_Tree->setObjectName(QString::fromUtf8("pushButton_showAABB_Tree"));

        verticalLayout_3->addWidget(pushButton_showAABB_Tree);

        line = new QFrame(tab_2);
        line->setObjectName(QString::fromUtf8("line"));
        line->setFrameShape(QFrame::HLine);
        line->setFrameShadow(QFrame::Sunken);

        verticalLayout_3->addWidget(line);

        doubleSpinBox_Chamber1 = new QDoubleSpinBox(tab_2);
        doubleSpinBox_Chamber1->setObjectName(QString::fromUtf8("doubleSpinBox_Chamber1"));
        doubleSpinBox_Chamber1->setMinimum(1.000000000000000);
        doubleSpinBox_Chamber1->setMaximum(100.000000000000000);
        doubleSpinBox_Chamber1->setValue(4.000000000000000);

        verticalLayout_3->addWidget(doubleSpinBox_Chamber1);

        doubleSpinBox_Chamber2 = new QDoubleSpinBox(tab_2);
        doubleSpinBox_Chamber2->setObjectName(QString::fromUtf8("doubleSpinBox_Chamber2"));
        doubleSpinBox_Chamber2->setMinimum(1.000000000000000);
        doubleSpinBox_Chamber2->setMaximum(100.000000000000000);
        doubleSpinBox_Chamber2->setValue(1.300000000000000);

        verticalLayout_3->addWidget(doubleSpinBox_Chamber2);

        doubleSpinBox_Chamber3 = new QDoubleSpinBox(tab_2);
        doubleSpinBox_Chamber3->setObjectName(QString::fromUtf8("doubleSpinBox_Chamber3"));
        doubleSpinBox_Chamber3->setMinimum(1.000000000000000);
        doubleSpinBox_Chamber3->setMaximum(100.000000000000000);
        doubleSpinBox_Chamber3->setValue(1.300000000000000);

        verticalLayout_3->addWidget(doubleSpinBox_Chamber3);

        doubleSpinBox_Chamber4 = new QDoubleSpinBox(tab_2);
        doubleSpinBox_Chamber4->setObjectName(QString::fromUtf8("doubleSpinBox_Chamber4"));
        doubleSpinBox_Chamber4->setMinimum(1.000000000000000);
        doubleSpinBox_Chamber4->setMaximum(100.000000000000000);
        doubleSpinBox_Chamber4->setValue(1.300000000000000);

        verticalLayout_3->addWidget(doubleSpinBox_Chamber4);

        pushButton_ChamberDeformation = new QPushButton(tab_2);
        pushButton_ChamberDeformation->setObjectName(QString::fromUtf8("pushButton_ChamberDeformation"));

        verticalLayout_3->addWidget(pushButton_ChamberDeformation);

        line_2 = new QFrame(tab_2);
        line_2->setObjectName(QString::fromUtf8("line_2"));
        line_2->setFrameShape(QFrame::HLine);
        line_2->setFrameShadow(QFrame::Sunken);

        verticalLayout_3->addWidget(line_2);

        label_2 = new QLabel(tab_2);
        label_2->setObjectName(QString::fromUtf8("label_2"));

        verticalLayout_3->addWidget(label_2);

        doubleSpinBox_ballMoveX = new QDoubleSpinBox(tab_2);
        doubleSpinBox_ballMoveX->setObjectName(QString::fromUtf8("doubleSpinBox_ballMoveX"));
        doubleSpinBox_ballMoveX->setMinimum(-100.000000000000000);
        doubleSpinBox_ballMoveX->setMaximum(100.000000000000000);
        doubleSpinBox_ballMoveX->setValue(80.000000000000000);

        verticalLayout_3->addWidget(doubleSpinBox_ballMoveX);

        doubleSpinBox_ballMoveY = new QDoubleSpinBox(tab_2);
        doubleSpinBox_ballMoveY->setObjectName(QString::fromUtf8("doubleSpinBox_ballMoveY"));
        doubleSpinBox_ballMoveY->setMinimum(-100.000000000000000);
        doubleSpinBox_ballMoveY->setMaximum(100.000000000000000);
        doubleSpinBox_ballMoveY->setValue(-60.000000000000000);

        verticalLayout_3->addWidget(doubleSpinBox_ballMoveY);

        doubleSpinBox_ballMoveZ = new QDoubleSpinBox(tab_2);
        doubleSpinBox_ballMoveZ->setObjectName(QString::fromUtf8("doubleSpinBox_ballMoveZ"));
        doubleSpinBox_ballMoveZ->setMinimum(-100.000000000000000);
        doubleSpinBox_ballMoveZ->setMaximum(100.000000000000000);
        doubleSpinBox_ballMoveZ->setValue(0.000000000000000);

        verticalLayout_3->addWidget(doubleSpinBox_ballMoveZ);

        pushButton_MoveBall = new QPushButton(tab_2);
        pushButton_MoveBall->setObjectName(QString::fromUtf8("pushButton_MoveBall"));

        verticalLayout_3->addWidget(pushButton_MoveBall);

        pushButton_CollisionChecking = new QPushButton(tab_2);
        pushButton_CollisionChecking->setObjectName(QString::fromUtf8("pushButton_CollisionChecking"));

        verticalLayout_3->addWidget(pushButton_CollisionChecking);

        verticalSpacer = new QSpacerItem(20, 142, QSizePolicy::Minimum, QSizePolicy::Expanding);

        verticalLayout_3->addItem(verticalSpacer);

        tabWidget->addTab(tab_2, QString());
        tab_3 = new QWidget();
        tab_3->setObjectName(QString::fromUtf8("tab_3"));
        verticalLayout = new QVBoxLayout(tab_3);
        verticalLayout->setSpacing(6);
        verticalLayout->setContentsMargins(11, 11, 11, 11);
        verticalLayout->setObjectName(QString::fromUtf8("verticalLayout"));
        label_3 = new QLabel(tab_3);
        label_3->setObjectName(QString::fromUtf8("label_3"));

        verticalLayout->addWidget(label_3);

        horizontalLayout_3 = new QHBoxLayout();
        horizontalLayout_3->setSpacing(6);
        horizontalLayout_3->setObjectName(QString::fromUtf8("horizontalLayout_3"));
        checkBox_onlyLeafNode = new QCheckBox(tab_3);
        checkBox_onlyLeafNode->setObjectName(QString::fromUtf8("checkBox_onlyLeafNode"));

        horizontalLayout_3->addWidget(checkBox_onlyLeafNode);

        horizontalSpacer = new QSpacerItem(78, 17, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout_3->addItem(horizontalSpacer);


        verticalLayout->addLayout(horizontalLayout_3);

        horizontalLayout_4 = new QHBoxLayout();
        horizontalLayout_4->setSpacing(6);
        horizontalLayout_4->setObjectName(QString::fromUtf8("horizontalLayout_4"));
        checkBox_overlappingNode = new QCheckBox(tab_3);
        checkBox_overlappingNode->setObjectName(QString::fromUtf8("checkBox_overlappingNode"));

        horizontalLayout_4->addWidget(checkBox_overlappingNode);

        horizontalSpacer_2 = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout_4->addItem(horizontalSpacer_2);


        verticalLayout->addLayout(horizontalLayout_4);

        horizontalLayout_5 = new QHBoxLayout();
        horizontalLayout_5->setSpacing(6);
        horizontalLayout_5->setObjectName(QString::fromUtf8("horizontalLayout_5"));
        horizontalSlider_AABBTree = new QSlider(tab_3);
        horizontalSlider_AABBTree->setObjectName(QString::fromUtf8("horizontalSlider_AABBTree"));
        horizontalSlider_AABBTree->setOrientation(Qt::Horizontal);

        horizontalLayout_5->addWidget(horizontalSlider_AABBTree);

        lcdNumber_AABBTree = new QLCDNumber(tab_3);
        lcdNumber_AABBTree->setObjectName(QString::fromUtf8("lcdNumber_AABBTree"));

        horizontalLayout_5->addWidget(lcdNumber_AABBTree);


        verticalLayout->addLayout(horizontalLayout_5);

        line_3 = new QFrame(tab_3);
        line_3->setObjectName(QString::fromUtf8("line_3"));
        line_3->setFrameShape(QFrame::HLine);
        line_3->setFrameShadow(QFrame::Sunken);

        verticalLayout->addWidget(line_3);

        label_4 = new QLabel(tab_3);
        label_4->setObjectName(QString::fromUtf8("label_4"));

        verticalLayout->addWidget(label_4);

        horizontalLayout_6 = new QHBoxLayout();
        horizontalLayout_6->setSpacing(6);
        horizontalLayout_6->setObjectName(QString::fromUtf8("horizontalLayout_6"));
        lineEdit_bodyOutputName = new QLineEdit(tab_3);
        lineEdit_bodyOutputName->setObjectName(QString::fromUtf8("lineEdit_bodyOutputName"));

        horizontalLayout_6->addWidget(lineEdit_bodyOutputName);

        pushButton_outputBody = new QPushButton(tab_3);
        pushButton_outputBody->setObjectName(QString::fromUtf8("pushButton_outputBody"));

        horizontalLayout_6->addWidget(pushButton_outputBody);


        verticalLayout->addLayout(horizontalLayout_6);

        horizontalLayout_7 = new QHBoxLayout();
        horizontalLayout_7->setSpacing(6);
        horizontalLayout_7->setObjectName(QString::fromUtf8("horizontalLayout_7"));
        lineEdit_chamberOutputName = new QLineEdit(tab_3);
        lineEdit_chamberOutputName->setObjectName(QString::fromUtf8("lineEdit_chamberOutputName"));

        horizontalLayout_7->addWidget(lineEdit_chamberOutputName);

        pushButton_outputChamber = new QPushButton(tab_3);
        pushButton_outputChamber->setObjectName(QString::fromUtf8("pushButton_outputChamber"));

        horizontalLayout_7->addWidget(pushButton_outputChamber);


        verticalLayout->addLayout(horizontalLayout_7);

        horizontalLayout_8 = new QHBoxLayout();
        horizontalLayout_8->setSpacing(6);
        horizontalLayout_8->setObjectName(QString::fromUtf8("horizontalLayout_8"));
        lineEdit_tetOutputName = new QLineEdit(tab_3);
        lineEdit_tetOutputName->setObjectName(QString::fromUtf8("lineEdit_tetOutputName"));

        horizontalLayout_8->addWidget(lineEdit_tetOutputName);

        pushButton_outputTet = new QPushButton(tab_3);
        pushButton_outputTet->setObjectName(QString::fromUtf8("pushButton_outputTet"));

        horizontalLayout_8->addWidget(pushButton_outputTet);


        verticalLayout->addLayout(horizontalLayout_8);

        verticalSpacer_3 = new QSpacerItem(20, 232, QSizePolicy::Minimum, QSizePolicy::Expanding);

        verticalLayout->addItem(verticalSpacer_3);

        tabWidget->addTab(tab_3, QString());
        tab_4 = new QWidget();
        tab_4->setObjectName(QString::fromUtf8("tab_4"));
        verticalLayout_4 = new QVBoxLayout(tab_4);
        verticalLayout_4->setSpacing(6);
        verticalLayout_4->setContentsMargins(11, 11, 11, 11);
        verticalLayout_4->setObjectName(QString::fromUtf8("verticalLayout_4"));
        verticalSpacer_4 = new QSpacerItem(20, 454, QSizePolicy::Minimum, QSizePolicy::Expanding);

        verticalLayout_4->addItem(verticalSpacer_4);

        tabWidget->addTab(tab_4, QString());

        gridLayout->addWidget(tabWidget, 1, 0, 1, 1);

        dockWidget->setWidget(dockWidgetContents);
        MainWindow->addDockWidget(static_cast<Qt::DockWidgetArea>(2), dockWidget);
        menuBar = new QMenuBar(MainWindow);
        menuBar->setObjectName(QString::fromUtf8("menuBar"));
        menuBar->setGeometry(QRect(0, 0, 1360, 21));
        menuBar->setLayoutDirection(Qt::LeftToRight);
        menuFile = new QMenu(menuBar);
        menuFile->setObjectName(QString::fromUtf8("menuFile"));
        menuView = new QMenu(menuBar);
        menuView->setObjectName(QString::fromUtf8("menuView"));
        menuSelect = new QMenu(menuBar);
        menuSelect->setObjectName(QString::fromUtf8("menuSelect"));
        MainWindow->setMenuBar(menuBar);

        navigationToolBar->addAction(actionFront);
        navigationToolBar->addAction(actionBack);
        navigationToolBar->addAction(actionTop);
        navigationToolBar->addAction(actionBottom);
        navigationToolBar->addAction(actionLeft);
        navigationToolBar->addAction(actionRight);
        navigationToolBar->addAction(actionIsometric);
        navigationToolBar->addSeparator();
        navigationToolBar->addAction(actionZoom_In);
        navigationToolBar->addAction(actionZoom_Out);
        navigationToolBar->addAction(actionZoom_All);
        navigationToolBar->addAction(actionZoom_Window);
        navigationToolBar->addSeparator();
        navigationToolBar->addAction(actionShade);
        navigationToolBar->addAction(actionMesh);
        navigationToolBar->addAction(actionNode);
        navigationToolBar->addAction(actionProfile);
        navigationToolBar->addAction(actionFaceNormal);
        navigationToolBar->addAction(actionNodeNormal);
        toolBar->addAction(actionOpen);
        toolBar->addAction(actionSave);
        toolBar->addSeparator();
        selectionToolBar->addSeparator();
        selectionToolBar->addAction(actionSelectNode);
        selectionToolBar->addAction(actionSelectEdge);
        selectionToolBar->addAction(actionSelectFace);
        menuBar->addAction(menuFile->menuAction());
        menuBar->addAction(menuView->menuAction());
        menuBar->addAction(menuSelect->menuAction());
        menuFile->addAction(actionOpen);
        menuFile->addAction(actionSave);
        menuView->addAction(actionFront);
        menuView->addAction(actionBack);
        menuView->addAction(actionTop);
        menuView->addAction(actionBottom);
        menuView->addAction(actionLeft);
        menuView->addAction(actionRight);
        menuView->addAction(actionIsometric);
        menuView->addSeparator();
        menuView->addAction(actionZoom_In);
        menuView->addAction(actionZoom_Out);
        menuView->addAction(actionZoom_All);
        menuView->addAction(actionZoom_Window);
        menuView->addSeparator();
        menuView->addAction(actionShade);
        menuView->addAction(actionMesh);
        menuView->addAction(actionNode);
        menuView->addAction(actionProfile);
        menuView->addSeparator();
        menuView->addAction(actionShifttoOrigin);
        menuSelect->addAction(actionSelectNode);
        menuSelect->addAction(actionSelectEdge);
        menuSelect->addAction(actionSelectFace);

        retranslateUi(MainWindow);

        tabWidget->setCurrentIndex(1);


        QMetaObject::connectSlotsByName(MainWindow);
    } // setupUi

    void retranslateUi(QMainWindow *MainWindow)
    {
        MainWindow->setWindowTitle(QApplication::translate("MainWindow", "MainWindow", nullptr));
        actionOpen->setText(QApplication::translate("MainWindow", "Open", nullptr));
        actionFront->setText(QApplication::translate("MainWindow", "Front", nullptr));
        actionBack->setText(QApplication::translate("MainWindow", "Back", nullptr));
        actionTop->setText(QApplication::translate("MainWindow", "Top", nullptr));
        actionBottom->setText(QApplication::translate("MainWindow", "Bottom", nullptr));
        actionLeft->setText(QApplication::translate("MainWindow", "Left", nullptr));
        actionRight->setText(QApplication::translate("MainWindow", "Right", nullptr));
        actionIsometric->setText(QApplication::translate("MainWindow", "Isometric", nullptr));
        actionZoom_In->setText(QApplication::translate("MainWindow", "Zoom In", nullptr));
        actionZoom_Out->setText(QApplication::translate("MainWindow", "Zoom Out", nullptr));
        actionZoom_All->setText(QApplication::translate("MainWindow", "Zoom All", nullptr));
        actionZoom_Window->setText(QApplication::translate("MainWindow", "Zoom Window", nullptr));
        actionShade->setText(QApplication::translate("MainWindow", "Shade", nullptr));
        actionMesh->setText(QApplication::translate("MainWindow", "Mesh", nullptr));
        actionNode->setText(QApplication::translate("MainWindow", "Node", nullptr));
        actionSave->setText(QApplication::translate("MainWindow", "Save", nullptr));
        actionSelectNode->setText(QApplication::translate("MainWindow", "Node", nullptr));
        actionSelectFace->setText(QApplication::translate("MainWindow", "Face", nullptr));
        actionShifttoOrigin->setText(QApplication::translate("MainWindow", "Shift to Origin", nullptr));
        actionProfile->setText(QApplication::translate("MainWindow", "Profile", nullptr));
        actionFaceNormal->setText(QApplication::translate("MainWindow", "FaceNormal", nullptr));
        actionNodeNormal->setText(QApplication::translate("MainWindow", "NodeNormal", nullptr));
        actionSelectEdge->setText(QApplication::translate("MainWindow", "Edge", nullptr));
        actionGenerate->setText(QApplication::translate("MainWindow", "Generate", nullptr));
        actionTest_1->setText(QApplication::translate("MainWindow", "Test_1", nullptr));
        navigationToolBar->setWindowTitle(QApplication::translate("MainWindow", "navigationToolBar", nullptr));
        toolBar->setWindowTitle(QApplication::translate("MainWindow", "toolBar", nullptr));
        selectionToolBar->setWindowTitle(QApplication::translate("MainWindow", "selectionToolBar", nullptr));
        checkBox_combinedInput->setText(QApplication::translate("MainWindow", "Combined Input", nullptr));
        label->setText(QApplication::translate("MainWindow", "IterTime", nullptr));
        comboBox_planeDir->setItemText(0, QApplication::translate("MainWindow", "X", nullptr));
        comboBox_planeDir->setItemText(1, QApplication::translate("MainWindow", "Y", nullptr));
        comboBox_planeDir->setItemText(2, QApplication::translate("MainWindow", "Z", nullptr));

        checkBox_readChamberRegion->setText(QApplication::translate("MainWindow", "Read Chamber Region", nullptr));
        pushButton_generateTETMesh->setText(QApplication::translate("MainWindow", "TET Mesh Generation", nullptr));
        pushButton_runCollisionChecking->setText(QApplication::translate("MainWindow", "Collision Response Testing", nullptr));
        pushButton_clearAll->setText(QApplication::translate("MainWindow", "Clear All", nullptr));
        pushButton_trajectoryGeneration->setText(QApplication::translate("MainWindow", "trajectory Generation", nullptr));
        pushButton_CollisionResponse->setText(QApplication::translate("MainWindow", "...", nullptr));
        tabWidget->setTabText(tabWidget->indexOf(tab), QApplication::translate("MainWindow", "Pneumatic Actuation", nullptr));
        checkBox_showAABB->setText(QApplication::translate("MainWindow", "Enable AABB Tree", nullptr));
        pushButton_inputFourChambers->setText(QApplication::translate("MainWindow", "Input Finger", nullptr));
        pushButton_inputMem->setText(QApplication::translate("MainWindow", "Input Obstacle", nullptr));
        pushButton_GenerateChamberTetMesh->setText(QApplication::translate("MainWindow", "Generate Tet Mesh", nullptr));
        pushButton_showAABB_Tree->setText(QApplication::translate("MainWindow", "Update AABB Tree", nullptr));
        pushButton_ChamberDeformation->setText(QApplication::translate("MainWindow", "Chamber Deformation", nullptr));
        label_2->setText(QApplication::translate("MainWindow", " Move Obstacle", nullptr));
        pushButton_MoveBall->setText(QApplication::translate("MainWindow", " Move Obstacle", nullptr));
        pushButton_CollisionChecking->setText(QApplication::translate("MainWindow", "Collision Checking Response", nullptr));
        tabWidget->setTabText(tabWidget->indexOf(tab_2), QApplication::translate("MainWindow", "Soft Finger Demo", nullptr));
        label_3->setText(QApplication::translate("MainWindow", "AABB Tree Visualizer", nullptr));
        checkBox_onlyLeafNode->setText(QApplication::translate("MainWindow", "Mode: Only Leaf Node", nullptr));
        checkBox_overlappingNode->setText(QApplication::translate("MainWindow", "Mode: Overlapping Depth", nullptr));
        label_4->setText(QApplication::translate("MainWindow", "Output Deformed Mesh", nullptr));
        pushButton_outputBody->setText(QApplication::translate("MainWindow", "Output Body", nullptr));
        pushButton_outputChamber->setText(QApplication::translate("MainWindow", "Output Chamber", nullptr));
        pushButton_outputTet->setText(QApplication::translate("MainWindow", "Output Tet", nullptr));
        tabWidget->setTabText(tabWidget->indexOf(tab_3), QApplication::translate("MainWindow", "Visualization Setting", nullptr));
        tabWidget->setTabText(tabWidget->indexOf(tab_4), QApplication::translate("MainWindow", "Output Setting", nullptr));
        menuFile->setTitle(QApplication::translate("MainWindow", "File", nullptr));
        menuView->setTitle(QApplication::translate("MainWindow", "View", nullptr));
        menuSelect->setTitle(QApplication::translate("MainWindow", "Select", nullptr));
    } // retranslateUi

};

namespace Ui {
    class MainWindow: public Ui_MainWindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAINWINDOW_H

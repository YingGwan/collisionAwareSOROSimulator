#ifndef POLYGENMESH_H
#define POLYGENMESH_H

#include <string>

#include "../GLKLib/GLKLib.h"
#include "../GLKLib/GLKObList.h"

#include "../QMeshLib/QMeshPatch.h"
#include "../QMeshLib/QMeshFace.h"
#include "../QMeshLib/QMeshEdge.h"
#include "../QMeshLib/QMeshNode.h"
#include "../QMeshLib/QMeshTetra.h"


class PolygenMesh : public GLKEntity
{
public:
    PolygenMesh();
    ~PolygenMesh();

    void ImportOBJFile(char *filename, std::string modelName);
	void ImportTETFile(char *filename, std::string modelName);

    virtual void DeleteGLList();
    virtual void BuildGLList(bool bVertexNormalShading);

    virtual void drawShade();
    virtual void drawMesh();
    virtual void drawNode();
    virtual void drawProfile();
    virtual void drawFaceNormal();
    virtual void drawNodeNormal();
    virtual float getRange() {return m_range;}

    void drawBox(float xx, float yy, float zz, float r);

    void ClearAll();
    void computeRange();
    GLKObList &GetMeshList() {return meshList;};
    void CompBoundingBox(double boundingBox[]);

    int m_materialTypeNum;
    bool m_bVertexNormalShading;

    Eigen::Vector3d tipPos;

public:
    GLKObList meshList;
    float m_range;
    int m_drawListID;
    int m_drawListNumber;

    void _buildDrawShadeList(bool bVertexNormalShading);
    void _buildDrawMeshList();
    void _buildDrawNodeList();
    void _buildDrawProfileList();
    void _buildDrawFaceNormalList();
    void _buildDrawNodeNormalList();
//    void _buildDrawPreMeshList();
    void _changeValueToColor(int nType, float & nRed, float & nGreen, float & nBlue);
    void _changeValueToColor(double maxValue, double minValue, double Value,
                                 float & nRed, float & nGreen, float & nBlue);

    void setModelName(std::string name) {modelName=name;};
    void setTransparent() {isTransparent=true;};
    void resetTransparent() { isTransparent = false; };
    void setEdgeColor() {edgeColor=true;};
    std::string getModelName() {return modelName;};

    Eigen::MatrixXd trajPos;
    bool isVolume = false;

    //visualization
    //@drawIdx: 0 -> body
    //          1 -> chamber
    //          2 -> tet 

    //          4 -> AABB Bounding box
  

    int drawIdx = -1;       


    //final version of visualization
    //@drawIdx:  0 -> body surface mesh
    //           1 -> chamber surface mesh
    //           2 -> 
    //           3 -> tet mesh

    //           4 -> AABB Tree Bounding Box 

    //          11 -> both self-collision and collision with env: correspondence

    int paintIdx = -1;


    //bounding box drawing
    int draw_depth_idx = -1;        //if this value is -1: we draw all depth of bounding box
                                    //if this value is bigger than -1, then draw the specific depth of bounding box
    bool _drawAllNode = true;       //true  -> draw all nodes including internal nodes and leaf nodes
    bool _drawOverlapping = false;  //false -> only single depth will be drawn


private:
    std::string modelName;
    bool isTransparent=false;
    bool edgeColor=false;
};

#endif // POLYGENMESH_H

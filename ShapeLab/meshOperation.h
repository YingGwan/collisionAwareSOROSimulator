#pragma once

#include "PolygenMesh.h"

class meshOperation
{

public:
	meshOperation() {};
	~meshOperation() {};

	void seperateMesh(QMeshPatch* inputMesh, QMeshPatch* body, QMeshPatch* chamber);

	void tetMeshGeneration_singleSurfaceMesh(
		QMeshPatch* inputMesh, QMeshPatch* outputMesh, std::string tetgenCommand);
	
	void tetMeshGeneration_outerSkin_Chamber(
		QMeshPatch* skin, QMeshPatch* chamber, QMeshPatch* outputMesh, std::string tetgenCommand);

	void tetMeshReading(QMeshPatch* outputMesh, int chamberIdx);


	void tetMeshGeneration_outerSkinProtection_ChamberRemesh(
		QMeshPatch* skin, QMeshPatch* chamber, QMeshPatch* tetMesh, std::string tetgenCommand);

	void tetMeshGeneration_outerSkinProtection_ChamberRemesh_New(
		QMeshPatch* skin, QMeshPatch* chamber, QMeshPatch* tetMesh, std::string tetgenCommand);
	
	void MannequinTetMeshOutput(QMeshPatch *mesh, int chamberIdx);
	void MannequinTetMeshSelectionOutput(QMeshPatch *mesh, int chamberIdx);

	void chamberSelection(QMeshPatch* tetMesh, QMeshPatch* chamber, bool readFile=false);
	void chamberSelection_PQP(QMeshPatch* tetMesh, QMeshPatch* chamber, bool checkMode);
	void readChamberSelection(QMeshPatch* tetMesh, int chamberIdx);

	void buildTopologyConnection_chamberSelection_FAST(
		QMeshPatch* tetMesh, QMeshPatch* body, QMeshPatch* chamber, bool updateConnection);

	void buildTopologyConnection(QMeshPatch* tetMesh, QMeshPatch* body_init, QMeshPatch* chamber_init);

	void outputSimulationResult(
		QMeshPatch* tetMesh, QMeshPatch* body, QMeshPatch* chamber,
		std::string modelName, int iter, bool outputTET, bool collRespond);

	void topologyCheck(QMeshPatch* tetMesh);


	void updateOBJMesh_chamber(QMeshPatch* tetMesh, QMeshPatch* chamber);
	void updateOBJMesh_chamber(QMeshPatch* chamber);

	void updateOBJMesh_skin(QMeshPatch* tetMesh, QMeshPatch* skin);
	void updateOBJMesh_skin(QMeshPatch* skin);
	void updateOBJMesh_chamber_skin(QMeshPatch* chamber, QMeshPatch* skin);


	double chamberVolumeEvaluation(QMeshPatch* tetMesh);
	void selectShift_rigidRegion(QMeshPatch* surfaceMesh, std::string modelName);
	void selectShift_rigidRegion(QMeshPatch* tetMesh, QMeshPatch* surfaceMesh);

	void saveRigidRegion(
		QMeshPatch* tetMesh, std::vector<Eigen::Vector3d> &rigidNodePosSet, bool bodyProtected);

	void loadRigidRegion(QMeshPatch* tetMesh, std::vector<Eigen::Vector3d>& rigidNodePosSet);
	void materialSpaceSelection_rigid(QMeshPatch* tetMesh, std::string modelName);

	void _combineTwoSurfaceMesh(
		QMeshPatch* skin, QMeshPatch* chamber, QMeshPatch* combinedMesh);

	void laplacianSmoothSurface(QMeshPatch* chamber, QMeshPatch* tetMesh);

private:
	

	bool _IntersectTriangle(const Eigen::Vector3d& orig, const Eigen::Vector3d& dir,
		Eigen::Vector3d& v0, Eigen::Vector3d& v1, Eigen::Vector3d& v2);

	bool _calculatePointInsideMesh(QMeshPatch* target_mesh, Eigen::Vector3d& orig);
	int _segementMesh_withFlag(QMeshPatch* inputMesh);
	void _generateNewMeshPart(QMeshPatch* inputMesh, QMeshPatch* newMesh, int partIndex);

	bool _detectSegmentationOrder(QMeshPatch* inputMesh);
	void _updateNodeState(QMeshPatch* tetMesh);

	void _generateTetMesh_keepBodyEle_ChangeChamber(QMeshPatch* tetMesh, QMeshPatch* newTetMesh);

};


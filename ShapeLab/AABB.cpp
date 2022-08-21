#include"AABB.h"
#define TetIter QMeshTetra* 

AABBManager::AABBManager()
{

}


AABBManager::~AABBManager()
{
	//free memory
	if (memoryAllocated__TetraPtrArray_obstacle == true)
	{
		delete[] TetraPtrArray_obstacle;
	}
	if (memoryAllocated__obstacleTree == true)
	{
		delete[] _obstacleTree->nodes;
		delete[] _obstacleTree;
	}


	//free memory
	if (_treeInit == true)
	{
		delete[] TetraPtrArray;
	}
	if (_treeInit == true)
	{
		delete[] _tree->nodes;
		delete[] _tree;
	}
}

//////////////////////////////////////////////////////////
///////Print Bounding Box Information/////////////////////
//////////////////////////////////////////////////////////
void AABBManager::_printBoundingBoxInfo(std::string _name, AABB _box)
{
	printf("Bounding box of %s is (%lf %lf %lf), (%lf, %lf, %lf)\n",
		_name.c_str(), _box.lowerBound[0], _box.lowerBound[1], _box.lowerBound[2], _box.upperBound[0], _box.upperBound[1], _box.upperBound[2]);
}

//////////////////////////////////////////////////////////
///////AABB Bounding Box Union////////////////////////////
//////////////////////////////////////////////////////////
AABB AABBManager::Union(AABB A, AABB B)
{
	AABB C;
	for (int i = 0; i < 3; i++)
	{
		C.lowerBound[i] = MIN(A.lowerBound[i], B.lowerBound[i]);
		C.upperBound[i] = MAX(A.upperBound[i], B.upperBound[i]);
	}

	return C;
}


//////////////////////////////////////////////////////////
///////AABB Insertion Box Union///////////////////////////
//////////////////////////////////////////////////////////
//Insert node containing _ele to the _nodeIdx's left 
//or right child. Specially, when the _flag = 0, then 
//we insert it into the root node

//There should be one var describing whether it contains only one tetrahedron
//Also the merge of AABB should be included
//para: @_nodeIdx: parent node idx. 
//		@_flag:  0 -> root node; 1 -> left child; 2 -> right child
//		@_ele:	 containment in child node 
void AABBManager::_insert2Tree(int _flag,int _nodeIdx,AABB _ele)
{
	if (_flag == 0)
	{
		_tree->rootIndex = 0;
		_tree->nodes[0].box = _ele;
		_tree->nodes[0].isLeaf = false;
		_tree->nodeCount = 1;
		_tree->nodes[0].collisionBox = _ele;
		_tree->nodes[0].depth = 0;

		//_tree->nodes[0].tetraIndexList
		
		
		for (int i = 0; i < TetraPtrArrayLen; i++)
		{
			//travel all points to update the minimal x, y and z
			//							  the maximum x, y and z
			_tree->nodes[0].tetraList.push_back(TetraPtrArray[i]);
		}

	}
	else {
		//not used for now
		int insertIdx = _tree->nodeCount;
		_tree->nodes[insertIdx].box = _ele;							

		if (_flag == 1)
			_tree->nodes[_nodeIdx].child1 = insertIdx;		//this newly inserted node is left child
		else if (_flag == 2)
			_tree->nodes[_nodeIdx].child2 = insertIdx;		//this newly inserted node is right child
			
		_tree->nodeCount++;
	}
}



//Split the bounding box to be two parts by cutting the longest dimension
//para: @_old: old boundingbox to be split
//		@left:  contain the new bounding box that contains the small split portion
//		@right:	contain the new bounding box that contains the large split portion

void AABBManager::splitBoundingBox(AABB _old, AABB* left, AABB* right)
{
	double len[3],longest;
	int flag = -1;			//@flag: 0 -> x axis being split; 1 -> y; 2 -> z.
	for(int i=0;i<3;i++)
		len[i] = -_old.lowerBound[i] + _old.upperBound[i];
	
	if (len[0] > len[1]){flag = 0;longest = len[0];}
	else{flag = 1;longest = len[1];}

	if (longest > len[2]){}
	else {longest = len[2];flag = 2;}
	
	//printf("longest is %d, and value is %lf\n",flag, longest);
	double splitPoint = (_old.upperBound[flag] + _old.lowerBound[flag]) / 2.0;

	for (int i = 0; i < 3; i++)
	{
		if (i == flag)
		{
			//if this is the split edge
			left->lowerBound[i] = _old.lowerBound[i];
			left->upperBound[i] = splitPoint;

			right->lowerBound[i] = splitPoint;
			right->upperBound[i] = _old.upperBound[i];
		}
		else 
		{
			//if this is not the split edge
			
			left->lowerBound[i] = _old.lowerBound[i];
			left->upperBound[i] = _old.upperBound[i];
				
			right->lowerBound[i] = _old.lowerBound[i];
			right->upperBound[i] = _old.upperBound[i];

		}
	}


	//printf("left:  lower [%lf, %lf, %lf] -> upper [%lf, %lf, %lf]\n", left->lowerBound[0], left->lowerBound[1], left->lowerBound[2], left->upperBound[0], left->upperBound[1], left->upperBound[2]);
	//printf("right: lower [%lf, %lf, %lf] -> upper [%lf, %lf, %lf]\n", right->lowerBound[0], right->lowerBound[1], right->lowerBound[2], right->upperBound[0], right->upperBound[1], right->upperBound[2]);

}


//Recursively construct AABB tree
//given the known bounding box, it will split the largest dimension

//para: @_nodeIdx: parent node idx. 
//		@_flag:  1 -> left child; 2 -> right child
//		@_oldBoundingBox:	newly split bounding box, we need to update this bounding box according to all tetra whose center is within _oldBoundingBox
void AABBManager::_recursiveConstruction(int _nodeIdx, int _flag, AABB _old, int _recursiveLayer)
{

	//update the bounding box of _old
	long startTime = clock();
	int _insertIdx = _tree->nodeCount;
	AABB _oldCollisionBox;
	for (int i = 0; i < 3; i++)
	{
		_oldCollisionBox.lowerBound[i] = INF;
		_oldCollisionBox.upperBound[i] = NINF;
	}
	
	double pp[3];
	int _inOldBoxNum = 0;
	int _objectIdx = -1;

	//version 1: search from all tetrahedron
	//the speed here is too slow
	//we just need to search from the parent's containment
	//for (int i = 0; i < TetraPtrArrayLen; i++)
	//{
	//	//travel all points to update the minimal x, y and z
	//	//							  the maximum x, y and z
	//	QMeshTetra* tet = TetraPtrArray[i];
	//	pp[0] = tet->_center[0];
	//	pp[1] = tet->_center[1];
	//	pp[2] = tet->_center[2];
	//	//if tet center is within _old, then update the bounding box including this tet
	//	
	//	if ((pp[0] > _old.lowerBound[0] && pp[0] < _old.upperBound[0])&& (pp[1] > _old.lowerBound[1] && pp[1] < _old.upperBound[1])&& (pp[2] > _old.lowerBound[2] && pp[2] < _old.upperBound[2]) )
	//	{
	//		//within _old bounding box
	//		_inOldBoxNum++;
	//		_objectIdx = i;
	//		//update bounding box
	//		for (int j = 0; j < 3; j++)
	//		{
	//			if (tet->_lowerBounding[j] < _oldCollisionBox.lowerBound[j])
	//			{
	//				_oldCollisionBox.lowerBound[j] = tet->_lowerBounding[j];
	//			}
	//			if (tet->_upperBounding[j] > _oldCollisionBox.upperBound[j])
	//			{
	//				_oldCollisionBox.upperBound[j] = tet->_upperBounding[j];
	//			}
	//		}
	//	}
	//}
	
	//version 2: search from all tetrahedron from parent (i.e., travel  _tree->nodes[_nodeIdx].tetraList)
	
	for (TetIter tet: (_tree->nodes[_nodeIdx].tetraList))
	{
		pp[0] = tet->_center[0];
		pp[1] = tet->_center[1];
		pp[2] = tet->_center[2];
		
		if ((pp[0] > _old.lowerBound[0] && pp[0] < _old.upperBound[0]) && (pp[1] > _old.lowerBound[1] && pp[1] < _old.upperBound[1]) && (pp[2] > _old.lowerBound[2] && pp[2] < _old.upperBound[2]))
		{
			//within _old bounding box
			_tree->nodes[_insertIdx].tetraList.push_back(tet);					//update this AABB Tree node's containment list
			_inOldBoxNum++;
			_objectIdx = tet->_idx;
			//update bounding box
			for (int j = 0; j < 3; j++)
			{
				if (tet->_lowerBounding[j] < _oldCollisionBox.lowerBound[j])
				{
					_oldCollisionBox.lowerBound[j] = tet->_lowerBounding[j];
				}

				if (tet->_upperBounding[j] > _oldCollisionBox.upperBound[j])
				{
					_oldCollisionBox.upperBound[j] = tet->_upperBounding[j];
				}
			}
		}

	}
	//for (int i = 0; i < TetraPtrArrayLen; i++)
	//{
	//	//travel all points to update the minimal x, y and z
	//	//							  the maximum x, y and z
	//	_tree->nodes[0].tetraList.push_back(TetraPtrArray[i]);
	//}



	long endTime = clock();
	long Time1 = endTime - startTime;
	//ending criterion: check whether this node is the leef node and insert it into AABB Tree.
	//update the insert index of current node
	
	if (_inOldBoxNum == 1)
	{
		//leaf node that contains only one tetrahedron whose idx is objectIndex.
		//here we end the recursive function
		_tree->nodes[_insertIdx].isLeaf = true;
		_tree->nodes[_insertIdx].parentIndex = _nodeIdx;
		_tree->nodes[_insertIdx].box = _old;
		_tree->nodes[_insertIdx].collisionBox = _oldCollisionBox;

		_tree->nodes[_insertIdx].objectIndex = _objectIdx;
		_tree->nodes[_insertIdx].depth = _recursiveLayer;
		_tree->nodeCount++;

		if (_flag == 1)
			_tree->nodes[_nodeIdx].child1 = _insertIdx;		//this newly inserted node is left child
		else if (_flag == 2)
			_tree->nodes[_nodeIdx].child2 = _insertIdx;		//this newly inserted node is right child

		if (_recursiveLayer > _tree->depth)
			_tree->depth = _recursiveLayer;
		//printf("Reach leaf node: in old box num is %d\n",1);
		return;

	}
	else if (_inOldBoxNum == 0)
	{
		//leaf that dosen't contain any tetrahedron because the highly distorted tetrahedron this time
		//here we end the recursive function
		//use _old to be This AABB 
		_tree->nodes[_insertIdx].isLeaf = true;
		_tree->nodes[_insertIdx].parentIndex = _nodeIdx;
		_tree->nodes[_insertIdx].box = _old;
		_tree->nodes[_insertIdx].collisionBox = _old;
		_tree->nodes[_insertIdx].objectIndex = -1;
		_tree->nodes[_insertIdx].depth = _recursiveLayer;
		_tree->nodeCount++;

		if (_flag == 1)
			_tree->nodes[_nodeIdx].child1 = _insertIdx;		//this newly inserted node is left child
		else if (_flag == 2)
			_tree->nodes[_nodeIdx].child2 = _insertIdx;		//this newly inserted node is right child

		if (_recursiveLayer > _tree->depth)
			_tree->depth = _recursiveLayer;
		//printf("Reach leaf node: in old box num is %d\n", 0);
		return;
	}
	else {
		//leaf node that contain at leat two tets, we continue the recursive function
		_tree->nodes[_insertIdx].isLeaf = false;
		_tree->nodes[_insertIdx].parentIndex = _nodeIdx;
		_tree->nodes[_insertIdx].box = _old;
		_tree->nodes[_insertIdx].collisionBox = _oldCollisionBox;
		_tree->nodes[_insertIdx].objectIndex = -2;			//this var dosent matter here
		_tree->nodes[_insertIdx].depth = _recursiveLayer;
		_tree->nodeCount++;

		if (_flag == 1)
			_tree->nodes[_nodeIdx].child1 = _insertIdx;		//this newly inserted node is left child
		else if (_flag == 2)
			_tree->nodes[_nodeIdx].child2 = _insertIdx;		//this newly inserted node is right child

	}
	/*if (_recursiveLayer == 10)
		return;*/
	//normal execution
	AABB left, right;
	splitBoundingBox(_old, &left, &right);
	/*if (_flag == 1)
	{
		counter[_recursiveLayer]++;
		if (counter[_recursiveLayer] == 1)
		{

			printf("\nLeft Layer: %d\n", _recursiveLayer);
			_printBoundingBoxInfo(to_string(_recursiveLayer), _old);
			_printBoundingBoxInfo("Next would be:", left);
			
		}
			
	}*/
	long Time2 = (long)clock() - startTime;
	//printf("part is taking %3.3lf percent\n",(double)Time1/ (double)Time2 *100.0);

	_recursiveConstruction(_insertIdx, 1, left, _recursiveLayer+1);
	_recursiveConstruction(_insertIdx, 2, right, _recursiveLayer+1);
	
	//debug output
	/*if (_flag == 1)
		printf("left: [%lf, %lf, %lf] -> [%lf, %lf, %lf]\n", _old.lowerBound[0], _old.lowerBound[1], _old.lowerBound[2], _old.upperBound[0], _old.upperBound[1], _old.upperBound[2]);
	if (_flag == 2)
		printf("right: [%lf, %lf, %lf] -> [%lf, %lf, %lf]\n", _old.lowerBound[0], _old.lowerBound[1], _old.lowerBound[2], _old.upperBound[0], _old.upperBound[1], _old.upperBound[2]);*/

}


void AABBManager::_addSingleAABBboundingBox_debugQuery(AABB _aabb, QMeshPatch* _patch, int _depthIdx)
{
	
	static int edgePair[24] = { 0,1,1,2,2,3,3,0,4,5,5,6,6,7,7,4,0,4,1,5,2,6,3,7 };		//bounding box edges from point indices
	AABB box = _aabb;
	int startIndex = _patch->GetNodeNumber();
	QMeshNode* node = new QMeshNode;
	node->SetCoord3D(box.lowerBound[0], box.lowerBound[1], box.lowerBound[2]); _patch->GetNodeList().AddTail(node);
	node = new QMeshNode;
	node->SetCoord3D(box.upperBound[0], box.lowerBound[1], box.lowerBound[2]); _patch->GetNodeList().AddTail(node);
	node = new QMeshNode;
	node->SetCoord3D(box.upperBound[0], box.upperBound[1], box.lowerBound[2]); _patch->GetNodeList().AddTail(node);
	node = new QMeshNode;
	node->SetCoord3D(box.lowerBound[0], box.upperBound[1], box.lowerBound[2]); _patch->GetNodeList().AddTail(node);

	node = new QMeshNode;
	node->SetCoord3D(box.lowerBound[0], box.lowerBound[1], box.upperBound[2]); _patch->GetNodeList().AddTail(node);
	node = new QMeshNode;
	node->SetCoord3D(box.upperBound[0], box.lowerBound[1], box.upperBound[2]); _patch->GetNodeList().AddTail(node);
	node = new QMeshNode;
	node->SetCoord3D(box.upperBound[0], box.upperBound[1], box.upperBound[2]); _patch->GetNodeList().AddTail(node);
	node = new QMeshNode;
	node->SetCoord3D(box.lowerBound[0], box.upperBound[1], box.upperBound[2]);
	GLKPOSITION temp = _patch->GetNodeList().AddTail(node);


	for (int i = 0; i < 12; i++)
	{
		int EdgeStartNodeIdx = edgePair[i * 2 + 0];
		int EdgeEndNodeIdx = edgePair[i * 2 + 1];

		QMeshEdge* edge = new QMeshEdge;

		//qDebug("test1");
		GLKPOSITION STARTPOS = _patch->GetNodeList().FindIndexFrom(startIndex + EdgeStartNodeIdx, _patch->startIndexfromPos, _patch->startIndexfromPos_int);
		GLKPOSITION ENDPOS = _patch->GetNodeList().FindIndexFrom(startIndex + EdgeEndNodeIdx, _patch->startIndexfromPos, _patch-> startIndexfromPos_int);
		//qDebug("test2");
		edge->SetStartPoint((QMeshNode*)_patch->GetNodeList().GetAt(STARTPOS));
		edge->SetEndPoint((QMeshNode*)_patch->GetNodeList().GetAt(ENDPOS));
		edge->_depthIdx = 3;
		edge->_depth = _tree->depth;
		_patch->GetEdgeList().AddTail(edge);


	}
	_patch->startIndexfromPos = temp;
	_patch->startIndexfromPos_int = _patch->GetNodeList().GetCount() - 1;
}

void AABBManager::_addSingleTetrahedron_debugQuery(QMeshTetra* tet, QMeshPatch* _patch)
{

	GLKPOSITION temp;
	int start[6] = { 0,0,0,1,1,2 };
	int end[6] = {1,2,3,2,3,3};
	double pos[3];
	for (int i = 0; i <= 5; i++)
	{

		QMeshNode* node = new QMeshNode;
		tet->GetNodeRecordPtr(start[i]+1)->GetCoord3D(pos[0], pos[1], pos[2]);
		node->SetCoord3D(pos[0], pos[1], pos[2]);
		/*qDebug("%d edge start point is %lf %lf %lf", i, pos[0], pos[1], pos[2]);*/
		_patch->GetNodeList().AddTail(node);

		QMeshNode* node2 = new QMeshNode;
		tet->GetNodeRecordPtr(end[i]+1)->GetCoord3D(pos[0], pos[1], pos[2]);
		//qDebug("%d edge end point is %lf %lf %lf", i, pos[0], pos[1], pos[2]);
		node2->SetCoord3D(pos[0], pos[1], pos[2]);
		_patch->GetNodeList().AddTail(node2);

		QMeshEdge* edge = new QMeshEdge;
		edge->SetStartPoint(node);
		edge->SetEndPoint(node2);
		_patch->GetEdgeList().AddTail(edge);
		edge->_depthIdx = 27;

		/*_patch->GetNodeList().AddTail(tet->GetEdgeRecordPtr(i)->GetStartPoint());
		temp = _patch->GetNodeList().AddTail(tet->GetEdgeRecordPtr(i)->GetEndPoint());
		_patch->GetEdgeList().AddTail(tet->GetEdgeRecordPtr(i));
		tet->GetEdgeRecordPtr(i)->_depthIdx = 17;*/
	}

	/*_patch->startIndexfromPos = temp;
	_patch->startIndexfromPos_int = _patch->GetNodeList().GetCount() - 1;*/
}

//add a new bounding box to be further visualized 
void AABBManager::_addSingleAABBboundingBox(AABB _aabb, QMeshPatch* _patch, int _depthIdx)
{
	//static GLKPOSITION startIndexfromPos= NULL;
	//static int startIndexfromPos_int = 0;
	//static int edgePair[24] = {0,1,1,2,2,3,3,0,4,5,5,6,6,7,7,4,0,4,1,5,2,6,3,7};		//bounding box edges from point indices
	//AABB box = _aabb;
	//int startIndex = _patch->GetNodeNumber();
	//QMeshNode* node = new QMeshNode;
	//node->SetCoord3D(box.lowerBound[0], box.lowerBound[1], box.lowerBound[2]); _treeVisualizationMesh->GetNodeList().AddTail(node);
	//node = new QMeshNode;
	//node->SetCoord3D(box.upperBound[0], box.lowerBound[1], box.lowerBound[2]); _treeVisualizationMesh->GetNodeList().AddTail(node);
	//node = new QMeshNode;
	//node->SetCoord3D(box.upperBound[0], box.upperBound[1], box.lowerBound[2]); _treeVisualizationMesh->GetNodeList().AddTail(node);
	//node = new QMeshNode;
	//node->SetCoord3D(box.lowerBound[0], box.upperBound[1], box.lowerBound[2]); _treeVisualizationMesh->GetNodeList().AddTail(node);

	//node = new QMeshNode;
	//node->SetCoord3D(box.lowerBound[0], box.lowerBound[1], box.upperBound[2]); _treeVisualizationMesh->GetNodeList().AddTail(node);
	//node = new QMeshNode;
	//node->SetCoord3D(box.upperBound[0], box.lowerBound[1], box.upperBound[2]); _treeVisualizationMesh->GetNodeList().AddTail(node);
	//node = new QMeshNode;
	//node->SetCoord3D(box.upperBound[0], box.upperBound[1], box.upperBound[2]); _treeVisualizationMesh->GetNodeList().AddTail(node);
	//node = new QMeshNode;
	//node->SetCoord3D(box.lowerBound[0], box.upperBound[1], box.upperBound[2]); 
	//GLKPOSITION temp = _treeVisualizationMesh->GetNodeList().AddTail(node);

	//
	//for (int i = 0; i < 12; i++)
	//{
	//	int EdgeStartNodeIdx =	edgePair[i * 2 + 0];
	//	int EdgeEndNodeIdx   =	edgePair[i * 2 + 1];

	//	QMeshEdge* edge = new QMeshEdge;

	//	GLKPOSITION STARTPOS=_treeVisualizationMesh->GetNodeList().FindIndexFrom(startIndex + EdgeStartNodeIdx, startIndexfromPos, startIndexfromPos_int);
	//	GLKPOSITION ENDPOS = _treeVisualizationMesh->GetNodeList().FindIndexFrom(startIndex + EdgeEndNodeIdx, startIndexfromPos, startIndexfromPos_int);

	//	edge->SetStartPoint((QMeshNode*)_treeVisualizationMesh->GetNodeList().GetAt(STARTPOS));
	//	edge->SetEndPoint((QMeshNode*)_treeVisualizationMesh->GetNodeList().GetAt(ENDPOS));
	//	edge->_depthIdx = _depthIdx;
	//	edge->_depth = _tree->depth;
	//	_treeVisualizationMesh->GetEdgeList().AddTail(edge);

	//	
	//}
	//startIndexfromPos = temp;
	//startIndexfromPos_int = _treeVisualizationMesh->GetNodeList().GetCount() - 1;

}


void AABBManager::_addSingleAABBboundingBox_compatiblewith_BuildAllTreeNode(AABBTREEArrayNode* _treeNode, AABB _aabb, QMeshPatch* _patch, int _depthIdx)
{
	static int edgePair[24] = { 0,1,1,2,2,3,3,0,4,5,5,6,6,7,7,4,0,4,1,5,2,6,3,7 };		//bounding box edges from point indices
	AABB box = _aabb;
	int startIndex = _patch->GetNodeNumber();

	QMeshPatch* Patch = (QMeshPatch*)_refitBoundingBoxPoly->GetMeshList().GetHead();
	QMeshNode* node = new QMeshNode;
	node->SetCoord3D(box.lowerBound[0], box.lowerBound[1], box.lowerBound[2]); Patch->GetNodeList().AddTail(node);
	node = new QMeshNode;
	node->SetCoord3D(box.upperBound[0], box.lowerBound[1], box.lowerBound[2]); Patch->GetNodeList().AddTail(node);
	node = new QMeshNode;
	node->SetCoord3D(box.upperBound[0], box.upperBound[1], box.lowerBound[2]); Patch->GetNodeList().AddTail(node);
	node = new QMeshNode;
	node->SetCoord3D(box.lowerBound[0], box.upperBound[1], box.lowerBound[2]); Patch->GetNodeList().AddTail(node);

	node = new QMeshNode;
	node->SetCoord3D(box.lowerBound[0], box.lowerBound[1], box.upperBound[2]); Patch->GetNodeList().AddTail(node);
	node = new QMeshNode;
	node->SetCoord3D(box.upperBound[0], box.lowerBound[1], box.upperBound[2]); Patch->GetNodeList().AddTail(node);
	node = new QMeshNode;
	node->SetCoord3D(box.upperBound[0], box.upperBound[1], box.upperBound[2]); Patch->GetNodeList().AddTail(node);
	node = new QMeshNode;
	node->SetCoord3D(box.lowerBound[0], box.upperBound[1], box.upperBound[2]);
	GLKPOSITION temp = Patch->GetNodeList().AddTail(node);


	for (int i = 0; i < 12; i++)
	{
		int EdgeStartNodeIdx = edgePair[i * 2 + 0];
		int EdgeEndNodeIdx = edgePair[i * 2 + 1];

		QMeshEdge* edge = new QMeshEdge;
		GLKPOSITION STARTPOS = Patch->GetNodeList().FindIndexFrom(startIndex + EdgeStartNodeIdx, startIndexfromPos, startIndexfromPos_int);
		GLKPOSITION ENDPOS = Patch->GetNodeList().FindIndexFrom(startIndex + EdgeEndNodeIdx, startIndexfromPos, startIndexfromPos_int);

		edge->SetStartPoint((QMeshNode*)Patch->GetNodeList().GetAt(STARTPOS));
		edge->SetEndPoint((QMeshNode*)Patch->GetNodeList().GetAt(ENDPOS));
		edge->_depthIdx = _depthIdx;
		edge->_depth = _tree->depth;
		Patch->GetEdgeList().AddTail(edge);

		edge->_isLeafNode = _treeNode->isLeaf;


	}

	startIndexfromPos = temp;
	startIndexfromPos_int = Patch->GetNodeList().GetCount() - 1;
}

void AABBManager::_updateSingleAABBboundingBox_compatiblewith_BuildAllTreeNode(AABBTREEArrayNode* _treeNode, AABB _aabb, QMeshPatch* _patch, int _depthIdx )	//depth index used to draw color
{
	//startIndexPos

	AABB box = _aabb;
	QMeshPatch* Patch = _treeVisualizationMesh;
	QMeshNode* node = (QMeshNode*)Patch->GetNodeList().GetNext(startIndexPos);
	node->SetCoord3D(box.lowerBound[0], box.lowerBound[1], box.lowerBound[2]); 
	node = (QMeshNode*)Patch->GetNodeList().GetNext(startIndexPos);
	node->SetCoord3D(box.upperBound[0], box.lowerBound[1], box.lowerBound[2]); 
	node = (QMeshNode*)Patch->GetNodeList().GetNext(startIndexPos);
	node->SetCoord3D(box.upperBound[0], box.upperBound[1], box.lowerBound[2]); 
	node = (QMeshNode*)Patch->GetNodeList().GetNext(startIndexPos);
	node->SetCoord3D(box.lowerBound[0], box.upperBound[1], box.lowerBound[2]); 

	node = (QMeshNode*)Patch->GetNodeList().GetNext(startIndexPos);
	node->SetCoord3D(box.lowerBound[0], box.lowerBound[1], box.upperBound[2]); 
	node = (QMeshNode*)Patch->GetNodeList().GetNext(startIndexPos);
	node->SetCoord3D(box.upperBound[0], box.lowerBound[1], box.upperBound[2]); 
	node = (QMeshNode*)Patch->GetNodeList().GetNext(startIndexPos);
	node->SetCoord3D(box.upperBound[0], box.upperBound[1], box.upperBound[2]); 
	node = (QMeshNode*)Patch->GetNodeList().GetNext(startIndexPos);
	node->SetCoord3D(box.lowerBound[0], box.upperBound[1], box.upperBound[2]);

}

//add several new bounding boxes which belong to same level depth to be further visualized 
void AABBManager::_addSameTreeDepthBoundingBox(int depth)
{
	printf("tree node is %d\n", _tree->nodeCount);
	for (int i = 0; i < _tree->nodeCount; i++)
	{
		
		if (_tree->nodes[i].depth==depth)
		{
			//if the node doesnt contain tetrahedron
			if (_tree->nodes[i].objectIndex == -1)
				continue;
			_addSingleAABBboundingBox(_tree->nodes[i].collisionBox, _treeVisualizationMesh, depth);
		}
		
	}
	printf("Tree depth %d is finished...\n",depth);
}

void AABBManager::_buildAllTreeNodeBoundingBox(void)
{
	int interval = _tree->nodeCount / 11;
	for (int i = 0; i < _tree->nodeCount; i++)
	{
		if (_tree->nodes[i].objectIndex == -1)
			continue;
		_addSingleAABBboundingBox_compatiblewith_BuildAllTreeNode(&(_tree->nodes[i]),_tree->nodes[i].collisionBox, _treeVisualizationMesh, _tree->nodes[i].depth);
		//qDebug("%d", _tree->nodes[i].depth);
		if(i% interval ==0)
			printf("Build Percent: %3.3lf \n",(double)(i+1)/(double)(_tree->nodeCount)*100.0);
	}
}

//_updateAllTreeNodeBoundingBox
void AABBManager::_updateAllTreeNodeBoundingBox(void)
{
	
	QMeshPatch* _patch = _treeVisualizationMesh;
	startIndexPos = _patch->GetNodeList().GetHeadPosition();
	int interval = _tree->nodeCount / 11;
	printf("Tree Node Count: %d\n", _tree->nodeCount);
	for (int i = 0; i < _tree->nodeCount; i++)
	{
		if (_tree->nodes[i].objectIndex == -1)
			continue;
		_updateSingleAABBboundingBox_compatiblewith_BuildAllTreeNode(&(_tree->nodes[i]), _tree->nodes[i].collisionBox, _treeVisualizationMesh, _tree->nodes[i].depth);

		/*if (i % interval == 0)
			printf("Update Percent: %3.3lf \n", (double)(i + 1) / (double)(_tree->nodeCount) * 100.0);*/
	}
}

//PolygenMesh* AABBManager::GenerateVisualizationMesh(int NameIdx)
//{
//	PolygenMesh* poly = new PolygenMesh(BOUNDING_BOX);
//	poly->setModelName("AABB Tree "+to_string(NameIdx));
//	_treeVisualizationMesh = new QMeshPatch;
//	poly->GetMeshList().AddTail(_treeVisualizationMesh);
//	_refitBoundingBoxPoly = poly;
//	printf("(%2d) Tree depth is %d\n", NameIdx, _tree->depth);
//
//	//build the specific depth layer(s)
//	/*for (int i = 0; i <= 19; i++)
//	{
//		_addSameTreeDepthBoundingBox(i);
//	}*/
//
//	//build all depth layers
//	_buildAllTreeNodeBoundingBox();
//	
//	
//	return poly;
//}

void AABBManager::UpdateVisualizationMesh_refit(void)
{
	_updateAllTreeNodeBoundingBox();
}

void AABBManager::ReBuildVisualizationMesh_rebuild(void)
{
	//remove and clean all the elements
	_treeVisualizationMesh->ClearAll();
	//_refitBoundingBoxPoly
	_buildAllTreeNodeBoundingBox();

}

//////////////////////////////////////////////////////////
///////AABB Tree Construction from Tetrahedron Mesh///////
//////////////////////////////////////////////////////////
//children relationships are being built when inserting child nodes.
void AABBManager::TreeConstructionTop2Bot(QMeshPatch* _tetMesh)
{

	_tetModelMesh = _tetMesh;
	//------------------------------------------------------------------------------------------------
	//	Step 1: memory allocation of tree; grouping all tetra element into one array
	//			calculate the center of tetrahedron and bounding box.
	_tree = new aabbTree;
	_tree->nodes = new AABBTREEArrayNode[MAXNUM_OF_AABBNODE_ONTREE];
	_tree->nodeCount = -1;
	_tree->rootIndex = -1;

	TetraPtrArrayLen = _tetMesh->GetTetraList().GetCount();
	TetraPtrArray = new QMeshTetra*[TetraPtrArrayLen];
	int eleIndex = 0;

	//calculate the bounding box and its center
	//mark the index of each tetrahedron
	for (GLKPOSITION Pos = _tetMesh->GetTetraList().GetHeadPosition(); Pos != NULL; eleIndex++) {
		QMeshTetra* Tet = (QMeshTetra*)(_tetMesh->GetTetraList().GetNext(Pos));


		Tet->GetNodeRecordPtr(1)->GetCoord3D(Tet->pos1[0], Tet->pos1[1], Tet->pos1[2]);
		Tet->GetNodeRecordPtr(2)->GetCoord3D(Tet->pos2[0], Tet->pos2[1], Tet->pos2[2]);
		Tet->GetNodeRecordPtr(3)->GetCoord3D(Tet->pos3[0], Tet->pos3[1], Tet->pos3[2]);
		Tet->GetNodeRecordPtr(4)->GetCoord3D(Tet->pos4[0], Tet->pos4[1], Tet->pos4[2]);

		TetraPtrArray[eleIndex] = Tet;		//assign it to be inside the tetra array.
		Tet->_idx = eleIndex;
		Tet->CalCenCalBoundingBox();		//calculate tetrahedron center and its bounding box.

		if (eleIndex < 5)
		{
			printf("Vertex pos: %lf %lf %lf\n", Tet->_center[0], Tet->_center[1], Tet->_center[2]);
		}
	}

	printf("\nStep 1 finished...\n");
	//------------------------------------------------------------------------------------------------


	//------------------------------------------------------------------------------------------------
	//	Step 2: creating the the root node and insert it into the tree
	//  first, calculate the bounding box of all elements
	for (int i = 0; i < 3; i++) {
		_init.lowerBound[i] = INF; _init.upperBound[i] = NINF;
	}

	double pos[3];
	for (int i = 0; i < TetraPtrArrayLen; i++)
	{
		//travel all points to update the minimal x, y and z
		//							  the maximum x, y and z
		QMeshTetra* tet = TetraPtrArray[i];
		for (int j = 0; j < 4; j++)
		{
			(tet->GetNodeRecordPtr(j+1))->GetCoord3D(pos[0],pos[1],pos[2]);
			_init.lowerBound[0] = MIN(pos[0], _init.lowerBound[0]);
			_init.lowerBound[1] = MIN(pos[1], _init.lowerBound[1]);
			_init.lowerBound[2] = MIN(pos[2], _init.lowerBound[2]);

			_init.upperBound[0] = MAX(pos[0], _init.upperBound[0]);
			_init.upperBound[1] = MAX(pos[1], _init.upperBound[1]);
			_init.upperBound[2] = MAX(pos[2], _init.upperBound[2]);
		}
	}
	// second, insert the root node
	// _printBoundingBoxInfo("all node", _init);
	_insert2Tree(0,-1, _init);			
	printf("\nStep 2 finished...\n");
	//------------------------------------------------------------------------------------------------
	       

	//------------------------------------------------------------------------------------------------
	//	Step 3: gradually split the bounding box and add them to the box
	//first, split the bounding box.
	AABB left, right;
	splitBoundingBox(_init,&left,&right);

	printf("Start to call recursive function...\n");
	
	_recursiveConstruction(0, 1, left,1);
	_recursiveConstruction(0, 2, right,1);
	printf("Stop calling  recursive function...\n");
	printf("\nStep 3 finished...\n");
	
	//------------------------------------------------------------------------------------------------

}


int AABBManager::GetTreeDepth(void)
{
	return _tree->depth;
}



//////////////////////////////////////////////////////////
///////Rebuild AABB Tree from New Tetrahedron Mesh////////
//////////////////////////////////////////////////////////
//children relationships are being built when inserting child nodes.
void AABBManager::TreeConstructionTop2Bot_rebuild(QMeshPatch* _tetMesh)
{
	//the variable when fast building the edge list should be reset
	startIndexfromPos = NULL;
	startIndexfromPos_int = 0;


	//------------------------------------------------------------------------------------------------
	//	Step 1: memory allocation of tree; grouping all tetra element into one array
	//			calculate the center of tetrahedron and bounding box.
	//clear old _tree 
	//maybe we should not delete this, directly define the nodeCount and rootIndex should be fine
	/*delete []_tree->nodes;
	_tree->nodes = new AABBTREEArrayNode[MAXNUM_OF_AABBNODE_ONTREE];*/

	if (!selfCollisionTree_built)
	{
		_tree = new aabbTree;
		_tree->nodes = new AABBTREEArrayNode[MAXNUM_OF_AABBNODE_ONTREE];
		_tree->nodeCount = -1;
		_tree->rootIndex = -1;
		_treeInit = true;

		TetraPtrArrayLen = _tetMesh->GetTetraList().GetCount();
		TetraPtrArray = new QMeshTetra * [TetraPtrArrayLen];
		int eleIndex = 0;

		//calculate the bounding box and its center
		//mark the index of each tetrahedron
		for (GLKPOSITION Pos = _tetMesh->GetTetraList().GetHeadPosition(); Pos != NULL; eleIndex++) {
			QMeshTetra* Tet = (QMeshTetra*)(_tetMesh->GetTetraList().GetNext(Pos));


			Tet->GetNodeRecordPtr(1)->GetCoord3D(Tet->pos1[0], Tet->pos1[1], Tet->pos1[2]);
			Tet->GetNodeRecordPtr(2)->GetCoord3D(Tet->pos2[0], Tet->pos2[1], Tet->pos2[2]);
			Tet->GetNodeRecordPtr(3)->GetCoord3D(Tet->pos3[0], Tet->pos3[1], Tet->pos3[2]);
			Tet->GetNodeRecordPtr(4)->GetCoord3D(Tet->pos4[0], Tet->pos4[1], Tet->pos4[2]);

			TetraPtrArray[eleIndex] = Tet;		//assign it to be inside the tetra array.
			Tet->_idx = eleIndex;
			Tet->CalCenCalBoundingBox();		//calculate tetrahedron center and its bounding box.

			/*if (eleIndex < 5)
			{
				printf("Vertex pos: %lf %lf %lf\n", Tet->_center[0], Tet->_center[1], Tet->_center[2]);
			}*/
		}
	}
	else
	{
		_treeInit = true;
		for (int i = 0; i < _tree->nodeCount; i++)
		{
			_tree->nodes[i].tetraList.clear();
			_tree->nodes[i].isLeaf = false;
			_tree->nodes[i].objectIndex = -2;
			_tree->nodes[i].depth = -1;
		}
		_tree->rootIndex = 0;
		_tree->nodeCount = 0;
		_tree->depth = 0;



		delete[] TetraPtrArray;
		TetraPtrArrayLen = _tetMesh->GetTetraList().GetCount();
		TetraPtrArray = new QMeshTetra * [TetraPtrArrayLen];

		int eleIndex = 0;

		//calculate the bounding box and its center
		//mark the index of each tetrahedron
		for (GLKPOSITION Pos = _tetMesh->GetTetraList().GetHeadPosition(); Pos != NULL; eleIndex++) {
			QMeshTetra* Tet = (QMeshTetra*)(_tetMesh->GetTetraList().GetNext(Pos));
			TetraPtrArray[eleIndex] = Tet;		//assign it to be inside the tetra array.
			Tet->CalCenCalBoundingBox();		//calculate tetrahedron center and its bounding box.
			Tet->_idx = eleIndex;

			/*if (eleIndex < 5)
			{
				printf("Vertex pos: %lf %lf %lf\n", Tet->_center[0], Tet->_center[1], Tet->_center[2]);
			}*/

		}

	}
	
	

	printf("\nStep 1 finished...\n");
	//------------------------------------------------------------------------------------------------


	//------------------------------------------------------------------------------------------------
	//	Step 2: creating the the root node and insert it into the tree
	//  first, calculate the bounding box of all elements

	//Yingjun: check here.
	for (int i = 0; i < 3; i++) {
		_init.lowerBound[i] = INF; _init.upperBound[i] = NINF;
	}

	double pos[3];
	for (int i = 0; i < TetraPtrArrayLen; i++)
	{
		//travel all points to update the minimal x, y and z
		//							  the maximum x, y and z
		QMeshTetra* tet = TetraPtrArray[i];
		for (int j = 0; j < 4; j++)
		{
			(tet->GetNodeRecordPtr(j + 1))->GetCoord3D(pos[0], pos[1], pos[2]);
			_init.lowerBound[0] = MIN(pos[0], _init.lowerBound[0]);
			_init.lowerBound[1] = MIN(pos[1], _init.lowerBound[1]);
			_init.lowerBound[2] = MIN(pos[2], _init.lowerBound[2]);

			_init.upperBound[0] = MAX(pos[0], _init.upperBound[0]);
			_init.upperBound[1] = MAX(pos[1], _init.upperBound[1]);
			_init.upperBound[2] = MAX(pos[2], _init.upperBound[2]);
		}
	}
	// second, insert the root node
	// _printBoundingBoxInfo("all node", _init);
	_insert2Tree(0, -1, _init);
	printf("\nStep 2 finished...\n");
	//------------------------------------------------------------------------------------------------


	//------------------------------------------------------------------------------------------------
	//	Step 3: gradually split the bounding box and add them to the box
	//first, split the bounding box.
	AABB left, right;
	splitBoundingBox(_init, &left, &right);

	printf("Start to call recursive function...\n");

	_recursiveConstruction_rebuild(0, 1, left, 1);
	_recursiveConstruction_rebuild(0, 2, right, 1);
	printf("Stop calling  recursive function...\n");
	printf("\nStep 3 finished...\n");

	//------------------------------------------------------------------------------------------------

	selfCollisionTree_built = true;
}


//Recursively construct AABB tree in rebuilding process
//given the known bounding box, it will split the largest dimension

//para: @_nodeIdx: parent node idx. 
//		@_flag:  1 -> left child; 2 -> right child
//		@_oldBoundingBox:	newly split bounding box, we need to update this bounding box according to all tetra whose center is within _oldBoundingBox
void AABBManager::_recursiveConstruction_rebuild(int _nodeIdx, int _flag, AABB _old, int _recursiveLayer)
{

	//update the bounding box of _old
	//long startTime = clock();
	int _insertIdx = _tree->nodeCount;
	AABB _oldCollisionBox;
	for (int i = 0; i < 3; i++)
	{
		_oldCollisionBox.lowerBound[i] = INF;
		_oldCollisionBox.upperBound[i] = NINF;
	}

	double pp[3];
	int _inOldBoxNum = 0;
	int _objectIdx = -1;

	//version 1: search from all tetrahedron
	//the speed here is too slow
	//we just need to search from the parent's containment
	//for (int i = 0; i < TetraPtrArrayLen; i++)
	//{
	//	//travel all points to update the minimal x, y and z
	//	//							  the maximum x, y and z
	//	QMeshTetra* tet = TetraPtrArray[i];
	//	pp[0] = tet->_center[0];
	//	pp[1] = tet->_center[1];
	//	pp[2] = tet->_center[2];
	//	//if tet center is within _old, then update the bounding box including this tet
	//	
	//	if ((pp[0] > _old.lowerBound[0] && pp[0] < _old.upperBound[0])&& (pp[1] > _old.lowerBound[1] && pp[1] < _old.upperBound[1])&& (pp[2] > _old.lowerBound[2] && pp[2] < _old.upperBound[2]) )
	//	{
	//		//within _old bounding box
	//		_inOldBoxNum++;
	//		_objectIdx = i;
	//		//update bounding box
	//		for (int j = 0; j < 3; j++)
	//		{
	//			if (tet->_lowerBounding[j] < _oldCollisionBox.lowerBound[j])
	//			{
	//				_oldCollisionBox.lowerBound[j] = tet->_lowerBounding[j];
	//			}
	//			if (tet->_upperBounding[j] > _oldCollisionBox.upperBound[j])
	//			{
	//				_oldCollisionBox.upperBound[j] = tet->_upperBounding[j];
	//			}
	//		}
	//	}
	//}

	//version 2: search from all tetrahedron from parent (i.e., travel  _tree->nodes[_nodeIdx].tetraList)

	for (TetIter tet : (_tree->nodes[_nodeIdx].tetraList))
	{
		pp[0] = tet->_center[0];
		pp[1] = tet->_center[1];
		pp[2] = tet->_center[2];

		if ((pp[0] > _old.lowerBound[0] && pp[0] < _old.upperBound[0]) && (pp[1] > _old.lowerBound[1] && pp[1] < _old.upperBound[1]) && (pp[2] > _old.lowerBound[2] && pp[2] < _old.upperBound[2]))
		{
			//within _old bounding box
			_tree->nodes[_insertIdx].tetraList.push_back(tet);					//update this AABB Tree node's containment list
			_inOldBoxNum++;
			_objectIdx = tet->_idx;
			//update bounding box
			for (int j = 0; j < 3; j++)
			{
				if (tet->_lowerBounding[j] < _oldCollisionBox.lowerBound[j])
				{
					_oldCollisionBox.lowerBound[j] = tet->_lowerBounding[j];
				}

				if (tet->_upperBounding[j] > _oldCollisionBox.upperBound[j])
				{
					_oldCollisionBox.upperBound[j] = tet->_upperBounding[j];
				}
			}
		}

	}
	//for (int i = 0; i < TetraPtrArrayLen; i++)
	//{
	//	//travel all points to update the minimal x, y and z
	//	//							  the maximum x, y and z
	//	_tree->nodes[0].tetraList.push_back(TetraPtrArray[i]);
	//}



	//long endTime = clock();
	//long Time1 = endTime - startTime;
	//ending criterion: check whether this node is the leef node and insert it into AABB Tree.
	//update the insert index of current node

	if (_inOldBoxNum == 1)
	{
		//leaf node that contains only one tetrahedron whose idx is objectIndex.
		//here we end the recursive function
		_tree->nodes[_insertIdx].isLeaf = true;
		_tree->nodes[_insertIdx].parentIndex = _nodeIdx;
		_tree->nodes[_insertIdx].box = _old;
		_tree->nodes[_insertIdx].collisionBox = _oldCollisionBox;

		_tree->nodes[_insertIdx].objectIndex = _objectIdx;
		_tree->nodes[_insertIdx].depth = _recursiveLayer;
		_tree->nodeCount++;

		if (_flag == 1)
			_tree->nodes[_nodeIdx].child1 = _insertIdx;		//this newly inserted node is left child
		else if (_flag == 2)
			_tree->nodes[_nodeIdx].child2 = _insertIdx;		//this newly inserted node is right child

		if (_recursiveLayer > _tree->depth)
			_tree->depth = _recursiveLayer;
		//printf("Reach leaf node: in old box num is %d\n",1);
		return;

	}
	else if (_inOldBoxNum == 0)
	{
		//leaf that dosen't contain any tetrahedron because the highly distorted tetrahedron this time
		//here we end the recursive function
		//use _old to be This AABB 
		_tree->nodes[_insertIdx].isLeaf = true;
		_tree->nodes[_insertIdx].parentIndex = _nodeIdx;
		_tree->nodes[_insertIdx].box = _old;
		_tree->nodes[_insertIdx].collisionBox = _old;
		_tree->nodes[_insertIdx].objectIndex = -1;
		_tree->nodes[_insertIdx].depth = _recursiveLayer;
		_tree->nodeCount++;

		if (_flag == 1)
			_tree->nodes[_nodeIdx].child1 = _insertIdx;		//this newly inserted node is left child
		else if (_flag == 2)
			_tree->nodes[_nodeIdx].child2 = _insertIdx;		//this newly inserted node is right child

		if (_recursiveLayer > _tree->depth)
			_tree->depth = _recursiveLayer;
		//printf("Reach leaf node: in old box num is %d\n", 0);
		return;
	}
	else {
		//leaf node that contain at leat two tets, we continue the recursive function
		_tree->nodes[_insertIdx].isLeaf = false;
		_tree->nodes[_insertIdx].parentIndex = _nodeIdx;
		_tree->nodes[_insertIdx].box = _old;
		_tree->nodes[_insertIdx].collisionBox = _oldCollisionBox;
		_tree->nodes[_insertIdx].objectIndex = -2;			//this var dosent matter here
		_tree->nodes[_insertIdx].depth = _recursiveLayer;
		_tree->nodeCount++;

		if (_flag == 1)
			_tree->nodes[_nodeIdx].child1 = _insertIdx;		//this newly inserted node is left child
		else if (_flag == 2)
			_tree->nodes[_nodeIdx].child2 = _insertIdx;		//this newly inserted node is right child

	}
	/*if (_recursiveLayer == 10)
		return;*/
		//normal execution
	AABB left, right;
	splitBoundingBox(_old, &left, &right);
	/*if (_flag == 1)
	{
		counter[_recursiveLayer]++;
		if (counter[_recursiveLayer] == 1)
		{

			printf("\nLeft Layer: %d\n", _recursiveLayer);
			_printBoundingBoxInfo(to_string(_recursiveLayer), _old);
			_printBoundingBoxInfo("Next would be:", left);

		}

	}*/
	//long Time2 = (long)clock() - startTime;
	//printf("part is taking %3.3lf percent\n",(double)Time1/ (double)Time2 *100.0);

	_recursiveConstruction_rebuild(_insertIdx, 1, left, _recursiveLayer + 1);
	_recursiveConstruction_rebuild(_insertIdx, 2, right, _recursiveLayer + 1);

	//debug output
	/*if (_flag == 1)
		printf("left: [%lf, %lf, %lf] -> [%lf, %lf, %lf]\n", _old.lowerBound[0], _old.lowerBound[1], _old.lowerBound[2], _old.upperBound[0], _old.upperBound[1], _old.upperBound[2]);
	if (_flag == 2)
		printf("right: [%lf, %lf, %lf] -> [%lf, %lf, %lf]\n", _old.lowerBound[0], _old.lowerBound[1], _old.lowerBound[2], _old.upperBound[0], _old.upperBound[1], _old.upperBound[2]);*/

}


//////////////////////////////////////////////////////////
///////Refit AABB Tree from New Tetrahedron Mesh//////////
//////////////////////////////////////////////////////////
//children relationships remain the same
void AABBManager::TreeConstructionTop2Bot_refit(QMeshPatch* _tetMesh)
{

	////the variable when fast building the edge list should be reset
	//startIndexfromPos = NULL;
	//startIndexfromPos_int = 0;

	//------------------------------------------------------------------------------------------------
	//	Step 1: do the postorder traversal around the tree
	//			calculate the center of tetrahedron and bounding box.
	
	//calculate the bounding box and its center
	//mark the index of each tetrahedron
	int eleIndex = 0;
	for (GLKPOSITION Pos = _tetMesh->GetTetraList().GetHeadPosition(); Pos != NULL; eleIndex++) {
		QMeshTetra* Tet = (QMeshTetra*)(_tetMesh->GetTetraList().GetNext(Pos));
		//TetraPtrArray[eleIndex] = Tet;		//assign it to be inside the tetra array.
		Tet->_idx = eleIndex;
		Tet->CalCenCalBoundingBox();		//calculate tetrahedron center and its bounding box.
		
	}
	_recursivePostOrderTraversal_refit(0, 1, 1);
	_recursivePostOrderTraversal_refit(0, 2, 1);


	int leftChildIdx = _tree->nodes[0].child1;
	int rightChildIdx = _tree->nodes[0].child2;
	_tree->nodes[0].collisionBox = Union(_tree->nodes[leftChildIdx].collisionBox, _tree->nodes[rightChildIdx].collisionBox);

	printf("\nRefit Finished...\n");
	//------------------------------------------------------------------------------------------------


	
}

//Recursively update AABB tree: update collision box of each node including internal nodes and leaf nodes
//(collision tetraList of each tree node would not be updated)
//given the known node no matter which depth this node is, it will update the collision bounding box according to its children

//para: @_nodeIdx: parent node idx. 
//		@_flag:  1 -> left child; 2 -> right child (This child node is the left or right node of its parent)
//		@_oldBoundingBox:	newly split bounding box, we need to update this bounding box according to all tetra whose center is within _oldBoundingBox
void AABBManager::_recursivePostOrderTraversal_refit(int _nodeIdx, int _flag, int _recursiveLayer)
{
	
	//------------------------------------------------------------------------------------------------
	//	Step 1: check the terminal condition is reached and update the collision box
	//------------------------------------------------------------------------------------------------
	AABBTREEArrayNode* thisNode;
	int thisNodeIdx = -1;
	if (_flag == 1)
		thisNodeIdx = _tree->nodes[_nodeIdx].child1;
	else
		thisNodeIdx = _tree->nodes[_nodeIdx].child2;

	thisNode = &(_tree->nodes[thisNodeIdx]);

	if (thisNode->isLeaf == true)
	{
		int tetIdx = thisNode->objectIndex;
		if (tetIdx == -1)
		{
			//dont do anything
			return;
		}
		else 
		{
			//update the collision box
			QMeshTetra* tetraPtr = TetraPtrArray[tetIdx];
			//tetraPtr->CalCenCalBoundingBox();
			for (int i = 0; i < 3; i++)
			{
				thisNode->collisionBox.lowerBound[i] = tetraPtr->_lowerBounding[i];
				thisNode->collisionBox.upperBound[i] = tetraPtr->_upperBounding[i];
			}
			return;
		}

	}
	

	//------------------------------------------------------------------------------------------------
	//	Step 2: traversal the left children
	//------------------------------------------------------------------------------------------------
	_recursivePostOrderTraversal_refit(thisNodeIdx,1, _recursiveLayer+1);

	//------------------------------------------------------------------------------------------------
	//	Step 3: traversal the right children
	//------------------------------------------------------------------------------------------------
	_recursivePostOrderTraversal_refit(thisNodeIdx, 2, _recursiveLayer + 1);

	//------------------------------------------------------------------------------------------------
	//	Step 4: merge children's collision bounding box into this node's collision bounding box 
	//			and then return
	//  first get left child's bounding box info

	int leftChildIndex = thisNode->child1;
	int rightChildIndex = thisNode->child2;
	
	if ((_tree->nodes[leftChildIndex].objectIndex != -1)&&(_tree->nodes[rightChildIndex].objectIndex != -1))
	{
		//if the left child is not the empty left node and right child is also not.
		//merge two collision box together and assign it to be this node's new collision box
		thisNode->collisionBox = Union(_tree->nodes[leftChildIndex].collisionBox, _tree->nodes[rightChildIndex].collisionBox);
	}
	else if ((_tree->nodes[leftChildIndex].objectIndex == -1) && (_tree->nodes[rightChildIndex].objectIndex != -1))
	{
		//if the left child is empty leaf node and right child is not.
		thisNode->collisionBox = _tree->nodes[rightChildIndex].collisionBox;
	}
	else if ((_tree->nodes[leftChildIndex].objectIndex != -1) && (_tree->nodes[rightChildIndex].objectIndex == -1))
	{
		//if the left child is not empty leaf node and right child is.
		thisNode->collisionBox = _tree->nodes[leftChildIndex].collisionBox;
	}
	else {
		//if all are empty
		printf("Please check your code: this situation can not be happened cuz there are no two empty leaf nodes attached to the parent node...");

	}
	//------------------------------------------------------------------------------------------------

	

}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////Collision Query Area/////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


bool AABBManager::_isCollided(double* _pnt, AABB _queryBox)
{
	for (int i = 0; i < 3; i++)
	{
		if (_pnt[i]< _queryBox.lowerBound[i] || _pnt[i] > _queryBox.upperBound[i])
		{
			return false;
		}
	}
	return true;
}


//Self Collision Detection Conducted into the Imported Model
void AABBManager::SelfCollisionDetection(void)
{
	static int counter_collided = 0;

	//if (counter_collided != 0)
	//{
	//	int tetNum = _debugPatch->GetTetraNumber();
	//	int edgeNum = _debugPatch->GetEdgeNumber();
	//	qDebug("Tet num is %d, while edge num is %d", tetNum, edgeNum);
	//	//clean the collided bounding box
	//	_debugPatch->ClearAllTet();

	//	_debugPatch->startIndexfromPos = NULL;
	//	_debugPatch->startIndexfromPos_int = 0;

	//}

	//------------------------------------------------------------------------------------------------
	//	Step 1: mark the boundary points which are lied on the boundary surface
	//  mark: this part is correct
	int eleIndex = 0;
	for (GLKPOSITION Pos = _tetModelMesh->GetNodeList().GetHeadPosition(); Pos != NULL; eleIndex++) {
		QMeshNode* node = (QMeshNode*)(_tetModelMesh->GetNodeList().GetNext(Pos));
		node->isBoundaryNode = false;
	}
	for (GLKPOSITION Pos = _tetModelMesh->GetFaceList().GetHeadPosition(); Pos != NULL; eleIndex++) {
		QMeshFace* face = (QMeshFace*)(_tetModelMesh->GetFaceList().GetNext(Pos));
		if (face->GetLeftTetra() == NULL || face->GetRightTetra() == NULL)
			for (int i = 0; i < 3; i++) face->GetNodeRecordPtr(i)->isBoundaryNode =true;
	}
	qDebug("Boundary Point Set Established...");
	//------------------------------------------------------------------------------------------------
	

	//------------------------------------------------------------------------------------------------
	//	Step 2: for each poitn on the boundary surface, we just check the collision result
	//  mark: this part in coding
	eleIndex = 0;
	double pos[3];
	eleIndex = 0;
	long startT = clock();
	int counter_boundaryPoint = 0;
	int counter_collidedPoint = 0;

	//debug 
	/*if (counter_collided == 0)
	{
		_debugPatch = new QMeshPatch;
	}*/
	

	for (GLKPOSITION Pos = _tetModelMesh->GetNodeList().GetHeadPosition(); Pos != NULL; ) {
		QMeshNode* node = (QMeshNode*)(_tetModelMesh->GetNodeList().GetNext(Pos));
		if (node->isBoundaryNode ==true)
		{
			counter_boundaryPoint++;
			//check this point's collision result
			bool _checkResult =  _singlePntQuery(node);
			if (_checkResult)
			{
				
				int collidedTetNum = node->_collidedRealTetraList.size();

				//for debug usage
				/*if (counter_collidedPoint != 0)
				{
					node->isCollided = false;
				}*/

				counter_collidedPoint++;
				//qDebug("point %5d (from zero) is in collision and collided tet num is %d....\n", eleIndex, collidedTetNum);
				/*if (counter_collidedPoint >= 5)
				{
					break;
				}*/
				//break;
			}
			else
			{
				//qDebug("point %5d (from zero) isn't in collision....\n",eleIndex);
			}
				
			eleIndex++;
			
				
		}
	}
	long endT = clock();
	//_tetModelMesh->collidedPntNum = counter_collidedPoint;
	printf("Time Elapse of Self Collision Detection: %3.3lf s\n", (double)(endT - startT) / (double)(CLOCKS_PER_SEC));
	qDebug("All boundary points are %3d, while collided points are %3d", counter_boundaryPoint, counter_collidedPoint);
	qDebug("Collision Detection Of Single Point Finished...");
	//------------------------------------------------------------------------------------------------

	counter_collided++;
}

void AABBManager::SelfCollisionDetectionCorrespondenceChecking_softFinger(void)
{
	

	if (init_softFingerSelfCollision)
	{
		//update the correspondence patch
		int tetNum = _debugPatch->GetTetraNumber();
		int edgeNum = _debugPatch->GetEdgeNumber();
		qDebug("Tet num is %d, while edge num is %d", tetNum, edgeNum);


		//clean the collided bounding box
		_debugPatch->ClearAllTet();
		_debugPatch->startIndexfromPos = NULL;
		_debugPatch->startIndexfromPos_int = 0;

		_debugPatch_tetDraw->ClearAllTet();
		_debugPatch_tetDraw->startIndexfromPos = NULL;
		_debugPatch_tetDraw->startIndexfromPos_int = 0;

		_corresPatch_selfCollision->ClearAllTet();

	}

	//debug 
	if (init_softFingerSelfCollision == false)
	{
		_debugPatch = new QMeshPatch;
		_debugPatch->startIndexfromPos = NULL;
		_debugPatch->startIndexfromPos_int = 0;

		_debugPatch_tetDraw = new QMeshPatch;
		_debugPatch_tetDraw->startIndexfromPos = NULL;
		_debugPatch_tetDraw->startIndexfromPos_int = 0;

		_corresPatch_selfCollision = new QMeshPatch;
	}

	

	qDebug("Boundary Point Set Established...");

	//------------------------------------------------------------------------------------------------
	//	Step 2: for each poitn on the boundary surface, we just check the collision result
	int eleIndex = 0;
	double pos[3];
	long startT = clock();
	int counter_boundaryPoint = 0;
	int counter_collidedPoint = 0;


	//face!

	//I. if it is in collision last time.
	//	1. when the point is on the outside of collided face but it is beyond a threshold (not collision at all), we still call _singlePntQuery to see whether it is truly collision-free
	//	2. when the point is on the outside of collided face but it is within a threshold, we keep the old spring and not call _singlePntQuery
	//	3. if it is on the same side of collided face, then we keep on to _singlePntQuery
	//II. if it is not in collision last time, directly call _singlePntQuery

	for (GLKPOSITION Pos = _tetModelMesh->GetNodeList().GetHeadPosition(); Pos != NULL; ) 
	{
		QMeshNode* node = (QMeshNode*)(_tetModelMesh->GetNodeList().GetNext(Pos));
		if (node->_isBoundaryNode == true)
		{
			counter_boundaryPoint++;
			//face!
			//I. if it is in collision last time.
			//	1. when the point is on the outside of collided face but it is beyond a threshold (not collision at all), we still call _singlePntQuery to see whether it is truly collision-free
			//	2. when the point is on the outside of collided face but it is within a threshold, we keep the old spring and not call _singlePntQuery
			//	3. if it is on the same side of collided face, then we keep on to _singlePntQuery
			//II. if it is not in collision last time, directly call _singlePntQuery


			//II. last time, if it is not in collision, we have nothing to keep, just run the normal function
			if (node->isCollided_last == false)
			{
				//check this point's collision result
				bool _checkResult = _singlePntQuery(node);


				if (_checkResult)
				{
					
					node->CalNormal();
					Eigen::Vector3d normal;
					Eigen::Vector3d xAxis = {1.0,0.0,0.0};

					node->GetNormal(normal);
					if (fabs(normal.dot(xAxis)) > 0.3)
					{
						counter_collidedPoint++;
						node->isCollided = true;
					}
					else {
						node->isCollided = false;
					}

					if(node->isFixed) node->isCollided = false;

				}
				else { node->isCollided = false; }

				//if collided, update correspondence
				if (node->isCollided == true)
				{
					//for each collided point
					if (node->_collidedRealTetraList.size() != 1)
					{
						qDebug("(%d) (from zero) collided tet number is bigger than 1, error...", eleIndex);
					}

					for (auto it = node->_collidedRealTetraList.begin(); it != node->_collidedRealTetraList.end(); ++it)
					{
						double pos[3], norm[3];
						node->GetCoord3D(pos[0], pos[1], pos[2]);
						node->GetNormal(norm[0], norm[1], norm[2]);
						bool _intersectedFace = false;
						Ray _normal2;
						int _ringNum = 0;
						_normal2.Origin = Eigen::Vector3d(pos[0], pos[1], pos[2]);

						//here normal direction is in-ward normal
						_normal2.Dir = Eigen::Vector3d(norm[0], norm[1], norm[2]);

						QMeshTetra* _collidedTet = TetraPtrArray[(*it)];

						//First, collect all two-ring boundary faces around the boundary face of _collidedTet. Then check the intersection
						//using std::set
						//To be more specific, face's edgelist -> facelist -> edgelist -> facelist
						std::vector<QMeshFace*> thisShouldVisitFaceVec;
						std::set<QMeshFace*> nextShouldVisitFaceSet;
						std::set<QMeshFace*> alreadyVisitedFaceSet;
						QMeshFace* _boundaryFace;
						bool _flag1 = false;

						//find the boundary tet first one-ring, then second-ring, then three-ring...until find the boundary tet
						auto _boundaryTetSet = _collidedTet->CalBoundaryTetList();
						//qDebug("%d boundary set num: %d", eleIndex, _boundaryTetSet.size());

						//acquire the original of flooding faceList
						for (auto itBoundaryTet = _boundaryTetSet.begin(); itBoundaryTet != _boundaryTetSet.end(); ++itBoundaryTet)
						{
							QMeshTetra* _bTet = (*itBoundaryTet);
							for (int i = 0; i < 4; i++)
							{
								QMeshFace* face = _bTet->GetFaceRecordPtr(i + 1);
								if (face->inner == false)
									thisShouldVisitFaceVec.push_back(face);
							}

						}

						//until the intersecting face is found, we then will jump out of loop
						while (1)
						{
							/*if(eleIndex==9)
								break;*/

							_ringNum++;
							//for each face in this loop should be visited
							for (int i = 0; i < thisShouldVisitFaceVec.size(); i++)
							{
								QMeshFace* _face = thisShouldVisitFaceVec[i];
								//if this face is intersecting with the Ray

								//boundary face.
								Eigen::Vector3d _A, _B, _C, _N;
								double u, v, t;
								_face->GetNodeRecordPtr(0)->GetCoord3D(_A[0], _A[1], _A[2]);
								_face->GetNodeRecordPtr(1)->GetCoord3D(_B[0], _B[1], _B[2]);
								_face->GetNodeRecordPtr(2)->GetCoord3D(_C[0], _C[1], _C[2]);

								bool _flag = false;
								if (node->GetIndexNo() == _face->GetNodeRecordPtr(0)->GetIndexNo() || node->GetIndexNo() == _face->GetNodeRecordPtr(1)->GetIndexNo() || node->GetIndexNo() == _face->GetNodeRecordPtr(2)->GetIndexNo())
								{
									//if the checking point is one of the checking triangle, then ignore this
								}
								else {
									//if not, directly check whether this is intersecting with triangle.
									_flag = _tetModelMesh->_intersect_triangle(_normal2, _A, _B, _C, t, u, v, _N);
								}


								if (_flag == true)
								{


									_intersectedFace = true;
									//qDebug("(%4d) node collided with face (%4d)", eleIndex, _face->GetIndexNo());
									node->_collidedFacePtr = _face;
									node->_barycentricCoord[0] = 1 - u - v;
									node->_barycentricCoord[1] = u;
									node->_barycentricCoord[2] = v;
									////debug usage: why intersecting face is so far
									//Eigen::Vector3d originPnt(_normal.Origin);
									//if ((originPnt - _A).norm() > 10)
									//{
									//	qDebug("pos: %lf %lf %lf", originPnt[0], originPnt[1], originPnt[2]);
									//	qDebug("(%4d) node collided too far", eleIndex);
									//	qDebug("%d boundary set num: %d", eleIndex, _boundaryTetSet.size());
									//}

									//node coorespondence finding
									node->_rayOrigin = _normal2.Origin;
									node->_rayDir = _normal2.Dir;
									node->_collidedT = t;


									//delete the wrong spring
									Eigen::Vector3d distanceVec = node->_rayDir * node->_collidedT;
									//qDebug("distance norm is %lf", distanceVec.norm());
									if (distanceVec.norm() >= threshold_selfCollisionSpring_maximum)
									{
										_tetModelMesh->collidedPntNum--;
										node->isCollided = false;
									}


									//Add correspondence
									QMeshNode* correNodeStart = new QMeshNode;
									QMeshNode* correNodeEnd = new QMeshNode;
									Eigen::Vector3d corre1, corre2;
									node->GetCoord3D(corre1[0], corre1[1], corre1[2]);
									correNodeStart->SetCoord3D(corre1[0], corre1[1], corre1[2]);
									_corresPatch_selfCollision->GetNodeList().AddTail(correNodeStart);

									corre2 = node->_rayOrigin + node->_rayDir * (node->_collidedT + coeff_extension_spring);
									correNodeEnd->SetCoord3D(corre2[0], corre2[1], corre2[2]);
									_corresPatch_selfCollision->GetNodeList().AddTail(correNodeEnd);

									QMeshEdge* correEdge = new QMeshEdge;
									correEdge->_selfCollisionType = 0;
									correEdge->SetStartPoint(correNodeStart);
									correEdge->SetEndPoint(correNodeEnd);
									_corresPatch_selfCollision->GetEdgeList().AddTail(correEdge);


									////Add colliding bounding box
									
									

									if (node->_collidedRealTreeLeafNodeIndexList.empty() == false)
									{
										
										for (auto it2 = node->_collidedRealTreeLeafNodeIndexList.begin(); it2 != node->_collidedRealTreeLeafNodeIndexList.end(); ++it2)
										{
											_addSingleAABBboundingBox_debugQuery(_tree->nodes[(*it2)].collisionBox, _debugPatch);
										}

										for (auto it2 = node->_collidedRealTetraList.begin(); it2 != node->_collidedRealTetraList.end(); ++it2)
										{
											_addSingleTetrahedron_debugQuery(TetraPtrArray[(*it2)], _debugPatch_tetDraw);
										}

									}

									/*if(t>10)
										qDebug("(%4d) node collided too far", eleIndex);*/
										//auto _reference = node->_rayOrigin + node->_rayDir * (node->_collidedT);

										//node->SetCoord3D(_reference[0], _reference[1], _reference[2]);

									break;
								}
								else {
									//if (eleIndex == 9)
									//{

									//	/*qDebug("9th point coordinate: %lf %lf %lf", pos[0], pos[1], pos[2]);
									//	qDebug("t: %lf, u: %lf, v: %lf",t,u,v);*/
									//}
									//qDebug("(%4d) node not collided neighbours", eleIndex);
								}



								//if not, prepare the data for next ring searching
								auto _oneRingFaceSet = _face->GetNeighbourFacesOnSurface();
								for (auto it2 = _oneRingFaceSet.begin(); it2 != _oneRingFaceSet.end(); ++it2)
								{
									nextShouldVisitFaceSet.insert((*it2));
								}

								alreadyVisitedFaceSet.insert(_face);

							}

							if (_intersectedFace == true)
								break;
							else {
								/*if(_ringNum%100==0)
									qDebug("Index: %d", _ringNum);*/
								if (_ringNum >= 10)
								{
									node->isCollided = false;
									qDebug("\n*************************\n\n\n\nERROR Jump Out\n\n");
									break;
								}

							}

							//delete the extra elements in nextShouldVisitFaceSet comparing to alreadyVisitedFaceSet to form thisShouldVisitFaceVec
							thisShouldVisitFaceVec.clear();
							for (auto it3 = nextShouldVisitFaceSet.begin(); it3 != nextShouldVisitFaceSet.end(); ++it3)
							{
								if (alreadyVisitedFaceSet.count((*it3)) == 0)
								{
									thisShouldVisitFaceVec.push_back(*it3);
								}
							}
							nextShouldVisitFaceSet.clear();

						}


						if (_intersectedFace == true)
						{
							node->_collidedFaceLastPtr = node->_collidedFacePtr;
							break;
						}

					}


				}

			}
			//I. if last time is in collision, we will check whether we should keep old spring even if it is not in any tet
			/// Not debug below
			else
			{

				//check whether this node is still in collision
				//if yes, update spring location
				bool _checkResult = _singlePntQuery(node);
				if (_checkResult == true)
				{
					node->isCollided = true;
					node->keepOldSpring = false;		//update correspondence
					counter_collidedPoint++;

					node->CalNormal();
					Eigen::Vector3d normal;
					Eigen::Vector3d xAxis = { 1.0,0.0,0.0 };

					node->GetNormal(normal);
					if (fabs(normal.dot(xAxis)) > 0.3)
					{
						
					}
					else {
						node->isCollided = false;
						continue;
					}
					if (node->isFixed) node->isCollided = false;


					//for each collided point
					if (node->_collidedRealTetraList.size() != 1)
					{
						qDebug("(%d) (from zero) collided tet number is bigger than 1, error...", eleIndex);
					}

					for (auto it = node->_collidedRealTetraList.begin(); it != node->_collidedRealTetraList.end(); ++it)
					{
						double pos[3], norm[3];
						node->GetCoord3D(pos[0], pos[1], pos[2]);
						node->GetNormal(norm[0], norm[1], norm[2]);
						bool _intersectedFace = false;
						Ray _normal2;
						int _ringNum = 0;
						_normal2.Origin = Eigen::Vector3d(pos[0], pos[1], pos[2]);

						//here normal direction is in-ward normal
						_normal2.Dir = Eigen::Vector3d(norm[0], norm[1], norm[2]);

						QMeshTetra* _collidedTet = TetraPtrArray[(*it)];

						//First, collect all two-ring boundary faces around the boundary face of _collidedTet. Then check the intersection
						//using std::set
						//To be more specific, face's edgelist -> facelist -> edgelist -> facelist
						std::vector<QMeshFace*> thisShouldVisitFaceVec;
						std::set<QMeshFace*> nextShouldVisitFaceSet;
						std::set<QMeshFace*> alreadyVisitedFaceSet;
						QMeshFace* _boundaryFace;
						bool _flag1 = false;

						//find the boundary tet first one-ring, then second-ring, then three-ring...until find the boundary tet
						auto _boundaryTetSet = _collidedTet->CalBoundaryTetList();
						//qDebug("%d boundary set num: %d", eleIndex, _boundaryTetSet.size());

						//acquire the original of flooding faceList
						for (auto itBoundaryTet = _boundaryTetSet.begin(); itBoundaryTet != _boundaryTetSet.end(); ++itBoundaryTet)
						{
							QMeshTetra* _bTet = (*itBoundaryTet);
							for (int i = 0; i < 4; i++)
							{
								QMeshFace* face = _bTet->GetFaceRecordPtr(i + 1);
								if (face->inner == false)
									thisShouldVisitFaceVec.push_back(face);
							}

						}

						//until the intersecting face is found, we then will jump out of loop
						while (1)
						{
							/*if(eleIndex==9)
								break;*/

							_ringNum++;
							//for each face in this loop should be visited
							for (int i = 0; i < thisShouldVisitFaceVec.size(); i++)
							{
								QMeshFace* _face = thisShouldVisitFaceVec[i];
								//if this face is intersecting with the Ray

								//boundary face.
								Eigen::Vector3d _A, _B, _C, _N;
								double u, v, t;
								_face->GetNodeRecordPtr(0)->GetCoord3D(_A[0], _A[1], _A[2]);
								_face->GetNodeRecordPtr(1)->GetCoord3D(_B[0], _B[1], _B[2]);
								_face->GetNodeRecordPtr(2)->GetCoord3D(_C[0], _C[1], _C[2]);

								bool _flag = false;
								if (node->GetIndexNo() == _face->GetNodeRecordPtr(0)->GetIndexNo() || node->GetIndexNo() == _face->GetNodeRecordPtr(1)->GetIndexNo() || node->GetIndexNo() == _face->GetNodeRecordPtr(2)->GetIndexNo())
								{
									//if the checking point is one of the checking triangle, then ignore this
								}
								else {
									//if not, directly check whether this is intersecting with triangle.
									_flag = _tetModelMesh->_intersect_triangle(_normal2, _A, _B, _C, t, u, v, _N);
								}


								if (_flag == true)
								{


									_intersectedFace = true;
									//qDebug("(%4d) node collided with face (%4d)", eleIndex, _face->GetIndexNo());
									node->_collidedFacePtr = _face;
									node->_barycentricCoord[0] = 1 - u - v;
									node->_barycentricCoord[1] = u;
									node->_barycentricCoord[2] = v;
									////debug usage: why intersecting face is so far
									//Eigen::Vector3d originPnt(_normal.Origin);
									//if ((originPnt - _A).norm() > 10)
									//{
									//	qDebug("pos: %lf %lf %lf", originPnt[0], originPnt[1], originPnt[2]);
									//	qDebug("(%4d) node collided too far", eleIndex);
									//	qDebug("%d boundary set num: %d", eleIndex, _boundaryTetSet.size());
									//}

									//node coorespondence finding
									node->_rayOrigin = _normal2.Origin;
									node->_rayDir = _normal2.Dir;
									node->_collidedT = t;


									//delete the wrong spring
									Eigen::Vector3d distanceVec = node->_rayDir * node->_collidedT;
									//qDebug("distance norm is %lf", distanceVec.norm());
									if (distanceVec.norm() >= threshold_selfCollisionSpring_maximum)
									{
										_tetModelMesh->collidedPntNum--;
										node->isCollided = false;
									}


									//Add Correspondence
									QMeshNode* correNodeStart = new QMeshNode;
									QMeshNode* correNodeEnd = new QMeshNode;
									Eigen::Vector3d corre1, corre2;
									node->GetCoord3D(corre1[0], corre1[1], corre1[2]);
									correNodeStart->SetCoord3D(corre1[0], corre1[1], corre1[2]);
									_corresPatch_selfCollision->GetNodeList().AddTail(correNodeStart);

									corre2 = node->_rayOrigin + node->_rayDir * (node->_collidedT + coeff_extension_spring);
									correNodeEnd->SetCoord3D(corre2[0], corre2[1], corre2[2]);
									_corresPatch_selfCollision->GetNodeList().AddTail(correNodeEnd);

									QMeshEdge* correEdge = new QMeshEdge;
									correEdge->_selfCollisionType = 1;
									correEdge->SetStartPoint(correNodeStart);
									correEdge->SetEndPoint(correNodeEnd);
									_corresPatch_selfCollision->GetEdgeList().AddTail(correEdge);


									//Add colliding bounding box
									for (auto it2 = node->_collidedRealTreeLeafNodeIndexList.begin(); it2 != node->_collidedRealTreeLeafNodeIndexList.end(); ++it2)
									{
										_addSingleAABBboundingBox_debugQuery(_tree->nodes[(*it2)].collisionBox, _debugPatch);
									}

									for (auto it2 = node->_collidedRealTetraList.begin(); it2 != node->_collidedRealTetraList.end(); ++it2)
									{
										_addSingleTetrahedron_debugQuery(TetraPtrArray[(*it2)], _debugPatch_tetDraw);
									}
									/*if(t>10)
										qDebug("(%4d) node collided too far", eleIndex);*/
										//auto _reference = node->_rayOrigin + node->_rayDir * (node->_collidedT);

										//node->SetCoord3D(_reference[0], _reference[1], _reference[2]);

									break;
								}
								else {
									//if (eleIndex == 9)
									//{

									//	/*qDebug("9th point coordinate: %lf %lf %lf", pos[0], pos[1], pos[2]);
									//	qDebug("t: %lf, u: %lf, v: %lf",t,u,v);*/
									//}
									//qDebug("(%4d) node not collided neighbours", eleIndex);
								}



								//if not, prepare the data for next ring searching
								auto _oneRingFaceSet = _face->GetNeighbourFacesOnSurface();
								for (auto it2 = _oneRingFaceSet.begin(); it2 != _oneRingFaceSet.end(); ++it2)
								{
									nextShouldVisitFaceSet.insert((*it2));
								}

								alreadyVisitedFaceSet.insert(_face);

							}

							if (_intersectedFace == true)
								break;
							else
							{
								if (_ringNum > 10)
								{
									qDebug("\n**************************\nERROR!\n**************************\n");
									node->isCollided = false;
									break;
								}
								/*if(_ringNum%100==0)
									qDebug("Index: %d", _ringNum);*/
									/*if (_ringNum >= 10)
										break;*/
							}

							//delete the extra elements in nextShouldVisitFaceSet comparing to alreadyVisitedFaceSet to form thisShouldVisitFaceVec
							thisShouldVisitFaceVec.clear();
							for (auto it3 = nextShouldVisitFaceSet.begin(); it3 != nextShouldVisitFaceSet.end(); ++it3)
							{
								if (alreadyVisitedFaceSet.count((*it3)) == 0)
								{
									thisShouldVisitFaceVec.push_back(*it3);
								}
							}
							nextShouldVisitFaceSet.clear();

						}

						if (_intersectedFace == true)
						{
							node->_collidedFaceLastPtr = node->_collidedFacePtr;
							break;
						}



					}


					counter_collidedPoint++;
					continue;
				}


				//below is else's situation

				//check the distance between node and its correspondence on the collided face (now is just free of intersection now)
				//we need to protect this correpondence a little bit to give the constraints
				Eigen::Vector3d nodePos;
				Eigen::Vector3d tempPos;
				Eigen::Vector3d correspondenceNodePos = { 0.0,0.0,0.0 };
				node->GetCoord3D(nodePos[0], nodePos[1], nodePos[2]);


				node->_collidedFacePtr->GetNodePos(0, tempPos[0], tempPos[1], tempPos[2]);
				correspondenceNodePos = correspondenceNodePos + node->_barycentricCoord[0] * tempPos;


				node->_collidedFacePtr->GetNodePos(1, tempPos[0], tempPos[1], tempPos[2]);
				correspondenceNodePos = correspondenceNodePos + node->_barycentricCoord[1] * tempPos;

				node->_collidedFacePtr->GetNodePos(2, tempPos[0], tempPos[1], tempPos[2]);
				correspondenceNodePos = correspondenceNodePos + node->_barycentricCoord[2] * tempPos;

				double distance = (nodePos - correspondenceNodePos).norm();
				double threshold = coeff_extension_epsilon;
				if (distance > threshold)
				{
					//discard spring 
					node->isCollided = false;
					qDebug("\n*********************\nOutside Boundary 3, discard it\n*********************\n");
					//Sleep(50);
				}
				else {
					//keep spring but dont update the spring location
					node->isCollided = true;
					counter_collidedPoint++;
					double nx[3];
					node->GetNormal(node->_rayDir[0], node->_rayDir[1], node->_rayDir[2]);	//update spring direction only
					qDebug("\n*********************\nInside Boundary 3, Keep it\n*********************\n");


					//Add Correspondence
					QMeshNode* correNodeStart = new QMeshNode;
					QMeshNode* correNodeEnd = new QMeshNode;
					Eigen::Vector3d corre1, corre2;
					node->GetCoord3D(corre1[0], corre1[1], corre1[2]);
					correNodeStart->SetCoord3D(corre1[0], corre1[1], corre1[2]);
					_corresPatch_selfCollision->GetNodeList().AddTail(correNodeStart);

					corre2 = node->_rayOrigin + node->_rayDir * (node->_collidedT + coeff_extension_spring);
					correNodeEnd->SetCoord3D(corre2[0], corre2[1], corre2[2]);
					_corresPatch_selfCollision->GetNodeList().AddTail(correNodeEnd);

					QMeshEdge* correEdge = new QMeshEdge;
					correEdge->_selfCollisionType = 1;
					correEdge->SetStartPoint(correNodeStart);
					correEdge->SetEndPoint(correNodeEnd);
					_corresPatch_selfCollision->GetEdgeList().AddTail(correEdge);


					//Sleep(50);
				}

			}

			//eleIndex++;


		}
	}

	

	int collidedNum_local = 0;
	for (GLKPOSITION Pos = _tetModelMesh->GetNodeList().GetHeadPosition(); Pos != NULL; )
	{
		QMeshNode* node = (QMeshNode*)(_tetModelMesh->GetNodeList().GetNext(Pos));
		if (node->isCollided == true)
			collidedNum_local++;
	}
	_tetModelMesh->collidedPntNum = collidedNum_local;
	init_softFingerSelfCollision = true;
}

void AABBManager::SelfCollisionDetectionCorrespondenceChecking(void)
{
	static int counter_collided = 0;

	if (counter_collided != 0)
	{
		//update the correspondence patch
		int tetNum = _debugPatch->GetTetraNumber();
		int edgeNum = _debugPatch->GetEdgeNumber();
		qDebug("Tet num is %d, while edge num is %d", tetNum, edgeNum);
		//clean the collided bounding box
		_debugPatch->ClearAllTet();
		_debugPatch->startIndexfromPos = NULL;
		_debugPatch->startIndexfromPos_int = 0;

		_corresPatch_selfCollision->ClearAllTet();

	}

	//debug 
	if (counter_collided == 0)
	{
		_debugPatch = new QMeshPatch;
		_corresPatch_selfCollision = new QMeshPatch;
	}

	//------------------------------------------------------------------------------------------------
	//	Step 1: mark the boundary points which are lied on the boundary surface
	// first check whether it is in collision
	int eleIndex = 0;
	for (GLKPOSITION Pos = _tetModelMesh->GetNodeList().GetHeadPosition(); Pos != NULL; eleIndex++) {
		QMeshNode* node = (QMeshNode*)(_tetModelMesh->GetNodeList().GetNext(Pos));

		node->isCollided_last = node->isCollided;	//last iteration
		node->keepOldSpring = false;				//default: dont keep any spring attached on this point and its correspondence
		node->_isBoundaryNode = false;
		node->isCollided = false;

		node->CalNormal();
	}



	for (GLKPOSITION Pos = _tetModelMesh->GetFaceList().GetHeadPosition(); Pos != NULL; eleIndex++) {
		QMeshFace* face = (QMeshFace*)(_tetModelMesh->GetFaceList().GetNext(Pos));
		if (face->GetLeftTetra() == NULL || face->GetRightTetra() == NULL)
		{
			/*qDebug("Surface Boundary Face Found....\n");*/
			for (int i = 0; i < 3; i++) { face->GetNodeRecordPtr(i)->_isBoundaryNode = true; }
			face->CalPlaneEquation();

		}

	}

	qDebug("Boundary Point Set Established...");

	//------------------------------------------------------------------------------------------------
	//	Step 2: for each poitn on the boundary surface, we just check the collision result
	eleIndex = 0;
	double pos[3];
	eleIndex = 0;
	long startT = clock();
	int counter_boundaryPoint = 0;
	int counter_collidedPoint = 0;


	//face!

	//I. if it is in collision last time.
	//	1. when the point is on the outside of collided face but it is beyond a threshold (not collision at all), we still call _singlePntQuery to see whether it is truly collision-free
	//	2. when the point is on the outside of collided face but it is within a threshold, we keep the old spring and not call _singlePntQuery
	//	3. if it is on the same side of collided face, then we keep on to _singlePntQuery
	//II. if it is not in collision last time, directly call _singlePntQuery

	for (GLKPOSITION Pos = _tetModelMesh->GetNodeList().GetHeadPosition(); Pos != NULL; ) {
		QMeshNode* node = (QMeshNode*)(_tetModelMesh->GetNodeList().GetNext(Pos));
		if (node->_isBoundaryNode == true)
		{
			counter_boundaryPoint++;
			//face!
			//I. if it is in collision last time.
			//	1. when the point is on the outside of collided face but it is beyond a threshold (not collision at all), we still call _singlePntQuery to see whether it is truly collision-free
			//	2. when the point is on the outside of collided face but it is within a threshold, we keep the old spring and not call _singlePntQuery
			//	3. if it is on the same side of collided face, then we keep on to _singlePntQuery
			//II. if it is not in collision last time, directly call _singlePntQuery


			//II. last time, if it is not in collision, we have nothing to keep, just run the normal function
			if (node->isCollided_last == false)
			{
				//check this point's collision result
				bool _checkResult = _singlePntQuery(node);
				if (_checkResult)
				{
					counter_collidedPoint++;
					node->isCollided = true;
				}
				else { node->isCollided = false; }

				//if collided, update correspondence
				if (node->isCollided == true)
				{
					//for each collided point
					if (node->_collidedRealTetraList.size() != 1)
					{
						qDebug("(%d) (from zero) collided tet number is bigger than 1, error...", eleIndex);
					}

					for (auto it = node->_collidedRealTetraList.begin(); it != node->_collidedRealTetraList.end(); ++it)
					{
						double pos[3], norm[3];
						node->GetCoord3D(pos[0], pos[1], pos[2]);
						node->GetNormal(norm[0], norm[1], norm[2]);
						bool _intersectedFace = false;
						Ray _normal2;
						int _ringNum = 0;
						_normal2.Origin = Eigen::Vector3d(pos[0], pos[1], pos[2]);

						//here normal direction is in-ward normal
						_normal2.Dir = Eigen::Vector3d(norm[0], norm[1], norm[2]);

						QMeshTetra* _collidedTet = TetraPtrArray[(*it)];

						//First, collect all two-ring boundary faces around the boundary face of _collidedTet. Then check the intersection
						//using std::set
						//To be more specific, face's edgelist -> facelist -> edgelist -> facelist
						std::vector<QMeshFace*> thisShouldVisitFaceVec;
						std::set<QMeshFace*> nextShouldVisitFaceSet;
						std::set<QMeshFace*> alreadyVisitedFaceSet;
						QMeshFace* _boundaryFace;
						bool _flag1 = false;

						//find the boundary tet first one-ring, then second-ring, then three-ring...until find the boundary tet
						auto _boundaryTetSet = _collidedTet->CalBoundaryTetList();
						//qDebug("%d boundary set num: %d", eleIndex, _boundaryTetSet.size());

						//acquire the original of flooding faceList
						for (auto itBoundaryTet = _boundaryTetSet.begin(); itBoundaryTet != _boundaryTetSet.end(); ++itBoundaryTet)
						{
							QMeshTetra* _bTet = (*itBoundaryTet);
							for (int i = 0; i < 4; i++)
							{
								QMeshFace* face = _bTet->GetFaceRecordPtr(i + 1);
								if (face->inner == false)
									thisShouldVisitFaceVec.push_back(face);
							}

						}

						//until the intersecting face is found, we then will jump out of loop
						while (1)
						{
							/*if(eleIndex==9)
								break;*/

							_ringNum++;
							//for each face in this loop should be visited
							for (int i = 0; i < thisShouldVisitFaceVec.size(); i++)
							{
								QMeshFace* _face = thisShouldVisitFaceVec[i];
								//if this face is intersecting with the Ray

								//boundary face.
								Eigen::Vector3d _A, _B, _C, _N;
								double u, v, t;
								_face->GetNodeRecordPtr(0)->GetCoord3D(_A[0], _A[1], _A[2]);
								_face->GetNodeRecordPtr(1)->GetCoord3D(_B[0], _B[1], _B[2]);
								_face->GetNodeRecordPtr(2)->GetCoord3D(_C[0], _C[1], _C[2]);

								bool _flag = false;
								if (node->GetIndexNo() == _face->GetNodeRecordPtr(0)->GetIndexNo() || node->GetIndexNo() == _face->GetNodeRecordPtr(1)->GetIndexNo() || node->GetIndexNo() == _face->GetNodeRecordPtr(2)->GetIndexNo())
								{
									//if the checking point is one of the checking triangle, then ignore this
								}
								else {
									//if not, directly check whether this is intersecting with triangle.
									_flag = _tetModelMesh->_intersect_triangle(_normal2, _A, _B, _C, t, u, v, _N);
								}


								if (_flag == true)
								{


									_intersectedFace = true;
									//qDebug("(%4d) node collided with face (%4d)", eleIndex, _face->GetIndexNo());
									node->_collidedFacePtr = _face;
									node->_barycentricCoord[0] = 1 - u - v;
									node->_barycentricCoord[1] = u;
									node->_barycentricCoord[2] = v;
									////debug usage: why intersecting face is so far
									//Eigen::Vector3d originPnt(_normal.Origin);
									//if ((originPnt - _A).norm() > 10)
									//{
									//	qDebug("pos: %lf %lf %lf", originPnt[0], originPnt[1], originPnt[2]);
									//	qDebug("(%4d) node collided too far", eleIndex);
									//	qDebug("%d boundary set num: %d", eleIndex, _boundaryTetSet.size());
									//}

									//node coorespondence finding
									node->_rayOrigin = _normal2.Origin;
									node->_rayDir = _normal2.Dir;
									node->_collidedT = t;


									//delete the wrong spring
									Eigen::Vector3d distanceVec = node->_rayDir * node->_collidedT;
									//qDebug("distance norm is %lf", distanceVec.norm());
									if (distanceVec.norm() >= threshold_selfCollisionSpring_maximum)
									{
										_tetModelMesh->collidedPntNum--;
										node->isCollided = false;
									}


									//To delete
									QMeshNode* correNodeStart = new QMeshNode;
									QMeshNode* correNodeEnd = new QMeshNode;
									Eigen::Vector3d corre1, corre2;
									node->GetCoord3D(corre1[0], corre1[1], corre1[2]);
									correNodeStart->SetCoord3D(corre1[0], corre1[1], corre1[2]);
									_corresPatch_selfCollision->GetNodeList().AddTail(correNodeStart);

									corre2 = node->_rayOrigin + node->_rayDir * (node->_collidedT + coeff_extension_spring);
									correNodeEnd->SetCoord3D(corre2[0], corre2[1], corre2[2]);
									_corresPatch_selfCollision->GetNodeList().AddTail(correNodeEnd);

									QMeshEdge* correEdge = new QMeshEdge;
									correEdge->_selfCollisionType = 0;
									correEdge->SetStartPoint(correNodeStart);
									correEdge->SetEndPoint(correNodeEnd);
									_corresPatch_selfCollision->GetEdgeList().AddTail(correEdge);
									/*if(t>10)
										qDebug("(%4d) node collided too far", eleIndex);*/
										//auto _reference = node->_rayOrigin + node->_rayDir * (node->_collidedT);

										//node->SetCoord3D(_reference[0], _reference[1], _reference[2]);

									break;
								}
								else {
									//if (eleIndex == 9)
									//{

									//	/*qDebug("9th point coordinate: %lf %lf %lf", pos[0], pos[1], pos[2]);
									//	qDebug("t: %lf, u: %lf, v: %lf",t,u,v);*/
									//}
									//qDebug("(%4d) node not collided neighbours", eleIndex);
								}



								//if not, prepare the data for next ring searching
								auto _oneRingFaceSet = _face->GetNeighbourFacesOnSurface();
								for (auto it2 = _oneRingFaceSet.begin(); it2 != _oneRingFaceSet.end(); ++it2)
								{
									nextShouldVisitFaceSet.insert((*it2));
								}

								alreadyVisitedFaceSet.insert(_face);

							}

							if (_intersectedFace == true)
								break;
							else {
								/*if(_ringNum%100==0)
									qDebug("Index: %d", _ringNum);*/
								if (_ringNum >= 10)
								{
									node->isCollided = false;
									qDebug("\n*************************\n\n\n\nERROR Jump Out\n\n");
									break;
								}
									
							}

							//delete the extra elements in nextShouldVisitFaceSet comparing to alreadyVisitedFaceSet to form thisShouldVisitFaceVec
							thisShouldVisitFaceVec.clear();
							for (auto it3 = nextShouldVisitFaceSet.begin(); it3 != nextShouldVisitFaceSet.end(); ++it3)
							{
								if (alreadyVisitedFaceSet.count((*it3)) == 0)
								{
									thisShouldVisitFaceVec.push_back(*it3);
								}
							}
							nextShouldVisitFaceSet.clear();

						}


						if (_intersectedFace == true)
						{
							node->_collidedFaceLastPtr = node->_collidedFacePtr;
							break;
						}

					}


				}

			}
			//I. if last time is in collision, we will check whether we should keep old spring even if it is not in any tet
			/// Not debug below
			else
			{

				//check whether this node is still in collision
				//if yes, update spring location
				bool _checkResult = _singlePntQuery(node);
				if (_checkResult == true)
				{
					node->isCollided = true;
					node->keepOldSpring = false;		//update correspondence
					counter_collidedPoint++;
					//for each collided point
					if (node->_collidedRealTetraList.size() != 1)
					{
						qDebug("(%d) (from zero) collided tet number is bigger than 1, error...", eleIndex);
					}

					for (auto it = node->_collidedRealTetraList.begin(); it != node->_collidedRealTetraList.end(); ++it)
					{
						double pos[3], norm[3];
						node->GetCoord3D(pos[0], pos[1], pos[2]);
						node->GetNormal(norm[0], norm[1], norm[2]);
						bool _intersectedFace = false;
						Ray _normal2;
						int _ringNum = 0;
						_normal2.Origin = Eigen::Vector3d(pos[0], pos[1], pos[2]);

						//here normal direction is in-ward normal
						_normal2.Dir = Eigen::Vector3d(norm[0], norm[1], norm[2]);

						QMeshTetra* _collidedTet = TetraPtrArray[(*it)];

						//First, collect all two-ring boundary faces around the boundary face of _collidedTet. Then check the intersection
						//using std::set
						//To be more specific, face's edgelist -> facelist -> edgelist -> facelist
						std::vector<QMeshFace*> thisShouldVisitFaceVec;
						std::set<QMeshFace*> nextShouldVisitFaceSet;
						std::set<QMeshFace*> alreadyVisitedFaceSet;
						QMeshFace* _boundaryFace;
						bool _flag1 = false;

						//find the boundary tet first one-ring, then second-ring, then three-ring...until find the boundary tet
						auto _boundaryTetSet = _collidedTet->CalBoundaryTetList();
						//qDebug("%d boundary set num: %d", eleIndex, _boundaryTetSet.size());

						//acquire the original of flooding faceList
						for (auto itBoundaryTet = _boundaryTetSet.begin(); itBoundaryTet != _boundaryTetSet.end(); ++itBoundaryTet)
						{
							QMeshTetra* _bTet = (*itBoundaryTet);
							for (int i = 0; i < 4; i++)
							{
								QMeshFace* face = _bTet->GetFaceRecordPtr(i + 1);
								if (face->inner == false)
									thisShouldVisitFaceVec.push_back(face);
							}

						}

						//until the intersecting face is found, we then will jump out of loop
						while (1)
						{
							/*if(eleIndex==9)
								break;*/

							_ringNum++;
							//for each face in this loop should be visited
							for (int i = 0; i < thisShouldVisitFaceVec.size(); i++)
							{
								QMeshFace* _face = thisShouldVisitFaceVec[i];
								//if this face is intersecting with the Ray

								//boundary face.
								Eigen::Vector3d _A, _B, _C, _N;
								double u, v, t;
								_face->GetNodeRecordPtr(0)->GetCoord3D(_A[0], _A[1], _A[2]);
								_face->GetNodeRecordPtr(1)->GetCoord3D(_B[0], _B[1], _B[2]);
								_face->GetNodeRecordPtr(2)->GetCoord3D(_C[0], _C[1], _C[2]);

								bool _flag = false;
								if (node->GetIndexNo() == _face->GetNodeRecordPtr(0)->GetIndexNo() || node->GetIndexNo() == _face->GetNodeRecordPtr(1)->GetIndexNo() || node->GetIndexNo() == _face->GetNodeRecordPtr(2)->GetIndexNo())
								{
									//if the checking point is one of the checking triangle, then ignore this
								}
								else {
									//if not, directly check whether this is intersecting with triangle.
									_flag = _tetModelMesh->_intersect_triangle(_normal2, _A, _B, _C, t, u, v, _N);
								}


								if (_flag == true)
								{


									_intersectedFace = true;
									//qDebug("(%4d) node collided with face (%4d)", eleIndex, _face->GetIndexNo());
									node->_collidedFacePtr = _face;
									node->_barycentricCoord[0] = 1 - u - v;
									node->_barycentricCoord[1] = u;
									node->_barycentricCoord[2] = v;
									////debug usage: why intersecting face is so far
									//Eigen::Vector3d originPnt(_normal.Origin);
									//if ((originPnt - _A).norm() > 10)
									//{
									//	qDebug("pos: %lf %lf %lf", originPnt[0], originPnt[1], originPnt[2]);
									//	qDebug("(%4d) node collided too far", eleIndex);
									//	qDebug("%d boundary set num: %d", eleIndex, _boundaryTetSet.size());
									//}

									//node coorespondence finding
									node->_rayOrigin = _normal2.Origin;
									node->_rayDir = _normal2.Dir;
									node->_collidedT = t;


									//delete the wrong spring
									Eigen::Vector3d distanceVec = node->_rayDir * node->_collidedT;
									//qDebug("distance norm is %lf", distanceVec.norm());
									if (distanceVec.norm() >= threshold_selfCollisionSpring_maximum)
									{
										_tetModelMesh->collidedPntNum--;
										node->isCollided = false;
									}


									//To delete
									QMeshNode* correNodeStart = new QMeshNode;
									QMeshNode* correNodeEnd = new QMeshNode;
									Eigen::Vector3d corre1, corre2;
									node->GetCoord3D(corre1[0], corre1[1], corre1[2]);
									correNodeStart->SetCoord3D(corre1[0], corre1[1], corre1[2]);
									_corresPatch_selfCollision->GetNodeList().AddTail(correNodeStart);

									corre2 = node->_rayOrigin + node->_rayDir * (node->_collidedT + coeff_extension_spring);
									correNodeEnd->SetCoord3D(corre2[0], corre2[1], corre2[2]);
									_corresPatch_selfCollision->GetNodeList().AddTail(correNodeEnd);

									QMeshEdge* correEdge = new QMeshEdge;
									correEdge->_selfCollisionType = 1;
									correEdge->SetStartPoint(correNodeStart);
									correEdge->SetEndPoint(correNodeEnd);
									_corresPatch_selfCollision->GetEdgeList().AddTail(correEdge);
									/*if(t>10)
										qDebug("(%4d) node collided too far", eleIndex);*/
										//auto _reference = node->_rayOrigin + node->_rayDir * (node->_collidedT);

										//node->SetCoord3D(_reference[0], _reference[1], _reference[2]);

									break;
								}
								else {
									//if (eleIndex == 9)
									//{

									//	/*qDebug("9th point coordinate: %lf %lf %lf", pos[0], pos[1], pos[2]);
									//	qDebug("t: %lf, u: %lf, v: %lf",t,u,v);*/
									//}
									//qDebug("(%4d) node not collided neighbours", eleIndex);
								}



								//if not, prepare the data for next ring searching
								auto _oneRingFaceSet = _face->GetNeighbourFacesOnSurface();
								for (auto it2 = _oneRingFaceSet.begin(); it2 != _oneRingFaceSet.end(); ++it2)
								{
									nextShouldVisitFaceSet.insert((*it2));
								}

								alreadyVisitedFaceSet.insert(_face);

							}

							if (_intersectedFace == true)
								break;
							else 
							{
								if (_ringNum > 10)
								{
									qDebug("\n**************************\nERROR!\n**************************\n");
									node->isCollided = false;
									break;
								}
								/*if(_ringNum%100==0)
									qDebug("Index: %d", _ringNum);*/
									/*if (_ringNum >= 10)
										break;*/
							}

							//delete the extra elements in nextShouldVisitFaceSet comparing to alreadyVisitedFaceSet to form thisShouldVisitFaceVec
							thisShouldVisitFaceVec.clear();
							for (auto it3 = nextShouldVisitFaceSet.begin(); it3 != nextShouldVisitFaceSet.end(); ++it3)
							{
								if (alreadyVisitedFaceSet.count((*it3)) == 0)
								{
									thisShouldVisitFaceVec.push_back(*it3);
								}
							}
							nextShouldVisitFaceSet.clear();

						}

						if (_intersectedFace == true)
						{
							node->_collidedFaceLastPtr = node->_collidedFacePtr;
							break;
						}



					}


					counter_collidedPoint++;
					continue;
				}


				//check the distance between node and its correspondence on the collided face (now is just free of intersection now)
				//we need to protect this correpondence a little bit to give the constraints
				Eigen::Vector3d nodePos;
				Eigen::Vector3d tempPos;
				Eigen::Vector3d correspondenceNodePos = { 0.0,0.0,0.0 };
				node->GetCoord3D(nodePos[0], nodePos[1], nodePos[2]);


				node->_collidedFacePtr->GetNodePos(0, tempPos[0], tempPos[1], tempPos[2]);
				correspondenceNodePos = correspondenceNodePos + node->_barycentricCoord[0] * tempPos;


				node->_collidedFacePtr->GetNodePos(1, tempPos[0], tempPos[1], tempPos[2]);
				correspondenceNodePos = correspondenceNodePos + node->_barycentricCoord[1] * tempPos;

				node->_collidedFacePtr->GetNodePos(2, tempPos[0], tempPos[1], tempPos[2]);
				correspondenceNodePos = correspondenceNodePos + node->_barycentricCoord[2] * tempPos;

				double distance = (nodePos - correspondenceNodePos).norm();
				double threshold = coeff_extension_epsilon;
				if (distance > threshold)
				{
					//discard spring 
					node->isCollided = false;
					qDebug("\n*********************\nOutside Boundary 3, discard it\n*********************\n");
					//Sleep(50);
				}
				else {
					//keep spring but dont update the spring location
					node->isCollided = true;
					counter_collidedPoint++;
					double nx[3];
					node->GetNormal(node->_rayDir[0], node->_rayDir[1], node->_rayDir[2]);	//update spring direction only
					qDebug("\n*********************\nInside Boundary 3, Keep it\n*********************\n");
					//Sleep(50);
				}

			}

			//eleIndex++;


		}
	}

	_tetModelMesh->collidedPntNum = counter_collidedPoint;



	counter_collided++;
}

void AABBManager::_detectSprintCorrespondance(QMeshNode* node) {

	// make sure only single tetra detected
	if (node->_collidedRealTetraList.size() != 1) qDebug("(%d) (from zero) collided tet number is bigger than 1, error...", node->GetIndexNo());

	for (auto it = node->_collidedRealTetraList.begin(); it != node->_collidedRealTetraList.end(); ++it)
	{
		double pos[3], norm[3];
		node->GetCoord3D(pos[0], pos[1], pos[2]);
		node->GetNormal(norm[0], norm[1], norm[2]);
		bool _intersectedFace = false;
		Ray _normal2;
		int _ringNum = 0;
		_normal2.Origin = Eigen::Vector3d(pos[0], pos[1], pos[2]);

		//here normal direction is in-ward normal
		_normal2.Dir = Eigen::Vector3d(norm[0], norm[1], norm[2]);

		QMeshTetra* _collidedTet = TetraPtrArray[(*it)];

		//First, collect all two-ring boundary faces around the boundary face of _collidedTet. Then check the intersection
		//using std::set
		//To be more specific, face's edgelist -> facelist -> edgelist -> facelist
		std::vector<QMeshFace*> thisShouldVisitFaceVec;
		std::set<QMeshFace*> nextShouldVisitFaceSet;
		std::set<QMeshFace*> alreadyVisitedFaceSet;
		QMeshFace* _boundaryFace;
		bool _flag1 = false;

		//find the boundary tet first one-ring, then second-ring, then three-ring...until find the boundary tet
		auto _boundaryTetSet = _collidedTet->CalBoundaryTetList();
		//qDebug("%d boundary set num: %d", eleIndex, _boundaryTetSet.size());

		//acquire the original of flooding faceList
		for (auto itBoundaryTet = _boundaryTetSet.begin(); itBoundaryTet != _boundaryTetSet.end(); ++itBoundaryTet)
		{
			QMeshTetra* _bTet = (*itBoundaryTet);
			for (int i = 0; i < 4; i++)
			{
				QMeshFace* face = _bTet->GetFaceRecordPtr(i + 1);
				if (face->inner == false)
					thisShouldVisitFaceVec.push_back(face);
			}

		}

		//until the intersecting face is found, we then will jump out of loop
		while (1)
		{
			/*if(eleIndex==9)
				break;*/

			_ringNum++;
			//for each face in this loop should be visited
			for (int i = 0; i < thisShouldVisitFaceVec.size(); i++)
			{
				QMeshFace* _face = thisShouldVisitFaceVec[i];
				//if this face is intersecting with the Ray

				//boundary face.
				Eigen::Vector3d _A, _B, _C, _N;
				double u, v, t;
				_face->GetNodeRecordPtr(0)->GetCoord3D(_A[0], _A[1], _A[2]);
				_face->GetNodeRecordPtr(1)->GetCoord3D(_B[0], _B[1], _B[2]);
				_face->GetNodeRecordPtr(2)->GetCoord3D(_C[0], _C[1], _C[2]);

				bool _flag = false;
				if (node->GetIndexNo() == _face->GetNodeRecordPtr(0)->GetIndexNo() || node->GetIndexNo() == _face->GetNodeRecordPtr(1)->GetIndexNo() || node->GetIndexNo() == _face->GetNodeRecordPtr(2)->GetIndexNo())
				{
					//if the checking point is one of the checking triangle, then ignore this
				}
				else {
					//if not, directly check whether this is intersecting with triangle.
					_flag = this->_intersect_triangle(_normal2, _A, _B, _C, t, u, v, _N);
				}


				if (_flag == true)
				{


					_intersectedFace = true;
					//qDebug("(%4d) node collided with face (%4d)", eleIndex, _face->GetIndexNo());
					node->_collidedFacePtr = _face;
					node->_barycentricCoord[0] = 1 - u - v;
					node->_barycentricCoord[1] = u;
					node->_barycentricCoord[2] = v;
					////debug usage: why intersecting face is so far
					//Eigen::Vector3d originPnt(_normal.Origin);
					//if ((originPnt - _A).norm() > 10)
					//{
					//	qDebug("pos: %lf %lf %lf", originPnt[0], originPnt[1], originPnt[2]);
					//	qDebug("(%4d) node collided too far", eleIndex);
					//	qDebug("%d boundary set num: %d", eleIndex, _boundaryTetSet.size());
					//}

					//node coorespondence finding
					node->_rayOrigin = _normal2.Origin;
					node->_rayDir = _normal2.Dir;
					node->_collidedT = t;


					//delete the wrong spring
					Eigen::Vector3d distanceVec = node->_rayDir * node->_collidedT;
					//qDebug("distance norm is %lf", distanceVec.norm());
					if (distanceVec.norm() >= threshold_selfCollisionSpring_maximum)
						node->isCollided = false;
					
					break;
				}
				
				//if not, prepare the data for next ring searching
				auto _oneRingFaceSet = _face->GetNeighbourFacesOnSurface();
				for (auto it2 = _oneRingFaceSet.begin(); it2 != _oneRingFaceSet.end(); ++it2)
				{
					nextShouldVisitFaceSet.insert((*it2));
				}

				alreadyVisitedFaceSet.insert(_face);

			}

			if (_intersectedFace == true)
				break;

			//delete the extra elements in nextShouldVisitFaceSet comparing to alreadyVisitedFaceSet to form thisShouldVisitFaceVec
			thisShouldVisitFaceVec.clear();
			for (auto it3 = nextShouldVisitFaceSet.begin(); it3 != nextShouldVisitFaceSet.end(); ++it3)
			{
				if (alreadyVisitedFaceSet.count((*it3)) == 0)
				{
					thisShouldVisitFaceVec.push_back(*it3);
				}
			}
			nextShouldVisitFaceSet.clear();

		}


		if (_intersectedFace == true)
		{
			node->_collidedFaceLastPtr = node->_collidedFacePtr;
			break;
		}

	}
}



//Query if this point is inside the tetrahedrons of model
//Basically, we will find out all the collided leaf nodes in the AABB tree, and check whether there is a collision
//There will be two possible situations:
//	1. _node is inside the collision box but not inside the tetrahedron the collision box contains
//  2. _node is inside the collision box and also inside the tetrahedron the collision box contains
//			2.1 _node is one of the tetrahedron node -> ignore
//			2.2 _node is not one of the tetrahedron node -> SELF COLLIDED POINT -> What we want!

//To be more detailed, we will check 

//para: @_node:  node to be checked against the whole model
//return: @(bool): true -> collision exist
bool AABBManager::_singlePntQuery(QMeshNode* _node)
{
	//------------------------------------------------------------------------------------------------
	//	Step 1: traverse the tree to find out all the possible collided tree nodes
	_node->_collidedTetraList.clear();					//clean the embeded potential collided tetralist 
	_node->_collidedRealTetraList.clear();				//clean the embeded real collided tetralist
	_node->_collidedTreeLeafNodeIndexList.clear();		//clean
	_node->_collidedRealTreeLeafNodeIndexList.clear();	//clean
	_node->isCollided = false;							//reset collision flag

	double pos[3];
	_node->GetCoord3D(pos[0],pos[1],pos[2]);
	//check if root tree node is inside the bounding box
	bool _collided = _isCollided(pos, _tree->nodes[_tree->rootIndex].collisionBox);
	if (_collided == false)
	{
		qDebug("No Collision....\n");
		return false;
	}
	
	//check left child and right child, and further push the potential tetra into _collidedTetraList.
	_recursiveQuery(_node, _tree->rootIndex, 1, 1);
	_recursiveQuery(_node, _tree->rootIndex, 2, 1);

	//
	//------------------------------------------------------------------------------------------------

	//------------------------------------------------------------------------------------------------
	//	Step 2: post-processing to see whether there is actually a collision 
	//There will be two possible situations:
	//	1. _node is inside the collision box but not inside the tetrahedron the collision box contains
	//  2. _node is inside the collision box and also inside the tetrahedron the collision box contains
	//			2.1 _node is one of the tetrahedron node -> ignore
	//			2.2 _node is not one of the tetrahedron node -> SELF COLLIDED POINT -> What we want!
	// So the collision will be: inside the tetrahedron and this tetrahedron should not contain our query node
	
	_node->GetCoord3D(pos[0],pos[1],pos[2]);
	auto leafNodeIter = _node->_collidedTreeLeafNodeIndexList.begin();
	for (auto it = _node->_collidedTetraList.begin(); it != _node->_collidedTetraList.end(); ++it, ++leafNodeIter) {
		QMeshTetra* _potentialTet = TetraPtrArray[(*it)];
		//check  whether _node is inside this tetrahedron 
		//bool _inside = _potentialTet->_judgeInsideTet(pos);	//true->inside the tet
		bool _inside = this->_judgeInsideTet(pos, _potentialTet);
		if (_inside == false) continue;
		//judge whether this _node is one of the tetrahedron points
		//bool _inTetPntList = _potentialTet->_judgeIsInPntList(_node);
		bool _inTetPntList = this->_judgeIsInPntList(_node, _potentialTet);

		if (_inTetPntList == false)
		{
			//if it is the real collided case
			_node->_collidedRealTetraList.push_back((*it));
			_node->_collidedRealTreeLeafNodeIndexList.push_back((*leafNodeIter));
		}
		else {
			//qDebug("Inside tet but is one of the tet node...\n");
		}


	}


	

	if (_node->_collidedRealTreeLeafNodeIndexList.empty() == false)
	{
		static int count = 0;
		count++;
		_node->isCollided = true;
		_node->isCollided_debug = true;		//only for debug
		double pos_debug[3];
		_node->GetCoord3D(pos_debug[0], pos_debug[1], pos_debug[2]);
		//qDebug("9th point coordinate: %lf %lf %lf", pos_debug[0], pos_debug[1], pos_debug[2]);
		
		/*for (auto it = _node->_collidedRealTreeLeafNodeIndexList.begin(); it != _node->_collidedRealTreeLeafNodeIndexList.end(); ++it)
		{
			_addSingleAABBboundingBox_debugQuery(_tree->nodes[(*it)].collisionBox, _debugPatch);
		}

		for (auto it = _node->_collidedRealTetraList.begin(); it != _node->_collidedRealTetraList.end(); ++it)
		{
			_addSingleTetrahedron_debugQuery(TetraPtrArray[(*it)], _debugPatch_tetDraw);
		}*/


		

		

		return true;
	}
	else
	{
		_node->isCollided = false;
		return false;

	}
		
	//------------------------------------------------------------------------------------------------

}

//Recursively query AABB tree: check which leaf node is potentially colliding with pos[3]

//para: @pos: point that will be checked
//		@_nodeIdx: parent node idx
//		@_flag:  1 -> left child; 2 -> right child (This child node is the left or right node of its parent)
//		@_recursiveLayer:	This node's depth
void AABBManager::_recursiveQuery(QMeshNode* _node, int _nodeIdx, int _flag, int _recursiveLayer)
{

	//------------------------------------------------------------------------------------------------
	//	Step 1: terminal condition: when it is leaf node
	AABBTREEArrayNode* thisNode;
	int thisNodeIdx = -1;
	if (_flag == 1)
		thisNodeIdx = _tree->nodes[_nodeIdx].child1;
	else
		thisNodeIdx = _tree->nodes[_nodeIdx].child2;

	thisNode = &(_tree->nodes[thisNodeIdx]);
	double pos[3];
	_node->GetCoord3D(pos[0], pos[1], pos[2]);

	if (thisNode->isLeaf == true)
	{
		
		if(thisNode->objectIndex !=-1)
		{
			//when this tree node contains one tetra
			
			//check whether this leaf node's collisionBox is contradicting with _node 
			bool _collided = _isCollided(pos, thisNode->collisionBox);
			//add tetra into the _node's tetralist
			if (_collided == true)
			{
				for (auto it = thisNode->tetraList.begin(); it != thisNode->tetraList.end(); ++it) {
					_node->_collidedTetraList.push_back((*it)->_idx);
					_node->_collidedTreeLeafNodeIndexList.push_back(thisNodeIdx);
				}

			}
			
		}
		
		return;
	}

	//------------------------------------------------------------------------------------------------

	//------------------------------------------------------------------------------------------------
	//	Step 2: check whether this level is intersecting with _node
	//			if not, directly return 
	//			if yes, go ahead to check whether its left and right child will collided
	bool _collided = _isCollided(pos, thisNode->collisionBox);
	if (_collided == false)
		return;
	//------------------------------------------------------------------------------------------------


	//------------------------------------------------------------------------------------------------
	//	Step 3: call recursive function 
	_recursiveQuery(_node, thisNodeIdx, 1,  _recursiveLayer + 1);
	_recursiveQuery(_node, thisNodeIdx, 2,  _recursiveLayer + 1);

	//------------------------------------------------------------------------------------------------

}


//QMeshPatch* AABBManager::SendDebugBoundingBoxPatch2MainWindow(void)
//{
//	return _debugPatch;
//}

//Checking whether a ray is intersecting with one triangle. If yes, return intersecting point
//para: @R -> representing ray
//		@A, B, C -> vertices of the triangle
//		@t -> a arbitary point on the ray can be written as P = O + t * D
//		@u, v -> coefficients to represent the barycentric coordinate of intersecting point if exist
//				 P = A + u * E1 + v * E2 = A + u * (B - A) + v * (C-A); u>=0, v>=0 and (u+v)<=1

bool AABBManager::_intersect_triangle(RAY R, Eigen::Vector3d A, Eigen::Vector3d B, Eigen::Vector3d C, double& t, double& u, double& v, Eigen::Vector3d& N)
{
	Eigen::Vector3d E1 = B - A; Eigen::Vector3d E2 = C - A;
	N = E1.cross(E2);
	float det = -R.Dir.dot(N);
	float invdet = 1.0 / det;
	Eigen::Vector3d AO = R.Origin - A;
	Eigen::Vector3d DAO = AO.cross(R.Dir);
	
	u = E2.dot(DAO) * invdet;
	v = -E1.dot(DAO) * invdet;
	t = AO.dot(N) * invdet;

	return (abs(det) >= 1e-6 && t >= 0.0 && u >= 0.0 && v >= 0.0 && (u + v) <= 1.0);
}

void AABBManager::SelfCollisionCorrespondenceCalculation(void)
{
	//------------------------------------------------------------------------------------------------
	//	Step 1: find the correspondence on the shell element
	//  Detail: for each collided point:
	//				find the intersection point along the inverse normal direction (of collided point) on the neighbour shell of collided tet element (to be exact, boundary triangle on the shell)
	//				also mark down the corresponding face pointer at each intersecting point
	//				mark down the barycentric coordinates of tracking point represented in its triangle
	//Functions we need: 
	//			1. check whether a ray intersecting with a triangle:
	//			   website: https://stackoverflow.com/questions/42740765/intersection-between-line-and-triangle-in-3d
	//			
	//node->isCollided

	/*for (GLKPOSITION Pos = _tetModelMesh->GetTetraList().GetHeadPosition(); Pos != NULL; ) {
		QMeshTetra* tet = (QMeshTetra*)(_tetModelMesh->GetTetraList().GetNext(Pos));
		tet->_boundaryFlag = false;
		tet->_boundaryFlag = false;
		for (int i = 0; i < 4; i++)
		{
			QMeshFace* _face = tet->GetFaceRecordPtr(i + 1);
			if (_face->inner == false)
			{
				tet->_boundaryFlag = true;
				break;
			}

		}
	}*/

	/*if (_tetModelMesh->collidedPntNum == 0)
	{
		return;
	}
*/


	int eleIndex = 0;
	Eigen::Vector3d _normal;
	for (GLKPOSITION Pos = _tetModelMesh->GetNodeList().GetHeadPosition(); Pos != NULL; ) {
		QMeshNode* node = (QMeshNode*)(_tetModelMesh->GetNodeList().GetNext(Pos));
		if (node->isBoundaryNode == true && node->isCollided == true)
		{
			node->CalNormal();

			//for each 
			if (node->_collidedRealTetraList.size() != 1)
			{
				qDebug("(%d) (from zero) collided tet number is bigger than 1, error...", eleIndex);
			}

			for (auto it = node->_collidedRealTetraList.begin(); it != node->_collidedRealTetraList.end(); ++it)
			{
				double pos[3], norm[3];
				node->GetCoord3D(pos[0], pos[1], pos[2]);
				node->GetNormal(norm[0], norm[1], norm[2]);
				bool _intersectedFace = false;
				Ray _normal2;
				int _ringNum = 0;
				_normal2.Origin = Eigen::Vector3d(pos[0], pos[1], pos[2]);

				//here normal direction is in-ward normal
				_normal2.Dir = Eigen::Vector3d(norm[0], norm[1], norm[2]);

				QMeshTetra* _collidedTet = TetraPtrArray[(*it)];

				//First, collect all two-ring boundary faces around the boundary face of _collidedTet. Then check the intersection
				//using std::set
				//To be more specific, face's edgelist -> facelist -> edgelist -> facelist
				std::vector<QMeshFace*> thisShouldVisitFaceVec;
				std::set<QMeshFace*> nextShouldVisitFaceSet;
				std::set<QMeshFace*> alreadyVisitedFaceSet;
				QMeshFace* _boundaryFace;
				bool _flag1 = false;

				//find the boundary tet first one-ring, then second-ring, then three-ring...until find the boundary tet
				auto _boundaryTetSet = _collidedTet->CalBoundaryTetList();
				//qDebug("%d boundary set num: %d", eleIndex, _boundaryTetSet.size());

				//acquire the original of flooding faceList
				for (auto itBoundaryTet = _boundaryTetSet.begin(); itBoundaryTet != _boundaryTetSet.end(); ++itBoundaryTet)
				{
					QMeshTetra* _bTet = (*itBoundaryTet);
					for (int i = 0; i < 4; i++)
					{
						QMeshFace* face = _bTet->GetFaceRecordPtr(i + 1);
						if (face->inner == false)
							thisShouldVisitFaceVec.push_back(face);
					}

				}

				//until the intersecting face is found, we then will jump out of loop
				while (1)
				{
					/*if(eleIndex==9)
						break;*/

					_ringNum++;
					//for each face in this loop should be visited
					for (int i = 0; i < thisShouldVisitFaceVec.size(); i++)
					{
						QMeshFace* _face = thisShouldVisitFaceVec[i];
						//if this face is intersecting with the Ray

						//boundary face.
						Eigen::Vector3d _A, _B, _C, _N;
						double u, v, t;
						_face->GetNodeRecordPtr(0)->GetCoord3D(_A[0], _A[1], _A[2]);
						_face->GetNodeRecordPtr(1)->GetCoord3D(_B[0], _B[1], _B[2]);
						_face->GetNodeRecordPtr(2)->GetCoord3D(_C[0], _C[1], _C[2]);

						bool _flag = false;
						if (node->GetIndexNo() == _face->GetNodeRecordPtr(0)->GetIndexNo() || node->GetIndexNo() == _face->GetNodeRecordPtr(1)->GetIndexNo() || node->GetIndexNo() == _face->GetNodeRecordPtr(2)->GetIndexNo())
						{
							//if the checking point is one of the checking triangle, then ignore this
						}
						else {
							//if not, directly check whether this is intersecting with triangle.
							_flag = this->_intersect_triangle(_normal2, _A, _B, _C, t, u, v, _N);
						}


						if (_flag == true)
						{
							_intersectedFace = true;
							qDebug("(%4d) node collided with face (%4d)", eleIndex, _face->GetIndexNo());
							node->_collidedFacePtr = _face;
							node->_barycentricCoord[0] = 1 - u - v;
							node->_barycentricCoord[1] = u;
							node->_barycentricCoord[2] = v;
							////debug usage: why intersecting face is so far
							//Eigen::Vector3d originPnt(_normal.Origin);
							//if ((originPnt - _A).norm() > 10)
							//{
							//	qDebug("pos: %lf %lf %lf", originPnt[0], originPnt[1], originPnt[2]);
							//	qDebug("(%4d) node collided too far", eleIndex);
							//	qDebug("%d boundary set num: %d", eleIndex, _boundaryTetSet.size());
							//}

							//node coorespondence finding
							node->_rayOrigin = _normal2.Origin;
							node->_rayDir = _normal2.Dir;
							node->_collidedT = t;


							//delete the wrong spring
							Eigen::Vector3d distanceVec = node->_rayDir * node->_collidedT;
							qDebug("distance norm is %lf", distanceVec.norm());
							if (distanceVec.norm() >= 5)
							{
								//_tetModelMesh->collidedPntNum--;
								node->isCollided = false;
							}


							//To delete
							/*QMeshNode* correNodeStart = new QMeshNode;
							QMeshNode* correNodeEnd = new QMeshNode;
							Eigen::Vector3d corre1, corre2;
							node->GetCoord3D(corre1[0], corre1[1], corre1[2]);
							correNodeStart->SetCoord3D(corre1[0], corre1[1], corre1[2]);
							_corresPatch->GetNodeList().AddTail(correNodeStart);

							corre2 = node->_rayOrigin + node->_rayDir * node->_collidedT;
							correNodeEnd->SetCoord3D(corre2[0], corre2[1], corre2[2]);
							_corresPatch->GetNodeList().AddTail(correNodeEnd);

							QMeshEdge* correEdge = new QMeshEdge;
							correEdge->SetStartPoint(correNodeStart);
							correEdge->SetEndPoint(correNodeEnd);
							_corresPatch->GetEdgeList().AddTail(correEdge);*/
							/*if(t>10)
								qDebug("(%4d) node collided too far", eleIndex);*/
								//auto _reference = node->_rayOrigin + node->_rayDir * (node->_collidedT);

								//node->SetCoord3D(_reference[0], _reference[1], _reference[2]);

							break;
						}
						else {
							//if (eleIndex == 9)
							//{

							//	/*qDebug("9th point coordinate: %lf %lf %lf", pos[0], pos[1], pos[2]);
							//	qDebug("t: %lf, u: %lf, v: %lf",t,u,v);*/
							//}
							//qDebug("(%4d) node not collided neighbours", eleIndex);
						}



						//if not, prepare the data for next ring searching
						auto _oneRingFaceSet = _face->GetNeighbourFacesOnSurface();
						for (auto it2 = _oneRingFaceSet.begin(); it2 != _oneRingFaceSet.end(); ++it2)
						{
							nextShouldVisitFaceSet.insert((*it2));
						}

						alreadyVisitedFaceSet.insert(_face);

					}

					if (_intersectedFace == true)
						break;
					else {
						/*if(_ringNum%100==0)
							qDebug("Index: %d", _ringNum);*/
							/*if (_ringNum >= 10)
								break;*/
					}

					//delete the extra elements in nextShouldVisitFaceSet comparing to alreadyVisitedFaceSet to form thisShouldVisitFaceVec
					thisShouldVisitFaceVec.clear();
					for (auto it3 = nextShouldVisitFaceSet.begin(); it3 != nextShouldVisitFaceSet.end(); ++it3)
					{
						if (alreadyVisitedFaceSet.count((*it3)) == 0)
						{
							thisShouldVisitFaceVec.push_back(*it3);
						}
					}
					nextShouldVisitFaceSet.clear();

				}



			}
			eleIndex++;
		}



	}

}

void AABBManager::GetVolumeObstacleMesh(QMeshPatch* _patch)
{
	_obstacleMesh = _patch;
}

bool AABBManager::_judgeInsideTet(double* pos, QMeshTetra* tetra)
{
		//index from 1
	// face: 123 other node: 4
	// face: 243 other node: 1
	// face: 341 other node: 2
	// face: 421 other node: 3

	//index from 0
	// face: 012 other node: 3
	// face: 132 other node: 0
	// face: 230 other node: 1
	// face: 310 other node: 2
	// double posMat[4][3];
	// 
	//old code
	double pos1[3], pos2[3], pos3[3], pos4[3];


	tetra->GetNodeRecordPtr(1)->GetCoord3D(pos1[0], pos1[1], pos1[2]);
	tetra->GetNodeRecordPtr(2)->GetCoord3D(pos2[0], pos2[1], pos2[2]);
	tetra->GetNodeRecordPtr(3)->GetCoord3D(pos3[0], pos3[1], pos3[2]);
	tetra->GetNodeRecordPtr(4)->GetCoord3D(pos4[0], pos4[1], pos4[2]);


	bool _isInside = _sameSide(pos1, pos2, pos3, pos4, pos) && _sameSide(pos2, pos4, pos3, pos1, pos) && _sameSide(pos3, pos4, pos1, pos2, pos) && _sameSide(pos4, pos2, pos1, pos3, pos);
	return _isInside;
}

//if this node is one of the nodes belonging to this tet, return true 
//Otherwise return false
bool AABBManager::_judgeIsInPntList(QMeshNode* _node, QMeshTetra* tetra)
{
	for (int i = 1; i <= 4; i++) {
		QMeshNode* _checkNode = tetra->GetNodeRecordPtr(i);
		if (_node == _checkNode) { return true; }
	}

	return false;
}

// pnt 1 -> pnt 2 -> pnt 3: anti-clockwise triangle, and the rest one is pnt4
//
bool AABBManager::_sameSide(double* pnt1, double* pnt2, double* pnt3, double* pnt4, double* queryPnt)
{
	// 2-1 3-2 cross product, normalize to be normal vector
	double vecA[3], vecB[3], vecC[3], vecQueryVec[3], sameSideVec[3];
	for (int i = 0; i < 3; i++)
	{
		vecA[i] = pnt2[i] - pnt1[i];
		vecB[i] = pnt3[i] - pnt2[i];
		vecQueryVec[i] = queryPnt[i] - pnt1[i];
		sameSideVec[i] = pnt4[i] - pnt1[i];
	}
	_crossProduct(vecA, vecB, vecC);
	double a = _dotProduct(vecC, vecQueryVec);
	double b = _dotProduct(vecC, sameSideVec);

	//a and b are the same sign, then they are on the same side
	if (a >= 0 && b >= 0)
		return true;
	else if (a <= 0 && b <= 0)
		return true;
	else
		return false;
}

double AABBManager::_dotProduct(double* vecA, double* vecB)
{
	double product = 0;
	// Loop for calculate dot product
	for (int i = 0; i < 3; i++)
		product = product + vecA[i] * vecB[i];
	return product;
}

//return normalized cross product result
void AABBManager::_crossProduct(double* vecA, double* vecB, double* vecC)
{
	/*vecC[0] = vecA[1] * vecB[2] - vecA[2] * vecC[1];
	vecC[1] = vecA[2] * vecB[0] - vecA[0] * vecC[2];
	vecC[2] = vecA[0] * vecB[1] - vecA[1] * vecC[0];*/

	vecC[0] = vecA[1] * vecB[2] - vecA[2] * vecB[1];
	vecC[1] = vecA[2] * vecB[0] - vecA[0] * vecB[2];
	vecC[2] = vecA[0] * vecB[1] - vecA[1] * vecB[0];

	double norm = vecC[0] * vecC[0] + vecC[1] * vecC[1] + vecC[2] * vecC[2];
	vecC[0] /= (norm);
	vecC[1] /= (norm);
	vecC[2] /= (norm);
}



void AABBManager::ObstacleTreeConstructionTop2Bot_rebuild(void)
{
	qDebug("Rebuild AABB Tree Representing Volume Obstacle...");
	//the variable when fast building the edge list should be reset
	startIndexfromPos_obstacle = NULL;
	startIndexfromPos_int_obstacle = 0;


	//------------------------------------------------------------------------------------------------
	//	Step 1: memory allocation of tree; grouping all tetra element into one array
	//			calculate the center of tetrahedron and bounding box.
	//clear old _tree 
	//we should not delete tree nodearray, directly define the nodeCount and rootIndex should be fine

	if (!_isConstructed)
	{
		//
		//------------------------------------------------------------------------------------------------
		//	Step 1: memory allocation of tree; grouping all tetra element into one array
		//			calculate the center of tetrahedron and bounding box.
		_obstacleTree = new aabbTree;
		memoryAllocated__obstacleTree = true;
		_obstacleTree->nodes = new AABBTREEArrayNode[MAXNUM_OF_AABBNODE_ONTREE];
		_obstacleTree->nodeCount = -1;
		_obstacleTree->rootIndex = -1;

		TetraPtrArrayLen_obstacle = _obstacleMesh->GetTetraList().GetCount();
		TetraPtrArray_obstacle = new QMeshTetra * [TetraPtrArrayLen_obstacle];
		memoryAllocated__TetraPtrArray_obstacle = true;							//used to free memory
		
		int eleIndex = 0;

		//calculate the bounding box and its center
		//mark the index of each tetrahedron
		for (GLKPOSITION Pos = _obstacleMesh->GetTetraList().GetHeadPosition(); Pos != NULL; eleIndex++) {
			QMeshTetra* Tet = (QMeshTetra*)(_obstacleMesh->GetTetraList().GetNext(Pos));


			Tet->GetNodeRecordPtr(1)->GetCoord3D(Tet->pos1[0], Tet->pos1[1], Tet->pos1[2]);
			Tet->GetNodeRecordPtr(2)->GetCoord3D(Tet->pos2[0], Tet->pos2[1], Tet->pos2[2]);
			Tet->GetNodeRecordPtr(3)->GetCoord3D(Tet->pos3[0], Tet->pos3[1], Tet->pos3[2]);
			Tet->GetNodeRecordPtr(4)->GetCoord3D(Tet->pos4[0], Tet->pos4[1], Tet->pos4[2]);

			TetraPtrArray_obstacle[eleIndex] = Tet;		//assign it to be inside the tetra array.
			Tet->_idx = eleIndex;
			Tet->CalCenCalBoundingBox();		//calculate tetrahedron center and its bounding box.

			/*if (eleIndex < 5)
			{
				printf("Vertex pos: %lf %lf %lf\n", Tet->_center[0], Tet->_center[1], Tet->_center[2]);
			}*/
		}

		for (GLKPOSITION Pos = _obstacleMesh->GetFaceList().GetHeadPosition(); Pos != NULL; eleIndex++) {
			QMeshFace* face = (QMeshFace*)(_obstacleMesh->GetFaceList().GetNext(Pos));

			if (face->GetLeftTetra() == NULL || face->GetRightTetra() == NULL)
			{
				face->inner = false;
			}
			else {
				face->inner = true;
			}


		}



		_isConstructed = true;
	}
	else
	{
		for (int i = 0; i < _obstacleTree->nodeCount; i++)
		{
			_obstacleTree->nodes[i].tetraList.clear();
			_obstacleTree->nodes[i].isLeaf = false;
			_obstacleTree->nodes[i].objectIndex = -2;
			_obstacleTree->nodes[i].depth = -1;
		}
	}


	_obstacleTree->rootIndex = 0;
	_obstacleTree->nodeCount = 0;
	_obstacleTree->depth = 0;
	/*aabbTree* _obstacleTree;
	QMeshPatch* _obstacleMesh;

	QMeshTetra** TetraPtrArray_obstacle;
	int TetraPtrArrayLen_obstacle;
	AABB _init_obstacle;*/


	//clear old TetraPtrArray_obstacle: dont need to do here
	//TetraPtrArrayLen_obstacle = _obstacleMesh->GetTetraList().GetCount();
	//delete[] TetraPtrArray_obstacle;

	//TetraPtrArray_obstacle = new QMeshTetra * [TetraPtrArrayLen_obstacle];
	int eleIndex = 0;

	//calculate the bounding box and its center
	//mark the index of each tetrahedron
	for (GLKPOSITION Pos = _obstacleMesh->GetTetraList().GetHeadPosition(); Pos != NULL; eleIndex++) {
		QMeshTetra* Tet = (QMeshTetra*)(_obstacleMesh->GetTetraList().GetNext(Pos));
		Tet->CalCenCalBoundingBox();		//calculate tetrahedron center and its bounding box.
		//Tet->_idx = eleIndex;

		/*if (eleIndex < 5)
		{
			printf("Vertex pos: %lf %lf %lf\n", Tet->_center[0], Tet->_center[1], Tet->_center[2]);
		}*/

	}

	printf("\nStep 1 finished...\n");
	//------------------------------------------------------------------------------------------------


	//------------------------------------------------------------------------------------------------
	//	Step 2: creating the the root node and insert it into the tree
	//  first, calculate the bounding box of all elements

	//Yingjun: check here.
	for (int i = 0; i < 3; i++) {
		_init_obstacle.lowerBound[i] = INF; _init_obstacle.upperBound[i] = NINF;
	}

	double pos[3];
	for (int i = 0; i < TetraPtrArrayLen_obstacle; i++)
	{
		//travel all points to update the minimal x, y and z
		//							  the maximum x, y and z
		QMeshTetra* tet = TetraPtrArray_obstacle[i];
		for (int j = 0; j < 4; j++)
		{
			(tet->GetNodeRecordPtr(j + 1))->GetCoord3D(pos[0], pos[1], pos[2]);
			_init_obstacle.lowerBound[0] = MIN(pos[0], _init_obstacle.lowerBound[0]);
			_init_obstacle.lowerBound[1] = MIN(pos[1], _init_obstacle.lowerBound[1]);
			_init_obstacle.lowerBound[2] = MIN(pos[2], _init_obstacle.lowerBound[2]);

			_init_obstacle.upperBound[0] = MAX(pos[0], _init_obstacle.upperBound[0]);
			_init_obstacle.upperBound[1] = MAX(pos[1], _init_obstacle.upperBound[1]);
			_init_obstacle.upperBound[2] = MAX(pos[2], _init_obstacle.upperBound[2]);
		}
	}


	// second, insert the root node
	// _printBoundingBoxInfo("all node", _init);
	_insert2Tree_obstacle(0, -1, _init_obstacle);
	printf("\nStep 2 finished...\n");
	//------------------------------------------------------------------------------------------------


	//------------------------------------------------------------------------------------------------
	//	Step 3: gradually split the bounding box and add them to the box
	//first, split the bounding box.
	AABB left, right;
	splitBoundingBox(_init_obstacle, &left, &right);

	printf("Start to call recursive function...\n");

	_recursiveConstruction_rebuild_obstacle(0, 1, left, 1);
	_recursiveConstruction_rebuild_obstacle(0, 2, right, 1);
	printf("Stop calling  recursive function...\n");
	printf("\nStep 3 finished...\n");

	//------------------------------------------------------------------------------------------------


	qDebug("Rebuilding of AABB Tree of Obstacle ends, the depth of obstacle tree is %d...", _obstacleTree->depth);
}


//Insert node containing _ele to the _nodeIdx's left 
//or right child. Specially, when the _flag = 0, then 
//we insert it into the root node

//There should be one var describing whether it contains only one tetrahedron
//Also the merge of AABB should be included
//para: @_nodeIdx: parent node idx. 
//		@_flag:  0 -> root node; 1 -> left child; 2 -> right child
//		@_ele:	 containment in child node 
void AABBManager::_insert2Tree_obstacle(int _flag, int _nodeIdx, AABB _ele)
{
	if (_flag == 0)
	{
		_obstacleTree->rootIndex = 0;
		_obstacleTree->nodes[0].box = _ele;
		_obstacleTree->nodes[0].isLeaf = false;
		_obstacleTree->nodeCount = 1;
		_obstacleTree->nodes[0].collisionBox = _ele;
		_obstacleTree->nodes[0].depth = 0;

		//_tree->nodes[0].tetraIndexList


		for (int i = 0; i < TetraPtrArrayLen_obstacle; i++)
		{
			//travel all points to update the minimal x, y and z
			//							  the maximum x, y and z
			_obstacleTree->nodes[0].tetraList.push_back(TetraPtrArray_obstacle[i]);
		}

	}

}



//Recursively construct AABB tree in rebuilding process
//given the known bounding box, it will split the largest dimension

//para: @_nodeIdx: parent node idx. 
//		@_flag:  1 -> left child; 2 -> right child
//		@_oldBoundingBox:	newly split bounding box, we need to update this bounding box according to all tetra whose center is within _oldBoundingBox
void AABBManager::_recursiveConstruction_rebuild_obstacle(int _nodeIdx, int _flag, AABB _old, int _recursiveLayer)
{

	//update the bounding box of _old
	//long startTime = clock();
	int _insertIdx = _obstacleTree->nodeCount;
	AABB _oldCollisionBox;
	for (int i = 0; i < 3; i++)
	{
		_oldCollisionBox.lowerBound[i] = INF;
		_oldCollisionBox.upperBound[i] = NINF;
	}

	double pp[3];
	int _inOldBoxNum = 0;
	int _objectIdx = -1;


	//version 2: search from all tetrahedron from parent (i.e., travel  _tree->nodes[_nodeIdx].tetraList)

	for (TetIter tet : (_obstacleTree->nodes[_nodeIdx].tetraList))
	{
		pp[0] = tet->_center[0];
		pp[1] = tet->_center[1];
		pp[2] = tet->_center[2];

		if ((pp[0] > _old.lowerBound[0] && pp[0] < _old.upperBound[0]) && (pp[1] > _old.lowerBound[1] && pp[1] < _old.upperBound[1]) && (pp[2] > _old.lowerBound[2] && pp[2] < _old.upperBound[2]))
		{
			//within _old bounding box
			_obstacleTree->nodes[_insertIdx].tetraList.push_back(tet);					//update this AABB Tree node's containment list
			_inOldBoxNum++;
			_objectIdx = tet->_idx;
			//update bounding box
			for (int j = 0; j < 3; j++)
			{
				if (tet->_lowerBounding[j] < _oldCollisionBox.lowerBound[j])
				{
					_oldCollisionBox.lowerBound[j] = tet->_lowerBounding[j];
				}

				if (tet->_upperBounding[j] > _oldCollisionBox.upperBound[j])
				{
					_oldCollisionBox.upperBound[j] = tet->_upperBounding[j];
				}
			}
		}

	}
	//for (int i = 0; i < TetraPtrArrayLen; i++)
	//{
	//	//travel all points to update the minimal x, y and z
	//	//							  the maximum x, y and z
	//	_tree->nodes[0].tetraList.push_back(TetraPtrArray[i]);
	//}



	//long endTime = clock();
	//long Time1 = endTime - startTime;
	//ending criterion: check whether this node is the leef node and insert it into AABB Tree.
	//update the insert index of current node

	if (_inOldBoxNum == 1)
	{
		//leaf node that contains only one tetrahedron whose idx is objectIndex.
		//here we end the recursive function
		_obstacleTree->nodes[_insertIdx].isLeaf = true;
		_obstacleTree->nodes[_insertIdx].parentIndex = _nodeIdx;
		_obstacleTree->nodes[_insertIdx].box = _old;
		_obstacleTree->nodes[_insertIdx].collisionBox = _oldCollisionBox;

		_obstacleTree->nodes[_insertIdx].objectIndex = _objectIdx;
		_obstacleTree->nodes[_insertIdx].depth = _recursiveLayer;
		_obstacleTree->nodeCount++;

		if (_flag == 1)
			_obstacleTree->nodes[_nodeIdx].child1 = _insertIdx;		//this newly inserted node is left child
		else if (_flag == 2)
			_obstacleTree->nodes[_nodeIdx].child2 = _insertIdx;		//this newly inserted node is right child

		if (_recursiveLayer > _obstacleTree->depth)
			_obstacleTree->depth = _recursiveLayer;
		//printf("Reach leaf node: in old box num is %d\n",1);
		return;

	}
	else if (_inOldBoxNum == 0)
	{
		//leaf that dosen't contain any tetrahedron because the highly distorted tetrahedron this time
		//here we end the recursive function
		//use _old to be This AABB 
		_obstacleTree->nodes[_insertIdx].isLeaf = true;
		_obstacleTree->nodes[_insertIdx].parentIndex = _nodeIdx;
		_obstacleTree->nodes[_insertIdx].box = _old;
		_obstacleTree->nodes[_insertIdx].collisionBox = _old;
		_obstacleTree->nodes[_insertIdx].objectIndex = -1;
		_obstacleTree->nodes[_insertIdx].depth = _recursiveLayer;
		_obstacleTree->nodeCount++;

		if (_flag == 1)
			_obstacleTree->nodes[_nodeIdx].child1 = _insertIdx;		//this newly inserted node is left child
		else if (_flag == 2)
			_obstacleTree->nodes[_nodeIdx].child2 = _insertIdx;		//this newly inserted node is right child

		if (_recursiveLayer > _obstacleTree->depth)
			_obstacleTree->depth = _recursiveLayer;
		//printf("Reach leaf node: in old box num is %d\n", 0);
		return;
	}
	else {
		//leaf node that contain at leat two tets, we continue the recursive function
		_obstacleTree->nodes[_insertIdx].isLeaf = false;
		_obstacleTree->nodes[_insertIdx].parentIndex = _nodeIdx;
		_obstacleTree->nodes[_insertIdx].box = _old;
		_obstacleTree->nodes[_insertIdx].collisionBox = _oldCollisionBox;
		_obstacleTree->nodes[_insertIdx].objectIndex = -2;			//this var dosent matter here
		_obstacleTree->nodes[_insertIdx].depth = _recursiveLayer;
		_obstacleTree->nodeCount++;

		if (_flag == 1)
			_obstacleTree->nodes[_nodeIdx].child1 = _insertIdx;		//this newly inserted node is left child
		else if (_flag == 2)
			_obstacleTree->nodes[_nodeIdx].child2 = _insertIdx;		//this newly inserted node is right child

	}
	/*if (_recursiveLayer == 10)
		return;*/
		//normal execution
	AABB left, right;
	splitBoundingBox(_old, &left, &right);
	/*if (_flag == 1)
	{
		counter[_recursiveLayer]++;
		if (counter[_recursiveLayer] == 1)
		{

			printf("\nLeft Layer: %d\n", _recursiveLayer);
			_printBoundingBoxInfo(to_string(_recursiveLayer), _old);
			_printBoundingBoxInfo("Next would be:", left);

		}

	}*/
	//long Time2 = (long)clock() - startTime;
	//printf("part is taking %3.3lf percent\n",(double)Time1/ (double)Time2 *100.0);

	_recursiveConstruction_rebuild_obstacle(_insertIdx, 1, left, _recursiveLayer + 1);
	_recursiveConstruction_rebuild_obstacle(_insertIdx, 2, right, _recursiveLayer + 1);

	//debug output
	/*if (_flag == 1)
		printf("left: [%lf, %lf, %lf] -> [%lf, %lf, %lf]\n", _old.lowerBound[0], _old.lowerBound[1], _old.lowerBound[2], _old.upperBound[0], _old.upperBound[1], _old.upperBound[2]);
	if (_flag == 2)
		printf("right: [%lf, %lf, %lf] -> [%lf, %lf, %lf]\n", _old.lowerBound[0], _old.lowerBound[1], _old.lowerBound[2], _old.upperBound[0], _old.upperBound[1], _old.upperBound[2]);*/

}


//////////////////////////////////////////////////////////
///////Refit AABB Tree from New Tetrahedron Mesh//////////
//////////////////////////////////////////////////////////
//Envionment Obstacle
void AABBManager::TreeConstructionTop2Bot_refit_obstacle(void)
{
	////the variable when fast building the edge list should be reset
	//startIndexfromPos = NULL;
	//startIndexfromPos_int = 0;

	//------------------------------------------------------------------------------------------------
	//	Step 1: do the postorder traversal around the tree
	//			calculate the center of tetrahedron and bounding box.

	//calculate the bounding box and its center
	//mark the index of each tetrahedron
	int eleIndex = 0;
	for (GLKPOSITION Pos = _obstacleMesh->GetTetraList().GetHeadPosition(); Pos != NULL; eleIndex++) {
		QMeshTetra* Tet = (QMeshTetra*)(_obstacleMesh->GetTetraList().GetNext(Pos));
		//TetraPtrArray[eleIndex] = Tet;		//assign it to be inside the tetra array.
		Tet->_idx = eleIndex;
		Tet->CalCenCalBoundingBox();		//calculate tetrahedron center and its bounding box.

	}
	_recursivePostOrderTraversal_refit_obstacle(0, 1, 1);
	_recursivePostOrderTraversal_refit_obstacle(0, 2, 1);


	int leftChildIdx = _obstacleTree->nodes[0].child1;
	int rightChildIdx = _obstacleTree->nodes[0].child2;
	_obstacleTree->nodes[0].collisionBox = Union(_obstacleTree->nodes[leftChildIdx].collisionBox, _obstacleTree->nodes[rightChildIdx].collisionBox);

	printf("\nRefit Finished...\n");
	//------------------------------------------------------------------------------------------------
}


//Recursively update AABB tree: update collision box of each node including internal nodes and leaf nodes
//(collision tetraList of each tree node would not be updated)
//given the known node no matter which depth this node is, it will update the collision bounding box according to its children

//para: @_nodeIdx: parent node idx. 
//		@_flag:  1 -> left child; 2 -> right child (This child node is the left or right node of its parent)
//		@_oldBoundingBox:	newly split bounding box, we need to update this bounding box according to all tetra whose center is within _oldBoundingBox
void AABBManager::_recursivePostOrderTraversal_refit_obstacle(int _nodeIdx, int _flag, int _recursiveLayer)
{
	//------------------------------------------------------------------------------------------------
	//	Step 1: check the terminal condition is reached and update the collision box
	//------------------------------------------------------------------------------------------------
	AABBTREEArrayNode* thisNode;
	int thisNodeIdx = -1;
	if (_flag == 1)
		thisNodeIdx = _obstacleTree->nodes[_nodeIdx].child1;
	else
		thisNodeIdx = _obstacleTree->nodes[_nodeIdx].child2;

	thisNode = &(_obstacleTree->nodes[thisNodeIdx]);

	if (thisNode->isLeaf == true)
	{
		int tetIdx = thisNode->objectIndex;
		if (tetIdx == -1)
		{
			//dont do anything
			return;
		}
		else
		{
			//update the collision box
			QMeshTetra* tetraPtr = TetraPtrArray_obstacle[tetIdx];
			//tetraPtr->CalCenCalBoundingBox();
			for (int i = 0; i < 3; i++)
			{
				thisNode->collisionBox.lowerBound[i] = tetraPtr->_lowerBounding[i];
				thisNode->collisionBox.upperBound[i] = tetraPtr->_upperBounding[i];
			}
			return;
		}

	}


	//------------------------------------------------------------------------------------------------
	//	Step 2: traversal the left children
	//------------------------------------------------------------------------------------------------
	_recursivePostOrderTraversal_refit_obstacle(thisNodeIdx, 1, _recursiveLayer + 1);

	//------------------------------------------------------------------------------------------------
	//	Step 3: traversal the right children
	//------------------------------------------------------------------------------------------------
	_recursivePostOrderTraversal_refit_obstacle(thisNodeIdx, 2, _recursiveLayer + 1);

	//------------------------------------------------------------------------------------------------
	//	Step 4: merge children's collision bounding box into this node's collision bounding box 
	//			and then return
	//  first get left child's bounding box info

	int leftChildIndex = thisNode->child1;
	int rightChildIndex = thisNode->child2;

	if ((_obstacleTree->nodes[leftChildIndex].objectIndex != -1) && (_obstacleTree->nodes[rightChildIndex].objectIndex != -1))
	{
		//if the left child is not the empty left node and right child is also not.
		//merge two collision box together and assign it to be this node's new collision box
		thisNode->collisionBox = Union(_obstacleTree->nodes[leftChildIndex].collisionBox, _obstacleTree->nodes[rightChildIndex].collisionBox);
	}
	else if ((_obstacleTree->nodes[leftChildIndex].objectIndex == -1) && (_obstacleTree->nodes[rightChildIndex].objectIndex != -1))
	{
		//if the left child is empty leaf node and right child is not.
		thisNode->collisionBox = _obstacleTree->nodes[rightChildIndex].collisionBox;
	}
	else if ((_obstacleTree->nodes[leftChildIndex].objectIndex != -1) && (_obstacleTree->nodes[rightChildIndex].objectIndex == -1))
	{
		//if the left child is not empty leaf node and right child is.
		thisNode->collisionBox = _obstacleTree->nodes[leftChildIndex].collisionBox;
	}
	else {
		//if all are empty
		printf("Please check your code: this situation can not be happened cuz there are no two empty leaf nodes attached to the parent node...");

	}
	//------------------------------------------------------------------------------------------------
}

/*Check collision of _tetModelMesh with _obstacleMesh, return result contained in collidedResult:
	Chamber Idx is from 1 to 4.*/
Eigen::VectorXd AABBManager::CollisionWithEnvQueryCheckingReturnResult(int chamberIdx)
{
	Eigen::VectorXd collidedResult = Eigen::VectorXd::Zero(_tetModelMesh->GetNodeNumber());	//_tetModelMesh->GetNodeNumber()

	counter_init_env++;
	Eigen::MatrixXd POS(3, 1);
	bool flag = false;
	Eigen::Vector3d rayOri, rayDir, faceVer[3];
	double u, v, t;
	int counterSpring_env = 0;

	//isCollided_env_last
	if (counter_init_env == 1)
	{
		_corresPatch = new QMeshPatch;

	
		//update PQP Model for the closest point query
		int counter_faceNum = 0;
		_obstaclePQPModel_volumeSurface = new PQP_Model();
		memoryAllocated_obstaclePQPModel_volumeSurface = true;		//memory free flag

		for (GLKPOSITION Pos = _obstacleMesh->GetFaceList().GetHeadPosition(); Pos; )
		{
			QMeshFace* face = (QMeshFace*)(_obstacleMesh->GetFaceList().GetNext(Pos));
			if (face->inner == false)
			{
				counter_faceNum++;
			}
		}

		_len_faceArray_volumeSurface_obstacle = counter_faceNum;
		_faceArray_volumeSurface_obstacle = new QMeshFace * [_len_faceArray_volumeSurface_obstacle];
		memoryAllocated_faceArray_volumeSurface_obstacle = true;	//memory free flag

		_obstaclePQPModel_volumeSurface->BeginModel();
		int idx = 0;
		double posPNTArray[3][3];
		for (GLKPOSITION Pos = _obstacleMesh->GetFaceList().GetHeadPosition(); Pos; )
		{
			QMeshFace* face = (QMeshFace*)(_obstacleMesh->GetFaceList().GetNext(Pos));
			if (face->inner == false)
			{
				_faceArray_volumeSurface_obstacle[idx] = face;

				//get vertices information
				for (int i = 0; i < 3; i++)
				{
					face->GetNodeRecordPtr(i)->GetCoord3D(posPNTArray[i][0], posPNTArray[i][1], posPNTArray[i][2]);
				}
				_obstaclePQPModel_volumeSurface->AddTri(posPNTArray[0], posPNTArray[1], posPNTArray[2], idx);
				idx++;
			}
		}
		_obstaclePQPModel_volumeSurface->EndModel();


	}
	else
	{
		_corresPatch->ClearAll();

		delete _obstaclePQPModel_volumeSurface;
		_obstaclePQPModel_volumeSurface = new PQP_Model();
		_obstaclePQPModel_volumeSurface->BeginModel();
		int idx = 0;
		double posPNTArray[3][3];
		for (GLKPOSITION Pos = _obstacleMesh->GetFaceList().GetHeadPosition(); Pos; )
		{
			QMeshFace* face = (QMeshFace*)(_obstacleMesh->GetFaceList().GetNext(Pos));
			if (face->inner == false)
			{
				//get vertices information
				for (int i = 0; i < 3; i++)
				{
					face->GetNodeRecordPtr(i)->GetCoord3D(posPNTArray[i][0], posPNTArray[i][1], posPNTArray[i][2]);
				}
				_obstaclePQPModel_volumeSurface->AddTri(posPNTArray[0], posPNTArray[1], posPNTArray[2], idx);
				idx++;
			}
		}
		_obstaclePQPModel_volumeSurface->EndModel();



	}

	//------------------------------------------------------------------------------------------------
	//	Step 1:
	// first check whether it is in collision

	for (GLKPOSITION Pos = _obstacleMesh->GetFaceList().GetHeadPosition(); Pos != NULL;) {
		QMeshFace* face = (QMeshFace*)(_obstacleMesh->GetFaceList().GetNext(Pos));
		face->CalPlaneEquation();
	}


	int eleIndex = 0;

	//useful
	//for (GLKPOSITION Pos = _obstacleMesh->GetNodeList().GetHeadPosition(); Pos != NULL;)
	//{
	//	QMeshNode* node = (QMeshNode*)(_obstacleMesh->GetNodeList().GetNext(Pos));
	//	node->isCollided_env_last = node->isCollided_env;	//last iteration
	//	//node->keepOldSpring = false;				//default: dont keep any spring attached on this point and its correspondence
	//	node->isCollided_env = false;
	//	node->CalNormal();
	//}



	//------------------------------------------------------------------------------------------------
	//	Step 2: collision checking and correspondence updating
	//  now, only check collision of deformable object with obstacle

	qDebug("Checking Collision with Env...");
	int tetNodeIdx = 0;
	for (GLKPOSITION Pos = _tetModelMesh->GetNodeList().GetHeadPosition(); Pos; tetNodeIdx++)
	{
		QMeshNode* node = (QMeshNode*)_tetModelMesh->GetNodeList().GetNext(Pos);
		Eigen::Vector3d coord;
		node->GetCoord3D(coord[0], coord[1], coord[2]);

		//node->isCollided_env_last = node->isCollided_env;					//status update has been done already

		//when it is in RoI
		if (node->MannequinEnvCollisionRoI == true)
		{
			//check whether boundary node is inside the obstacle
			//if yes, then establish the correspondence
			//new code

			//check whether the point is inside env tetrahedron
			bool _checkResult = _singlePntQueryWithTree(node, _obstacleTree, TetraPtrArray_obstacle);


			if (_checkResult)
			{
				node->collidedChamberIdx = chamberIdx;
				counterSpring_env++;
				collidedResult[tetNodeIdx] = 1;
				//new: find correspondence with closest point query
				//find the closest point  and the its normal
				PQP_REAL p[3]; node->GetCoord3D(p[0], p[1], p[2]);
				PQP_DistanceResult dres;	dres.last_tri = _obstaclePQPModel_volumeSurface->last_tri;
				PQP_Distance(&dres, _obstaclePQPModel_volumeSurface, p, 0, 0.0);
				//dres.p1[0], dres.p1[1], dres.p1[2]
				Eigen::Vector3d normalClosestPnt;
				Eigen::Vector3d posClosestPnt;
				Eigen::Vector3d closestPnt_queryPntVec;
				double dd;
				posClosestPnt[0] = dres.p1[0];
				posClosestPnt[1] = dres.p1[1];
				posClosestPnt[2] = dres.p1[2];
				int minTriIndex = dres.last_tri->id;

				QMeshFace* _collidedFace = _faceArray_volumeSurface_obstacle[minTriIndex];
				Eigen::Vector3d normalCollidedFace;
				_collidedFace->GetNormal(normalCollidedFace[0], normalCollidedFace[1], normalCollidedFace[2]);
				normalCollidedFace = -normalCollidedFace;

				Eigen::Vector3d springAttach = posClosestPnt + normalCollidedFace * coeff_extension_spring_env;
				node->closestPntOnObstacle[0] = springAttach[0];
				node->closestPntOnObstacle[1] = springAttach[1];
				node->closestPntOnObstacle[2] = springAttach[2];

				////To delete
				QMeshNode* correNodeStart = new QMeshNode;
				QMeshNode* correNodeEnd = new QMeshNode;
				Eigen::Vector3d corre1, corre2;
				node->GetCoord3D(corre1[0], corre1[1], corre1[2]);
				correNodeStart->SetCoord3D(corre1[0], corre1[1], corre1[2]);
				_corresPatch->GetNodeList().AddTail(correNodeStart);
				corre2 = posClosestPnt + normalCollidedFace * coeff_extension_spring_env;
				correNodeEnd->SetCoord3D(corre2[0], corre2[1], corre2[2]);
				_corresPatch->GetNodeList().AddTail(correNodeEnd);
				QMeshEdge* correEdge = new QMeshEdge;
				correEdge->_selfCollisionType = 1;
				correEdge->SetStartPoint(correNodeStart);
				correEdge->SetEndPoint(correNodeEnd);
				_corresPatch->GetEdgeList().AddTail(correEdge);


				//old code below
				//old: find corrspondence with normal direction
				//correspondence calculation
				//for each collided point
				//if (node->_collidedRealTetraList.size() != 1)
				//{
				//	qDebug("(%d) (from zero) collided tet number is bigger than 1, error...", eleIndex);
				//}
				////calculate correspondence
				//for (auto it = node->_collidedRealTetraList.begin(); it != node->_collidedRealTetraList.end(); ++it)
				//{
				//	double pos[3], norm[3];
				//	node->GetCoord3D(pos[0], pos[1], pos[2]);
				//	node->GetNormal(norm[0], norm[1], norm[2]);
				//	bool _intersectedFace = false;
				//	Ray _normal2;
				//	int _ringNum = 0;
				//	_normal2.Origin = Eigen::Vector3d(pos[0], pos[1], pos[2]);
				//	//here normal direction is in-ward normal
				//	_normal2.Dir = Eigen::Vector3d(norm[0], norm[1], norm[2]);
				//	QMeshTetra* _collidedTet = TetraPtrArray_obstacle[(*it)];
				//	//First, collect all two-ring boundary faces around the boundary face of _collidedTet. Then check the intersection
				//	//using std::set
				//	//To be more specific, face's edgelist -> facelist -> edgelist -> facelist
				//	std::vector<QMeshFace*> thisShouldVisitFaceVec;
				//	std::set<QMeshFace*> nextShouldVisitFaceSet;
				//	std::set<QMeshFace*> alreadyVisitedFaceSet;
				//	QMeshFace* _boundaryFace;
				//	bool _flag1 = false;
				//	//find the boundary tet first one-ring, then second-ring, then three-ring...until find the boundary tet
				//	auto _boundaryTetSet = _collidedTet->CalBoundaryTetList();
				//	//qDebug("%d boundary set num: %d", eleIndex, _boundaryTetSet.size());
				//	//acquire the original of flooding faceList
				//	for (auto itBoundaryTet = _boundaryTetSet.begin(); itBoundaryTet != _boundaryTetSet.end(); ++itBoundaryTet)
				//	{
				//		QMeshTetra* _bTet = (*itBoundaryTet);
				//		for (int i = 0; i < 4; i++)
				//		{
				//			QMeshFace* face = _bTet->GetFaceRecordPtr(i + 1);
				//			if (face->inner == false)
				//				thisShouldVisitFaceVec.push_back(face);
				//		}
				//	}
				//	//until the intersecting face is found, we then will jump out of loop
				//	while (1)
				//	{
				//		/*if(eleIndex==9)
				//			break;*/
				//		_ringNum++;
				//		//for each face in this loop should be visited
				//		for (int i = 0; i < thisShouldVisitFaceVec.size(); i++)
				//		{
				//			QMeshFace* _face = thisShouldVisitFaceVec[i];
				//			//if this face is intersecting with the Ray
				//			//boundary face.
				//			Eigen::Vector3d _A, _B, _C, _N;
				//			double u, v, t;
				//			_face->GetNodeRecordPtr(0)->GetCoord3D(_A[0], _A[1], _A[2]);
				//			_face->GetNodeRecordPtr(1)->GetCoord3D(_B[0], _B[1], _B[2]);
				//			_face->GetNodeRecordPtr(2)->GetCoord3D(_C[0], _C[1], _C[2]);
				//			bool _flag = false;
				//			if (node == _face->GetNodeRecordPtr(0) || node==_face->GetNodeRecordPtr(1)|| node == _face->GetNodeRecordPtr(2))
				//			{
				//				//if the checking point is one of the checking triangle, then ignore this
				//			}
				//			else {
				//				//if not, directly check whether this is intersecting with triangle.
				//				_flag = _tetModelMesh->_intersect_triangle(_normal2, _A, _B, _C, t, u, v, _N);
				//			}
				//			if (_flag == true)
				//			{
				//				_intersectedFace = true;
				//				//qDebug("(%4d) node collided with face (%4d)", eleIndex, _face->GetIndexNo());
				//				node->_collidedFacePtr_obstacle = _face;
				//				node->_barycentricCoord_obstacle[0] = 1 - u - v;
				//				node->_barycentricCoord_obstacle[1] = u;
				//				node->_barycentricCoord_obstacle[2] = v;
				//				////debug usage: why intersecting face is so far
				//				//Eigen::Vector3d originPnt(_normal.Origin);
				//				//if ((originPnt - _A).norm() > 10)
				//				//{
				//				//	qDebug("pos: %lf %lf %lf", originPnt[0], originPnt[1], originPnt[2]);
				//				//	qDebug("(%4d) node collided too far", eleIndex);
				//				//	qDebug("%d boundary set num: %d", eleIndex, _boundaryTetSet.size());
				//				//}
				//				//node coorespondence finding
				//				node->_rayOrigin_obstacle = _normal2.Origin;
				//				node->_rayDir_obstacle = _normal2.Dir;
				//				node->_collidedT_obstacle = t;
				//				//delete the wrong spring
				//				Eigen::Vector3d distanceVec = node->_rayDir_obstacle * node->_collidedT_obstacle;
				//				//qDebug("distance norm is %lf", distanceVec.norm());
				//				if (distanceVec.norm() >= threshold_envCollisionSpring_maximum)
				//				{
				//					counterSpring_env--;
				//					node->isCollided_env = false;
				//				}
				//				//To delete
				//				QMeshNode* correNodeStart = new QMeshNode;
				//				QMeshNode* correNodeEnd = new QMeshNode;
				//				Eigen::Vector3d corre1, corre2;
				//				node->GetCoord3D(corre1[0], corre1[1], corre1[2]);
				//				correNodeStart->SetCoord3D(corre1[0], corre1[1], corre1[2]);
				//				_corresPatch->GetNodeList().AddTail(correNodeStart);
				//				corre2 = node->_rayOrigin_obstacle + node->_rayDir_obstacle * (node->_collidedT_obstacle + ShapeUpOperator->coeff_extension_spring_env);
				//				correNodeEnd->SetCoord3D(corre2[0], corre2[1], corre2[2]);
				//				_corresPatch->GetNodeList().AddTail(correNodeEnd);
				//				QMeshEdge* correEdge = new QMeshEdge;
				//				correEdge->_selfCollisionType = 0;
				//				correEdge->SetStartPoint(correNodeStart);
				//				correEdge->SetEndPoint(correNodeEnd);
				//				_corresPatch->GetEdgeList().AddTail(correEdge);
				//				/*if(t>10)
				//					qDebug("(%4d) node collided too far", eleIndex);*/
				//					//auto _reference = node->_rayOrigin + node->_rayDir * (node->_collidedT);
				//					//node->SetCoord3D(_reference[0], _reference[1], _reference[2]);
				//				break;
				//			}
				//			else {
				//				//if (eleIndex == 9)
				//				//{
				//				//	/*qDebug("9th point coordinate: %lf %lf %lf", pos[0], pos[1], pos[2]);
				//				//	qDebug("t: %lf, u: %lf, v: %lf",t,u,v);*/
				//				//}
				//				//qDebug("(%4d) node not collided neighbours", eleIndex);
				//			}
				//			//if not, prepare the data for next ring searching
				//			auto _oneRingFaceSet = _face->GetNeighbourFacesOnSurface();
				//			for (auto it2 = _oneRingFaceSet.begin(); it2 != _oneRingFaceSet.end(); ++it2)
				//			{
				//				nextShouldVisitFaceSet.insert((*it2));
				//			}
				//			alreadyVisitedFaceSet.insert(_face);
				//		}
				//		if (_intersectedFace == true)
				//			break;
				//		else {
				//			/*if(_ringNum%100==0)
				//				qDebug("Index: %d", _ringNum);*/
				//				/*if (_ringNum >= 10)
				//					break;*/
				//		}
				//		//delete the extra elements in nextShouldVisitFaceSet comparing to alreadyVisitedFaceSet to form thisShouldVisitFaceVec
				//		thisShouldVisitFaceVec.clear();
				//		for (auto it3 = nextShouldVisitFaceSet.begin(); it3 != nextShouldVisitFaceSet.end(); ++it3)
				//		{
				//			if (alreadyVisitedFaceSet.count((*it3)) == 0)
				//			{
				//				thisShouldVisitFaceVec.push_back(*it3);
				//			}
				//		}
				//		nextShouldVisitFaceSet.clear();
				//	}
				//	//if (_intersectedFace == true)
				//	//{
				//	//	node->_collidedFaceLastPtr = node->_collidedFacePtr;
				//	//	break;
				//	//}
				//}


			}
			else 
			{ 
				//node->isCollided_env = false;
				collidedResult[tetNodeIdx] = 0;
			}

			//if last time, it is in collision, we need to let some points out of boundary while within boundary 3 to remain its spring
			if (node->isCollided_env_last == true)
			{

				//if this time not in collision, but we need to remain spring that is relatively close to collsion
				//check which we should add 
				if (collidedResult[tetNodeIdx] == 0 && node->collidedChamberIdx == chamberIdx)
				{
					
					Eigen::Vector3d nodePosVec;
					PQP_REAL p[3]; node->GetCoord3D(p[0], p[1], p[2]);
					nodePosVec[0] = p[0]; nodePosVec[1] = p[1]; nodePosVec[2] = p[2];

					PQP_DistanceResult dres;	dres.last_tri = _obstaclePQPModel_volumeSurface->last_tri;
					PQP_Distance(&dres, _obstaclePQPModel_volumeSurface, p, 0, 0.0);
					//dres.p1[0], dres.p1[1], dres.p1[2]
					Eigen::Vector3d normalClosestPnt;
					Eigen::Vector3d posClosestPnt;
					Eigen::Vector3d closestPnt_queryPntVec;
					Eigen::Vector3d correspondenceNodePos = { 0.0,0.0,0.0 };
					double dd;
					posClosestPnt[0] = dres.p1[0];
					posClosestPnt[1] = dres.p1[1];
					posClosestPnt[2] = dres.p1[2];
					int minTriIndex = dres.last_tri->id;

					QMeshFace* _collidedFace = _faceArray_volumeSurface_obstacle[minTriIndex];
					Eigen::Vector3d normalCollidedFace;

					_collidedFace->CalPlaneEquation();


					_collidedFace->GetNormal(normalCollidedFace[0], normalCollidedFace[1], normalCollidedFace[2]);

					//why the normal direction is different
					normalCollidedFace = -normalCollidedFace;
					double distance = (nodePosVec - posClosestPnt).dot(normalCollidedFace);
					if (distance > coeff_extension_epsilon_env)
					{
						//node->isCollided_env = false;
						collidedResult[tetNodeIdx] = 0;
						qDebug("\n*********************\nOutside Boundary 3, discard it\n*********************\n");
					}
					else {
						//keep spring but dont update the spring location
						//node->isCollided_env = true;
						collidedResult[tetNodeIdx] = 1;
						counterSpring_env++;
						double nx[3];

						//we need to calculate barycenteric coordinate if we need obstacle to be soft; temporarily not.

						//or, we need to calculate the exact corresponding point
						Eigen::Vector3d springAttach = posClosestPnt + normalCollidedFace * coeff_extension_spring_env;
						node->closestPntOnObstacle[0] = springAttach[0];
						node->closestPntOnObstacle[1] = springAttach[1];
						node->closestPntOnObstacle[2] = springAttach[2];


						//node->GetNormal(node->_rayDir_obstacle[0], node->_rayDir_obstacle[1], node->_rayDir_obstacle[2]);	//update spring direction only
						////To delete
						QMeshNode* correNodeStart = new QMeshNode;
						QMeshNode* correNodeEnd = new QMeshNode;
						Eigen::Vector3d corre1, corre2;
						node->GetCoord3D(corre1[0], corre1[1], corre1[2]);
						correNodeStart->SetCoord3D(corre1[0], corre1[1], corre1[2]);
						_corresPatch->GetNodeList().AddTail(correNodeStart);

						corre2 = posClosestPnt + normalCollidedFace * coeff_extension_spring_env;
						correNodeEnd->SetCoord3D(corre2[0], corre2[1], corre2[2]);
						_corresPatch->GetNodeList().AddTail(correNodeEnd);
						QMeshEdge* correEdge = new QMeshEdge;
						correEdge->_selfCollisionType = 0;
						correEdge->SetStartPoint(correNodeStart);
						correEdge->SetEndPoint(correNodeEnd);
						_corresPatch->GetEdgeList().AddTail(correEdge);
						qDebug("\n*********************\nInside Boundary 3, Keep it\n*********************\n");
						//Sleep(50);
					}

					//old code below

					//old code:
					////check the distance between node and its correspondence on the collided face (now is just free of intersection now)
					////we need to protect this correpondence a little bit to give the constraints
					//Eigen::Vector3d nodePos;
					//Eigen::Vector3d tempPos;
					//Eigen::Vector3d correspondenceNodePos = { 0.0,0.0,0.0 };
					//node->GetCoord3D(nodePos[0], nodePos[1], nodePos[2]);
					//node->_collidedFacePtr_obstacle->GetNodePos(0, tempPos[0], tempPos[1], tempPos[2]);
					//correspondenceNodePos = correspondenceNodePos + node->_barycentricCoord_obstacle[0] * tempPos;
					//node->_collidedFacePtr_obstacle->GetNodePos(1, tempPos[0], tempPos[1], tempPos[2]);
					//correspondenceNodePos = correspondenceNodePos + node->_barycentricCoord_obstacle[1] * tempPos;
					//node->_collidedFacePtr_obstacle->GetNodePos(2, tempPos[0], tempPos[1], tempPos[2]);
					//correspondenceNodePos = correspondenceNodePos + node->_barycentricCoord_obstacle[2] * tempPos;
					//double distance = (nodePos - correspondenceNodePos).norm();
					//double threshold = ShapeUpOperator->coeff_extension_epsilon_env;
					//if (distance > threshold)
					//{
					//	//discard spring 
					//	node->isCollided_env = false;
					//	qDebug("\n*********************\nOutside Boundary 3, discard it\n*********************\n");
					//	//Sleep(50);
					//}
					//else {
					//	//keep spring but dont update the spring location
					//	node->isCollided_env = true;
					//	counterSpring_env++;
					//	double nx[3];
					//	node->GetNormal(node->_rayDir_obstacle[0], node->_rayDir_obstacle[1], node->_rayDir_obstacle[2]);	//update spring direction only
					//	//To delete
					//	QMeshNode* correNodeStart = new QMeshNode;
					//	QMeshNode* correNodeEnd = new QMeshNode;
					//	Eigen::Vector3d corre1, corre2;
					//	node->GetCoord3D(corre1[0], corre1[1], corre1[2]);
					//	correNodeStart->SetCoord3D(corre1[0], corre1[1], corre1[2]);
					//	_corresPatch->GetNodeList().AddTail(correNodeStart);
					//	corre2 = node->_rayOrigin_obstacle + node->_rayDir_obstacle * (node->_collidedT_obstacle + ShapeUpOperator->coeff_extension_epsilon_env);
					//	correNodeEnd->SetCoord3D(corre2[0], corre2[1], corre2[2]);
					//	_corresPatch->GetNodeList().AddTail(correNodeEnd);
					//	QMeshEdge* correEdge = new QMeshEdge;
					//	correEdge->_selfCollisionType = 0;
					//	correEdge->SetStartPoint(correNodeStart);
					//	correEdge->SetEndPoint(correNodeEnd);
					//	_corresPatch->GetEdgeList().AddTail(correEdge);
					//	qDebug("\n*********************\nInside Boundary 3, Keep it\n*********************\n");
					//	//Sleep(50);
					//}

				}
			}

		}

	}
	qDebug("Collision with env: %d", counterSpring_env);
	//_tetModelMesh->collidedPntNum_env = counterSpring_env;
	////////!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!//////////////
	////then check the collision of obstacle mesh with deformable object
	//specific: the boundary points of obstacle with tet elements of deformable object 


	//------------------------------------------------------------------------------------------------


	//------------------------------------------------------------------------------------------------
	//	Step 2: establish rendering patch
	//TetMesh->collidedPntNum_evc = counterSpring_env;
	//if (counter == 1)
	//{
	//	PolygenMesh* poly = new PolygenMesh(TRI);
	//	poly->GetMeshList().AddTail(_patch_corre_env);
	//	poly->setModelName("Correspondence with obstacle");
	//	poly->draw_idx = 2;
	//	AddPolygenToList(poly);
	//}
	//qDebug("Interacting with environment: %d", counterSpring_env);
	////------------------------------------------------------------------------------------------------

	//return true;

	



	return collidedResult;
}

void AABBManager::MarkBoundaryFaceForTetMesh(QMeshPatch* tetMesh)
{
	int eleIndex = 0;
	for (GLKPOSITION Pos = tetMesh->GetFaceList().GetHeadPosition(); Pos != NULL; eleIndex++)
	{
		QMeshFace* face = (QMeshFace*)(tetMesh->GetFaceList().GetNext(Pos));
		if (face->GetLeftTetra() == NULL || face->GetRightTetra() == NULL)
		{
			face->inner = false;
			face->boundary = true;
			for (int i = 0; i < 3; i++) { face->GetNodeRecordPtr(i)->isBoundaryNode = true; face->GetNodeRecordPtr(i)->_isBoundaryNode = true; }
		}
	}

}

//use this to update deformable object's status
void AABBManager::UpdateEnvCollisionStatus(QMeshPatch* tetMesh)
{
	int eleIndex = 0;
	for (GLKPOSITION Pos = tetMesh->GetNodeList().GetHeadPosition(); Pos != NULL;)
	{
		QMeshNode* node = (QMeshNode*)(tetMesh->GetNodeList().GetNext(Pos));

		node->isCollided_env_last = node->isCollided_env;	//last iteration
		node->_isBoundaryNode = false;
		node->MannequinEnvCollisionRoI = false;
		node->isCollided_env = false;

		node->isCollided_last = node->isCollided;
		node->isCollided = false;

		node->CalNormal();
	}

	MarkBoundaryFaceForTetMesh(tetMesh);


	////for each point, when its normal is smaller than 90 degree, then we will check its collision
	Eigen::Vector3d nodePos;
	Eigen::Vector3d normalJudge = {0,-1,0};
	Eigen::Vector3d normalNode;

	//soft finger
	for (GLKPOSITION Pos = tetMesh->GetNodeList().GetHeadPosition(); Pos != NULL;)
	{
		QMeshNode* node = (QMeshNode*)(tetMesh->GetNodeList().GetNext(Pos));
		if (node->isBoundaryNode == true)
		{
			node->MannequinEnvCollisionRoI = true;
			//node->GetNormal(normalNode);
			//double dotVal = normalNode.dot(normalJudge);
			//if (dotVal > 0)
			//{
			//	node->MannequinEnvCollisionRoI = true;

			//	//qDebug("Node is RoI");
			//}

			//else
			//	node->MannequinEnvCollisionRoI = false;
		}
		else
			node->MannequinEnvCollisionRoI = false;
		
	}


}

void AABBManager::SumUpCollisionResultWithEnv(QMeshPatch* tetMesh, Eigen::VectorXd& collidedResult)
{
	int nodeIdx = 0;
	int counter_collision_env = 0;
	for (GLKPOSITION Pos = tetMesh->GetNodeList().GetHeadPosition(); Pos != NULL; nodeIdx++)
	{
		QMeshNode* node = (QMeshNode*)(tetMesh->GetNodeList().GetNext(Pos));
		if (collidedResult[nodeIdx] >= 1)
		{
			node->isCollided_env = true;
			counter_collision_env++;
		}
		else {
			node->isCollided_env = false;
		}
	}
	tetMesh->collidedPntNum_env = counter_collision_env;

}

void AABBManager::operator_updateAABBTree(PolygenMesh* poly)
{
	qDebug("Update AABB Tree...");

	QMeshPatch* treeMesh = (QMeshPatch*)poly->GetMeshList().GetHead();

	_treeVisualizationMesh = treeMesh;
	_refitBoundingBoxPoly = poly;
	
	_treeVisualizationMesh->ClearAllTet();
	//build all depth layers
	_buildAllTreeNodeBoundingBox();

	printf("Tree depth is %d\n", _tree->depth);


}

void AABBManager::CollisionWithEnvQueryChecking(void)
{

#ifdef TET_OBSTACLE
	static int counter = 0;
	counter++;
	Eigen::MatrixXd POS(3, 1);
	bool flag = false;
	Eigen::Vector3d rayOri, rayDir, faceVer[3];
	double u, v, t;
	int counterSpring_env = 0;

	//isCollided_env_last
	if (counter == 1)
	{
		_corresPatch = new QMeshPatch;

		//delete _obstaclePQPModel_volumeSurface;

		//update PQP Model for the closest point query
		int counter_faceNum = 0;
		_obstaclePQPModel_volumeSurface = new PQP_Model();
		for (GLKPOSITION Pos = _obstacleMesh->GetFaceList().GetHeadPosition(); Pos; )
		{
			QMeshFace* face = (QMeshFace*)(_obstacleMesh->GetFaceList().GetNext(Pos));
			if (face->inner == false)
			{
				counter_faceNum++;
			}

		}
		_len_faceArray_volumeSurface_obstacle = counter_faceNum;
		_faceArray_volumeSurface_obstacle = new QMeshFace * [_len_faceArray_volumeSurface_obstacle];

		_obstaclePQPModel_volumeSurface->BeginModel();
		int idx = 0;
		double posPNTArray[3][3];
		for (GLKPOSITION Pos = _obstacleMesh->GetFaceList().GetHeadPosition(); Pos; )
		{
			QMeshFace* face = (QMeshFace*)(_obstacleMesh->GetFaceList().GetNext(Pos));
			if (face->inner == false)
			{
				_faceArray_volumeSurface_obstacle[idx] = face;

				//get vertices information
				for (int i = 0; i < 3; i++)
				{
					face->GetNodeRecordPtr(i)->GetCoord3D(posPNTArray[i][0], posPNTArray[i][1], posPNTArray[i][2]);
				}
				_obstaclePQPModel_volumeSurface->AddTri(posPNTArray[0], posPNTArray[1], posPNTArray[2], idx);
				idx++;
			}
		}
		_obstaclePQPModel_volumeSurface->EndModel();


	}
	else
	{
		_corresPatch->ClearAll();


		delete _obstaclePQPModel_volumeSurface;
		_obstaclePQPModel_volumeSurface = new PQP_Model();
		_obstaclePQPModel_volumeSurface->BeginModel();
		int idx = 0;
		double posPNTArray[3][3];
		for (GLKPOSITION Pos = _obstacleMesh->GetFaceList().GetHeadPosition(); Pos; )
		{
			QMeshFace* face = (QMeshFace*)(_obstacleMesh->GetFaceList().GetNext(Pos));
			if (face->inner == false)
			{
				//get vertices information
				for (int i = 0; i < 3; i++)
				{
					face->GetNodeRecordPtr(i)->GetCoord3D(posPNTArray[i][0], posPNTArray[i][1], posPNTArray[i][2]);
				}
				_obstaclePQPModel_volumeSurface->AddTri(posPNTArray[0], posPNTArray[1], posPNTArray[2], idx);
				idx++;
			}
		}
		_obstaclePQPModel_volumeSurface->EndModel();



	}

	//------------------------------------------------------------------------------------------------
	//	Step 1: mark the boundary points which are lied on the boundary surface
	// first check whether it is in collision

	for (GLKPOSITION Pos = _obstacleMesh->GetFaceList().GetHeadPosition(); Pos != NULL;) {
		QMeshFace* face = (QMeshFace*)(_obstacleMesh->GetFaceList().GetNext(Pos));
		if (face->GetLeftTetra() == NULL || face->GetRightTetra() == NULL)
		{
			face->boundary = true;
			/*qDebug("Surface Boundary Face Found....\n");*/
			for (int i = 0; i < 3; i++) { face->GetNodeRecordPtr(i)->_isBoundaryNode = true; }
			face->CalPlaneEquation();
		}
		else { face->CalPlaneEquation(); }
	}

	int eleIndex = 0;
	for (GLKPOSITION Pos = _obstacleMesh->GetNodeList().GetHeadPosition(); Pos != NULL;)
	{
		QMeshNode* node = (QMeshNode*)(_obstacleMesh->GetNodeList().GetNext(Pos));
		node->isCollided_env_last = node->isCollided_env;	//last iteration
		//node->keepOldSpring = false;				//default: dont keep any spring attached on this point and its correspondence
		node->_isBoundaryNode = false;
		node->isCollided_env = false;
		node->CalNormal();
	}



	//------------------------------------------------------------------------------------------------
	//	Step 2: collision checking and correspondence updating
	//  now, only check collision of deformable object with obstacle

	qDebug("Checking Collision with Env...");
	for (GLKPOSITION Pos = _tetModelMesh->GetNodeList().GetHeadPosition(); Pos;)
	{
		QMeshNode* node = (QMeshNode*)_tetModelMesh->GetNodeList().GetNext(Pos);
		Eigen::Vector3d coord;
		node->GetCoord3D(coord[0], coord[1], coord[2]);

		node->isCollided_env_last = node->isCollided_env;					//status update

		if (node->_isBoundaryNode == true)
		{
			//check whether boundary node is inside the obstacle
			//if yes, then establish the correspondence
			//new code

			//check whether the point is inside env tetrahedron
			bool _checkResult = _singlePntQueryWithTree(node, _obstacleTree, TetraPtrArray_obstacle);


			if (_checkResult)
			{
				counterSpring_env++;
				node->isCollided_env = true;

				//new: find correspondence with closest point query
				//find the closest point  and the its normal
				PQP_REAL p[3]; node->GetCoord3D(p[0], p[1], p[2]);
				PQP_DistanceResult dres;	dres.last_tri = _obstaclePQPModel_volumeSurface->last_tri;
				PQP_Distance(&dres, _obstaclePQPModel_volumeSurface, p, 0, 0.0);
				//dres.p1[0], dres.p1[1], dres.p1[2]
				Eigen::Vector3d normalClosestPnt;
				Eigen::Vector3d posClosestPnt;
				Eigen::Vector3d closestPnt_queryPntVec;
				double dd;
				posClosestPnt[0] = dres.p1[0];
				posClosestPnt[1] = dres.p1[1];
				posClosestPnt[2] = dres.p1[2];
				int minTriIndex = dres.last_tri->id;

				QMeshFace* _collidedFace = _faceArray_volumeSurface_obstacle[minTriIndex];
				Eigen::Vector3d normalCollidedFace;
				_collidedFace->GetNormal(normalCollidedFace[0], normalCollidedFace[1], normalCollidedFace[2]);
				normalCollidedFace = -normalCollidedFace;

				Eigen::Vector3d springAttach = posClosestPnt + normalCollidedFace * coeff_extension_spring_env;
				node->closestPntOnObstacle[0] = springAttach[0];
				node->closestPntOnObstacle[1] = springAttach[1];
				node->closestPntOnObstacle[2] = springAttach[2];

				//To delete
				QMeshNode* correNodeStart = new QMeshNode;
				QMeshNode* correNodeEnd = new QMeshNode;
				Eigen::Vector3d corre1, corre2;
				node->GetCoord3D(corre1[0], corre1[1], corre1[2]);
				correNodeStart->SetCoord3D(corre1[0], corre1[1], corre1[2]);
				_corresPatch->GetNodeList().AddTail(correNodeStart);
				corre2 = posClosestPnt + normalCollidedFace * coeff_extension_spring_env;

				correNodeEnd->SetCoord3D(corre2[0], corre2[1], corre2[2]);
				_corresPatch->GetNodeList().AddTail(correNodeEnd);
				QMeshEdge* correEdge = new QMeshEdge;
				correEdge->_selfCollisionType = 1;
				correEdge->SetStartPoint(correNodeStart);
				correEdge->SetEndPoint(correNodeEnd);
				_corresPatch->GetEdgeList().AddTail(correEdge);


				//old code below

				//old: find corrspondence with normal direction
				//correspondence calculation
				//for each collided point
				//if (node->_collidedRealTetraList.size() != 1)
				//{
				//	qDebug("(%d) (from zero) collided tet number is bigger than 1, error...", eleIndex);
				//}
				////calculate correspondence
				//for (auto it = node->_collidedRealTetraList.begin(); it != node->_collidedRealTetraList.end(); ++it)
				//{
				//	double pos[3], norm[3];
				//	node->GetCoord3D(pos[0], pos[1], pos[2]);
				//	node->GetNormal(norm[0], norm[1], norm[2]);
				//	bool _intersectedFace = false;
				//	Ray _normal2;
				//	int _ringNum = 0;
				//	_normal2.Origin = Eigen::Vector3d(pos[0], pos[1], pos[2]);
				//	//here normal direction is in-ward normal
				//	_normal2.Dir = Eigen::Vector3d(norm[0], norm[1], norm[2]);
				//	QMeshTetra* _collidedTet = TetraPtrArray_obstacle[(*it)];
				//	//First, collect all two-ring boundary faces around the boundary face of _collidedTet. Then check the intersection
				//	//using std::set
				//	//To be more specific, face's edgelist -> facelist -> edgelist -> facelist
				//	std::vector<QMeshFace*> thisShouldVisitFaceVec;
				//	std::set<QMeshFace*> nextShouldVisitFaceSet;
				//	std::set<QMeshFace*> alreadyVisitedFaceSet;
				//	QMeshFace* _boundaryFace;
				//	bool _flag1 = false;
				//	//find the boundary tet first one-ring, then second-ring, then three-ring...until find the boundary tet
				//	auto _boundaryTetSet = _collidedTet->CalBoundaryTetList();
				//	//qDebug("%d boundary set num: %d", eleIndex, _boundaryTetSet.size());
				//	//acquire the original of flooding faceList
				//	for (auto itBoundaryTet = _boundaryTetSet.begin(); itBoundaryTet != _boundaryTetSet.end(); ++itBoundaryTet)
				//	{
				//		QMeshTetra* _bTet = (*itBoundaryTet);
				//		for (int i = 0; i < 4; i++)
				//		{
				//			QMeshFace* face = _bTet->GetFaceRecordPtr(i + 1);
				//			if (face->inner == false)
				//				thisShouldVisitFaceVec.push_back(face);
				//		}
				//	}
				//	//until the intersecting face is found, we then will jump out of loop
				//	while (1)
				//	{
				//		/*if(eleIndex==9)
				//			break;*/
				//		_ringNum++;
				//		//for each face in this loop should be visited
				//		for (int i = 0; i < thisShouldVisitFaceVec.size(); i++)
				//		{
				//			QMeshFace* _face = thisShouldVisitFaceVec[i];
				//			//if this face is intersecting with the Ray
				//			//boundary face.
				//			Eigen::Vector3d _A, _B, _C, _N;
				//			double u, v, t;
				//			_face->GetNodeRecordPtr(0)->GetCoord3D(_A[0], _A[1], _A[2]);
				//			_face->GetNodeRecordPtr(1)->GetCoord3D(_B[0], _B[1], _B[2]);
				//			_face->GetNodeRecordPtr(2)->GetCoord3D(_C[0], _C[1], _C[2]);
				//			bool _flag = false;
				//			if (node == _face->GetNodeRecordPtr(0) || node==_face->GetNodeRecordPtr(1)|| node == _face->GetNodeRecordPtr(2))
				//			{
				//				//if the checking point is one of the checking triangle, then ignore this
				//			}
				//			else {
				//				//if not, directly check whether this is intersecting with triangle.
				//				_flag = _tetModelMesh->_intersect_triangle(_normal2, _A, _B, _C, t, u, v, _N);
				//			}
				//			if (_flag == true)
				//			{
				//				_intersectedFace = true;
				//				//qDebug("(%4d) node collided with face (%4d)", eleIndex, _face->GetIndexNo());
				//				node->_collidedFacePtr_obstacle = _face;
				//				node->_barycentricCoord_obstacle[0] = 1 - u - v;
				//				node->_barycentricCoord_obstacle[1] = u;
				//				node->_barycentricCoord_obstacle[2] = v;
				//				////debug usage: why intersecting face is so far
				//				//Eigen::Vector3d originPnt(_normal.Origin);
				//				//if ((originPnt - _A).norm() > 10)
				//				//{
				//				//	qDebug("pos: %lf %lf %lf", originPnt[0], originPnt[1], originPnt[2]);
				//				//	qDebug("(%4d) node collided too far", eleIndex);
				//				//	qDebug("%d boundary set num: %d", eleIndex, _boundaryTetSet.size());
				//				//}
				//				//node coorespondence finding
				//				node->_rayOrigin_obstacle = _normal2.Origin;
				//				node->_rayDir_obstacle = _normal2.Dir;
				//				node->_collidedT_obstacle = t;
				//				//delete the wrong spring
				//				Eigen::Vector3d distanceVec = node->_rayDir_obstacle * node->_collidedT_obstacle;
				//				//qDebug("distance norm is %lf", distanceVec.norm());
				//				if (distanceVec.norm() >= threshold_envCollisionSpring_maximum)
				//				{
				//					counterSpring_env--;
				//					node->isCollided_env = false;
				//				}
				//				//To delete
				//				QMeshNode* correNodeStart = new QMeshNode;
				//				QMeshNode* correNodeEnd = new QMeshNode;
				//				Eigen::Vector3d corre1, corre2;
				//				node->GetCoord3D(corre1[0], corre1[1], corre1[2]);
				//				correNodeStart->SetCoord3D(corre1[0], corre1[1], corre1[2]);
				//				_corresPatch->GetNodeList().AddTail(correNodeStart);
				//				corre2 = node->_rayOrigin_obstacle + node->_rayDir_obstacle * (node->_collidedT_obstacle + ShapeUpOperator->coeff_extension_spring_env);
				//				correNodeEnd->SetCoord3D(corre2[0], corre2[1], corre2[2]);
				//				_corresPatch->GetNodeList().AddTail(correNodeEnd);
				//				QMeshEdge* correEdge = new QMeshEdge;
				//				correEdge->_selfCollisionType = 0;
				//				correEdge->SetStartPoint(correNodeStart);
				//				correEdge->SetEndPoint(correNodeEnd);
				//				_corresPatch->GetEdgeList().AddTail(correEdge);
				//				/*if(t>10)
				//					qDebug("(%4d) node collided too far", eleIndex);*/
				//					//auto _reference = node->_rayOrigin + node->_rayDir * (node->_collidedT);
				//					//node->SetCoord3D(_reference[0], _reference[1], _reference[2]);
				//				break;
				//			}
				//			else {
				//				//if (eleIndex == 9)
				//				//{
				//				//	/*qDebug("9th point coordinate: %lf %lf %lf", pos[0], pos[1], pos[2]);
				//				//	qDebug("t: %lf, u: %lf, v: %lf",t,u,v);*/
				//				//}
				//				//qDebug("(%4d) node not collided neighbours", eleIndex);
				//			}
				//			//if not, prepare the data for next ring searching
				//			auto _oneRingFaceSet = _face->GetNeighbourFacesOnSurface();
				//			for (auto it2 = _oneRingFaceSet.begin(); it2 != _oneRingFaceSet.end(); ++it2)
				//			{
				//				nextShouldVisitFaceSet.insert((*it2));
				//			}
				//			alreadyVisitedFaceSet.insert(_face);
				//		}
				//		if (_intersectedFace == true)
				//			break;
				//		else {
				//			/*if(_ringNum%100==0)
				//				qDebug("Index: %d", _ringNum);*/
				//				/*if (_ringNum >= 10)
				//					break;*/
				//		}
				//		//delete the extra elements in nextShouldVisitFaceSet comparing to alreadyVisitedFaceSet to form thisShouldVisitFaceVec
				//		thisShouldVisitFaceVec.clear();
				//		for (auto it3 = nextShouldVisitFaceSet.begin(); it3 != nextShouldVisitFaceSet.end(); ++it3)
				//		{
				//			if (alreadyVisitedFaceSet.count((*it3)) == 0)
				//			{
				//				thisShouldVisitFaceVec.push_back(*it3);
				//			}
				//		}
				//		nextShouldVisitFaceSet.clear();
				//	}
				//	//if (_intersectedFace == true)
				//	//{
				//	//	node->_collidedFaceLastPtr = node->_collidedFacePtr;
				//	//	break;
				//	//}
				//}


			}
			else { node->isCollided_env = false; }

			//if last time, it is in collision, we need to let some points out of boundary while within boundary 3 to remain its spring
			if (node->isCollided_env_last == true)
			{

				//if this time not in collision, but we need to remain spring that is relatively close to collsion
				if (node->isCollided_env == false)
				{
					Eigen::Vector3d nodePosVec;
					PQP_REAL p[3]; node->GetCoord3D(p[0], p[1], p[2]);
					nodePosVec[0] = p[0]; nodePosVec[1] = p[1]; nodePosVec[2] = p[2];

					PQP_DistanceResult dres;	dres.last_tri = _obstaclePQPModel_volumeSurface->last_tri;
					PQP_Distance(&dres, _obstaclePQPModel_volumeSurface, p, 0, 0.0);
					//dres.p1[0], dres.p1[1], dres.p1[2]
					Eigen::Vector3d normalClosestPnt;
					Eigen::Vector3d posClosestPnt;
					Eigen::Vector3d closestPnt_queryPntVec;
					Eigen::Vector3d correspondenceNodePos = { 0.0,0.0,0.0 };
					double dd;
					posClosestPnt[0] = dres.p1[0];
					posClosestPnt[1] = dres.p1[1];
					posClosestPnt[2] = dres.p1[2];
					int minTriIndex = dres.last_tri->id;

					QMeshFace* _collidedFace = _faceArray_volumeSurface_obstacle[minTriIndex];
					Eigen::Vector3d normalCollidedFace;
					_collidedFace->GetNormal(normalCollidedFace[0], normalCollidedFace[1], normalCollidedFace[2]);
					normalCollidedFace = -normalCollidedFace;
					double distance = (nodePosVec - posClosestPnt).dot(normalCollidedFace);
					if (distance > coeff_extension_epsilon_env)
					{
						node->isCollided_env = false;
						qDebug("\n*********************\nOutside Boundary 3, discard it\n*********************\n");
					}
					else {
						//keep spring but dont update the spring location
						node->isCollided_env = true;
						counterSpring_env++;
						double nx[3];

						//we need to calculate barycenteric coordinate if we need obstacle to be soft

						//or, we need to calculate the exact corresponding point
						Eigen::Vector3d springAttach = posClosestPnt + normalCollidedFace * coeff_extension_spring_env;
						node->closestPntOnObstacle[0] = springAttach[0];
						node->closestPntOnObstacle[1] = springAttach[1];
						node->closestPntOnObstacle[2] = springAttach[2];


						//node->GetNormal(node->_rayDir_obstacle[0], node->_rayDir_obstacle[1], node->_rayDir_obstacle[2]);	//update spring direction only
						//To delete
						QMeshNode* correNodeStart = new QMeshNode;
						QMeshNode* correNodeEnd = new QMeshNode;
						Eigen::Vector3d corre1, corre2;
						node->GetCoord3D(corre1[0], corre1[1], corre1[2]);
						correNodeStart->SetCoord3D(corre1[0], corre1[1], corre1[2]);
						_corresPatch->GetNodeList().AddTail(correNodeStart);

						corre2 = posClosestPnt + normalCollidedFace * coeff_extension_spring_env;
						correNodeEnd->SetCoord3D(corre2[0], corre2[1], corre2[2]);
						_corresPatch->GetNodeList().AddTail(correNodeEnd);
						QMeshEdge* correEdge = new QMeshEdge;
						correEdge->_selfCollisionType = 0;
						correEdge->SetStartPoint(correNodeStart);
						correEdge->SetEndPoint(correNodeEnd);
						_corresPatch->GetEdgeList().AddTail(correEdge);
						qDebug("\n*********************\nInside Boundary 3, Keep it\n*********************\n");
						//Sleep(50);
					}

					//old code below

					//old code:
					////check the distance between node and its correspondence on the collided face (now is just free of intersection now)
					////we need to protect this correpondence a little bit to give the constraints
					//Eigen::Vector3d nodePos;
					//Eigen::Vector3d tempPos;
					//Eigen::Vector3d correspondenceNodePos = { 0.0,0.0,0.0 };
					//node->GetCoord3D(nodePos[0], nodePos[1], nodePos[2]);
					//node->_collidedFacePtr_obstacle->GetNodePos(0, tempPos[0], tempPos[1], tempPos[2]);
					//correspondenceNodePos = correspondenceNodePos + node->_barycentricCoord_obstacle[0] * tempPos;
					//node->_collidedFacePtr_obstacle->GetNodePos(1, tempPos[0], tempPos[1], tempPos[2]);
					//correspondenceNodePos = correspondenceNodePos + node->_barycentricCoord_obstacle[1] * tempPos;
					//node->_collidedFacePtr_obstacle->GetNodePos(2, tempPos[0], tempPos[1], tempPos[2]);
					//correspondenceNodePos = correspondenceNodePos + node->_barycentricCoord_obstacle[2] * tempPos;
					//double distance = (nodePos - correspondenceNodePos).norm();
					//double threshold = ShapeUpOperator->coeff_extension_epsilon_env;
					//if (distance > threshold)
					//{
					//	//discard spring 
					//	node->isCollided_env = false;
					//	qDebug("\n*********************\nOutside Boundary 3, discard it\n*********************\n");
					//	//Sleep(50);
					//}
					//else {
					//	//keep spring but dont update the spring location
					//	node->isCollided_env = true;
					//	counterSpring_env++;
					//	double nx[3];
					//	node->GetNormal(node->_rayDir_obstacle[0], node->_rayDir_obstacle[1], node->_rayDir_obstacle[2]);	//update spring direction only
					//	//To delete
					//	QMeshNode* correNodeStart = new QMeshNode;
					//	QMeshNode* correNodeEnd = new QMeshNode;
					//	Eigen::Vector3d corre1, corre2;
					//	node->GetCoord3D(corre1[0], corre1[1], corre1[2]);
					//	correNodeStart->SetCoord3D(corre1[0], corre1[1], corre1[2]);
					//	_corresPatch->GetNodeList().AddTail(correNodeStart);
					//	corre2 = node->_rayOrigin_obstacle + node->_rayDir_obstacle * (node->_collidedT_obstacle + ShapeUpOperator->coeff_extension_epsilon_env);
					//	correNodeEnd->SetCoord3D(corre2[0], corre2[1], corre2[2]);
					//	_corresPatch->GetNodeList().AddTail(correNodeEnd);
					//	QMeshEdge* correEdge = new QMeshEdge;
					//	correEdge->_selfCollisionType = 0;
					//	correEdge->SetStartPoint(correNodeStart);
					//	correEdge->SetEndPoint(correNodeEnd);
					//	_corresPatch->GetEdgeList().AddTail(correEdge);
					//	qDebug("\n*********************\nInside Boundary 3, Keep it\n*********************\n");
					//	//Sleep(50);
					//}

				}
			}

		}

	}
	qDebug("Collision with env: %d", counterSpring_env);
	_tetModelMesh->collidedPntNum_env = counterSpring_env;
	////////!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!//////////////
	////then check the collision of obstacle mesh with deformable object
	//specific: the boundary points of obstacle with tet elements of deformable object 


	//------------------------------------------------------------------------------------------------


	//------------------------------------------------------------------------------------------------
	//	Step 2: establish rendering patch
	//TetMesh->collidedPntNum_evc = counterSpring_env;
	//if (counter == 1)
	//{
	//	PolygenMesh* poly = new PolygenMesh(TRI);
	//	poly->GetMeshList().AddTail(_patch_corre_env);
	//	poly->setModelName("Correspondence with obstacle");
	//	poly->draw_idx = 2;
	//	AddPolygenToList(poly);
	//}
	//qDebug("Interacting with environment: %d", counterSpring_env);
	////------------------------------------------------------------------------------------------------

	//return true;
#endif

}



//Query if this point is inside the tetrahedrons of model
//Basically, we will find out all the collided leaf nodes in the AABB tree, and check whether there is a collision
//There will be two possible situations:
//	1. _node is inside the collision box but not inside the tetrahedron the collision box contains
//  2. _node is inside the collision box and also inside the tetrahedron the collision box contains
//			2.1 _node is one of the tetrahedron node -> ignore
//			2.2 _node is not one of the tetrahedron node -> SELF COLLIDED POINT -> What we want!


//para: @_node:  node to be checked against the whole model
//return: @(bool): true -> collision exist
bool AABBManager::_singlePntQueryWithTree(QMeshNode* _node, aabbTree* _aabbtree, QMeshTetra** _tetraPtrArray)
{

	//------------------------------------------------------------------------------------------------
	//	Step 1: traverse the tree to find out all the possible collided tree nodes
	_node->_collidedTetraList.clear();					//clean the embeded potential collided tetralist 
	_node->_collidedRealTetraList.clear();				//clean the embeded real collided tetralist
	_node->_collidedTreeLeafNodeIndexList.clear();		//clean
	_node->_collidedRealTreeLeafNodeIndexList.clear();	//clean
	//_node->isCollided = false;							//reset collision flag

	double pos[3];
	_node->GetCoord3D(pos[0], pos[1], pos[2]);
	//check if root tree node is inside the bounding box
	bool _collided = _isCollided(pos, _aabbtree->nodes[_aabbtree->rootIndex].collisionBox);
	if (_collided == false)
	{
		//qDebug("No Collision....\n");
		return false;
	}
	else {
		//qDebug("Hello..... Reach here");
	}

	//check left child and right child, and further push the potential tetra into _collidedTetraList.
	_recursiveQueryWithTree(_aabbtree, _node, _aabbtree->rootIndex, 1, 1);
	_recursiveQueryWithTree(_aabbtree, _node, _aabbtree->rootIndex, 2, 1);

	//
	//------------------------------------------------------------------------------------------------

	//------------------------------------------------------------------------------------------------
	//	Step 2: post-processing to see whether there is actually a collision 
	//There will be two possible situations:
	//	1. _node is inside the collision box but not inside the tetrahedron the collision box contains
	//  2. _node is inside the collision box and also inside the tetrahedron the collision box contains
	//			2.1 _node is one of the tetrahedron node -> ignore
	//			2.2 _node is not one of the tetrahedron node -> SELF COLLIDED POINT -> What we want!
	// So the collision will be: inside the tetrahedron and this tetrahedron should not contain our query node

	_node->GetCoord3D(pos[0], pos[1], pos[2]);
	auto leafNodeIter = _node->_collidedTreeLeafNodeIndexList.begin();
	for (auto it = _node->_collidedTetraList.begin(); it != _node->_collidedTetraList.end(); ++it, ++leafNodeIter) {
		QMeshTetra* _potentialTet = _tetraPtrArray[(*it)];
		//check  whether _node is inside this tetrahedron 
		bool _inside = _potentialTet->_judgeInsideTet(pos);	//true->inside the tet
		if (_inside == false) continue;
		//judge whether this _node is one of the tetrahedron points
		bool _inTetPntList = _potentialTet->_judgeIsInPntList(_node);
		if (_inTetPntList == false)
		{
			//if it is the real collided case
			_node->_collidedRealTetraList.push_back((*it));
			_node->_collidedRealTreeLeafNodeIndexList.push_back((*leafNodeIter));
		}
		else {
			//qDebug("Inside tet but is one of the tet node...\n");
		}


	}




	if (_node->_collidedRealTreeLeafNodeIndexList.empty() == false)
	{
		static int count = 0;
		count++;
		//_node->isCollided = true;
		//_node->isCollided_debug = true;		//only for debug
		double pos_debug[3];
		_node->GetCoord3D(pos_debug[0], pos_debug[1], pos_debug[2]);
		//qDebug("9th point coordinate: %lf %lf %lf", pos_debug[0], pos_debug[1], pos_debug[2]);


		/*if (_node->GetIndexNo() == 2231)
		{

		}*/
		/*for (auto it = _node->_collidedRealTreeLeafNodeIndexList.begin(); it != _node->_collidedRealTreeLeafNodeIndexList.end(); ++it)
		{
			_addSingleAABBboundingBox_debugQuery(_aabbtree->nodes[(*it)].collisionBox, _debugPatch);
		}

		for (auto it = _node->_collidedRealTetraList.begin(); it != _node->_collidedRealTetraList.end(); ++it)
		{
			_addSingleTetrahedron_debugQuery(TetraPtrArray[(*it)], _debugPatch);
		}*/





		return true;
	}
	else
	{
		//_node->isCollided = false;
		return false;

	}

	//------------------------------------------------------------------------------------------------
}

void AABBManager::GetTetMesh(QMeshPatch* tetMesh)
{
	_tetModelMesh = tetMesh;
}



//Recursively query AABB tree: check which leaf node is potentially colliding with pos[3]

//para: @pos: point that will be checked
//		@_nodeIdx: parent node idx
//		@_flag:  1 -> left child; 2 -> right child (This child node is the left or right node of its parent)
//		@_recursiveLayer:	This node's depth
void AABBManager::_recursiveQueryWithTree(aabbTree* _aabbTree, QMeshNode* _node, int _nodeIdx, int _flag, int _recursiveLayer)
{

	//------------------------------------------------------------------------------------------------
	//	Step 1: terminal condition: when it is leaf node
	AABBTREEArrayNode* thisNode;
	int thisNodeIdx = -1;
	if (_flag == 1)
		thisNodeIdx = _aabbTree->nodes[_nodeIdx].child1;
	else
		thisNodeIdx = _aabbTree->nodes[_nodeIdx].child2;

	thisNode = &(_aabbTree->nodes[thisNodeIdx]);
	double pos[3];
	_node->GetCoord3D(pos[0], pos[1], pos[2]);

	if (thisNode->isLeaf == true)
	{

		if (thisNode->objectIndex != -1)
		{
			//when this tree node contains one tetra

			//check whether this leaf node's collisionBox is contradicting with _node 
			bool _collided = _isCollided(pos, thisNode->collisionBox);
			//add tetra into the _node's tetralist
			if (_collided == true)
			{
				for (auto it = thisNode->tetraList.begin(); it != thisNode->tetraList.end(); ++it) {
					_node->_collidedTetraList.push_back((*it)->_idx);
					_node->_collidedTreeLeafNodeIndexList.push_back(thisNodeIdx);
				}

			}

		}

		return;
	}

	//------------------------------------------------------------------------------------------------

	//------------------------------------------------------------------------------------------------
	//	Step 2: check whether this level is intersecting with _node
	//			if not, directly return 
	//			if yes, go ahead to check whether its left and right child will collided
	bool _collided = _isCollided(pos, thisNode->collisionBox);
	if (_collided == false)
		return;
	//------------------------------------------------------------------------------------------------


	//------------------------------------------------------------------------------------------------
	//	Step 3: call recursive function 
	_recursiveQueryWithTree(_aabbTree, _node, thisNodeIdx, 1, _recursiveLayer + 1);
	_recursiveQueryWithTree(_aabbTree, _node, thisNodeIdx, 2, _recursiveLayer + 1);

	//------------------------------------------------------------------------------------------------

}
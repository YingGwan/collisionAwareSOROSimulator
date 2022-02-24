#pragma once
//
//  AABBTreeStructure.h
//  AABB tree structure to fast collision query for the tetrahedron mesh
//
//  Created by Yingjun on 18/01/22.
//  Copyright (c) 2022 Yingjun Tian. All rights reserved.
//
#include<list>
#include"QMeshTetra.h"
//frequently used operation
#define MIN(a,b)	((a)<(b))?(a):(b)
#define MAX(a,b)	((a)>(b))?(a):(b)

//following operation is in 3d
#define CROSS(dest,v1,v2)                      \
              dest[0]=v1[1]*v2[2]-v1[2]*v2[1]; \
              dest[1]=v1[2]*v2[0]-v1[0]*v2[2]; \
              dest[2]=v1[0]*v2[1]-v1[1]*v2[0];

#define DOT(v1,v2) (v1[0]*v2[0]+v1[1]*v2[1]+v1[2]*v2[2])

#define SUB(dest,v1,v2) dest[0]=v1[0]-v2[0]; dest[1]=v1[1]-v2[1]; dest[2]=v1[2]-v2[2]; 

#define ADD(dest,v1,v2) dest[0]=v1[0]+v2[0]; dest[1]=v1[1]+v2[1]; dest[2]=v1[2]+v2[2]; 

#define MULT(dest,v,factor) dest[0]=factor*v[0]; dest[1]=factor*v[1]; dest[2]=factor*v[2];

#define SET(dest,src) dest[0]=src[0]; dest[1]=src[1]; dest[2]=src[2]; 


//maximum node allowance
#define MAXNUM_OF_AABBNODE_ONTREE		(1000000)	// Specifying the maximal num of nodes,
													// where different number should be use with different PCs according to the memory
#define MAXLEVEL_OF_STACK				(1000)      // Specifying the maximal level of stack for tree-traversal
#define INF								(1e10)		// Infinity	
#define NINF							(-1e10)		// Negative infinity


//each node of AABB Tree contains one bounding box enclosing one tetrahedron
struct AABB
{
	double lowerBound[3];
	double upperBound[3];
};

//tree node of AABB Tree
typedef struct aabbTreeArrayNode {
	AABB box;						//each node should contain a bounding box
	AABB collisionBox;				//box used to check collision
	int parentIndex;				//index in the Tree
	int child1;						//index in the Tree
	int child2;						//index in the Tree
	bool isLeaf=false;				//whether it is leaf node
	int objectIndex=-2;				//point to the tetrahedron index in the tetrahedron mesh
									// -2    -> init value
									// -1    -> no tet inside
									// other -> tet index
	int depth = -1;					// 0	 -> root level
									// 1     -> second level
	//need to specific the tetrahedron index it contains
	std::list<QMeshTetra*> tetraList;

	//this is not appropriate to pre-allocate a large memory cuz this would casue lackage of memory
	//int tetraIndexSet[1000];

}AABBTREEArrayNode;

//aabb Tree data structure
typedef struct aabbTree
{
	AABBTREEArrayNode* nodes;		//memcpy(treePtr->planeArray,bspPlaneArray,sizeof(BSPTREEArrayPlane)*bspPlaneArraySize);
	int nodeCount;
	int rootIndex;		
	int depth=0;					//maximum depth of all nodes

}AABBTREE;

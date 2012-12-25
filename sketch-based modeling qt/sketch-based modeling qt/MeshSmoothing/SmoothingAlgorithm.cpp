#include "SmoothingAlgorithm.h"
#include "../GeometryAlgorithm.h"
#include "../OBJHandle.h"

void CSmoothingAlgorithm::BilateralSmooth(KW_Mesh& Mesh,vector<Vertex_handle>& vecVertexToSmooth,double dSigmaC,double dKernelSize,double dSigmaS,int iNormalRingNum)
{
	//calculate the normal,just one-ring involved at present
	OBJHandle::UnitizeCGALPolyhedron(Mesh,false,false);
	
	if (dSigmaC==0)
	{
		dSigmaC=0.05;
		dKernelSize=dSigmaC*2;
	}

	assert(dKernelSize==2*dSigmaC);

	vector<Point_3> vecNewPos;

	for (unsigned int iVertex=0;iVertex<vecVertexToSmooth.size();iVertex++)
	{
		//get kernel vertices and their distances to the current vertex
		vector<Vertex_handle> vecKernelVertex;
		vector<double> vecDistance;
		GetKernelVertex(Mesh,vecVertexToSmooth.at(iVertex),dKernelSize,vecKernelVertex,vecDistance);

		//get offsets and derivation
		vector<double> vecOffset;
		GetOffsets(vecVertexToSmooth.at(iVertex),vecKernelVertex,vecOffset);
		double dDerivation=GeometryAlgorithm::GetDerivation(vecOffset);
		dSigmaS=dDerivation;
		//compute new positions
		double dSum,dNormalizer;
		dSum=dNormalizer=0;
		for (unsigned int iKernelVertex=0;iKernelVertex<vecKernelVertex.size();iKernelVertex++)
		{
			double dWC=exp(-vecDistance.at(iKernelVertex)*vecDistance.at(iKernelVertex)/(2*dSigmaC*dSigmaC));
			double dWS=exp(-vecOffset.at(iKernelVertex)*vecOffset.at(iKernelVertex)/(2*dSigmaS*dSigmaS));
			dSum=dSum+dWC*dWS*vecOffset.at(iKernelVertex);
			dNormalizer=dNormalizer+dWC*dWS;
		}
		Vector_3 VectorToMove=vecVertexToSmooth.at(iVertex)->normal()*(dSum/dNormalizer);
		if (dSum==0)
		{
			VectorToMove=Vector_3(0,0,0);
		}
		//DBWindowWrite("vertex: %d  VectorToMove: %f %f %f\n",iVertex,VectorToMove.x(),VectorToMove.y(),VectorToMove.z());
		Point_3 NewPos=vecVertexToSmooth.at(iVertex)->point()+VectorToMove;
		vecNewPos.push_back(NewPos);
	}

	//update vertex position
	for (unsigned int i=0;i<vecNewPos.size();i++)
	{
		//DBWindowWrite("old pos: %f %f %f\n",vecVertexToSmooth.at(i)->point().x(),vecVertexToSmooth.at(i)->point().y(),vecVertexToSmooth.at(i)->point().z());
		//DBWindowWrite("new pos: %f %f %f\n",vecNewPos.at(i).x(),vecNewPos.at(i).y(),vecNewPos.at(i).z());
		vecVertexToSmooth.at(i)->point()=vecNewPos.at(i);
	}

	OBJHandle::UnitizeCGALPolyhedron(Mesh,false,false);
	Mesh.SetRenderInfo(true,true,false,false,false);
}

int CSmoothingAlgorithm::GetKernelVertex(KW_Mesh& Mesh,Vertex_handle hVertex,double dKernelSize,vector<Vertex_handle>& vecKernelVertex,vector<double>& vecDistance)
{
	//for (Vertex_iterator i=Mesh.vertices_begin();i!=Mesh.vertices_end();i++)
	//{
	//	if (i==hVertex)
	//	{
	//		continue;
	//	}
	//	double dDistance=sqrt(CGAL::squared_distance(hVertex->point(),i->point()));
	//	if (dDistance<dKernelSize)
	//	{
	//		vecKernelVertex.push_back(i);
	//		vecDistance.push_back(dDistance);
	//	}
	//}

	vector<Vertex_handle> vecCurrentKernelRing;
	Halfedge_around_vertex_circulator Havc=hVertex->vertex_begin();
	do 
	{	
		double dDistance=sqrt(CGAL::squared_distance(hVertex->point(),Havc->opposite()->vertex()->point()));
		if (dDistance<dKernelSize)
		{
			vecCurrentKernelRing.push_back(Havc->opposite()->vertex());
			vecKernelVertex.push_back(Havc->opposite()->vertex());
			vecDistance.push_back(dDistance);
		}
		Havc++;
	} while(Havc!=hVertex->vertex_begin());

	while (!vecCurrentKernelRing.empty())
	{
		vector<Vertex_handle> vecNextKernelRing;
		for (unsigned int i=0;i<vecCurrentKernelRing.size();i++)
		{
			Halfedge_around_vertex_circulator Havc=vecCurrentKernelRing.at(i)->vertex_begin();
			do 
			{
				if (Havc->opposite()->vertex()!=hVertex)
				{
					vector<Vertex_handle>::iterator pFind=find(vecKernelVertex.begin(),vecKernelVertex.end(),Havc->opposite()->vertex());
					vector<Vertex_handle>::iterator pFind2=find(vecNextKernelRing.begin(),vecNextKernelRing.end(),Havc->opposite()->vertex());
					if (pFind==vecKernelVertex.end()&&pFind2==vecNextKernelRing.end())
					{
						double dDistance=sqrt(CGAL::squared_distance(hVertex->point(),Havc->opposite()->vertex()->point()));
						if (dDistance<dKernelSize)
						{
							vecNextKernelRing.push_back(Havc->opposite()->vertex());
							vecKernelVertex.push_back(Havc->opposite()->vertex());
							vecDistance.push_back(dDistance);
						}
					}
				}
				Havc++;
			} while(Havc!=vecCurrentKernelRing.at(i)->vertex_begin());

		}
		vecCurrentKernelRing=vecNextKernelRing;
		vecNextKernelRing.clear();
	}

	return vecKernelVertex.size();
}

int CSmoothingAlgorithm::GetOffsets(Vertex_handle hVertex,vector<Vertex_handle>& vecKernelVertex,vector<double>& vecOffsets)
{
	for (unsigned int iKernelVertex=0;iKernelVertex<vecKernelVertex.size();iKernelVertex++)
	{
		Vector_3 EdgeVector=vecKernelVertex.at(iKernelVertex)->point()-hVertex->point();
		double dOffset=EdgeVector.x()*hVertex->normal().x()+EdgeVector.y()*hVertex->normal().y()+EdgeVector.z()*hVertex->normal().z();
		//DBWindowWrite("offset: %f\n",dOffset);
		vecOffsets.push_back(dOffset);
	}

	return vecOffsets.size();
}
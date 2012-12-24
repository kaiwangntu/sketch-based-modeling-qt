#include "MeshDeformation.h"

void CMeshDeformation::DeformationLocalRefine(KW_Mesh OldMesh,KW_Mesh& NewMesh,vector<Point_3> OldHandlePos,vector<Point_3> NewHandlePos, double dSquaredDistanceThreshold,
											  vector<Vertex_handle>& vecHandleNb,vector<Vertex_handle>& ROIVertices,vector<Vertex_handle>& vecAnchorVertices, 
											  vector<Point_3>& testCentroidPoint,vector<Point_3>& testmovedCentroidPoint,vector<Facet_handle>& testfhRefineTri)
{
	std::vector<Facet_handle> fhRefineTri;
	GetRefineTri(OldMesh,NewMesh,OldHandlePos,NewHandlePos,dSquaredDistanceThreshold,
		testCentroidPoint,testmovedCentroidPoint,fhRefineTri);
	testfhRefineTri=fhRefineTri;

	//restore the mesh geometry
	for ( Vertex_iterator i=OldMesh.vertices_begin(),j=NewMesh.vertices_begin();
		i!=OldMesh.vertices_end(),j!=NewMesh.vertices_end(); i++,j++)
	{
		j->point()=i->point();
	}

	//mark the vertices
	SetVerMark(NewMesh,vecHandleNb,ROIVertices,vecAnchorVertices);

	//compute the new edge point positions in advance
	//make use of the edge index and NewMidpoint in wedge-edge-based deformation
	//may be very slow!and may contain error for border edges
	Halfedge_iterator  HI=NewMesh.halfedges_begin();
	do 
	{
		HI->SetEdgeIndex(-1);
		HI++;
	} while(HI!=NewMesh.halfedges_end());
	//set the index of each edge starting from 0
	int iIndex=0;
	HI=NewMesh.halfedges_begin();
	do 
	{
		if (HI->GetEdgeIndex()==-1 || HI->opposite()->GetEdgeIndex()==-1)
		{
			HI->SetEdgeIndex(iIndex);
			HI->opposite()->SetEdgeIndex(iIndex);
			iIndex++;
			//compute the new edge point position
			Point_3 NearPoints[2];
			NearPoints[0]=HI->vertex()->point();
			NearPoints[1]=HI->opposite()->vertex()->point();
			Point_3 FarPoints[2];
			FarPoints[0]=HI->next()->vertex()->point();
			FarPoints[1]=HI->opposite()->next()->vertex()->point();
			double NewX=3.0/8.0*(NearPoints[0].x()+NearPoints[1].x())+1.0/8.0*(FarPoints[0].x()+FarPoints[1].x());
			double NewY=3.0/8.0*(NearPoints[0].y()+NearPoints[1].y())+1.0/8.0*(FarPoints[0].y()+FarPoints[1].y());
			double NewZ=3.0/8.0*(NearPoints[0].z()+NearPoints[1].z())+1.0/8.0*(FarPoints[0].z()+FarPoints[1].z());
			HI->SetNewMidPoint(Point_3(NewX,NewY,NewZ));
			HI->opposite()->SetNewMidPoint(Point_3(NewX,NewY,NewZ));
		}
		HI++;
	} while(HI!=NewMesh.halfedges_end());

	cout<<"no. of vertices before refinement: "<<NewMesh.size_of_vertices()<<endl;

	//change the topology
	vector<Vertex_handle> vecNewEdgeVertex,vecOriVertex;
	GeometryAlgorithm::DivideFacets(NewMesh,fhRefineTri,vecNewEdgeVertex,vecOriVertex);

	cout<<"no. of vertices after refinement: "<<NewMesh.size_of_vertices()<<endl;

	//update positions
	LocalRefineUpdateVertexPos(vecNewEdgeVertex,vecOriVertex);


	//since more vertices are added, the roi and static vertices need to be adjusted
	ResetRoiStaticVer(NewMesh,ROIVertices,vecAnchorVertices);

}

void CMeshDeformation::SetVerMark(KW_Mesh& NewMesh,vector<Vertex_handle>& vecHandleNb,vector<Vertex_handle>& ROIVertices,vector<Vertex_handle>& vecAnchorVertices)
{
	//mark all vertices: 0: irrelevant vertices; 1: handle; 2: roi; 3:static; 4: new roi; 5: new static
	for (Vertex_iterator VerIter=NewMesh.vertices_begin();VerIter!=NewMesh.vertices_end();VerIter++)
	{
		VerIter->SetReserved(0);
	}
	for (unsigned int i=0;i<vecHandleNb.size();i++)
	{
		vecHandleNb.at(i)->SetReserved(1);
	}
	for (unsigned int i=0;i<ROIVertices.size();i++)
	{
		ROIVertices.at(i)->SetReserved(2);
	}
	for (unsigned int i=0;i<vecAnchorVertices.size();i++)
	{
		vecAnchorVertices.at(i)->SetReserved(3);
	}
}

void CMeshDeformation::ResetRoiStaticVer(KW_Mesh& NewMesh,vector<Vertex_handle>& ROIVertices,vector<Vertex_handle>& vecAnchorVertices)
{
	//0: irrelevant vertices; 1: handle; 2: roi; 3:static; 4: new roi; 5: new static
	for (Vertex_iterator VerIter=NewMesh.vertices_begin();VerIter!=NewMesh.vertices_end();VerIter++)
	{
		if (VerIter->GetReserved()==0 || VerIter->GetReserved()==1 || VerIter->GetReserved()==2 || VerIter->GetReserved()==3
			|| VerIter->GetReserved()==4 || VerIter->GetReserved()==5)
		{
			continue;
		}
		else//newly added vertex
		{
			//if adjacent to 2 old static vertices, or 1 old static vertex and 1 irrelevant vertex,treated as new static vertices
			//otherwise, treated as roi vertices
			int iMarkStat=0;
			Halfedge_around_vertex_circulator Havc=VerIter->vertex_begin();
			do 
			{
				if (Havc->opposite()->vertex()->GetReserved()==0 || Havc->opposite()->vertex()->GetReserved()==3)
				{
					iMarkStat++;
				}
				if (iMarkStat==2)
				{
					break;
				}
				Havc++;
			} while(Havc!=VerIter->vertex_begin());

			if (iMarkStat==2)
			{
				VerIter->SetReserved(5);
				vecAnchorVertices.push_back(VerIter);
			}
			else
			{
				VerIter->SetReserved(4);
				ROIVertices.push_back(VerIter);
			}
		}
	}
	//reset all the reserved mark
	for (Vertex_iterator VerIter=NewMesh.vertices_begin();VerIter!=NewMesh.vertices_end();VerIter++)
	{
		VerIter->SetReserved(0);
	}
}

int CMeshDeformation::GetRefineTri(KW_Mesh OldMesh,KW_Mesh& NewMesh,
									std::vector<Point_3> OldHandlePos,
									std::vector<Point_3> NewHandlePos,
									double dSquaredDistanceThreshold,
									std::vector<Point_3>& testCentroidPoint,
									std::vector<Point_3>& testmovedCentroidPoint,
									std::vector<Facet_handle>& fhRefineTri)
{
	std::vector<Facet_handle> OldTri;
	std::vector<Facet_handle> NewTri;
	for ( Facet_iterator i=OldMesh.facets_begin(),j=NewMesh.facets_begin();
		i!=OldMesh.facets_end(),j!=NewMesh.facets_end(); i++,j++)
	{
		Halfedge_around_facet_circulator k = i->facet_begin();
		vector<Point_3> OldTriVertex;
		do 
		{
			OldTriVertex.push_back(k->vertex()->point());
		} while(++k != i->facet_begin());
		Point_3 OldCentroidPoint=CGAL::centroid(OldTriVertex.begin(),OldTriVertex.end());
		Triangle_3 OldTriangle(OldTriVertex.at(0),OldTriVertex.at(1),OldTriVertex.at(2));

		Halfedge_around_facet_circulator k1 = j->facet_begin();
		vector<Point_3> NewTriVertex;
		do 
		{
			NewTriVertex.push_back(k1->vertex()->point());
		} while(++k1 != j->facet_begin());
		Point_3 NewCentroidPoint=CGAL::centroid(NewTriVertex.begin(),NewTriVertex.end());
		Triangle_3 NewTriangle(NewTriVertex.at(0),NewTriVertex.at(1),NewTriVertex.at(2));

		if (OldCentroidPoint!=NewCentroidPoint)
		{
			//			if (CGAL::squared_distance(OldCentroidPoint,NewCentroidPoint)>0.2)
			if (NewTriangle.squared_area()>=1.2*OldTriangle.squared_area())
			{
				OldTri.push_back(i);
				NewTri.push_back(j);
				//kw new
				fhRefineTri.push_back(j);
			} 
		}
	}
	//kw new
	return fhRefineTri.size();


	//for (unsigned int i=0;i<OldTri.size();i++)
	//{
	//	Halfedge_around_facet_circulator k = OldTri.at(i)->facet_begin();
	//	std::list<Point_3> OldTriVertex;
	//	do 
	//	{
	//		OldTriVertex.push_back(k->vertex()->point());
	//	} while(++k != OldTri.at(i)->facet_begin());
	//	Point_3 OldCentroidPoint=CGAL::centroid(OldTriVertex.begin(),OldTriVertex.end());

	//	double dSumweight=0;
	//	Vector_3 SumVectorByWeight(CGAL::NULL_VECTOR);
	//	double dMaxweight=0;
	//	for (unsigned int j=0;j<OldHandlePos.size();j++)
	//	{
	//		Sphere_3 sphere(OldHandlePos.at(j),dSquaredDistanceThreshold);
	//		if (sphere.has_on_unbounded_side(OldCentroidPoint))
	//		{
	//			continue;
	//		}
	//		double dCurrentSquaredDistance=CGAL::squared_distance(OldHandlePos.at(j),NewHandlePos.at(j));
	//		double dCurrentWeight=pow((1-dCurrentSquaredDistance/dSquaredDistanceThreshold),3);
	//		SumVectorByWeight=(NewHandlePos.at(j)-OldHandlePos.at(j))*dCurrentWeight+SumVectorByWeight;
	//		dSumweight+=dCurrentWeight;
	//		if (dCurrentWeight>=dMaxweight)
	//		{
	//			dMaxweight=dCurrentWeight;
	//		}
	//	}
	//	if (dMaxweight==0)
	//	{
	//		continue;
	//	}
	//	SumVectorByWeight=SumVectorByWeight/dSumweight*dMaxweight;

	//	Point_3 MovedOldCentroidPoint=OldCentroidPoint+SumVectorByWeight;

	//	Halfedge_around_facet_circulator n = NewTri.at(i)->facet_begin();
	//	Point_3 Tripoint[3];
	//	int index=0;
	//	do 
	//	{
	//		Tripoint[index]=(n->vertex()->point());
	//		index++;
	//	} while(++n != NewTri.at(i)->facet_begin());
	//	Triangle_3 FacetTri(Tripoint[0],Tripoint[1],Tripoint[2]);
	//	Point_3 MovedOldCentroidPointProj=FacetTri.supporting_plane().projection(MovedOldCentroidPoint);
	//	//Line_3 line(MovedOldCentroidPoint,MovedOldCentroidPointProj);
	//	//if (!CGAL::do_intersect(FacetTri,line))
	//	if (!FacetTri.has_on(MovedOldCentroidPointProj))
	//	{
	//		testCentroidPoint.push_back(OldCentroidPoint);
	//		testmovedCentroidPoint.push_back(MovedOldCentroidPointProj);
	//		fhRefineTri.push_back(NewTri.at(i));
	//	}
	//}
	//return fhRefineTri.size();
}

void CMeshDeformation::LocalRefineUpdateVertexPos(vector<Vertex_handle> vecNewEdgeVertex,vector<Vertex_handle> vecOriVertex)
{
	//update pos of new edge vertices
	for (unsigned int i=0;i<vecNewEdgeVertex.size();i++)
	{
		vecNewEdgeVertex.at(i)->point()=vecNewEdgeVertex.at(i)->GetLRNewPos();
	}
	//calculate new pos of original vertices
	for (unsigned int i=0;i<vecOriVertex.size();i++)
	{
		////original loop scheme
		//double dDegree=(double)vecOriVertex.at(i)->degree();
		//double dBeta=(5.0/8.0-(3.0/8.0+cos(2*CGAL_PI/dDegree)/4.0)*(3.0/8.0+cos(2*CGAL_PI/dDegree)/4.0))/dDegree;
		//double dCenterWeight=1.0-dDegree*dBeta;
		//double dNewX,dNewY,dNewZ;
		//dNewX=dNewY=dNewZ=0;
		//Halfedge_around_vertex_circulator Havc=vecOriVertex.at(i)->vertex_begin();
		//do 
		//{
		//	dNewX=dNewX+Havc->opposite()->vertex()->point().x()*dBeta;
		//	dNewY=dNewY+Havc->opposite()->vertex()->point().y()*dBeta;
		//	dNewZ=dNewZ+Havc->opposite()->vertex()->point().z()*dBeta;
		//	Havc++;
		//} while(Havc!=vecOriVertex.at(i)->vertex_begin());
		//dNewX=dNewX+dCenterWeight*vecOriVertex.at(i)->point().x();
		//dNewY=dNewY+dCenterWeight*vecOriVertex.at(i)->point().y();
		//dNewZ=dNewZ+dCenterWeight*vecOriVertex.at(i)->point().z();

		//joe warren scheme
		double dDegree=(double)vecOriVertex.at(i)->degree();
		double dBeta=0;
		if (dDegree==3)
		{
			dBeta=3.0/16.0;
		}
		else
		{
			dBeta=3.0/(8.0*dDegree);
		}
		double dCenterWeight=1.0-dDegree*dBeta;
		double dNewX,dNewY,dNewZ;
		dNewX=dNewY=dNewZ=0;
		Halfedge_around_vertex_circulator Havc=vecOriVertex.at(i)->vertex_begin();
		do 
		{
			dNewX=dNewX+Havc->opposite()->vertex()->point().x()*dBeta;
			dNewY=dNewY+Havc->opposite()->vertex()->point().y()*dBeta;
			dNewZ=dNewZ+Havc->opposite()->vertex()->point().z()*dBeta;
			Havc++;
		} while(Havc!=vecOriVertex.at(i)->vertex_begin());
		dNewX=dNewX+dCenterWeight*vecOriVertex.at(i)->point().x();
		dNewY=dNewY+dCenterWeight*vecOriVertex.at(i)->point().y();
		dNewZ=dNewZ+dCenterWeight*vecOriVertex.at(i)->point().z();


		vecOriVertex.at(i)->SetLRNewPos(Point_3(dNewX,dNewY,dNewZ));
	}
	//update new pos of original vertices
	for (unsigned int i=0;i<vecOriVertex.size();i++)
	{
		vecOriVertex.at(i)->point()=vecOriVertex.at(i)->GetLRNewPos();
	}
}
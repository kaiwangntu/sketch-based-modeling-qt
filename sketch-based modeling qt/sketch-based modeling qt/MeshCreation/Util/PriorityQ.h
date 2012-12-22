#ifndef _PRIORITYQ_H
#define _PRIORITYQ_H
#include <iostream>
using namespace std;
#include "./mesh/geometry.h"

class edgeValue
{
public:
	edge* edgep;
	float len;
	edgeValue(){ edgep = NULL;}
	edgeValue(edge* p, float llen){ edgep = p; len = llen;}
	void init( edge* p, float llen){ edgep = p; len = llen;}
	~edgeValue(){ edgep = NULL; }
};
//const int DEFAULTLEN = 1024;
class PriorityQue
{
	int max;
	int n;
	edgeValue *edgevaluelist;
public:
	PriorityQue()
	{
		edgevaluelist = NULL;
		n = 0;
		max = 0;
	}
	PriorityQue( int m )
	{
		edgevaluelist = new edgeValue[ m ];
		n = 0;
		max = m;
	}
	~PriorityQue()
	{
		if( edgevaluelist != NULL)
			delete []edgevaluelist;
	}
	void freeQueue()
	{
		if ( edgevaluelist == NULL)return;
		delete []edgevaluelist;
	}
	void init( int m)
	{
		edgevaluelist = new edgeValue[ m ];
		n = 0;
		max = m;
	}
	bool isEmptyPriorityQue_heap()
	{
		return (n == 0);
	}
	bool add_heap(edgeValue elem)
	{
		if( n >= max)
		{
			cout<<"Overflow!!\n";
			return false;
		}
		if(isEmptyPriorityQue_heap())
		{
			edgevaluelist[0].edgep = elem.edgep;
			edgevaluelist[0].len = elem.len;
			n = 1;
			return true;
		}
		int i;
		for( i= n; i>0 && edgevaluelist[(i-1)/2].len < elem.len; i=(i-1)/2)
		{
			edgevaluelist[ i ].edgep = edgevaluelist[ (i - 1)/2 ].edgep;
			edgevaluelist[ i ].len = edgevaluelist[ (i - 1)/2].len;
		}
//		edgevaluelist[ i ].edgep = edgevaluelist[ (i - 1)/2 ].edgep;
//		edgevaluelist[ i ].len = edgevaluelist[ (i - 1)/2].len;
	/*	if( i > 0)
		cout<<i<<edgevaluelist[ (i - 1)/2].len << elem.len<<endl;
		else
			cout<<i<<edgevaluelist[ 0].len << elem.len<<endl;*/
		edgevaluelist[i].edgep = elem.edgep;
		edgevaluelist[i].len = elem.len;
		n++;
		//for( int j = 0; j < (getCurLen()-1)/2; j++)
		//{
		//	if ( ((getItem(j)->len) < (getItem(j*2)->len)) ||  ((getItem(j)->len) < (getItem(j*2+1)->len)) )
		//	{
		//		if((getItem(j)->len) < (getItem(j*2)->len))
		//			cout<<"first";
		//		else
		//			cout<<"second";
		//		//cout << j << " "<<pq.getItem(j)->len<<" children"<< pq.getItem(j*2)->len<<"  "<< pq.getItem(j*2+1)->len<<endl;
		//		cout << j << getItem(j)->len-getItem(j*2)->len<<"  "<< getItem(j)->len-getItem(j*2+1)->len<<endl;
		//	}
		//}
		return true;
	}
	bool sift(int size,int p)
	{
		edgeValue temp;
		temp.edgep = edgevaluelist[p].edgep;
		temp.len = edgevaluelist[p].len;

		int i=p;
		int child=2*i+1;
		while( child < size)
		{
			if(child<size-1 && edgevaluelist[child].len < edgevaluelist[child+1].len)
				++child;

			if(temp.len < edgevaluelist[child].len )
			{
				edgevaluelist[i].edgep = edgevaluelist[child].edgep;
				edgevaluelist[i].len = edgevaluelist[child].len;
				i=child;
				child=2*i+1;
			}
			else
				break;
		}
		edgevaluelist[i].edgep = temp.edgep;
		edgevaluelist[i].len = temp.len;
		return true;
	}
	bool removeMax_heap(edgeValue& copy)
	{
		if(isEmptyPriorityQue_heap())
		{
			printf("It is empty!!\n");
			return false;
		}
		n--;
		copy.edgep = edgevaluelist[0].edgep;
		copy.len = edgevaluelist[0].len ;
		edgevaluelist[0].edgep = edgevaluelist[n].edgep;
		edgevaluelist[0].len = edgevaluelist[n].len;

		if (sift(n,0))
			return true;
		return false;
	}
	void clearPQ()
	{
	//	if( edgevaluelist != NULL)
	//		delete []edgevaluelist;
	//	edgevaluelist = NULL;
		n = 0;
	}
	void printPQ()
	{
		for( int i = 0; i < n; i ++)
		{
			edgevaluelist[i].edgep->printInfo();
		}
	}
	void printPQ2()
	{
		for( int i = 0; i < n; i ++)
		{
			cout<<edgevaluelist[i].len<<" ";
		}
		cout<<endl<<endl;
	}
	edgeValue* getItem(int i)
	{
		if( i >= n)
		{
			cout<<"Out of bound of the queue! ERROR!"<<endl;
			return NULL;
		}
		return &edgevaluelist[i];
	}
	float getItemLen( int i)
	{
		if( i > n)
		{
			cout<<"Out of bound of the queue! ERROR!"<<endl;
			return NULL;
		}
		return edgevaluelist[i].len;
	}
	int getCurLen(){return n;}
};
#endif
#ifndef _MAPARRAYSR_H_
#define _MAPARRAYSR_H_

#include <vector>
#include <iostream>
using namespace std;
#include "../Util/sort.h"

/**
*	map from keys to value, implemented by array and only for small range
*/
class MapArraySR{
	int* posinval;	//save the position in the value list, index in this should be computed by keys
	vector<int> val;
	int keymax;		//key can be in the range [ 0, keymax -1 ]
	int arraydim;	//array dimension, how many keys for one value
	bool sorted;	//sorted keys to value or not
public:
	/**
	* Function: constructor
	* @param maxkey, keys can only be in the range [ 0, maxkey - 1 ]
	* @param sorted, keys should be sorted or not
	* @param keynum: how many keys determine one value
	*/
	MapArraySR( ){ posinval = NULL; }
	void setParam( int maxkey, bool sorted, int keynum);
	MapArraySR(int maxkey, bool sorted, int keynum);
	~MapArraySR();
	bool getKeyPos( int * keys, int keynum , bool issorted, int& pos);
	bool insertKeyVal( int* keys, int keynum,bool issorted, int value );
	bool replaceKeyVal( int* keys, int keynum, bool issorted, int value );
	bool getKeyVal(int * key, int keynum,bool issorted, int& value );

};

void testMapArraySR();
#endif
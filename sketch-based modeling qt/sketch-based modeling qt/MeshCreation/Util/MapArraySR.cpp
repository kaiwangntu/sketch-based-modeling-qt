#include "../Util/MapArraySR.h"
void MapArraySR::setParam( int maxkey, bool sorted, int keynum)
{
	this->keymax = maxkey;
	this->sorted = sorted;
	this->arraydim = keynum;

	int possize = 1;
	for(int i = 0; i < keynum; i ++)
	{
		possize *= keymax;
	}
	posinval = new int[ possize ];
	for( int i = 0; i < possize; i ++)
		posinval[ i ] = -1;
}
MapArraySR::MapArraySR(int maxkey, bool sorted, int keynum)
{
	this->keymax = maxkey;
	this->sorted = sorted;
	this->arraydim = keynum;

	int possize = 1;
	for(int i = 0; i < keynum; i ++)
	{
		possize *= keymax;
	}
    posinval = new int[ possize ];
	for( int i = 0; i < possize; i ++)
		posinval[ i ] = -1;
}
MapArraySR::~MapArraySR()
{
	if( posinval != NULL )
		delete []posinval;
	val.clear();
}
bool MapArraySR::getKeyPos( int * keys, int keynum , bool issorted,int& pos)
{
	if( keynum != arraydim )
	{
		cout<<"You offered wrong number of keys!"<<endl;
		return false;
	}

	//compute the position
	pos = 0;
	if( sorted && !issorted)
	{
		selectSortInc( keys, keynum );
	}

	int base = 1;
	for( int i = 0; i < arraydim; i ++ )
	{
		if( keys[ i ] >= keymax )
		{
			cout<<"Key value :" <<keys[ i ]<<"is out of range!"<<endl;
			return false;
		}
		pos += base * keys[ i ];
		base *= keymax;
	}
	return true;
}
/**
* insertKeyVal: inserted the pair keys - val, these keys must be haven't set yet!
* @keys the keys
* @keynum number of keys passed in
* @val value for these keys
*/
bool MapArraySR::insertKeyVal( int* keys, int keynum, bool issorted,int value )
{
	int* ikeys = new int[ keynum ];
	memcpy( ikeys, keys, sizeof( int ) * keynum );
	int pos = 0;
	if( !getKeyPos( ikeys, keynum ,issorted, pos))
	{
		cout<<"Unable to compute the position for the keys!"<<endl;
		delete []ikeys;
		return false;
	}
	
	//////////////////////////////////////////////////////////////////////////
	//cout<<"pos:"<<pos<<endl;
	//////////////////////////////////////////////////////////////////////////

	if( posinval[ pos ] != -1 )
	{
		cout<<"These keys have already been set!"<<endl;
		delete []ikeys;
		return false;
	}
	
	val.push_back( value );
	posinval[ pos ] = val.size() - 1;
	delete []ikeys;
	return true;
}
/**
* insertKeyVal: inserted the pair keys - val, value for these keys will be replaced if exists, otherwise, set it!
* @keys the keys
* @keynum number of keys passed in
* @val value for these keys
*/
bool MapArraySR::replaceKeyVal( int* keys, int keynum,bool issorted, int value )
{
	int* ikeys = new int[ keynum ];
	memcpy( ikeys, keys, sizeof( int ) * keynum );
	//compute the position
	int pos = 0;
	if( !getKeyPos( ikeys,  keynum , issorted,pos))
	{
		cout<<"Unable to compute the position for the keys!"<<endl;
		delete []ikeys;
		return false;
	}

	if( posinval[ pos ] != -1 )
	{
		val[ posinval[ pos ] ] = value;
	}
	else
	{
		val.push_back( value );
		posinval[ pos ] = val.size() - 1;
	}	
	delete []ikeys;
	return true;
}
bool MapArraySR::getKeyVal(int * keys, int keynum,bool issorted, int& value )
{
	int* ikeys = new int[ keynum ];
	memcpy( ikeys, keys, sizeof( int ) * keynum );
	//compute the position
	int pos = 0;
	if( !getKeyPos(  ikeys, keynum ,issorted, pos))
	{
		cout<<"Unable to compute the position for the keys!"<<endl;
		delete []ikeys;
		return false;
	}

	value = posinval[ pos ];
	if( value == -1 )	//no corresponding value to these keys
	{
		delete []ikeys;
		return false;
	}
	value = val[ value ];	//the real value to these keys
	delete []ikeys;
	return true;
}


void testMapArraySR()
{
	cout<<"hello , i am in main!"<<endl;
	//MapArraySR(int maxkey, bool sorted, int keynum);
	MapArraySR two2onemap( 3, true, 2);
	int keys[ 2 ];
	for( int i = 0; i < 3; i ++)
	{
		for( int j = i + 1; j < 3; j ++)
		{
			keys[ 0 ] = i;
			keys[ 1 ] = j ;
			two2onemap.insertKeyVal(keys, 2,false, i*10 + j );
		}
	}
	for( int i = 2; i >= 0; i --)
		for( int j = i - 1; j >= 0; j --)
		{
			keys[ 0 ] = i;
			keys[ 1 ] = j;
			int value = 0;
			two2onemap.getKeyVal(keys, 2, false, value);
			cout<<"keys:"<<i<<","<<j<<"value:"<<value<<endl;
		}

}
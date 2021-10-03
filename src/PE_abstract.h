/*
 * PE_abstract.h
 *
 *  Created on: 26 Feb 2018
 *      Author: manupa
 */

#ifndef PE_abstract_H_
#define PE_abstract_H_

#include "Module.h"
#include "RegFile.h"
#include <assert.h>
#include <unordered_set>

namespace CGRAXMLCompile
{

class PE_abstract
{
public:
	PE_abstract(int x, int y, int id)
	{


		assert(y != -1);
		assert(x != -1);
		this->X = x;
		this->Y = y;
		this->id = id;

	};

	std::set<PE_abstract *> neighbors;

	int X;
	int Y;
	int id;

private:


};

} /* namespace CGRAXMLCompile */

#endif /* PE_abstract_H_ */

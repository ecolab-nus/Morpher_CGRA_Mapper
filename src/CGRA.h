/*
 * cgra.h
 *
 *  Created on: 20 Feb 2018
 *      Author: manupa
 */

#ifndef CGRA_H_
#define CGRA_H_


#include "Module.h"
#include "PE.h"

namespace CGRAXMLCompile {

class CGRA : public Module {
public:
	CGRA(const Module* Parent, std::string name, int t, int y, int x, std::string peType = "GENERIC_8REGF", int numberofDPs = 1) : Module(Parent,name){
		createGenericCGRA(x,y,t,peType,numberofDPs);
		this->peType=peType;
		this->numberofDPs=numberofDPs;
	}
	void createGenericCGRA(int x, int y, int t, std::string peType = "GENERIC_8REGF", int numberofDPs = 1);

	std::map<int,std::map<int,std::map<int,PE*>>> PEArr;
	void adjustII(int newII);
	int get_t_max(){return t_max;}
	int get_y_max(){return y_max;}
	int get_x_max(){return x_max;}

//	bool isGenericPE=false;
	std::string peType;
	int  numberofDPs=1;

private:
	int x_max;
	int y_max;
	int t_max;

};

} /* namespace CGRAXMLCompile */

#endif /* CGRA_H_ */

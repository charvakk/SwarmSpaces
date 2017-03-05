/*
 * SwarmSpaces.hpp
 *
 *  Created on: Feb 22, 2017
 *      Author: Charvak Kondapalli
 */

#ifndef SWARMSPACES_SWARMSPACES_H_
#define SWARMSPACES_SWARMSPACES_H_

#include <tr1/unordered_map>
#include <argos3/core/utility/math/vector2.h>
#include <argos3/core/utility/configuration/argos_configuration.h>
#include <swarmspaces/CSwarmTuple.cpp>

using namespace std;

/*
struct VectorHash
    {
        inline size_t operator() (const argos::CVector2& v) const
        {
        	size_t const h1 (hash<argos::Real>{}(v.GetX()) );
        	size_t const h2 (hash<argos::Real>{}(v.GetY()) );
        	return h1 ^ (h2 << 1);
        }
    };
*/

class SwarmSpaces {

public:
	SwarmSpaces();

//	void write(argos::CVector2 pos, string value);
	void write(CSwarmTuple &tuple);

//	string read(argos::CVector2 pos);
	CSwarmTuple read(int const &id);

//	string remove(argos::CVector2 pos);
	CSwarmTuple remove(int const &id);

	vector<int> getIDs();

	size_t size();

	vector<CSwarmTuple> getAllTuples();

private:
//	tr1::unordered_map<argos::CVector2, string, VectorHash> space;
	tr1::unordered_map<int, CSwarmTuple> space;
};



#endif /* SWARMSPACES_SWARMSPACES_H_ */

/*
 * SwarmSpaces.cpp
 *
 *  Created on: Feb 22, 2017
 *      Author: Charvak Kondapalli
 */

#include <swarmspaces/SwarmSpaces.h>
#include <exception>
#include <argos3/core/utility/configuration/argos_configuration.h>
#include <functional>

SwarmSpaces::SwarmSpaces():
	space() {}

/*void SwarmSpaces::write(argos::CVector2 pos, string value){
	space[pos] = value;
}*/

void SwarmSpaces::write(CSwarmTuple &tuple){
//	space[tuple.getId()] = tuple;
	int id = tuple.getId();
	space.insert({id, tuple});
}

/*
string SwarmSpaces::read(argos::CVector2 pos){
	tr1::unordered_map<argos::CVector2, string, VectorHash>::const_iterator got = space.find(pos);
	if(got == space.end()){
		throw 1;
	}else{
		return got->second;
	}
}
*/

CSwarmTuple SwarmSpaces::read(int const &id){
	tr1::unordered_map<int, CSwarmTuple>::const_iterator got = space.find(id);
	if(got == space.end()){
		throw 1;
	}else{
		return got->second;
	}
}


/*string SwarmSpaces::remove(argos::CVector2 pos){
	string temp = space[pos];
	space.erase(pos);
	return temp;
}*/

CSwarmTuple SwarmSpaces::remove(int const &id){
	CSwarmTuple temp = space[id];
	space.erase(id);
	return temp;
}

vector<int> SwarmSpaces::getIDs(){
	vector<int> keys;
	keys.reserve(space.size());

//	for(tr1::unordered_map<int, CSwarmTuple>::const_iterator it = space.begin(); it != space.end(); ++it) {
//	  keys.push_back(it->first);
//	}
	for(auto kv : space){
		keys.push_back(kv.first);
	}
	return keys;
}

size_t SwarmSpaces::size(){
	return space.size();
}

vector<CSwarmTuple> SwarmSpaces::getAllTuples(){
	vector<CSwarmTuple> tuples;
	tuples.reserve(space.size());

	for(auto kv : space){
		tuples.push_back(kv.second);
	}
	return tuples;
}

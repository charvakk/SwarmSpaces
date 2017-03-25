
#ifndef SWARMSPACES_LOOP_FUNCTIONS_H
#define SWARMSPACES_LOOP_FUNCTIONS_H

#include <argos3/core/simulator/loop_functions.h>
#include <swarmspaces/swarmtuple.h>
#include <fstream>
#include <string>

using namespace argos;

class CSwarmSpacesLF : public CLoopFunctions {

public:

	CSwarmSpacesLF();
	virtual ~CSwarmSpacesLF();

	virtual void Init(TConfigurationNode& t_tree);
	virtual void PostStep();
	virtual void Reset();
	virtual void Destroy();
	virtual void PreStep();
	virtual bool IsExperimentFinished();

private:
//	enum TuplePosition{ center = "center", corner = "corner" };

	void PlaceWalls(UInt32 un_robots,
	                   UInt32 un_data_size,
	                   Real f_distance);

	void PlaceUniformly(UInt32 un_robots,
	                       UInt32 un_data_size,
	                       CRange<Real> c_area_range);
	void OpenFile(std::ofstream& c_stream,
	                 const std::string& str_prefix);
	void CloseFile(std::ofstream& c_stream);

	void SetCornerPosition(CVector2);

private:

	std::string m_strOutFile;
	bool m_bDone;
	bool isCheckValid;
	size_t tupleCount;
	std::ofstream m_cOutFile;
	CSwarmTuple tuple;
	bool isSpawned;
	CVector2 cornerPosition;

};

#endif


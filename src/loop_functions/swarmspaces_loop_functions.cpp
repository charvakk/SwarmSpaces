#include "swarmspaces_loop_functions.h"
#include "controllers/ck_footbot_diffusion.h"
#include "swarmspaces/SwarmSpaces.h"

#include <argos3/plugins/simulator/entities/box_entity.h>
#include <argos3/plugins/robots/foot-bot/simulator/footbot_entity.h>

#include <sstream>
#include <list>
#include <errno.h>

/****************************************/
/****************************************/

static const std::string FILE_PREFIX      = "data/SS_";
//static const Real        FB_RADIUS        = 0.085036758f;
//static const Real        FB_AREA          = ARGOS_PI * Square(0.085036758f);
static const std::string FB_CONTROLLER    = "fdc";
static const UInt32      MAX_PLACE_TRIALS = 20;
//static const UInt32      MAX_ROBOT_TRIALS = 20;
static const Real        RAB_RANGE        = 0.7f;
//static const Real        SF_RANGE         = RAB_RANGE / Sqrt(2);
//static const Real        HALF_SF_RANGE    = SF_RANGE * 0.5f;
static const Real        WALL_THICKNESS   = 0.1;
static const Real        WALL_HEIGHT      = 0.5;


/****************************************/
/****************************************/

CSwarmSpacesLF::CSwarmSpacesLF() :
   m_bDone(false),
   isCheckValid(false),
   tuple(1, CVector2(0.0, 0.0), 0.5, "First Tuple!"),
   tupleCount(0),
   isSpawned(false),
   cornerPosition(){}

/****************************************/
/****************************************/

CSwarmSpacesLF::~CSwarmSpacesLF() {
}

/**
    * Varies the size of Arena based on density using PlaceWalls()
    * Distributes robots using PlaceUniformly()
    * Spawns the tuple in the environment
    */
void CSwarmSpacesLF::Init(TConfigurationNode& t_tree) {
   LOG.Flush();
   LOGERR.Flush();
   try {
      /* Parse the configuration file */
      GetNodeAttribute(t_tree, "outfile", m_strOutFile);
      UInt32 unRobots;
      GetNodeAttribute(t_tree, "robots", unRobots);
      UInt32 unDataSize;
      GetNodeAttribute(t_tree, "data_size", unDataSize);
      Real fDensity;
      GetNodeAttribute(t_tree, "density", fDensity);

      bool bWalls;
      GetNodeAttribute(t_tree, "walls", bWalls);
      if(bWalls) {
    	  PlaceWalls(unRobots, unDataSize, fDensity);
      }
      std::string position;
      GetNodeAttribute(t_tree, "tuple_position", position);

      float k_Rp;
      GetNodeAttribute(t_tree, "k_Rp", k_Rp);

      if(position == "center")
    	  tuple.setVfPosition(CVector2(0.0, 0.0));
      else if(position == "corner")
    	  tuple.setVfPosition(cornerPosition);
      else
    	  throw CARGoSException("Invalid tuple_position entered: try 'center' or 'corner'.");
      /* Initialize the rest */
      Reset();

      /* Setting Rp for all controllers. */
      CSpace::TMapPerType& m_cFootbots = GetSpace().GetEntitiesByType("foot-bot");
      for(CSpace::TMapPerType::iterator it = m_cFootbots.begin();
	           it != m_cFootbots.end();
	           ++it) {
	            /* Get handle to foot-bot entity and controller */
	            CFootBotEntity& cFootBot = *any_cast<CFootBotEntity*>(it->second);
	            CFootBotDiffusion& cController = dynamic_cast<CFootBotDiffusion&>(cFootBot.GetControllableEntity().GetController());
				cController.Rp = k_Rp;
		}
      m_cOutFile << unRobots << "\t" << fDensity << "\t" << position << "\t" << k_Rp << "\t";
   }
   catch(CARGoSException& ex) {
      THROW_ARGOSEXCEPTION_NESTED("Error initializing the loop functions", ex);
   }
}

/****************************************/
/****************************************/

void CSwarmSpacesLF::PreStep() {
	// Iterates through all the robots and the first one in range of the SwarmTuple spawns it
	if(!isSpawned){
		CSpace::TMapPerType& m_cFootbots = GetSpace().GetEntitiesByType("foot-bot");

		for(CSpace::TMapPerType::iterator it = m_cFootbots.begin();
				it != m_cFootbots.end();
				++it) {
			/* Get handle to foot-bot entity and controller */
			CFootBotEntity& cFootBot = *any_cast<CFootBotEntity*>(it->second);
			CFootBotDiffusion& cController = dynamic_cast<CFootBotDiffusion&>(cFootBot.GetControllableEntity().GetController());
			if(cController.InTupleRange(tuple)){
				cController.GetVisibleSpace().write(tuple);
				LOG << "Tuple spawned" << tuple.getSInfo() << std::endl;
				isSpawned = true;
				break;
			}
		}
	}
}


/**
    * Writes the current time and the number of
    * tuples alive in the swarm to the output file
    *
    * isCheckValid becomes true after the number of
    * live tuples in the swarm becomes at least 1
    */
void CSwarmSpacesLF::PostStep() {
	CSpace::TMapPerType& m_cFootbots = GetSpace().GetEntitiesByType("foot-bot");
	tupleCount = 0;
	size_t visibleCount = 0;
	size_t invisibleCount = 0;
	for(CSpace::TMapPerType::iterator it = m_cFootbots.begin();
	           it != m_cFootbots.end();
	           ++it) {
	            /* Get handle to foot-bot entity and controller */
	            CFootBotEntity& cFootBot = *any_cast<CFootBotEntity*>(it->second);
	            CFootBotDiffusion& cController = dynamic_cast<CFootBotDiffusion&>(cFootBot.GetControllableEntity().GetController());
	            size_t numVisible = cController.GetVisibleSpace().size();
	            size_t numInvisible = cController.GetInvisibleSpace().size();
	            size_t aliveCount = numVisible + numInvisible;
	            tupleCount += aliveCount;
	            visibleCount += numVisible;
	            invisibleCount += numInvisible;
    }
	if(!isCheckValid && tupleCount > 0){
		isCheckValid = true;
	}
//	m_cOutFile << GetSpace().GetSimulationClock() << "\t"
//	          << tupleCount << "\t" << visibleCount<< "\t" << invisibleCount << std::endl;
}

/****************************************/
/****************************************/

void CSwarmSpacesLF::Reset() {
   OpenFile(m_cOutFile, FILE_PREFIX);
}

/****************************************/
/****************************************/

void CSwarmSpacesLF::Destroy() {
	m_cOutFile << GetSpace().GetSimulationClock() << std::endl;
	CloseFile(m_cOutFile);
}

/**
    * The experiment id finished if the number of tuples alive in
    * the swarm goes to zero
    *
    * The zero at the start of the process is accounted for using isCheckValid
    */

bool CSwarmSpacesLF::IsExperimentFinished() {
	if((isCheckValid == true && tupleCount == 0) || GetSpace().GetSimulationClock() >= 50000){
		m_bDone = true;
	}
	return m_bDone;
}

/****************************************/
/****************************************/

void CSwarmSpacesLF::PlaceWalls(UInt32 un_robots,
                        UInt32 un_data_size,
                        Real f_density) {
   /* Calculate arena side */
   Real fArenaSide =
      RAB_RANGE *
      Sqrt((25.0 * ARGOS_PI * un_robots) /
           ((100.0 - 4.0 * ARGOS_PI) * f_density));
   Real fArenaSide2 = fArenaSide / 2.0;
   Real fArenaSide5 = fArenaSide / 5.0;
   Real fArenaSide10 = fArenaSide / 10.0;
   /* Place the north wall */
   AddEntity(
      *new CBoxEntity("wall_north",
                      CVector3(fArenaSide2, 0, 0),
                      CQuaternion(),
                      false,
                      CVector3(WALL_THICKNESS, fArenaSide, WALL_HEIGHT)));
   /* Place the south wall */
   AddEntity(
      *new CBoxEntity("wall_south",
                      CVector3(-fArenaSide2, 0, 0),
                      CQuaternion(),
                      false,
                      CVector3(WALL_THICKNESS, fArenaSide, WALL_HEIGHT)));
   /* Place the west wall */
   AddEntity(
      *new CBoxEntity("wall_west",
                      CVector3(0, fArenaSide2, 0),
                      CQuaternion(),
                      false,
                      CVector3(fArenaSide, WALL_THICKNESS, WALL_HEIGHT)));
   /* Place the east wall */
   AddEntity(
      *new CBoxEntity("wall_east",
                      CVector3(0, -fArenaSide2, 0),
                      CQuaternion(),
                      false,
                      CVector3(fArenaSide, WALL_THICKNESS, WALL_HEIGHT)));

   /* Calculate side of the region in which the robots are scattered */
   CRange<Real> cAreaRange(-fArenaSide2, fArenaSide2);
   /* Place robots */
   PlaceUniformly(un_robots, un_data_size, cAreaRange);
   SetCornerPosition(CVector2(fArenaSide2, -fArenaSide2));
}

/****************************************/
/****************************************/

void CSwarmSpacesLF::PlaceUniformly(UInt32 un_robots,
                            UInt32 un_data_size,
                            CRange<Real> c_area_range) {
   UInt32 unTrials;
   CFootBotEntity* pcFB;
   std::ostringstream cFBId;
   CVector3 cFBPos;
   CQuaternion cFBRot;
   /* Create a RNG (it is automatically disposed of by ARGoS) */
   CRandom::CRNG* pcRNG = CRandom::CreateRNG("argos");
   /* For each robot */
   for(size_t i = 0; i < un_robots; ++i) {
      /* Make the id */
      cFBId.str("");
      cFBId << "fb" << i;
      /* Create the robot in the origin and add it to ARGoS space */
      pcFB = new CFootBotEntity(
         cFBId.str(),
         FB_CONTROLLER,
         CVector3(),
         CQuaternion(),
         RAB_RANGE,
         un_data_size);
      AddEntity(*pcFB);
      /* Try to place it in the arena */
      unTrials = 0;
      bool bDone;
      do {
         /* Choose a random position */
         ++unTrials;
         cFBPos.Set(pcRNG->Uniform(c_area_range),
                    pcRNG->Uniform(c_area_range),
                    0.0f);
         cFBRot.FromAngleAxis(pcRNG->Uniform(CRadians::UNSIGNED_RANGE),
                              CVector3::Z);
         bDone = MoveEntity(pcFB->GetEmbodiedEntity(), cFBPos, cFBRot);
      } while(!bDone && unTrials <= MAX_PLACE_TRIALS);
      if(!bDone) {
         THROW_ARGOSEXCEPTION("Can't place " << cFBId.str());
      }
   }
}

/****************************************/
/****************************************/

void CSwarmSpacesLF::OpenFile(std::ofstream& c_stream,
                      const std::string& str_prefix) {
   /* Make filename */
   std::string strFName = str_prefix + m_strOutFile;
   /* Close file and reopen it */
   CloseFile(c_stream);
//   c_stream.open(strFName.c_str(),
//                 std::ofstream::out | std::ofstream::trunc);
   c_stream.open(strFName.c_str(),
                    std::ofstream::out | std::ofstream::app);
   if(c_stream.fail())
      THROW_ARGOSEXCEPTION("Error opening \"" << strFName << "\": " << strerror(errno));
}

/****************************************/
/****************************************/

void CSwarmSpacesLF::CloseFile(std::ofstream& c_stream) {
   if(c_stream.is_open()) c_stream.close();
}

void CSwarmSpacesLF::SetCornerPosition(CVector2 position){
	cornerPosition = position;
}
/****************************************/
/****************************************/

REGISTER_LOOP_FUNCTIONS(CSwarmSpacesLF, "swarmspaces_loop_functions");

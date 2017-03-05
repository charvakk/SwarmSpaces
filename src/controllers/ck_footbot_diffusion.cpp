/* Include the controller definition */
#include "ck_footbot_diffusion.h"
/* Function definitions for XML parsing */
#include <argos3/core/utility/configuration/argos_configuration.h>
/* 2D vector definition */
#include <argos3/core/utility/math/vector2.h>

#include <argos3/core/utility/logging/argos_log.h>

#include <argos3/plugins/robots/foot-bot/simulator/footbot_entity.h>

#include <math.h>
#include <exception>
#include <algorithm>

#define MSG_SIZE 100
bool firstLoop = true;

/****************************************/
/****************************************/
CSwarmTuple tuple1(1, CVector2(0.0, 0.0), 0.5, "First Tuple!");

CFootBotDiffusion::CFootBotDiffusion() :
m_pcWheels(NULL),
m_pcProximity(NULL),
m_cAlpha(10.0f),
m_fDelta(0.5f),                 //0.5f for gas particle simulation
m_fWheelVelocity(2.5f),
positionSensor(NULL),
ledActuator(NULL),
swarmSpace(),
currentPosition(),
RABA(NULL),
RABS(NULL),
m_cGoStraightAngleRange(-ToRadians(m_cAlpha),
		ToRadians(m_cAlpha)) {}


/****************************************/
/****************************************/

void CFootBotDiffusion::Init(TConfigurationNode& t_node) {
	/*
	 * Get sensor/actuator handles
	 *
	 * The passed string (ex. "differential_steering") corresponds to the
	 * XML tag of the device whose handle we want to have. For a list of
	 * allowed values, type at the command prompt:
	 *
	 * $ argos3 -q actuators
	 *
	 * to have a list of all the possible actuators, or
	 *
	 * $ argos3 -q sensors
	 *
	 * to have a list of all the possible sensors.
	 *
	 * NOTE: ARGoS creates and initializes actuators and sensors
	 * internally, on the basis of the lists provided the configuration
	 * file at the <controllers><footbot_diffusion><actuators> and
	 * <controllers><footbot_diffusion><sensors> sections. If you forgot to
	 * list a device in the XML and then you request it here, an error
	 * occurs.
	 */
	m_pcWheels    = GetActuator<CCI_DifferentialSteeringActuator>("differential_steering");
	m_pcProximity = GetSensor  <CCI_FootBotProximitySensor      >("footbot_proximity"    );
	positionSensor = GetSensor <CCI_PositioningSensor           >("positioning"          );
	ledActuator = GetActuator  <CCI_LEDsActuator                >("leds");
	RABA = GetActuator<CCI_RangeAndBearingActuator>("range_and_bearing");
	RABS = GetSensor<CCI_RangeAndBearingSensor>("range_and_bearing");

	/*
	 * Parse the configuration file
	 *
	 * The user defines this part. Here, the algorithm accepts three
	 * parameters and it's nice to put them in the config file so we don't
	 * have to recompile if we want to try other settings.
	 */
	GetNodeAttributeOrDefault(t_node, "alpha", m_cAlpha, m_cAlpha);
	m_cGoStraightAngleRange.Set(-ToRadians(m_cAlpha), ToRadians(m_cAlpha));
	GetNodeAttributeOrDefault(t_node, "delta", m_fDelta, m_fDelta);
	GetNodeAttributeOrDefault(t_node, "velocity", m_fWheelVelocity, m_fWheelVelocity);
	ledActuator->SetAllColors(CColor::BLUE);
}

/****************************************/
/****************************************/

void CFootBotDiffusion::ControlStep() {

	ObstacleAvoidance();
	const CCI_PositioningSensor::SReading& positionReading = positionSensor->GetReading();

	std::string id = GetId();
	currentPosition.SetX(positionReading.Position.GetX());
	currentPosition.SetY(positionReading.Position.GetY());


	if(firstLoop && id == "fb1" && InRange(tuple1)){
		firstLoop = false;
		try{
			swarmSpace.read(tuple1.getIId());
		}catch(int e){
			swarmSpace.write(tuple1);
			bool sent = sendTuple(tuple1);
			LOG << "first tuple written" << endl;
		}
	}

	// check if you receive anything
	bool received = false;
	try{
		CSwarmTuple tuple = receiveTuple();
		received = true;
		swarmSpace.write(tuple);
	}catch (int &a){received = false;}

	
	/*// if you receive a tuple
	if(received){
		LOG << "got one" << endl;
		// if you're in range, add the tuple to SS and send to neighbors
		if(InRange(tuple)){
			swarmSpace.write(tuple);
			bool sent = sendTuple(tuple);
			LOG << "Saving it " << tuple.getSInfo() << endl;
		}else if(In2Range(tuple)){	// if not in range but in 2R, don't add to SS but send to neighbors
			bool sent = sendTuple(tuple);
			LOG << "passing it on " << tuple.getSInfo() << endl;
		}	// if out of 2R, do nothing
		else{
			RABA->ClearData();
		}
	}*/

	// If you have a tuple, send it to neighbors
	if(swarmSpace.size() == 0){
		RABA->ClearData();
	}else{
		CSwarmTuple whatIhave = swarmSpace.read(swarmSpace.getIDs().at(0));
		bool sent = sendTuple(whatIhave);
		// LOG << "sending what i have" << endl;
	}

	//Check all tuples and delete ones not in range
	vector<CSwarmTuple> tuples = swarmSpace.getAllTuples();
	for(CSwarmTuple t : tuples){
		if(!InRange(t)){
			swarmSpace.remove(t.getIId());
			// LOG << "tuple deleted" << endl;
		}
	}

	// if you have the tuple, LED red
	std::vector<int> ids = swarmSpace.getIDs();
	if(std::find(ids.begin(), ids.end(), 1) != ids.end()){
		ledActuator->SetAllColors(CColor::RED);
	}else{
		ledActuator->SetAllColors(CColor::BLUE);
	}
}

void CFootBotDiffusion::ObstacleAvoidance() {
	/* Get readings from proximity sensor */
	const CCI_FootBotProximitySensor::TReadings& tProxReads = m_pcProximity->GetReadings();
	/* Sum them together */
	CVector2 cAccumulator;
	for(size_t i = 0; i < tProxReads.size(); ++i) {
		cAccumulator += CVector2(tProxReads[i].Value, tProxReads[i].Angle);
	}
	cAccumulator /= tProxReads.size();
	/* If the angle of the vector is small enough and the closest obstacle
	 * is far enough, continue going straight, otherwise curve a little
	 */
	CRadians cAngle = cAccumulator.Angle();
	if(m_cGoStraightAngleRange.WithinMinBoundIncludedMaxBoundIncluded(cAngle) &&
			cAccumulator.Length() < m_fDelta ) {
		/* Go straight */
		m_pcWheels->SetLinearVelocity(m_fWheelVelocity, m_fWheelVelocity);
	}
	else {
		/* Turn, depending on the sign of the angle */
		if(cAngle.GetValue() > 0.0f) {
			m_pcWheels->SetLinearVelocity(m_fWheelVelocity, 0.0f);
		}
		else {
			m_pcWheels->SetLinearVelocity(0.0f, m_fWheelVelocity);
		}
	}
}

Real CFootBotDiffusion::DistanceFrom(Real const x, Real const y) const{
	Real dx = x - currentPosition.GetX();
	Real dy = y - currentPosition.GetY();
	return sqrt(pow(dx,2) + pow(dy,2));
}

bool CFootBotDiffusion::InRange(CSwarmTuple const &tuple) const{
	if(DistanceFrom(tuple.getVfPosition().GetX(), tuple.getVfPosition().GetY()) <= tuple.getFRange()){
		return true;
	}else{
		return false;
	}
}

bool CFootBotDiffusion::In2Range(CSwarmTuple const &tuple) const{
	if(DistanceFrom(tuple.getVfPosition().GetX(), tuple.getVfPosition().GetY()) <= 2*tuple.getFRange()){
		return true;
	}else{
		return false;
	}
}

bool CFootBotDiffusion::sendTuple(CSwarmTuple &tuple){
	CByteArray byteArray;
	byteArray << tuple.getIId() << tuple.getVfPosition().GetX() << tuple.getVfPosition().GetY() << tuple.getFRange() << tuple.getSInfo();

	size_t arraySize = byteArray.Size();
	for(size_t i = 0; i < MSG_SIZE-arraySize; i++) byteArray << static_cast<UInt8>(0);

	try{
		RABA->SetData(byteArray);
		return true;
	}catch(exception &e){
		std::cout << e.what() << std::endl;
		return false;
	}
}

CSwarmTuple CFootBotDiffusion::receiveTuple(){
	const CCI_RangeAndBearingSensor::TReadings& packets = RABS->GetReadings();
	if(packets.size() == 0 || packets[0].Data.Size() == 0 || (packets[0].Data[0] == 0 && packets[0].Data[25] == 0 && packets[0].Data[50] == 0 && packets[0].Data[70] == 0)) throw 0;
	else{
		int id; CVector2 position; Real range, x, y; string info;

		CByteArray byteArray = packets[0].Data;
		byteArray >> id >> x >> y >> range >> info;

		position.SetX(x); position.SetY(y);

		CSwarmTuple tuple(id, position, range, info);

		if(info.empty()) throw 1;
		return tuple;
	}


}

/****************************************/
/****************************************/

/*
 * This statement notifies ARGoS of the existence of the controller.
 * It binds the class passed as first argument to the string passed as
 * second argument.
 * The string is then usable in the configuration file to refer to this
 * controller.
 * When ARGoS reads that string in the configuration file, it knows which
 * controller class to instantiate.
 * See also the configuration files for an example of how this is used.
 */
REGISTER_CONTROLLER(CFootBotDiffusion, "footbot_diffusion_controller")

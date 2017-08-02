#ifndef osg_robot_Recorder
#define osg_robot_Recorder
#include <memory>
#include <list>
#include <osg\vec4>
#include <osg\matrix>
#include <iostream>

class osg_Recorder
{
public:
	void Start_Recording();
	void End_Recording();
	void Start_Playback();
	void End_Playback();
	void AddStep(std::pair<osg::Matrixf, uint8_t>);
	std::pair<osg::Matrixf, uint8_t> GetStep();
	std::pair<osg::Matrixf, uint8_t>  * PeekStep();
	bool StepsRemain();
	bool IsRecording();
	bool IsPlaying();

	//factory method to prevent multiple recorders
	inline static osg_Recorder & GetRecorder()
	{
		if (Local_Ptr == nullptr)
			Local_Ptr.reset(new osg_Recorder);
		
		return *Local_Ptr;
	}
private:
	bool Recording;
	bool Playing;
	inline osg_Recorder() : Recording(false), Playing(false) {};
	static std::unique_ptr<osg_Recorder> Local_Ptr;

	std::list<std::pair<osg::Matrixf, uint8_t>> Data_Points;
};

#endif

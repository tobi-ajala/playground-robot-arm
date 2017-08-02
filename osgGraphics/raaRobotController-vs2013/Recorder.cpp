#include "Recorder.h"

std::unique_ptr<osg_Recorder> osg_Recorder::Local_Ptr;

void osg_Recorder::Start_Recording()
{
	std::cout << "Started Recording\n";
	Recording = true;
	//clear any existing recording
	Data_Points.clear();
}

void osg_Recorder::End_Recording()
{
	std::cout << "Stopped Recording at - " << Data_Points.size() << " Frames \n";
	Recording = false;
}

void osg_Recorder::Start_Playback()
{
	std::cout << "Started Playback\n";
	Playing = true;
}

void osg_Recorder::End_Playback()
{
	std::cout << "Stopped Playback - " << Data_Points.size() << " Frames remaining\n";
	Playing = false;
}

void osg_Recorder::AddStep(std::pair<osg::Matrixf, uint8_t> Value)
{
	Data_Points.push_back(Value);
}

std::pair<osg::Matrixf, uint8_t> osg_Recorder::GetStep()
{
	//prevent exception if called without data
	if (!StepsRemain()) return std::make_pair(osg::Matrixf(), -1);

	//copy last datapoint
	std::pair<osg::Matrixf, uint8_t> RetVal = Data_Points.back();
	Data_Points.pop_back();
	return RetVal;
}

std::pair<osg::Matrixf, uint8_t>* osg_Recorder::PeekStep()
{
	if (!StepsRemain()) return nullptr;
	else return &Data_Points.back();
}

bool osg_Recorder::StepsRemain()
{
	//constant time operation - returns if steps still remain
	return (Data_Points.size() > 0);
}

bool osg_Recorder::IsRecording()
{
	return Recording;
}

bool osg_Recorder::IsPlaying()
{
	return Playing;
}

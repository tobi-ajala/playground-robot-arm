#include "main.h"

//Physical definition of predeclared variables
osg::Node *g_pModel = 0;
osg::ref_ptr<osg::Node> _selectedNode;
osg_Recorder * RecordPlayback = nullptr;

PickHandler::PickHandler() : _mX(0.), _mY(0.) {};
bool PickHandler::handle(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa)
{
	osgViewer::Viewer* viewer =
		dynamic_cast<osgViewer::Viewer*>(&aa);
	if (!viewer)
		return(false);

	switch (ea.getEventType()) {
	case osgGA::GUIEventAdapter::PUSH: //recording mouse location required for key press & moving instances
	case osgGA::GUIEventAdapter::MOVE: {
		_mX = ea.getX();
		_mY = ea.getY();
		return(false);
	}
	case osgGA::GUIEventAdapter::RELEASE: {
		//conditioning the mouse to performa  pick if it hasn't moved since the last button press or move event, otherwise the trackball manipulator will take over
		if (_mX == ea.getX() && _mY == ea.getY()) {
			if (pick(ea.getXnormalized(),
				ea.getYnormalized(), viewer))
				return(true);
		}
		return(false);
	}
	default:
		return(false);
	}
}

bool PickHandler::pick(const double x, const double y, osgViewer::Viewer* viewer)
{
	if (!viewer->getSceneData())
		return(false); //if there's nothing to pick, retun nothing

	double w(.05), h(.05);
	osgUtil::PolytopeIntersector* picker =
		new osgUtil::PolytopeIntersector(
			osgUtil::Intersector::PROJECTION,
			x - w, y - h, x + w, y + h);

	osgUtil::IntersectionVisitor iv(picker);
	viewer->getCamera()->accept(iv);

	if (picker->containsIntersections()) {
		const osg::NodePath& node_Path =
			picker->getFirstIntersection().nodePath;
		unsigned int idx = node_Path.size();
		while (idx--)  //loop to find the last MatrixTransform in the mode path which will be the Matrix Transform to attach the callback to
		{
			osg::MatrixTransform* mt = dynamic_cast<osg::MatrixTransform*>(node_Path[idx]);
			if (mt == NULL)
				continue; //at this point a MatrixTransform would have be found in the nodePath
			if (mt->getName() == "BodyRotator" || mt->getName() == "UpperArmRotator" || mt->getName() == "LowerArmRotator" || mt->getName() == "HandRotator")
			{
				//if condition for if any of the above parts are selected to do the following
				if (child) { //if condition for if there's a child instance
					osg::Node * parent = child->getParent(0)->getParent(0); //to find the grandparent of the child
					parent->asGroup()->addChild(child); //to add the child to the new parent
					parent->asGroup()->removeChild(child->getParent(0)); //to remove the original parent
				}

				std::cout << "highlighting: " << mt->getName() << std::endl;

				osgFX::Outline* outline = new osgFX::Outline();

				outline->setColor(osg::Vec4(0.0f, 0.0f, 1.0f, 1.0f));
				outline->setWidth(6);
				outline->addChild(mt->getChild(1)); //to make the outline the parent of the child (the model)
				child = mt->getChild(1); //to make the child node the child
				mt->replaceChild(mt->getChild(1), outline);

				break;
			}
		}
	}
	return true;
}

//class to encapsulate the data so it is available to both the event handler and update callback
robotInputDeviceStateType::robotInputDeviceStateType() :
	moveRequest(false), direction(0.0), elevation(0.0), uaElevation(0.0) {};

//callback for Robot Body by defining which axis of motion goes with what body part
updateRobotPosCallback::updateRobotPosCallback(robotInputDeviceStateType* robotIDevState, std::string partName)
{
	robotInputDeviceState = robotIDevState;
	bodyPartName = partName;
}
void updateRobotPosCallback::operator()(osg::Node* node, osg::NodeVisitor* nv)
{
	//if currently playing back
	if (RecordPlayback->IsPlaying())
	{
		osg::MatrixTransform * mt = dynamic_cast<osg::MatrixTransform *> (node);
		//ensure steps remain to playback
		if (!RecordPlayback->StepsRemain())
		{
			RecordPlayback->End_Playback();
			return;
		}
		//look at the step ahead and ensure we match
		std::pair<osg::Matrixf, uint8_t> * Step_P = RecordPlayback->PeekStep();
		bool Continue = false;
		if (Step_P->second == 0 && bodyPartName == "BodyRotator")
			Continue = true;
		if (Step_P->second == 1 && bodyPartName == "LowerArmRotator")
			Continue = true;
		if (Step_P->second == 2 && bodyPartName == "UpperArmRotator")
			Continue = true;
		if (Continue == false)
		{
			traverse(node, nv);
			return;
		}

		//get the next step (removing it from the recording list
		std::pair<osg::Matrixf, uint8_t> Step = RecordPlayback->GetStep();
		//set matrix equal to recorded matrix
		mt->setMatrix(Step.first);

		return;
	}

	if (robotInputDeviceState->moveRequest) {
		osg::MatrixTransform * mt = dynamic_cast<osg::MatrixTransform *> (node);

		if (bodyPartName == "BodyRotator") {
			const osg::Vec3d axis(0, 0, 1);
			mt->setMatrix(mt->getMatrix() * osg::Matrix::rotate(robotInputDeviceState->direction, axis));
			//record motion
			if (RecordPlayback->IsRecording())
				RecordPlayback->AddStep(
					std::make_pair(mt->getMatrix(), 0));
		}
		else if (bodyPartName == "LowerArmRotator") {
			const osg::Vec3d axis(0, 1, 0);
			mt->setMatrix(mt->getMatrix() * osg::Matrix::rotate(robotInputDeviceState->elevation, axis));
			if (RecordPlayback->IsRecording())
				RecordPlayback->AddStep(
					std::make_pair(mt->getMatrix(), 1));
		}
		else if (bodyPartName == "UpperArmRotator") {
			const osg::Vec3d axis(0, 1, 0);
			mt->setMatrix(mt->getMatrix() * osg::Matrix::rotate(robotInputDeviceState->uaElevation, axis));
			if (RecordPlayback->IsRecording())
				RecordPlayback->AddStep(
					std::make_pair(mt->getMatrix(), 2));
		}
		traverse(node, nv);
	}
}

//keyboard event handler
myKeyboardEventHandler::myKeyboardEventHandler(robotInputDeviceStateType* tids)

{
	robotInputDeviceState = tids;
}

//key press commands to conrol the arm movement
bool myKeyboardEventHandler::handle(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa)

{
	switch (ea.getEventType())
	{ //key press commands detection
	case osgGA::GUIEventAdapter::KEYDOWN:
	{
		switch (ea.getKey()) {
		case (osgGA::GUIEventAdapter::KEY_Down):
			std::cout << "Upper Arm moving down" << std::endl;
			robotInputDeviceState->uaElevation = 0.005;
			robotInputDeviceState->moveRequest = true;
			return false;
			break;
		case osgGA::GUIEventAdapter::KEY_Up:
			std::cout << "Upper Arm moving up" << std::endl;
			robotInputDeviceState->uaElevation = -0.005;
			robotInputDeviceState->moveRequest = true;
			return false;
			break;
		case osgGA::GUIEventAdapter::KEY_Left:
			std::cout << "Robot rotating Anti-Clockwise" << std::endl;
			robotInputDeviceState->direction = 0.005;
			robotInputDeviceState->moveRequest = true;
			return false;
			break;
		case osgGA::GUIEventAdapter::KEY_Right:
			std::cout << "Robot rotating Clockwise" << std::endl;
			robotInputDeviceState->direction = -0.005;
			robotInputDeviceState->moveRequest = true;
			return false;
			break;
		case osgGA::GUIEventAdapter::KEY_Return:
			std::cout << "Lower Arm extending" << std::endl;
			robotInputDeviceState->elevation = -0.005;
			robotInputDeviceState->moveRequest = true;
			return false;
			break;
		case osgGA::GUIEventAdapter::KEY_Shift_R:
			std::cout << "Lower Arm retracting" << std::endl;
			robotInputDeviceState->elevation = 0.005;
			robotInputDeviceState->moveRequest = true;
			return false;
			break;
		default:
			return false;
		}
	}
	case(osgGA::GUIEventAdapter::KEYUP): {
		switch (ea.getKey()) {
		case osgGA::GUIEventAdapter::KEY_Right:
		case osgGA::GUIEventAdapter::KEY_Left:
			std::cout << "Key released" << std::endl;
			robotInputDeviceState->direction = 0.0;
			robotInputDeviceState->moveRequest = false;
			return false;
			break;
		case osgGA::GUIEventAdapter::KEY_Up:
		case osgGA::GUIEventAdapter::KEY_Down:
			std::cout << "Key released" << std::endl;
			robotInputDeviceState->uaElevation = 0.0;
			robotInputDeviceState->moveRequest = false;
			return false;
			break;
		case osgGA::GUIEventAdapter::KEY_Return:
		case osgGA::GUIEventAdapter::KEY_Shift_R:
			std::cout << "Key released" << std::endl;
			robotInputDeviceState->elevation = 0.0;
			robotInputDeviceState->moveRequest = false;
			return false;
			break;
		case osgGA::GUIEventAdapter::KEY_R:
			//cannot record while playing
			if (RecordPlayback->IsPlaying()) return false;
			//stop or start as required
			if (!RecordPlayback->IsRecording())
				RecordPlayback->Start_Recording();
			else
				RecordPlayback->End_Recording();
			break;
		case osgGA::GUIEventAdapter::KEY_P:
			//cannot Play while recording
			if (RecordPlayback->IsRecording()) return false;
			//stop or start as required
			if (!RecordPlayback->IsPlaying())
				RecordPlayback->Start_Playback();
			else
				RecordPlayback->End_Playback();
			break;
		default:
			return false;
		}
	}
	default:
		return false;
	}
}

//function to run before rendering starts -> use this to identify nodes
void init()
{
	if (g_pModel) {
		std::cout << "Loaded Model" << std::endl;
	}
	else {
		std::cout << " Failed to load a Model" << std::endl;
	}
}


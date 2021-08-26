/* This is my first plugin file. */
/*
 * Copyright (C) 2012-2015 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

#include "gazebo/physics/physics.hh"
#include "gazebo/transport/transport.hh"
#include "DiffDrivePlugin.hh"

using namespace gazebo;
GZ_REGISTER_MODEL_PLUGIN(DiffDrivePlugin)

enum {RIGHT, LEFT};

/////////////////////////////////////////////////
DiffDrivePlugin::DiffDrivePlugin()
{
  this->wheelSpeed[LEFT] = this->wheelSpeed[RIGHT] = 0;
}

/////////////////////////////////////////////////
void DiffDrivePlugin::Load(physics::ModelPtr _model,
                           sdf::ElementPtr _sdf)
{
  this->model = _model;

  this->node = transport::NodePtr(new transport::Node());
#if(GAZEBO_MAJOR_VERSION <= 7)
  this->node->Init(this->model->GetWorld()->GetName());
#endif
#if(GAZEBO_MAJOR_VERSION >= 8)
  this->node->Init(this->model->GetWorld()->Name());
#endif

  // Read joint name of the left joint.
  if (!_sdf->HasElement("left_joint"))
    gzerr << "DiffDrive plugin missing <left_joint> element\n";
  this->leftJoint = _model->GetJoint(
      _sdf->GetElement("left_joint")->Get<std::string>());
  if (!this->leftJoint)
    gzerr << "Unable to find left joint["
          << _sdf->GetElement("left_joint")->Get<std::string>() << "]\n";

  // Read joint name of the right joint.
  if (!_sdf->HasElement("right_joint"))
    gzerr << "DiffDrive plugin missing <right_joint> element\n";
  this->rightJoint = _model->GetJoint(
      _sdf->GetElement("right_joint")->Get<std::string>());
  if (!this->rightJoint)
    gzerr << "Unable to find right joint["
          << _sdf->GetElement("right_joint")->Get<std::string>() << "]\n";

  // Sample : Read joint name of the shoulder joint.
  if (!_sdf->HasElement("shoulderTAG"))
    gzerr << "DiffDrive plugin missing <shoulderTAG> element\n";
  this->shoulderJoint = _model->GetJoint(
      _sdf->GetElement("shoulderTAG")->Get<std::string>());
  if (!this->shoulderJoint)
    gzerr << "Unable to find shoulder joint["
          << _sdf->GetElement("shoulderTAG")->Get<std::string>() << "]\n";

/*
  if (_sdf->HasElement("torque"))
  {
    this->torque = _sdf->GetElement("torque")->Get<double>();
    gzwarn << "The MaxForce API is deprecated in Gazebo, "
           << "and the torque tag is no longer used in this plugin."
           << std::endl;
  }
*/

// About details of keyboard topic , see below URL.
// https://bitbucket.org/osrf/gazebo/pull-requests/2652/added-support-for-tracked-vehicles/diff
// This needs KEYPUBLISHER(SEE: https://github.com/osrf/car_demo/issues/25), 
// Add <plugin name='keyboard' filename='libKeyboardGUIPlugin.so'/> inside <gui></gui> in your world file. 

  this->keySub = this->node->Subscribe(std::string("~/keyboard/keypress"), 
      &DiffDrivePlugin::OnKeyPress, this);

  this->velSub = this->node->Subscribe(std::string("~/") +
      this->model->GetName() + "/vel_cmd", &DiffDrivePlugin::OnVelMsg, this);

  this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          boost::bind(&DiffDrivePlugin::OnUpdate, this));
}

/////////////////////////////////////////////////
void DiffDrivePlugin::Init()
{
  this->wheelSeparation = this->leftJoint->Anchor(0).Distance(
      this->rightJoint->Anchor(0));

  physics::EntityPtr parent = boost::dynamic_pointer_cast<physics::Entity>(
      this->leftJoint->GetChild());

  ignition::math::Box bb = parent->BoundingBox();
  // This assumes that the largest dimension of the wheel is the diameter
#if(GAZEBO_MAJOR_VERSION < 8)
  this->wheelRadius = bb.GetSize().GetMax() * 0.5;
#endif
#if(GAZEBO_MAJOR_VERSION >= 8 && GAZEBO_MAJOR_VERSION < 9)
  this->wheelRadius = bb.GetSize().Ign().Max() * 0.5;
#endif
#if(GAZEBO_MAJOR_VERSION >= 9)
  this->wheelRadius = bb.Size().Max() * 0.5;
#endif
}

/////////////////////////////////////////////////
void DiffDrivePlugin::OnVelMsg(ConstPosePtr &_msg)
{
  double vr, va;

  vr = _msg->position().x();
#if(GAZEBO_MAJOR_VERSION == 5)
  va =  msgs::Convert(_msg->orientation()).GetAsEuler().z;
#endif
#if(GAZEBO_MAJOR_VERSION >= 7)
  va =  msgs::ConvertIgn(_msg->orientation()).Euler().Z();
#endif

  this->wheelSpeed[LEFT] = vr + va * this->wheelSeparation / 2.0;
  this->wheelSpeed[RIGHT] = vr - va * this->wheelSeparation / 2.0;

  double leftVelDesired = (this->wheelSpeed[LEFT] / this->wheelRadius);
  double rightVelDesired = (this->wheelSpeed[RIGHT] / this->wheelRadius);

  this->leftJoint->SetVelocity(0, leftVelDesired);
  this->rightJoint->SetVelocity(0, rightVelDesired);
}

#define _MAX(X,Y)	((X > Y)?X:Y)
#define _MIN(X,Y)	((X < Y)?X:Y)

void	DiffDrivePlugin::OnKeyPress(ConstAnyPtr &_msg)
{
  const auto key = static_cast<const unsigned int>(_msg->int_value());
	static float	left_v = 0, right_v = 0;
  gzmsg << "KEY(" << key << ") pressed\n";
	switch(key)
	{
		case 'q': left_v += 0.1;
                                left_v = _MIN(left_v, 1);
			  break;
		case 'a': left_v = 0;
			  break;
		case 'z': left_v -= 0.1;
                                left_v = _MAX(left_v, -1);
			  break;
		case 'e': right_v += 0.1;
                                right_v = _MIN(right_v, 1);
			  break;
		case 'd': right_v = 0;
			  break;
		case 'c': right_v -= 0.1;
                                right_v = _MAX(right_v, -1);
			  break;
	}
	this->leftJoint->SetVelocity(0, left_v);
	this->rightJoint->SetVelocity(0, right_v);
}

// Before Gazebo7 including Gazebo7, doslike_kbhit and doslike_getch worked correctly.
// But after Gazebo8, they no longer worked.
// Especially, the funcion tcsetattr won't work.
inline int	doslike_kbhit(void)
{
	struct termios	oldt, newt;
	int	ch;
	int	oldf;
	tcgetattr(STDIN_FILENO, &oldt);
	newt = oldt;
	newt.c_lflag &= ~(BRKINT | ISTRIP | IXON);
	newt.c_lflag &= ~(ICANON | ECHO | ECHOE | ECHOK | ECHONL);
	tcsetattr(STDIN_FILENO, TCSANOW, &newt);
	oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
	fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);
	ch = getchar();
	tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
	fcntl(STDIN_FILENO, F_SETFL, oldf);
	if(ch != EOF)
	{
		ungetc(ch, stdin);
		return 1;
	}
	return 0;
}

inline int	doslike_getch(void)
{
	static struct termios	oldt, newt;
	tcgetattr(STDIN_FILENO, &oldt);
	newt = oldt;
	newt.c_lflag &= ~(BRKINT | ISTRIP | IXON);
	newt.c_lflag &= ~(ICANON | ECHO | ECHOE | ECHOK | ECHONL);
	tcsetattr(STDIN_FILENO, TCSANOW, &newt);
	int c = getchar();
	tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
	return c;
}

void	DiffDrivePlugin::check_key_command(void)
{
	static float	left_v = 0, right_v = 0;
	if(doslike_kbhit())
	{
		int cmd = doslike_getch();
		switch(cmd)
		{
			case 'q': left_v += 0.1;
                                  left_v = _MIN(left_v, 1);
				  break;
			case 'a': left_v = 0;
				  break;
			case 'z': left_v -= 0.1;
                                  left_v = _MAX(left_v, -1);
				  break;
			case 'e': right_v += 0.1;
                                  right_v = _MIN(right_v, 1);
				  break;
			case 'd': right_v = 0;
				  break;
			case 'c': right_v -= 0.1;
                                  right_v = _MAX(right_v, -1);
				  break;
		}
		this->leftJoint->SetVelocity(0, left_v);
		this->rightJoint->SetVelocity(0, right_v);
	}
}

/////////////////////////////////////////////////
void DiffDrivePlugin::OnUpdate()
{
  /* double d1, d2;
  double dr, da;

  this->prevUpdateTime = currTime;

  // Distance travelled by front wheels
  d1 = stepTime.Double() * this->wheelRadius * this->leftJoint->GetVelocity(0);
  d2 = stepTime.Double() * this->wheelRadius * this->rightJoint->GetVelocity(0);

  dr = (d1 + d2) / 2;
  da = (d1 - d2) / this->wheelSeparation;
  common::Time currTime = this->model->GetWorld()->GetSimTime();
  common::Time stepTime = currTime - this->prevUpdateTime;
  */

//  check_key_command();
}

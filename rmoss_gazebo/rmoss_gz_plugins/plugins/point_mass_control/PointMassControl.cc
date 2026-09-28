// Copyright 2025 RoboMaster-OSS
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <mutex>

#include <gz/common/Util.hh>
#include <gz/plugin/Register.hh>
#include <gz/transport/Node.hh>

#include <gz/sim/components/AngularVelocity.hh>
#include <gz/sim/components/LinearVelocity.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Util.hh>

#include <gz/math/Pose3.hh>
#include <gz/math/Vector3.hh>
#include <gz/msgs/twist.pb.h>
#include <gz/msgs/Utility.hh>

#include "PointMassControl.hh"

using namespace gz;
using namespace sim;
using namespace systems;

class gz::sim::systems::PointMassControlPrivate
{
public:
    void OnCmdVel(const gz::msgs::Twist &_msg);

public:
    transport::Node node;
    Model model{kNullEntity};
    std::string linkName;
    Entity link{kNullEntity};

    std::mutex targetVelMutex;
    msgs::Twist targetVel;
};

PointMassControl::PointMassControl()
    : dataPtr(std::make_unique<PointMassControlPrivate>())
{
}

void PointMassControl::Configure(const Entity &_entity,
                                 const std::shared_ptr<const sdf::Element> &_sdf,
                                 EntityComponentManager &_ecm,
                                 EventManager & /*_eventMgr*/)
{
    this->dataPtr->model = Model(_entity);
    if (!this->dataPtr->model.Valid(_ecm))
    {
        gzerr << "PointMassControl should be attached to a model entity. Failed to initialize." << std::endl;
        return;
    }

    this->dataPtr->linkName = _sdf->Get<std::string>("link_name", "baselink").first;
    this->dataPtr->link = this->dataPtr->model.LinkByName(_ecm, this->dataPtr->linkName);
    if (this->dataPtr->link == kNullEntity)
    {
        gzerr << "PointMassControl: link[" << this->dataPtr->linkName << "] not found." << std::endl;
        return;
    }

    std::string defaultTopic{"/model/" + this->dataPtr->model.Name(_ecm) + "/cmd_vel"};
    std::string topic = _sdf->Get<std::string>("topic", defaultTopic).first;
    this->dataPtr->node.Subscribe(topic, &PointMassControlPrivate::OnCmdVel, this->dataPtr.get());
    gzmsg << "PointMassControl controlling link[" << this->dataPtr->linkName
          << "] subscribing to [" << topic << "]" << std::endl;
}

void PointMassControl::PreUpdate(const gz::sim::UpdateInfo &_info,
                                 gz::sim::EntityComponentManager &_ecm)
{
    if (_info.paused)
    {
        return;
    }

    Link link(this->dataPtr->link);
    if (!_ecm.Component<components::WorldPose>(this->dataPtr->link))
    {
        _ecm.CreateComponent(this->dataPtr->link, components::WorldPose());
    }
    if (!_ecm.Component<components::LinearVelocity>(this->dataPtr->link))
    {
        _ecm.CreateComponent(this->dataPtr->link, components::LinearVelocity());
    }
    if (!_ecm.Component<components::AngularVelocity>(this->dataPtr->link))
    {
        _ecm.CreateComponent(this->dataPtr->link, components::AngularVelocity());
    }

    // 目标速度（body 系）
    msgs::Twist target;
    {
        std::lock_guard<std::mutex> lock(this->dataPtr->targetVelMutex);
        target = this->dataPtr->targetVel;
    }

    // 注意：gz 的 LinearVelocity/AngularVelocity 组件是“Link 坐标系下”的速度
    // （见 Link::SetLinearVelocity 文档：velocity to set in Link's Frame），
    // 所以 body 系指令直接写入即可，不要再自己旋到世界系（否则车身转过角度后方向会被多转一次）。
    const auto curLinVel = _ecm.Component<components::LinearVelocity>(this->dataPtr->link)->Data();
    const auto curAngVel = _ecm.Component<components::AngularVelocity>(this->dataPtr->link)->Data();

    // 水平速度直接用指令；z 分量保留物理算出来的值（重力/碰撞），这样能爬坡/落地/下降
    math::Vector3d newLinVel(target.linear().x(), target.linear().y(), curLinVel.Z());
    // 偏航角速度用指令；roll/pitch 保留物理值（过坡道时自然俯仰）
    math::Vector3d newAngVel(curAngVel.X(), curAngVel.Y(), target.angular().z());

    link.SetLinearVelocity(_ecm, newLinVel);
    link.SetAngularVelocity(_ecm, newAngVel);
}

void PointMassControlPrivate::OnCmdVel(const gz::msgs::Twist &_msg)
{
    std::lock_guard<std::mutex> lock(this->targetVelMutex);
    this->targetVel = _msg;
}

/******************register*************************************************/
GZ_ADD_PLUGIN(PointMassControl,
              gz::sim::System,
              PointMassControl::ISystemConfigure,
              PointMassControl::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(PointMassControl, "gz::sim::systems::PointMassControl")

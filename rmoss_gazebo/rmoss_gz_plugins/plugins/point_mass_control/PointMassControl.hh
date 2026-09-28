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


#ifndef GZ_SIM_SYSTEMS_POINT_MASS_CONTROL_HH
#define GZ_SIM_SYSTEMS_POINT_MASS_CONTROL_HH

#include <memory>
#include <gz/sim/System.hh>

namespace gz
{
    namespace sim
    {
        namespace systems
        {
            class PointMassControlPrivate;
            /// \brief 质点式平面运动控制：
            ///  - 水平方向（body 系 x/y）与偏航角速度直接赋值给指定 link 的速度，
            ///    不受摩擦、轮子/悬挂等运动模型影响；
            ///  - 垂直方向不动速度，交给物理（重力 + 碰撞），所以会正常爬坡、落地、下落；
            ///  - 模型自身碰撞保持有效。
            class GZ_SIM_VISIBLE PointMassControl
                : public gz::sim::System,
                  public ISystemConfigure,
                  public ISystemPreUpdate
            {
            public:
                PointMassControl();
                ~PointMassControl() override = default;

            public:
                void Configure(const Entity &_entity,
                               const std::shared_ptr<const sdf::Element> &_sdf,
                               EntityComponentManager &_ecm,
                               EventManager &_eventMgr) override;
                void PreUpdate(const gz::sim::UpdateInfo &_info,
                               gz::sim::EntityComponentManager &_ecm) override;

            private:
                std::unique_ptr<PointMassControlPrivate> dataPtr;
            };
        } // namespace systems
    }     // namespace sim
} // namespace gz

#endif  // GZ_SIM_SYSTEMS_POINT_MASS_CONTROL_HH

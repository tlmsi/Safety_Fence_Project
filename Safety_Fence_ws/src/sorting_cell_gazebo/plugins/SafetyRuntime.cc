#include <chrono>
#include <mutex>
#include <memory>
#include <optional>
#include <string>

#include <gz/math/Pose3.hh>

#include <gz/msgs/boolean.pb.h>
#include <gz/msgs/light.pb.h>
#include <gz/msgs/stringmsg.pb.h>
#include <gz/msgs/visual.pb.h>

#include <gz/plugin/Register.hh>

#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>

#include <gz/sim/components/Light.hh>
#include <gz/sim/components/LightCmd.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/Visual.hh>
#include <gz/sim/components/VisualCmd.hh>

#include <gz/transport/Node.hh>


namespace sorting_cell
{

class SafetyRuntime :
  public gz::sim::System,
  public gz::sim::ISystemConfigure,
  public gz::sim::ISystemPreUpdate
{
public:
  void Configure(
    const gz::sim::Entity &,
    const std::shared_ptr<const sdf::Element> &,
    gz::sim::EntityComponentManager &,
    gz::sim::EventManager &) override
  {
    this->node.Subscribe(
      "/safety/state",
      &SafetyRuntime::OnSafetyState,
      this);

    this->node.Subscribe(
      "/safety/gate_visual_open",
      &SafetyRuntime::OnGateState,
      this);
  }


public:
  void PreUpdate(
    const gz::sim::UpdateInfo &_info,
    gz::sim::EntityComponentManager &_ecm) override
  {
    bool stateChanged = false;
    bool gateChanged = false;

    {
      std::lock_guard<std::mutex> lock(
        this->mutex);

      if (this->pendingState)
      {
        this->state = *this->pendingState;
        this->pendingState.reset();
        stateChanged = true;
      }

      if (this->pendingGate)
      {
        this->gateOpen = *this->pendingGate;
        this->pendingGate.reset();
        gateChanged = true;
      }
    }

    this->ResolveEntities(_ecm);

    if (!this->initialized)
    {
      this->ApplyGate(_ecm);
      this->ApplyLights(_ecm, true);

      this->initialized = true;
    }

    if (gateChanged)
    {
      this->ApplyGate(_ecm);
    }

    if (stateChanged)
    {
      this->ApplyLights(_ecm, true);
    }

    if (this->state == "PROTECTIVE_STOP")
    {
      const auto milliseconds =
        std::chrono::duration_cast<
          std::chrono::milliseconds>(
            _info.simTime).count();

      const bool phase =
        ((milliseconds / 500) % 2) == 0;

      if (phase != this->lastBlinkPhase)
      {
        this->lastBlinkPhase = phase;
        this->ApplyLights(_ecm, false);
      }
    }
  }


private:
  void OnSafetyState(
    const gz::msgs::StringMsg &_message)
  {
    std::lock_guard<std::mutex> lock(
      this->mutex);

    this->pendingState =
      _message.data();
  }


private:
  void OnGateState(
    const gz::msgs::Boolean &_message)
  {
    std::lock_guard<std::mutex> lock(
      this->mutex);

    this->pendingGate =
      _message.data();
  }


private:
  void ResolveEntities(
    const gz::sim::EntityComponentManager &_ecm)
  {
    // --------------------------------------------------------
    // Gate model
    //
    // EntityByName performs the name lookup.
    // Component<Model>() then verifies that the named entity
    // really is a Gazebo model.
    // --------------------------------------------------------

    if (
      this->gateEntity
      == gz::sim::kNullEntity)
    {
      const auto entity =
        _ecm.EntityByName(
          "safety_gate");

      if (
        entity.has_value()
        && _ecm.Component<
          gz::sim::components::Model>(
            *entity) != nullptr)
      {
        this->gateEntity =
          *entity;
      }
    }


    // --------------------------------------------------------
    // Gazebo point-light entities
    //
    // Do NOT use:
    //
    //   EntityByComponents(Light(), Name(...))
    //
    // because components::Light contains sdf::Light, which
    // cannot be equality-compared by EntityByComponents().
    // --------------------------------------------------------

    if (
      this->greenLight
      == gz::sim::kNullEntity)
    {
      const auto entity =
        _ecm.EntityByName(
          "safety_stack_green");

      if (
        entity.has_value()
        && _ecm.Component<
          gz::sim::components::Light>(
            *entity) != nullptr)
      {
        this->greenLight =
          *entity;
      }
    }


    if (
      this->yellowLight
      == gz::sim::kNullEntity)
    {
      const auto entity =
        _ecm.EntityByName(
          "safety_stack_yellow");

      if (
        entity.has_value()
        && _ecm.Component<
          gz::sim::components::Light>(
            *entity) != nullptr)
      {
        this->yellowLight =
          *entity;
      }
    }


    if (
      this->redLight
      == gz::sim::kNullEntity)
    {
      const auto entity =
        _ecm.EntityByName(
          "safety_stack_red");

      if (
        entity.has_value()
        && _ecm.Component<
          gz::sim::components::Light>(
            *entity) != nullptr)
      {
        this->redLight =
          *entity;
      }
    }


    // --------------------------------------------------------
    // Stack-light lens visuals
    //
    // Visual is a NoData marker component, so we look up the
    // entity by name and then verify the Visual component.
    // --------------------------------------------------------

    if (
      this->greenVisual
      == gz::sim::kNullEntity)
    {
      const auto entity =
        _ecm.EntityByName(
          "green_light_visual");

      if (
        entity.has_value()
        && _ecm.Component<
          gz::sim::components::Visual>(
            *entity) != nullptr)
      {
        this->greenVisual =
          *entity;
      }
    }


    if (
      this->yellowVisual
      == gz::sim::kNullEntity)
    {
      const auto entity =
        _ecm.EntityByName(
          "yellow_light_visual");

      if (
        entity.has_value()
        && _ecm.Component<
          gz::sim::components::Visual>(
            *entity) != nullptr)
      {
        this->yellowVisual =
          *entity;
      }
    }


    if (
      this->redVisual
      == gz::sim::kNullEntity)
    {
      const auto entity =
        _ecm.EntityByName(
          "red_light_visual");

      if (
        entity.has_value()
        && _ecm.Component<
          gz::sim::components::Visual>(
            *entity) != nullptr)
      {
        this->redVisual =
          *entity;
      }
    }
  }


private:
  void ApplyGate(
    gz::sim::EntityComponentManager &_ecm)
  {
    if (
      this->gateEntity
      == gz::sim::kNullEntity)
    {
      return;
    }

    const gz::math::Pose3d closedPose(
      0.000000000, -1.700000000, 0.000000000, 0.000000000, 0.000000000, 0.000000000);

    const gz::math::Pose3d openPose(
      1.300000000, -1.700000000, 0.000000000, 0.000000000, 0.000000000, 0.000000000);

    gz::sim::Model gateModel(
      this->gateEntity);

    gateModel.SetWorldPoseCmd(
      _ecm,
      this->gateOpen
        ? openPose
        : closedPose);
  }


private:
  static void SetColor(
    gz::msgs::Color *_color,
    double _r,
    double _g,
    double _b,
    double _a = 1.0)
  {
    _color->set_r(_r);
    _color->set_g(_g);
    _color->set_b(_b);
    _color->set_a(_a);
  }


private:
  void ApplyLight(
    gz::sim::Entity _lightEntity,
    gz::sim::Entity _visualEntity,
    const std::string &_name,
    double _r,
    double _g,
    double _b,
    bool _on,
    gz::sim::EntityComponentManager &_ecm)
  {
    if (
      _lightEntity
      != gz::sim::kNullEntity)
    {
      gz::msgs::Light command;

      command.set_name(_name);
      command.set_type(
        gz::msgs::Light::POINT);

      command.set_cast_shadows(false);

      command.set_range(0.65);

      command.set_attenuation_constant(
        0.4);

      command.set_attenuation_linear(
        0.8);

      command.set_attenuation_quadratic(
        3.0);

      command.set_intensity(
        _on ? 3.5 : 0.0);

      SetColor(
        command.mutable_diffuse(),
        _r,
        _g,
        _b);

      SetColor(
        command.mutable_specular(),
        _r,
        _g,
        _b);

      _ecm.SetComponentData<
        gz::sim::components::LightCmd>(
          _lightEntity,
          command);
    }

    if (
      _visualEntity
      != gz::sim::kNullEntity)
    {
      gz::msgs::Visual visualCommand;

      visualCommand.set_id(
        _visualEntity);

      auto *material =
        visualCommand.mutable_material();

      const double scale =
        _on ? 1.0 : 0.18;

      SetColor(
        material->mutable_ambient(),
        _r * scale,
        _g * scale,
        _b * scale);

      SetColor(
        material->mutable_diffuse(),
        _r * scale,
        _g * scale,
        _b * scale);

      SetColor(
        material->mutable_emissive(),
        _on ? _r * 0.85 : 0.0,
        _on ? _g * 0.85 : 0.0,
        _on ? _b * 0.85 : 0.0);

      _ecm.SetComponentData<
        gz::sim::components::VisualCmd>(
          _visualEntity,
          visualCommand);
    }
  }


private:
  void ApplyLights(
    gz::sim::EntityComponentManager &_ecm,
    bool _stateChanged)
  {
    bool green = false;
    bool yellow = false;
    bool red = false;

    if (this->state == "RUNNING")
    {
      green = true;
    }
    else if (
      this->state == "MANUAL_PAUSE")
    {
      yellow = true;
    }
    else if (
      this->state == "PROTECTIVE_STOP")
    {
      yellow = this->lastBlinkPhase;
    }
    else if (
      this->state == "E_STOP")
    {
      red = true;
    }
    else
    {
      yellow = true;
    }

    if (
      _stateChanged
      && this->state == "PROTECTIVE_STOP")
    {
      this->lastBlinkPhase = true;
      yellow = true;
    }

    this->ApplyLight(
      this->greenLight,
      this->greenVisual,
      "safety_stack_green",
      0.05,
      1.0,
      0.10,
      green,
      _ecm);

    this->ApplyLight(
      this->yellowLight,
      this->yellowVisual,
      "safety_stack_yellow",
      1.0,
      0.72,
      0.02,
      yellow,
      _ecm);

    this->ApplyLight(
      this->redLight,
      this->redVisual,
      "safety_stack_red",
      1.0,
      0.03,
      0.03,
      red,
      _ecm);
  }


private:
  gz::transport::Node node;

  std::mutex mutex;

  std::optional<std::string>
    pendingState;

  std::optional<bool>
    pendingGate;

  std::string state{
    "MANUAL_PAUSE"};

  bool gateOpen{false};

  bool initialized{false};

  bool lastBlinkPhase{true};

  gz::sim::Entity gateEntity{
    gz::sim::kNullEntity};

  gz::sim::Entity greenLight{
    gz::sim::kNullEntity};

  gz::sim::Entity yellowLight{
    gz::sim::kNullEntity};

  gz::sim::Entity redLight{
    gz::sim::kNullEntity};

  gz::sim::Entity greenVisual{
    gz::sim::kNullEntity};

  gz::sim::Entity yellowVisual{
    gz::sim::kNullEntity};

  gz::sim::Entity redVisual{
    gz::sim::kNullEntity};
};

}  // namespace sorting_cell


GZ_ADD_PLUGIN(
  sorting_cell::SafetyRuntime,
  gz::sim::System,
  sorting_cell::SafetyRuntime::ISystemConfigure,
  sorting_cell::SafetyRuntime::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(
  sorting_cell::SafetyRuntime,
  "sorting_cell::SafetyRuntime")

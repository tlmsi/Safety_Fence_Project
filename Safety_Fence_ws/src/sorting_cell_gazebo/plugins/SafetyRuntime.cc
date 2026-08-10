#include <algorithm>
#include <chrono>
#include <cmath>
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
      "/safety/gui/command",
      &SafetyRuntime::OnGuiCommand,
      this);

    this->pnp1Publisher =
      this->node.Advertise<gz::msgs::Boolean>(
        "/safety/pnp1");

    this->pnp2Publisher =
      this->node.Advertise<gz::msgs::Boolean>(
        "/safety/pnp2");

    this->pnp3Publisher =
      this->node.Advertise<gz::msgs::Boolean>(
        "/safety/pnp3");

    this->pnp4Publisher =
      this->node.Advertise<gz::msgs::Boolean>(
        "/safety/pnp4");
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

    const double simSeconds =
      std::chrono::duration_cast<
        std::chrono::duration<double>>(
          _info.simTime).count();

    if (!this->initialized)
    {
      // CLOSED:
      // door lies along the front fence at yaw = 0.
      this->currentGateYaw = 0.0;
      this->gateTargetYaw = 0.0;

      this->ApplyGatePose(
        _ecm,
        this->currentGateYaw);

      this->ApplyLights(
        _ecm,
        true);

      this->initialized = true;
    }

    // Advance an active hinge movement before processing a
    // possible reversal request.
    this->UpdateGateMotion(
      simSeconds,
      _ecm);

    if (gateChanged)
    {
      this->StartGateMotion(
        this->gateOpen,
        simSeconds);

      this->UpdateGateMotion(
        simSeconds,
        _ecm);
    }

    const auto pnpMilliseconds =
      std::chrono::duration_cast<
        std::chrono::milliseconds>(
          _info.simTime).count();

    if (
      !this->pnpPublished
      || pnpMilliseconds
        < this->lastPnpPublishMs
      || (
        pnpMilliseconds
        - this->lastPnpPublishMs
      ) >= 100
    )
    {
      this->PublishPnpSensors();

      this->lastPnpPublishMs =
        pnpMilliseconds;

      this->pnpPublished =
        true;
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
  void OnGuiCommand(
    const gz::msgs::StringMsg &_message)
  {
    const std::string command =
      _message.data();

    if (
      command != "gate_open"
      && command != "gate_close"
    )
    {
      return;
    }

    std::lock_guard<std::mutex> lock(
      this->mutex);

    this->pendingGate =
      (command == "gate_open");
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
  void ApplyGatePose(
    gz::sim::EntityComponentManager &_ecm,
    double _yaw)
  {
    if (
      this->gateEntity
      == gz::sim::kNullEntity)
    {
      return;
    }

    // --------------------------------------------------------
    // HINGE GEOMETRY
    // --------------------------------------------------------
    //
    // At the CLOSED pose:
    //
    //   gate model origin = (0.00, -1.70)
    //   left physical edge = -0.60 m
    //
    // The fixed left gate post ends at approximately x=-0.60,
    // so this is the natural hinge axis.
    //
    // We cannot simply rotate the model around its own centre,
    // because that would make the whole door orbit around its
    // middle.
    //
    // Instead we calculate a new model origin for every angle
    // so that the LEFT EDGE remains fixed in world space.
    //
    // Hinge world position:
    //
    //   X = -0.60
    //   Y = -1.70
    //
    // Door swings OUTWARD toward negative Y.

    constexpr double hingeWorldX =
      -0.600000000;

    constexpr double hingeWorldY =
      -1.700000000;

    constexpr double hingeLocalX =
      -0.600000000;

    constexpr double hingeLocalY =
      0.000000000;

    const double cosine =
      std::cos(_yaw);

    const double sine =
      std::sin(_yaw);

    const double rotatedHingeX =
      cosine * hingeLocalX
      - sine * hingeLocalY;

    const double rotatedHingeY =
      sine * hingeLocalX
      + cosine * hingeLocalY;

    // Move the model origin so the hinge point itself stays
    // stationary while the rest of the door rotates around it.
    const double modelX =
      hingeWorldX
      - rotatedHingeX;

    const double modelY =
      hingeWorldY
      - rotatedHingeY;

    const gz::math::Pose3d gatePose(
      modelX,
      modelY,
      0.0,
      0.0,
      0.0,
      _yaw);

    gz::sim::Model gateModel(
      this->gateEntity);

    gateModel.SetWorldPoseCmd(
      _ecm,
      gatePose);
  }


private:
  void StartGateMotion(
    bool _open,
    double _simSeconds)
  {
    constexpr double closedYaw =
      0.0;

    // -90 degrees.
    //
    // Negative yaw makes the door swing from the front fence
    // toward negative Y: backwards / outside the robot cell.
    constexpr double openYaw =
      -1.5707963267948966;

    constexpr double fullTravelAngle =
      1.5707963267948966;

    // Keep the same smooth-motion duration used by the
    // previous door animation.
    constexpr double fullTravelDurationSeconds =
      2.0;

    const double requestedTarget =
      _open
        ? openYaw
        : closedYaw;

    this->gateMotionStartYaw =
      this->currentGateYaw;

    this->gateTargetYaw =
      requestedTarget;

    this->gateMotionStartSeconds =
      _simSeconds;

    const double remainingAngle =
      std::abs(
        this->gateTargetYaw
        - this->gateMotionStartYaw);

    if (remainingAngle <= 0.000001)
    {
      this->currentGateYaw =
        this->gateTargetYaw;

      this->gateMotionDurationSeconds =
        0.0;

      this->gateMotionActive =
        false;

      return;
    }

    // Reversing halfway through takes proportionally less
    // time than travelling the full 90 degrees.
    this->gateMotionDurationSeconds =
      fullTravelDurationSeconds
      * (
        remainingAngle
        / fullTravelAngle
      );

    this->gateMotionActive =
      true;
  }


private:
  void UpdateGateMotion(
    double _simSeconds,
    gz::sim::EntityComponentManager &_ecm)
  {
    if (!this->gateMotionActive)
    {
      return;
    }

    if (
      this->gateMotionDurationSeconds
      <= 0.0)
    {
      this->currentGateYaw =
        this->gateTargetYaw;

      this->gateMotionActive =
        false;

      this->ApplyGatePose(
        _ecm,
        this->currentGateYaw);

      return;
    }

    double progress =
      (
        _simSeconds
        - this->gateMotionStartSeconds
      )
      / this->gateMotionDurationSeconds;

    progress =
      std::max(
        0.0,
        std::min(
          1.0,
          progress));

    // Smooth-step motion:
    //
    //   3t² - 2t³
    //
    // Starts gently, moves faster through the middle, and
    // slows again as the door reaches its final angle.
    const double easedProgress =
      progress
      * progress
      * (
        3.0
        - 2.0 * progress
      );

    this->currentGateYaw =
      this->gateMotionStartYaw
      + (
        this->gateTargetYaw
        - this->gateMotionStartYaw
      )
      * easedProgress;

    this->ApplyGatePose(
      _ecm,
      this->currentGateYaw);

    if (progress >= 1.0)
    {
      this->currentGateYaw =
        this->gateTargetYaw;

      this->gateMotionActive =
        false;

      // Apply exact endpoint:
      //
      // CLOSED = 0 degrees
      // OPEN   = -90 degrees
      this->ApplyGatePose(
        _ecm,
        this->currentGateYaw);
    }
  }


private:
  void PublishPnpSensors()
  {
    // --------------------------------------------------------
    // PHYSICAL PNP GEOMETRY
    // --------------------------------------------------------
    //
    // All four sensors have the same XY relationship but are
    // mounted at four independent heights.
    //
    // CLOSED configuration:
    //
    // fixed PNP sensing face:
    //   x = +0.609 m
    //   y = -1.700 m
    //
    // moving metal target sensing surface:
    //   x = +0.604 m
    //   y = -1.700 m
    //
    // initial non-contact air gap:
    //
    //   0.609 - 0.604 = 0.005 m = 5 mm
    //
    // The target rotates with the hinged door around:
    //
    //   hinge = (-0.600, -1.700)
    //
    // Target radius from hinge:
    //
    //   0.604 - (-0.600) = 1.204 m
    //
    // The PNP is considered ON while the target remains
    // within 12 mm of the sensing point.

    constexpr double hingeX =
      -0.600;

    constexpr double hingeY =
      -1.700;

    constexpr double sensorX =
      0.609;

    constexpr double sensorY =
      -1.700;

    constexpr double targetRadius =
      1.204;

    constexpr double sensingDistance =
      0.012;

    const double targetX =
      hingeX
      + std::cos(
        this->currentGateYaw)
      * targetRadius;

    const double targetY =
      hingeY
      + std::sin(
        this->currentGateYaw)
      * targetRadius;

    const double dx =
      targetX - sensorX;

    const double dy =
      targetY - sensorY;

    const double targetDistance =
      std::sqrt(
        dx * dx
        + dy * dy);

    const bool detected =
      targetDistance
      <= sensingDistance;

    // All four channels independently represent their own
    // sensor / target pair.
    //
    // Normal CLOSED pattern: 1111
    // Normal OPEN pattern:   0000
    const bool pnp1 = detected;
    const bool pnp2 = detected;
    const bool pnp3 = detected;
    const bool pnp4 = detected;

    gz::msgs::Boolean message;

    message.set_data(pnp1);
    this->pnp1Publisher.Publish(message);

    message.set_data(pnp2);
    this->pnp2Publisher.Publish(message);

    message.set_data(pnp3);
    this->pnp3Publisher.Publish(message);

    message.set_data(pnp4);
    this->pnp4Publisher.Publish(message);
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

  gz::transport::Node::Publisher
    pnp1Publisher;

  gz::transport::Node::Publisher
    pnp2Publisher;

  gz::transport::Node::Publisher
    pnp3Publisher;

  gz::transport::Node::Publisher
    pnp4Publisher;

  long long lastPnpPublishMs{-1000};

  bool pnpPublished{false};

  std::mutex mutex;

  std::optional<std::string>
    pendingState;

  std::optional<bool>
    pendingGate;

  std::string state{
    "MANUAL_PAUSE"};

  bool gateOpen{false};

  // Smooth hinged gate motion.
  //
  // CLOSED = 0 rad
  // OPEN   = -pi/2 rad
  //
  // The model origin is translated automatically while
  // rotating so the physical hinge remains stationary.
  double currentGateYaw{0.0};

  double gateMotionStartYaw{0.0};

  double gateTargetYaw{0.0};

  double gateMotionStartSeconds{0.0};

  double gateMotionDurationSeconds{0.0};

  bool gateMotionActive{false};

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

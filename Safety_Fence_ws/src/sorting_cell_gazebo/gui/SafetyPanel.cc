#include <iostream>
#include <mutex>
#include <string>

#include <QString>

#include <gz/msgs/boolean.pb.h>
#include <gz/msgs/stringmsg.pb.h>
#include <gz/plugin/Register.hh>

#include "SafetyPanel.hh"


SafetyPanel::SafetyPanel()
  : gz::gui::Plugin()
{
  this->title = "Sorting Cell Safety";

  this->publisher =
    this->node.Advertise<gz::msgs::StringMsg>(
      "/safety/gui/command");

  if (!this->publisher)
  {
    std::cerr
      << "Could not advertise "
      << "/safety/gui/command"
      << std::endl;
  }

  const bool stateSubscribed =
    this->node.Subscribe(
      "/safety/state",
      &SafetyPanel::OnSafetyState,
      this);

  const bool gateSubscribed =
    this->node.Subscribe(
      "/safety/gate_visual_open",
      &SafetyPanel::OnGateState,
      this);

  if (!stateSubscribed)
  {
    std::cerr
      << "Could not subscribe to "
      << "/safety/state"
      << std::endl;
  }

  if (!gateSubscribed)
  {
    std::cerr
      << "Could not subscribe to "
      << "/safety/gate_visual_open"
      << std::endl;
  }
}


SafetyPanel::~SafetyPanel() = default;


void SafetyPanel::SendCommand(
  const QString &_command)
{
  gz::msgs::StringMsg message;

  message.set_data(
    _command.toStdString());

  this->publisher.Publish(
    message);
}


QString SafetyPanel::SafetyState() const
{
  std::lock_guard<std::mutex> lock(
    this->mutex);

  return QString::fromStdString(
    this->safetyState);
}


bool SafetyPanel::GateOpen() const
{
  std::lock_guard<std::mutex> lock(
    this->mutex);

  return this->gateOpen;
}


void SafetyPanel::OnSafetyState(
  const gz::msgs::StringMsg &_message)
{
  std::lock_guard<std::mutex> lock(
    this->mutex);

  this->safetyState =
    _message.data();
}


void SafetyPanel::OnGateState(
  const gz::msgs::Boolean &_message)
{
  std::lock_guard<std::mutex> lock(
    this->mutex);

  this->gateOpen =
    _message.data();
}


GZ_ADD_PLUGIN(
  SafetyPanel,
  gz::gui::Plugin)

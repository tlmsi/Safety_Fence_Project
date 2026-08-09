#include <iostream>
#include <string>

#include <gz/msgs/stringmsg.pb.h>
#include <gz/plugin/Register.hh>

#include "SafetyPanel.hh"


SafetyPanel::SafetyPanel()
  : gz::gui::Plugin()
{
  this->title = "Safety Control";

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


GZ_ADD_PLUGIN(
  SafetyPanel,
  gz::gui::Plugin)

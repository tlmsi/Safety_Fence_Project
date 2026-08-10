#ifndef SORTING_CELL_SAFETY_PANEL_HH_
#define SORTING_CELL_SAFETY_PANEL_HH_

#include <mutex>
#include <string>

#include <QString>

#include <gz/gui/Plugin.hh>
#include <gz/msgs/boolean.pb.h>
#include <gz/msgs/stringmsg.pb.h>
#include <gz/transport/Node.hh>


class SafetyPanel : public gz::gui::Plugin
{
  Q_OBJECT

public:
  SafetyPanel();

  ~SafetyPanel() override;

public:
  Q_INVOKABLE void SendCommand(
    const QString &_command);

  Q_INVOKABLE QString SafetyState() const;

  Q_INVOKABLE bool GateOpen() const;

private:
  void OnSafetyState(
    const gz::msgs::StringMsg &_message);

  void OnGateState(
    const gz::msgs::Boolean &_message);

private:
  gz::transport::Node node;

  gz::transport::Node::Publisher publisher;

  mutable std::mutex mutex;

  std::string safetyState{"UNKNOWN"};

  bool gateOpen{false};
};

#endif

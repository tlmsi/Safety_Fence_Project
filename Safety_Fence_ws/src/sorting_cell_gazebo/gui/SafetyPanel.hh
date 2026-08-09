#ifndef SORTING_CELL_SAFETY_PANEL_HH_
#define SORTING_CELL_SAFETY_PANEL_HH_

#include <QString>

#include <gz/gui/Plugin.hh>
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

private:
  gz::transport::Node node;

  gz::transport::Node::Publisher publisher;
};

#endif

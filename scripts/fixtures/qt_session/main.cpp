// Check the Qt/D-Bus lifetime without initializing a window or GLX. Rendering
// and its allocation lifetimes are covered by the separate desktop suite.
#include <QCoreApplication>
#include <QDBusConnection>
#include <QDBusConnectionInterface>
#include <QDBusReply>
#include <QTimer>
#include <iostream>

int main(int argc, char **argv) {
  QCoreApplication app(argc, argv);
  const auto bus = QDBusConnection::sessionBus();
  if (!bus.isConnected()) return 2;
  const QDBusReply<uint> pid = bus.interface()->servicePid("org.freedesktop.DBus");
  if (!pid.isValid()) return 3;
  std::cout << "BUS_ADDRESS=" << qgetenv("DBUS_SESSION_BUS_ADDRESS").constData() << '\n'
            << "BUS_PID=" << pid.value() << std::endl;
  QTimer::singleShot(200, &app, &QCoreApplication::quit);
  const int result = app.exec();
  return result ? result : (argc > 1 ? 17 : 0);
}

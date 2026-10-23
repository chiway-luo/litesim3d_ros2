// Minimal Qt lifetime used to check the headless test session, without rendering
// engines or leak suppressions masking D-Bus initialization/shutdown failures.
#include <QApplication>
#include <QDBusConnection>
#include <QDBusConnectionInterface>
#include <QDBusReply>
#include <QTimer>
#include <QWidget>
#include <iostream>

int main(int argc, char **argv) {
  QApplication app(argc, argv);
  QWidget window;
  window.resize(200, 100);
  window.show();
  const auto bus = QDBusConnection::sessionBus();
  if (!bus.isConnected()) return 2;
  const QDBusReply<uint> pid = bus.interface()->servicePid("org.freedesktop.DBus");
  if (!pid.isValid()) return 3;
  std::cout << "BUS_ADDRESS=" << qgetenv("DBUS_SESSION_BUS_ADDRESS").constData() << '\n'
            << "BUS_PID=" << pid.value() << std::endl;
  QTimer::singleShot(200, &app, &QApplication::quit);
  const int result = app.exec();
  return result ? result : (argc > 1 ? 17 : 0);
}

#ifndef INPUT_H
#define INPUT_H

#include <Qt>
#include <QPoint>

class Input
{
public:

  // Possible key states
  enum InputState
  {
    InputInvalid,
    InputRegistered,
    InputUnregistered,
    InputTriggered,
    InputPressed,
    InputReleased
  };

  // State checking
  static InputState keyState(Qt::Key key);
  static bool keyTriggered(Qt::Key key);
  static bool keyPressed(Qt::Key key);
  static bool keyReleased(Qt::Key key);

private:

  // State updating
  static void update();
  static void registerKeyPress(int key);
  static void registerKeyRelease(int key);
  static void reset();
  friend class simulatorWindow;
};

inline bool Input::keyTriggered(Qt::Key key)
{
  return keyState(key) == InputTriggered;
}

inline bool Input::keyPressed(Qt::Key key)
{
  return keyState(key) == InputPressed;
}

inline bool Input::keyReleased(Qt::Key key)
{
  return keyState(key) == InputReleased;
}

#endif // INPUT_H

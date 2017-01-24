#include "input.h"
#include <QCursor>
#include <vector>
#include <algorithm>

// Static Helper Structs
template <typename T>
struct InputInstance : std::pair<T, Input::InputState>
{
  typedef std::pair<T, Input::InputState> base_class;
  inline InputInstance(T value) : base_class(value, Input::InputInvalid) {}
  inline InputInstance(T value, Input::InputState state) : base_class(value, state) {}
  inline bool operator==(const InputInstance &rhs) const
  {
    return this->first == rhs.first;
  }
};

// Instance types
typedef InputInstance<Qt::Key> KeyInstance;

// Container types
typedef std::vector<KeyInstance> KeyContainer;

// Globals
static KeyContainer sg_keyInstances;

// Static Helper Fucntions
static inline KeyContainer::iterator FindKey(Qt::Key value)
{
  return std::find(sg_keyInstances.begin(), sg_keyInstances.end(), value);
}

template <typename TPair>
static inline void UpdateStates(TPair &instance)
{
  switch (instance.second)
  {
  case Input::InputRegistered:
    instance.second = Input::InputTriggered;
    break;
  case Input::InputTriggered:
    instance.second = Input::InputPressed;
    break;
  case Input::InputUnregistered:
    instance.second = Input::InputReleased;
    break;
  default:
    break;
  }
}

template <typename TPair>
static inline bool CheckReleased(const TPair &instance)
{
  return instance.second == Input::InputReleased;
}

template <typename Container>
static inline void Update(Container &container)
{
  typedef typename Container::iterator Iter;
  typedef typename Container::value_type TPair;

  // Remove old data
  Iter remove = std::remove_if(container.begin(), container.end(), &CheckReleased<TPair>);
  container.erase(remove, container.end());

  // Update existing data
  std::for_each(container.begin(), container.end(), &UpdateStates<TPair>);
}

// Input Implementation
Input::InputState Input::keyState(Qt::Key k)
{
  KeyContainer::iterator it = FindKey(k);
  return (it != sg_keyInstances.end()) ? it->second : InputInvalid;
}

void Input::update()
{
  // Update KeyState values
  Update(sg_keyInstances);
}

void Input::registerKeyPress(int k)
{
  KeyContainer::iterator it = FindKey((Qt::Key)k);
  if (it == sg_keyInstances.end())
  {
    sg_keyInstances.push_back(KeyInstance((Qt::Key)k, InputRegistered));
  }
}

void Input::registerKeyRelease(int k)
{
  KeyContainer::iterator it = FindKey((Qt::Key)k);
  if (it != sg_keyInstances.end())
  {
    it->second = InputUnregistered;
  }
}

void Input::reset()
{
  sg_keyInstances.clear();
}

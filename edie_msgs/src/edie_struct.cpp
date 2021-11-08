#include <edie_struct.h>

std::ostream& operator<<(std::ostream& os, EmotionState state)
{
  switch(state)
  {
    case EmotionState::BLINK         : os << "BLINK"; break;
    case EmotionState::FLINCH        : os << "FLINCH"; break;
    case EmotionState::LAUGH         : os << "LAUGH"; break;
    case EmotionState::LOOK_AROUND   : os << "LOOK_AROUND"; break;
    case EmotionState::LOVE          : os << "LOVE"; break;
    case EmotionState::SURPRISE      : os << "SURPRISE"; break;
    default                          : os << "ERROR : " << static_cast<int>(state); break;
  }
  return os; 
}

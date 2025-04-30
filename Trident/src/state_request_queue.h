#include "model.h"

void initStateQueue();
bool enqueueStateRequest(StateRequestT req, StateSourceT source);
void processStateQueue();
StateRequestT getCurrentState();

#include "../common/types.h"
#include "state_store.h"

class Quadcopter {
    public: 
        Quadcopter(StateStore* state_store);

        
    private:
        StateStore* state_store;

};
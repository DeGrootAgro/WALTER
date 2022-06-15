#include <map>

enum States{
    ST_IDLE,
    ST_CHARGING,
    ST_NAVIGATE_TO_CHARGING,
    ST_MANUAL_CONTROL,
    ST_COLLECTING,
    ST_NAVIGATE_TO_LOADING,
    ST_LOADING,
    ST_EMERGENCY_STOP
};
enum Events{
    E_BATT_LOW,
    E_ARRIVED_AT_CHARGING,
    E_BATT_FULL,
    E_START_MANUAL_CONTROL,
    E_START_COLLECTING,
    E_EMERGENCY_STOP,
    E_LOAD_FULL,
    E_ARRIVED_AT_LOADING,
    E_GO_IDLE
};

class state_machine {
    public:
     void update(void);
     void raise_event(std::map<States,Events> &event);
};
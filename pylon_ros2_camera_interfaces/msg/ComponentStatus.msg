# component id; it must be unique among all registered components
# @TODO: use on one topic and identify by id
#string component_id

# the official status id of the component
# possible values are
int8 INITIALIZED = 0
int8 STOPPED = 1
int8 RUNNING = 2
int8 CONFIG_NEEDED = 3
int8 ERROR = 4
int8 INTERACTION_REQUEST = 5
int8 DEACTIVATED = 6
int8 status_id

# an individual message for config or error cases
# it should describe the type of needed config or occurred error briefly
# it should be possible to extract automaticly subsequent actions/instructions from the message if this is needed
string status_msg

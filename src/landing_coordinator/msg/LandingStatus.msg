# Authoritative single-landing-resource mission state.
uint8 AVAILABLE=0
uint8 RESERVED=1
uint8 COMPLETED=2

# Coordinator-local mission identity; retained across status updates.
uint64 mission_id
# Current owner; retained when COMPLETED so the landing slot stays occupied.
string owner_id
uint8 state

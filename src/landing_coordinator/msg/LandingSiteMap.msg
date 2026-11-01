# Atomically retained catalogue for one completed mission.
# The owner is kept to correlate this data with LandingStatus.COMPLETED.
uint64 mission_id
string owner_id
string frame_id
LandingSite[] sites

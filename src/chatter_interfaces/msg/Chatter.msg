uint64 timestamp
uint64 count
# Shared memory backend can only deal with fixed size messages.
# 33177600 = 3840x2160x4 (largest encoding have 4 channels per pixel)
uint64 MAX_DATA_SIZE = 33177600
char[33177600] data
uint64 data_length
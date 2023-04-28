# This function searches for the name of the variable and returns its value
# Assumptions: 
#   - The texts around the variable is in the following format:
#         variable_name = number
#   - The number has to locate between the equal sign and newline
def FindVarValueInString(file_string, string_to_search):
  # We search from the end of the file
  word_location = file_string.rfind(string_to_search)
  number_idx_start = 0
  number_idx_end = 0
  idx = word_location
  while True:
    if file_string[idx] == '=':
      number_idx_start = idx
    elif file_string[idx] == '\n':
      number_idx_end = idx
      break
    idx += 1
  value = float(file_string[number_idx_start + 1: number_idx_end])
  return value

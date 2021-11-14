# Cassie Log Plotting Script Technical Notes

## Reading LCM messages
To read and process lcm messages from a set of channels, a dictionary of these channels (keys) and their lcm types (items) and a corresponding processing callback function must be passed to `process_lcm_log.get_log_data()`. See `load_default_channels` in `cassie_plotting_utils` as an example callback function. 

## Default channels
*`load_default_channels`* returns the following three dictionaries, whose structure mirrors the lcm channels which their data comes from. 

### `robot_output` 
This contains the robots state and *measured* inputs and timestamps, each as a numpy array where the row is the time dimension and the column is the position/velocity index.

Dict keys: `'t_x', 'q', 'v', 'u'`

### `robot input`
This contains the commanded inputs and timestamps.

Dict keys: `'t_u', 'u'`

### `osc_debug`
This contains the osc debug timestamps, the osc debug lcm messages, and a list of osc tracking datas in the same conveniece class we have been using. Implementation of further processing functions for the OSC is ongoing. 

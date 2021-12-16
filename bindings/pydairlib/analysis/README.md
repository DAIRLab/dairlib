# Log Plotting Script Technical Notes

## Reading LCM messages
To read and process lcm messages from a set of channels, a dictionary of these channels (keys) and their lcm types (items) and a corresponding processing callback function must be passed to `process_lcm_log.get_log_data()`. See `load_default_channels` in `mbp_plotting_utils` as an example callback function. 

## Default channels
*`load_default_channels`* returns the following three dictionaries, whose structure mirrors the lcm channels which their data comes from. 

### `robot_output` 
This contains the robots state and *measured* inputs and timestamps, each as a numpy array where the row is the time dimension and the column is the position/velocity index.

Dict keys: `'t_x', 'q', 'v', 'u'`

### `robot input`
This contains the commanded inputs and timestamps.

Dict keys: `'t_u', 'u'`

### `osc_debug`
This contains the osc debug timestamps, the osc debug lcm messages, and a list of osc tracking datas in the same conveniece class we have been using for Cassie logs (see `osc_debug.py`). 

Dict keys :
```
't_osc', 'input_cost', 'acceleration_cost', 'soft_constraint_cost',
'qp_solve_time', 'u_sol', 'lambda_c_sol, 'lambda_h_sol', 'dv_sol', 
'epsilon_sol', 'osc_output', 'tracking_cost', 'osc_debug_tracking_datas', 'fsm'
```

# Ongoing Progress:
As of Dec. 16 2021, plotting for `lambda_c_sol` and `epsilon_sol` in `osc_debug`
have been implemented but not added to `cassie_plot_config` or the default yaml 
file

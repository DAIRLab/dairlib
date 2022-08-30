# Notes for residual analysis

## Main function:
"Residual analysis.py"

e.g. : "bazel-bin/bindings/pydairlib/analysis/residual_analysis/residual_analysis log/hardware_data/03_15_22/lcmlog-11"

There choose flags to use wandb, which is an online software for quick make plots without processing data over again 

## For log reading:

Refer to "log_processor.py" for more information. 

Currently only load messages from "CASSIE_STATE_DISPATCHER" and "CASSIE_CONTACT_DISPATCHER"

## For cassie model used:

Refer to "cassie_model.py":
     
"changed_stiffness" mode means pre-fitted piecewise linear spring model

"constant spring" mode means linear spring model with parameters from urdf

## For post-data process:

Refer to "residual_processor.py":

The also provide some other functionality such as fit spring stiffness which maybe useful but not include in the residual_analysis.py main file
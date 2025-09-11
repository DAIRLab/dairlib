#!/bin/bash

# experiments=("lotion_and_R" "baby_toy_and_E" "B_and_3" "chicken_broth_and_expo" "clamp_and_I" "tape_and_A" "C" "eraser" "expo" "R" "letter_3" "egg" "E" "milk" "Y" "baby_toy" "G" "H" "I" "B" "wood_block" "lotion" "chicken_broth" "book" "tape" "gallon_milk" "A" "clamp" "S" "push_t")
# experiments=("RAS" "C3+" "ANY" "ING" "DIY" "clamp_lotion_book")
experiments=("DAY+")
for exp in "${experiments[@]}"; do
  echo ""
  echo "Processing experiment: $exp"
  python examples/sampling_c3/process_lcm_logs.py yaml "$exp"
  echo "Done with experiment: $exp"
  echo ""
done

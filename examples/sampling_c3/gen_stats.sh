#!/bin/bash

# All experiments:
experiments=("C" "eraser" "expo" "R" "letter_3" "egg" "E" "milk" "Y" "baby_toy" "G" "H" "I" "B" "wood_block" "lotion" "chicken_broth" "book" "tape" "gallon_milk" "A" "clamp" "S" "push_t" "xbox" "lotion_and_R" "baby_toy_and_E" "B_and_3" "chicken_broth_and_expo" "chicken_broth_and_wood_block" "clamp_and_I" "book_and_S" "tape_and_A" "T_and_H" "G_and_xbox" "RAS" "C3+" "ANY" "ING" "DIY" "clamp_lotion_book" "DAY+" "ICRA" "PUSH" "URDF" "C3PO")
for exp in "${experiments[@]}"; do
  echo ""
  echo "Processing experiment: $exp"
  python examples/sampling_c3/process_lcm_logs.py yaml "$exp" --skip-plots --videos
  echo "Done with experiment: $exp"
  echo ""
done

from pydairlib.perceptive_locomotion.perception_learning.sb3_contrib.common.recurrent.policies import (
    RecurrentActorCriticCnnPolicy,
    RecurrentActorCriticPolicy,
    RecurrentMultiInputActorCriticPolicy,
)

MlpLstmPolicy = RecurrentActorCriticPolicy
CnnLstmPolicy = RecurrentActorCriticCnnPolicy
MultiInputLstmPolicy = RecurrentMultiInputActorCriticPolicy

# This file is here just to define MlpPolicy/CnnPolicy
# that work for PPO
#from stable_baselines3.common.policies import ActorCriticCnnPolicy, ActorCriticPolicy, MultiInputActorCriticPolicy
from pydairlib.perceptive_locomotion.perception_learning.stable_baselines3.common.policies /
import ActorCriticCnnPolicy, ActorCriticPolicy, MultiInputActorCriticPolicy

MlpPolicy = ActorCriticPolicy
CnnPolicy = ActorCriticCnnPolicy
MultiInputPolicy = MultiInputActorCriticPolicy

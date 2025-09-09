import argparse
import os,sys
import pickle
import torch
import numpy as np

project_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(project_root)

from rsl_rl.runners import OnPolicyRunner
from chrono_env import ChronoQuadrupedEnv as RigidTerrainEnv
from chrono_crmenv import ChronoQuadrupedEnv as GranularTerrainEnv
def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("-e", "--exp_name", type=str, default="chrono-quadruped")
    parser.add_argument("--ckpt", type=int, default=2000, help="Checkpoint number to load")
    parser.add_argument("--max_steps", type=int, default=500, help="Maximum steps per episode")
    parser.add_argument("--env", type=str, default="rigid", help="Environment type (rigid, granular)")

    args = parser.parse_args()

    log_dir = f"{project_root}/data/rslrl"
    
    # Create environment
    device = 'cpu'
    env_type = args.env # rigid, granular

    if env_type == "granular":
        env_cfg, train_cfg = pickle.load(open(f"{log_dir}/cfgs_crm.pkl", "rb"))
        env = GranularTerrainEnv(
            num_envs=1,
            env_cfg=env_cfg,
            device=device,
            render = True
        )
    else:
        env_cfg, train_cfg = pickle.load(open(f"{log_dir}/cfgs_rigid.pkl", "rb"))
        env = RigidTerrainEnv(
            num_envs=1,
            env_cfg=env_cfg,
            device=device,
            render = True
        )
    
    # Create runner and load model
    runner = OnPolicyRunner(env, train_cfg, log_dir, device=device)
    resume_path = os.path.join(log_dir, f"model_{args.ckpt}.pt")
    
    try:
        runner.load(resume_path)
        print(f"Loaded model from {resume_path}")
    except FileNotFoundError:
        print(f"Model checkpoint not found at {resume_path}")
        print("Available checkpoints:")
        for file in os.listdir(log_dir):
            if file.startswith("model_") and file.endswith(".pt"):
                print(f"  - {file}")
        return
    
    # Get inference policy
    policy = runner.get_inference_policy(device=device)
    print("Policy loaded successfully")

    # Reset environment
    obs, _ = env.reset()
    print(f"Environment reset. Observation shape: {obs.shape}")
    
    # Evaluation statistics
    episode_rewards = []
    episode_lengths = []
    current_episode_reward = torch.zeros(1, device=device)
    current_episode_length = torch.zeros(1, device=device, dtype=torch.long)
    
    print(f"\nStarting evaluation for {args.max_steps} steps...")
    print("=" * 50)
    
    step_count = 0
    with torch.no_grad():
        while step_count < args.max_steps:
            # Get actions from policy
            actions = policy(obs)

            # Step environment
            obs, rewards, dones, extras = env.step(actions)
            #print(f"rewards: {rewards}")

            # Update statistics
            current_episode_reward += rewards
            current_episode_length += 1
            
            # Check for episode completion
            if torch.any(dones):
                completed_envs = dones.nonzero(as_tuple=False).flatten()
                for env_idx in completed_envs:
                    episode_rewards.append(current_episode_reward[env_idx].item())
                    episode_lengths.append(current_episode_length[env_idx].item())
                    
                    print(f"Episode completed in env {env_idx}: "
                          f"Reward = {current_episode_reward[env_idx]:.3f}, "
                          f"Length = {current_episode_length[env_idx]}")
                    
                    # Reset counters for completed environments
                    current_episode_reward[env_idx] = 0.0
                    current_episode_length[env_idx] = 0
            
            step_count += 1
            
            # Print progress every 100 steps
            if step_count % 100 == 0:
                print(f"Step {step_count}/{args.max_steps}")
                



if __name__ == "__main__":
    main()


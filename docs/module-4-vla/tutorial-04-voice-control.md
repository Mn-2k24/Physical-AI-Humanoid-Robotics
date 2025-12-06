# Tutorial 4: Voice-Controlled Humanoid Navigation

## Overview

In this hands-on tutorial, you'll build a complete voice-controlled navigation system for a simulated humanoid robot. This tutorial integrates all components from Module 4: speech recognition (Whisper), cognitive planning (LLM), visual perception (object detection), and autonomous navigation (Nav2).

**Learning Objectives**:
- Set up complete audio-to-action pipeline in ROS 2
- Configure Whisper for real-time speech recognition
- Implement LLM-based task planner
- Integrate with Nav2 for autonomous navigation
- Test end-to-end system with voice commands

**Prerequisites**:
- Completed Modules 1-3 (ROS 2, Simulation, Isaac)
- Chapters 13-15 of Module 4
- Ubuntu 22.04 with ROS 2 Humble
- Microphone for speech input

**Time Commitment**: 3-4 hours

## Tutorial Coming Soon

This tutorial is currently under development and will include:

### Part 1: Environment Setup
- Launch Gazebo/Isaac simulation with humanoid robot
- Configure RGB-D camera and sensors
- Set up navigation stack (Nav2)
- Verify robot teleoperation

### Part 2: Speech Recognition Pipeline
- Install Whisper and dependencies
- Create audio capture node
- Implement voice activity detection
- Test speech-to-text accuracy

### Part 3: Cognitive Planning Integration
- Configure LLM API (OpenAI or local model)
- Implement task decomposition prompt
- Create plan validation logic
- Test planning with sample commands

### Part 4: End-to-End Integration
- Connect speech → planning → navigation pipeline
- Implement plan executor for Nav2 actions
- Add execution monitoring and failure recovery
- Test complete workflow with voice commands

### Part 5: Evaluation
- Run test scenarios from Chapter 16
- Measure success rate and latency
- Debug common failure modes
- Document results

## Quick Start (Preview)

While the full tutorial is in development, you can explore the complete implementation in the companion code repository:

**Code Location**: `physical-ai-code/tutorials/module-4-vla/01-capstone/`

**File Structure**:
```
01-capstone/
├── README.md                    # Detailed setup instructions
├── speech/
│   ├── whisper_node.py         # Whisper STT integration
│   └── audio_capture.py        # Microphone interface
├── planning/
│   ├── llm_planner.py          # LLM cognitive planner
│   ├── prompts.py              # Prompt templates
│   └── safety_validator.py    # Safety checks
├── control/
│   ├── action_executor.py      # ROS 2 action client
│   └── plan_executor.py        # Executes action sequences
├── launch/
│   └── voice_control.launch.py # Launch all nodes
├── config/
│   ├── whisper_params.yaml
│   └── nav2_params.yaml
└── verify.py                    # Automated testing script
```

**Launch Command** (when available):
```bash
# Terminal 1: Start simulation
ros2 launch voice_control simulation.launch.py

# Terminal 2: Launch voice control system
ros2 launch voice_control voice_control.launch.py

# Terminal 3: Monitor transcriptions
ros2 topic echo /speech/transcription
```

## Example Voice Commands

Once the system is running, try these commands:

**Basic Navigation**:
- "Go to the kitchen"
- "Navigate to the living room"
- "Move to the table"

**Object Interaction**:
- "Find the red cup"
- "Look for books on the shelf"
- "Detect objects on the table"

**Multi-Step Tasks**:
- "Go to the kitchen and find the water bottle"
- "Navigate to the table and tell me what you see"

## Expected Behavior

When you say "Go to the kitchen":

1. **Whisper STT** transcribes audio to text: `"go to the kitchen"`
2. **LLM Planner** generates action sequence:
   ```json
   [
     {"action": "navigate", "target": "kitchen"}
   ]
   ```
3. **Plan Executor** sends goal to Nav2
4. **Nav2** computes path and executes navigation
5. **Robot** arrives at kitchen, reports completion

**Latency**: Expect 2-4 seconds from voice command to robot movement start.

## Troubleshooting

### Speech Recognition Not Working

**Problem**: No transcriptions appearing on `/speech/transcription`

**Solutions**:
- Check microphone permissions: `arecord -l`
- Verify Whisper model loaded: Check node logs
- Test with sample audio file instead of live microphone

### LLM Planning Errors

**Problem**: Planner generates invalid JSON or nonsensical actions

**Solutions**:
- Verify API key configured correctly
- Check prompt template includes examples
- Try simpler commands first ("go to kitchen" before multi-step tasks)
- Use GPT-4 instead of GPT-3.5 if available

### Navigation Failures

**Problem**: Robot gets stuck or collides with obstacles

**Solutions**:
- Increase inflation radius in costmap config
- Check localization working (run `ros2 topic echo /amcl_pose`)
- Verify TF tree connected: `ros2 run tf2_tools view_frames`

## Next Steps

After completing this tutorial, explore these extensions:

1. **Add Object Manipulation**: Implement pick-and-place actions
2. **Multi-Room Navigation**: Create larger world with multiple locations
3. **Conditional Commands**: "If you see a cup, pick it up"
4. **Voice Feedback**: Add text-to-speech for robot responses
5. **Web Dashboard**: Visualize detections and plans in browser

## Resources

**Code Repository**:
- Full tutorial implementation: [physical-ai-code/tutorials/module-4-vla/01-capstone/](https://github.com/nizam/physical-ai-code)

**Documentation**:
- [Whisper Setup Guide](https://github.com/openai/whisper)
- [Nav2 Documentation](https://navigation.ros.org)
- [ROS 2 Actions Tutorial](https://docs.ros.org/en/humble/Tutorials/Intermediate/Writing-an-Action-Server-Client/Py.html)

**Support**:
- Report issues: [GitHub Issues](https://github.com/nizam/Physical-AI-Humanoid-Robotics/issues)
- Ask questions: [ROS Answers](https://answers.ros.org)

---

**Note**: Full step-by-step tutorial will be added in a future update. In the meantime, refer to the code repository README for detailed implementation guidance.

---

**Previous**: [← Chapter 16: Capstone Project](./16-capstone-project.md) | **Module**: [Module 4 Overview](./index.md)

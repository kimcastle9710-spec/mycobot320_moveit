# mycobot320_moveit
The `fake_sym` branch is designed to decouple motion planning and controller
testing from physical hardware availability.

This allows:
- Early-stage development without robot access
- Faster iteration on MoveIt2 and controller configuration
- Clear separation between planning logic and hardware-specific issues

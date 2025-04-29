
    def pick_butter(self):
        # Open gripper, move down, close, and lift
        self.set_gripper_width(0.02)
        time.sleep(0.5)
        # pre-grasp joint pose (rad)
        pre_grasp = [0.0, 0.7, -1.2, 1.0]
        grasp = [0.0, 0.9, -1.4, 1.2]
        lift = [0.0, 0.5, -1.0, 0.8]
        for pose in [pre_grasp, grasp]:
            self.go_to_joint_state_arm(pose)
            time.sleep(1)
        self.set_gripper_width(-0.01)
        time.sleep(0.5)
        self.go_to_joint_state_arm(lift)
        time.sleep(1)

    def drop_butter(self):
        # Lower, open, and retract
        lower = [0.0, 0.6, -1.1, 0.9]
        self.go_to_joint_state_arm(lower)
        time.sleep(1)
        self.set_gripper_width(0.02)
        time.sleep(0.5)
        # home pose
        self.go_to_joint_state_arm([0.0, 0.0, 0.0, 0.0])
        time.sleep(1)
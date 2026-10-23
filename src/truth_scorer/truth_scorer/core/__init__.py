"""Pure-Python scoring core for the truth scorer.

Nothing in this subpackage may import rclpy or any other ROS module. The ROS nodes in the parent
package are thin wrappers around this code, which keeps the scoring maths unit-testable without a
running simulator, a sourced ROS environment, or a colcon build. A test enforces the rule.
"""

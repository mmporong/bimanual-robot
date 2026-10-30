"""Receive TF on a dedicated node and daemon thread, independent of the owner's spin."""

import threading

import rclpy
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from tf2_ros import TransformListener


class DedicatedTfListener:
    """Fill ``buffer`` from /tf while the owner node is busy or blocked in a lookup.

    The owner spins itself only between control computations, and a lookup with
    a timeout sleep-polls on the caller's thread, so /tf queued behind it. This
    listener spins its own node (never the owner: a second executor on it would
    race the owner's spin) on a daemon thread, so a constructor that fails after
    this point cannot keep the process alive. ``shutdown()`` stops it cleanly.
    """

    def __init__(self, buffer, owner, name):
        # Like tf2_ros's own helper node: a launch-wide __node remap must not
        # rename this node to the owner's name.
        self.node = rclpy.create_node(
            name, context=owner.context, cli_args=['--ros-args', '-r', f'__node:={name}'],
            enable_rosout=False, start_parameter_services=False)
        self.listener = TransformListener(buffer, self.node)
        self.executor = SingleThreadedExecutor(context=owner.context)
        self.executor.add_node(self.node)
        self.thread = threading.Thread(target=self._spin, name=name, daemon=True)
        self.thread.start()

    def _spin(self):
        try:
            self.executor.spin()
        except ExternalShutdownException:
            pass

    def shutdown(self):
        """Stop the spin thread, then release the subscriptions and the node."""
        self.executor.shutdown()
        self.thread.join(timeout=1.0)
        self.listener.unregister()
        self.node.destroy_node()

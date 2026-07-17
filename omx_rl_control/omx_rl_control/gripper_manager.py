"""Asynchronous GripperCommand action wrapper."""

from enum import Enum

from action_msgs.msg import GoalStatus
from control_msgs.action import GripperCommand
from rclpy.action import ActionClient


class GripperState(str, Enum):
    """Current state of one gripper command."""

    IDLE = 'IDLE'
    SENDING = 'SENDING'
    ACTIVE = 'ACTIVE'
    SUCCEEDED = 'SUCCEEDED'
    FAILED = 'FAILED'


def gripper_result_succeeded(
    status: int,
    reached_goal: bool,
    stalled: bool,
    allow_stall: bool,
) -> bool:
    """Accept contact stalls only for commands that explicitly allow them."""
    if status == GoalStatus.STATUS_SUCCEEDED:
        return bool(reached_goal or (allow_stall and stalled))
    return bool(
        status == GoalStatus.STATUS_ABORTED
        and allow_stall
        and stalled
    )


class GripperManager:
    """Send one gripper command and expose a polling-friendly result."""

    def __init__(self, node, action_name: str):
        self._client = ActionClient(node, GripperCommand, action_name)
        self.state = GripperState.IDLE
        self.message = ''
        self.position = 0.0
        self.effort = 0.0
        self.stalled = False
        self.reached_goal = False
        self._allow_stall = False
        self._goal_handle = None
        self._cancel_requested = False
        self._command_id = 0
        self._canceled_ids = set()

    @property
    def server_ready(self) -> bool:
        """Return whether the ros2_control action server is discoverable."""
        return bool(self._client.server_is_ready())

    @property
    def busy(self) -> bool:
        """Return true while a command is in flight."""
        return self.state in (GripperState.SENDING, GripperState.ACTIVE)

    @property
    def succeeded(self) -> bool:
        """Return true when the latest command met its completion rule."""
        return self.state == GripperState.SUCCEEDED

    @property
    def failed(self) -> bool:
        """Return true when the latest command was rejected or failed."""
        return self.state == GripperState.FAILED

    def reset(self) -> None:
        """Clear a completed result before the next command."""
        if self.busy:
            return
        self.state = GripperState.IDLE
        self.message = ''
        self._goal_handle = None
        self._cancel_requested = False

    def command(
        self,
        position: float,
        max_effort: float,
        allow_stall: bool,
    ) -> bool:
        """Start one GripperCommand goal if the server is ready."""
        if self.busy:
            return False
        self.reset()
        if not self.server_ready:
            self.state = GripperState.FAILED
            self.message = 'gripper action server is unavailable'
            return False

        goal = GripperCommand.Goal()
        goal.command.position = float(position)
        goal.command.max_effort = float(max_effort)
        self._allow_stall = bool(allow_stall)
        self._cancel_requested = False
        self._command_id += 1
        command_id = self._command_id
        self.state = GripperState.SENDING
        self.message = f'goal position={position:.4f}'
        future = self._client.send_goal_async(goal)
        future.add_done_callback(
            lambda result: self._on_goal_response(result, command_id)
        )
        return True

    def cancel(self) -> None:
        """Cancel an active goal and block late callbacks from succeeding."""
        self._cancel_requested = True
        if self._command_id:
            self._canceled_ids.add(self._command_id)
        if self._goal_handle is not None:
            self._goal_handle.cancel_goal_async()
        self.state = GripperState.FAILED
        self.message = 'gripper command canceled by safety stop'

    def _on_goal_response(self, future, command_id: int) -> None:
        try:
            goal_handle = future.result()
        except Exception as error:  # rclpy future exceptions are runtime data
            if command_id != self._command_id:
                return
            self.state = GripperState.FAILED
            self.message = f'gripper goal exception: {error}'
            return
        if command_id != self._command_id:
            if goal_handle.accepted:
                goal_handle.cancel_goal_async()
            return
        if not goal_handle.accepted:
            self.state = GripperState.FAILED
            self.message = 'gripper goal rejected'
            return
        self._goal_handle = goal_handle
        if command_id in self._canceled_ids:
            goal_handle.cancel_goal_async()
            return
        self.state = GripperState.ACTIVE
        self.message = 'gripper goal active'
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda result: self._on_result(result, command_id)
        )

    def _on_result(self, future, command_id: int) -> None:
        try:
            wrapped = future.result()
            if command_id != self._command_id:
                return
            result = wrapped.result
            self.position = float(result.position)
            self.effort = float(result.effort)
            self.stalled = bool(result.stalled)
            self.reached_goal = bool(result.reached_goal)
            if command_id in self._canceled_ids:
                self._canceled_ids.discard(command_id)
                self.state = GripperState.FAILED
                self.message = 'gripper command canceled by safety stop'
                return
            if gripper_result_succeeded(
                wrapped.status,
                self.reached_goal,
                self.stalled,
                self._allow_stall,
            ):
                self.state = GripperState.SUCCEEDED
                self.message = (
                    f'gripper complete reached={self.reached_goal} '
                    f'stalled={self.stalled}')
            else:
                self.state = GripperState.FAILED
                self.message = (
                    f'gripper failed status={wrapped.status} '
                    f'reached={self.reached_goal} stalled={self.stalled}')
        except Exception as error:  # rclpy future exceptions are runtime data
            if command_id != self._command_id:
                return
            self.state = GripperState.FAILED
            self.message = f'gripper result exception: {error}'
